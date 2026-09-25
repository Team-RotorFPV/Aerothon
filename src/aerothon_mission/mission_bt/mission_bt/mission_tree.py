#!/usr/bin/env python3
"""Mission 2 (SkyScan) behaviour tree — RUNNABLE (drives ArduPilot via MAVROS).

Root (Fallback)
├── Guard (Sequence): CriticalOK -> else StageAwareAbort
└── Mission (Sequence): RequireDeliveryZone -> UploadArenaFence -> SetModeArm
    -> Takeoff -> ScanStartQR -> AlignToBanner -> Corridor(avoid)
    -> EnterDeliveryZone -> Climb10 -> LawnmowerSearch (from the SW corner of
    the supplied boundary) -> CenterOnTarget -> WinchDrop -> return banner
    -> ReturnCorridor(avoid) -> GotoHome -> PrecisionDescent -> Land

build_root() below is the authoritative order.

Waypoints are local-ENU metres from the takeoff origin (params, so tunable /
sim-vs-real). The corridor + return hand control to the avoidance node
(velocity setpoints); every other leg streams GPS position setpoints.
"""

import time
import math
import rclpy
from rclpy.node import Node
import py_trees
try:
    import py_trees_ros
except ImportError:
    py_trees_ros = None
from std_msgs.msg import String

from mission_bt.mav_commander import Mav
from mission_bt.banner_orbit import (fence_ok, green_fix, leg_clear,
                                      orbit_plan, outbound_structure)
from mission_bt.decode_hover import DecodeHover
from mission_bt.delivery_zone import (nearest_point_in_zone, point_in_polygon,
                                      point_inside_with_margin)
from mission_bt.geofence import build_fence, compare_fences, rect_vertices
from mission_bt.leg_router import ARRIVED, BLOCKED, LegRouter
from mission_bt.scan_geometry import bearing_to_angle
from mission_bt.search_planner import (
    grow_zone,
    coverage_fraction_excluding,
    leg_hits_exclusion,
    min_track_altitude, plan_intersects_exclusions, plan_lawnmower_excluding,
    plan_search, point_in_exclusion, routing_obstacles,
    zone_is_plausible)

# Airframe half-span plus projection margin. Overridden per-stage from the
# `redzone_clearance` mission parameter; this is only the fallback for stages
# constructed without one.
DEFAULT_CLEARANCE_M = 1.5


# --------------------------------------------------------------------------- #
# Guard & Abort
# --------------------------------------------------------------------------- #
class CheckAbortTriggered(py_trees.behaviour.Behaviour):
    """Returns SUCCESS when an abort condition trips, triggering StageAwareAbort."""
    # A link that drops for less than this and comes back is not an outage.
    # MAVROS declares the FCU lost when its heartbeat timer fires; on a
    # starved host that timer can fire after a sim-clock jump but before the
    # queued heartbeats are read, and "connected" returns a fraction of a
    # second later (arena 1001, batch E: lost and back in 0.27 s, aircraft
    # hovering armed in GUIDED throughout). While the link is down the
    # companion cannot command anything either way, so the grace only
    # decides whether the mission may carry on once the link returns.
    LINK_GRACE_S = 2.0

    def __init__(self, mav, node, link_grace_s=LINK_GRACE_S, clock=None):
        super().__init__("CheckAbortTriggered")
        self.mav = mav
        self.node = node
        self.link_grace_s = link_grace_s
        self._clock = clock or _node_clock(node) or time.monotonic
        self._lost_at = None

    def _link_lost(self):
        """True once the FCU link has been down for the whole grace."""
        if self.mav.connected():
            self._lost_at = None
            return False
        now = self._clock()
        if self._lost_at is None:
            self._lost_at = now
        return now - self._lost_at >= self.link_grace_s

    def _trip(self, reason):
        self.feedback_message = reason
        if not self.mav.abort_latched:
            self.mav.abort_reason = reason
            self.mav.abort_latched = True
        return py_trees.common.Status.SUCCESS

    def update(self):
        # An abort that un-aborts itself is not an abort. Every condition below
        # is level-triggered — attitude recovers once RTL levels the aircraft,
        # battery voltage recovers under reduced load, the FCU reconnects — so
        # without this latch the mission resumed from its previous leg while
        # the aircraft was already flying itself home.
        if self.mav.abort_latched:
            self.feedback_message = f"latched: {self.mav.abort_reason}"
            return py_trees.common.Status.SUCCESS

        # Pre-flight disconnects and battery telemetry must not trigger LAND.
        # The abort guard becomes authoritative once the mission starts or the
        # aircraft is armed.
        if not self.mav.mission_started and not self.mav.state.armed:
            return py_trees.common.Status.FAILURE
        # Trigger abort if connection lost, abort topic flagged, or critical battery.
        if self._link_lost():
            return self._trip(
                f"FCU disconnected for over {self.link_grace_s:.0f} s")
        if self.mav.abort_requested:
            return self._trip("Abort requested")
        if self.mav.battery_critical():
            return self._trip("Battery critical")
        # An aircraft held past its attitude limit is not flying the mission,
        # it is falling into something. The previous live run sat at 54 deg
        # against a corridor wall and the tree happily reported GOTO_CORRIDOR.
        if self.mav.low_altitude_fault():
            return self._trip(
                f"Sank below airborne floor "
                f"({self.mav.alt():.2f} m < {self.mav.airborne_floor:.2f} m)")
        if self.mav.attitude_excessive():
            return self._trip(
                f"Excessive attitude "
                f"(roll={self.mav.roll_deg:.1f} pitch={self.mav.pitch_deg:.1f})")
        return py_trees.common.Status.FAILURE


class StageAwareAbort(py_trees.behaviour.Behaviour):
    def __init__(self, mav):
        super().__init__("StageAwareAbort")
        self.mav = mav
        self._command_sent = False

    def initialise(self):
        self._command_sent = False

    def update(self):
        if self._command_sent:
            return py_trees.common.Status.RUNNING
        self.mav.enable_avoidance(False)
        reason = self.mav.abort_reason or "unspecified"
        # In near-ground conditions, land immediately; otherwise RTL
        if self.mav.alt() < 1.5:
            self.mav.land()
            self.logger.warning("ABORT -> LAND (low alt)")
            self.mav.publish_result("ABORTED_LAND", reason)
        else:
            self.mav.set_mode("RTL")
            self.logger.warning("ABORT -> RTL")
            self.mav.publish_result("ABORTED_RTL", reason)
        self._command_sent = True
        return py_trees.common.Status.RUNNING


# --------------------------------------------------------------------------- #
# Mission leaves
# --------------------------------------------------------------------------- #
class WaitForMissionStart(py_trees.behaviour.Behaviour):
    """Hold in a safe, disarmed state until the GCS explicitly starts M2."""
    def __init__(self, mav):
        super().__init__("WaitForMissionStart")
        self.mav = mav

    def update(self):
        self.feedback_message = "waiting for GCS start" if not self.mav.mission_started else "start received"
        return (py_trees.common.Status.SUCCESS if self.mav.mission_started
                else py_trees.common.Status.RUNNING)


class RequireDeliveryZone(py_trees.behaviour.Behaviour):
    """Block arming unless the organiser's geographic field is usable."""

    def __init__(self, mav, clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__("RequireDeliveryZone")
        self.mav = mav
        self.clearance_m = float(clearance_m)

    def update(self):
        zone = self.mav.delivery_search_zone(self.clearance_m)
        if zone is None:
            reason = ("RequireDeliveryZone: " +
                      str(getattr(self.mav, "delivery_zone_reason",
                                  "delivery-zone boundary is missing")))
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        self.feedback_message = (
            f"delivery field ready: x {zone[0]:.1f}..{zone[1]:.1f}, "
            f"y {zone[2]:.1f}..{zone[3]:.1f}")
        return py_trees.common.Status.SUCCESS


class SetModeArm(py_trees.behaviour.Behaviour):
    def __init__(self, mav):
        super().__init__("SetModeArm"); self.mav = mav; self._t = 0

    def update(self):
        self._t += 1
        if self.mav.state.mode != "GUIDED":
            self.mav.set_mode("GUIDED"); return py_trees.common.Status.RUNNING
        if not self.mav.state.armed:
            if self._t % 10 == 0:
                self.mav.arm(True)
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, mav, alt):
        super().__init__("Takeoff"); self.mav = mav; self.alt = alt; self._sent = False

    def initialise(self):
        self._sent = False

    def update(self):
        if not self._sent:
            self.mav.takeoff(self.alt); self._sent = True
        if self.mav.alt() > self.alt - 0.5:
            # From here on, sinking to the ground is a fault, not a manoeuvre.
            self.mav.set_airborne_floor(max(1.0, self.alt * 0.4))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class SetCameraPose(py_trees.behaviour.Behaviour):
    """Command a named camera pose and WAIT until the joint is measured there.

    Phase 2. The handoff's root cause for the start-QR failure was that nothing
    ever pointed the camera down: the mission held at (0,0,5) with a
    forward-facing camera while the QR lay on the ground. Pointing is now a
    precondition with a measured success criterion, not an assumption.

    RUNNING until /camera/pose_state reports settled at this pose.
    FAILURE  if it does not settle within timeout_ticks — a perception stage
             must never run believing it is looking somewhere it is not.
    """

    def __init__(self, name, mav, pose, timeout_ticks=100):
        super().__init__(name)
        self.mav = mav
        self.pose = pose
        self.timeout = timeout_ticks
        self._t = 0

    def initialise(self):
        self._t = 0
        self.mav.set_camera_pose(self.pose)

    def update(self):
        self._t += 1
        # Re-assert periodically; camera_ctrl also self-repeats, but a leaf
        # re-entered after an abort must not rely on that.
        if self._t % 10 == 0:
            self.mav.set_camera_pose(self.pose)

        if self.mav.camera_settled(self.pose):
            self.feedback_message = f"{self.pose} settled"
            return py_trees.common.Status.SUCCESS

        if self._t > self.timeout:
            reason = (f"camera did not reach {self.pose} "
                      f"(state={self.mav.camera_state_summary()})")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"{self.pose}: {self.mav.camera_state_summary()}"
        return py_trees.common.Status.RUNNING


class AlignToBanner(py_trees.behaviour.Behaviour):
    """Stop, stare, decide -- then face the banner. That heading IS the corridor.

    PHASE 4, and what it replaces: `corridor_entry = (5.0, 0.0, 3.0)` -- a
    hardcoded waypoint asserting the corridor mouth is five metres east of
    home (geometry audit A5). The banner marks the gate; flying to a memorised
    coordinate only works in the arena the coordinate came from.

    WHY IT STOPS AND STARES

        This stage used to yaw a little further on every tick while the banner
        was unidentified, and switch to a bearing-following yaw the instant it
        was. Watched live that is an oscillation, and the operator described
        it exactly: "it yaws to the right, it detects the banner, but as soon
        as it detects the banner it yaws left". A detector that confirms for
        one frame in four made the stage reverse, which took the banner back
        out of frame, which made it reverse again. It never committed.

        So the sweep is now discrete. Command a heading, wait for the airframe
        to actually reach it, then hold it for a fixed dwell and collect what
        the detector says across the whole dwell. One frame decides nothing.

    WHY A FULL TURN IS NOW SAFE

        The old limit was half a turn, for a real reason: an unbounded sweep
        once turned 271 degrees and locked onto something to the south. But
        the cause of that was acting on WEAK evidence, not on covering too
        much ground -- a continuous yaw accepts the first frame that says yes,
        and there is always something greenish somewhere.

        Requiring a dwell to be CONFIDENT (`min_hit_ratio` of its samples) is
        what makes the decoy case safe, and once it is safe, covering the full
        turn is free. It also fixes the opposite failure: a banner behind the
        start heading used to be unfindable, which is where the return leg of
        arena seed 1001 kept stranding.

    Requires the banner to be IDENTIFIED, not merely green: `mav.banner_identified()`
    is true only for z=1.0, which perception_banner reserves for a green board
    carrying white lettering. A green tarpaulin gives z=0.5 and this stage
    keeps looking.

    FAILS closed with a per-step record -- which headings were stared at, and
    what each one saw. "No banner found" is not a diagnosis.
    """

    SEARCH, SETTLE, DWELL, CENTRE, SQUARE, RELOCATE = \
        "SEARCH", "SETTLE", "DWELL", "CENTRE", "SQUARE", "RELOCATE"
    MEASURE, TURNING, MOVING = "MEASURE", "TURNING", "MOVING"

    def __init__(self, mav, tol=0.10, yaw_step=0.25, timeout_ticks=200,
                 stable_frames=5, sweep_limit_rad=2 * math.pi,
                 step_rad=math.radians(30.0), dwell_s=5.0,
                 settle_tol_rad=math.radians(6.0), settle_timeout_s=8.0,
                 min_hit_ratio=0.6, min_samples=4, clock=None,
                 hfov_rad=1.0472, align_gain=0.8, align_dwell_s=1.0,
                 max_corrections=8, min_fallback_hits=4,
                 fallback_margin=0.5, strafe_step_m=1.5,
                 stall_before_square=2,
                 square_tol_rad=math.radians(5.0),
                 sector_half_width_rad=math.radians(35.0),
                 min_standoff_m=2.5, lidar_range_m=12.0,
                 lateral_tol_m=0.4, max_square_steps=14,
                 orbit_arrive_tol=0.7, alt_arrive_tol=0.25,
                 max_refusals=25,
                 descend_step_m=0.5, alt_floor_m=3.0, max_descents=8,
                 max_relocations=6, recovery_steps=None, alt_climb_m=2.0,
                 low_camera_pose="FORWARD", image_width_px=1280,
                 banner_w_m=3.7, banner_h_m=1.15, near_range_frac=0.8,
                 redzone_clearance_m=DEFAULT_CLEARANCE_M,
                 orbit_step_rad=math.radians(45.0), orbit_vantages=7,
                 orbit_min_radius_m=5.0, fence_margin_m=2.0,
                 ring_radius_m=5.0, arc_step_rad=math.radians(20.0),
                 radial_step_m=5.0):
        super().__init__("AlignToBanner")
        self.mav = mav
        self.redzone_clearance_m = float(redzone_clearance_m)
        # ---- WHICH banner: the near one ---- #
        # Both gates carry the same banner. From the takeoff pad the RETURN
        # gate is visible straight down the return lane, 12 m off, while the
        # entrance gate stood 45 degrees to the side -- and the stage took the
        # first banner in frame, squared up on the obstacles in front of it and
        # tried to duck under a board that was not there. The entrance is the
        # NEAR one: a board beyond `near_range_frac` of the lidar's reach is
        # not the gate in front of the aircraft, and it cannot be squared up on
        # from here either. Range comes from the board's apparent area.
        self.focal_px = (0.5 * float(image_width_px)
                         / math.tan(0.5 * float(hfov_rad)))
        self.banner_area_m2 = float(banner_w_m) * float(banner_h_m)
        self.near_range_m = float(near_range_frac) * float(lidar_range_m)
        self.tol = tol
        self.yaw_step = yaw_step
        self.timeout = timeout_ticks
        self.stable_frames = stable_frames
        self.sweep_limit = sweep_limit_rad
        self.step_rad = float(step_rad)
        self.dwell_s = float(dwell_s)
        self.settle_tol = float(settle_tol_rad)
        self.settle_timeout_s = float(settle_timeout_s)
        # A dwell is a find only if most of it agreed. This one number is what
        # makes a full turn safe to sweep; see the class docstring.
        self.min_hit_ratio = float(min_hit_ratio)
        self.min_samples = int(min_samples)
        # A bearing is a FRACTION OF THE HALF-FOV, so turning it into an angle
        # needs the lens, not a gain pulled out of the air. The old
        # `psi - 0.25 * bearing` was a hidden assumption about the camera that
        # happened to under-correct by half.
        self.hfov = float(hfov_rad)
        # Slightly under 1.0 on purpose: undershooting converges, overshooting
        # rings.
        self.align_gain = float(align_gain)
        self.align_dwell_s = float(align_dwell_s)
        self.max_corrections = int(max_corrections)
        # Fallback when a full turn produced no CONFIDENT dwell.
        self.min_fallback_hits = int(min_fallback_hits)
        self.fallback_margin = float(fallback_margin)
        # Yaw cannot centre a long structure -- MEASURED, two corrections
        # moved a 0.28 bearing by +0.01. When it stalls, stop yawing and let
        # the lidar, which knows the standoff, do the geometry.
        self.strafe_step_m = float(strafe_step_m)
        self.stall_before_square = int(stall_before_square)

        # ---- squaring up, all of it measured ---- #
        # How far off perpendicular still counts as square. This is an ANGLE
        # now, in radians, not an aspect ratio: it means what it says.
        self.square_tol = float(square_tol_rad)
        # The lidar sector to search, about the bearing the camera reports.
        # Wide enough to hold a gate seen obliquely, narrow enough that the
        # corridor behind an open gate is not what gets measured.
        self.sector_half_width = float(sector_half_width_rad)
        # A standoff BAND, not a target. Both ends are properties of this
        # aircraft and this sensor -- airframe clearance at one end, the C1's
        # useful range at the other -- and neither is a fact about the arena.
        self.min_standoff = float(min_standoff_m)
        self.max_standoff = 0.5 * float(lidar_range_m)
        self.lateral_tol_m = float(lateral_tol_m)
        self.max_square_steps = int(max_square_steps)
        self.orbit_arrive_tol = float(orbit_arrive_tol)
        self.alt_arrive_tol = float(alt_arrive_tol)
        self.max_refusals = int(max_refusals)
        # THE LIDAR IS A HORIZONTAL SLICE, and at the altitude the sweep
        # happens at that slice can pass clean over the gate. MEASURED on seed
        # 1001: 51 consecutive samples in BANNER_ALIGN at 5.0 m returned 0
        # finite ranges out of 720, and the same sensor at 3.0 m returned 289.
        # So the aircraft steps down until the surface enters the scan plane,
        # rather than assuming the gate is tall enough to be seen from wherever
        # the QR scan left it.
        self.descend_step_m = float(descend_step_m)
        self.alt_floor_m = float(alt_floor_m)
        self.max_descents = int(max_descents)
        # AND THE CAMERA HAS TO COME DOWN WITH IT. The BANNER pose looks 20
        # degrees below the horizon because from 5 m a gate a few metres ahead
        # sits under a level camera. Once the aircraft has descended to gate
        # height that same pose puts the board out of the TOP of the frame:
        # watched live, the detector went from 10/10 frames at 5.0 m to
        # refusing almost every frame at 3.0 m, and the stage then had a
        # perfectly good lidar and no idea where to point it.
        self.low_camera_pose = low_camera_pose

        # ---- recovering a banner that has gone out of view ---- #
        # A YAW SWEEP CANNOT FIX A POSITION ERROR. Watched live: the aircraft
        # descended, lost the board, and sat at one spot sweeping headings.
        # From that position no heading revealed the banner, so rotating
        # through all of them could not have worked -- it was searching a
        # one-dimensional space for something that had left it.
        #
        # So the yaw zigzag becomes the INNER loop and this is the outer one:
        # relocate, then sweep again. The offsets are (along the bearing the
        # banner was last seen on, up), and the directions are the ones the
        # detector's own refusals argue for -- "green region too small" says
        # CLOSING range helps, and the sighting was lost on a descent, so
        # CLIMBING helps.
        self.recovery_steps = tuple(recovery_steps) if recovery_steps else (
            (2.5, 0.0), (2.5, 1.5), (5.0, 0.0),
            (0.0, 2.0), (5.0, 1.5), (-2.5, 1.5))
        self.max_relocations = int(max_relocations)
        self.alt_climb_m = float(alt_climb_m)
        # ---- a banner seen only edge-on (banner_orbit.py) ---- #
        # When a full turn reads no banner but did see green, orbit the green
        # for a face-on view BEFORE the blind pattern above: from the side, no
        # amount of moving along the entry heading shows the board's face.
        self.orbit_step_rad = float(orbit_step_rad)
        self.orbit_vantages = int(orbit_vantages)
        self.orbit_radius_lo = float(orbit_min_radius_m)
        self.fence_margin_m = float(fence_margin_m)
        self.ring_radius_m = float(ring_radius_m)
        self.arc_step_rad = float(arc_step_rad)
        self.radial_step_m = float(radial_step_m)
        self._face_after_move = None

        self.clock = clock or time.monotonic
        self.n_steps = max(1, int(round(2 * math.pi / self.step_rad)))
        self.sweep_offsets = self._zigzag_offsets(self.step_rad, self.n_steps)
        self.step_index = 0
        self.step_reports = []
        self._reset()

    @staticmethod
    def _zigzag_offsets(step_rad, n):
        """Headings to stare at, as offsets from where the sweep began.

        EXPANDING ALTERNATELY, not round in a circle:

            0, -30, +30, -60, +60, -90, +90, -120, +120, -150, +150, 180

        The gate is in front of the aircraft far more often than behind it --
        the start pad faces it. A one-way rotation gives the likeliest
        headings no priority, so a banner 30 degrees to the right costs one
        step if you happen to turn that way and eleven if you do not. This
        tries the nearest headings first and still covers a full turn in the
        same number of steps.

        Right first (negative in ENU), as the operator asked for.
        """
        offsets = [0.0]
        k = 1
        while len(offsets) < n:
            offsets.append(-k * step_rad)
            if len(offsets) < n:
                offsets.append(k * step_rad)
            k += 1
        return offsets[:n]

    def _reset(self):
        self.phase = self.SEARCH
        self.step_index = 0
        self.step_reports = []
        self._target_yaw = None
        self._anchor = None
        self._phase_t0 = None
        self._hits = 0
        self._samples = 0
        self._best_bearing = None
        self._best_area = 0.0
        self._far = []              # confident dwells on a FAR banner
        self._stable = 0
        self._last_seen = 0.0
        self._sweep_origin = 0.0
        self._align_target = None
        self._align_settled = False
        self._align_t0 = 0.0
        self._corrections = 0
        self._stalled = 0
        self._last_bearing = None
        self._t = 0
        self._reset_recovery()
        self._enter_square_state()

    def _reset_recovery(self):
        self._good_vantage = None
        self._vantages = []
        self._relocations = 0
        self._returned_to_good = False
        self._entry_alt = None
        self._camera_lowered = False
        self._pending_sweep_yaw = 0.0
        self._transit_yaw = 0.0
        self._commanded_yaw = None
        self._entry_yaw = None
        self._seen_yaw = None
        self._recovery_origin = None
        self._green = None            # best green_fix seen while sweeping
        self._orbit = None            # planned orbit vantages, once planned
        self._ring = None             # ring vantages when nothing green seen
        self._orbit_alt = None
        self._offsets = self.sweep_offsets
        self._pending_offsets = self.sweep_offsets
        self._path = []

    def _enter_square_state(self):
        self._sq_phase = self.MEASURE
        self._face_after_move = None
        self._sq_steps = 0
        self._refusals = 0
        self._descents = 0
        self._surface = None
        self._standoff = None
        self._sector_bearing = 0.0
        self._last_good_bearing = 0.0

    def initialise(self):
        self._reset()

    # ---- helpers ---- #
    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _elapsed(self):
        return 0.0 if self._phase_t0 is None else self.clock() - self._phase_t0

    def _enter(self, phase):
        self.phase = phase
        self._phase_t0 = self.clock()

    def _hold(self, yaw):
        """Command the anchor position with a heading.

        The commanded heading is remembered because a relocation has to fly at
        whatever the aircraft was last TOLD to hold, not at whatever it has
        drifted to -- otherwise the transit changes position and heading in
        one command, which is the coupling the whole stage is built to avoid.
        """
        x, y, z = self._anchor
        box = self._on_red(x, y)
        if box:
            # Every square-up step, stand-off correction and relocation lands
            # here, and none of them knew about red ground. At the return gate
            # it can be close: in arena 1004 the main red zone's corner sits on
            # the arc round the return board. Take the nearest clear version of
            # the same move -- turned up to 80 deg or shortened -- so the
            # square-up still makes progress; holding in place instead (batch
            # F) re-requested the same blocked step fourteen times. Nothing
            # clear: hold, and the stage's own step budget decides.
            px, py = self.mav.pos()[:2]
            if not self._on_red(px, py):
                nx, ny = self._clear_step((px, py), (x, y))
                self.mav.log(f"AlignToBanner: ({x:.1f}, {y:.1f}) is on red "
                             f"ground (inflated box x {box[0]:.1f}..{box[1]:.1f}"
                             f" y {box[2]:.1f}..{box[3]:.1f}); going to "
                             f"({nx:.1f}, {ny:.1f}) instead", warn=True)
                self._anchor = (nx, ny, z)
                x, y = nx, ny
        self._commanded_yaw = yaw
        self.mav.goto(x, y, z, yaw)

    def _clear_step(self, frm, to):
        """The clear move nearest to frm->to; frm itself if there is none."""
        dx, dy = to[0] - frm[0], to[1] - frm[1]
        # The same merged, inflated boxes the endpoint test uses, so a move
        # cannot pass between two cells the router treats as one obstacle.
        blocks = routing_obstacles(
            list(getattr(self.mav, "exclusions", None) or []),
            self.redzone_clearance_m)
        best = None
        for deg in (20, 40, 60, 80):
            for sign in (1, -1):
                for scale in (1.0, 0.66, 0.33):
                    a = math.radians(deg * sign)
                    c, s = math.cos(a), math.sin(a)
                    p = (frm[0] + scale * (c * dx - s * dy),
                         frm[1] + scale * (s * dx + c * dy))
                    if self._on_red(*p) or leg_hits_exclusion(
                            frm, p, 0.0, blocks):
                        continue
                    cost = math.dist(p, to)
                    if best is None or cost < best[0]:
                        best = (cost, p)
        return best[1] if best else frm

    def _on_red(self, x, y):
        """The inflated red box containing (x, y), or None."""
        ex = list(getattr(self.mav, "exclusions", None) or [])
        if not ex:
            return None
        for b in routing_obstacles(ex, self.redzone_clearance_m):
            if b[0] <= x <= b[1] and b[2] <= y <= b[3]:
                return b
        return None

    def _close_dwell(self):
        ratio = (self._hits / self._samples) if self._samples else 0.0
        self.step_reports.append({
            "step": self.step_index,
            "heading_deg": round(math.degrees(self._wrap(self._target_yaw)), 1),
            "samples": self._samples,
            "hits": self._hits,
            "hit_ratio": round(ratio, 2),
            "range_m": (None if self._best_area <= 0.0
                        else round(self.range_from_area(self._best_area), 1)),
        })
        return ratio

    def range_from_area(self, area_px):
        """Slant range to a face-on board of known size, from its pixels.

        An oblique board looks smaller and so reads FARTHER, never nearer:
        the error can only make a near gate look far, not a far one near.
        """
        if not area_px or area_px <= 0.0:
            return float("inf")
        return self.focal_px * math.sqrt(self.banner_area_m2 / float(area_px))

    def _sighting_range(self):
        return self.range_from_area(getattr(self.mav, "banner_board_area", 0.0))

    def _is_near(self, rng):
        # No area reported at all (older detector, test fake) -> do not veto.
        return (not math.isfinite(rng)) or rng <= self.near_range_m

    def _confident(self, ratio):
        return (self._samples >= self.min_samples
                and ratio >= self.min_hit_ratio)

    def _remember_vantage(self):
        """Latch the pose the banner was actually seen from.

        A POSITION THAT DEMONSTRABLY WORKED beats any search pattern, and it
        is one setpoint away. The aircraft identified the banner cleanly at
        t+41 s of the watched run and then spent the rest of the stage unable
        to find it again from somewhere else, with no record of where it had
        been standing when it could.
        """
        x, y, z = self.mav.pos()
        self._good_vantage = (x, y, z, self.mav.yaw())
        self._seen_yaw = self.mav.yaw()

    def _lower_camera(self):
        """Point the camera where the banner is once the aircraft is low.

        Not a tidy-up: the BANNER pose's 20 degree depression is what put the
        board out of the top of the frame after the descent, and a stage that
        cannot see the banner cannot tell the lidar which sector to search.
        """
        if self._camera_lowered:
            return
        fn = getattr(self.mav, "set_camera_pose", None)
        if callable(fn):
            fn(self.low_camera_pose)
            self.mav.log(f"AlignToBanner: descended to gate height; pointing "
                         f"the camera {self.low_camera_pose} so the board "
                         f"stays in frame")
        self._camera_lowered = True

    def _relocate(self, why):
        """Move somewhere else and sweep again. The OUTER loop.

        Returns RUNNING once a new vantage point has been commanded, or
        FAILURE when the budget is spent. Every vantage point is recorded with
        what the best heading there saw, so a failed stage says where it stood
        as well as which way it looked.
        """
        best = (max(self.step_reports, key=lambda r: r["hits"])
                if self.step_reports else None)
        x, y, z = self.mav.pos()
        self._vantages.append({
            "at": (round(x, 1), round(y, 1), round(z, 1)),
            "headings": len(self.step_reports),
            "best_hits": (best or {}).get("hits", 0),
            "best_of": (best or {}).get("samples", 0),
            "why": why})

        # 0. ONLY EDGE-ON GREEN SO FAR: go round it. Not counted against the
        #    relocation budget -- it is its own, bounded, search.
        if self._good_vantage is None and self._green is not None:
            v = self._next_orbit_vantage(z)
            if v is not None:
                return v
        if self._relocations >= self.max_relocations:
            # LAST: NOTHING GREEN SEEN ANYWHERE. The pattern below (closer,
            # higher, along the informed heading) is spent and never saw even
            # green, so the board is hidden from that whole line -- behind
            # something, or off to a side it never faced. A ring round where
            # the search began, nearest the entry heading first; the first
            # green seen switches to the orbit above.
            if self._good_vantage is None and self._green is None:
                v = self._next_ring_vantage(z)
                if v is not None:
                    return v
            return self._out_of_vantage_points(why)

        # 1. THE POSE THAT WORKED, first.
        if self._good_vantage is not None and not self._returned_to_good:
            self._returned_to_good = True
            gx, gy, gz, gyaw = self._good_vantage
            self._relocations += 1
            self.mav.log(
                f"AlignToBanner: {why}; returning to ({gx:.1f}, {gy:.1f}, "
                f"{gz:.1f}) where the banner was last identified "
                f"(relocation {self._relocations}/{self.max_relocations})")
            return self._restart_sweep_at(gx, gy, gz, gyaw)

        # 2. Then the pattern: along the bearing the banner was last seen on,
        #    and up. Never down -- the descent is what lost it.
        #
        # THE DIRECTION MATTERS MORE THAN THE PATTERN. This first used
        # `mav.yaw()`, which at the end of a full sweep is whatever heading
        # the twelfth dwell happened to leave the aircraft on -- so "close
        # range along the last bearing" walked it 12 m AWAY from a gate 9 m
        # ahead. The informed headings, in order of how much they are worth:
        # where the banner was actually seen, then the heading the stage was
        # entered on, because the mission flew here pointing at the gate.
        step = self.recovery_steps[(self._relocations
                                    - (1 if self._good_vantage else 0))
                                   % len(self.recovery_steps)]
        along, up = step
        psi = self._seen_yaw
        if psi is None:
            psi = (self._entry_yaw if self._entry_yaw is not None
                   else self.mav.yaw())
        ceiling = (self._entry_alt if self._entry_alt is not None
                   else z) + self.alt_climb_m
        # The offsets are ABSOLUTE from where recovery began, not cumulative
        # from wherever the last one left the aircraft: applied cumulatively a
        # six-step pattern of 2.5 m steps travels 20 m, which is a different
        # search from the one the numbers describe.
        if self._recovery_origin is None:
            self._recovery_origin = (self._good_vantage[:3]
                                     if self._good_vantage else (x, y, z))
        base = self._recovery_origin
        nx = base[0] + along * math.cos(psi)
        ny = base[1] + along * math.sin(psi)
        nz = min(base[2] + up, ceiling)
        self._relocations += 1
        self.mav.log(
            f"AlignToBanner: {why}; a yaw sweep cannot fix a position error, "
            f"so moving {along:+.1f} m along the last bearing and {up:+.1f} m "
            f"up, to ({nx:.1f}, {ny:.1f}, {nz:.1f}) "
            f"(relocation {self._relocations}/{self.max_relocations})")
        return self._restart_sweep_at(nx, ny, nz, psi)

    def _next_ring_vantage(self, z):
        """RUNNING toward the next ring vantage, or None when there is none."""
        if getattr(self, "_ring", None) is None:
            ox, oy = (self._recovery_origin or self.mav.pos())[:2]
            self._recovery_origin = self._recovery_origin or self.mav.pos()
            psi = self._entry_yaw if self._entry_yaw is not None else self.mav.yaw()
            ok = fence_ok(self.mav, self.fence_margin_m,
                          on_red=lambda x, y: bool(self._on_red(x, y)))
            self._ring = []
            for k in (0, 1, -1, 2, -2, 3):
                a = psi + k * math.radians(60.0)
                p = (ox + self.ring_radius_m * math.cos(a),
                     oy + self.ring_radius_m * math.sin(a))
                if ok(*p):
                    self._ring.append((p, a))
            self._orbit_alt = self.alt_floor_m      # as for the orbit
            self._lower_camera()
            self.mav.log(
                f"AlignToBanner: nothing green in view from "
                f"({ox:.1f}, {oy:.1f}); moving through a ring of "
                f"{len(self._ring)} vantage point(s) {self.ring_radius_m:.1f} m "
                f"round it rather than turning here again")
        if not self._ring:
            return None
        (x, y), a = self._ring.pop(0)
        return self._restart_sweep_at(x, y, self._orbit_alt, a, guard=True)

    def _next_orbit_vantage(self, z):
        """RUNNING toward the next orbit vantage, or None when there is none."""
        if self._orbit is None:
            g = self._green
            here = self.mav.pos()[:2]
            radius = max(self.orbit_radius_lo, min(self.near_range_m - 1.0,
                                                   g["range"]))
            # AT CORRIDOR ALTITUDE, not the search altitude: below the wall
            # tops the aircraft cannot be "above the corridor" (a rule, and a
            # graded one), and in the lidar's plane the walls and blocks are
            # what leg_clear sees. The camera comes level with it.
            self._orbit_alt = self.alt_floor_m
            self._lower_camera()
            self._orbit = orbit_plan(
                (g["x"], g["y"]), here, radius,
                fence_ok(self.mav, self.fence_margin_m,
                         on_red=lambda x, y: bool(self._on_red(x, y))),
                step_rad=self.orbit_step_rad, n=self.orbit_vantages)
            self.mav.log(
                f"AlignToBanner: no banner READ, but green seen ~{g['range']:.1f}"
                f" m off at ({g['x']:.1f}, {g['y']:.1f}) -- a board seen "
                f"edge-on has no lettering to read. Orbiting it at "
                f"{radius:.1f} m for a face-on view: {len(self._orbit)} "
                f"vantage point(s)")
        if not self._orbit:
            return None
        v = self._orbit.pop(0)
        fan = [0.0, -self.hfov / 2.0, self.hfov / 2.0]
        return self._restart_sweep_at(v["at"][0], v["at"][1], self._orbit_alt,
                                      v["face"], path=v["path"], offsets=fan,
                                      transit_yaw=v["face"], guard=True)

    def _restart_sweep_at(self, x, y, z, yaw, path=None, offsets=None,
                          transit_yaw=None, guard=False):
        """Fly to a new vantage point, THEN sweep it.

        Translating and rotating in the same command reintroduces the coupling
        that made yaw useless in the first place, so the transit holds
        whatever heading the aircraft already has and the sweep starts once it
        has arrived. An orbit leg holds the heading that faces what it is
        orbiting instead (`transit_yaw`), and follows `path` -- arc waypoints
        ending at the vantage -- rather than a straight line that could cut
        across the structure.
        """
        legs = [(px, py, z) for px, py in (path or [])] or [(x, y, z)]
        self._anchor, self._path = legs[0], legs[1:]
        self._pending_offsets = offsets or self.sweep_offsets
        self._guard_leg = bool(guard)      # orbit legs: lidar-checked
        self._pending_sweep_yaw = self._wrap(yaw)
        self._transit_yaw = (transit_yaw if transit_yaw is not None
                             else self._commanded_yaw
                             if self._commanded_yaw is not None
                             else self.mav.yaw())
        self._enter_square_state()
        self._enter(self.RELOCATE)
        self._hold(self._transit_yaw)
        return py_trees.common.Status.RUNNING

    def _transit(self):
        """Hold heading and altitude discipline while relocating."""
        ax, ay, az = self._anchor
        if getattr(self, "_guard_leg", False) and not leg_clear(self.mav, ax, ay):
            # Something within reach along the leg, and the lidar can see it:
            # do not fly into it. Give this vantage up for the next one.
            self.mav.log(f"AlignToBanner: the leg to ({ax:.1f}, {ay:.1f}) is "
                         f"blocked on the lidar; skipping that vantage point",
                         warn=True)
            self._path = []
            self._anchor = self.mav.pos()
            return self._relocate("orbit leg blocked on the lidar")
        if (getattr(self, "_guard_leg", False) and self.mav.banner_identified()
                and self._is_near(self._sighting_range())):
            # The camera faces the gate for the whole orbit. The board read
            # mid-leg: stop HERE and confirm, rather than fly on past the view.
            px, py, _ = self.mav.pos()
            self.mav.log(f"AlignToBanner: lettering read on the way round, at "
                         f"({px:.1f}, {py:.1f}); stopping to confirm")
            self._anchor = (px, py, az)
            self._path = []
            self._pending_offsets = [0.0, -self.step_rad, self.step_rad]
            self._pending_sweep_yaw = self.mav.yaw()
            self._guard_leg = False
            ax, ay = px, py
        self._hold(self._transit_yaw)
        arrived = (self.mav.reached(ax, ay, az, self.orbit_arrive_tol)
                   and abs(self.mav.alt() - az) <= self.alt_arrive_tol)
        done = arrived or self._elapsed() > self.settle_timeout_s * 3.0
        if done and self._path:
            self._anchor = self._path.pop(0)        # the next arc waypoint
            self._enter(self.RELOCATE)
            return py_trees.common.Status.RUNNING
        if done:
            self._offsets = self._pending_offsets
            self.step_index = 0
            self.step_reports = []
            self._target_yaw = self._pending_sweep_yaw
            self._sweep_origin = self._pending_sweep_yaw
            self._hits = self._samples = 0
            self._best_bearing = None
            self._best_area = 0.0
            self._far = []
            self._enter(self.SETTLE)
            return py_trees.common.Status.RUNNING
        x, y, z = self.mav.pos()
        self.feedback_message = (
            f"relocating: {math.hypot(ax - x, ay - y):.1f} m across and "
            f"{az - z:+.1f} m up to vantage point {self._relocations}")
        return py_trees.common.Status.RUNNING

    def _out_of_vantage_points(self, why):
        seen = "; ".join(
            f"({v['at'][0]}, {v['at'][1]}, {v['at'][2]}) "
            f"{v['headings']} headings, best {v['best_hits']}/{v['best_of']}"
            for v in self._vantages)
        reason = (f"AlignToBanner: no AEROTHON banner found from "
                  f"{len(self._vantages)} vantage point(s) -- {seen}. Last "
                  f"reason: {why}")
        self.feedback_message = reason
        self.mav.abort_reason = reason
        self.mav.log(reason, warn=True)
        return py_trees.common.Status.FAILURE

    # ---- the tick ---- #
    def update(self):
        self._t += 1
        if self._anchor is None:
            self._anchor = self.mav.pos()
            self._entry_alt = self._anchor[2]
            # The heading the aircraft ARRIVED on. With no sighting yet this
            # is the only informed guess about where the gate is: the mission
            # flew here pointing at it.
            self._entry_yaw = self.mav.yaw()
        if self._target_yaw is None:
            self._sweep_origin = self.mav.yaw()
            self._target_yaw = self._wrap(self._sweep_origin
                                          + self.sweep_offsets[0])
            # LOOK FIRST. The zigzag exists to FIND a banner, and running it
            # when one is already in frame is what the operator watched as an
            # unexplained yaw and roll away from a board the aircraft could
            # see perfectly well. If it is already there, go and face it.
            rng = self._sighting_range()
            if self.mav.banner_identified() and not self._is_near(rng):
                self.mav.log(
                    f"AlignToBanner: a banner is in frame but about "
                    f"{rng:.0f} m off, beyond the {self.near_range_m:.1f} m a "
                    f"gate in front of the aircraft can be; sweeping for the "
                    f"near one")
            elif self.mav.banner_identified():
                self._remember_vantage()
                self.mav.log(
                    f"AlignToBanner: the banner is already in frame at "
                    f"bearing {self.mav.banner_bearing():+.2f}; centring on "
                    f"it rather than sweeping")
                self._align_target = self._target_yaw
                self._align_settled = False
                self._align_t0 = self.clock()
                self._last_seen = self.clock()
                self._enter(self.CENTRE)
                return self._centre()
            self._enter(self.SETTLE)

        if self.phase is self.RELOCATE:
            return self._transit()
        if self.phase is self.SQUARE:
            return self._square()
        if self.phase is self.CENTRE:
            return self._centre()
        if self.phase is self.SETTLE:
            return self._settle()
        return self._dwell()

    def _settle(self):
        """Wait for the AIRFRAME to reach the heading, not for the command to
        have been sent. Counting commanded yaw as achieved yaw is what once
        reported 180 degrees swept while 23 degrees had been turned."""
        self._hold(self._target_yaw)
        err = abs(self._wrap(self.mav.yaw() - self._target_yaw))
        if err <= self.settle_tol:
            self._hits = 0
            self._samples = 0
            self._best_bearing = None
            self._best_area = 0.0
            self._enter(self.DWELL)
            return py_trees.common.Status.RUNNING
        if self._elapsed() > self.settle_timeout_s:
            reason = (f"AlignToBanner: the aircraft would not hold heading "
                      f"{math.degrees(self._wrap(self._target_yaw)):.0f} deg "
                      f"({math.degrees(err):.0f} deg off after "
                      f"{self.settle_timeout_s:.0f} s); the sweep cannot stare "
                      f"at anything it cannot point at")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            self.mav.log(reason, warn=True)
            return py_trees.common.Status.FAILURE
        self.feedback_message = (
            f"turning to step {self.step_index + 1}/{len(self._offsets)} "
            f"({math.degrees(err):.0f} deg to go)")
        return py_trees.common.Status.RUNNING

    def _dwell(self):
        """Hold the heading and collect evidence. Nothing moves here."""
        self._hold(self._target_yaw)
        self._samples += 1
        seen = self.mav.banner_identified()      # ONE reading per sample
        if not seen:
            f = green_fix(self.mav, self.hfov,
                          exclude=outbound_structure(self.mav))
            if f is not None and (self._green is None
                                  or f["area"] > self._green["area"]):
                self._green = f
        if seen:
            self._hits += 1
            b = self.mav.banner_bearing()
            if self._best_bearing is None or abs(b) < abs(self._best_bearing):
                self._best_bearing = b
            self._best_area = max(self._best_area,
                                  float(getattr(self.mav, "banner_board_area",
                                                0.0) or 0.0))

        if self._elapsed() < self.dwell_s and not self._hopeless():
            self.feedback_message = (
                f"staring at step {self.step_index + 1}/{len(self._offsets)}, "
                f"{self._elapsed():.1f}/{self.dwell_s:.0f} s, "
                f"{self._hits}/{self._samples} frames identified")
            return py_trees.common.Status.RUNNING

        ratio = self._close_dwell()
        rng = self.range_from_area(self._best_area)
        if self._confident(ratio) and not self._is_near(rng):
            # A real banner, but the far gate. Remember it and keep looking:
            # the entrance is nearer, and first-confident-wins took this one.
            self._far.append((self._best_area, self._target_yaw, rng))
            self.mav.log(
                f"AlignToBanner: banner at "
                f"{math.degrees(self._wrap(self._target_yaw)):.0f} deg is about "
                f"{rng:.0f} m off -- the far gate, not the one in front; "
                f"sweeping on for a nearer one")
        elif self._confident(ratio):
            return self._accept_heading(self._target_yaw)

        if len(self.step_reports) >= len(self._offsets):
            if self._far:
                # Nothing nearer from here: the nearest far sighting is the
                # best evidence of where the gate is. Face it; CENTRE and the
                # lidar square-up then close the range.
                area, yaw, rng = max(self._far)
                self.mav.log(
                    f"AlignToBanner: no banner within {self.near_range_m:.1f} m "
                    f"after a full turn; taking the nearest one seen, about "
                    f"{rng:.0f} m off at {math.degrees(self._wrap(yaw)):.0f} deg")
                self._far = []
                self._target_yaw = yaw
                return self._accept_heading(yaw)
            return self._give_up()

        self.step_index += 1
        self._target_yaw = self._wrap(self._sweep_origin
                                      + self._offsets[self.step_index])
        self._enter(self.SETTLE)
        return py_trees.common.Status.RUNNING

    def _hopeless(self):
        """True once this dwell can no longer reach `min_hit_ratio`.

        Even if every remaining frame were a hit. Watched on a custom arena
        whose pad only saw the board edge-on: twelve headings of 0/25 frames,
        five seconds each, a minute of sim time turning on one spot before
        the search could move. The confidence rule is unchanged -- a dwell
        that COULD still pass runs to the end -- but an empty heading is over
        after ~40% of it, which is the point it stops being able to pass.
        """
        el = self._elapsed()
        if self._samples < self.min_samples or el <= 0.0:
            return False
        rate = self._samples / el
        left = rate * max(0.0, self.dwell_s - el)
        best = (self._hits + left) / (self._samples + left)
        return best < self.min_hit_ratio

    def _accept_heading(self, yaw):
        """Commit to the banner seen at `yaw` and move on to centring."""
        self._remember_vantage()
        # The sighting's heading, not wherever the sweep has since turned to:
        # after a full turn the airframe faces the LAST step, not the banner.
        x, y, z, _ = self._good_vantage
        self._good_vantage = (x, y, z, yaw)
        self._seen_yaw = yaw
        self.mav.log(
            f"AlignToBanner: banner identified at "
            f"{math.degrees(self._wrap(yaw)):.0f} deg after "
            f"staring at {len(self.step_reports)} heading(s) "
            f"({self._hits}/{self._samples} frames)")
        self._enter(self.CENTRE)
        self._stable = 0
        self._last_seen = self.clock()
        self._align_target = yaw
        self._align_settled = False
        self._align_t0 = self.clock()
        self._corrections = 0
        self._stalled = 0
        self._last_bearing = None
        self._enter_square_state()
        return py_trees.common.Status.RUNNING

    def _give_up(self):
        """A full turn from here saw nothing. Go and stand somewhere else.

        ONE BEHAVIOUR PER STATE. This used to have a second path: if no dwell
        reached the confidence floor but one heading was clearly the best, the
        stage aligned to that heading anyway. Together with peak detection,
        reversal-on-narrowing and the aspect threshold -- each added to rescue
        the one before it -- the aircraft looked like it was guessing, because
        it was. The operator's words: "as soon as it detects the banner it
        should not be confused and it should not try to do different things."

        The relocation loop is a better answer to a marginal sweep than
        committing to a marginal heading: sweeping again from two metres
        closer costs seconds and produces evidence, where aligning to a 6/12
        dwell commits the mission to it.
        """
        why = (self.mav.banner_rejection_summary()
               if hasattr(self.mav, "banner_rejection_summary")
               else "no detail available")
        covered = round(math.degrees(self.step_rad) * len(self.step_reports))
        # The stage name has to be IN the line: run artifacts are grepped, and
        # the seed 1001 failure was undiagnosable after the fact partly
        # because its log line did not say which stage produced it.
        self.mav.log("AlignToBanner stared at: " + "; ".join(
            f"{r['heading_deg']:.0f} deg {r['hits']}/{r['samples']}"
            for r in self.step_reports))
        # A FULL TURN FROM ONE SPOT IS NOT A SEARCH. Every heading has been
        # tried; what has not been tried is standing somewhere else.
        return self._relocate(
            f"{len(self.step_reports)} headings covering {covered} deg from "
            f"here saw no banner (detector said: {why})")

    def _centre(self):
        """Fine-align in DISCRETE corrections, for the same reason the sweep
        stops and stares.

        THE LIVE REGRESSION (seed 1001, watched, arena regression with GUI)

            The sweep worked -- "banner identified at -0 deg after staring at
            1 heading(s) (11/11 frames)" -- and then the aircraft swung
            between -1 and -31 degrees on a five-second cycle and never
            converged:

                t      yaw     bearing  identified
                6.5    -1.1     0.82    yes
                10.7  -26.7     0.72    yes
                11.2  -31.4     0.00    NO     <- detector drops it
                13.7   -1.3     0.00    NO     <- snapped back to 0
                14.2   -0.5     0.81    yes    <- re-acquired, starts over

        TWO CAUSES

            The correction was `psi - gain * bearing`, recomputed from the
            CURRENT heading on every tick. A real airframe is still turning
            when the next setpoint is computed, so the target moved away at
            the same rate the aircraft closed on it. This stack has met that
            before, in ApproachBanner, under the name "receding carrot".

            And when the banner dropped, the hold used the DWELL heading --
            throwing away every degree of alignment achieved since, which is
            what turned a wobble into a repeating cycle.

        SO: latch a target, wait for the airframe to actually reach it, hold
        it long enough for a still measurement, and only then decide whether
        another correction is needed. The bearing is converted to an angle
        with the CAMERA's field of view rather than a gain: a bearing is a
        fraction of the half-FOV, and the lens is already known.
        """
        psi = self.mav.yaw()

        if not self.mav.banner_identified():
            # Hold the ALIGNMENT target, not the dwell heading.
            self._hold(self._align_target)
            gone = self.clock() - self._last_seen
            if gone > self.settle_timeout_s:
                return self._relocate(
                    f"the banner confirmed at "
                    f"{math.degrees(self._wrap(self._target_yaw)):.0f} deg "
                    f"has not been seen for {gone:.0f} s of centring")
            self.feedback_message = (f"centring: banner lost for {gone:.1f} s, "
                                     f"holding {math.degrees(self._align_target):.0f} deg")
            return py_trees.common.Status.RUNNING

        self._last_seen = self.clock()
        bearing = self.mav.banner_bearing()
        self._last_good_bearing = bearing
        self._remember_vantage()
        self._hold(self._align_target)

        if not self._align_settled:
            err = abs(self._wrap(psi - self._align_target))
            if err <= self.settle_tol:
                self._align_settled = True
                self._align_t0 = self.clock()
            elif self.clock() - self._align_t0 > self.settle_timeout_s:
                # Measure from wherever it got to rather than hanging: a
                # heading that will not settle is a control problem, and
                # refusing to look at the banner does not fix it.
                self._align_settled = True
                self._align_t0 = self.clock()
            else:
                self.feedback_message = (
                    f"turning to {math.degrees(self._align_target):+.0f} deg "
                    f"({math.degrees(err):.0f} deg to go)")
            return py_trees.common.Status.RUNNING

        # Settled. Anything measured from here is measured from a still
        # aircraft, which is the only kind of bearing worth acting on.
        #
        # CENTRED IS NOT SQUARE ON. Facing the gate from off to one side and
        # committing to a waypoint through it drives at the board rather than
        # through the opening -- watched live, twice, and both times the
        # aircraft left the arena. Centring is where the ALIGN task ends and
        # where squaring up begins; the lidar does the second half.
        if abs(bearing) <= self.tol:
            self.mav.log(
                f"AlignToBanner: centred on the banner at "
                f"{math.degrees(self._align_target):+.0f} deg "
                f"(bearing {bearing:+.2f}); squaring up on the lidar")
            return self._begin_square()

        self._stable = 0
        if self.clock() - self._align_t0 < self.align_dwell_s:
            self.feedback_message = (f"measuring at "
                                     f"{math.degrees(self._align_target):+.0f} "
                                     f"deg, bearing={bearing:+.3f}")
            return py_trees.common.Status.RUNNING

        # DID THE LAST CORRECTION ACTUALLY HELP?
        #
        # MEASURED, seed 1001 run 9, with the banner locked 12/12:
        #
        #     correction 1: bearing +0.28 at -30 deg -> commanding -37
        #     correction 2: bearing +0.29 at -37 deg -> commanding -44
        #
        # Seven degrees of yaw toward a point target should cut a bearing of
        # 0.28 by about 0.23. It moved +0.01, the wrong way.
        #
        # The gate is not a point. It is a long structure running away from
        # the aircraft, so yawing toward it brings MORE of it into frame and
        # the box centroid slides right by as much as the rotation moved it
        # left. The two cancel, and no amount of yaw will centre it.
        #
        # That used to trigger a blind sideways step. It now hands over to the
        # lidar, which knows the standoff and can therefore work out how far
        # sideways -- rather than guessing a distance and re-measuring the
        # same proxy that stalled.
        improved = (self._last_bearing is None
                    or abs(bearing) < abs(self._last_bearing) - 0.03)
        if not improved:
            self._stalled += 1
        else:
            self._stalled = 0
        self._last_bearing = bearing

        if (self._stalled >= self.stall_before_square
                or self._corrections >= self.max_corrections):
            self.mav.log(
                f"AlignToBanner: yaw stopped closing on the banner "
                f"(bearing {bearing:+.2f} after {self._corrections} "
                f"correction(s)); squaring up on the lidar instead")
            return self._begin_square()

        # Positive bearing means the banner is to the RIGHT of frame centre,
        # so the aircraft must yaw right, which is NEGATIVE yaw in ENU.
        theta = self.align_gain * bearing * (self.hfov / 2.0)
        self._align_target = self._wrap(self._align_target - theta)
        self._corrections += 1
        self._align_settled = False
        self._align_t0 = self.clock()
        self.feedback_message = (
            f"correction {self._corrections}: bearing {bearing:+.2f} -> "
            f"{math.degrees(self._align_target):+.0f} deg")
        # LOGGED, not just set as feedback. A stage that hunted for a hundred
        # seconds in flight left no record of a single setpoint it commanded,
        # because feedback_message never reaches the run log. The next time
        # this misbehaves the log has to be able to answer "what did it ask
        # for, and what did the bearing do in response".
        self.mav.log(f"AlignToBanner correction {self._corrections}: "
                     f"bearing {bearing:+.2f} at "
                     f"{math.degrees(psi):+.0f} deg -> commanding "
                     f"{math.degrees(self._align_target):+.0f} deg")
        return py_trees.common.Status.RUNNING

    # ---- squaring up, on the lidar ---- #
    def _begin_square(self):
        self._enter(self.SQUARE)
        self._sq_phase = self.MEASURE
        self._align_t0 = self.clock()
        self._stable = 0
        return py_trees.common.Status.RUNNING

    def _measure_surface(self):
        """Ask the commander for the surface in the sector the camera names.

        The sector is DERIVED from the camera bearing, not fixed ahead. That
        is what stops the corridor wall behind an open gate being measured
        instead of the gate, and it is why the camera keeps a job here at all:
        it says WHERE to look, the lidar says what is there.
        """
        if self.mav.banner_identified():
            self._last_good_bearing = self.mav.banner_bearing()
        self._sector_bearing = bearing_to_angle(self._last_good_bearing,
                                                self.hfov)
        return self.mav.surface_ahead(self._sector_bearing,
                                      self.sector_half_width,
                                      expected_range_m=self._standoff)

    def _box_aspect(self):
        """The camera's own opinion of squareness, for CROSS-CHECK ONLY.

        A board is widest seen face-on, so the aspect of the derived box does
        carry angle information -- it just cannot be thresholded, because the
        box includes the gate posts and plateaus at 1.88-1.91. It is reported
        beside the lidar angle so that a persistent disagreement between the
        two instruments is visible in the run artifact rather than silent.

        NOTHING STEERS ON THIS. The lidar decides, per the spec.
        """
        fn = getattr(self.mav, "banner_aspect", None)
        try:
            return float(fn()) if callable(fn) else 0.0
        except Exception:                    # noqa: BLE001
            return 0.0

    def _report(self, fit):
        payload = {"stage": "SQUARE",
                   "ok": bool(fit.get("ok")),
                   "angle_deg": (None if fit.get("angle_rad") is None
                                 else round(math.degrees(fit["angle_rad"]), 1)),
                   "standoff_m": (None if fit.get("range_m") is None
                                  else round(fit["range_m"], 2)),
                   "points": int(fit.get("points") or 0),
                   "residual_m": (None if fit.get("residual_m") is None
                                  else round(fit["residual_m"], 3)),
                   "sector_deg": round(math.degrees(self._sector_bearing), 1),
                   "steps": self._sq_steps,
                   # The camera's cross-check, carried alongside so the two
                   # instruments can be compared after the fact.
                   "box_aspect": round(self._box_aspect(), 2),
                   "bearing": round(self._last_good_bearing, 3),
                   "pose": list(self.mav.pos()),
                   "yaw_deg": round(math.degrees(self.mav.yaw()), 2),
                   "banner_identified": bool(self.mav.banner_identified()),
                   "reason": fit.get("reason", "")}
        fn = getattr(self.mav, "publish_square_on", None)
        if callable(fn):
            fn(payload)

    def _square_failure(self, reason):
        self.feedback_message = reason
        self.mav.abort_reason = reason
        self.mav.log(reason, warn=True)
        return py_trees.common.Status.FAILURE

    def _measured(self):
        """The last measurement, as prose. Every refusal says this much."""
        if not self._surface:
            return "the lidar never measured a surface"
        return (f"last measured {math.degrees(self._surface['angle_rad']):+.1f} "
                f"deg off perpendicular at {self._surface['range_m']:.1f} m "
                f"standoff, from {self._surface['points']} returns")

    def _square(self):
        """Square on to the banner's face, on a MEASURED angle.

        WHAT THIS REPLACES

            An aspect-ratio gate. A board is widest seen face-on, so its
            apparent aspect was used as a proxy for perpendicularity -- but
            the derived box includes the gate posts, so it plateaus at 1.88 to
            1.91 whatever the aircraft does. A bar of 2.00 was unreachable and
            the aircraft orbited all fourteen steps having been in front of
            the gate since step eight; a bar of 1.75 with peak detection was
            satisfied by one noisy narrowing at 1.4 and the aircraft advanced
            while badly off-axis. Fourteen watched runs, three distinct
            failures, one root cause: the camera cannot answer "am I
            perpendicular to that surface".

            The lidar can, and does, in radians. There is no fallback to the
            aspect test, because a fallback that fires on bad data is exactly
            how the aircraft flew out of the world.

        THE STEP IS AN ARC, NOT A SLIDE

            Each step moves tangentially -- along the face, from the camera
            bearing and the measured standoff -- and radially, to keep the
            standoff inside a band the sensor can work in. One action per
            step, never a turn and a translation at once, and the aircraft
            arrives and stops before the next measurement is taken. Measuring
            while still moving is how three orbit steps in a row once reported
            an identical aspect: the stage was reading the old position.
        """
        self._hold(self._align_target)

        if self._sq_phase is self.TURNING:
            err = abs(self._wrap(self.mav.yaw() - self._align_target))
            timed_out = (self.clock() - self._align_t0
                         > self.settle_timeout_s)
            if err <= self.settle_tol or timed_out:
                self._sq_phase = self.MEASURE
                self._align_t0 = self.clock()
                return py_trees.common.Status.RUNNING
            self.feedback_message = (
                f"squaring up: {math.degrees(err):.0f} deg of yaw to go")
            return py_trees.common.Status.RUNNING

        if self._sq_phase is self.MOVING:
            ax, ay, az = self._anchor
            # ALTITUDE IS CHECKED SEPARATELY, and tightly. A descent step of
            # half a metre is already inside the horizontal arrival tolerance,
            # so a position check alone calls the aircraft "arrived" the
            # instant the target moves -- and the stage then spends its whole
            # descent budget in four seconds while the airframe is still at
            # the height it started from. Same class of defect as reading an
            # orbit step before the aircraft has taken it.
            arrived = (self.mav.reached(ax, ay, az, self.orbit_arrive_tol)
                       and abs(self.mav.alt() - az) <= self.alt_arrive_tol)
            timed_out = (self.clock() - self._align_t0
                         > self.settle_timeout_s)
            if arrived or timed_out:
                face = getattr(self, "_face_after_move", None)
                if face is not None:
                    # The arc step's second action: face the board from here.
                    self._face_after_move = None
                    self._align_target = self._wrap(face)
                    self._sq_phase = self.TURNING
                else:
                    self._sq_phase = self.MEASURE
                self._align_t0 = self.clock()
                return py_trees.common.Status.RUNNING
            x, y, z = self.mav.pos()
            self.feedback_message = (
                f"squaring up: {math.hypot(ax - x, ay - y):.1f} m across and "
                f"{az - z:+.1f} m down to the next vantage point")
            return py_trees.common.Status.RUNNING

        # STILL, and settled. Only now is a measurement worth anything.
        if self.clock() - self._align_t0 < self.align_dwell_s:
            self.feedback_message = "holding station for a still measurement"
            return py_trees.common.Status.RUNNING

        fit = self._measure_surface()
        self._report(fit)

        # WHICH WAY TO TURN IS A CAMERA QUESTION, and it is answered on every
        # tick the banner is identified, with no lidar involved.
        #
        # Watched live: the aircraft held station while CONTINUING TO DETECT
        # the banner, because the lidar could not confirm perpendicularity and
        # everything had been gated behind that confirmation. Fail-closed is
        # right for committing a waypoint through the gate. It is wrong for
        # turning to look at something the aircraft can already see.
        if self._recentre_needed():
            centred = self._recentre()
            if centred is not None:
                return centred

        if not fit["ok"]:
            return self._no_surface(fit)

        self._refusals = 0
        self._surface = fit
        self._standoff = fit["range_m"]
        alpha, standoff = fit["angle_rad"], fit["range_m"]

        # 1. OBLIQUITY IS FIXED BY MOVING, NOT BY TURNING.
        #
        # THE BUG THIS REPLACES, watched live and repeated four times in one
        # run. The aircraft measured a good surface -- 36 to 91 returns, 2 cm
        # residual -- found it 50 degrees off perpendicular, and TURNED 50
        # degrees to square up. The next scan had zero returns in the sector,
        # every time:
        #
        #     square-up 1/14: banner face is +51.2 deg off perpendicular at
        #     5.5 m (36 returns, residual 2.8 cm); turning to +8 deg
        #     [next tick] only 0 lidar return(s) inside the 70 deg sector
        #
        # `angle_rad` is the direction of the surface NORMAL, not the
        # direction of the surface. Turning to face the normal points the nose
        # along a line that misses the board entirely, so the aircraft ends up
        # perpendicular to the face while staring at the empty air beside it.
        #
        # Being square on needs the nose along the normal AND the board dead
        # ahead, and no rotation satisfies both: turning to fix one breaks the
        # other. The only manoeuvre that does is travelling round the board.
        # So obliquity commands a TANGENTIAL step and the camera keeps the
        # board centred with yaw -- which is the arc the spec asked for, and
        # the reason the two jobs are split between the two instruments.
        if abs(alpha) > self.square_tol:
            # THE ARC NEEDS BOTH INSTRUMENTS. A tangential step does not move
            # `alpha` by itself -- perpendicularity is a property of HEADING,
            # measured at station 3 of the ground probe as 1.1 degrees of
            # change for 3 metres of lateral travel. What the step does is
            # take the board off frame centre, and the camera's yaw correction
            # is what converts that into a change of heading. Translation and
            # re-centring together walk the arc; either alone goes nowhere.
            #
            # So with the detector blind there is no point spending a step:
            # hold station, keep measuring, and let the lost-banner timeout
            # decide when this has stopped being a dropped frame.
            if not self.mav.banner_identified():
                gone = self.clock() - self._last_seen
                if gone > self.settle_timeout_s:
                    return self._relocate(
                        f"the banner has not been seen for {gone:.0f} s while "
                        f"squaring up, and the arc cannot be flown without it")
                self.feedback_message = (
                    f"squaring up: {math.degrees(alpha):+.1f} deg off "
                    f"perpendicular, waiting {gone:.1f} s for the detector")
                return py_trees.common.Status.RUNNING
            if self._sq_steps >= self.max_square_steps:
                return self._square_failure(
                    f"AlignToBanner: {self._sq_steps} steps did not bring the "
                    f"aircraft square to the banner; {self._measured()}")
            self._sq_steps += 1
            # ROUND THE BOARD ON ITS OWN CIRCLE, UP TO `arc_step_rad` A STEP.
            #
            # This was a sideways slide of at most 1.5 m on a FIXED heading,
            # with the camera's yaw corrections left to turn it back onto the
            # board. From 41 deg off at 7-10 m that was ~9 deg per step: five
            # steps, a camera correction after each, and in the watched run
            # the board slid out of frame on the fourth and cost two minutes
            # of recovery. Now the step goes to the point on the circle of
            # the MEASURED standoff that is up to 20 deg nearer the face's
            # normal, and the next action turns to face the board from there.
            # Still one action per step -- a move, THEN a turn -- and 20 deg
            # keeps the board inside the 30 deg half-field of view while the
            # heading lags. A face whose perpendicular foot is to PORT means
            # the aircraft stands to starboard of the centreline, so it
            # travels starboard -- counter-clockwise round the board.
            ax, ay, az = self._anchor
            px, py = self.mav.pos()[:2]
            h = self.mav.yaw() + self._sector_bearing
            bx, by = px + standoff * math.cos(h), py + standoff * math.sin(h)
            theta = math.atan2(py - by, px - bx)
            dth = math.copysign(min(abs(alpha), self.arc_step_rad), alpha)
            nx = bx + standoff * math.cos(theta + dth)
            ny = by + standoff * math.sin(theta + dth)
            lat = -math.copysign(math.hypot(nx - px, ny - py), alpha)
            self._anchor = (nx, ny, az)
            self._face_after_move = math.atan2(by - ny, bx - nx)
            self._sq_phase = self.MOVING
            self._align_t0 = self.clock()
            self._stable = 0
            # A translation changes the geometry the yaw budget was being
            # spent against, so centring starts again from the new position.
            self._corrections = 0
            self.mav.log(
                f"AlignToBanner square-up {self._sq_steps}/"
                f"{self.max_square_steps}: banner face is "
                f"{math.degrees(alpha):+.1f} deg off perpendicular at "
                f"{standoff:.1f} m ({fit['points']} returns, residual "
                f"{fit['residual_m'] * 100:.1f} cm); travelling {abs(lat):.1f} m "
                f"{'port' if lat > 0 else 'starboard'} round it")
            return py_trees.common.Status.RUNNING

        # A perpendicular heading can still pass beside the gate. Exhausting
        # the yaw budget does not waive the camera's centring measurement.
        if self._recentre_needed():
            if self._sq_steps >= self.max_square_steps:
                return self._square_failure(
                    "AlignToBanner: banner remains off-centre after lateral "
                    f"corrections; {self._measured()}")
            bearing = self.mav.banner_bearing()
            lateral = standoff * math.tan(bearing_to_angle(bearing, self.hfov))
            lateral = max(-self.strafe_step_m, min(self.strafe_step_m, lateral))
            ax, ay, az = self._anchor
            psi = self._align_target
            self._anchor = (ax - lateral * math.sin(psi),
                            ay + lateral * math.cos(psi), az)
            self._sq_steps += 1
            self._sq_phase = self.MOVING
            self._align_t0 = self.clock()
            self._stable = 0
            self._corrections = 0
            self.mav.log(
                f"AlignToBanner: perpendicular but bearing {bearing:+.2f}; "
                f"moving {lateral:+.2f} m laterally to centre the banner")
            return py_trees.common.Status.RUNNING

        # 2. RANGE. Square to the face and centred on it; all that is left is
        #    standing at a distance the lidar and the camera both work at.
        #    AIM INSIDE THE BAND, not at its edge. Stepping exactly to the
        #    6.0 m limit left the aircraft at 6.0-6.1 m after lag and noise,
        #    so it stepped +0.0 / +0.1 m fourteen times and the stage failed
        #    square-on at -2.1 deg (seed 1001; run 3's return lap scraped in
        #    at 5.9 m after the same dance).
        radial = 0.0
        if standoff > self.max_standoff:
            radial = standoff - (self.max_standoff - 1.0)
        elif standoff < self.min_standoff:
            radial = standoff - (self.min_standoff + 0.5)
        if radial != 0.0:
            if self._sq_steps >= self.max_square_steps:
                return self._square_failure(
                    f"AlignToBanner: {self._sq_steps} steps did not bring the "
                    f"aircraft to a workable standoff; {self._measured()}")
            self._sq_steps += 1
            # Square to the face, so this is straight along its normal on a
            # range the lidar has just measured: aimed into the band, not at
            # its edge, there is nothing to overshoot. One move of up to
            # `radial_step_m`, not three of 1.5 m (watched: 10.0 -> 8.5 ->
            # 7.1 -> 5.4 m, a settle and a measurement each).
            fwd = max(-self.radial_step_m,
                      min(self.radial_step_m, radial))
            ax, ay, az = self._anchor
            psi = self._align_target
            self._anchor = (ax + fwd * math.cos(psi),
                            ay + fwd * math.sin(psi), az)
            self._sq_phase = self.MOVING
            self._align_t0 = self.clock()
            self._stable = 0
            self._corrections = 0
            self.mav.log(
                f"AlignToBanner square-up {self._sq_steps}/"
                f"{self.max_square_steps}: square to the face but standing at "
                f"{standoff:.1f} m, outside the {self.min_standoff:.1f}-"
                f"{self.max_standoff:.1f} m band; moving {fwd:+.1f} m along "
                f"the standoff")
            return py_trees.common.Status.RUNNING

        # 3. SQUARE ON. Held, not sampled once.
        self._stable += 1
        if self._stable >= self.stable_frames:
            self.mav.log(
                f"AlignToBanner: SQUARE ON -- "
                f"{math.degrees(alpha):+.1f} deg off perpendicular "
                f"(tolerance {math.degrees(self.square_tol):.0f} deg) at "
                f"{standoff:.1f} m standoff, bearing "
                f"{self._last_good_bearing:+.2f}, from {fit['points']} "
                f"lidar returns, pose {self.mav.pos()}, yaw "
                f"{math.degrees(self.mav.yaw()):+.1f} deg")
            self.feedback_message = (
                f"square on: {math.degrees(alpha):+.1f} deg, "
                f"{standoff:.1f} m")
            return py_trees.common.Status.SUCCESS
        self.feedback_message = (f"holding square "
                                 f"{self._stable}/{self.stable_frames}")
        return py_trees.common.Status.RUNNING

    def _recentre_needed(self):
        return (self.mav.banner_identified()
                and abs(self.mav.banner_bearing()) > self.tol)

    def _recentre(self):
        """Put the bounding box midpoint back on the frame midpoint. Yaw only.

        The user's definition of centring, and it is a bearing error that the
        detector already publishes. Discrete and latched, like every other
        correction in this stage, because a per-tick proportional correction
        against a lagging airframe is the receding carrot and this project has
        met it three times.

        This runs whether or not the lidar has anything to say. If the lidar
        is dead the aircraft still faces the banner; it simply never gets the
        confirmation that lets it advance through the gate.
        """
        bearing = self.mav.banner_bearing()
        self._last_good_bearing = bearing
        self._last_seen = self.clock()
        self._remember_vantage()
        self._stable = 0
        if self._corrections >= self.max_corrections:
            # Yaw has had its budget. The banner is in frame and off-centre,
            # which from a square heading is a LATERAL error -- and that is
            # the lidar's business, so fall through to it.
            return None
        theta = self.align_gain * bearing * (self.hfov / 2.0)
        self._align_target = self._wrap(self._align_target - theta)
        self._corrections += 1
        self._sq_phase = self.TURNING
        self._align_t0 = self.clock()
        self.mav.log(
            f"AlignToBanner: banner at bearing {bearing:+.2f} off frame "
            f"centre; yawing to {math.degrees(self._align_target):+.0f} deg "
            f"to face it (camera only, correction "
            f"{self._corrections}/{self.max_corrections})")
        return py_trees.common.Status.RUNNING

    def _no_surface(self, fit):
        """The lidar could not measure. The aircraft does not advance.

        THE ALTITUDE PROBLEM, MEASURED THREE TIMES. The C1 sweeps one
        horizontal plane, and at the altitude the QR scan leaves the aircraft
        at, that plane passes clean over the arena. On seed 1001, 51
        consecutive samples in BANNER_ALIGN at 5.0 m returned 0 finite ranges
        of 720; the same sensor at 3.0 m returned 289. Parked on the shipped
        arena the reading is the same: 0 of 720 at 5.0 m, 63 returns of the
        gate face at 3.0 m. Nothing is wrong with the lidar and nothing is
        wrong with the gate -- they are at different heights.

        So a refusal is first treated as "look from lower down", stepping
        toward a floor. Two things that ladder must NOT do, both learnt from
        watching it:

          * it must not keep descending once it has seen a surface. The
            aircraft found a real face at 3.5 m -- 36 returns, 2 cm residual
            -- turned toward it, measured nothing on the next tick because the
            TURN had moved the sector, and read that as another reason to
            descend. Altitude was not the problem by then.

          * it must not descend with the camera still pitched for a search
            from 5 m, which is what put the board out of the top of the frame
            and left the stage with a working lidar and nowhere to point it.

        And when the ladder is spent, the answer is to stand somewhere else,
        not to keep asking the same question from the same spot.
        """
        self._stable = 0
        ax, ay, az = self._anchor
        seen_before = self._surface is not None
        room_below = az - self.descend_step_m >= self.alt_floor_m - 1e-6
        # NOTHING AT ALL in the sector: the scan plane is clear over the
        # structure, so step twice as far -- still under the board's 1.15 m
        # height, so the plane cannot jump over it. Any return at all and the
        # fine step resumes. Four half-metre steps from 5 m was the watched
        # run's whole descent, ~10 s of sim each with its settle.
        step = self.descend_step_m
        if not fit.get("points"):
            step = max(step, min(2.0 * self.descend_step_m,
                                 az - self.alt_floor_m))
        if not seen_before and self._descents < self.max_descents \
                and room_below:
            self._descents += 1
            self._anchor = (ax, ay, az - step)
            self._sq_phase = self.MOVING
            self._align_t0 = self.clock()
            self._lower_camera()
            self.mav.log(
                f"AlignToBanner: the lidar sees no surface from {az:.1f} m "
                f"({fit['reason']}); descending to "
                f"{az - step:.1f} m to bring the banner into "
                f"the scan plane")
            return py_trees.common.Status.RUNNING

        self._refusals += 1
        if self._refusals >= self.max_refusals:
            return self._relocate(
                f"the lidar found no flat face in the "
                f"{math.degrees(2 * self.sector_half_width):.0f} deg sector "
                f"{math.degrees(self._sector_bearing):+.0f} deg off the nose "
                f"at {az:.1f} m -- {fit['reason']}")
        self.feedback_message = f"no surface measured: {fit['reason']}"
        return py_trees.common.Status.RUNNING


def offset_to_ground(mav, off, hfov_rad=1.0472, image_w_px=1280,
                     image_h_px=720):
    """Local (x, y) of the marker a nadir-camera image offset points at.

    Same image-to-body mapping CenterOnQR uses (forward = -offset.y, right =
    +offset.x), scaled by altitude and field of view.
    """
    alt = mav.alt()
    if off is None or alt is None or alt <= 0.0:
        return None
    half_w = alt * math.tan(hfov_rad / 2.0)
    half_h = half_w * float(image_h_px) / float(image_w_px)
    fwd = -float(off.y) * half_h
    right = float(off.x) * half_w
    psi = mav.yaw()
    x, y = mav.pos()[:2]
    return (x + fwd * math.cos(psi) + right * math.sin(psi),
            y + fwd * math.sin(psi) - right * math.cos(psi))


def note_marker(mav, **camera):
    """Remember where ANY marker in view is on the ground.

    The start marker is found with a narrow camera from above the take-off
    point: on the team airframe's C270 (48.8 deg) a 2.2 m marker 0.9 m ahead
    sits on the frame edge, is seen for a frame and lost, and CenterStartQR
    then held where it was -- where the marker is out of view -- until it
    timed out. With a fix it flies to where the marker was seen.
    """
    off = getattr(mav, "qr_offset", None)
    if off is not None and getattr(off, "z", 0.0) > 0.0:
        g = offset_to_ground(mav, off, **camera)
        if g is not None:
            mav.marker_xy = g


def note_target(mav, **camera):
    """Remember where the MATCHED pad is on the ground, whenever it is seen.

    The sweep matches on one frame at 2.5 m/s and the aircraft has moved on
    before the next stage ticks. Seed 1002: matched, then CenterOnTarget held
    the position it found itself at, never saw the pad again and timed out.
    A ground fix lets every later stage fly back to where the pad IS.
    """
    off = getattr(mav, "qr_offset", None)
    if off is not None and getattr(off, "z", 0.0) >= 0.99:
        g = offset_to_ground(mav, off, **camera)
        if g is not None:
            mav.target_xy = g


class CenterOnQR(py_trees.behaviour.Behaviour):
    """Drive the aircraft until the visible marker sits at frame centre.

    PHASE 3. The camera image offset was measured all along and never used:
    mav_commander subscribed /percep/qr/target_offset and nothing read it.

    Why centring matters rather than just "is it in frame": Phase 1 measured
    decode reliability collapsing beyond roughly a quarter of the half-FOV, and
    collapsing HARD for small markers — a 0.5 m marker managed 12% at 0.25 of
    half-FOV where a 2.2 m one managed 100% (docs/QR_DECODE_ENVELOPE.md). Being
    in frame is not enough; the marker has to be near the middle.

    The offset is normalised [-1,1] with x right and y DOWN in image space.
    With the camera at NADIR and the airframe level, +x image is +y (left) in
    the body ENU frame... which is exactly the kind of sign reasoning that put
    the corridor into a wall in Phase 2. So this works in the LOCAL frame using
    the aircraft's own yaw, and the mapping is asserted in tests.
    """

    def __init__(self, name, mav, tol=0.12, gain=1.2, max_step=1.5,
                 timeout_ticks=120, hold_alt=None, require_match=False,
                 settle_ticks=1, hfov_rad=1.0472, image_w_px=1280,
                 image_h_px=720):
        super().__init__(name)
        self.mav = mav
        self.camera = dict(hfov_rad=hfov_rad, image_w_px=image_w_px,
                           image_h_px=image_h_px)
        self.tol = tol
        self.gain = gain
        self.max_step = max_step
        self.timeout = timeout_ticks
        self.hold_alt = hold_alt
        # Over the delivery zone several markers can share the frame. The
        # detector reports the MATCHED one with z = 1.0; centring on "some
        # marker" there would deliver to the wrong pad.
        self.require_match = bool(require_match)
        # Consecutive centred ticks before success: one centred frame while
        # the aircraft is still moving is a pass-through, not a hold.
        self.settle_ticks = max(1, int(settle_ticks))
        self._t = 0
        self._held = 0
        self._hold_xy = None

    def initialise(self):
        self._t = 0
        self._held = 0
        self._hold_xy = None

    def _visible(self):
        if self.require_match:
            return self.mav.qr_offset.z >= 0.99
        return self.mav.qr_visible()

    def update(self):
        self._t += 1

        if self.require_match:
            note_target(self.mav, **self.camera)
        else:
            note_marker(self.mav, **self.camera)
        if not self._visible():
            self._held = 0
            self.feedback_message = ("matched target not in frame"
                                     if self.require_match else
                                     "no marker visible")
            # Go to where the matched pad was last fixed on the ground; with
            # no fix, hold where the target was last seen rather than drift.
            x, y, z = self.mav.pos()
            fix = getattr(self.mav, "target_xy" if self.require_match
                          else "marker_xy", None)
            if fix is not None:
                self._hold_xy = fix
                self.feedback_message += (f"; returning to its fix "
                                          f"({fix[0]:.1f}, {fix[1]:.1f})")
            elif self._hold_xy is None:
                self._hold_xy = (x, y)
            alt = self.hold_alt if self.hold_alt is not None else z
            self.mav.goto(self._hold_xy[0], self._hold_xy[1], alt,
                          self.mav.yaw())
            if self._t > self.timeout:
                reason = f"{self.name}: no marker to centre on"
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self.mav.qr_centred(self.tol):
            self._held += 1
            x, y, z = self.mav.pos()
            self._hold_xy = (x, y)
            alt = self.hold_alt if self.hold_alt is not None else z
            self.mav.goto(x, y, alt, self.mav.yaw())
            self.feedback_message = (
                f"centred ({self.mav.qr_offset.x:+.2f},"
                f"{self.mav.qr_offset.y:+.2f}) {self._held}/{self.settle_ticks}")
            if self._held >= self.settle_ticks:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING
        self._held = 0

        if self._t > self.timeout:
            reason = (f"{self.name}: failed to centre "
                      f"({self.mav.qr_offset.x:+.2f},{self.mav.qr_offset.y:+.2f}) "
                      f"tol={self.tol}")
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        x, y, z = self.mav.pos()
        alt = self.hold_alt if self.hold_alt is not None else z
        # Image +x is to the right of the frame, image +y is DOWN the frame.
        # With a nadir camera on a level airframe at yaw psi, moving the
        # aircraft forward moves the scene UP the image, and moving right moves
        # the scene LEFT. So to bring the marker to centre:
        #   forward correction  = -offset.y
        #   rightward correction = +offset.x
        fwd = -self.mav.qr_offset.y * self.gain
        right = self.mav.qr_offset.x * self.gain
        fwd = max(-self.max_step, min(self.max_step, fwd))
        right = max(-self.max_step, min(self.max_step, right))

        psi = self.mav.yaw()
        dx = fwd * math.cos(psi) + right * math.sin(psi)
        dy = fwd * math.sin(psi) - right * math.cos(psi)

        self.mav.goto(x + dx, y + dy, alt, psi)
        self._hold_xy = (x, y)
        self.feedback_message = (f"centring ({self.mav.qr_offset.x:+.2f},"
                                 f"{self.mav.qr_offset.y:+.2f})")
        return py_trees.common.Status.RUNNING


class FindStartQR(py_trees.behaviour.Behaviour):
    """Descend-and-retry ladder until a marker is visible below.

    PHASE 3, and the reason it exists: `scan_pose` was hardcoded to (0,0,5) —
    directly over the takeoff point — while the start pad sits about a metre
    away. At 5 m that puts the marker at roughly a third of the half-FOV, in
    frame but outside the reliable decode zone Phase 1 measured. The mission
    hovered over a guess and hoped.

    Instead: hold station, and if nothing is visible after a while, step down
    (a lower altitude both enlarges the marker and narrows the search area) and
    look again. Fails closed when the ladder bottoms out — the aircraft is not
    going to find a marker that is not there by descending into the ground.
    """

    def __init__(self, mav, start_alt, floor_alt=2.0, step=1.0,
                 dwell_ticks=40, hfov_rad=1.0472, image_w_px=1280,
                 image_h_px=720):
        super().__init__("FindStartQR")
        self.mav = mav
        self.camera = dict(hfov_rad=hfov_rad, image_w_px=image_w_px,
                           image_h_px=image_h_px)
        self.start_alt = start_alt
        self.floor_alt = floor_alt
        self.step = step
        self.dwell = dwell_ticks
        self._alt = start_alt
        self._t = 0
        self._x = 0.0
        self._y = 0.0

    def initialise(self):
        self._alt = self.start_alt
        self._t = 0
        self._x, self._y = self.mav.pos()[:2]

    def update(self):
        self._t += 1
        self.mav.goto(self._x, self._y, self._alt, self.mav.yaw())

        if self.mav.qr_visible():
            # Where it is, for CenterStartQR if it slips out of frame again.
            note_marker(self.mav, **self.camera)
            self.feedback_message = f"marker visible at {self._alt:.1f} m"
            return py_trees.common.Status.SUCCESS

        if self._t >= self.dwell:
            self._t = 0
            nxt = self._alt - self.step
            if nxt < self.floor_alt:
                reason = (f"no start marker found between {self.start_alt:.1f} "
                          f"and {self.floor_alt:.1f} m")
                self.feedback_message = reason
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            self._alt = nxt
            self.feedback_message = f"descending to {self._alt:.1f} m"

        return py_trees.common.Status.RUNNING


class ScanStartQR(py_trees.behaviour.Behaviour):
    """Hold at scan pose and decode the start QR. FAILS CLOSED.

    This was the handoff's headline defect: the leaf returned SUCCESS after a
    fixed number of ticks even with an empty decoded string, so a mission that
    had never read its target carried on to deliver to nobody. It is now:

      SUCCESS  only with a payload confirmed over `confirm_frames` consecutive
               frames, or an explicit operator override
      FAILURE  on timeout — the mission stops and says why
      RUNNING  otherwise

    The consecutive-frame requirement matters because the mission commits to a
    delivery target on the strength of this read; one frame of motion blur or a
    reflection should not decide it.

    goal.md Q19 allows the operator to supply the target by hand if the start
    QR is physically unreadable. That path is honoured here, but it is an
    explicit, logged substitution rather than a silent empty string.
    """

    def __init__(self, mav, pose=None, timeout_ticks=150, confirm_frames=3,
                 hover_s=5.0, clock=None):
        super().__init__("ScanStartQR")
        self.mav = mav
        # Hold over the marker after reading it, so the decode is something an
        # operator can watch happen rather than find in a log afterwards.
        self.hover = DecodeHover(hover_s=hover_s, clock=clock)
        self._confirmed = None
        # pose=None means "hold wherever centring left us", which is the whole
        # point of Phase 3: the previous hardcoded scan_pose (0,0,5) assumed the
        # marker was under the takeoff point and it is not.
        self.pose = pose
        self.timeout = timeout_ticks
        self.confirm_frames = confirm_frames
        self._t = 0
        self._hold = None

    def initialise(self):
        self._t = 0
        self._hold = None
        self._confirmed = None
        self.hover.reset()

    def update(self):
        self._t += 1
        if self.pose is not None:
            self.mav.goto(*self.pose)
        else:
            if self._hold is None:
                x, y, z = self.mav.pos()
                self._hold = (x, y, z, self.mav.yaw())
            self.mav.goto(*self._hold)

        if self.mav.target_override:
            self.mav.set_target(self.mav.target_override)
            self.feedback_message = f"OPERATOR target={self.mav.target_override}"
            return py_trees.common.Status.SUCCESS

        # LATCH the payload the moment it is confirmed. The hover that
        # follows holds station for several seconds, and without the latch a
        # single blurred frame in that window resets the decode streak, drops
        # the stage back to "still waiting", and fails the mission with
        # "start QR not decoded" about a marker it is hovering over.
        if self._confirmed is None and self.mav.qr_confident(self.confirm_frames):
            self._confirmed = self.mav.qr_decoded

        if self._confirmed is not None:
            if self.hover.tick(self.mav):
                self.feedback_message = (f"read '{self._confirmed}'; "
                                         f"holding over it")
                return py_trees.common.Status.RUNNING
            self.mav.set_target(self._confirmed)
            # The home pad's own position, for the landing: the aircraft is
            # centred over it here. Other markers overwrite `marker_xy` later.
            self.mav.home_marker_xy = tuple(self.mav.pos()[:2])
            self.feedback_message = f"target={self._confirmed}"
            return py_trees.common.Status.SUCCESS

        if self._t > self.timeout:
            reason = (f"start QR not decoded after {self._t} ticks "
                      f"(last='{self.mav.qr_decoded}', "
                      f"streak={self.mav.qr_streak}/{self.confirm_frames}); "
                      f"publish /mission/target_override to proceed manually")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        self.feedback_message = (f"scanning ({self._t}/{self.timeout}) "
                                 f"streak={self.mav.qr_streak}")
        return py_trees.common.Status.RUNNING


def _blocked(stage, router):
    """Every stage answers a blocked leg the same way: stop, and say why.

    Not a shared helper for tidiness -- a shared helper so that no stage can
    quietly grow its own softer answer. There is exactly one correct response
    to "this leg crosses a restricted zone", and flying it is not it.
    """
    reason = f"{stage.name}: {router.blocked_reason}"
    stage.feedback_message = reason
    stage.mav.abort_reason = reason
    return py_trees.common.Status.FAILURE


class Corridor(py_trees.behaviour.Behaviour):
    """Hand control to the corridor navigator until it reports the way out.

    There is deliberately NO exit_x. The stage used to end when the aircraft
    crossed a hardcoded x threshold (audit A6, A10), which assumed the
    corridor's length and placement were known in advance — and the return
    threshold turned out to be unreachable, stalling three live runs. Exit is
    now observed: both walls fall away and the navigator says so.

    Altitude is still checked, because a live run reported CORRIDOR_NAV while
    the aircraft dragged along the ground at z = 0.117 m — horizontal progress
    was being made, so nothing objected.

    `forward` is retained only for logging/telemetry; the navigator is
    direction-agnostic because it steers toward gaps, not along an axis.
    """

    def __init__(self, name, mav, forward=True, alt=None, alt_band=1.5):
        super().__init__(name)
        self.mav = mav
        self.forward = forward
        self.alt = alt
        self.alt_band = alt_band

    def update(self):
        z = self.mav.alt()

        if self.alt is not None and abs(z - self.alt) > self.alt_band:
            self.mav.enable_avoidance(False)
            reason = (f"{self.name}: altitude {z:.2f} m outside "
                      f"{self.alt:.1f} +/- {self.alt_band:.1f} m band")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        # Tell the navigator which altitude this traversal must hold. It
        # otherwise commands zero vertical RATE and calls that "hold altitude"
        # -- see MavCommander.enable_avoidance. This stage already knows the
        # answer: it is the band it is about to fail the mission over.
        self.mav.enable_avoidance(True, hold_alt=self.alt)
        px = self.mav.pos()[0]

        # The navigator reports STUCK once its recovery ladder is exhausted.
        # Believing it beats hovering against an obstacle indefinitely, which
        # is what two live runs did before this existed.
        if self.mav.avoidance_stuck():
            self.mav.enable_avoidance(False)
            reason = f"{self.name}: corridor navigator reported STUCK at x={px:.2f}"
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        # PERCEPTION decides the corridor has ended: both walls have fallen
        # away. Replaces corridor_exit_x / corridor_return_exit_x, which
        # assumed the corridor's length was known in advance (audit A6, A10).
        if self.mav.corridor_exited():
            self.mav.enable_avoidance(False)
            # Where the corridor opened out is the only thing that can anchor
            # the delivery zone and the return leg without asserting arena
            # coordinates (audit A7, A9). Record it while we are standing in it.
            self.mav.record_corridor_exit()
            self.feedback_message = f"corridor opened out at x={px:.2f}"
            return py_trees.common.Status.SUCCESS

        d = self.mav.avoid_detail
        self.feedback_message = (f"x={px:.1f} z={z:.2f} "
                                 f"L={d.get('side_left_m')} R={d.get('side_right_m')} "
                                 f"{d.get('state')}")
        return py_trees.common.Status.RUNNING


class DuckUnderBoard(py_trees.behaviour.Behaviour):
    """Find the board's bottom edge and drop below it before advancing.

    WHAT WAS WATCHED

        The aircraft squared up to the banner correctly, flew forward into the
        board, deflected off it, and left the world. The operator's read: "it
        is not even avoiding it by going beneath the banner using the lidar."

    THE DEFECT, IN TWO NUMBERS

        The advance flew at the corridor altitude, 3.0 m. The board spans
        2.805 to 3.955 m. No perception failed: the aircraft measured the
        board's angle correctly and then drove through the thing it had just
        measured.

    THE TRAP

        Squaring up and passing through are MUTUALLY EXCLUSIVE ALTITUDES, and
        that is geometry rather than a tuning accident. The lidar can only
        measure the board's angle where the scan plane cuts the board -- the
        usable band came out as 2.5 to 3.5 m -- and that is precisely the
        height at which the aircraft would hit it. Putting the align altitude
        in the middle of the lidar band put it in the middle of the collision.
        It was invisible for as long as the aircraft could not square up,
        because it never got far enough to advance.

    HOW THE EDGE IS FOUND

        Not written down. The standing constraint is no fixed arena geometry
        and the real gate will differ, so the aircraft measures it. The lidar
        gives two unmistakable signatures, both recorded on the shipped arena:

            at board height   one continuous face, ~60 returns, residual
                              under 1 cm
            below the board   two post clusters with a 3.8 m hole between them

        The altitude at which the first becomes the second IS the bottom edge.
        Descend in steps, watch for the transition, then drop a clearance
        margin below it.

    REFUSES RATHER THAN GUESSING. If the transition is never seen, or the
    floor arrives first, the stage fails. It does NOT fall back to advancing
    at the align altitude -- that fallback is the collision.
    """

    def __init__(self, mav, step_m=0.4, floor_m=1.2, margin_m=0.6,
                 need_clear_m=10.0, settle_s=1.5, arrive_tol_m=0.2,
                 sector_half_width_rad=math.radians(35.0),
                 hfov_rad=1.0472, clock=None, max_steps=12,
                 crossing_margin_m=0.6, obstacle_margin_m=0.4):
        # The margins must fit the gap between the posts and the first
        # obstacle behind them. On the shipped arena the return lane's first
        # slalom block stands 1.2 m past the return gate; 1.0 + 0.75 m could
        # never be satisfied there, so the return lap refused at every run.
        # 0.6 m puts the Iris's rear props past the 0.12 m board, and the
        # corridor navigator -- not this crossing leg -- steers round the
        # block from there.
        super().__init__("DuckUnderBoard")
        self.mav = mav
        self.step_m = float(step_m)
        self.floor_m = float(floor_m)
        self.margin_m = float(margin_m)
        self.need_clear_m = float(need_clear_m)
        self.settle_s = float(settle_s)
        self.arrive_tol_m = float(arrive_tol_m)
        self.sector_half_width = float(sector_half_width_rad)
        self.hfov = float(hfov_rad)
        self.max_steps = int(max_steps)
        self.crossing_margin_m = float(crossing_margin_m)
        self.obstacle_margin_m = float(obstacle_margin_m)
        self.clock = clock or time.monotonic
        self._reset()

    def _reset(self):
        self._anchor = None
        self._t0 = None
        self._tried = []
        self._saw_board = False
        self._edge_alt = None
        self._target_alt = None
        self._steps = 0
        self._advance_m = None
        self._heading = None

    def initialise(self):
        self._reset()
        # Takeoff arms a general low-altitude guard at 40% of takeoff height.
        # This stage deliberately flies below that floor, so move the guard to
        # just under this stage's own hard floor before commanding the descent.
        # The guard still catches a sink toward the ground; it no longer treats
        # the measured gate-opening maneuver as a fault.
        self.mav.set_airborne_floor(max(0.5,
                                        self.floor_m - self.arrive_tol_m))

    @property
    def alt(self):
        """The altitude the advance should fly at. None until measured."""
        return self._target_alt

    @property
    def advance_m(self):
        """Measured crossing distance, published only after confirmation."""
        return self._advance_m

    def _sector(self):
        # GateAdvance follows the aligned heading. Checking a camera bearing
        # off that heading would certify a different flight path.
        return 0.0

    def _hold(self, z):
        x, y, _ = self._anchor
        self.mav.goto(x, y, z, self._heading)

    def _give_up(self, why):
        tried = "; ".join(f"{a:.1f} m: {r}" for a, r in self._tried)
        reason = (f"DuckUnderBoard: {why}. Altitudes tried -- {tried}. "
                  f"Refusing to advance at board height")
        self.feedback_message = reason
        self.mav.abort_reason = reason
        self.mav.log(reason, warn=True)
        return py_trees.common.Status.FAILURE

    def update(self):
        if self._anchor is None:
            x, y, z = self.mav.pos()
            self._anchor = (x, y, z)
            self._heading = self.mav.yaw()
            self._t0 = self.clock()

        z_cmd = (self._target_alt if self._target_alt is not None
                 else self._anchor[2])
        self._hold(z_cmd)

        # Wait until the aircraft is actually AT the height being asked about.
        # A reading taken on the way down is a reading about somewhere else.
        arrived = abs(self.mav.alt() - z_cmd) <= self.arrive_tol_m
        if not arrived or self.clock() - self._t0 < self.settle_s:
            self.feedback_message = (
                f"settling at {z_cmd:.1f} m to look for the board's lower "
                f"edge ({self.mav.alt():.1f} m now)")
            return py_trees.common.Status.RUNNING

        opening = self.mav.opening_ahead(self._sector(),
                                         self.sector_half_width,
                                         need_clear_m=0.0)
        here = self.mav.alt()
        gate = opening.get("gate_m")
        distance = None
        if opening["open"] and gate is not None and math.isfinite(gate):
            candidate = gate + self.crossing_margin_m
            if (0.0 < candidate <= self.need_clear_m
                    and opening["clear_m"] >= candidate + self.obstacle_margin_m):
                distance = candidate
            else:
                opening = dict(opening, open=False, reason=(
                    f"gate at {gate:.2f} m, something standing at "
                    f"{opening['clear_m']:.2f} m; cannot cross with clearance"))
        elif opening["open"]:
            opening = dict(opening, open=False,
                           reason="no measured gate distance")

        if self._edge_alt is not None:
            # Below the edge already; this is the confirming look.
            if opening["open"]:
                self._advance_m = distance
                self.mav.log(
                    f"DuckUnderBoard: under the board at {here:.1f} m -- a "
                    f"{opening['gap_m']:.1f} m gap clear for "
                    f"{opening['clear_m']:.1f} m; edge measured at "
                    f"{self._edge_alt:.1f} m, flying {self.margin_m:.1f} m "
                    f"below it; gate {gate:.2f} m, advance {distance:.2f} m, "
                    f"pose {self.mav.pos()}, heading "
                    f"{math.degrees(self._heading):+.1f} deg")
                self.feedback_message = (f"under the board at {here:.1f} m, "
                                         f"gap {opening['gap_m']:.1f} m")
                return py_trees.common.Status.SUCCESS
            self._tried.append((here, opening["reason"][:60]))
            return self._give_up(
                f"dropped below the measured edge at {self._edge_alt:.1f} m "
                f"and the way through did not open -- {opening['reason']}")

        self._tried.append((here, opening["reason"][:60] or
                            f"open, {opening['gap_m']:.1f} m gap"))

        if not opening["open"]:
            if opening["clusters"] == 1:
                self._saw_board = True
            if self._steps >= self.max_steps or \
                    here - self.step_m < self.floor_m:
                return self._give_up(
                    "reached the floor without the board's lower edge ever "
                    "appearing")
            self._steps += 1
            self._target_alt = here - self.step_m
            self._t0 = self.clock()
            self.mav.log(
                f"DuckUnderBoard: at {here:.1f} m the lidar still sees "
                f"{opening['reason']}; descending to {self._target_alt:.1f} m "
                f"to find the board's lower edge")
            return py_trees.common.Status.RUNNING

        # THE TRANSITION. One face became a hole; this height is the edge.
        if not self._saw_board:
            return self._give_up(
                f"a way through was already open at {here:.1f} m without the "
                f"board ever being seen above it, so nothing here has been "
                f"identified as the gate")
        self._edge_alt = here
        self._target_alt = max(self.floor_m, here - self.margin_m)
        self._t0 = self.clock()
        self.mav.log(
            f"DuckUnderBoard: the board's lower edge is at {here:.1f} m -- "
            f"one face became {opening['clusters']} posts with a "
            f"{opening['gap_m']:.1f} m gap between them; dropping to "
            f"{self._target_alt:.1f} m to fly under it")
        return py_trees.common.Status.RUNNING


class GateAdvance(py_trees.behaviour.Behaviour):
    """Fly a measured distance THROUGH the gate, avoiding what is in the way.

    WHAT THIS REPLACES

        ApproachBanner: 1.5 m steps toward the banner, ending when the banner
        left the top of the frame. That end condition is "I got close to it",
        not "I went through it" -- so the mission treated a DETECTION as
        arrival and carried on into the delivery-zone stages while still at
        the gate. Watched live: "the drone thinks it has been into the drop
        location because it detected the banner".

        It also drove at the board. The banner marks the mouth; the aircraft
        has to pass through it, and how far through is a distance, not a
        detector state.

    WHY A LATCHED WAYPOINT AND NOT A CARROT

        The target is computed once, from where the aircraft was when the gate
        was identified, and re-issued until reached. Recomputing it from the
        current position every tick is the receding carrot that pitched the
        aircraft to 50 degrees in arena 1002.

    Obstacle avoidance is on for the whole leg: the corridor walls are close
    and the run has already confirmed red ground by this point.
    """

    def __init__(self, mav, advance_m=10.0, alt=3.0, tol=1.0,
                 timeout_ticks=600, clearance_m=DEFAULT_CLEARANCE_M,
                 exclusions=()):
        super().__init__("GateAdvance")
        self.mav = mav
        self.advance_m = advance_m
        self.alt = alt
        self.tol = float(tol)
        self.timeout_ticks = int(timeout_ticks)
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)
        self.exclusions = exclusions
        self._target = None
        self._flight_alt = None
        self._flight_distance = None
        self._t = 0

    def _current_exclusions(self):
        ex = self.exclusions
        if callable(ex):
            ex = ex()
        return list(ex or [])

    def initialise(self):
        self._target = None
        self._flight_alt = None
        self._flight_distance = None
        self._t = 0
        self.router.reset()

    def terminate(self, new_status):
        self.mav.enable_avoidance(False)

    def update(self):
        self._t += 1
        x, y, z = self.mav.pos()

        if self._flight_alt is None:
            value = self.alt() if callable(self.alt) else self.alt
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value) or value <= 0.0:
                reason = ("GateAdvance: no safe transit altitude was "
                          "measured; refusing to advance through the gate")
                self.feedback_message = reason
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            self._flight_alt = value

        if self._target is None:
            value = self.advance_m() if callable(self.advance_m) else self.advance_m
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value) or value <= 0.0:
                reason = "GateAdvance: no safe transit distance was measured"
                self.feedback_message = self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            self._flight_distance = value
            psi = self.mav.yaw()
            # Square on the gate, so this is the corridor axis. The return
            # stand-off is laid out along it (first write wins: outbound).
            record = getattr(self.mav, "record_gate_heading", None)
            if callable(record):
                record(psi)
            # The board stands about the measured gate distance ahead
            # (advance = gate + crossing margin). First write: outbound.
            rec_b = getattr(self.mav, "record_outbound_banner", None)
            if callable(rec_b):
                g = max(0.0, value - 0.6)
                rec_b(x + g * math.cos(psi), y + g * math.sin(psi))
            self._target = (x + value * math.cos(psi),
                            y + value * math.sin(psi))
            # ONE CONTROLLER AT A TIME.
            #
            # This used to hand control to the follow-the-gap navigator AND
            # keep streaming position setpoints at the target, because
            # `enable_avoidance(True)` clears the streamed setpoint and the
            # router's next tick sets it again. Both then publish, ten times a
            # second, and they do not want the same thing.
            #
            # MEASURED, run 18: the stage squared up, latched a target 10 m
            # ahead at (11.6, -4.8), and the aircraft finished at (16.0,
            # -68.0) -- sixty-three metres south, at a steady 3.0 m, into open
            # field. Follow-the-gap steers toward the widest opening and has
            # no notion of a destination, so given a wall on one side and an
            # empty arena on the other it flies at the arena. That is right
            # for traversing a corridor and wrong for covering a measured
            # distance to a point.
            #
            # The avoidance that belongs on this leg is the exclusion routing
            # the LegRouter already does: it detours the airframe around
            # confirmed red ground, and it refuses rather than flying a leg it
            # cannot make legal. The corridor stage that follows is where the
            # gap navigator earns its keep.
            self.mav.enable_avoidance(False)
            self.mav.log(
                f"gate identified and aligned; advancing {self._flight_distance:.2f} m "
                f"through it to ({self._target[0]:.1f}, {self._target[1]:.1f}) "
                f"at {math.degrees(psi):+.0f} deg, routed around "
                f"{len(self._current_exclusions())} confirmed exclusion(s)")

        if math.hypot(self._target[0] - x, self._target[1] - y) <= self.tol:
            self.feedback_message = f"advanced {self._flight_distance:.2f} m through the gate"
            return py_trees.common.Status.SUCCESS

        if self._t > self.timeout_ticks:
            reason = (f"GateAdvance: {self._flight_distance:.2f} m not covered in "
                      f"{self._t} ticks")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        status = self.router.fly(self.mav, self._target[0], self._target[1],
                                 self._flight_alt, self.mav.yaw())
        if status is BLOCKED:
            return _blocked(self, self.router)
        self.feedback_message = (
            f"advancing through the gate, "
            f"{math.hypot(self._target[0] - x, self._target[1] - y):.1f} m to go")
        return py_trees.common.Status.RUNNING


# FC parameters the arena fence needs before FENCE_ENABLE means anything.
#   FENCE_TYPE 5     = max altitude (1) + polygon (4)
#   FENCE_ACTION 1   = RTL, or LAND if RTL is impossible
#   FENCE_ALT_MAX    above the 10 m rulebook ceiling and above RTL_ALT (15 m),
#                    so a breach-triggered RTL cannot itself breach
#   FENCE_MARGIN     how far inside the fence ArduPilot starts to object
ARENA_FENCE_PARAMS = (("FENCE_TYPE", 5), ("FENCE_ACTION", 1),
                      ("FENCE_ALT_MAX", 20.0), ("FENCE_MARGIN", 2.0))


class UploadArenaFence(py_trees.behaviour.Behaviour):
    """Program the ORGANISER'S geofence into the flight controller, pre-arm.

    RULEBOOK, Mission 2 Operation: "Coordinates for the geo-fence boundary
    will be provided. Teams must program these into the ground station
    software to ensure the UAS stays within the designated area." Technical
    inspection scores it too (5.4, item 6.3).

    The fence used to be drawn in flight around a lidar estimate of the
    delivery zone, stretched by a 60 m search budget and never enabled -- so
    the one thing meant to hold the aircraft in the arena held nothing, and
    the search spiralled out of the area with nothing to stop it.

    Now: the supplied polygon, uploaded before arming, read back vertex by
    vertex, the fence parameters set, then FENCE_ENABLE=1 confirmed. Red-zone
    exclusions are NOT added here: they are camera-derived and appear in
    flight, and an exclusion fence the aircraft is already inside is a
    breach. The leg router keeps the airframe off red ground; this fence
    keeps it in the arena.

    FATAL: without a verified, enforced fence the aircraft does not arm.
    """

    def __init__(self, mav, timeout_ticks=900, home_margin_m=2.0,
                 zone_clearance_m=0.0, params=ARENA_FENCE_PARAMS):
        super().__init__("UploadArenaFence")
        self.mav = mav
        self.timeout_ticks = int(timeout_ticks)
        self.home_margin_m = float(home_margin_m)
        self.zone_clearance_m = float(zone_clearance_m)
        self.params = tuple(params) + (("FENCE_ENABLE", 1),)
        self._reset()

    def _reset(self):
        self._t = 0
        self._sent = None
        self._future = None
        self._param_i = 0
        self._param_future = None

    def initialise(self):
        self._reset()
        self.mav.fence_verified = False
        self.mav.fence_reason = "not uploaded"

    def _fail(self, reason):
        self.mav.fence_reason = reason
        self.feedback_message = reason
        self.mav.abort_reason = f"UploadArenaFence: {reason}"
        self.mav.log(self.mav.abort_reason, warn=True)
        return py_trees.common.Status.FAILURE

    def _geometry_ok(self, poly):
        hx, hy = self.mav.home_local_xy()
        if not point_inside_with_margin(hx, hy, poly, self.home_margin_m):
            return False, (f"home ({hx:.1f}, {hy:.1f}) is not at least "
                           f"{self.home_margin_m:.1f} m inside the geofence")
        zone = self.mav.delivery_search_zone(self.zone_clearance_m)
        if zone is not None:
            for cx, cy in rect_vertices(zone):
                if not point_in_polygon(cx, cy, poly):
                    return False, (f"delivery-zone corner ({cx:.1f}, {cy:.1f}) "
                                   "lies outside the geofence")
        return True, ""

    def update(self):
        self._t += 1
        home = self.mav.home_global()
        poly = getattr(self.mav, "geofence_local", None)
        if home is None or poly is None:
            if self._t < self.timeout_ticks:
                self.feedback_message = "waiting for home position and geofence"
                return py_trees.common.Status.RUNNING
            return self._fail(getattr(self.mav, "geofence_reason",
                                      "no home position or geofence"))

        if self._sent is None:
            ok, why = self._geometry_ok(poly)
            if not ok:
                return self._fail(why)
            from mavros_msgs.msg import Waypoint
            self._sent = build_fence(poly, [], home[0], home[1], Waypoint)
            self._future = self.mav.push_fence(self._sent)
            if self._future is None:
                if self._t < self.timeout_ticks:
                    self._sent = None
                    self.feedback_message = "waiting for the geofence push service"
                    return py_trees.common.Status.RUNNING
                return self._fail("geofence push service unavailable")
            self.feedback_message = f"pushing {len(self._sent)} fence vertices"
            return py_trees.common.Status.RUNNING

        got = self.mav.fence_readback
        if got is None or len(got) != len(self._sent):
            if self._t < self.timeout_ticks:
                self.feedback_message = "awaiting fence read-back"
                return py_trees.common.Status.RUNNING
            return self._fail("fence never read back intact")
        ok, why = compare_fences(self._sent, got, home[0], home[1])
        if not ok:
            return self._fail(why)

        while self._param_i < len(self.params):
            name, value = self.params[self._param_i]
            if self._param_future is None:
                self._param_future = self.mav.set_param(name, value)
                if self._param_future is None:
                    if self._t < self.timeout_ticks:
                        self.feedback_message = "waiting for the param service"
                        return py_trees.common.Status.RUNNING
                    return self._fail("MAVROS param service unavailable")
            if not self._param_future.done():
                if self._t >= self.timeout_ticks:
                    return self._fail(f"no reply setting {name}")
                self.feedback_message = f"setting {name}={value}"
                return py_trees.common.Status.RUNNING
            result = self._param_future.result()
            self._param_future = None
            if not (result and result.success):
                # MAVROS refuses sets until its parameter download finishes,
                # which on a slow link takes a while. Retry until timeout.
                if self._t < self.timeout_ticks:
                    self.feedback_message = f"{name} refused; retrying"
                    return py_trees.common.Status.RUNNING
                return self._fail(f"flight controller refused {name}={value}")
            self._param_i += 1

        self.mav.fence_verified = True
        self.mav.fence_reason = "organiser geofence verified and enforced"
        self.feedback_message = (f"geofence verified: {len(poly)} vertices, "
                                 "FENCE_ENABLE=1")
        self.mav.log(self.feedback_message)
        return py_trees.common.Status.SUCCESS


class ClimbInPlace(py_trees.behaviour.Behaviour):
    """Change to a commanded altitude without translating (up or down).

    Replaces Goto("Climb10", ...) which both moved to an asserted zone-entry
    coordinate and climbed to a constant. The altitude comes from the search
    plan, so it is whatever the camera and marker imply; the horizontal
    position is simply held.
    """

    def __init__(self, name, mav, alt, tol=0.6):
        super().__init__(name)
        self.mav = mav
        self._alt = alt
        self.tol = tol

    def target_alt(self):
        return float(self._alt() if callable(self._alt) else self._alt)

    def update(self):
        x, y, z = self.mav.pos()
        target = self.target_alt()
        # Hold the CURRENT yaw. goto()'s yaw argument defaults to 0.0, so
        # omitting it would spin the aircraft back to due east and throw away
        # the banner heading the previous stage just acquired.
        self.mav.goto(x, y, target, self.mav.yaw())
        self.feedback_message = f"changing altitude {z:.1f} -> {target:.1f} m"
        return (py_trees.common.Status.SUCCESS
                if abs(z - target) <= self.tol
                else py_trees.common.Status.RUNNING)


class EnterDeliveryZone(py_trees.behaviour.Behaviour):
    """Move from the corridor exit to the nearest safe point in the field."""

    def __init__(self, mav, alt=3.0, clearance_m=DEFAULT_CLEARANCE_M,
                 tol=0.5):
        super().__init__("EnterDeliveryZone")
        self.mav = mav
        self.alt = float(alt)
        self.clearance_m = float(clearance_m)
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)
        self._target = None

    def initialise(self):
        self.router.reset()
        self._target = None

    def update(self):
        zone = self.mav.delivery_search_zone(self.clearance_m)
        if zone is None:
            reason = ("EnterDeliveryZone: " +
                      str(getattr(self.mav, "delivery_zone_reason",
                                  "delivery-zone boundary unavailable")))
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        if self._target is None:
            self._target = nearest_point_in_zone(self.mav.pos()[:2], zone)
            self.mav.log(
                f"entering supplied delivery field at ({self._target[0]:.1f}, "
                f"{self._target[1]:.1f}) with {self.clearance_m:.1f} m "
                "boundary clearance")
        status = self.router.fly(self.mav, self._target[0], self._target[1],
                                 self.alt, self.mav.yaw())
        if status is BLOCKED:
            return _blocked(self, self.router)
        self.feedback_message = (
            f"entering delivery field at ({self._target[0]:.1f}, "
            f"{self._target[1]:.1f})")
        return (py_trees.common.Status.SUCCESS if status is ARRIVED
                else py_trees.common.Status.RUNNING)


class LawnmowerSearch(py_trees.behaviour.Behaviour):
    """Sweep the delivery zone on a lane plan DERIVED from the camera.

    Phase 6. Previously the altitude (10.0 m) and lane spacing (6.0 m, while
    goal.md Q9 said 5.0) were constants that contradicted each other and were
    not derived from anything. Phase 1 measured the decode floor
    (docs/QR_DECODE_ENVELOPE.md); mission_bt.search_planner turns that plus the
    camera geometry and the marker size into altitude, spacing and a coverage
    figure. Closes geometry audit A2 and B1.

    Marker size is an INPUT — the competition value is still unconfirmed, so
    the plan is computed for whatever it turns out to be rather than baked in.
    """

    def __init__(self, mav, zone, alt, spacing=None,
                 image_width_px=1280, hfov_rad=1.0472, marker_m=2.2,
                 modules=33, px_floor=5.3, overlap=0.30, clearance_m=1.5,
                 exclusions=None, search_budget_m=0.0, min_step_m=4.0,
                 hover_s=5.0, clock=None, image_height_px=None,
                 crab=False, search_speed_mps=None, fix_wait_ticks=10):
        """`zone` may be an (x0, x1, y0, y1) tuple or a CALLABLE returning one.

        `crab` flies the lanes yawed 90 degrees to their direction, so the
        image's LONG axis looks along the lane. Red ground is only avoided if
        it is seen, confirmed and routed round before the airframe gets
        there; flying nose-along-lane gave the camera 3.2 m of look-ahead at
        10 m, and live run 2 on the shipped arena entered red ground six
        times, every one of them mid-lane. Crabbed, the look-ahead is 5.8 m
        and lanes are spaced for the narrower across-track swath.
        `search_speed_mps` caps the ground speed for the same reason.

        A callable is what the mission uses: the zone is not known until
        ObserveZone has measured it, which happens long after the tree is
        built. A plain tuple is still accepted so the planner can be exercised
        against a fixed zone in tests.

        `search_budget_m` is how much further than the first observed window
        the aircraft may push the frontier before giving up. Zero keeps the
        old behaviour of sweeping the window once. See _advance_frontier().
        """
        super().__init__("LawnmowerSearch")
        self.mav = mav
        self._zone_src = zone
        self.crab = bool(crab)
        self._image_h = int(image_height_px or round(image_width_px * 9 / 16))
        swath_fov = hfov_rad
        if self.crab and image_height_px:
            # Across-track, a crabbed sweep sees the image HEIGHT.
            swath_fov = 2.0 * math.atan(math.tan(hfov_rad / 2.0)
                                        * float(image_height_px)
                                        / float(image_width_px))
        self.search_speed_mps = search_speed_mps
        # Ticks to hold, on a match, for the offset that places the pad.
        self.fix_wait_ticks = int(fix_wait_ticks)
        self._fix_wait = 0
        self._fix_hold = None
        self._speed_sent = False
        self._plan_args = dict(image_width_px=image_width_px,
                               hfov_rad=hfov_rad, marker_m=marker_m,
                               modules=modules, px_floor=px_floor,
                               overlap=overlap, max_alt=alt, axis="auto",
                               swath_fov=swath_fov)
        self.plan = None
        self.exclusions = []
        self.clearance_m = clearance_m
        # Lane SEGMENTS were clipped from the start. The transit from the end
        # of one segment to the start of the next never was, and on a zone with
        # red ground down the middle that transit is the diagonal straight
        # across it.
        self.router = LegRouter(clearance_m=clearance_m, tol=0.8)
        self.skipped = 0
        # Every DISTINCT marker the sweep passes over gets a pause, not only
        # the one that matches. Per-decode would park the aircraft over the
        # first pad until the mission clock ran out.
        self.hover = DecodeHover(hover_s=hover_s, clock=clock)
        self.alt = alt
        self.decode_alt = alt
        self.wps = []
        self._exclusions_src = exclusions
        self.i = 0
        self.search_budget_m = float(search_budget_m)
        self.min_step_m = float(min_step_m)
        self._budget_left = float(search_budget_m)
        self._frontier = None       # zone last swept; the strip to extend from
        self._heading = 0.0
        self._window_depth = 0.0    # set in initialise() from the observation
        self._replans = 0
        # Re-plans are no longer rationed (see _resume_index), only spaced:
        # red ground is confirmed cell by cell, so the exclusion set changes
        # most ticks while it is in view.
        self.replan_min_ticks = 10
        self._ticks_since_replan = self.replan_min_ticks
        self.expansions = 0
        self.swept_zone = None      # union of every strip actually swept
        # Forward first -- the zone usually opens out along the corridor --
        # then either side. See _advance_frontier().
        self.EXPANSION_DIRECTIONS = (0.0, math.pi / 2, -math.pi / 2)
        # Inside a SUPPLIED boundary there is nothing beyond it to expand into.
        # A full pass that misses (a pad beside a clipped lane, a frame lost
        # to motion) is followed by ONE cross-grid pass on the other axis,
        # also from the bottom-left corner, instead of spiralling outward.
        self.max_passes = 2
        self._pass = 0
        self._axis_override = None
        if not callable(zone):
            self._replan(zone, self._current_exclusions())

    def _replan(self, zone, exclusions=(), extend_swept=False):
        a = self._plan_args
        self.plan = plan_search(zone, a['image_width_px'], a['hfov_rad'],
                                a['marker_m'], a['modules'],
                                px_per_module_floor=a['px_floor'],
                                overlap=a['overlap'],
                                max_altitude=a['max_alt'],
                                axis=self._axis_override or a['axis'],
                                swath_fov_rad=a['swath_fov'])
        self._frontier = tuple(zone)
        if extend_swept and self.swept_zone:
            s = self.swept_zone
            self.swept_zone = (min(s[0], zone[0]), max(s[1], zone[1]),
                               min(s[2], zone[2]), max(s[3], zone[3]))
        else:
            self.swept_zone = tuple(zone)
        self.alt = self.plan["sweep_alt_m"]
        self.decode_alt = self.plan["decode_alt_m"]

        # Phase 7: route around red ground the camera georeferenced. The
        # unclipped plan is kept in self.plan for the coverage figure; the
        # FLOWN waypoints are the clipped ones.
        self.exclusions = list(exclusions)
        if self.exclusions:
            wps = plan_lawnmower_excluding(
                zone, self.plan["lane_spacing_m"], self.alt, a['swath_fov'],
                exclusions=self.exclusions, clearance_m=self.clearance_m,
                axis=self.plan["lane_axis"])
            self.plan["coverage"] = coverage_fraction_excluding(
                zone, self.plan["lane_spacing_m"], self.alt, a['swath_fov'],
                exclusions=self.exclusions, clearance_m=self.clearance_m,
                axis=self.plan["lane_axis"])
            self.plan["n_lanes"] = len(wps) // 2
        else:
            wps = self.plan["waypoints"]
        self.wps = [(w[0], w[1]) for w in wps]

    def _lane_coord(self, wp):
        """Which lane a waypoint belongs to: y for x-axis lanes, x for y."""
        return wp[1] if self.plan and self.plan["lane_axis"] == "x" else wp[0]

    def _resume_index(self, lane, pos=None):
        """Next unflown waypoint of the new plan, from the lane `lane` on.

        A re-plan used to restart the pattern at waypoint 0. The exclusion
        count climbs every time a red cell is confirmed, so the first lane was
        flown five and six times over (583 of 1908 search samples in live run
        2) while the far lanes -- where the target was -- were starved.
        Lanes run in increasing order from the bottom-left corner, so the
        remaining work is every lane from the current one on.

        On the current lane it is the next segment point AHEAD of the
        aircraft (`pos`), not the lane's first point: resuming at the lane
        start sent the aircraft back along a lane it had half flown, which
        is why re-plans had to be rationed -- and a rationed plan went stale
        (arena 1001, batch E: after the fifth re-plan, two lanes whose far
        ends had become red were skipped whole, and the target was on them).
        Waypoints come in (start, end) pairs, one per lane segment.
        """
        if lane is None:
            return 0
        tol = 0.5 * float(self.plan["lane_spacing_m"])
        along = 0 if self.plan["lane_axis"] == "x" else 1
        for k in range(0, len(self.wps) - 1, 2):
            a, b = self.wps[k], self.wps[k + 1]
            c = self._lane_coord(a)
            if c < lane - tol:
                continue
            if pos is None or abs(c - lane) > tol:
                return k                    # a later lane: from its start
            s = 1.0 if b[along] >= a[along] else -1.0
            p = pos[along]
            if (p - b[along]) * s >= -0.5:
                continue                    # this segment is already flown
            on_lane = abs(pos[1 - along] - c) <= tol
            if on_lane and (p - a[along]) * s > 0.5:
                return k + 1                # part-way along it: carry on
            return k
        return len(self.wps)

    def _yaw(self, target=None):
        """Heading to fly at: crabbed 90 deg to the direction of travel.

        Perpendicular to the leg actually being flown -- lanes, the hop
        between lanes and the transit to the first corner alike -- so the
        image's long axis always looks where the airframe is going. Of the
        two perpendiculars, the one nearer the current heading, so a
        boustrophedon does not spin the aircraft 180 degrees at every turn.
        """
        if not self.crab or not self.plan:
            return 0.0
        lane_yaw = math.pi / 2.0 if self.plan["lane_axis"] == "x" else 0.0
        if target is None:
            return lane_yaw
        x, y = self.mav.pos()[:2]
        dx, dy = target[0] - x, target[1] - y
        if math.hypot(dx, dy) < 0.5:
            return getattr(self, "_last_yaw", lane_yaw)
        d = math.atan2(dy, dx)
        now = self.mav.yaw() if hasattr(self.mav, "yaw") else lane_yaw
        options = (d + math.pi / 2.0, d - math.pi / 2.0)
        best = min(options, key=lambda a: abs(math.atan2(math.sin(a - now),
                                                         math.cos(a - now))))
        self._last_yaw = math.atan2(math.sin(best), math.cos(best))
        return self._last_yaw

    def _current_exclusions(self):
        src = self._exclusions_src
        if src is None:
            return []
        return list(src() if callable(src) else src)

    def initialise(self):
        self._fix_wait = 0
        self._fix_hold = None
        self.i = 0
        self.skipped = 0
        self.router.reset()
        self.hover.reset()
        self._budget_left = self.search_budget_m
        self.expansions = 0
        self._replans = 0
        self._ticks_since_replan = self.replan_min_ticks
        self._speed_sent = False
        self.mav.target_xy = None           # no pad fixed yet this mission
        self._pass = 0
        self._axis_override = None
        # The frontier advances the way the aircraft came out of the corridor.
        exit_pose = self.mav.corridor_exit_pose
        self._heading = exit_pose[3] if exit_pose else self.mav.yaw()
        if callable(self._zone_src):
            zone = self._zone_src()
            if zone is None:
                return                      # update() reports the failure
            self._replan(zone, self._current_exclusions())
        # ALWAYS begin at the bottom-left (south-west, min x / min y) corner of
        # the boundary. plan_lawnmower's first lane is the one nearest y0 (x
        # axis) or x0 (y axis), run from the low end, so the pattern is fixed
        # by the supplied coordinates alone and does not depend on where the
        # corridor happened to let the aircraft out.
        if self.wps:
            self.mav.log(f"lawnmower starts at the bottom-left corner: first "
                         f"waypoint ({self.wps[0][0]:.1f}, {self.wps[0][1]:.1f})")
        # How deep the observed window is ALONG THE HEADING. This is the step
        # the frontier advances by; see _advance_frontier() for why it is not
        # re-measured from sweep altitude.
        zx0, zx1, zy0, zy1 = self._zone()
        self._window_depth = abs((zx1 - zx0) * math.cos(self._heading)) + \
            abs((zy1 - zy0) * math.sin(self._heading))
        z = [round(v, 1) for v in self._zone()]
        self.mav.log(
            f"search plan over observed zone {z} "
            f"avoiding {len(self.exclusions)} red zone(s): "
            f"sweep {self.alt:.1f} m, decode {self.decode_alt:.1f} m, "
            f"spacing {self.plan['lane_spacing_m']:.1f} m, "
            f"{self.plan['n_lanes']} {self.plan['lane_axis']}-axis lanes, coverage "
            f"{self.plan['coverage'] * 100:.0f}%, "
            f"frontier budget {self._budget_left:.0f} m")

    def _zone(self):
        return self._zone_src() if callable(self._zone_src) else self._zone_src

    def _advance_frontier(self):
        """Sweep the next strip of ground beyond the window just finished.

        WHY THE SEARCH CANNOT STOP AT THE OBSERVED WINDOW

            ObserveZone bounds the zone with the lidar, and the lidar's range
            is not the zone's size. In the reference arena it reports 12 m of
            open ground for a delivery zone 40 m deep, so the sweep covered
            x 16.5..27.9 of a real 12..52 and reported "swept all lanes without
            matching the target".

            That was never noticed because every live run had been started with
            target C, the one pad inside the window. B, D and E -- and all
            three red zones -- are beyond it.

        The window is a frontier: when it is exhausted, the aircraft sweeps the
        next strip of the same depth, until the search budget runs out.

        WHY THIS DOES NOT ASK THE LIDAR HOW FAR TO GO

            It did, at first, and live run 13 never advanced once -- it failed
            with "no open ground left ahead and 45 m of search budget unused"
            while the target sat at x=45.

            The horizontal lidar is mounted on the airframe. At the corridor
            mouth, 3 m up between two walls, its forward reading is a real
            measurement of the opening. At sweep altitude, 10 m up over open
            ground, it is above everything and its forward reading is noise --
            /avoidance/detail reported open_depth_m = 0.32 with nothing
            whatsoever in front of the aircraft. Treating that as "the
            boundary is here" stopped the search dead.

            So the step is the depth of the window the lidar measured WHERE IT
            COULD SEE, repeated outward. That is still an observation, not an
            assumption about the arena: a different arena with a different
            opening gives a different step.

        Exclusions are re-read here, so red zones that only became visible
        during the sweep are avoided by the strips that follow.
        """
        if self._budget_left < self.min_step_m:
            return False
        step = min(self._window_depth, self._budget_left)
        if step < self.min_step_m:
            return False
        # FORWARD, THEN SIDEWAYS.
        #
        # This only ever advanced along the corridor heading. Seed 1002 swept
        # four strips out to x = 74.7 and never found pad E, because the pad
        # was not further down the corridor -- it was off to one side. A
        # search that can only go forward cannot cover a zone wider than the
        # opening it was measured from.
        #
        # Growth is taken from the UNION already swept, not from the last
        # strip, so every new band is adjacent to covered ground: no gaps
        # between a forward strip and a lateral one, and nothing re-flown.
        base = self.swept_zone or self._frontier
        for _ in range(len(self.EXPANSION_DIRECTIONS)):
            offset = self.EXPANSION_DIRECTIONS[
                self.expansions % len(self.EXPANSION_DIRECTIONS)]
            direction = self._heading + offset
            grown, band = grow_zone(base, direction, step)
            ok, _why = zone_is_plausible(band)
            if ok:
                break
            # A band that is implausible on its own (too thin a sliver at the
            # zone edge) should not stop the search -- try the next direction.
            self.expansions += 1
        else:
            return False

        self._budget_left -= step
        self.expansions += 1
        self._replan(band, self._current_exclusions(), extend_swept=True)
        self.i = 0
        self.router.reset()
        self._replans = 0
        name = {0.0: "forward", round(math.pi / 2, 3): "left",
                round(-math.pi / 2, 3): "right"}.get(round(offset, 3), "?")
        b = [round(v, 1) for v in band]
        self.mav.log(
            f"frontier advance {self.expansions}: stepping {step:.1f} m "
            f"{name} (the observed opening) -> sweeping {b} "
            f"avoiding {len(self.exclusions)} red zone(s), "
            f"{self.plan['n_lanes']} lanes, "
            f"{self._budget_left:.0f} m budget left")
        return True

    def _next_pass(self):
        """Cross-grid re-sweep of the same boundary, bottom-left first."""
        if self._pass + 1 >= self.max_passes or self.plan is None:
            return False
        zone = self._zone()
        if zone is None:
            return False
        self._pass += 1
        self._axis_override = "y" if self.plan["lane_axis"] == "x" else "x"
        self._replan(zone, self._current_exclusions())
        self.i = 0
        self.router.reset()
        self._replans = 0
        self.mav.log(
            f"search pass {self._pass + 1}: no match on pass {self._pass}; "
            f"cross-grid {self.plan['lane_axis']}-axis sweep from the "
            f"bottom-left corner, {self.plan['n_lanes']} lanes")
        return bool(self.wps)

    def _exclusions_changed(self):
        """A red zone confirmed mid-strip must be avoided by THIS strip.

        Exclusions used to be sampled once, before the sweep started, from the
        corridor exit -- where none of the red zones are visible. Anything the
        georeferencer confirmed while sweeping was recorded and then ignored.
        """
        # The georeferencer confirms red ground cell by cell, so the exclusion
        # count climbs steadily while the aircraft sweeps (0 -> 5 -> 26 -> 32
        # in live run 13). A re-plan resumes ahead of the aircraft and the
        # same tick flies on, so a growing set cannot stall the sweep; the
        # spacing only keeps the router from being reset every tick. It was
        # a hard cap of 5, and the plan it froze sent the aircraft at lane
        # ends that had since turned red, so whole lanes were skipped.
        if self._ticks_since_replan < self.replan_min_ticks:
            return False
        now = self._current_exclusions()
        if len(now) == len(self.exclusions):
            return False
        remaining = self.wps[self.i:]
        if not remaining:
            return False
        return plan_intersects_exclusions(
            [(x, y, self.alt) for x, y in remaining],
            self.clearance_m, now)

    def _replan_mid_sweep(self):
        """Re-plan against the red ground confirmed so far; resume ahead."""
        n_before = len(self.exclusions)
        lane = (self._lane_coord(self.wps[self.i])
                if self.i < len(self.wps) else None)
        self._replan(self._frontier, self._current_exclusions(),
                     extend_swept=True)
        self.i = self._resume_index(lane, self.mav.pos()[:2])
        self.router.reset()
        self._replans += 1
        self._ticks_since_replan = 0
        self.mav.log(
            f"red zone confirmed mid-sweep ({n_before} -> "
            f"{len(self.exclusions)}): re-planned, resuming at waypoint "
            f"{self.i} of {len(self.wps)}, "
            f"{self.plan['n_lanes']} lanes, coverage "
            f"{self.plan['coverage'] * 100:.0f}%")

    def update(self):
        if self.plan is None:
            reason = "LawnmowerSearch: no observed zone to sweep"
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        if self.search_speed_mps and not self._speed_sent:
            fn = getattr(self.mav, "set_speed", None)
            if callable(fn) and fn(self.search_speed_mps) is not None:
                self._speed_sent = True
                self.mav.log(f"search ground speed capped at "
                             f"{self.search_speed_mps:.1f} m/s so red ground is "
                             f"confirmed before the airframe reaches it")
        note_target(self.mav, hfov_rad=self._plan_args['hfov_rad'],
                    image_w_px=self._plan_args['image_width_px'],
                    image_h_px=self._image_h)
        if self.mav.qr_matched:
            # A MATCH WITHOUT A FIX IS NOT DONE. With the C270 the pad is
            # often matched at the frame edge, and the match can be handled
            # before the offset that places it: the sweep then ended with no
            # idea where the pad was, CenterOnTarget held the spot it stopped
            # at -- the pad just out of frame -- and timed out. Stop here and
            # give the offset a moment (2 s) to arrive.
            if getattr(self.mav, "target_xy", None) is None                     and self._fix_wait < self.fix_wait_ticks:
                self._fix_wait += 1
                x, y, z = self.mav.pos()
                if self._fix_hold is None:
                    self._fix_hold = (x, y, z, self.mav.yaw())
                self.mav.goto(*self._fix_hold)
                self.feedback_message = "matched; waiting for the pad's position"
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.SUCCESS
        self._fix_wait = 0
        self._fix_hold = None
        if self.hover.tick(self.mav):
            self.feedback_message = (f"holding over '{self.hover.payload}' "
                                     f"before resuming the sweep")
            return py_trees.common.Status.RUNNING
        self._ticks_since_replan += 1
        if self._exclusions_changed():
            self._replan_mid_sweep()
            # Fly on this tick: a re-plan is not a reason to stop.
        if self.i >= len(self.wps):
            if self._advance_frontier():
                return py_trees.common.Status.RUNNING
            if self._next_pass():
                return py_trees.common.Status.RUNNING
            swept = [round(v, 1) for v in (self.swept_zone or self._zone())]
            reason = (f"swept {swept} at {self.alt:.1f} m in "
                      f"{self._pass + 1} pass(es), {self.expansions + 1} "
                      f"strip(s), {self.skipped} waypoint(s) skipped for red "
                      f"zones, without matching the target")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE     # swept all, no match
        wx, wy = self.wps[self.i]
        status = self.router.fly(self.mav, wx, wy, self.alt,
                                 self._yaw((wx, wy)))
        if (status is BLOCKED
                and len(self._current_exclusions()) != len(self.exclusions)):
            # Red ground confirmed since the last plan, inside the re-plan
            # spacing (arena 1003, batch E). Re-plan now so the lane is
            # clipped at the new edge instead of losing the waypoint. The
            # plan then holds the current set, so this cannot repeat until
            # more red is confirmed.
            self._replan_mid_sweep()
            return py_trees.common.Status.RUNNING
        if status is BLOCKED:
            # A SKIPPED waypoint, not an aborted mission. Losing a lane costs
            # coverage; flying it costs 5 marks. Exhausting the re-plan budget
            # is not permission to fly the leg the re-plan would have removed.
            self.mav.log(f"skipping waypoint {self.i} of {len(self.wps)}: "
                         f"{self.router.blocked_reason}", warn=True)
            self.skipped += 1
            self.i += 1
            self.router.reset()
            return py_trees.common.Status.RUNNING
        if status is ARRIVED:
            self.i += 1
            self.router.reset()
        return py_trees.common.Status.RUNNING


class DescendToDecode(py_trees.behaviour.Behaviour):
    """Drop to the altitude where the payload QR can actually be READ.

    WHAT WAS MISSING (Phase 6)
        search_planner computed `decode_alt_m` and the mission ignored it: the
        sweep flew at the sweep altitude and expected to decode there. That
        only worked because the simulated pad is 2.2 m across. For a realistic
        marker the sweep altitude is far above the decode envelope measured in
        Phase 1, so the aircraft would have flown the whole pattern seeing pads
        it could never read.

    Sweep high to FIND, descend to READ. This is the descend half.

    Succeeds as soon as the QR decodes — no point continuing down — and stops
    at `floor_alt` so a bad decode altitude cannot fly the aircraft into the
    ground. If nothing decodes by the floor it fails closed, because a pad the
    aircraft cannot read is not a delivery target.
    """

    def __init__(self, mav, decode_alt=3.0, floor_alt=2.0, step=0.5,
                 timeout_ticks=300, tol=0.4, target_alt=None,
                 clearance_m=DEFAULT_CLEARANCE_M, exclusions=None):
        super().__init__("DescendToDecode")
        self.mav = mav
        self.decode_alt = target_alt if target_alt is not None else decode_alt
        self.floor_alt = floor_alt
        self.step = step
        self.timeout_ticks = timeout_ticks
        self.tol = tol
        self.clearance_m = float(clearance_m)
        self._exclusions_src = exclusions
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)
        self._t = 0
        self._target_z = None

    def _current_exclusions(self):
        src = self._exclusions_src
        if src is None:
            return list(getattr(self.mav, "exclusions", None) or [])
        return list(src() if callable(src) else src)

    def initialise(self):
        self._t = 0
        self._target_z = None
        self.router.reset()

    def update(self):
        self._t += 1
        x, y, z = self.mav.pos()

        if self.mav.qr_matched:
            self.feedback_message = f"decoded at {z:.1f} m"
            return py_trees.common.Status.SUCCESS

        if self._target_z is None:
            want = self.decode_alt() if callable(self.decode_alt) else self.decode_alt
            # DESCEND to decode -- never climb. For a large marker the decode
            # envelope reaches ABOVE the sweep altitude, and `max(want, floor)`
            # then commanded a climb: with the 2.2 m simulated pad the decode
            # altitude computes to 13.9 m, which would have taken the aircraft
            # above the rulebook's 10 m identification altitude in order to
            # "descend" to read a marker it could already read.
            self._target_z = min(z, max(float(want), self.floor_alt))

        # Hold station over the candidate while descending: drifting off the
        # pad on the way down loses the very thing being read.
        cx, cy = self.mav.qr_hold_xy(default=(x, y))
        # Descending onto a candidate is a commitment to sit above it. If the
        # georeferencer has confirmed red ground there, the aircraft would
        # spend the whole descent inside a restricted zone -- and unlike a
        # transit there is nowhere to route to. Stop, and say which.
        held = self._current_exclusions()
        if point_in_exclusion(cx, cy, self.clearance_m, held):
            reason = (f"DescendToDecode: the candidate at ({cx:.1f}, {cy:.1f}) "
                      f"stands on confirmed red ground ({len(held)} zone(s)); "
                      f"refusing to hold station inside a restricted zone")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        status = self.router.fly(self.mav, cx, cy, self._target_z,
                                 self.mav.yaw())
        if status is BLOCKED:
            return _blocked(self, self.router)

        if self._t > self.timeout_ticks:
            reason = (f"DescendToDecode: no decode by {z:.1f} m after "
                      f"{self._t} ticks")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        self.feedback_message = (f"descending {z:.1f} -> {self._target_z:.1f} m "
                                 f"for decode")
        return py_trees.common.Status.RUNNING


class ReturnToCorridorMouth(py_trees.behaviour.Behaviour):
    """Fly back to the point the corridor actually opened out at.

    WHAT THIS REPLACES (geometry audit A9)
        corridor_return_entry = (15.0, 0.0, 3.0, pi)

    An asserted arena coordinate, plus an asserted heading of exactly pi. The
    aircraft flew OUT of the corridor earlier in this same mission and recorded
    where and at what heading; the way back in is that point, reversed. No
    arena knowledge required, and it is correct for a corridor at any angle.

    Falls back to the takeoff origin only if no exit was ever recorded, which
    means the corridor stage never ran.
    """

    def __init__(self, mav, alt=3.0, tol=0.8, standoff_m=None,
                 camera_pitch_rad=math.radians(20.0), banner_centre_m=3.38,
                 clearance_m=DEFAULT_CLEARANCE_M, ident_alt=None,
                 lane_offset_m=-4.0):
        super().__init__("ReturnToCorridorMouth")
        self.mav = mav
        self.alt = alt
        self.lane_offset_m = float(lane_offset_m)   # see return_mouth()
        # The standoff is for where the banner will be LOOKED AT from, not
        # where this leg flies. The leg transits at 10 m but the return banner
        # is identified from 5 m; sizing the standoff for 10 m put the aircraft
        # 18 m out, where the lettering does not resolve and the far-gate
        # check (correctly) refused it.
        self.ident_alt = alt if ident_alt is None else ident_alt
        self.tol = tol
        self.standoff_m = standoff_m
        self.camera_pitch = float(camera_pitch_rad)
        self.banner_centre_m = float(banner_centre_m)
        # The whole point of this leg is to cross the delivery zone, which is
        # where the red zones are and where they have by now been confirmed.
        # Of every leg in the mission this is the one most likely to be routed.
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)
        self._standoff_key = None
        self._standoff_pt = None

    def initialise(self):
        self.router.reset()
        self._standoff_key = None
        self._standoff_pt = None

    def return_mouth(self, exit_pose):
        """(x, y, facing yaw) of the RETURN lane's mouth, from the outbound exit.

        The recorded exit is where the OUTBOUND lane opened out, but the
        return banner hangs over the other lane: in the corridor model the
        lanes are centred 4 m apart (y +2 outbound, -2 return; banners at
        (2, +2) and (12, -2)). Standing off the outbound exit put the
        aircraft 35 deg off the return board -- the lidar read "-31 deg at
        6.6 m" in arena 1004 -- and the square-up then had to travel round
        the board to reach its lane, which is where the red zone's corner
        lay (batch F). Laid out in front of the return lane, the square-up
        starts square.

        The axis is the heading squared on the outbound gate when it was
        recorded (lidar-measured to 5 deg), else the exit yaw; the corridor
        is one rigid structure, so both gates share it. `lane_offset_m` is
        port-positive facing OUT of the corridor, so the return lane at
        starboard is negative.
        """
        mx, my, _, exit_yaw = exit_pose
        axis = getattr(self.mav, "gate_heading", None)
        if axis is None:
            axis = exit_yaw
        off = self.lane_offset_m
        rx = mx - off * math.sin(axis)
        ry = my + off * math.cos(axis)
        tyaw = math.atan2(math.sin(axis + math.pi), math.cos(axis + math.pi))
        return rx, ry, tyaw

    def standoff(self):
        """How far SHORT of the mouth to stop, so the banner is in frame.

        WHY STOPPING AT THE MOUTH DOES NOT WORK

            The recorded exit pose is where the corridor opened out -- which
            is where the return banner hangs. Flying exactly there puts the
            aircraft on top of the banner: from 5 m with the camera at -20 deg
            the board is almost straight down and out of frame entirely.

            Seed 1001 failed here on every run, sweeping a full 180 deg while
            the detector reported "only 1 white component" -- it was looking
            past a banner that was below it.

            The outbound leg identifies the same banner without trouble
            because it starts about 6 m away.

        Derived, not chosen: the range at which a board whose centre sits
        `banner_centre_m` above the ground appears at the camera's depression
        angle from the current altitude.

            standoff = (altitude - banner_centre) / tan(camera_pitch)
        """
        if self.standoff_m is not None:
            return float(self.standoff_m)
        drop = max(0.5, float(self.ident_alt) - self.banner_centre_m)
        return drop / math.tan(self.camera_pitch)

    def clear_standoff(self, mx, my, tyaw, back):
        """The stand-off point, moved off red ground if it lies on some.

        Arena 1004 (batch E) put the main red zone's corner 2 m from the
        nominal point, inside the router's inflated box, and the mission
        failed "destination lies inside 1 confirmed red zone" with the
        corridor in plain sight. Nothing in the rulebook keeps red ground
        away from the corridor mouth, so the point has to give way: the
        nearest one along the approach and up to 2 m to either side that is
        clear, within the stand-off band AlignToBanner accepts (2.5-6.0 m,
        aimed 0.5 m inside it) -- it squares up on the board from wherever
        it starts. None clear: the nominal point, and the router refuses it.
        """
        ex = list(getattr(self.mav, "exclusions", None) or [])
        key = (round(mx, 2), round(my, 2), round(tyaw, 3), round(back, 2),
               len(ex))
        if key != self._standoff_key:
            self._standoff_key = key
            self._standoff_pt = self._pick_standoff(mx, my, tyaw, back, ex)
        return self._standoff_pt

    def _pick_standoff(self, mx, my, tyaw, back, ex):
        cx, cy = math.cos(tyaw), math.sin(tyaw)

        def at(b, side):
            # Back from the mouth along the approach; `side` across it.
            return (mx - b * cx - side * cy, my - b * cy + side * cx)

        nominal = at(back, 0.0)
        blocks = routing_obstacles(ex, self.router.clearance_m)
        if not blocks:
            return nominal

        def clear(p):
            return not any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1
                           for x0, x1, y0, y1 in blocks)

        if clear(nominal):
            return nominal
        backs = [3.0 + 0.25 * k for k in range(9)]           # 3.0 .. 5.0 m
        sides = [0.25 * k for k in range(-8, 9)]              # -2 .. +2 m
        cands = sorted(((b, s) for b in backs for s in sides),
                       key=lambda bs: math.hypot(bs[0] - back, bs[1]))
        for b, s in cands:
            p = at(b, s)
            if clear(p):
                if self._standoff_pt is None or math.dist(p, self._standoff_pt) > 0.1:
                    self.mav.log(f"return stand-off ({nominal[0]:.1f}, "
                                 f"{nominal[1]:.1f}) is on red ground; using "
                                 f"({p[0]:.1f}, {p[1]:.1f}), {b:.1f} m out "
                                 f"and {s:+.1f} m across the approach")
                return p
        return nominal

    def update(self):
        pose = self.mav.corridor_exit_pose
        if pose is None:
            hx, hy = self.mav.home_local_xy()
            tx, ty, tyaw = hx, hy, self.mav.yaw()
            self.feedback_message = "no recorded corridor exit; heading home"
        else:
            mx, my, tyaw = self.return_mouth(pose)
            # Stop SHORT of the mouth, on the delivery-zone side, so the
            # banner is in front of the camera rather than beneath it.
            back = self.standoff()
            tx, ty = self.clear_standoff(mx, my, tyaw, back)
            self.feedback_message = (f"returning to {back:.1f} m short of the "
                                     f"return-lane mouth ({tx:.1f}, {ty:.1f}) "
                                     f"yaw {tyaw:.2f}")
        status = self.router.fly(self.mav, tx, ty, self.alt,
                                 crab_yaw(self.mav, (tx, ty), tyaw))
        if status is BLOCKED:
            return _blocked(self, self.router)
        return (py_trees.common.Status.SUCCESS if status is ARRIVED
                else py_trees.common.Status.RUNNING)


class FindReturnBanner(py_trees.behaviour.Behaviour):
    """Find the RETURN corridor's banner with the camera, wherever it is.

    WHY
        The return stand-off is laid out from the outbound exit on the
        rulebook's arrangement (return lane beside the outbound one). A
        corridor placed anywhere else puts the aircraft in front of nothing,
        and AlignToBanner fails having swept the empty field. Nothing may say
        where the return corridor is -- no constant, no input -- so the
        camera has to find it.

    HOW
        1. Look HERE first: a full turn at the stand-off. For the rulebook
           layout the banner is straight ahead and this ends on the first
           heading, costing nothing.
        2. Then vantage points along the delivery zone's edge (the corridors
           lead INTO the zone), nearest the outbound exit first, alternating
           either way round, each looking OUTWARD across the boundary.
        3. A sighting is the identified banner (lettering read) on
           `hits_needed` samples of one dwell, not the OUTBOUND banner (its
           position was recorded on the way out), within the near range. Its
           position is estimated from bearing and board size, and the
           aircraft moves to a stand-off in front of it and faces it;
           AlignToBanner squares up from there.
        4. A board seen only EDGE-ON has no lettering to read, but its green
           is still where the gate is. As soon as a sweep has seen green that
           is not the outbound gate or corridor, the next vantage points are
           an orbit round it (banner_orbit.py), before the perimeter walk.
    """

    def __init__(self, mav, alt=5.0, clock=None, hfov_rad=1.0472,
                 image_width_px=1280, banner_w_m=3.7, banner_h_m=1.15,
                 near_range_m=9.6, standoff_m=5.0,
                 step_rad=math.radians(30.0), dwell_s=1.5, hits_needed=2,
                 spacing_m=8.0, inset_m=3.0, max_vantages=16,
                 exclude_radius_m=4.0, clearance_m=DEFAULT_CLEARANCE_M,
                 yaw_tol_rad=math.radians(10.0), turn_timeout_s=8.0,
                 orbit_step_rad=math.radians(45.0), orbit_vantages=7,
                 orbit_min_radius_m=5.0, fence_margin_m=2.0,
                 orbit_alt_m=3.0, zone_side_m=3.0):
        super().__init__("FindReturnBanner")
        self.mav = mav
        self.alt = float(alt)
        self.clock = clock or time.monotonic
        self.hfov = float(hfov_rad)
        self.focal_px = 0.5 * float(image_width_px) / math.tan(self.hfov / 2.0)
        self.banner_area_m2 = float(banner_w_m) * float(banner_h_m)
        self.near_range_m = float(near_range_m)
        self.standoff_m = float(standoff_m)
        self.step = float(step_rad)
        self.dwell_s = float(dwell_s)
        self.hits_needed = int(hits_needed)
        self.spacing_m = float(spacing_m)
        self.inset_m = float(inset_m)
        self.max_vantages = int(max_vantages)
        self.exclude_radius_m = float(exclude_radius_m)
        self.yaw_tol = float(yaw_tol_rad)
        self.turn_timeout_s = float(turn_timeout_s)
        self.router = LegRouter(clearance_m=clearance_m, tol=0.8)
        self.orbit_step_rad = float(orbit_step_rad)
        self.orbit_vantages = int(orbit_vantages)
        self.orbit_radius_lo = float(orbit_min_radius_m)
        self.fence_margin_m = float(fence_margin_m)
        self.orbit_alt_m = float(orbit_alt_m)
        # THE RETURN GATE IS ENTERED FROM THE ZONE. Its board faces the
        # delivery zone, so a view of it from farther out than this beyond the
        # zone's edge is its back -- a live run read it from inside the return
        # lane, stood off there and squared up facing the wrong way. Orbit
        # points, and the stand-off a sighting leads to, stay on the zone side.
        self.zone_side_m = float(zone_side_m)
        self.phase = None

    # ---- plan ---------------------------------------------------------------
    def initialise(self):
        self.router.reset()
        self.phase = "sweep"
        x, y = self.mav.pos()[:2]
        self.here = (x, y)
        now = self.mav.yaw()
        # Here: a full turn starting straight ahead.
        n = max(1, int(round(2 * math.pi / self.step)))
        order = [0] + [k * s for k in range(1, n // 2 + 1) for s in (1, -1)]
        self.headings = [self._wrap(now + k * self.step) for k in order][:n]
        self.hi = 0
        self.vantages = None
        self.vi = -1
        self.target = None
        self.seen_from = []
        self._green = None           # best green_fix, not the outbound gate
        self._orbit = None           # planned orbit vantages
        self._legs = []              # arc waypoints still to fly this transit
        self._guard_leg = False
        self._cur_alt = self.alt
        self._behind = 0             # sightings refused as the board's back
        self._begin_heading()

    def _begin_heading(self):
        self._turning = True
        self._since = None           # clock when the turn / dwell began
        self._hits = 0

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _perimeter_vantages(self):
        """Points along the zone boundary (inset), nearest the exit first,
        alternating either way round, each with its outward heading."""
        get = getattr(self.mav, "delivery_search_zone", None)
        zone = get(self.inset_m) if callable(get) else None
        if not zone:
            return []
        x0, x1, y0, y1 = zone
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        outward = [-math.pi / 2, 0.0, math.pi / 2, math.pi]   # S, E, N, W edges
        edges = []                                             # (a, b, out)
        for i in range(4):
            edges.append((corners[i], corners[(i + 1) % 4], outward[i]))
        per = sum(math.dist(a, b) for a, b, _ in edges)
        if per <= 0:
            return []

        def at(s):
            s %= per
            for a, b, out in edges:
                L = math.dist(a, b)
                if s <= L:
                    f = s / L if L else 0.0
                    return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), out)
                s -= L
            a, b, out = edges[-1]
            return (b[0], b[1], out)

        # Arc position nearest where the outbound corridor opened out.
        exit_pose = getattr(self.mav, "corridor_exit_pose", None)
        ex, ey = (exit_pose[0], exit_pose[1]) if exit_pose else self.here
        steps = max(8, int(per / 0.5))
        s0 = min((k * per / steps for k in range(steps)),
                 key=lambda s: math.dist(at(s)[:2], (ex, ey)))
        out = [at(s0)]
        k = 1
        while len(out) < self.max_vantages and k * self.spacing_m < per / 2 + self.spacing_m:
            for sign in (1, -1):
                out.append(at(s0 + sign * k * self.spacing_m))
            k += 1
        return out[:self.max_vantages]

    def _zone_side(self, x, y):
        """On the delivery zone's side of the return gate (see __init__)."""
        get = getattr(self.mav, "delivery_search_zone", None)
        zone = get(-self.zone_side_m) if callable(get) else None
        if not zone:
            return True
        x0, x1, y0, y1 = zone
        return x0 <= x <= x1 and y0 <= y <= y1

    def _next_orbit_vantage(self):
        """Aim the transit at the next orbit vantage; False if none left."""
        if self._orbit is None:
            g = self._green
            radius = max(self.orbit_radius_lo,
                         min(self.near_range_m - 1.0, g["range"]))
            fence = fence_ok(self.mav, self.fence_margin_m,
                             on_red=lambda x, y: point_in_exclusion(
                                 x, y, self.router.clearance_m,
                                 list(getattr(self.mav, "exclusions", None) or [])))
            ok = lambda x, y: fence(x, y) and self._zone_side(x, y)  # noqa: E731
            # Corridor altitude, camera level: see AlignToBanner's orbit.
            self._cur_alt = self.orbit_alt_m
            fn = getattr(self.mav, "set_camera_pose", None)
            if callable(fn):
                fn("FORWARD")
            self._orbit = orbit_plan((g["x"], g["y"]), self.mav.pos()[:2],
                                     radius, ok, step_rad=self.orbit_step_rad,
                                     n=self.orbit_vantages)
            self.mav.log(
                f"FindReturnBanner: no banner READ, but green seen "
                f"~{g['range']:.1f} m off at ({g['x']:.1f}, {g['y']:.1f}) -- a "
                f"board seen edge-on has no lettering to read. Orbiting it at "
                f"{radius:.1f} m: {len(self._orbit)} vantage point(s)")
        if not self._orbit:
            return False
        v = self._orbit.pop(0)
        self.target = (v["at"][0], v["at"][1], v["face"])
        self._legs = list(v["path"][:-1])
        self._guard_leg = True
        self.headings = [v["face"], self._wrap(v["face"] - self.step),
                         self._wrap(v["face"] + self.step)]
        self.hi = 0
        self.phase = "transit"
        self.router.reset()
        self._begin_heading()
        return True

    def _next_vantage(self):
        if self._green is not None and (self._orbit is None or self._orbit):
            if self._next_orbit_vantage():
                return True
        self._legs, self._guard_leg = [], False
        if self.vantages is None:
            self.vantages = self._perimeter_vantages()
        self.vi += 1
        if self.vi >= len(self.vantages):
            return False
        vx, vy, out = self.vantages[self.vi]
        self.target = (vx, vy, out)
        # Outward half only: the return entrance is at or beyond the edge.
        fan = [0, 1, -1, 2, -2, 3, -3]
        self.headings = [self._wrap(out + k * self.step) for k in fan]
        self.hi = 0
        self.phase = "transit"
        self.router.reset()
        self._begin_heading()
        return True

    # ---- sighting -----------------------------------------------------------
    def _sighting(self):
        """World (x, y) of an identified banner in view that is not the
        outbound one and is within the near range; else None."""
        if not self.mav.banner_identified():
            return None
        area = float(getattr(self.mav, "banner_board_area", 0.0) or 0.0)
        rng = (self.focal_px * math.sqrt(self.banner_area_m2 / area)
               if area > 0 else self.standoff_m)
        if rng > self.near_range_m:
            return None
        th = self.mav.yaw() + bearing_to_angle(self.mav.banner_bearing(), self.hfov)
        x, y = self.mav.pos()[:2]
        bx, by = x + rng * math.cos(th), y + rng * math.sin(th)
        ob = getattr(self.mav, "outbound_banner_xy", None)
        if ob is not None and math.dist((bx, by), ob) < self.exclude_radius_m:
            return None
        return bx, by, th, rng

    # ---- tick ---------------------------------------------------------------
    def update(self):
        if self.phase == "transit":
            vx, vy, out = self.target
            gx, gy = self._legs[0] if self._legs else (vx, vy)
            if self._guard_leg and not leg_clear(self.mav, gx, gy):
                self.mav.log(f"FindReturnBanner: the leg to ({gx:.1f}, "
                             f"{gy:.1f}) is blocked on the lidar; next vantage",
                             warn=True)
                if not self._next_vantage():
                    return self._give_up()
                return py_trees.common.Status.RUNNING
            if self._guard_leg:
                seen = self._sighting()
                if seen is not None:
                    done = self._found(*seen)
                    if done is not None:
                        self.mav.log("FindReturnBanner: lettering read on the "
                                     "way round the orbit")
                        return done
            st = self.router.fly(self.mav, gx, gy, self._cur_alt, self.headings[0])
            if st is ARRIVED and self._legs:
                self._legs.pop(0)                   # the next arc waypoint
                self.router.reset()
                return py_trees.common.Status.RUNNING
            if st is BLOCKED:
                self.mav.log(f"FindReturnBanner: vantage {self.vi + 1} "
                             f"({vx:.1f}, {vy:.1f}) unreachable "
                             f"({self.router.blocked_reason}); next", warn=True)
                if not self._next_vantage():
                    return self._give_up()
                return py_trees.common.Status.RUNNING
            if st is ARRIVED:
                self.here = (vx, vy)
                self.phase = "sweep"
                self._begin_heading()
            return py_trees.common.Status.RUNNING

        if self.phase == "sweep":
            if self.hi >= len(self.headings):
                self.seen_from.append(tuple(round(v, 1) for v in self.here))
                if not self._next_vantage():
                    return self._give_up()
                return py_trees.common.Status.RUNNING
            h = self.headings[self.hi]
            self.mav.goto(self.here[0], self.here[1], self._cur_alt, h)
            now = self.clock()
            if self._since is None:
                self._since = now
            if self._turning:
                # Dwell only once pointing where asked (or the turn stalled).
                err = abs(self._wrap(self.mav.yaw() - h))
                if err <= self.yaw_tol or now - self._since > self.turn_timeout_s:
                    self._turning, self._since = False, now
                return py_trees.common.Status.RUNNING
            s = self._sighting()
            if s is not None:
                self._hits += 1
                if self._hits >= self.hits_needed:
                    done = self._found(*s)
                    if done is not None:
                        return done
                    self._hits = 0
            elif not self.mav.banner_identified():
                f = green_fix(self.mav, self.hfov,
                              exclude=outbound_structure(self.mav),
                              exclude_m=self.exclude_radius_m)
                if f is not None and (self._green is None
                                      or f["area"] > self._green["area"]):
                    self._green = f
            if now - self._since >= self.dwell_s:
                self.hi += 1
                self._begin_heading()
            return py_trees.common.Status.RUNNING

        if self.phase == "approach":
            ax, ay, th = self.target
            st = self.router.fly(self.mav, ax, ay, self._cur_alt, th)
            if st is BLOCKED:
                # The stand-off is on red ground or cannot be routed: square
                # up from where the banner was seen instead.
                self.mav.log(f"FindReturnBanner: stand-off unreachable "
                             f"({self.router.blocked_reason}); aligning from "
                             f"here", warn=True)
                return py_trees.common.Status.SUCCESS
            if st is ARRIVED:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING

    def _found(self, bx, by, th, rng):
        """Stand off in front of the sighting -- or None if it is the back."""
        x, y = self.mav.pos()[:2]
        if rng <= self.standoff_m + 1.0:
            ax, ay = x, y
        else:
            ax = bx - self.standoff_m * math.cos(th)
            ay = by - self.standoff_m * math.sin(th)
        if not self._zone_side(ax, ay):
            self._behind += 1
            if self._behind in (1, 10, 100):
                self.mav.log(
                    f"FindReturnBanner: banner read at ({bx:.1f}, {by:.1f}) "
                    f"from ({x:.1f}, {y:.1f}), outside the zone side -- the "
                    f"back of the return gate, not a stand-off it can be "
                    f"entered from; searching on", warn=True)
            return None
        self.target = (ax, ay, th)
        self.phase = "approach"
        self.router.reset()
        where = ("from an orbit vantage" if self._guard_leg else
                 "here" if self.vi < 0 else
                 f"from vantage {self.vi + 1} of {len(self.vantages or [])}")
        self.mav.log(f"FindReturnBanner: return banner identified {where}, "
                     f"~{rng:.1f} m away at ({bx:.1f}, {by:.1f}); standing off "
                     f"at ({ax:.1f}, {ay:.1f}) facing {math.degrees(th):+.0f} deg")
        return py_trees.common.Status.RUNNING

    def _give_up(self):
        reason = (f"FindReturnBanner: no return banner identified from "
                  f"{len(self.seen_from)} vantage point(s) "
                  f"{self.seen_from[:6]}{' ...' if len(self.seen_from) > 6 else ''}")
        self.feedback_message = reason
        self.mav.abort_reason = reason
        return py_trees.common.Status.FAILURE


def crab_yaw(mav, target, final_yaw, turn_in_m=4.0):
    """Heading for a transit over the delivery zone.

    Perpendicular to travel, so the nadir camera's long axis (5.8 m at 10 m)
    looks where the airframe is going; `final_yaw` only for the last
    `turn_in_m`. Nose-first, the look-ahead is 3.2 m: seed 1004's return
    transit grazed an unmapped corner of the main red zone at 0.31 m.
    """
    x, y = mav.pos()[:2]
    dx, dy = target[0] - x, target[1] - y
    if math.hypot(dx, dy) <= turn_in_m:
        return final_yaw
    d = math.atan2(dy, dx)
    now = mav.yaw()
    return min((d + math.pi / 2.0, d - math.pi / 2.0),
               key=lambda a: abs(math.atan2(math.sin(a - now),
                                            math.cos(a - now))))


class WinchDrop(py_trees.behaviour.Behaviour):
    """Descend to 5 m over the matched QR, lower + release, climb back."""
    def __init__(self, mav, drop_alt=5.0, cruise_alt=10.0,
                 lower_timeout=200, release_timeout=100,
                 hfov_rad=1.0472, image_w_px=1280, image_h_px=720,
                 marker_m=2.2, max_offset_age_ticks=40, centre_tol_m=0.25,
                 settle_ticks=10, servo_timeout_ticks=150,
                 payload_size_m=0.12, confirm_frames=5,
                 confirm_timeout_ticks=200, stow_timeout_ticks=300):
        super().__init__("WinchDrop"); self.mav = mav
        # CAMERA CONFIRMATION of the drop (phase 4). The winch's "released"
        # is only what it was told to do; the payload on the ground under the
        # pad is what the rulebook scores, so the nadir camera must see it
        # there -- at the pixel size a payload of this edge length has from
        # this altitude -- after the hook has wound back up. A payload still
        # on the hook would be reeled up out of frame with it.
        self.payload_size_m = float(payload_size_m)
        self.confirm_frames = int(confirm_frames)
        self.confirm_timeout = int(confirm_timeout_ticks)
        self.stow_timeout = int(stow_timeout_ticks)
        self._confirm = 0
        self.drop_alt = drop_alt; self.cruise_alt = cruise_alt
        self.lower_timeout = lower_timeout
        self.release_timeout = release_timeout
        self.hfov = float(hfov_rad)
        self.image_w_px = int(image_w_px)
        self.image_h_px = int(image_h_px)
        self.marker_m = float(marker_m)
        # How stale a fallback offset may be. A reading from thirty seconds
        # ago is a measurement of somewhere else.
        self.max_offset_age_ticks = int(max_offset_age_ticks)
        # Visual servo over the matched pad during the descent to drop_alt.
        self.centre_tol_m = float(centre_tol_m)
        self.servo_gain = 0.6
        self.servo_max_step_m = 0.6
        self.settle_ticks = int(settle_ticks)
        self.servo_timeout = int(servo_timeout_ticks)
        self._settle = 0
        self._yaw = 0.0
        self.drop_x = 0.0; self.drop_y = 0.0
        self.phase = 0; self._t = 0
        self._phase_started = 0
        self._last_offset = None            # (ox, oy, alt, tick)

    def tracking_floor(self):
        """The altitude below which the whole pad cannot fit in frame.

        Below it the delivery offset is not unlikely, it is geometrically
        impossible: there is no view of the pad to measure against. A drop
        altitude that makes the scored quantity unmeasurable is not a
        trade-off anyone chose, so it is raised rather than honoured.
        """
        return min_track_altitude(self.marker_m, self.hfov,
                                  self.image_w_px, self.image_h_px)

    def observe_offset(self):
        """Remember the most recent VALID sighting of the pad.

        Sampled all the way down, not only at the instant of release. The
        payload swings under the aircraft as the winch pays out, and it is in
        the nadir camera's view at exactly the moment the release fires --
        which is how a completed run came to report `delivery UNMEASURED`.
        """
        off = getattr(self.mav, "qr_offset", None)
        if off is None or getattr(off, "z", 0.0) <= 0.0:
            return
        alt = self.mav.alt()
        if alt is None:
            return
        self._last_offset = (float(off.x), float(off.y), float(alt), self._t)

    def initialise(self):
        self.phase = 0
        self._t = 0
        self._phase_started = 0
        self._last_offset = None
        self._confirm = 0
        self._last_payload_reason = "no payload detection received"
        self.mav.delivery_confirmed = None
        floor = self.tracking_floor()
        if self.drop_alt < floor:
            self.mav.log(
                f"drop altitude {self.drop_alt:.2f} m is below the "
                f"{floor:.2f} m tracking floor for a {self.marker_m:.1f} m "
                f"pad; raising it, because below the floor the delivery "
                f"offset cannot be measured at all", warn=True)
            self.drop_alt = floor
        # Start from the current position; _servo_drop_point() then walks it
        # onto the matched pad during the descent.
        self.drop_x, self.drop_y = self.mav.pos()[:2]
        self._settle = 0
        # Hold heading through the drop. goto()'s yaw defaults to 0, which
        # spun the aircraft to due east over the pad mid-delivery.
        self._yaw = self.mav.yaw()

    def _servo_drop_point(self):
        """Nudge the drop point toward the matched pad. True when centred."""
        off = getattr(self.mav, "qr_offset", None)
        alt = self.mav.alt()
        if off is None or getattr(off, "z", 0.0) < 0.99 or not alt or alt <= 0:
            return False
        half_w = alt * math.tan(self.hfov / 2.0)
        half_h = half_w * float(self.image_h_px) / float(self.image_w_px)
        # Same image->body mapping as CenterOnQR: forward = -y, right = +x.
        fwd = -float(off.y) * half_h
        right = float(off.x) * half_w
        err = math.hypot(fwd, right)
        step = min(self.servo_max_step_m, self.servo_gain * err)
        if err > 1e-6:
            psi = self.mav.yaw()
            ex = fwd * math.cos(psi) + right * math.sin(psi)
            ey = fwd * math.sin(psi) - right * math.cos(psi)
            x, y = self.mav.pos()[:2]
            self.drop_x = x + ex / err * step
            self.drop_y = y + ey / err * step
        return err <= self.centre_tol_m

    def _record_delivery_offset(self):
        """How far off the pad centre the payload was released, in metres.

        WHY THIS EXISTS

            "Payload Delivery Accuracy" is 15 rulebook marks -- "drop accuracy,
            altitude compliance, and stability during delivery" -- and nothing
            measured any of it. The mission reported

                landing PRECISE (committed at 3.86 m, 0 re-acquisition(s))

            which is a TRACKING-QUALITY claim wearing an accuracy-sounding
            name: it says how high the aircraft was when visual lock committed
            and how many times the pad was re-acquired. It says nothing about
            how far from the pad anything ended up.

        Derived from perception, not from the world. The QR detector reports
        the target's offset from frame centre, normalised; at a known altitude
        and field of view that converts to metres on the ground. Reading the
        pad's true pose out of the simulator would measure the simulator, and
        would be exactly the hardcoding the whole architecture removed.

        The complementary ground-truth check lives outside the mission, in
        sim/measure_delivery.py, where knowing the answer is legitimate.
        """
        off = getattr(self.mav, "qr_offset", None)
        alt = self.mav.alt()
        live = (off is not None and getattr(off, "z", 0.0) > 0.0
                and alt is not None)

        if live:
            ox, oy, at_alt, age = float(off.x), float(off.y), float(alt), 0
        elif self._last_offset is not None:
            ox, oy, at_alt, when = self._last_offset
            age = self._t - when
            if age > self.max_offset_age_ticks:
                self.mav.delivery_offset_m = None
                self.mav.delivery_note = (
                    f"no target in frame at release; the last sighting was "
                    f"{age} ticks earlier, too stale to stand for it")
                self.mav.log(f"delivery accuracy: {self.mav.delivery_note}",
                             warn=True)
                return
        elif off is None or getattr(off, "z", 0.0) <= 0.0:
            self.mav.delivery_offset_m = None
            self.mav.delivery_note = "no target in frame at release"
            return
        else:
            self.mav.delivery_offset_m = None
            self.mav.delivery_note = "altitude unknown at release"
            return

        # Metres come from the altitude the SIGHTING was taken at, not from
        # wherever the aircraft is now: the same image offset is a different
        # ground distance from a different height.
        half_w = at_alt * math.tan(self.hfov / 2.0)
        half_h = half_w * float(self.image_h_px) / float(self.image_w_px)
        self.mav.delivery_offset_m = math.hypot(ox * half_w, oy * half_h)
        when = ("" if live else
                f", last seen {age} tick(s) before release")
        release_alt = "unknown" if alt is None else f"{alt:.2f} m"
        self.mav.delivery_note = (
            f"estimated offset {self.mav.delivery_offset_m:.2f} m from pad centre; "
            f"release altitude {release_alt}, sighting altitude {at_alt:.2f} m "
            f"(image offset {ox:+.2f}, {oy:+.2f}{when})")
        self.mav.log(f"delivery accuracy: {self.mav.delivery_note}")

    def update(self):
        self._t += 1
        w = self.mav.winch_status
        # Sample all the way down. Reading the offset only on the tick the
        # release fires throws away every good frame that came before it.
        self.observe_offset()

        if self.phase == 0:                            # descend, servoing
            # Keep the MATCHED pad under the aircraft all the way down. The
            # drop point used to be latched once at stage start, so any error
            # left by the search went straight into the scored drop accuracy.
            centred = self._servo_drop_point()
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt,
                          self.mav.yaw())
            at_alt = self.mav.reached(self.drop_x, self.drop_y,
                                      self.drop_alt, 0.5)
            self._settle = self._settle + 1 if (at_alt and centred) else 0
            waited = self._t - self._phase_started
            if self._settle >= self.settle_ticks or \
                    (at_alt and waited > self.servo_timeout):
                if self._settle < self.settle_ticks:
                    self.mav.log("drop: pad not held centred within the "
                                 f"servo window ({waited} ticks); lowering "
                                 "at the last tracked point", warn=True)
                self.phase = 1
                self._phase_started = self._t

        elif self.phase == 1:                          # lower until DOWN
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt, self._yaw)
            self.mav.winch("lower")
            self.feedback_message = (f"lowering: payout="
                                     f"{w.get('payout_m', 0.0)} "
                                     f"state={w.get('state', '?')}")
            # Wait for the winch to REPORT the payload down, not for a timer.
            # The old code released after a fixed 20 ticks whether or not
            # anything had moved — and nothing was subscribed to move.
            if w.get("state") in ("AT_GROUND",) or w.get("ground"):
                self.phase = 2
                self._phase_started = self._t
            elif self._t - self._phase_started > self.lower_timeout:
                reason = (f"winch did not reach the ground in "
                          f"{self._t - self._phase_started} ticks "
                          f"(status={w or 'no /winch/status'})")
                self.feedback_message = reason
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE

        elif self.phase == 2:                          # release, gated
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt, self._yaw)
            if w.get("released"):
                self._record_delivery_offset()
                self.phase = 3
                self._phase_started = self._t
            elif w.get("release_ok"):
                self.mav.winch("release")
            else:
                self.feedback_message = ("waiting on release interlocks: "
                                         + "; ".join(w.get("blockers", [])))
                if self._t - self._phase_started > self.release_timeout:
                    reason = ("payload release never permitted: "
                              + "; ".join(w.get("blockers", ["no status"])))
                    self.mav.abort_reason = reason
                    return py_trees.common.Status.FAILURE

        elif self.phase == 3:                          # stow, over the pad
            # Hold the drop point while the hook winds up, so the camera's
            # view of the pad is the same one the payload fell into.
            self.mav.winch("stow")
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt, self._yaw)
            stowed = float(w.get("payout_m", 0.0)) <= 0.1
            if stowed or self._t - self._phase_started > self.stow_timeout:
                self.phase = 4
                self._phase_started = self._t
                self._confirm = 0

        elif self.phase == 4:                          # the camera confirms
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt, self._yaw)
            verdict = self._check_payload()
            if verdict is not None:
                self._settle_delivery(True, verdict)
                self.phase = 5
            elif self._t - self._phase_started > self.confirm_timeout:
                self._settle_delivery(False, self._last_payload_reason)
                self.phase = 5

        elif self.phase == 5:                          # climb back
            self.mav.goto(self.drop_x, self.drop_y, self.cruise_alt, self._yaw)
            if self.mav.reached(self.drop_x, self.drop_y, self.cruise_alt, 0.6):
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    # ---- camera confirmation -------------------------------------------------
    _last_payload_reason = "no payload detection received"

    def _ground_offset(self, ox, oy, alt):
        """Image offset (normalised) -> (forward, right) metres on the ground."""
        half_w = alt * math.tan(self.hfov / 2.0)
        half_h = half_w * float(self.image_h_px) / float(self.image_w_px)
        return -float(oy) * half_h, float(ox) * half_w

    def _check_payload(self):
        """Count consecutive frames with the payload ON THE GROUND in view.

        Returns None until `confirm_frames` in a row agree, then a dict with
        the payload's distance from the pad centre if the pad is in the same
        frame (None if not).
        """
        seen = getattr(self.mav, "payload_seen", None)
        det = seen() if callable(seen) else None
        alt = self.mav.alt()
        if not det or not det.get("visible") or not alt or alt <= 0:
            self._confirm = 0
            self._last_payload_reason = ("the camera does not see the payload"
                                         if det is not None else
                                         "no fresh payload detection")
            return None
        img_w = float(det.get("img_w", self.image_w_px))
        expected = (self.payload_size_m / (2.0 * alt * math.tan(self.hfov / 2.0))
                    * img_w)
        size = float(max(det.get("w_px", 0), det.get("h_px", 0)))
        if not (0.4 * expected <= size <= 2.5 * expected):
            self._confirm = 0
            self._last_payload_reason = (
                f"a yellow blob {size:.0f} px across, but a payload on the "
                f"ground {alt:.1f} m below would be ~{expected:.0f} px")
            return None
        self._confirm += 1
        if self._confirm < self.confirm_frames:
            return None
        pf, pr = self._ground_offset(det["x"], det["y"], alt)
        off = getattr(self.mav, "qr_offset", None)
        if off is not None and getattr(off, "z", 0.0) >= 0.99:
            qf, qr_ = self._ground_offset(off.x, off.y, alt)
            return {"from_pad_m": math.hypot(pf - qf, pr - qr_), "size_px": size}
        return {"from_pad_m": None, "size_px": size}

    def _settle_delivery(self, confirmed, detail):
        self.mav.delivery_confirmed = bool(confirmed)
        if confirmed:
            d = detail.get("from_pad_m")
            if d is not None:
                # The camera's own measurement of the thing that is scored:
                # payload against pad, in one frame. It replaces the release-
                # time estimate, which measured the AIRCRAFT against the pad.
                self.mav.delivery_offset_m = d
                self.mav.delivery_note = (
                    f"payload confirmed on the ground by the camera, "
                    f"{d:.2f} m from the pad centre")
            else:
                self.mav.delivery_note = (
                    "payload confirmed on the ground by the camera (pad not in "
                    "the same frame; offset is the release-time estimate)")
            self.mav.log(f"delivery: {self.mav.delivery_note}")
        else:
            self.mav.delivery_note = (f"payload NOT confirmed on the ground "
                                      f"after release: {detail}")
            self.mav.log(f"delivery: {self.mav.delivery_note}", warn=True)


class GotoHome(py_trees.behaviour.Behaviour):
    """Return to the FC's home position, not to a hardcoded (0, 0).

    Audit A11. `home = (0.0, 0.0, 5.0)` was correct only because the EKF origin
    is set at the arming point, which made the constant and the truth coincide.
    Asking the flight controller states the dependency instead of relying on it.
    """

    def __init__(self, mav, alt, tol=0.8, clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__("GotoHome")
        self.mav = mav
        self.alt = alt
        self.tol = tol
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)

    def initialise(self):
        self.router.reset()

    def update(self):
        hx, hy = self.mav.home_local_xy()
        status = self.router.fly(self.mav, hx, hy, self.alt)
        if status is BLOCKED:
            return _blocked(self, self.router)
        if status is ARRIVED:
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"home=({hx:.1f},{hy:.1f})"
        return py_trees.common.Status.RUNNING


class PrecisionDescent(py_trees.behaviour.Behaviour):
    """Descend onto the home pad under continuous visual lock (Phase 9).

    WHAT WAS WRONG
        Landing was `mav.land()` — hand ArduPilot the LAND mode and hope. LAND
        descends wherever the aircraft happens to be. GPS put it within a few
        metres of home; the pad is not a few metres wide. Accuracy was never
        measured because there was nothing to measure: the stack had no idea
        where the pad was during the descent.

    HOW THIS WORKS
        GPS gets the aircraft over home — that is the one legitimate global
        reference. From there the nadir camera locks the home fiducial and the
        descent is closed-loop on the image offset, exactly the way CenterOnQR
        works, with the same derived sign mapping.

    LOSING LOCK IS NOT A REASON TO KEEP DESCENDING
        Above `commit_alt` a lost fiducial stops the descent and holds, then
        climbs to re-acquire. Continuing blind is how a precision landing
        becomes an ordinary one that believes it was precise.

        BELOW `commit_alt` the aircraft is close enough that the pad has left
        the camera's field of view, and climbing back up to look for it would
        be worse than finishing. That is a deliberate, bounded commitment, not
        an oversight.
    """

    def __init__(self, mav, start_alt=5.0, commit_alt=1.5, step_m=0.4,
                 tol=0.10, gain=0.9, max_step=0.8, reacquire_climb=0.5,
                 timeout_ticks=600, lost_grace=10,
                 marker_m=None, hfov_rad=1.0472,
                 image_w_px=1280, image_h_px=720):
        super().__init__("PrecisionDescent")
        self.mav = mav
        self.start_alt = start_alt
        # The commit altitude cannot be lower than the altitude at which the
        # marker still fits in frame. `land_commit_alt` was 1.5 m; for the
        # 2.2 m pad the marker overflows the picture below about 2.5 m, so the
        # descent was being asked to hold a lock the camera could not provide.
        # Runs 12 and 14 both oscillated between 3.4 m and 4.1 m until the
        # stage timed out -- chasing a floor the geometry forbids.
        self.track_floor = (
            min_track_altitude(marker_m, hfov_rad, image_w_px, image_h_px)
            if marker_m else 0.0)
        self.commit_alt = max(commit_alt, self.track_floor)
        # ROOM TO LOCK. The descent tracks down to commit_alt, and commit_alt
        # is where the marker stops fitting in the frame -- 4.96 m for a 2.2 m
        # marker in the C270's 28.6 deg front-to-back field. Starting at 5 m
        # left 4 cm of tracking: on the team airframe it never locked, and
        # landed 2.7 m from the pad. Start at least this far above it.
        self.start_alt = max(start_alt, self.commit_alt + 1.5)
        self.step_m = step_m
        self.tol = tol
        self.gain = gain
        self.max_step = max_step
        self.reacquire_climb = reacquire_climb
        self.timeout_ticks = timeout_ticks
        self.lost_grace = lost_grace
        self._t = 0
        self._lost = 0
        self.reacquisitions = 0

    def initialise(self):
        self._t = 0
        self._lost = 0
        self.reacquisitions = 0
        self.mav.landing_precision = "UNKNOWN"
        self.mav.log(
            f"precision descent: committing at {self.commit_alt:.2f} m "
            f"(marker leaves frame below {self.track_floor:.2f} m)")

    def update(self):
        self._t += 1
        x, y, z = self.mav.pos()
        psi = self.mav.yaw()

        if z <= self.commit_alt:
            # Committed: close enough that the pad is out of frame anyway.
            self.mav.landing_precision = (
                f"PRECISE (committed at {z:.2f} m, "
                f"{self.reacquisitions} re-acquisition(s))")
            self.feedback_message = (f"committed at {z:.2f} m "
                                     f"({self.reacquisitions} re-acquisitions)")
            return py_trees.common.Status.SUCCESS

        if self._t > self.timeout_ticks:
            # DEGRADE, do not fail. The payload has already been delivered;
            # the only thing at stake here is whether the touchdown is on the
            # pad or merely near it, and an ordinary landing is a worse
            # outcome than a precise one -- not a failed mission.
            #
            # A live run hung here at 2.57 m for 601 ticks and reported the
            # whole mission FAILED after a successful delivery, which is a
            # less truthful answer than "landed, precision not achieved".
            reason = (f"precision lock not achieved by {z:.2f} m after "
                      f"{self._t} ticks; landing normally")
            self.mav.landing_precision = f"DEGRADED ({reason})"
            self.feedback_message = reason
            self.mav.log(f"PrecisionDescent: {reason}", warn=True)
            return py_trees.common.Status.SUCCESS

        if not self.mav.qr_visible():
            self._lost += 1
            if self._lost == self.lost_grace + 1:
                self.reacquisitions += 1
            # WHERE THE PAD IS, not where the aircraft is. GPS puts it over
            # home; the pad was read and centred on at the start, 1 m from
            # home in the shipped layout -- at the frame edge from here.
            fix = getattr(self.mav, "home_marker_xy", None)
            if fix is not None and math.hypot(fix[0] - x, fix[1] - y) > 0.3:
                self.mav.goto(fix[0], fix[1], max(z, self.start_alt), psi)
                self.feedback_message = (f"marker not in view; going to the "
                                         f"pad's fix ({fix[0]:.1f}, {fix[1]:.1f})")
                return py_trees.common.Status.RUNNING
            if self._lost > self.lost_grace:
                # Climb to widen the footprint and look again. Descending
                # blind would make an ordinary landing look like a precise one.
                self.mav.goto(x, y, min(self.start_alt,
                                        z + self.reacquire_climb), psi)
                self.feedback_message = (f"lock lost at {z:.2f} m; climbing to "
                                         f"re-acquire")
            else:
                self.mav.goto(x, y, z, psi)         # hold, do not descend
                self.feedback_message = f"lock lost at {z:.2f} m; holding"
            return py_trees.common.Status.RUNNING

        self._lost = 0
        # Same image->local mapping CenterOnQR uses and its tests assert:
        # forward correction = -offset.y, right = +offset.x, rotated by yaw.
        off = self.mav.qr_offset
        err = math.hypot(off.x, off.y)
        fwd = max(-self.max_step, min(self.max_step, -off.y * self.gain))
        right = max(-self.max_step, min(self.max_step, off.x * self.gain))
        tx = x + fwd * math.cos(psi) + right * math.sin(psi)
        ty = y + fwd * math.sin(psi) - right * math.cos(psi)

        # Only descend once centred: coming down off-centre just moves the
        # error closer to the ground where there is less room to fix it.
        tz = z - self.step_m if err <= self.tol else z
        self.mav.goto(tx, ty, max(self.commit_alt, tz), psi)
        self.feedback_message = (f"descending {z:.2f} m, err {err:.3f} "
                                 f"{'(centred)' if err <= self.tol else ''}")
        return py_trees.common.Status.RUNNING


class Land(py_trees.behaviour.Behaviour):
    def __init__(self, mav):
        super().__init__("Land"); self.mav = mav

    def initialise(self):
        # Tell the commander that the coming disarm is ours, so the external
        # intervention watchdog does not read a normal landing as a takeover.
        self.mav.expect_disarm(True)
        # Descending to the ground is the point of this leaf.
        self.mav.clear_airborne_floor()

    def update(self):
        self.mav.land()
        # Disarmed means the FCU said so. A dropped link defaults the MAVROS
        # state to armed=False, and the abort guard gives a drop a short
        # grace, so without the connected check a heartbeat hiccup during
        # the descent would be reported as a completed landing.
        if self.mav.state.armed or not self.mav.connected():
            return py_trees.common.Status.RUNNING
        precision = getattr(self.mav, "landing_precision", "UNKNOWN")
        # Delivery accuracy is 15 rulebook marks; landing is 5. Reporting the
        # tracking-quality figure and staying silent about the drop put the
        # smaller number in front of the operator and hid the larger one.
        drop = getattr(self.mav, "delivery_offset_m", None)
        drop_txt = ("delivery UNMEASURED" if drop is None
                    else f"delivery {drop:.2f} m from pad centre")
        # A drop the camera could not confirm is not a completed delivery,
        # whatever the winch said. The aircraft still comes home and lands --
        # that is the safe thing to do -- but the outcome says so, and the
        # regression does not count it.
        confirmed = getattr(self.mav, "delivery_confirmed", None)
        state = "COMPLETED"
        if confirmed is True:
            drop_txt += " (payload seen on the ground by the camera)"
        elif confirmed is False:
            state = "DELIVERY_UNCONFIRMED"
            drop_txt = ("payload NOT confirmed on the ground by the camera; "
                        + drop_txt)
        self.mav.publish_result(
            state, f"landed and disarmed; {drop_txt}; landing {precision}")
        return py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------- #
def _node_clock(node):
    """Seconds on the node's ROS clock (sim time in simulation), or None.

    None lets a stage fall back to time.monotonic(), which is what the tests
    (a MagicMock node, no /clock) and the aircraft (sim time == wall time)
    both expect.
    """
    if not isinstance(node, Node):
        return None
    return lambda: node.get_clock().now().nanoseconds * 1e-9


def build_root(mav, node, p):
    root = py_trees.composites.Selector(name="Root", memory=False)
    
    # Guard sequence: only ticks StageAwareAbort when CheckAbortTriggered succeeds
    abort_branch = py_trees.composites.Sequence(name="AbortBranch", memory=False)
    abort_branch.add_children([CheckAbortTriggered(mav, node), StageAwareAbort(mav)])

    # Camera pointing is now an explicit, gated step before every stage whose
    # perception depends on it (Phase 2). goal.md Q18:
    #   NADIR   (-90) start-QR scan, lawnmower search, winch drop, landing
    #   FORWARD (  0) banner alignment, corridor navigation
    # The sweep plans itself from the zone ObserveZone measures, which does not
    # exist when the tree is built -- hence a callable, resolved at initialise().
    # The search area is the ORGANISER'S delivery-zone boundary (rulebook:
    # "Delivery zone geo-fence coordinates will be provided to teams"), inset
    # by the boundary clearance. It used to be a lidar glance from the
    # corridor mouth, grown outward by a 60 m budget until something matched
    # -- which is what flew the aircraft out of the area.
    zone_clearance = p.get('zone_boundary_clearance', 1.0)
    # EVERY DWELL AND HOVER ON THE NODE'S CLOCK. The tree ticks on a ROS timer,
    # which follows /clock in simulation, but the dwell stages read
    # time.monotonic(). At a real-time factor of 0.28 a "5 s" banner dwell was
    # 1.4 simulated seconds -- three camera frames against a four-sample
    # minimum -- so no dwell could ever be confident. On the aircraft the two
    # clocks are the same clock.
    clock = _node_clock(node)
    search = LawnmowerSearch(mav, lambda: mav.delivery_search_zone(zone_clearance),
                             p['search_alt'], clock=clock, crab=True,
                             image_height_px=p.get('image_height_px', 720),
                             search_speed_mps=p.get('search_speed_mps', 2.5),
                             exclusions=lambda: mav.exclusions,
                             clearance_m=p.get('redzone_clearance', 1.5),
                             hover_s=p.get('qr_hover_s', 5.0),
                             image_width_px=p.get('image_width_px', 1280),
                             hfov_rad=p.get('camera_hfov', 1.0472),
                             marker_m=p.get('target_marker_m', 2.2),
                             px_floor=p.get('px_per_module_floor', 5.3),
                             overlap=p.get('lane_overlap', 0.30),
                             # No outward expansion inside a supplied boundary.
                             search_budget_m=0.0)
    gate_advance_m = p.get('gate_advance_m', 10.0)
    outbound_duck = DuckUnderBoard(mav, need_clear_m=gate_advance_m, clock=clock)
    return_duck = DuckUnderBoard(mav, need_clear_m=gate_advance_m, clock=clock)

    mission = py_trees.composites.Sequence(name="Mission", memory=True)
    mission.add_children([
        WaitForMissionStart(mav),
        # Both organiser inputs are pre-flight requirements: no delivery-zone
        # boundary or no enforced arena geofence, no arming.
        RequireDeliveryZone(mav, clearance_m=zone_clearance),
        UploadArenaFence(mav, zone_clearance_m=0.0),
        SetModeArm(mav),
        Takeoff(mav, p['takeoff_alt']),
        SetCameraPose("CameraNadirForQR", mav, "NADIR"),
        # Phase 3: find the marker, centre on it, THEN decode. Replaces a
        # hardcoded hover over a guessed scan_pose (geometry audit A4).
        FindStartQR(mav, p['takeoff_alt'], floor_alt=p.get('scan_floor_alt', 2.0),
                    hfov_rad=p.get('camera_hfov', 1.0472),
                    image_w_px=p.get('image_width_px', 1280),
                    image_h_px=p.get('image_height_px', 720)),
        # The lens it was built without: every centring step was scaled for a
        # 60 deg camera, 25 % long on the C270.
        CenterOnQR("CenterStartQR", mav,
                   hfov_rad=p.get('camera_hfov', 1.0472),
                   image_w_px=p.get('image_width_px', 1280),
                   image_h_px=p.get('image_height_px', 720)),
        ScanStartQR(mav, hover_s=p.get('qr_hover_s', 5.0), clock=clock),
        SetCameraPose("CameraForwardForCorridor", mav, "FORWARD"),
        # Phase 4: the banner marks the corridor mouth. Aligning to it replaces
        # the hardcoded corridor_entry waypoint (geometry audit A5).
        # RULEBOOK ORDER (Mission 2, Operation / Corridor Navigation):
        # "identify the AeroTHON 2026 Green Banner marking the entrance of the
        # navigation corridor and align itself BEFORE descending to the
        # corridor navigation altitude".
        #
        # Descending first was tried and works, but it inverts the required
        # order. The reason it was tempting is that from 5 m a gate a few
        # metres ahead sits entirely below a LEVEL camera, so the aircraft
        # swept 271 degrees past it. The fix is to point the camera where the
        # banner actually is rather than to move the aircraft: the BANNER pose
        # looks 20 degrees down, which covers the gate from scan altitude.
        SetCameraPose("CameraBannerSearch", mav, "BANNER"),
        AlignToBanner(mav, clock=clock,
                      image_width_px=p.get('image_width_px', 1280),
                      redzone_clearance_m=p.get('redzone_clearance', 1.5),
                      sweep_limit_rad=p.get('banner_sweep_limit', 2 * math.pi),
                      step_rad=p.get('banner_sweep_step', math.radians(45.0)),
                      dwell_s=p.get('banner_dwell_s', 5.0),
                      min_hit_ratio=p.get('banner_min_hit_ratio', 0.6),
                      hfov_rad=p.get('camera_hfov', 1.0472),
                      square_tol_rad=p.get('square_tol_rad',
                                           math.radians(5.0)),
                      lidar_range_m=p.get('lidar_range_m', 12.0),
                      min_standoff_m=p.get('min_standoff_m', 2.5),
                      # The lidar sweeps ONE horizontal plane, and from the
                      # scan altitude that plane can clear the gate entirely
                      # -- 0 finite returns of 720, measured. The stage may
                      # step down to the corridor altitude to bring the banner
                      # into it, which is where the next leaf takes it anyway.
                      alt_floor_m=p['corridor_alt']),
        # LOWER ONLY ONCE SQUARED UP. Descending while still off to one side
        # of the gate put the aircraft low and pointed at the board, and it
        # drove into it. AlignToBanner now finishes with the aircraft in front
        # of the banner (yaw to face it, roll to come into line -- never
        # pitch), so this is a descent in place at the right spot.
        ClimbInPlace("DescendToCorridorAlt", mav, p['corridor_alt']),
        SetCameraPose("CameraForwardForCorridor2", mav, "FORWARD"),
        outbound_duck,
        # Aligning to the banner is not the same as arriving at it: the gate
        # stands off the takeoff axis, and an aligned-but-not-approached
        # aircraft flew straight past it into a corner (audit A5).
        # A MEASURED DISTANCE THROUGH THE GATE, not "until the banner leaves
        # the frame". The old end condition treated getting close to the board
        # as having passed it, so the mission entered the delivery-zone stages
        # while still at the mouth.
        GateAdvance(mav, advance_m=lambda: outbound_duck.advance_m, tol=0.25,
                    alt=lambda: outbound_duck.alt,
                    clearance_m=p.get('redzone_clearance', 1.5),
                    exclusions=lambda: mav.exclusions),
        Corridor("Corridor", mav, forward=True, alt=p['corridor_alt']),
        # Nadir BEFORE entering the field, so the red-zone detector sees the
        # ground being flown over from the first metre (off nadir it reports
        # NOT_VISIBLE and nothing is mapped).
        SetCameraPose("CameraNadirForSearch", mav, "NADIR"),
        # Into the supplied delivery field at corridor altitude (routed round
        # red ground), then "ascend to approximately 10 meters altitude".
        EnterDeliveryZone(mav, alt=p['corridor_alt'],
                          clearance_m=zone_clearance),
        ClimbInPlace("ClimbToSweep", mav, lambda: search.alt),
        search,
        # Sweep high to FIND, descend to READ: the sweep altitude is set by
        # pad detectability, which is well above the measured decode envelope.
        DescendToDecode(mav, decode_alt=lambda: search.decode_alt,
                        floor_alt=p.get('scan_floor_alt', 2.0),
                        clearance_m=p.get('redzone_clearance', 1.5),
                        exclusions=lambda: mav.exclusions),
        # Centre over the MATCHED pad before descending to the drop altitude.
        # Drop accuracy is 15 marks; the decode point is wherever the lane
        # happened to be when the pad came into frame.
        CenterOnQR("CenterOnTarget", mav, tol=0.08, require_match=True,
                   settle_ticks=5, timeout_ticks=200,
                   hfov_rad=p.get('camera_hfov', 1.0472),
                   image_w_px=p.get('image_width_px', 1280),
                   image_h_px=p.get('image_height_px', 720)),
        WinchDrop(mav, p['drop_alt'], p['search_alt'],
                  hfov_rad=p.get('camera_hfov', 1.0472),
                  image_w_px=p.get('image_width_px', 1280),
                  image_h_px=p.get('image_height_px', 720),
                  marker_m=p.get('target_marker_m', 2.2)),
        # RULEBOOK: "After payload delivery, the UAS must ascend to 10-meter
        # altitude, navigate back through the corridor". Transiting the
        # delivery zone at corridor altitude was both a rule deviation and
        # needlessly low over the pads it had just flown over.
        ClimbInPlace("ClimbForReturn", mav, p['search_alt']),
        # Back to where the corridor actually opened out (audit A9) -- at
        # transit altitude, not corridor altitude.
        #
        # CAMERA STAYS NADIR FOR THE TRANSIT. It used to switch to the BANNER
        # pose first, and off nadir the red-zone detector reports NOT_VISIBLE:
        # the whole crossing of the delivery zone was flown blind to red
        # ground it had not already mapped. Seed 1004 clipped an unmapped
        # corner of the main zone at 0.35 m that way. Point it at the banner
        # only once the aircraft is standing off the corridor mouth.
        SetCameraPose("CameraNadirForReturnTransit", mav, "NADIR"),
        ReturnToCorridorMouth(mav, alt=p['search_alt'],
                              ident_alt=p['takeoff_alt'],
                              clearance_m=p.get('redzone_clearance', 1.5),
                              lane_offset_m=p.get('return_lane_offset_m', -4.0)),
        SetCameraPose("CameraBannerReturn", mav, "BANNER"),
        # RULEBOOK, "Corridor Entry Detection Return Lap": the banner must be
        # detected and the aircraft aligned again for the return lap. This is
        # a separately scored task; the return leg previously just flew back
        # through on the recorded pose without ever looking for the gate.
        #
        # Identify from the SAME altitude the outbound leg uses. The
        # rulebook's "ascend to 10 m" governs crossing the delivery zone,
        # which ReturnToCorridorMouth above has now done; identification then
        # mirrors the outbound sequence the rulebook itself prescribes --
        # identify at 5 m, then descend to the 3 m corridor altitude.
        #
        # Seed 1001 failed here on every recorded run. Sweeping from 10 m the
        # detector reported, across one sweep:
        #
        #     only 1 white component(s); lettering expected   (x895)
        #     only 0 white component(s)                       (x92)
        #     board aspect 0.73 outside 1.2-8.0               (x70)
        #
        # The lettering is not resolvable at that range and depression angle,
        # so the text rescue cannot help either: it needs letter blobs to read
        # and there is one. The outbound leg identifies the same banner from
        # 5 m without trouble.
        ClimbInPlace("DescendToReturnIdent", mav, p['takeoff_alt']),
        # WHERE the return corridor is, found with the camera. Beside the
        # outbound lane (the rulebook drawing) it is identified from the
        # stand-off on the first look; anywhere else, the aircraft searches
        # the zone's edge for it. See FindReturnBanner.
        FindReturnBanner(mav, alt=p['takeoff_alt'], clock=clock,
                         hfov_rad=p.get('camera_hfov', 1.0472),
                         image_width_px=p.get('image_width_px', 1280),
                         near_range_m=0.8 * p.get('lidar_range_m', 12.0),
                         clearance_m=p.get('redzone_clearance', 1.5),
                         orbit_alt_m=p['corridor_alt']),
        AlignToBanner(mav, clock=clock,
                      image_width_px=p.get('image_width_px', 1280),
                      redzone_clearance_m=p.get('redzone_clearance', 1.5),
                      sweep_limit_rad=p.get('banner_sweep_limit', 2 * math.pi),
                      step_rad=p.get('banner_sweep_step', math.radians(45.0)),
                      dwell_s=p.get('banner_dwell_s', 5.0),
                      min_hit_ratio=p.get('banner_min_hit_ratio', 0.6),
                      hfov_rad=p.get('camera_hfov', 1.0472),
                      square_tol_rad=p.get('square_tol_rad',
                                           math.radians(5.0)),
                      lidar_range_m=p.get('lidar_range_m', 12.0),
                      min_standoff_m=p.get('min_standoff_m', 2.5),
                      # The lidar sweeps ONE horizontal plane, and from the
                      # scan altitude that plane can clear the gate entirely
                      # -- 0 finite returns of 720, measured. The stage may
                      # step down to the corridor altitude to bring the banner
                      # into it, which is where the next leaf takes it anyway.
                      alt_floor_m=p['corridor_alt']),
        ClimbInPlace("DescendToReturnCorridor", mav, p['corridor_alt']),
        SetCameraPose("CameraForwardForReturn", mav, "FORWARD"),
        return_duck,
        GateAdvance(mav, advance_m=lambda: return_duck.advance_m, tol=0.25,
                    alt=lambda: return_duck.alt,
                    clearance_m=p.get('redzone_clearance', 1.5),
                    exclusions=lambda: mav.exclusions),
        Corridor("ReturnCorridor", mav, forward=False, alt=p['corridor_alt']),
        GotoHome(mav, p['takeoff_alt'],
                 clearance_m=p.get('redzone_clearance', 1.5)),
        SetCameraPose("CameraNadirForLanding", mav, "NADIR"),
        # Phase 9: GPS gets us over home; the camera puts us on the pad.
        PrecisionDescent(mav, start_alt=p['takeoff_alt'],
                         commit_alt=p.get('land_commit_alt', 1.5),
                         marker_m=p.get('target_marker_m', 2.2),
                         hfov_rad=p.get('camera_hfov', 1.0472),
                         image_w_px=p.get('image_width_px', 1280),
                         image_h_px=p.get('image_height_px', 720)),
        Land(mav),
    ])
    root.add_children([abort_branch, mission])
    return root


def apply_pending_reset(root, mav):
    """Fail-closed rail: return the tree to WAITING after an intervention.

    A py_trees Sequence with memory=True retains its child index, so an
    external LAND/DISARM previously left the mission "stuck" at whatever leg
    it had reached — the last live run reported GOTO_CORRIDOR while sitting
    disarmed on the ground. Invalidating the root forces every leaf back
    through initialise() on the next tick.

    Returns the reset reason if one was applied, else None.
    """
    reason = mav.consume_reset()
    if reason is None:
        return None
    mav.publish_result("INTERRUPTED", reason)
    root.stop(py_trees.common.Status.INVALID)
    return reason


def latch_mission_failure(root, mav):
    """Park the tree on a failed mission instead of silently restarting it.

    The root is a Selector over [abort_branch, mission]. When the mission
    Sequence fails — search exhausted with no matching QR, start QR never
    decoded, corridor altitude lost — the Selector has no viable child and the
    root returns FAILURE. On the very next tick py_trees re-initialises the
    memory Sequence from its first child and the whole mission starts again,
    mid-flight. Observed live as SEARCH_QR -> START_QR with the aircraft still
    airborne.

    A failed mission is a terminal outcome. Latch it, report why, and require
    an explicit new START.

    Returns the reason if a failure was latched on this tick, else None.
    """
    if mav.mission_failed or root.status != py_trees.common.Status.FAILURE:
        return None
    if not mav.mission_started:
        return None

    reason = mav.abort_reason or "mission sequence failed"
    mav.mission_failed = True
    mav.publish_result("FAILED", reason)
    mav.mission_started = False
    mav.enable_avoidance(False)
    root.stop(py_trees.common.Status.INVALID)
    return reason


def latch_mission_success(root, mav):
    """A completed mission is terminal too, and must not restart itself.

    latch_mission_failure() parked FAILED missions; SUCCESS was left to fall
    through. The Selector returns SUCCESS, py_trees re-initialises the memory
    Sequence on the very next tick, and the aircraft arms and takes off again
    on its own. Observed live: "Mission result: COMPLETED (landed and
    disarmed)" followed 0.4 s later by "Mission state -> ARMING".

    An aircraft that re-arms after a successful delivery is a safety problem,
    not a cosmetic one. Require an explicit new START.

    Returns the reason if a success was latched on this tick, else None.
    """
    if mav.mission_complete or root.status != py_trees.common.Status.SUCCESS:
        return None
    if not mav.mission_started:
        return None

    mav.mission_complete = True
    mav.publish_result("COMPLETED", "landed and disarmed")
    mav.mission_started = False
    mav.enable_avoidance(False)
    root.stop(py_trees.common.Status.INVALID)
    return "landed and disarmed"


def declare_mission_params(node):
    """Every mission constant as a REAL ROS parameter.

    These lived in a plain Python dict whose own comment said "declare as
    params" — they never were, so none of them could be overridden at launch,
    per-arena, or from the GCS. `CURRENT_PROGRESS_HANDOFF.md` called this out
    and it survived every phase until now.

    Values that perception has replaced are gone entirely rather than being
    left as dead parameters:
      * scan_pose            -> FindStartQR + CenterOnQR      (audit A4)
      * corridor_entry        -> AlignToBanner                 (audit A5)
      * corridor_exit_x       -> navigator's corridor_exited   (audit A6)
      * corridor_return_exit_x-> same detector                 (audit A10)
      * search_alt / spacing  -> search_planner                (audit A2, B1)

    What remains is genuinely input, not assumption: rulebook altitudes, the
    camera description, the marker description, and the delivery-zone bounds
    that still await zone-boundary perception (audit A7, A8).
    """
    d = node.declare_parameter

    # --- rulebook altitudes -------------------------------------------- #
    d('takeoff_alt', 5.0)
    d('search_alt_max', 10.0)      # CEILING for the derived sweep, not a target
    d('drop_alt', 5.0)

    # --- camera + marker: inputs to the derived search geometry --------- #
    d('image_width_px', 1280)
    d('camera_hfov', 1.0472)
    d('target_marker_m', 2.2)      # competition size UNCONFIRMED; see
                                   # docs/QR_DECODE_ENVELOPE.md
    d('qr_modules', 33)
    d('px_per_module_floor', 5.3)  # MEASURED in Phase 1
    d('lane_overlap', 0.30)

    # --- corridor / zone ------------------------------------------------ #
    # zone_entry, zone_bounds and corridor_return_entry are GONE (audit A7, A8,
    # A9): the zone is measured at the corridor mouth by ObserveZone and the
    # way back in is the recorded exit, reversed. What is left is a safety
    # inset and the rulebook corridor altitude.
    d('zone_margin', 1.0)
    d('fence_margin', 5.0)
    # How far PAST the first observed window the search may push its frontier.
    #
    # This is not the arena's size -- the aircraft never assumes that. It is
    # how far this airframe is willing to go looking on the available
    # endurance, which is a property of the vehicle. The lidar decides where
    # to stop; this decides when to give up.
    d('search_budget_m', 60.0)
    d('redzone_clearance', 1.5)
    # Ground speed during the lawnmower. Look-ahead (5.8 m crabbed at 10 m)
    # has to cover the red-zone confirmation latency plus braking.
    d('search_speed_mps', 2.5)
    d('corridor_alt', 3.0)

    # --- tolerances (audit B3, C3) -------------------------------------- #
    d('waypoint_tol', 0.8)
    d('drop_tol', 0.5)
    d('scan_floor_alt', 2.0)
    # A FLOOR, not the commit altitude: PrecisionDescent raises it to the
    # altitude at which the marker still fits in frame (see min_track_altitude).
    d('land_commit_alt', 1.5)
    d('image_height_px', 720)
    d('banner_sweep_limit', 2 * math.pi)
    # 45 deg: eight headings a turn. The 60 deg field of view still overlaps
    # each neighbour by 15 deg, so no bearing falls between two stares; 30 deg
    # stared at every direction twice.
    d('banner_sweep_step', math.radians(45.0))
    d('banner_dwell_s', 5.0)
    d('banner_min_hit_ratio', 0.6)
    # SQUARENESS, measured with the lidar rather than inferred from a camera
    # bounding box. An angle in degrees means what it says; the aspect ratio
    # it replaces plateaued at 1.88-1.91 and no threshold above that was
    # reachable.
    d('square_tol_deg', 5.0)
    # Properties of the SENSOR and the AIRFRAME, not of the arena: the C1's
    # useful range sets how far off the banner the aircraft may drift, and the
    # airframe clearance sets how close it may come.
    d('lidar_range_m', 12.0)
    d('min_standoff_m', 2.5)
    # The corridor's geometry, not a position in the arena: the return lane's
    # centreline relative to the outbound lane's, port-positive facing out of
    # the corridor. Measure it on the real corridor.
    d('return_lane_offset_m', -4.0)
    d('qr_hover_s', 5.0)
    d('gate_advance_m', 10.0)

    g = lambda n: node.get_parameter(n).value
    return {
        'takeoff_alt': float(g('takeoff_alt')),
        'search_alt': float(g('search_alt_max')),
        'drop_alt': float(g('drop_alt')),
        'image_width_px': int(g('image_width_px')),
        'image_height_px': int(g('image_height_px')),
        'camera_hfov': float(g('camera_hfov')),
        'target_marker_m': float(g('target_marker_m')),
        'qr_modules': int(g('qr_modules')),
        'px_per_module_floor': float(g('px_per_module_floor')),
        'lane_overlap': float(g('lane_overlap')),
        'zone_margin': float(g('zone_margin')),
        'fence_margin': float(g('fence_margin')),
        'search_budget_m': float(g('search_budget_m')),
        'redzone_clearance': float(g('redzone_clearance')),
        'corridor_alt': float(g('corridor_alt')),
        'waypoint_tol': float(g('waypoint_tol')),
        'drop_tol': float(g('drop_tol')),
        'scan_floor_alt': float(g('scan_floor_alt')),
        'land_commit_alt': float(g('land_commit_alt')),
        'banner_sweep_limit': float(g('banner_sweep_limit')),
        'banner_sweep_step': float(g('banner_sweep_step')),
        'banner_dwell_s': float(g('banner_dwell_s')),
        'banner_min_hit_ratio': float(g('banner_min_hit_ratio')),
        'square_tol_rad': math.radians(float(g('square_tol_deg'))),
        'lidar_range_m': float(g('lidar_range_m')),
        'min_standoff_m': float(g('min_standoff_m')),
        'return_lane_offset_m': float(g('return_lane_offset_m')),
        'qr_hover_s': float(g('qr_hover_s')),
        'gate_advance_m': float(g('gate_advance_m')),
    }


def main():
    rclpy.init()
    node = Node("mission_bt")

    p = declare_mission_params(node)
    mav = Mav(node)
    tree = py_trees.trees.BehaviourTree(build_root(mav, node, p))
    tree.setup(timeout=15.0)
    state_pub = node.create_publisher(String, "/mission/state", 10)
    state_names = {
        "WaitForMissionStart": "WAITING", "SetModeArm": "ARMING",
        "Takeoff": "TAKEOFF", "ScanStartQR": "START_QR",
        "CameraNadirForQR": "CAMERA_NADIR", "CameraForwardForCorridor": "CAMERA_FWD",
        "CameraNadirForSearch": "CAMERA_NADIR", "CameraForwardForReturn": "CAMERA_FWD",
        "CameraNadirForLanding": "CAMERA_NADIR",
        "AlignToBanner": "BANNER_ALIGN",
        "Corridor": "CORRIDOR_NAV",
        "GotoZone": "ENTER_ZONE", "Climb10": "ENTER_ZONE",
        "RequireDeliveryZone": "PREFLIGHT", "UploadArenaFence": "PREFLIGHT",
        "EnterDeliveryZone": "ENTER_ZONE", "ClimbToSweep": "ENTER_ZONE",
        "CenterOnTarget": "CENTER_TARGET", "DescendToDecode": "SEARCH_QR",
        "ClimbForReturn": "RETURN_TRANSIT",
        "ReturnToCorridorMouth": "RETURN_TRANSIT",
        "DescendToReturnIdent": "RETURN_TRANSIT",
        "FindReturnBanner": "RETURN_GATE_SEARCH",
        "DuckUnderBoard": "GATE_CROSSING", "GateAdvance": "GATE_CROSSING",
        "DescendToCorridorAlt": "GATE_CROSSING",
        "DescendToReturnCorridor": "GATE_CROSSING",
        "FindStartQR": "START_QR", "CenterStartQR": "START_QR",
        "PrecisionDescent": "LAND",
        "LawnmowerSearch": "SEARCH_QR", "WinchDrop": "WINCH_DROP",
        "ReturnToCorridor": "RETURN", "ReturnCorridor": "RETURN_CORRIDOR",
        "GotoHome": "RETURN", "Land": "LAND", "StageAwareAbort": "ABORT",
    }
    last_state = {"value": ""}

    def tick_tree():
        # BehaviourTree.tick_tock() blocks forever. Running it before
        # rclpy.spin() previously prevented every subscription, service
        # response and setpoint timer in this node from being processed.
        tree.tick()
        tip = tree.tip()
        state = state_names.get(tip.name if tip else "", "IDLE")

        # A failed mission sequence is terminal; latch it before anything else
        # so the memory Sequence cannot restart from the top on the next tick.
        failure_reason = latch_mission_failure(tree.root, mav)
        if failure_reason is not None:
            node.get_logger().error(f"Mission FAILED: {failure_reason}")
            state = "FAILED"

        # A COMPLETED mission is terminal in exactly the same way. Without
        # this the tree re-armed and took off again 0.4 s after landing.
        success_reason = latch_mission_success(tree.root, mav)
        if success_reason is not None:
            node.get_logger().info(f"Mission COMPLETE: {success_reason}")
            state = "COMPLETED"

        # Consume interventions *after* ticking so the Land leaf still gets to
        # observe its own disarm and latch COMPLETED before we reset.
        reset_reason = apply_pending_reset(tree.root, mav)
        if reset_reason is not None:
            node.get_logger().warning(f"Mission tree reset: {reset_reason}")
            state = "WAITING"

        # Name the leaf that is actually executing, so the altitude trace in
        # MavCommander can attribute a divergence to a stage instead of a
        # timestamp. tip() is the deepest RUNNING behaviour.
        tip = tree.root.tip()
        mav.active_stage = tip.name if tip is not None else ""

        state_pub.publish(String(data=state))
        if state != last_state["value"]:
            node.get_logger().info(f"Mission state -> {state}")
            last_state["value"] = state

    node.create_timer(0.2, tick_tree)   # 5 Hz without blocking ROS callbacks

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
