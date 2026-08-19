#!/usr/bin/env python3
"""Mission 2 (SkyScan) behaviour tree — RUNNABLE (drives ArduPilot via MAVROS).

Root (Fallback)
├── Guard (Sequence): CriticalOK -> else StageAwareAbort
└── Mission (Sequence): SetModeArm -> Takeoff -> ScanStartQR -> GotoCorridor
    -> Corridor(avoid) -> GotoZone -> Climb10 -> LawnmowerSearch
    -> WinchDrop -> ReturnCorridor(avoid) -> Land

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
from std_msgs.msg import Bool, String

from mission_bt.mav_commander import Mav
from mission_bt.decode_hover import DecodeHover
from mission_bt.geofence import build_fence, compare_fences, inclusion_from_zone
from mission_bt.leg_router import ARRIVED, BLOCKED, LegRouter
from mission_bt.scan_geometry import bearing_to_angle
from mission_bt.search_planner import (
    grow_zone,
    coverage_fraction_excluding, extend_zone, frontier_strip,
    leg_hits_exclusion,
    min_track_altitude, plan_intersects_exclusions, plan_lawnmower_excluding,
    plan_search, point_in_exclusion, zone_from_observation, zone_is_plausible)

# Airframe half-span plus projection margin. Overridden per-stage from the
# `redzone_clearance` mission parameter; this is only the fallback for stages
# constructed without one.
DEFAULT_CLEARANCE_M = 1.5


# --------------------------------------------------------------------------- #
# Guard & Abort
# --------------------------------------------------------------------------- #
class CheckAbortTriggered(py_trees.behaviour.Behaviour):
    """Returns SUCCESS when an abort condition trips, triggering StageAwareAbort."""
    def __init__(self, mav, node):
        super().__init__("CheckAbortTriggered")
        self.mav = mav
        self.node = node

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
        if not self.mav.connected():
            return self._trip("FCU disconnected")
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
                 low_camera_pose="FORWARD"):
        super().__init__("AlignToBanner")
        self.mav = mav
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

    def _enter_square_state(self):
        self._sq_phase = self.MEASURE
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
        self._commanded_yaw = yaw
        self.mav.goto(x, y, z, yaw)

    def _close_dwell(self):
        ratio = (self._hits / self._samples) if self._samples else 0.0
        self.step_reports.append({
            "step": self.step_index,
            "heading_deg": round(math.degrees(self._wrap(self._target_yaw)), 1),
            "samples": self._samples,
            "hits": self._hits,
            "hit_ratio": round(ratio, 2),
        })
        return ratio

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

        if self._relocations >= self.max_relocations:
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

    def _restart_sweep_at(self, x, y, z, yaw):
        """Fly to a new vantage point, THEN sweep it.

        Translating and rotating in the same command reintroduces the coupling
        that made yaw useless in the first place, so the transit holds
        whatever heading the aircraft already has and the sweep starts once it
        has arrived.
        """
        self._anchor = (x, y, z)
        self._pending_sweep_yaw = self._wrap(yaw)
        self._transit_yaw = (self._commanded_yaw
                             if self._commanded_yaw is not None
                             else self.mav.yaw())
        self._enter_square_state()
        self._enter(self.RELOCATE)
        self._hold(self._transit_yaw)
        return py_trees.common.Status.RUNNING

    def _transit(self):
        """Hold heading and altitude discipline while relocating."""
        self._hold(self._transit_yaw)
        ax, ay, az = self._anchor
        arrived = (self.mav.reached(ax, ay, az, self.orbit_arrive_tol)
                   and abs(self.mav.alt() - az) <= self.alt_arrive_tol)
        if arrived or self._elapsed() > self.settle_timeout_s * 3.0:
            self.step_index = 0
            self.step_reports = []
            self._target_yaw = self._pending_sweep_yaw
            self._sweep_origin = self._pending_sweep_yaw
            self._hits = self._samples = 0
            self._best_bearing = None
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
            if self.mav.banner_identified():
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
            f"turning to step {self.step_index + 1}/{self.n_steps} "
            f"({math.degrees(err):.0f} deg to go)")
        return py_trees.common.Status.RUNNING

    def _dwell(self):
        """Hold the heading and collect evidence. Nothing moves here."""
        self._hold(self._target_yaw)
        self._samples += 1
        if self.mav.banner_identified():
            self._hits += 1
            b = self.mav.banner_bearing()
            if self._best_bearing is None or abs(b) < abs(self._best_bearing):
                self._best_bearing = b

        if self._elapsed() < self.dwell_s:
            self.feedback_message = (
                f"staring at step {self.step_index + 1}/{self.n_steps}, "
                f"{self._elapsed():.1f}/{self.dwell_s:.0f} s, "
                f"{self._hits}/{self._samples} frames identified")
            return py_trees.common.Status.RUNNING

        ratio = self._close_dwell()
        if self._confident(ratio):
            self._remember_vantage()
            self.mav.log(
                f"AlignToBanner: banner identified at "
                f"{math.degrees(self._wrap(self._target_yaw)):.0f} deg after "
                f"staring at {len(self.step_reports)} heading(s) "
                f"({self._hits}/{self._samples} frames)")
            self._enter(self.CENTRE)
            self._stable = 0
            self._last_seen = self.clock()
            self._align_target = self._target_yaw
            self._align_settled = False
            self._align_t0 = self.clock()
            self._corrections = 0
            self._stalled = 0
            self._last_bearing = None
            self._enter_square_state()
            return py_trees.common.Status.RUNNING

        if len(self.step_reports) >= self.n_steps:
            return self._give_up()

        self.step_index += 1
        self._target_yaw = self._wrap(self._sweep_origin
                                      + self.sweep_offsets[self.step_index])
        self._enter(self.SETTLE)
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
            # The chord of the arc that removes this much obliquity at this
            # radius. `d * tan(alpha)` is the tangent line and overshoots
            # badly past 30 degrees; the chord is what actually gets flown.
            chord = 2.0 * standoff * math.sin(min(abs(alpha), math.pi / 2.0)
                                              / 2.0)
            # A face whose perpendicular foot is to PORT means the aircraft is
            # standing off to starboard of the board's centreline, so it
            # travels to starboard-negative -- toward the foot.
            lat = -math.copysign(min(chord, self.strafe_step_m), alpha)
            ax, ay, az = self._anchor
            psi = self._align_target
            self._anchor = (ax + lat * math.cos(psi + math.pi / 2.0),
                            ay + lat * math.sin(psi + math.pi / 2.0), az)
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

        # 2. RANGE. Square to the face and centred on it; all that is left is
        #    standing at a distance the lidar and the camera both work at.
        radial = 0.0
        if standoff > self.max_standoff:
            radial = standoff - self.max_standoff
        elif standoff < self.min_standoff:
            radial = standoff - self.min_standoff
        if radial != 0.0:
            if self._sq_steps >= self.max_square_steps:
                return self._square_failure(
                    f"AlignToBanner: {self._sq_steps} steps did not bring the "
                    f"aircraft to a workable standoff; {self._measured()}")
            self._sq_steps += 1
            fwd = max(-self.strafe_step_m,
                      min(self.strafe_step_m, radial))
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
                f"lidar returns")
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
        if not seen_before and self._descents < self.max_descents \
                and room_below:
            self._descents += 1
            self._anchor = (ax, ay, az - self.descend_step_m)
            self._sq_phase = self.MOVING
            self._align_t0 = self.clock()
            self._lower_camera()
            self.mav.log(
                f"AlignToBanner: the lidar sees no surface from {az:.1f} m "
                f"({fit['reason']}); descending to "
                f"{az - self.descend_step_m:.1f} m to bring the banner into "
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
                 timeout_ticks=120, hold_alt=None):
        super().__init__(name)
        self.mav = mav
        self.tol = tol
        self.gain = gain
        self.max_step = max_step
        self.timeout = timeout_ticks
        self.hold_alt = hold_alt
        self._t = 0

    def initialise(self):
        self._t = 0

    def update(self):
        self._t += 1

        if not self.mav.qr_visible():
            self.feedback_message = "no marker visible"
            if self._t > self.timeout:
                reason = f"{self.name}: no marker to centre on"
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self.mav.qr_centred(self.tol):
            self.feedback_message = (
                f"centred ({self.mav.qr_offset.x:+.2f},"
                f"{self.mav.qr_offset.y:+.2f})")
            return py_trees.common.Status.SUCCESS

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
                 dwell_ticks=40):
        super().__init__("FindStartQR")
        self.mav = mav
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


class Goto(py_trees.behaviour.Behaviour):
    def __init__(self, name, mav, x, y, z, yaw=0.0, tol=0.6,
                 clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__(name); self.mav = mav
        self.x, self.y, self.z, self.yaw, self.tol = x, y, z, yaw, tol
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)

    def initialise(self):
        self.router.reset()

    def update(self):
        status = self.router.fly(self.mav, self.x, self.y, self.z, self.yaw)
        if status is BLOCKED:
            return _blocked(self, self.router)
        return (py_trees.common.Status.SUCCESS if status is ARRIVED
                else py_trees.common.Status.RUNNING)


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


class ApproachBanner(py_trees.behaviour.Behaviour):
    """Fly TO the corridor mouth the banner marks, not merely face it.

    WHAT THIS FIXES (geometry audit A5, second half)

        corridor_entry = (5.0, 0.0, 3.0) was replaced by AlignToBanner, which
        yaws until the banner is centred. That gets the heading right and the
        POSITION wrong: the simulated gate stands at y = +2, so an aircraft
        that aligned at y = 0 and then handed over to the corridor navigator
        flew forward past the gate and wedged itself in a corner at
        (4.9, -3.3) with 0.19 m of clearance ahead.

        Aligning is not entering. The deleted waypoint was doing two jobs and
        only one of them was replaced.

    Approach is translation along the banner's own bearing — no arena
    coordinate involved.

    WHAT COUNTS AS ARRIVING

        The navigator's `corridor_entered` was tried first and is too eager
        here: the gate's own left post stands about a metre off the takeoff
        point, so "walls on both sides" is true before the aircraft has moved
        at all, and the approach handed over instantly.

        The gate is a thing you go THROUGH. So the success condition is
        exactly that: the banner was tracked, centred, and then left the
        frame — which is what passing under it looks like from a
        forward-facing camera. Momentary occlusion cannot fake it, because it
        requires a sustained lock beforehand and a centred last bearing.
    """

    def __init__(self, mav, alt=3.0, step_m=1.5, hfov_rad=1.0472,
                 timeout_ticks=200, lost_grace=25, min_lock_ticks=8,
                 passed_ticks=5, centred_tol=0.35, arrive_tol=0.6,
                 clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__("ApproachBanner")
        self.mav = mav
        self.clearance_m = float(clearance_m)
        self.alt = alt
        self.step_m = step_m
        self.hfov = hfov_rad
        self.timeout_ticks = timeout_ticks
        self.lost_grace = lost_grace
        self.min_lock_ticks = min_lock_ticks
        self.passed_ticks = passed_ticks
        self.centred_tol = centred_tol
        self.arrive_tol = arrive_tol
        self._target = None
        self._t = 0
        self._lost = 0
        self._locked = 0
        self._gone = 0
        self._last_bearing = 0.0
        self._last_elev = 0.0

    def initialise(self):
        self._t = 0
        self._lost = 0
        self._locked = 0
        self._gone = 0
        self._last_bearing = 0.0
        self._last_elev = 0.0
        self._target = None

    def update(self):
        self._t += 1

        # Went through the gate: locked on, centred, then it left the frame.
        if (self._gone >= self.passed_ticks
                and self._locked >= self.min_lock_ticks
                and abs(self._last_bearing) <= self.centred_tol):
            self.feedback_message = "passed through the gate; handing over"
            return py_trees.common.Status.SUCCESS

        if self._t > self.timeout_ticks:
            reason = (f"ApproachBanner: never reached the corridor mouth in "
                      f"{self._t} ticks")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        x, y, z = self.mav.pos()
        psi = self.mav.yaw()

        if not self.mav.banner_identified():
            # Passing under the gate takes the banner out of frame, which is
            # progress, not failure -- so hold the last heading briefly rather
            # than aborting the moment it disappears.
            self._lost += 1
            self._gone += 1
            if self._lost > self.lost_grace:
                # Record HOW it was lost. "banner lost" alone cannot say
                # whether the aircraft flew under the gate (banner exits the
                # top of the frame -- success) or drifted off it (lost
                # sideways or mid-frame -- failure), and those need opposite
                # responses. Arena regression seed 1001 failed here twice
                # without saying which.
                reason = (f"ApproachBanner: banner lost before reaching the "
                          f"mouth (last seen bearing {self._last_bearing:+.2f}, "
                          f"elevation {self._last_elev:+.2f} "
                          f"[-1=top of frame], locked {self._locked} ticks, "
                          f"needed {self.min_lock_ticks})")
                self.feedback_message = reason
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE
            target_yaw = psi
        else:
            self._lost = 0
            self._gone = 0
            self._locked += 1
            self._last_bearing = self.mav.banner_bearing()
            self._last_elev = (self.mav.banner_elevation()
                               if hasattr(self.mav, "banner_elevation")
                               else 0.0)
            # Image +x is to the right of frame; a right-hand bearing is a
            # NEGATIVE yaw change in ENU. Same sign convention AlignToBanner
            # uses and its tests assert.
            theta = self._last_bearing * (self.hfov / 2.0)
            target_yaw = math.atan2(math.sin(psi - theta), math.cos(psi - theta))

        # A WAYPOINT, re-issued until reached -- not a receding carrot.
        #
        # This used to command `current_position + step_m` on EVERY tick. The
        # target therefore moved away exactly as fast as the aircraft chased
        # it, so the position controller saw a permanent 1.5 m error and
        # accelerated continuously. On the shipped arena the gate is 2.8 m
        # away and the transit ends before that matters. On a randomised one
        # it does not:
        #
        #   arena 1002: ABORTED_RTL  Excessive attitude (roll=-3.5 pitch=50.9)
        #   arena 1001: flew to the gate and sank 3.2 m -> 0.3 m
        #
        # A multirotor held at 50 degrees of pitch has lost most of its
        # vertical thrust component, so the runaway and the sink are the same
        # event. Latching the target lets the aircraft decelerate into it.
        # Arrival is judged HORIZONTALLY. mav.reached() also checks altitude,
        # which would be a trap here: if the aircraft were sagging below the
        # corridor band it would never "arrive", the target would never
        # advance, and the approach would stall until it timed out -- turning
        # an altitude problem into a navigation one.
        arrived = self._target is not None and math.hypot(
            self._target[0] - x, self._target[1] - y) <= self.arrive_tol
        if self._target is None or arrived:
            self._target = (x + self.step_m * math.cos(target_yaw),
                            y + self.step_m * math.sin(target_yaw))
        # This stage cannot route: it is a visual servo, and a detour would
        # take the banner out of frame and end the very lock it depends on.
        # So it checks instead. Flying at a gate with a restricted zone in
        # front of it is a violation whether or not the gate is real.
        held = list(getattr(self.mav, "exclusions", None) or [])
        if held and leg_hits_exclusion((x, y), self._target,
                                       self.clearance_m, held):
            reason = (f"ApproachBanner: the next step toward the mouth "
                      f"({self._target[0]:.1f}, {self._target[1]:.1f}) crosses "
                      f"confirmed red ground ({len(held)} zone(s)); a visual "
                      f"approach cannot route around it")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        self.mav.goto(self._target[0], self._target[1], self.alt, target_yaw)
        self.feedback_message = (
            f"approaching mouth from ({x:.1f}, {y:.1f}) toward "
            f"({self._target[0]:.1f}, {self._target[1]:.1f}) "
            f"heading {target_yaw:+.2f}")
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
                 timeout_ticks=600, clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__("GateAdvance")
        self.mav = mav
        self.advance_m = float(advance_m)
        self.alt = alt
        self.tol = float(tol)
        self.timeout_ticks = int(timeout_ticks)
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)
        self._target = None
        self._t = 0

    def initialise(self):
        self._target = None
        self._t = 0
        self.router.reset()

    def terminate(self, new_status):
        self.mav.enable_avoidance(False)

    def update(self):
        self._t += 1
        x, y, z = self.mav.pos()

        if self._target is None:
            psi = self.mav.yaw()
            self._target = (x + self.advance_m * math.cos(psi),
                            y + self.advance_m * math.sin(psi))
            self.mav.enable_avoidance(True, hold_alt=self.alt)
            self.mav.log(
                f"gate identified and aligned; advancing {self.advance_m:.0f} m "
                f"through it to ({self._target[0]:.1f}, {self._target[1]:.1f}) "
                f"at {math.degrees(psi):+.0f} deg, avoidance on")

        if math.hypot(self._target[0] - x, self._target[1] - y) <= self.tol:
            self.mav.enable_avoidance(False)
            self.mav.record_corridor_exit()
            self.feedback_message = f"advanced {self.advance_m:.0f} m past the gate"
            return py_trees.common.Status.SUCCESS

        if self.mav.avoidance_stuck():
            reason = (f"GateAdvance: the navigator could not find a way "
                      f"through the gate after "
                      f"{math.hypot(x - self._target[0], y - self._target[1]):.1f} m "
                      f"remaining")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        if self._t > self.timeout_ticks:
            reason = (f"GateAdvance: {self.advance_m:.0f} m not covered in "
                      f"{self._t} ticks")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        status = self.router.fly(self.mav, self._target[0], self._target[1],
                                 self.alt, self.mav.yaw())
        if status is BLOCKED:
            return _blocked(self, self.router)
        self.feedback_message = (
            f"advancing through the gate, "
            f"{math.hypot(self._target[0] - x, self._target[1] - y):.1f} m to go")
        return py_trees.common.Status.RUNNING


class UploadFence(py_trees.behaviour.Behaviour):
    """Upload the geofence and PROVE it arrived intact.

    The red-zone routing in LawnmowerSearch is a plan. The fence is what holds
    when the plan is wrong: perception mis-detects, the aircraft drifts, a gust
    pushes it, an operator takes control. goal.md Q13/Q24 make the fence the
    authoritative boundary and demote the camera layer to supplementary.

    So the upload is not fire-and-forget. It is pushed, read back off
    /mavros/geofence/fences, and compared vertex by vertex in the local frame
    (mission_bt.geofence.compare_fences). An acknowledged-but-wrong fence is
    the failure mode worth engineering against: it is believed.

    NON-FATAL BY DESIGN. A missing fence service must not ground a mission
    that is otherwise flyable — the outcome is recorded on the aircraft and
    surfaced to the GCS rather than silently assumed good.
    """

    def __init__(self, mav, margin_m=5.0, timeout_ticks=100, required=False,
                 search_budget_m=0.0):
        super().__init__("UploadFence")
        self.mav = mav
        self.margin_m = margin_m
        self.timeout_ticks = timeout_ticks
        self.required = required
        # The fence must contain the ground the search may REACH, not just the
        # first window the lidar happened to see. LawnmowerSearch pushes its
        # frontier up to search_budget_m beyond that window; a fence drawn
        # round the window alone would breach on the first expansion.
        self.search_budget_m = float(search_budget_m)
        self._t = 0
        self._sent = None
        self._future = None

    def initialise(self):
        self._t = 0
        self._sent = None
        self._future = None
        self.mav.fence_verified = False
        self.mav.fence_reason = "not uploaded"

    def _give_up(self, reason):
        self.mav.fence_reason = reason
        self.feedback_message = reason
        if self.required:
            self.mav.abort_reason = f"UploadFence: {reason}"
            return py_trees.common.Status.FAILURE
        self.mav.log(f"fence not verified: {reason}", warn=True)
        return py_trees.common.Status.SUCCESS

    def update(self):
        self._t += 1
        home = self.mav.home_global()
        zone = self.mav.observed_zone

        if home is None or zone is None:
            if self._t < self.timeout_ticks:
                self.feedback_message = "waiting for home position and zone"
                return py_trees.common.Status.RUNNING
            return self._give_up("no home position or observed zone to fence")

        if self._sent is None:
            from mavros_msgs.msg import Waypoint
            envelope = zone
            if self.search_budget_m > 0.0:
                exit_pose = self.mav.corridor_exit_pose
                heading = exit_pose[3] if exit_pose else self.mav.yaw()
                envelope = extend_zone(zone, heading, self.search_budget_m)
            self._sent = build_fence(inclusion_from_zone(envelope, self.margin_m),
                                     self.mav.exclusions, home[0], home[1],
                                     Waypoint)
            self._future = self.mav.push_fence(self._sent)
            if self._future is None:
                return self._give_up("geofence push service unavailable")
            self.feedback_message = f"pushing {len(self._sent)} fence items"
            return py_trees.common.Status.RUNNING

        got = self.mav.fence_readback
        if got is None:
            if self._t < self.timeout_ticks:
                self.feedback_message = "awaiting fence read-back"
                return py_trees.common.Status.RUNNING
            return self._give_up("fence never read back")

        ok, why = compare_fences(self._sent, got, home[0], home[1])
        self.mav.fence_verified = ok
        self.mav.fence_reason = why or "verified against read-back"
        if not ok:
            return self._give_up(why)
        self.feedback_message = (f"fence verified: {len(self._sent)} items, "
                                 f"{len(self.mav.exclusions)} exclusion(s)")
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


class ObserveZone(py_trees.behaviour.Behaviour):
    """Bound the delivery zone from what the lidar sees, not from a constant.

    WHAT THIS REPLACES (geometry audit A7, A8)

        zone_entry  = (18.0, 0.0, 3.0)
        zone_bounds = (20.0, 52.0, -12.0, 12.0)

    Both asserted where the delivery zone was and how big it was. They are
    true of exactly one arena — the one they were measured in — and the
    mission is supposed to be perception-driven.

    At the moment the corridor opens out, the aircraft knows its own position
    and heading, and the navigator has just measured how far the open area
    extends ahead and how wide it is. That is a zone bound, observed.

    Fails closed on an implausible observation rather than sweeping it: a lidar
    that sees nothing returns max range everywhere, which would otherwise plan
    a sweep that never finishes.
    """

    def __init__(self, mav, margin_m=1.0, settle_ticks=8):
        super().__init__("ObserveZone")
        self.mav = mav
        self.margin_m = margin_m
        self.settle_ticks = settle_ticks
        self.zone = None
        self._t = 0

    def initialise(self):
        self._t = 0
        self.zone = None

    def update(self):
        self._t += 1
        exit_pose = self.mav.corridor_exit_pose
        if exit_pose is None:
            # Not through the corridor yet -> nothing observed. Anchor on the
            # aircraft itself so a directly-started search still has a frame.
            x, y, _ = self.mav.pos()
            yaw = self.mav.yaw()
        else:
            x, y, _, yaw = exit_pose

        depth, width = self.mav.open_extent()
        if depth <= 0.0 or width <= 0.0:
            if self._t < self.settle_ticks:
                self.feedback_message = "waiting for a lidar view of the zone"
                return py_trees.common.Status.RUNNING
            reason = "ObserveZone: no lidar extent to bound the delivery zone"
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE

        zone = zone_from_observation((x, y), yaw, depth, width,
                                     margin_m=self.margin_m)
        ok, why = zone_is_plausible(zone)
        if not ok:
            if self._t < self.settle_ticks:
                self.feedback_message = f"zone not yet plausible: {why}"
                return py_trees.common.Status.RUNNING
            self.feedback_message = f"ObserveZone: {why}"
            self.mav.abort_reason = f"ObserveZone: {why}"
            return py_trees.common.Status.FAILURE

        self.zone = zone
        self.mav.observed_zone = zone
        self.feedback_message = (
            f"zone observed: x {zone[0]:.1f}..{zone[1]:.1f}, "
            f"y {zone[2]:.1f}..{zone[3]:.1f} "
            f"(depth {depth:.1f} m, width {width:.1f} m)")
        self.mav.log(self.feedback_message)
        return py_trees.common.Status.SUCCESS


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
                 hover_s=5.0, clock=None):
        """`zone` may be an (x0, x1, y0, y1) tuple or a CALLABLE returning one.

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
        self._plan_args = dict(image_width_px=image_width_px,
                               hfov_rad=hfov_rad, marker_m=marker_m,
                               modules=modules, px_floor=px_floor,
                               overlap=overlap, max_alt=alt)
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
        self.max_replans_per_strip = 5
        self.expansions = 0
        self.swept_zone = None      # union of every strip actually swept
        # Forward first -- the zone usually opens out along the corridor --
        # then either side. See _advance_frontier().
        self.EXPANSION_DIRECTIONS = (0.0, math.pi / 2, -math.pi / 2)
        if not callable(zone):
            self._replan(zone, self._current_exclusions())

    def _replan(self, zone, exclusions=(), extend_swept=False):
        a = self._plan_args
        self.plan = plan_search(zone, a['image_width_px'], a['hfov_rad'],
                                a['marker_m'], a['modules'],
                                px_per_module_floor=a['px_floor'],
                                overlap=a['overlap'],
                                max_altitude=a['max_alt'])
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
                zone, self.plan["lane_spacing_m"], self.alt, a['hfov_rad'],
                exclusions=self.exclusions, clearance_m=self.clearance_m)
            self.plan["coverage"] = coverage_fraction_excluding(
                zone, self.plan["lane_spacing_m"], self.alt, a['hfov_rad'],
                exclusions=self.exclusions, clearance_m=self.clearance_m)
            self.plan["n_lanes"] = len(wps) // 2
        else:
            wps = self.plan["waypoints"]
        self.wps = [(w[0], w[1]) for w in wps]

    def _current_exclusions(self):
        src = self._exclusions_src
        if src is None:
            return []
        return list(src() if callable(src) else src)

    def initialise(self):
        self.i = 0
        self.skipped = 0
        self.router.reset()
        self.hover.reset()
        self._budget_left = self.search_budget_m
        self.expansions = 0
        self._replans = 0
        # The frontier advances the way the aircraft came out of the corridor.
        exit_pose = self.mav.corridor_exit_pose
        self._heading = exit_pose[3] if exit_pose else self.mav.yaw()
        if callable(self._zone_src):
            zone = self._zone_src()
            if zone is None:
                return                      # update() reports the failure
            self._replan(zone, self._current_exclusions())
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
            f"{self.plan['n_lanes']} lanes, coverage "
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

    def _exclusions_changed(self):
        """A red zone confirmed mid-strip must be avoided by THIS strip.

        Exclusions used to be sampled once, before the sweep started, from the
        corridor exit -- where none of the red zones are visible. Anything the
        georeferencer confirmed while sweeping was recorded and then ignored.
        """
        # The georeferencer confirms red ground cell by cell, so the exclusion
        # count climbs steadily while the aircraft sweeps (0 -> 5 -> 26 -> 32
        # in live run 13). Each re-plan restarts the strip, so an unbounded
        # re-plan would let a growing exclusion set stall the sweep forever.
        # Bounded here; the geofence remains the backstop either way.
        if self._replans >= self.max_replans_per_strip:
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

    def update(self):
        if self.plan is None:
            reason = "LawnmowerSearch: no observed zone to sweep"
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE
        if self.mav.qr_matched:
            return py_trees.common.Status.SUCCESS
        if self.hover.tick(self.mav):
            self.feedback_message = (f"holding over '{self.hover.payload}' "
                                     f"before resuming the sweep")
            return py_trees.common.Status.RUNNING
        if self._exclusions_changed():
            n_before = len(self.exclusions)
            self._replan(self._frontier, self._current_exclusions(),
                         extend_swept=True)
            self.i = 0
            self.router.reset()
            self._replans += 1
            self.mav.log(
                f"red zone confirmed mid-sweep ({n_before} -> "
                f"{len(self.exclusions)}): re-planned the current strip, "
                f"{self.plan['n_lanes']} lanes, coverage "
                f"{self.plan['coverage'] * 100:.0f}%")
            return py_trees.common.Status.RUNNING
        if self.i >= len(self.wps):
            if self._advance_frontier():
                return py_trees.common.Status.RUNNING
            swept = [round(v, 1) for v in (self.swept_zone or self._zone())]
            reason = (f"swept {swept} at {self.alt:.1f} m over "
                      f"{self.expansions + 1} strip(s) without matching the "
                      f"target; search budget exhausted "
                      f"({self._budget_left:.0f} m left, below the "
                      f"{self.min_step_m:.0f} m minimum step)")
            self.feedback_message = reason
            self.mav.abort_reason = reason
            return py_trees.common.Status.FAILURE     # swept all, no match
        wx, wy = self.wps[self.i]
        status = self.router.fly(self.mav, wx, wy, self.alt)
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
        status = self.router.fly(self.mav, cx, cy, self._target_z)
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
                 clearance_m=DEFAULT_CLEARANCE_M):
        super().__init__("ReturnToCorridorMouth")
        self.mav = mav
        self.alt = alt
        self.tol = tol
        self.standoff_m = standoff_m
        self.camera_pitch = float(camera_pitch_rad)
        self.banner_centre_m = float(banner_centre_m)
        # The whole point of this leg is to cross the delivery zone, which is
        # where the red zones are and where they have by now been confirmed.
        # Of every leg in the mission this is the one most likely to be routed.
        self.router = LegRouter(clearance_m=clearance_m, tol=tol)

    def initialise(self):
        self.router.reset()

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
        drop = max(0.5, float(self.alt) - self.banner_centre_m)
        return drop / math.tan(self.camera_pitch)

    def update(self):
        pose = self.mav.corridor_exit_pose
        if pose is None:
            hx, hy = self.mav.home_local_xy()
            tx, ty, tyaw = hx, hy, self.mav.yaw()
            self.feedback_message = "no recorded corridor exit; heading home"
        else:
            mx, my, _, exit_yaw = pose
            tyaw = math.atan2(math.sin(exit_yaw + math.pi),
                              math.cos(exit_yaw + math.pi))   # wrapped reverse
            # Stop SHORT of the mouth, on the delivery-zone side, so the
            # banner is in front of the camera rather than beneath it.
            back = self.standoff()
            tx = mx - back * math.cos(tyaw)
            ty = my - back * math.sin(tyaw)
            self.feedback_message = (f"returning to {back:.1f} m short of the "
                                     f"corridor mouth ({tx:.1f}, {ty:.1f}) "
                                     f"yaw {tyaw:.2f}")
        status = self.router.fly(self.mav, tx, ty, self.alt, tyaw)
        if status is BLOCKED:
            return _blocked(self, self.router)
        return (py_trees.common.Status.SUCCESS if status is ARRIVED
                else py_trees.common.Status.RUNNING)


class WinchDrop(py_trees.behaviour.Behaviour):
    """Descend to 5 m over the matched QR, lower + release, climb back."""
    def __init__(self, mav, drop_alt=5.0, cruise_alt=10.0,
                 lower_timeout=200, release_timeout=100,
                 hfov_rad=1.0472, image_w_px=1280, image_h_px=720,
                 marker_m=2.2, max_offset_age_ticks=40):
        super().__init__("WinchDrop"); self.mav = mav
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
        self.drop_x = 0.0; self.drop_y = 0.0
        self.phase = 0; self._t = 0
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
        self._last_offset = None
        floor = self.tracking_floor()
        if self.drop_alt < floor:
            self.mav.log(
                f"drop altitude {self.drop_alt:.2f} m is below the "
                f"{floor:.2f} m tracking floor for a {self.marker_m:.1f} m "
                f"pad; raising it, because below the floor the delivery "
                f"offset cannot be measured at all", warn=True)
            self.drop_alt = floor
        # Latch current horizontal coordinates to prevent drift during descent
        self.drop_x, self.drop_y = self.mav.pos()[:2]

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
        self.mav.delivery_note = (
            f"released {self.mav.delivery_offset_m:.2f} m from pad centre "
            f"at {at_alt:.2f} m (image offset {ox:+.2f}, {oy:+.2f}{when})")
        self.mav.log(f"delivery accuracy: {self.mav.delivery_note}")

    def update(self):
        self._t += 1
        w = self.mav.winch_status
        # Sample all the way down. Reading the offset only on the tick the
        # release fires throws away every good frame that came before it.
        self.observe_offset()

        if self.phase == 0:                            # descend
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt)
            if self.mav.reached(self.drop_x, self.drop_y, self.drop_alt, 0.5):
                self.phase = 1
                self._t = 0

        elif self.phase == 1:                          # lower until DOWN
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt)
            self.mav.winch("lower")
            self.feedback_message = (f"lowering: payout="
                                     f"{w.get('payout_m', 0.0)} "
                                     f"state={w.get('state', '?')}")
            # Wait for the winch to REPORT the payload down, not for a timer.
            # The old code released after a fixed 20 ticks whether or not
            # anything had moved — and nothing was subscribed to move.
            if w.get("state") in ("AT_GROUND",) or w.get("ground"):
                self.phase = 2
                self._t = 0
            elif self._t > self.lower_timeout:
                reason = (f"winch did not reach the ground in {self._t} ticks "
                          f"(status={w or 'no /winch/status'})")
                self.feedback_message = reason
                self.mav.abort_reason = reason
                return py_trees.common.Status.FAILURE

        elif self.phase == 2:                          # release, gated
            self.mav.goto(self.drop_x, self.drop_y, self.drop_alt)
            if w.get("released"):
                self._record_delivery_offset()
                self.phase = 3
                self._t = 0
            elif w.get("release_ok"):
                self.mav.winch("release")
            else:
                self.feedback_message = ("waiting on release interlocks: "
                                         + "; ".join(w.get("blockers", [])))
                if self._t > self.release_timeout:
                    reason = ("payload release never permitted: "
                              + "; ".join(w.get("blockers", ["no status"])))
                    self.mav.abort_reason = reason
                    return py_trees.common.Status.FAILURE

        elif self.phase == 3:                          # stow + climb back
            self.mav.winch("stow")
            self.mav.goto(self.drop_x, self.drop_y, self.cruise_alt)
            if self.mav.reached(self.drop_x, self.drop_y, self.cruise_alt, 0.6):
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


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
        if self.mav.state.armed:
            return py_trees.common.Status.RUNNING
        precision = getattr(self.mav, "landing_precision", "UNKNOWN")
        # Delivery accuracy is 15 rulebook marks; landing is 5. Reporting the
        # tracking-quality figure and staying silent about the drop put the
        # smaller number in front of the operator and hid the larger one.
        drop = getattr(self.mav, "delivery_offset_m", None)
        drop_txt = ("delivery UNMEASURED" if drop is None
                    else f"delivery {drop:.2f} m from pad centre")
        self.mav.publish_result(
            "COMPLETED",
            f"landed and disarmed; {drop_txt}; landing {precision}")
        return py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------- #
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
    search = LawnmowerSearch(mav, lambda: mav.observed_zone, p['search_alt'],
                             exclusions=lambda: mav.exclusions,
                             clearance_m=p.get('redzone_clearance', 1.5),
                             hover_s=p.get('qr_hover_s', 5.0),
                             image_width_px=p.get('image_width_px', 1280),
                             hfov_rad=p.get('camera_hfov', 1.0472),
                             marker_m=p.get('target_marker_m', 2.2),
                             px_floor=p.get('px_per_module_floor', 5.3),
                             overlap=p.get('lane_overlap', 0.30),
                             search_budget_m=p.get('search_budget_m', 45.0))

    mission = py_trees.composites.Sequence(name="Mission", memory=True)
    mission.add_children([
        WaitForMissionStart(mav),
        SetModeArm(mav),
        Takeoff(mav, p['takeoff_alt']),
        SetCameraPose("CameraNadirForQR", mav, "NADIR"),
        # Phase 3: find the marker, centre on it, THEN decode. Replaces a
        # hardcoded hover over a guessed scan_pose (geometry audit A4).
        FindStartQR(mav, p['takeoff_alt'], floor_alt=p.get('scan_floor_alt', 2.0)),
        CenterOnQR("CenterStartQR", mav),
        ScanStartQR(mav, hover_s=p.get('qr_hover_s', 5.0)),
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
        AlignToBanner(mav,
                      sweep_limit_rad=p.get('banner_sweep_limit', 2 * math.pi),
                      step_rad=p.get('banner_sweep_step', math.radians(30.0)),
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
        # Aligning to the banner is not the same as arriving at it: the gate
        # stands off the takeoff axis, and an aligned-but-not-approached
        # aircraft flew straight past it into a corner (audit A5).
        # A MEASURED DISTANCE THROUGH THE GATE, not "until the banner leaves
        # the frame". The old end condition treated getting close to the board
        # as having passed it, so the mission entered the delivery-zone stages
        # while still at the mouth.
        GateAdvance(mav, advance_m=p.get('gate_advance_m', 10.0),
                    alt=p['corridor_alt'],
                    clearance_m=p.get('redzone_clearance', 1.5)),
        Corridor("Corridor", mav, forward=True, alt=p['corridor_alt']),
        # Phase 6: the delivery zone is MEASURED at the corridor mouth, not
        # asserted. Replaces zone_entry + zone_bounds (audit A7, A8).
        ObserveZone(mav, margin_m=p.get('zone_margin', 1.0)),
        UploadFence(mav, margin_m=p.get('fence_margin', 5.0),
                    search_budget_m=p.get('search_budget_m', 45.0)),
        ClimbInPlace("ClimbToSweep", mav, lambda: search.alt),
        SetCameraPose("CameraNadirForSearch", mav, "NADIR"),
        search,
        # Sweep high to FIND, descend to READ: the sweep altitude is set by
        # pad detectability, which is well above the measured decode envelope.
        DescendToDecode(mav, decode_alt=lambda: search.decode_alt,
                        floor_alt=p.get('scan_floor_alt', 2.0),
                        clearance_m=p.get('redzone_clearance', 1.5),
                        exclusions=lambda: mav.exclusions),
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
        SetCameraPose("CameraBannerReturn", mav, "BANNER"),
        # Back to where the corridor actually opened out (audit A9) -- at
        # transit altitude, not corridor altitude.
        ReturnToCorridorMouth(mav, alt=p['search_alt'],
                              clearance_m=p.get('redzone_clearance', 1.5)),
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
        AlignToBanner(mav,
                      sweep_limit_rad=p.get('banner_sweep_limit', 2 * math.pi),
                      step_rad=p.get('banner_sweep_step', math.radians(30.0)),
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
        GateAdvance(mav, advance_m=p.get('gate_advance_m', 10.0),
                    alt=p['corridor_alt'],
                    clearance_m=p.get('redzone_clearance', 1.5)),
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
    d('search_budget_m', 45.0)
    d('redzone_clearance', 1.5)
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
    d('banner_sweep_step', math.radians(30.0))
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
