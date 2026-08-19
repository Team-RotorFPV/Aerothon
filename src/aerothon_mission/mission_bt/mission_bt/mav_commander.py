#!/usr/bin/env python3
"""MAVROS commander — thin wrapper the behaviour tree calls into.

Holds the MAVROS clients/pubs/subs and a 10 Hz setpoint streamer so the
py_trees leaves stay simple (they poll state + issue intents; this object
does the ROS work). ArduPilot GUIDED flow: set GUIDED -> arm -> takeoff ->
stream position setpoints; hand the corridor to the avoidance node.

Phase 0 fail-closed rails live here:
  * setpoints are never published while the aircraft is disarmed
  * external LAND / DISARM / mode changes raise a mission-reset request
  * excessive attitude is measured and exposed to the abort guard
  * mission outcome is latched on /mission/result with an explicit reason
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32, String
from mavros_msgs.msg import HomePosition, State, WaypointList
from mavros_msgs.srv import WaypointPush as _WaypointPush
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from mission_bt.scan_geometry import fit_surface, no_surface as _no_surface


# Modes the aircraft may legitimately enter under our own command. A change
# into any other mode, or into one of these without us asking, is treated as
# an outside intervention (safety pilot, Mission Planner, failsafe).
_TERMINAL_MODES = ("LAND", "RTL")

# How long after we issue a mode command we still consider that mode "ours".
_COMMAND_OWNERSHIP_S = 5.0


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Transient-local QoS so a late-joining GCS still sees the last result."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Mav:
    def __init__(self, node: Node):
        self.node = node
        self.state = State()
        self.battery = BatteryState()
        self.pose = PoseStamped()
        self.qr_decoded = ""
        self.qr_matched = False
        # Consecutive-frame confidence. A single decode is not a decision: one
        # frame of motion blur or a reflection can produce a spurious read, and
        # the mission commits to a delivery target on the strength of it.
        self._qr_streak = 0
        self._qr_last = ""
        # Operator-supplied target (goal.md Q19): if the start QR is physically
        # unreadable the mission can still run, but the substitution is
        # explicit and logged rather than a silent empty string.
        self.target_override = ""
        # Latched terminal failure of the mission sequence, distinct from an
        # abort. Cleared only by an explicit new START.
        self.mission_failed = False
        # Latched terminal SUCCESS. Also cleared only by an explicit new START:
        # a memory Sequence that reaches its last child returns SUCCESS and is
        # re-initialised on the next tick, which re-armed and took off again
        # 0.4 s after a successful landing.
        self.mission_complete = False
        # How the aircraft got down: PRECISE (visual lock held to the commit
        # altitude) or DEGRADED with the reason. Reported in the outcome
        # rather than left implicit -- "landed" alone does not say whether it
        # landed on the pad.
        self.landing_precision = "UNKNOWN"
        # Once airborne, dropping below this altitude means something is wrong;
        # set on takeoff, cleared on landing. See low_altitude_fault().
        self.airborne_floor = None
        self._low_alt_samples = 0
        self.qr_offset = Vector3()
        self.banner = Vector3()
        self.banner_reject_reason = ""
        self.banner_reject_counts = {}
        self.banner_board_aspect = 0.0
        self.banner_board_area = 0.0
        self.abort_requested = False
        self.abort_reason = ""
        # Once an abort fires it must STAY fired. The guard condition is
        # level-triggered (attitude recovers, battery sag recovers, the FCU
        # reconnects), so without a latch the mission silently resumed mid-air
        # after an abort had already commanded RTL. Cleared only by an explicit
        # new START or a mission reset.
        self.abort_latched = False
        self.mission_started = False

        # Default matches the official Iris SITL 3S battery. Override this ROS
        # parameter for a real airframe's battery chemistry / cell count.
        self.critical_battery_voltage = node.declare_parameter(
            'critical_battery_voltage', 10.5).value
        # Beyond this roll/pitch the aircraft is not flying the mission any
        # more, it is falling into something. The last recorded live run sat at
        # 54 deg against a corridor wall and nothing in the stack objected.
        self.attitude_limit_deg = node.declare_parameter(
            'attitude_limit_deg', 45.0).value
        # Consecutive pose samples beyond the limit before the guard trips, so
        # a single noisy quaternion cannot abort a healthy flight.
        self.attitude_limit_samples = node.declare_parameter(
            'attitude_limit_samples', 5).value

        self._sp = None            # streamed PoseStamped target (local ENU)
        # Altitude-divergence trace -- see _track_altitude_error().
        self.active_stage = ""
        self.alt_error_worst = 0.0
        self.alt_error_worst_stage = ""
        self.alt_error_warn = 0.8
        self._alt_warned_stage = None
        self._alt_target = None
        self._alt_arrived = False

        # ---- fail-closed bookkeeping ---- #
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self._attitude_violations = 0
        self.setpoints_suppressed = 0      # counts gated publishes (testable)
        self.setpoint_block_reason = ""
        self._reset_requested = False
        self._reset_reason = ""
        self._expect_disarm = False        # set by the Land leaf
        self._commanded_modes = {}         # mode -> monotonic timestamp
        self._result = None                # latched (state, reason)

        qos = 10
        node.create_subscription(State, '/mavros/state', self._on_state, qos)
        node.create_subscription(BatteryState, '/mavros/battery', self._on_battery,
                                 qos_profile_sensor_data)
        node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        node.create_subscription(String, '/percep/qr/decoded', self._on_qr, qos)
        node.create_subscription(Bool, '/percep/qr/matched', self._on_match, qos)
        node.create_subscription(Vector3, '/percep/qr/target_offset', self._on_off, qos)
        node.create_subscription(Vector3, '/percep/banner', self._on_banner, qos)
        # WHY the detail and not just the Vector3: when AlignToBanner gives up,
        # "green objects seen but rejected, or none" is all the aircraft could
        # say. The detector knows exactly WHY it rejected each candidate --
        # aspect, area, missing lettering -- and publishes it here, where
        # nothing was reading it. A failed sweep was undiagnosable from the run
        # artifacts (arena regression seed 1001).
        node.create_subscription(String, '/percep/banner/detail',
                                 self._on_banner_detail, qos)
        node.create_subscription(String, '/percep/redzone/detail',
                                 self._on_redzone, qos)
        # THE LIDAR, finally reaching the mission tree.
        #
        # /scan has been live and bridged since Phase 0, and the only things
        # reading it were the avoidance navigator and the GCS aggregator. The
        # mission itself had no way to ask "am I perpendicular to that
        # surface", so it inferred squareness from a camera bounding box that
        # includes the gate posts -- a proxy whose plateau (1.88 to 1.91) made
        # every threshold above it unreachable, and which failed fourteen
        # watched runs.
        self._scan = None
        self._scan_t = None
        node.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.pub_square_on = node.create_publisher(String, '/mission/square_on',
                                                   qos)
        node.create_subscription(Bool, '/mission/abort', self._on_abort, qos)
        node.create_subscription(Bool, '/mission/start', self._on_start, qos)
        node.create_subscription(String, '/mission/target_override',
                                 self._on_target_override, qos)
        # Home is the ONE legitimate global reference (audit A11). It was
        # assumed to be local (0,0) — true only because the EKF origin happens
        # to be set at the arming point. Read it from the FC instead.
        self.home = None
        node.create_subscription(HomePosition, '/mavros/home_position/home',
                                 self._on_home, qos)
        # Geofence: pushed, then READ BACK. An unverified fence is worse than
        # none because it is believed (goal.md Q13/Q24).
        self.fence_readback = None
        node.create_subscription(WaypointList, '/mavros/geofence/fences',
                                 self._on_fence_readback, _latched_qos(depth=1))
        self.cli_fence_push = node.create_client(_WaypointPush,
                                                 '/mavros/geofence/push')

        self.pub_sp = node.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)
        self.pub_enable = node.create_publisher(Bool, '/avoidance/enable', qos)
        self.pub_hold_alt = node.create_publisher(Float32, '/avoidance/hold_alt', qos)
        self.pub_target = node.create_publisher(String, '/mission/target', qos)
        self.pub_winch = node.create_publisher(String, '/winch/cmd', qos)
        self.pub_result = node.create_publisher(String, '/mission/result', _latched_qos())
        self.pub_camera_pose = node.create_publisher(String, '/camera/set_pose', 10)

        # Corridor navigator feedback: exit is DETECTED, not assumed from a
        # hardcoded x threshold (geometry audit A6/A10).
        self.avoid_detail = {}
        # Observed geometry, recorded in flight. These replace the asserted
        # zone_entry / zone_bounds / corridor_return_entry constants (audit
        # A7, A8, A9): everything here is measured during this mission.
        self.corridor_exit_pose = None      # (x, y, z, yaw) where walls fell away
        self.observed_zone = None           # (x0, x1, y0, y1) from the lidar
        # Red ground the camera has mapped, as local-frame rectangles. The
        # search planner routes lanes around these and the geofence encodes
        # them; the fence is the authoritative one (goal.md Q13/Q24).
        self.exclusions = []
        self.redzone_status = "NOT_VISIBLE"
        self.fence_verified = False
        self.fence_reason = "not uploaded"
        node.create_subscription(String, '/avoidance/detail',
                                 self._on_avoid_detail, qos)

        # Winch feedback. Until winch_ctrl existed this stayed empty and
        # WinchDrop released on a fixed timer instead.
        self.winch_status = {}
        node.create_subscription(String, '/winch/status',
                                 self._on_winch_status, qos)

        # Phase 2: camera orientation is measured state, not an assumption.
        self.camera_state = {}
        node.create_subscription(String, '/camera/pose_state',
                                 self._on_camera_state, qos)

        self.cli_arm = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = node.create_client(SetMode, '/mavros/set_mode')
        self.cli_takeoff = node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = node.create_client(CommandTOL, '/mavros/cmd/land')

        node.create_timer(0.1, self._stream)   # 10 Hz setpoint stream

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #
    def _on_battery(self, m): self.battery = m
    def _on_match(self, m): self.qr_matched = m.data

    def _on_qr(self, m):
        value = m.data
        if value and value == self._qr_last:
            self._qr_streak += 1
        elif value:
            self._qr_streak = 1
        else:
            self._qr_streak = 0
        self._qr_last = value
        self.qr_decoded = value

    def _on_home(self, m):
        self.home = m

    def home_local_xy(self):
        """Home in the LOCAL frame.

        MAVROS publishes home as a global fix plus a local offset. The local
        origin IS the EKF origin, so home is (0,0) in local coordinates
        whenever the EKF origin was set at the home point — which is the normal
        case and what the old constant silently relied on. Returning it through
        this accessor means the assumption is stated in one place and can be
        replaced with a real transform if it ever stops holding.
        """
        if self.home is None:
            return (0.0, 0.0)
        return (float(self.home.position.x), float(self.home.position.y))

    def _on_target_override(self, m):
        if m.data and m.data != self.target_override:
            self.node.get_logger().warning(
                f"Operator supplied target override: {m.data}")
        self.target_override = m.data

    def qr_confident(self, frames):
        """True only after the SAME payload has been read `frames` times."""
        return bool(self.qr_decoded) and self._qr_streak >= int(frames)

    @property
    def qr_streak(self):
        return self._qr_streak
    def _on_off(self, m): self.qr_offset = m

    def qr_visible(self):
        """Any marker in frame, whether or not it is the delivery target.

        qr_node encodes this in the offset's z: 1.0 matched target, 0.5 some
        marker, 0.0 nothing. Before Phase 3 the offset was only populated for a
        MATCHED target, so during the start scan — when no target is known yet —
        the mission had no way to tell a marker was sitting off to one side.
        """
        return self.qr_offset.z > 0.0

    def qr_centred(self, tol):
        """Marker within `tol` of frame centre, in normalised [-1,1] units."""
        if not self.qr_visible():
            return False
        return abs(self.qr_offset.x) <= tol and abs(self.qr_offset.y) <= tol
    def _on_banner(self, m): self.banner = m

    def _on_banner_detail(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        # How SQUARE the aircraft is to the board.
        #
        # A banner is widest seen face-on and compresses as you move off its
        # perpendicular. So the apparent aspect is a direct, measured answer
        # to "am I in front of it yet" -- which is the question that has to be
        # answered before committing to a waypoint through the gate. Watched
        # live, the aircraft set a 10 m waypoint while still off to one side
        # and flew away from the arena.
        if d.get('identified'):
            self.banner_board_aspect = float(d.get('board_aspect') or 0.0)
            self.banner_board_area = float(d.get('board_area_px') or 0.0)

        reason = (d.get('reason') or '').strip()
        if reason:
            self.banner_reject_reason = reason
            self.banner_reject_counts[reason] = \
                self.banner_reject_counts.get(reason, 0) + 1

    def banner_aspect(self):
        """Apparent width/height of the board. Peaks when square to it."""
        return float(getattr(self, "banner_board_aspect", 0.0))

    def banner_rejection_summary(self, top=3):
        """The commonest reasons candidates were rejected, most frequent first.

        Turns "no banner found" into "found green things and rejected them
        for THIS reason", which is the difference between a diagnosable
        failure and a shrug.
        """
        if not self.banner_reject_counts:
            return "nothing green ever entered the frame"
        ranked = sorted(self.banner_reject_counts.items(),
                        key=lambda kv: -kv[1])[:top]
        return "; ".join(f"{r} (x{n})" for r, n in ranked)

    # ------------------------------------------------------------------ #
    # lidar
    # ------------------------------------------------------------------ #
    def _on_scan(self, m):
        self._scan = m
        self._scan_t = self.node.get_clock().now().nanoseconds / 1e9

    def scan_age_s(self):
        """Seconds since the last scan, or None if none has ever arrived."""
        if self._scan_t is None:
            return None
        return (self.node.get_clock().now().nanoseconds / 1e9) - self._scan_t

    def surface_ahead(self, bearing_rad, half_width_rad,
                      expected_range_m=None, stale_s=1.0, **kw):
        """The flat face in that sector of the scan, or an explicit refusal.

        THE SEAM. The behaviour tree never touches a LaserScan: it asks for a
        sector and gets back the same dict `fit_surface` returns, so the
        squareness logic can be tested against a synthetic scan and the tree
        against a fake commander, and neither test is grading a copy of the
        other's arithmetic.

        A missing or stale scan is a REFUSAL, not a zero. An aircraft that
        cannot measure its angle to the banner must not advance through it,
        and a zero here reads as "perfectly square".
        """
        age = self.scan_age_s()
        if self._scan is None or age is None:
            return _no_surface("no lidar scan has arrived on /scan; the "
                               "aircraft cannot measure its angle to anything")
        if age > float(stale_s):
            return _no_surface(
                f"the last lidar scan is {age:.1f} s old; refusing to square "
                f"up on a stale measurement")
        scan = self._scan
        return fit_surface(scan.angle_min, scan.angle_increment, scan.ranges,
                           bearing_rad, half_width_rad,
                           range_min=float(scan.range_min),
                           range_max=float(scan.range_max),
                           expected_range_m=expected_range_m, **kw)

    def publish_square_on(self, payload):
        """Report the squareness measurement to the GCS while it converges.

        The operator watching a flight could see that the aircraft was moving
        and not what it believed about its angle to the banner, which is the
        one number the stage's decision turns on.
        """
        self.pub_square_on.publish(String(data=json.dumps(payload)))

    def banner_identified(self):
        """The banner, not merely something green.

        perception_banner encodes this in z: 1.0 identified banner, 0.5 a green
        object rejected as not-the-banner, 0.0 nothing green in view. Before
        Phase 4 any green rectangle produced 1.0 and the GCS said ALIGNED.
        """
        return self.banner.z >= 1.0

    def banner_elevation(self):
        """Where the banner sits VERTICALLY in frame, [-1, 1], -1 = top edge.

        perception_banner has always published this as `banner.y` and nothing
        read it. It is the signal that distinguishes the two ways of losing
        the banner during an approach: flying UNDER a gate pushes it out of
        the top of the frame, while drifting off it loses it sideways or in
        the middle. Those are success and failure respectively, and
        ApproachBanner could not tell them apart.
        """
        return self.banner.y if self.banner_identified() else 0.0

    def banner_bearing(self):
        """Normalised horizontal offset, [-1,1], negative = banner to the left."""
        return self.banner.x if self.banner_identified() else 0.0

    def _on_abort(self, m):
        self.abort_requested = m.data

    def _on_start(self, m):
        self.mission_started = m.data
        if m.data:
            self._result = None
            # A previous run's Land leaf leaves _expect_disarm set. Carrying it
            # into a new mission would make the watchdog swallow a genuine
            # external disarm, so every start begins with a clean slate.
            self._expect_disarm = False
            self.abort_reason = ""
            self.abort_latched = False
            self.mission_failed = False
            self.mission_complete = False
            self._attitude_violations = 0
            self.clear_airborne_floor()
            self.node.get_logger().info("Mission 2 start received from GCS")

    def _on_pose(self, m):
        self.pose = m
        self.roll_deg, self.pitch_deg = self._rp_deg(m.pose.orientation)
        worst = max(abs(self.roll_deg), abs(self.pitch_deg))
        if worst > self.attitude_limit_deg:
            self._attitude_violations += 1
        else:
            self._attitude_violations = 0

    def _on_state(self, m):
        prev = self.state
        self.state = m
        # Only an active mission can be interrupted; a disarmed idle aircraft
        # changing mode on the bench is not an event.
        if not self.mission_started:
            return

        if prev.armed and not m.armed:
            if self._expect_disarm:
                self.node.get_logger().info("Disarm observed during commanded landing")
            else:
                self._request_reset("external disarm")

        if m.mode != prev.mode and m.mode in _TERMINAL_MODES:
            if not self._mode_was_ours(m.mode):
                self._request_reset(f"external mode change to {m.mode}")

    # ------------------------------------------------------------------ #
    # attitude
    # ------------------------------------------------------------------ #
    @staticmethod
    def _rp_deg(q):
        """Roll and pitch in degrees from a geometry_msgs Quaternion."""
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return math.degrees(roll), math.degrees(pitch)

    def attitude_excessive(self):
        """True once the aircraft has held an unflyable attitude for N samples."""
        return self._attitude_violations >= self.attitude_limit_samples

    # ------------------------------------------------------------------ #
    # altitude floor while airborne
    # ------------------------------------------------------------------ #
    def set_airborne_floor(self, floor_m):
        """Arm the low-altitude fault once the aircraft is genuinely flying.

        A live run reached CORRIDOR_NAV with the aircraft at z = 0.117 m,
        crawling along the ground while the tree reported corridor navigation.
        No stage had an altitude success condition, so nothing noticed. After
        takeoff the aircraft must stay above a floor until it is deliberately
        landed.
        """
        self.airborne_floor = float(floor_m)
        self._low_alt_samples = 0

    def clear_airborne_floor(self):
        self.airborne_floor = None
        self._low_alt_samples = 0

    def low_altitude_fault(self, samples_required=8):
        """True when an airborne aircraft has sagged below its floor."""
        if self.airborne_floor is None or not self.state.armed:
            self._low_alt_samples = 0
            return False
        if self.alt() < self.airborne_floor:
            self._low_alt_samples += 1
        else:
            self._low_alt_samples = 0
        return self._low_alt_samples >= samples_required

    # ------------------------------------------------------------------ #
    # mission reset (external intervention)
    # ------------------------------------------------------------------ #
    def _request_reset(self, reason):
        if self._reset_requested:
            return
        self._reset_requested = True
        self._reset_reason = reason
        self.node.get_logger().warning(f"Mission reset requested: {reason}")

    def reset_pending(self):
        return self._reset_requested

    def consume_reset(self):
        """Returns the pending reset reason (or None) and clears mission state."""
        if not self._reset_requested:
            return None
        reason = self._reset_reason
        self._reset_requested = False
        self._reset_reason = ""
        self.mission_started = False
        self.abort_requested = False
        self.abort_latched = False
        self.mission_failed = False
        self.mission_complete = False
        self._expect_disarm = False
        self._attitude_violations = 0
        self.clear_airborne_floor()
        self._sp = None
        return reason

    def expect_disarm(self, value=True):
        self._expect_disarm = bool(value)

    # ------------------------------------------------------------------ #
    # outcome reporting
    # ------------------------------------------------------------------ #
    def publish_result(self, state, reason=""):
        """Latch a terminal mission outcome. First writer per run wins."""
        if self._result is not None:
            return
        self._result = (state, reason)
        payload = json.dumps({
            "state": state,
            "reason": reason,
            # As a NUMBER as well as inside the prose. Delivery accuracy is 15
            # rulebook marks; leaving it only in a sentence means the panel
            # can show it but cannot compare, threshold or chart it.
            "delivery_offset_m": getattr(self, "delivery_offset_m", None),
            "landing_precision": getattr(self, "landing_precision", None),
            "t": self.node.get_clock().now().nanoseconds / 1e9,
        })
        self.pub_result.publish(String(data=payload))
        self.node.get_logger().info(f"Mission result: {state} ({reason})")

    @property
    def result(self):
        return self._result

    # ------------------------------------------------------------------ #
    # setpoint streaming (gated)
    # ------------------------------------------------------------------ #
    def _stream(self):
        if self._sp is None:
            return
        # Fail-closed rail: a disarmed aircraft must never be streamed
        # position setpoints. Previously the mission kept publishing after an
        # external disarm, so the tree looked alive while nothing was flying.
        if not self.state.armed:
            self.setpoints_suppressed += 1
            self.setpoint_block_reason = "disarmed"
            return
        self.setpoint_block_reason = ""
        self._sp.header.stamp = self.node.get_clock().now().to_msg()
        self._sp.header.frame_id = 'map'
        self.pub_sp.publish(self._sp)
        self._track_altitude_error()

    def _track_altitude_error(self):
        """Watch commanded z against measured z while position-streaming.

        WHY (Phase 11, unresolved sink)

            Three randomised arenas fail with the aircraft below the corridor
            altitude band, and the two facts that must be reconciled are:

              * the corridor's own altitude hold demonstrably works -- 25
                consecutive live samples at exactly 3.0 m;
              * the approach commands POSITION setpoints at a constant
                z = 3.0 m, and a position setpoint should not permit a 1.5 m
                descent.

            So either the sink happens in a phase neither covers, or one of
            them is not in force when it happens. Rather than reasoning about
            which stage "should" be in control, this records the tick where
            commanded and measured altitude actually diverge, and which leaf
            was running at the time.
        """
        z_cmd = float(self._sp.pose.position.z)
        z_now = self.alt()
        if z_now is None:
            return
        err = z_cmd - z_now                     # positive = aircraft is LOW
        stage = self.active_stage or "?"

        # A commanded CHANGE of altitude is not a divergence. ClimbToSweep
        # commands 10 m from 3 m, and WinchDrop commands 5 m from 10 m; both
        # were reported as 7.01 m and 5.00 m "divergences" by the first
        # version of this, which is exactly the kind of false alarm that
        # makes an instrument worse than none.
        #
        # What is being hunted is the aircraft ARRIVING at a held altitude and
        # then falling away from it. So the target has to be reached once
        # before a shortfall counts, and reaching a new target re-arms it.
        # `is None` rather than a numeric sentinel: with nan, every
        # comparison is False, so the target would never update and the
        # arrival flag would never re-arm -- the guard would silently do
        # nothing. Caught by test_a_COMMANDED_climb_is_not_a_divergence.
        if self._alt_target is None or abs(z_cmd - self._alt_target) > 0.05:
            self._alt_target = z_cmd
            self._alt_arrived = False
        if abs(err) <= 0.5:
            self._alt_arrived = True
        if not self._alt_arrived:
            return

        if err > self.alt_error_worst:
            self.alt_error_worst = err
            self.alt_error_worst_stage = stage
        if err > self.alt_error_warn and stage != self._alt_warned_stage:
            self._alt_warned_stage = stage
            self.log(f"ALTITUDE DIVERGENCE in {stage}: commanded "
                     f"{z_cmd:.2f} m, measured {z_now:.2f} m "
                     f"(low by {err:.2f} m) at "
                     f"({self.pos()[0]:.1f}, {self.pos()[1]:.1f})", warn=True)

    # ------------------------------------------------------------------ #
    # intents (non-blocking)
    # ------------------------------------------------------------------ #
    def connected(self):
        return self.state.connected

    def battery_critical(self, min_volt=None, min_pct=0.15):
        """Returns True if battery is below safe critical threshold."""
        if min_volt is None:
            min_volt = self.critical_battery_voltage
        if self.battery.voltage > 0.0 and self.battery.voltage < min_volt:
            return True
        if self.battery.percentage > 0.0 and self.battery.percentage < min_pct:
            return True
        return False

    def _note_mode_command(self, mode):
        self._commanded_modes[mode] = self.node.get_clock().now().nanoseconds / 1e9

    def _mode_was_ours(self, mode):
        t = self._commanded_modes.get(mode)
        if t is None:
            return False
        now = self.node.get_clock().now().nanoseconds / 1e9
        return (now - t) <= _COMMAND_OWNERSHIP_S

    def set_mode(self, mode='GUIDED'):
        self._note_mode_command(mode)
        if self.cli_mode.service_is_ready():
            req = SetMode.Request(); req.custom_mode = mode
            self.cli_mode.call_async(req)

    def arm(self, value=True):
        if self.cli_arm.service_is_ready():
            req = CommandBool.Request(); req.value = value
            self.cli_arm.call_async(req)

    def takeoff(self, alt):
        if self.cli_takeoff.service_is_ready():
            req = CommandTOL.Request(); req.altitude = float(alt)
            self.cli_takeoff.call_async(req)

    def land(self):
        self._note_mode_command("LAND")
        if self.cli_land.service_is_ready():
            req = CommandTOL.Request()
            self.cli_land.call_async(req)
        else:
            self.set_mode("LAND")

    def goto(self, x, y, z, yaw=0.0):
        """Stream position setpoint with heading orientation (yaw in radians)."""
        sp = PoseStamped()
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        half_yaw = float(yaw) / 2.0
        sp.pose.orientation.z = math.sin(half_yaw)
        sp.pose.orientation.w = math.cos(half_yaw)
        self._sp = sp

    def pos(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def reached(self, x, y, z, tol=0.6):
        px, py, pz = self.pos()
        return math.dist((px, py, pz), (x, y, z)) < tol

    def alt(self):
        return self.pose.pose.position.z

    def yaw(self):
        """Heading in radians from the local-ENU pose quaternion."""
        q = self.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def enable_avoidance(self, on, hold_alt=None):
        """Hand control to the follow-the-gap navigator.

        `hold_alt` states the altitude the traversal must maintain. Without
        it the navigator commands `velocity.z = 0` -- zero vertical RATE, not
        a held altitude -- and any thrust bias integrates into a sink. Arena
        regression seed 1002 sank from 2.8 m to 0.9 m while stalled in the
        corridor before the altitude-band guard aborted the mission.
        """
        if on and hold_alt is not None:
            # Publish BEFORE enabling, so the navigator's first setpoint
            # already carries the correction rather than a zero.
            self.pub_hold_alt.publish(Float32(data=float(hold_alt)))
        self.pub_enable.publish(Bool(data=bool(on)))
        if on:
            self._sp = None    # stop position streaming; avoidance drives velocity

    def set_target(self, s):
        self.pub_target.publish(String(data=s))

    def winch(self, cmd):
        self.pub_winch.publish(String(data=cmd))   # "lower" | "release" | "stow"

    # ------------------------------------------------------------------ #
    # camera pointing (Phase 2)
    # ------------------------------------------------------------------ #
    def _on_avoid_detail(self, m):
        try:
            self.avoid_detail = json.loads(m.data)
        except json.JSONDecodeError:
            self.avoid_detail = {}

    def corridor_exited(self):
        return bool(self.avoid_detail.get("corridor_exited"))

    def avoidance_stuck(self):
        return self.avoid_detail.get("state") == "STUCK"

    def _on_winch_status(self, m):
        try:
            self.winch_status = json.loads(m.data)
        except json.JSONDecodeError:
            self.winch_status = {}

    def _on_camera_state(self, m):
        try:
            self.camera_state = json.loads(m.data)
        except json.JSONDecodeError:
            self.camera_state = {}

    def set_camera_pose(self, pose):
        self.pub_camera_pose.publish(String(data=str(pose)))

    def camera_settled(self, pose=None):
        """True only when the joint has been MEASURED at the requested pose.

        Never infer this from having sent a command: the whole point of the
        Phase 2 rail is that a perception stage cannot run while the camera is
        somewhere other than where the mission believes it to be.
        """
        st = self.camera_state
        if not st or not st.get("settled"):
            return False
        if st.get("stale"):
            return False
        if pose is not None and st.get("requested") != pose:
            return False
        return True

    def camera_state_summary(self):
        st = self.camera_state
        if not st:
            return "no /camera/pose_state yet"
        err = st.get("error_deg")
        err_s = "?" if err is None else f"{err:.1f}deg"
        return (f"req={st.get('requested', '?')} err={err_s} "
                f"settled={st.get('settled')} stale={st.get('stale')}")


    # ---------------------------------------------------------------- #
    # Observed geometry (geometry audit A7, A8, A9)
    # ---------------------------------------------------------------- #

    def record_corridor_exit(self):
        """Latch where and at what heading the corridor opened out.

        Called once by the Corridor leaf on success. The return leg and the
        delivery-zone bound are both anchored here instead of on asserted
        arena coordinates. First write wins: the outbound exit is the mouth,
        and the return trip must not overwrite it.
        """
        if self.corridor_exit_pose is None:
            x, y, z = self.pos()
            self.corridor_exit_pose = (x, y, z, self.yaw())
        return self.corridor_exit_pose

    def _on_fence_readback(self, m):
        self.fence_readback = list(m.waypoints)

    def home_global(self):
        """(lat, lon) from the FC, or None. The fence needs a global anchor."""
        if self.home is None:
            return None
        return (float(self.home.geo.latitude), float(self.home.geo.longitude))

    def push_fence(self, items):
        """Upload a fence. Returns the service future, or None if unavailable."""
        if not self.cli_fence_push.service_is_ready():
            return None
        req = _WaypointPush.Request()
        req.start_index = 0
        req.waypoints = items
        self.fence_readback = None
        return self.cli_fence_push.call_async(req)

    def _on_redzone(self, m):
        """Georeferenced red zones, replacing a positionless Bool (Phase 7)."""
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.redzone_status = d.get("status", "NOT_VISIBLE")
        ex = d.get("exclusions") or []
        self.exclusions = [tuple(float(v) for v in r) for r in ex if len(r) == 4]

    def log(self, msg, warn=False):
        """Route a stage's findings to the ROS log.

        py_trees `self.logger` goes to the py_trees logger, which nothing in
        this stack configures — so the observed zone bounds, the derived sweep
        and decode altitudes, the coverage figure and the fence verification
        result were all computed, used, and never recorded anywhere. The
        evidence pack for Phase 6 and Phase 7 depends on exactly those lines.
        """
        (self.node.get_logger().warning if warn
         else self.node.get_logger().info)(msg)

    def corridor_entered(self):
        """Has the navigator OBSERVED walls close in on both sides?

        Published whether or not the navigator is steering, because the stage
        that hands control to it has to know this before it does so.
        """
        return bool((self.avoid_detail or {}).get("corridor_entered", False))

    def open_extent(self):
        """(depth_ahead_m, width_m) of the open area, as the navigator sees it.

        Published by the corridor navigator on /avoid/detail. Returns (0, 0)
        when the navigator has not reported, which ObserveZone treats as
        "nothing observed yet" rather than "a zone of size zero".
        """
        d = self.avoid_detail or {}
        return (float(d.get("open_depth_m", 0.0) or 0.0),
                float(d.get("open_width_m", 0.0) or 0.0))

    def qr_hold_xy(self, default=(0.0, 0.0), gain=0.8, max_step=1.0):
        """Where to hold station while descending onto a QR candidate.

        Same image->local mapping CenterOnQR uses and tests assert (image +x
        right, +y DOWN; forward correction = -offset.y, right = +offset.x,
        rotated by the aircraft's own yaw). Duplicating that reasoning with a
        different sign is exactly what flew the aircraft into a wall in Phase
        2, so it is derived here the same way.

        With no candidate in view the caller's position is held unchanged.
        """
        if not self.qr_visible():
            return default
        x, y, _ = self.pos()
        off = self.qr_offset
        fwd = max(-max_step, min(max_step, -float(off.y) * gain))
        right = max(-max_step, min(max_step, float(off.x) * gain))
        psi = self.yaw()
        return (x + fwd * math.cos(psi) + right * math.sin(psi),
                y + fwd * math.sin(psi) - right * math.cos(psi))
