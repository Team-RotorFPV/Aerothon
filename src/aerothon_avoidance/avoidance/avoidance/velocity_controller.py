#!/usr/bin/env python3
"""Corridor navigation by follow-the-gap, with recovery.

WHAT REPLACED WHAT (Phase 5)

  The previous controller computed a front/left/right sector minimum, slowed as
  the front closed, stopped inside stop_dist, and nudged sideways in proportion
  to the left/right imbalance. Two consequences, both seen live:

    * With symmetric walls the imbalance is zero, so when something blocks the
      front it commands a full stop and NOTHING ELSE. The first end-to-end run
      sat at x=4.86 for over 250 s with front_m=0.77, cmd_vx=0.0,
      centering_err=-0.0 — stopped, centred, and permanently stuck.
    * It could not choose a side to pass an obstacle, because it never measured
      obstacle extent — only three sector minima.

  This version steers toward the largest navigable GAP in the forward arc,
  which handles centring, obstacle avoidance and pass-side selection with one
  mechanism, and escalates through recovery states rather than stalling.

FRAME CONVENTION — unchanged from Phase 2 and still the thing to be careful of.
  MAVROS takes body-frame setpoints in ROS FLU and converts to FRD itself, so
  on /mavros/setpoint_raw/local with coordinate_frame 8/9:
      velocity.x = forward     velocity.y = LEFT      velocity.z = UP
  Getting this backwards once already flew the aircraft into a wall.

STATES
  CRUISE    a gap is open; drive toward its bearing
  BLOCKED   no acceptable gap; hold and re-evaluate briefly
  BACKOFF   reverse slowly to open the field of view, then try again
  STUCK     escalation exhausted; reported so the mission can fail closed

LIDAR PRE-PROCESSING (goal.md Q21, previously absent)
  * angular masking of the frame/arm sectors
  * range clamping to [range_min_valid, range_max_valid]
  * median filter to drop isolated outliers (dust, sunlight speckle)
  * stale-scan failsafe: no scan for stale_scan_s means stop, not coast

Topics
  sub  <scan_topic>                sensor_msgs/LaserScan
  sub  /avoidance/enable           std_msgs/Bool
  sub  /avoidance/cruise           std_msgs/Float32
  pub  /mavros/setpoint_raw/local  mavros_msgs/PositionTarget  body velocity
  pub  /avoidance/status           geometry_msgs/Vector3  x=front y=gap_bearing z=cmd_vx
  pub  /avoidance/detail           std_msgs/String        JSON diagnostics
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.msg import PositionTarget

# PositionTarget type_mask: use vx,vy,vz + yaw_rate; ignore pos, accel, yaw.
IGN_PX, IGN_PY, IGN_PZ = 1, 2, 4
IGN_AFX, IGN_AFY, IGN_AFZ = 64, 128, 256
IGN_YAW = 1024
VEL_YAWRATE_MASK = (IGN_PX | IGN_PY | IGN_PZ |
                    IGN_AFX | IGN_AFY | IGN_AFZ | IGN_YAW)  # = 1479
FRAME_BODY_OFFSET_NED = 9


class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('rate_hz', 20.0)
        p('cruise_speed', 0.8)
        p('max_lateral', 0.6)
        # Yaw rate CAP, not the yaw rate itself -- see _yaw_rate_for(). This
        # was published directly as sp.yaw_rate, so at its old value of 0.0
        # the aircraft could never turn.
        p('max_yaw_rate', 0.5)        # rad/s ceiling on corridor alignment
        p('yaw_align_gain', 1.2)      # rad/s per radian of gap bearing

        # Gap search
        p('search_fov_deg', 120.0)    # +/- around forward to look for a gap
        p('safety_radius', 0.8)       # goal.md Q8 keep-out bubble (m)
        p('min_gap_width_deg', 14.0)  # narrower than this is not a way through
        p('gap_bearing_gain', 1.4)    # lateral m/s per radian of gap bearing
        p('brake_dist', 2.5)
        p('stop_dist', 0.8)

        # Lidar conditioning (Q21)
        p('mask_sectors_deg', [150.0, 210.0])   # rear arm/standoff shadow
        p('range_min_valid', 0.15)
        p('range_max_valid', 12.0)
        p('median_window', 3)
        p('stale_scan_s', 0.7)

        # Escalation
        p('blocked_ticks_before_backoff', 30)
        p('corridor_open_m', 3.5)         # side clearance meaning "not in a corridor"
        p('corridor_open_ticks', 15)      # sustained open before declaring exit
        p('corridor_enter_ticks', 10)     # sustained ENCLOSED before "inside"
        p('progress_window_s', 12.0)      # look-back for forward progress
        p('min_progress_m', 0.8)          # less than this over the window = stalled
        # ---- altitude hold ----
        # `velocity.z = 0.0` is a command for zero vertical RATE, not for a
        # held altitude: any thrust bias or disturbance integrates into a sink.
        # Arena regression seed 1002 sank from 2.8 m to 0.9 m in eight seconds
        # while stalled in the corridor, and the mission's altitude-band guard
        # (correctly) aborted. The loop has to be closed.
        p('alt_hold_gain', 0.8)       # m/s of vz per metre of error
        p('max_vz', 0.6)              # ceiling on the correction
        p('backoff_ticks', 40)
        p('backoff_speed', 0.35)
        p('max_backoffs', 3)

        self.enabled = False
        self.cruise = float(self.get_parameter('cruise_speed').value)
        self.scan = None
        self._scan_t = None
        self.state = "CRUISE"
        self._blocked = 0
        self._backoff = 0
        self._backoffs_done = 0
        self._last_detail = {}
        self._pos = None
        self._alt = None            # current altitude, from /local_position/pose
        self._hold_alt = None       # altitude this traversal should maintain
        self._progress_ref = None
        self._progress_t = None
        self._open_ticks = 0
        # You cannot exit a corridor you were never inside. Without this the
        # detector fired on the open apron BEFORE the corridor: a live run
        # reported "corridor opened out" one second after entering the stage,
        # at x = 1.2 m, and the mission then flew a delivery-zone search on
        # the takeoff pad and reported COMPLETED. The hardcoded GotoZone
        # waypoint had been dragging it to the real zone and hiding this.
        self._enclosed_ticks = 0
        self._entered = False

        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avoidance/enable', self._on_enable, 10)
        self.create_subscription(Float32, '/avoidance/cruise', self._on_cruise, 10)
        self.create_subscription(Float32, '/avoidance/hold_alt',
                                 self._on_hold_alt, 10)
        self.pub_sp = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.pub_status = self.create_publisher(Vector3, '/avoidance/status', 10)
        self.pub_detail = self.create_publisher(String, '/avoidance/detail', 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"velocity_controller (follow-the-gap) up; scan={scan_topic}")

    # ------------------------------------------------------------------ #
    def _g(self, n):
        return self.get_parameter(n).value

    def _on_enable(self, m: Bool):
        if m.data and not self.enabled:
            self.state = "CRUISE"
            self._blocked = 0
            self._backoff = 0
            self._backoffs_done = 0
            self._progress_ref = None
            self._progress_t = None
            # Each corridor stage is its own traversal: the return trip has to
            # observe entry again before it can claim an exit.
            self._open_ticks = 0
            self._enclosed_ticks = 0
            self._entered = False
            self._exited = False
            # Latch the altitude to hold for this traversal. The mission can
            # override it on /avoidance/hold_alt; without that, whatever the
            # aircraft was flying at when avoidance was handed control is the
            # honest default.
            if self._alt is not None:
                self._hold_alt = self._alt
        if not m.data:
            self._hold_alt = None
        self.enabled = m.data

    def _on_cruise(self, m: Float32):
        self.cruise = float(m.data)

    def _alt_correction(self):
        """Vertical velocity that holds `_hold_alt`, in m/s (ENU, +up).

        Zero until an altitude to hold and a measurement of the current one
        are both available -- an unknown altitude must not produce a
        confident correction.
        """
        if self._hold_alt is None or self._alt is None:
            return 0.0
        err = self._hold_alt - self._alt
        vz = float(self._g('alt_hold_gain')) * err
        cap = float(self._g('max_vz'))
        return max(-cap, min(cap, vz))

    def _on_hold_alt(self, m: Float32):
        """The mission states the altitude this traversal should hold.

        Better than latching on enable: the corridor stage knows it wants the
        rulebook's 3 m, whereas the latched value is only whatever the
        aircraft happened to be at when control was handed over -- which may
        already be wrong.
        """
        self._hold_alt = float(m.data) if m.data > 0.0 else None

    def _on_pose(self, m: PoseStamped):
        self._pos = (m.pose.position.x, m.pose.position.y)
        # z was thrown away here, which is why "hold altitude" could never be
        # more than a comment: the controller did not know its own altitude.
        self._alt = float(m.pose.position.z)

    def stalled(self):
        """True when the aircraft has not covered ground recently.

        Gap-absence is not the only way to get stuck: the live run had a gap
        the whole time and still made no progress for four minutes. Progress
        is measured, not inferred from the controller's own intentions.
        """
        if self._pos is None:
            return False
        now = self._now()
        if self._progress_ref is None:
            self._progress_ref, self._progress_t = self._pos, now
            return False
        window = float(self._g('progress_window_s'))
        if (now - self._progress_t) < window:
            return False
        moved = math.dist(self._pos, self._progress_ref)
        self._progress_ref, self._progress_t = self._pos, now
        return moved < float(self._g('min_progress_m'))

    def _on_scan(self, m: LaserScan):
        self.scan = m
        self._scan_t = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_stale(self):
        if self._scan_t is None:
            return True
        return (self._now() - self._scan_t) > float(self._g('stale_scan_s'))

    # ------------------------------------------------------------------ #
    def conditioned(self, scan):
        """(bearings, ranges) after masking, clamping and median filtering."""
        lo = float(self._g('range_min_valid'))
        hi = float(self._g('range_max_valid'))
        mask = [math.radians(a) for a in self._g('mask_sectors_deg')]
        bearings, ranges = [], []

        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            wrapped = (ang + math.pi) % (2 * math.pi) - math.pi
            # Angular mask: drop the arm/standoff shadow sector.
            if len(mask) == 2:
                a_deg = math.degrees(wrapped) % 360.0
                if mask[0] <= math.radians(a_deg) <= mask[1]:
                    continue
            if r != r or r in (float('inf'), float('-inf')):
                r = hi
            r = max(lo, min(hi, float(r)))
            bearings.append(wrapped)
            ranges.append(r)

        # Median filter: a single short return between two long ones is noise,
        # not a wall, and braking for it is how a corridor run stalls.
        w = int(self._g('median_window'))
        if w >= 3 and len(ranges) >= w:
            half = w // 2
            smoothed = []
            for i in range(len(ranges)):
                lo_i = max(0, i - half)
                hi_i = min(len(ranges), i + half + 1)
                window = sorted(ranges[lo_i:hi_i])
                smoothed.append(window[len(window) // 2])
            ranges = smoothed
        return bearings, ranges

    def find_gap(self, bearings, ranges):
        """Widest contiguous arc, within the search FOV, that is clear enough.

        Returns (bearing_rad, width_rad, depth_m) or None. Steering at the
        centre of the widest gap gives corridor centring, obstacle avoidance
        and pass-side selection from one computation.
        """
        fov = math.radians(float(self._g('search_fov_deg'))) / 2.0
        clear = float(self._g('safety_radius'))
        min_w = math.radians(float(self._g('min_gap_width_deg')))

        pts = [(b, r) for b, r in zip(bearings, ranges) if abs(b) <= fov]
        if not pts:
            return None
        pts.sort(key=lambda t: t[0])

        best = None
        run_start = None
        prev_b = None
        for b, r in pts:
            passable = r > clear
            if passable and run_start is None:
                run_start = b
            if (not passable or b == pts[-1][0]) and run_start is not None:
                run_end = prev_b if not passable else b
                width = run_end - run_start
                if width >= min_w:
                    mid = 0.5 * (run_start + run_end)
                    depth = min(rr for bb, rr in pts if run_start <= bb <= run_end)
                    # Prefer wide gaps, and among similar widths the one
                    # requiring least turning.
                    score = width - 0.25 * abs(mid)
                    if best is None or score > best[0]:
                        best = (score, mid, width, depth)
                run_start = None
            prev_b = b

        if best is None:
            return None
        _, mid, width, depth = best
        return mid, width, depth

    @staticmethod
    def _front_min(bearings, ranges, half_fov=math.radians(20)):
        vals = [r for b, r in zip(bearings, ranges) if abs(b) <= half_fov]
        return min(vals) if vals else float('inf')

    @staticmethod
    def _side_min(bearings, ranges, centre_deg, half_fov=math.radians(35)):
        c = math.radians(centre_deg)
        vals = []
        for b, r in zip(bearings, ranges):
            d = (b - c + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half_fov:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def open_extent(self, bearings, ranges):
        """(depth_ahead, width) of the open area, in metres.

        Used to bound the delivery zone from observation instead of asserting
        it (geometry audit A7, A8). Depth is the clear distance dead ahead;
        width is the span between the nearest returns to either side.
        """
        depth = self._front_min(bearings, ranges, math.radians(15))
        left = self._side_min(bearings, ranges, 90.0)
        right = self._side_min(bearings, ranges, -90.0)
        return depth, left + right

    def corridor_open(self, bearings, ranges):
        """Both walls have fallen away: we are no longer inside a corridor.

        This is what replaces the hardcoded corridor_exit_x / 
        corridor_return_exit_x thresholds (geometry audit A6 and A10). A
        position threshold assumes you already know where the corridor ends;
        the lidar can simply see that it has.
        """
        thresh = float(self._g('corridor_open_m'))
        left = self._side_min(bearings, ranges, 90.0)
        right = self._side_min(bearings, ranges, -90.0)
        return (left > thresh and right > thresh), left, right

    # ------------------------------------------------------------------ #
    _exited = False
    _entered = False
    _left_m = 0.0
    _right_m = 0.0
    _open_depth = 0.0
    _open_width = 0.0

    def _observe_only(self):
        """Update and publish the corridor picture without commanding anything."""
        if self.scan is None or self.scan_stale():
            return
        bearings, ranges = self.conditioned(self.scan)
        if not ranges:
            return
        is_open, left_m, right_m = self.corridor_open(bearings, ranges)
        self._open_depth, self._open_width = self.open_extent(bearings, ranges)
        self._open_ticks = self._open_ticks + 1 if is_open else 0
        self._enclosed_ticks = 0 if is_open else self._enclosed_ticks + 1
        if self._enclosed_ticks >= int(self._g('corridor_enter_ticks')):
            self._entered = True
        self._left_m, self._right_m = left_m, right_m
        self.pub_detail.publish(String(data=json.dumps({
            "state": "OBSERVING",
            "front_m": round(self._front_min(bearings, ranges), 2),
            "corridor_entered": bool(self._entered),
            "corridor_exited": bool(self._exited),
            "side_left_m": round(left_m, 2),
            "side_right_m": round(right_m, 2),
            "open_depth_m": round(self._open_depth, 2),
            "open_width_m": round(self._open_width, 2),
        })))

    def _yaw_rate_for(self, bearing):
        """Turn to face the gap, capped at `max_yaw_rate` (rad/s).

        WHAT WAS WRONG

            sp.yaw_rate = float(self._g('max_yaw_rate'))

            That published a PARAMETER as the command -- a constant -- and the
            parameter was 0.0, commented "heading is owned by the mission". So
            the navigator measured a gap bearing every tick and then threw it
            away: the aircraft could not turn, and had to cross a rotated
            corridor by sliding sideways on `vy` alone.

            In the shipped arena the corridor is axis-aligned, so entering it
            on the banner heading is already correct and nothing shows. Phase
            11 rotates it, and the outcome tracks the rotation exactly:

                seed 1001   10.4 deg   FAILED
                seed 1002  -11.2 deg   FAILED
                seed 1003    8.9 deg   FAILED
                seed 1004    4.5 deg   completed
                seed 1005    1.5 deg   completed

            Recorded flight of seed 1002: the aircraft cruised in cleanly at
            3.00 m and 0.8 m/s while drifting from y = -1.8 to y = -2.9, wedged
            with an obstacle 0.3 m ahead, exhausted its backoff ladder, and
            tipped to 47.8 degrees of pitch. The commanded velocities at the
            moment it tipped were 0.00, 0.00, 0.03 -- the pitch-over is a
            COLLISION, not a commanded manoeuvre.

        The mission still owns the heading everywhere else; it hands control to
        this node only for the traversal, and inside the corridor the thing
        worth pointing at is the gap.
        """
        gain = float(self._g('yaw_align_gain'))
        cap = float(self._g('max_yaw_rate'))
        rate = gain * float(bearing)
        return max(-cap, min(cap, rate))

    def _publish(self, vx, vy, front, bearing, extra):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = FRAME_BODY_OFFSET_NED
        sp.type_mask = VEL_YAWRATE_MASK
        sp.velocity.x = float(vx)      # forward
        sp.velocity.y = float(vy)      # LEFT (MAVROS converts FLU -> FRD)
        sp.velocity.z = float(self._alt_correction())   # CLOSED-LOOP hold
        sp.yaw_rate = float(self._yaw_rate_for(bearing))
        self.pub_sp.publish(sp)

        self.pub_status.publish(Vector3(x=float(front), y=float(bearing),
                                        z=float(vx)))
        detail = {"state": self.state, "front_m": round(front, 2),
                  "corridor_entered": bool(self._entered),
                  "corridor_exited": bool(self._exited),
                  "side_left_m": round(self._left_m, 2),
                  "side_right_m": round(self._right_m, 2),
                  "open_depth_m": round(self._open_depth, 2),
                  "open_width_m": round(self._open_width, 2),
                  "gap_bearing_deg": round(math.degrees(bearing), 1),
                  "cmd_vx": round(vx, 2), "cmd_vy_left": round(vy, 2),
                  "cmd_vz": round(self._alt_correction(), 2),
                  "cmd_yaw_rate": round(self._yaw_rate_for(bearing), 2),
                  "hold_alt_m": (None if self._hold_alt is None
                                 else round(self._hold_alt, 2)),
                  "alt_m": (None if self._alt is None else round(self._alt, 2)),
                  "backoffs": self._backoffs_done}
        detail.update(extra)
        self._last_detail = detail
        self.pub_detail.publish(String(data=json.dumps(detail)))

    def _tick(self):
        # Observation continues whether or not this node is steering. A stage
        # that has to decide "am I inside the corridor yet?" needs the answer
        # BEFORE it hands control over, and a detector that only reports while
        # it is driving cannot supply it.
        if not self.enabled:
            self._observe_only()
            return

        if self.scan is None or self.scan_stale():
            # Never coast on stale data: an unknown world is a stop, not a
            # continuation of the last command.
            self._publish(0.0, 0.0, 0.0, 0.0,
                          {"fault": "scan stale or missing"})
            return

        bearings, ranges = self.conditioned(self.scan)
        if not ranges:
            self._publish(0.0, 0.0, 0.0, 0.0, {"fault": "no valid returns"})
            return

        front = self._front_min(bearings, ranges)
        gap = self.find_gap(bearings, ranges)

        is_open, left_m, right_m = self.corridor_open(bearings, ranges)
        self._open_depth, self._open_width = self.open_extent(bearings, ranges)
        self._open_ticks = self._open_ticks + 1 if is_open else 0
        # Entry must be observed before exit can mean anything.
        self._enclosed_ticks = 0 if is_open else self._enclosed_ticks + 1
        if self._enclosed_ticks >= int(self._g('corridor_enter_ticks')):
            self._entered = True
        exited = (self._entered
                  and self._open_ticks >= int(self._g('corridor_open_ticks')))
        self._exited, self._left_m, self._right_m = exited, left_m, right_m
        brake = float(self._g('brake_dist'))
        stop = float(self._g('stop_dist'))
        max_lat = float(self._g('max_lateral'))

        # ---- BACKOFF: reverse to open the view, then re-evaluate ---- #
        if self.state == "BACKOFF":
            self._backoff -= 1
            if self._backoff <= 0:
                self.state = "CRUISE"
                self._blocked = 0
            self._publish(-float(self._g('backoff_speed')), 0.0, front, 0.0,
                          {"note": "reversing to find a way through"})
            return

        if gap is not None and self.stalled():
            # A gap we are not actually travelling through.
            self.get_logger().warning(
                "gap present but no ground covered; treating as blocked")
            gap = None

        if gap is None:
            self._blocked += 1
            if self._blocked >= int(self._g('blocked_ticks_before_backoff')):
                if self._backoffs_done < int(self._g('max_backoffs')):
                    self._backoffs_done += 1
                    self._backoff = int(self._g('backoff_ticks'))
                    self.state = "BACKOFF"
                    self.get_logger().warning(
                        f"no gap; backing off "
                        f"({self._backoffs_done}/{self._g('max_backoffs')})")
                else:
                    self.state = "STUCK"
                    self.get_logger().error(
                        "no way through after all recovery attempts")
            else:
                self.state = "BLOCKED"
            self._publish(0.0, 0.0, front, 0.0,
                          {"note": "no navigable gap", "blocked_ticks": self._blocked})
            return

        bearing, width, depth = gap
        self.state = "CRUISE"
        self._blocked = 0

        # Travel ALONG the gap, decomposed into body axes.
        #
        # Gating forward speed on straight-ahead clearance while steering
        # sideways deadlocks whenever the gap is off-axis: the live run found a
        # gap 31 deg to the left with dead-ahead blocked at 0.79 m, so it
        # commanded vx=0, vy=0.6 and slid sideways for four minutes without
        # ever moving along the corridor. Speed comes from the clearance in the
        # direction actually being travelled.
        reach = max(depth, front if abs(bearing) < math.radians(20) else depth)
        if reach <= stop:
            speed = 0.0
        elif reach >= brake:
            speed = self.cruise
        else:
            speed = self.cruise * (reach - stop) / max(1e-3, brake - stop)

        vx = speed * math.cos(bearing)
        # POSITIVE vy is LEFT and a positive bearing is to the left in ROS
        # LaserScan convention, so the sign is direct — asserted in tests.
        vy = speed * math.sin(bearing) + \
            float(self._g('gap_bearing_gain')) * bearing * 0.25
        vy = max(-max_lat, min(max_lat, vy))

        self._publish(vx, vy, front, bearing,
                      {"gap_width_deg": round(math.degrees(width), 1),
                       "gap_depth_m": round(depth, 2),
                       "reach_m": round(reach, 2)})

    @property
    def last_detail(self):
        return self._last_detail


def main():
    rclpy.init()
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
