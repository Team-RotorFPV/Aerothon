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

import numpy as np
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

# The constants math.degrees / math.radians multiply by.
_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0


def _polar_xy(bearings, ranges):
    """Body-frame (x, y) of each return. The trig is `math`'s, per return,
    so the points are exactly those the per-ray loop produced."""
    b = bearings.tolist()
    cos = np.fromiter(map(math.cos, b), np.float64, len(b))
    sin = np.fromiter(map(math.sin, b), np.float64, len(b))
    return ranges * cos, ranges * sin


def _headings(n, step):
    """Candidate headings -n..n steps: theta, and cos / sin as columns."""
    th = np.arange(-n, n + 1) * step
    t = th.tolist()
    c = np.fromiter(map(math.cos, t), np.float64, len(t))
    s = np.fromiter(map(math.sin, t), np.float64, len(t))
    return th, c[:, None], s[:, None]


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
        # +/- 90 deg around forward to look for a gap. It was +/- 60: in a
        # tight slalom (a custom arena's 3.2 m lane, 1.7 m gaps) the way past a
        # block 0.9 m ahead is a slide SIDEWAYS, and at 60 deg the block's
        # corner capped every candidate, so the aircraft crept into the block
        # and reported STUCK.
        p('search_fov_deg', 180.0)
        p('yaw_follow_limit_deg', 60.0)   # max heading off the corridor axis; see _yaw_rate_for
        p('safety_radius', 0.8)       # goal.md Q8 keep-out bubble (m)
        p('min_gap_width_deg', 14.0)  # narrower than this is not a way through
        # Physical passage test (find_gap). Iris: 0.25 m arm plus 0.127 m
        # prop radius ~= 0.4 m, plus 0.25 m for the airframe's lag behind a
        # velocity command; the comfort strip centres it. At 0.45 m live run
        # 4 slid down o2's flank 0.17 m off the block and hit it.
        # Swept in sim/test_slalom_traverse.py against the shipped return lane
        # with velocity lag: 0.75+ finds no heading through its 2.07 m gaps,
        # 0.7 / 1.3 clears every block by 0.60-0.65 m from all three starts.
        p('passage_half_width', 0.7)
        p('airframe_radius', 0.4)
        p('passage_comfort_width', 1.3)
        p('lookahead_m', 5.0)
        p('turn_penalty_m_per_rad', 1.0)
        p('gap_commit_m_per_rad', 0.3)   # hysteresis on the chosen heading
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
        self._yaw = None            # current heading, from the pose
        self._axis = None           # the corridor axis this traversal started on
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
            # The corridor's axis: the heading the aircraft is handed control
            # on, square to the banner. Latched on the first steering tick.
            self._axis = None
            self._last_gap_bearing = None
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
        q = m.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

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
        """(bearings, ranges) after masking, clamping and median filtering.

        Both are float64 numpy arrays. Every step is the same IEEE operation
        the per-ray loop this replaced performed, in the same order, so the
        output is bit-identical to it (sim/test_navigator_equivalence.py);
        on a Pi 5 the loop was most of the 20 Hz tick.
        """
        lo = float(self._g('range_min_valid'))
        hi = float(self._g('range_max_valid'))
        mask = [math.radians(a) for a in self._g('mask_sectors_deg')]
        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
        wrapped = (ang + math.pi) % (2 * math.pi) - math.pi
        # Angular mask: drop the arm/standoff shadow sector.
        if len(mask) == 2:
            a_rad = ((wrapped * _RAD2DEG) % 360.0) * _DEG2RAD
            keep = ~((mask[0] <= a_rad) & (a_rad <= mask[1]))
            wrapped, r = wrapped[keep], r[keep]
        r = np.where(np.isfinite(r), r, hi)
        ranges = np.maximum(lo, np.minimum(hi, r))

        # Median filter: a single short return between two long ones is noise,
        # not a wall, and braking for it is how a corridor run stalls.
        w = int(self._g('median_window'))
        n = ranges.size
        if w >= 3 and n >= w:
            half = w // 2
            smoothed = np.empty_like(ranges)
            # The window is truncated at the ends, and the median of an
            # even-length window is its upper middle element.
            if n > 2 * half:
                win = np.stack([ranges[k:n - 2 * half + k]
                                for k in range(2 * half + 1)], axis=1)
                smoothed[half:n - half] = np.sort(win, axis=1)[:, half]
            for i in list(range(min(half, n))) + list(range(max(half, n - half), n)):
                window = sorted(ranges[max(0, i - half):min(n, i + half + 1)])
                smoothed[i] = window[len(window) // 2]
            ranges = smoothed
        return wrapped, ranges

    def find_gap(self, bearings, ranges):
        """The heading along which the AIRFRAME has the most room.

        Returns (bearing_rad, width_rad, depth_m) or None, where depth is how
        far a strip as wide as the airframe stays clear along that heading.

        WHAT THIS REPLACES, AND WHY

            The widest ANGULAR arc of rays longer than the 0.8 m safety
            radius. An obstacle 2.3 m ahead is "longer than 0.8 m", so it
            counted as open, and in the return slalom the widest open arc
            pointed straight at the next block. Live run 3 on the shipped
            arena: out of the return gate past o4, then a steady drift into
            o3's shadow, creeping at centimetres a second with the block
            0.35 m off the nose, and a 74 degree roll when it touched. Seed
            1002 in the old regression wedged the same way.

            An angle says nothing about whether the aircraft FITS. So each
            candidate heading is tested physically: every return inside a
            strip of `passage_half_width` either side of the line bounds how
            far the aircraft can fly along it. A wider `passage_comfort_width`
            strip adds a pull toward the middle of a passage, and a turn
            penalty keeps it flying straight when straight is as good.
        """
        strips = self._strips(bearings, ranges)
        gap = self._find_gap(bearings, ranges,
                             float(self._g('passage_half_width')), strips)
        if gap is None:
            # Nothing fits the full strip. Before calling it blocked, try the
            # bare airframe plus 0.1 m: a tight spot should be crept through,
            # not frozen in front of.
            gap = self._find_gap(bearings, ranges,
                                 float(self._g('airframe_radius')) + 0.1,
                                 strips)
        if gap is None:
            gap = self._escape(bearings, ranges)
        return gap

    def _escape(self, bearings, ranges):
        """Already too close to something: creep the way that opens room.

        Every forward heading is inside the strip of a return a few tens of
        centimetres off. Backing off blind is what pinned seed 1002 against a
        pillar. Instead take the heading whose line passes the near returns
        widest, and give it a reach just past the stop distance so the speed
        law creeps (~0.15 m/s) rather than drives.
        """
        fov = math.radians(float(self._g('search_fov_deg'))) / 2.0
        r_air = float(self._g('airframe_radius'))
        stop = float(self._g('stop_dist'))
        b, r = np.asarray(bearings), np.asarray(ranges)
        sel = r < 1.5
        if not sel.any():
            return None
        near_x, near_y = _polar_xy(b[sel], r[sel])
        step = math.radians(2.0)
        th, c, s = _headings(int(fov / step), step)
        x, y = near_x[None, :], near_y[None, :]
        ahead = x * c + y * s > 0.0
        lat = np.abs(-x * s + y * c)
        m = np.where(ahead, lat, np.inf).min(axis=1)
        m = np.where(ahead.any(axis=1), m, 1.5)
        score = m - 0.05 * np.abs(th)
        j = int(np.argmax(score))          # the first of equal bests, as before
        if m[j] < r_air - 0.05:
            return None
        return float(th[j]), step, stop + 0.3

    def _strips(self, bearings, ranges):
        """Every candidate heading against every return, for _find_gap.

        None of it depends on the strip width, so the full-strip pass and the
        bare-airframe retry share one computation. Rows are headings -n..n in
        2 degree steps, columns returns.
        """
        fov = math.radians(float(self._g('search_fov_deg'))) / 2.0
        comfort = float(self._g('passage_comfort_width'))
        look = float(self._g('lookahead_m'))
        far = float(self._g('range_max_valid')) - 1e-3
        b, r = np.asarray(bearings), np.asarray(ranges)
        # Beyond look-ahead plus a strip width a return cannot shorten any
        # clearance; skipping it keeps 20 Hz cheap.
        sel = (r < far) & (r <= look + comfort)
        b, r = b[sel], r[sel]
        ox, oy = _polar_xy(b, r)
        step = math.radians(2.0)
        n = int(fov / step)
        th, c, s = _headings(n, step)
        x, y = ox[None, :], oy[None, :]
        along = x * c + y * s
        lat = np.abs(-x * s + y * c)
        # Returns right beside the airframe (along < 0.3 m) are what it is
        # sliding past, not what it is flying into.
        wide_of = np.where((lat <= comfort) & (0.3 < along), along, look).min(
            axis=1, initial=look)
        return n, step, th, along, lat, along > 0.0, np.minimum(1.0, r), wide_of

    def _find_gap(self, bearings, ranges, half_w, strips=None):
        look = float(self._g('lookahead_m'))
        stop = float(self._g('stop_dist'))
        turn_k = float(self._g('turn_penalty_m_per_rad'))
        r_air = float(self._g('airframe_radius'))
        n, step, th, along, lat, ahead, r_1, wide_of = (
            strips if strips is not None else self._strips(bearings, ranges))
        # Strip half-width for each return. The margin beyond the airframe
        # radius is taken in full from 1 m out and shrinks to none at
        # contact: an aircraft already beside a block has to be allowed to
        # peel AWAY from it, which a full-margin strip forbids in every
        # direction and so freezes it there.
        hw = r_air + (half_w - r_air) * r_1
        narrow_of = np.where(ahead & (lat <= hw[None, :]), along, look).min(
            axis=1, initial=look)
        # THE TURN PENALTY SCALES WITH HOW OPEN STRAIGHT AHEAD IS. It keeps
        # the aircraft from weaving down a clear lane; in front of a block it
        # made "0.9 m to the block, straight on" outscore "2.3 m clear to the
        # side" (0.88 against 2.27 - 1.57), and the aircraft crept into the
        # block. Full strength with the look-ahead clear, a tenth of it when
        # straight ahead is about to be blocked.
        ahead_m = float(narrow_of[n])
        k = turn_k * max(0.1, min(1.0, ahead_m / look))
        # COMMIT TO A SIDE. With a block dead centre both sides score alike,
        # and choosing afresh every tick flipped left-right-left in front of
        # it for two minutes. A small pull toward last tick's choice makes the
        # first decision stick unless the other side becomes clearly better.
        last = getattr(self, "_last_gap_bearing", None)
        commit = float(self._g('gap_commit_m_per_rad'))
        open_ = narrow_of > stop
        if not open_.any():
            return None
        score = 0.6 * narrow_of + 0.4 * wide_of - k * np.abs(th)
        if last is not None:
            score = score - commit * np.abs(th - last)
        j = int(np.argmax(np.where(open_, score, -np.inf)))   # first of equals
        self._last_gap_bearing = (j - n) * step
        depth = float(narrow_of[j])
        # Angular width of the open set of headings around the chosen one.
        shut = np.flatnonzero(~open_)
        left, right = shut[shut < j], shut[shut > j]
        lo = int(left[-1]) + 1 if left.size else 0
        hi = int(right[0]) - 1 if right.size else 2 * n
        return (j - n) * step, (hi - lo + 1) * step, depth

    @staticmethod
    def _front_min(bearings, ranges, half_fov=math.radians(20)):
        vals = np.asarray(ranges)[np.abs(bearings) <= half_fov]
        return float(vals.min()) if vals.size else float('inf')

    @staticmethod
    def _side_min(bearings, ranges, centre_deg, half_fov=math.radians(35)):
        c = math.radians(centre_deg)
        d = (np.asarray(bearings) - c + math.pi) % (2 * math.pi) - math.pi
        vals = np.asarray(ranges)[np.abs(d) <= half_fov]
        return float(vals.min()) if vals.size else float('inf')

    def open_extent(self, bearings, ranges, sides=None):
        """(depth_ahead, width) of the open area, in metres.

        Used to bound the delivery zone from observation instead of asserting
        it (geometry audit A7, A8). Depth is the clear distance dead ahead;
        width is the span between the nearest returns to either side.
        """
        depth = self._front_min(bearings, ranges, math.radians(15))
        # `sides`: corridor_open's (left, right), the same two minima.
        left, right = sides if sides is not None else (
            self._side_min(bearings, ranges, 90.0),
            self._side_min(bearings, ranges, -90.0))
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
        if not len(ranges):
            return
        is_open, left_m, right_m = self.corridor_open(bearings, ranges)
        self._open_depth, self._open_width = self.open_extent(
            bearings, ranges, (left_m, right_m))
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
        # NEVER MORE THAN `yaw_follow_limit_deg` OFF THE CORRIDOR'S AXIS. With
        # the gap search opened to +/- 90 deg, facing every sideways gap in a
        # slalom turned the aircraft round a step at a time until it flew back
        # out of the corridor entrance. Facing the gap is still right -- a
        # rotated corridor, a diagonal slip past a block -- only the heading it
        # asks for is clamped to the axis the traversal started on.
        gain = float(self._g('yaw_align_gain'))
        cap = float(self._g('max_yaw_rate'))
        yaw = getattr(self, "_yaw", None)
        if yaw is None:
            return max(-cap, min(cap, gain * float(bearing)))
        if getattr(self, "_axis", None) is None:
            self._axis = yaw
        wrap = lambda a: math.atan2(math.sin(a), math.cos(a))   # noqa: E731
        lim = math.radians(float(self._g('yaw_follow_limit_deg')))
        off = max(-lim, min(lim, wrap(yaw + float(bearing) - self._axis)))
        rate = gain * wrap(self._axis + off - yaw)
        return max(-cap, min(cap, rate))

    def _publish(self, vx, vy, front, bearing, extra):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = FRAME_BODY_OFFSET_NED
        sp.type_mask = VEL_YAWRATE_MASK
        sp.velocity.x = float(vx)      # forward
        sp.velocity.y = float(vy)      # LEFT (MAVROS converts FLU -> FRD)
        vz = float(self._alt_correction())
        yaw_rate = float(self._yaw_rate_for(bearing))
        sp.velocity.z = vz             # CLOSED-LOOP hold
        sp.yaw_rate = yaw_rate
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
                  "cmd_vz": round(vz, 2),
                  "cmd_yaw_rate": round(yaw_rate, 2),
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
        if not len(ranges):
            self._publish(0.0, 0.0, 0.0, 0.0, {"fault": "no valid returns"})
            return

        front = self._front_min(bearings, ranges)
        gap = self.find_gap(bearings, ranges)

        is_open, left_m, right_m = self.corridor_open(bearings, ranges)
        self._open_depth, self._open_width = self.open_extent(
            bearings, ranges, (left_m, right_m))
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
        # The strip clearance along the chosen heading IS the reach; the
        # dead-ahead minimum could overstate it once the heading is off-axis.
        reach = depth
        if reach <= stop:
            speed = 0.0
        elif reach >= brake:
            speed = self.cruise
        else:
            speed = self.cruise * (reach - stop) / max(1e-3, brake - stop)

        vx = speed * math.cos(bearing)
        # POSITIVE vy is LEFT and a positive bearing is to the left in ROS
        # LaserScan convention, so the sign is direct — asserted in tests.
        # Travel along the chosen heading. The old extra `gain * bearing`
        # push is gone: with a physical heading it only overshot the strip.
        vy = speed * math.sin(bearing)
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
