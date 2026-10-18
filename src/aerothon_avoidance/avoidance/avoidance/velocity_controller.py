#!/usr/bin/env python3
"""Corridor navigation over connected lidar-observed free space.

All paths and recovery velocities enforce 0.70 m centre-to-surface clearance.
Unobserved reverse paths, stale poses/scans and exhausted recovery stop.
MAVROS body velocities use FLU: +x forward, +y left, +z up.
No venue geometry or simulator truth is consumed here.
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import PositionTarget
from avoidance.fast_connected_planner_candidate import FastConnectedPlanner as LocalPlanner
from avoidance.scan_geometry import register_rays
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from tf2_msgs.msg import TFMessage

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
        p('body_frame', 'base_link')
        p('rate_hz', 20.0)
        p('cruise_speed', 0.8)
        p('motion_computation_budget_s', 0.20)
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
        # Hard centre-to-surface clearance, also enforced by LocalPlanner.
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
        p('steer_min_step_m', 0.3)        # a first waypoint nearer than this is a stub (steer_point)
        # ---- altitude hold ----
        # `velocity.z = 0.0` is a command for zero vertical RATE, not for a
        # held altitude: any thrust bias or disturbance integrates into a sink.
        # Arena regression seed 1002 sank from 2.8 m to 0.9 m in eight seconds
        # while stalled in the corridor, and the mission's altitude-band guard
        # (correctly) aborted. The loop has to be closed.
        p('alt_hold_gain', 0.8)       # m/s of vz per metre of error
        p('max_vz', 0.6)              # ceiling on the correction
        # Compare measured world velocity with the lagged command response.
        # Keep the observer in world coordinates as aircraft yaw changes.
        # These response/filter settings have simulation evidence; hardware
        # response and pose noise remain bench qualification items.
        p('vel_response_s', 0.6)
        # Rendered FCU identification found ~0.38 s command transport delay.
        # This simulation setting must be remeasured before hardware flight.
        p('vel_command_delay_s', 0.4)
        p('drift_filter_s', 0.05)
        p('drift_comp_gain', 1.0)
        p('max_drift_comp', 1.5)
        p('backoff_ticks', 40)
        p('backoff_speed', 0.35)
        p('max_backoffs', 3)
        p('braking_accel_mps2', 0.5)

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
        self._fcu_vz = None
        self._alt_motion_origin = None
        self._alt_motion_previous = None
        self._alt_motion_displacement = 0.0
        self._alt_motion_fault = ''
        self._yaw = None            # current heading, from the pose
        self._axis = None           # the corridor axis this traversal started on
        self._progress_ref = None
        self._progress_t = None
        self._origin = None
        self._max_progress = 0.0
        self._entered_progress = None
        self._pose_t = None
        self._fcu_velocity = None
        self._fcu_velocity_t = None
        self._ground_velocity_history = deque(maxlen=100)
        self._velocity_fault = ''
        self._measured_speed = 0.0
        self._pose_history = deque(maxlen=100)
        self._scan_pose = None
        self._scan_orientation = None
        self._tf_buffer = self._tf_listener = None
        self._mount_frames = {str(self._g('body_frame'))}
        self._planner = LocalPlanner(float(self._g('passage_half_width')),
                                     occupied_radius=float(self._g('airframe_radius')))
        self._path_world = []
        self._plan_t = None
        self._open_ticks = 0
        # You cannot exit a corridor you were never inside. Without this the
        # detector fired on the open apron BEFORE the corridor: a live run
        # reported "corridor opened out" one second after entering the stage,
        # at x = 1.2 m, and the mission then flew a delivery-zone search on
        # the takeoff pad and reported COMPLETED. The hardcoded GotoZone
        # waypoint had been dragging it to the real zone and hiding this.
        self._enclosed_ticks = 0
        self._entered = False
        self._reset_drift()

        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avoidance/enable', self._on_enable, 10)
        self.intent_enabled = False
        self._intent_speed_cap = self.cruise
        self._intent = None
        self._intent_t = None
        self._intent_steady = None
        self.create_subscription(Bool, '/avoidance/intent_enabled', self._on_intent_enable, 10)
        self.create_subscription(Vector3, '/avoidance/intent', self._on_intent, 10)
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

    def _on_intent_enable(self, m):
        if self.intent_enabled != m.data:
            self._axis = None
            self._origin = None
            self._planner.path = []
            self._intent = None
        self.intent_enabled = m.data

    def _on_intent(self, m):
        self._intent = m
        self._intent_t = self._now()
        self._intent_steady = time.monotonic()

    def _on_enable(self, m: Bool):
        if m.data and not self.enabled:
            self._alt_motion_origin = self._alt_motion_previous = None
            self._alt_motion_displacement = 0.0
            self._alt_motion_fault = ''
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
            self._origin = self._pos
            self._max_progress = 0.0
            self._entered_progress = None
            self._path_world = []
            self._plan_t = None
            self._last_gap_bearing = None
            self._reset_drift()
            if self._velocity_fresh():
                # Existing EKF motion is the initial response state, not a
                # newly discovered disturbance at control handover.
                self._ground_world = self._fcu_velocity.copy()
                self._expect_world = self._ground_world.copy()
                self._model_t = self._fcu_velocity_t
                self._drift_prev,self._drift_prev_t = self._pos,self._fcu_velocity_t
            # Latch the altitude to hold for this traversal. The mission can
            # override it on /avoidance/hold_alt; without that, whatever the
            # aircraft was flying at when avoidance was handed control is the
            # honest default.
            if self._alt is not None:
                self._hold_alt = self._alt
                if (math.isfinite(self._alt) and self._velocity_fresh()
                        and self._pose_t is not None
                        and 0 <= self._now()-self._pose_t <= float(self._g('stale_scan_s'))
                        and self._fcu_vz is not None):
                    self._alt_motion_origin = self._alt
                    self._alt_motion_previous = (self._now(), self._fcu_vz)
        if not m.data:
            self._hold_alt = None
            self._alt_motion_origin = self._alt_motion_previous = None
            self._alt_motion_displacement = 0.0
            self._alt_motion_fault = ''
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
        if not all(math.isfinite(value) for value in (self._hold_alt,self._alt)):
            return 0.0
        if self._alt_motion_origin is not None:
            if not self._velocity_fresh() or self._alt_motion_fault:
                return 0.0
            # Preserve the physical handover height. Differencing barometer
            # fixes turns estimator wander into actual climbing or sinking.
            altitude = self._alt_motion_origin+self._alt_motion_displacement
        elif self._fcu_velocity_t is not None:
            # A received but unusable motion stream cannot fall back to
            # trusting changing barometer fixes as physical displacement.
            return 0.0
        else:
            altitude = self._alt
        err = self._hold_alt - altitude
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
        now = self._now()
        stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        previous_t = self._pose_t
        previous = self._pos
        self._pos = (m.pose.position.x, m.pose.position.y)
        self._pose_t = min(now, stamp) if stamp > 0 else now
        if previous is not None and previous_t is not None and now > previous_t:
            self._measured_speed = math.dist(previous, self._pos)/(now-previous_t)
        # z was thrown away here, which is why "hold altitude" could never be
        # more than a comment: the controller did not know its own altitude.
        self._alt = float(m.pose.position.z)
        q = m.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose_history.append((self._pose_t,*self._pos,self._yaw,(q.x,q.y,q.z,q.w)))

    def _velocity_fresh(self):
        return (not self._velocity_fault and not self._alt_motion_fault
                and self._fcu_velocity_t is not None
                and 0 <= self._now()-self._fcu_velocity_t <= float(self._g('stale_scan_s')))

    def _on_velocity(self, message: TwistStamped):
        # MAVROS velocity_local values are local ENU even when the header
        # names base_link. These values must not be rotated into body axes.
        now = self._now()
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        velocity = np.array([message.twist.linear.x,message.twist.linear.y])
        vz = float(message.twist.linear.z)
        if not np.isfinite(velocity).all() or not math.isfinite(vz) or stamp < 0 or stamp > now+.1:
            self._velocity_fault = 'invalid FCU velocity observation'
            if self.enabled and self._alt_motion_origin is not None:
                self._alt_motion_fault = 'invalid vertical motion capture'
            return
        stamp = min(now,stamp) if stamp > 0 else now
        if self._fcu_velocity_t is not None and stamp <= self._fcu_velocity_t:
            return
        if self.enabled and self._alt_motion_previous is not None:
            previous_t, previous_vz = self._alt_motion_previous
            if stamp > previous_t:
                dt = stamp-previous_t
                if dt > 3*float(self._g('stale_scan_s')):
                    self._alt_motion_fault = 'vertical motion capture gap'
                elif not self._alt_motion_fault:
                    # A gap up to 3x stale_scan_s is bridged by the two FCU
                    # velocities either side of it (still no barometer
                    # trust). Latching every gap also made _velocity_fresh()
                    # false for the rest of the traversal: one telemetry
                    # hiccup (CPU load in SITL; a busy 115200 TELEM link on the
                    # aircraft) froze the aircraft in the corridor until the
                    # battery failed (16 Oct campaign).
                    self._alt_motion_displacement += .5*dt*(previous_vz+vz)
                self._alt_motion_previous = (stamp,vz)
        self._fcu_vz = vz
        self._fcu_velocity,self._fcu_velocity_t = velocity,stamp
        self._ground_velocity_history.append((stamp,velocity.copy()))
        self._velocity_fault = ''

    def _scan_ground_displacement(self):
        """Short capture-to-pose motion without differencing noisy GPS fixes."""
        start,end = self._scan_t,self._pose_t
        if start is None or end is None or end < start:
            raise ValueError('lidar motion capture interval unavailable')
        if not self._velocity_fresh():
            raise ValueError('ground velocity capture stale')
        if end == start:
            return np.zeros(2)
        history = list(self._ground_velocity_history)
        if (not history or start < history[0][0] or
                end-history[-1][0] > .06+1e-9):
            raise ValueError('ground velocity does not cover lidar capture')
        times = np.array([t for t,v in history])
        velocities = np.array([v for t,v in history])
        # The last velocity may precede pose delivery by at most one nominal
        # FCU capture interval. Longer gaps are unknown, never GPS movement.
        knots = np.concatenate(([start],times[(times>start)&(times<end)],[end]))
        for a,b in zip(times,times[1:]):
            if a < end and b > start and b-a > .12:
                raise ValueError('ground velocity capture history has a gap')
        values = np.column_stack([np.interp(knots,times,velocities[:,axis]) for axis in (0,1)])
        return np.sum((values[:-1]+values[1:])*.5*np.diff(knots)[:,None],axis=0)

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
        axis = self._axis if self._axis is not None else self._yaw
        axis = axis if axis is not None else 0.0
        moved = ((self._pos[0]-self._progress_ref[0])*math.cos(axis)
                 +(self._pos[1]-self._progress_ref[1])*math.sin(axis))
        if moved < float(self._g('min_progress_m')):
            # Keep this expired reference until movement earns a new window.
            # Resetting it on every stall produced a single blocked tick per
            # window; CRUISE cleared that tick before recovery could begin.
            return True
        self._progress_ref, self._progress_t = self._pos, now
        return False

    def _on_scan(self, m: LaserScan):
        self.scan = m
        stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        self._scan_t = min(self._now(), stamp) if stamp > 0 else self._now()
        self._scan_pose = None
        self._scan_orientation = None
        if self._pose_history:
            closest = min(self._pose_history,key=lambda p:abs(p[0]-self._scan_t))
            if abs(closest[0]-self._scan_t)<=.2:
                self._scan_pose = closest[1:4]
                self._scan_orientation = closest[4]

    def _registered_scan(self, bearings, ranges):
        if self._scan_pose is None or self._scan_orientation is None:
            raise ValueError('capture-time aircraft pose unavailable')
        frame = self.scan.header.frame_id
        if not frame:
            raise ValueError('lidar frame missing')
        if frame == self._g('body_frame'):
            translation, quaternion = (0,0,0), (0,0,0,1)
        else:
            if self._tf_buffer is None:
                self._tf_buffer = Buffer()
                self._tf_listener = self.create_subscription(
                    TFMessage, '/tf_static', self._on_static_mounts,
                    QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._g('body_frame'), frame, Time(seconds=self._scan_t)).transform
            except TransformException as error:
                raise ValueError(f'lidar mount {self._g("body_frame")} <- {frame} unavailable: {error}') from error
            t,q = transform.translation, transform.rotation
            translation, quaternion = (t.x,t.y,t.z), (q.x,q.y,q.z,q.w)
        current_xy = self._pos
        self._scan_motion_basis = 'paired FCU pose fallback'
        self._scan_motion_world = None
        if self._fcu_velocity_t is not None:
            displacement = self._scan_ground_displacement()
            current_xy = np.asarray(self._scan_pose[:2])+displacement
            self._scan_motion_basis = 'fresh FCU world ground velocity captures'
            self._scan_motion_world = displacement.tolist()
        return register_rays(bearings,ranges,translation,quaternion,
                             self._scan_pose[:2],self._scan_orientation,
                             current_xy,self._yaw)

    def _on_static_mounts(self, message):
        # Only relative equipment calibration below the aircraft body belongs
        # in this buffer. Never subscribe to dynamic TF or install an absolute
        # vehicle pose. Aircraft motion comes exclusively from the FCU pose.
        forbidden={'world','map','odom',str(self._g('body_frame'))}
        pending=[t for t in message.transforms if t.child_frame_id not in forbidden]
        while pending:
            remaining=[]
            for transform in pending:
                if transform.header.frame_id in self._mount_frames:
                    self._tf_buffer.set_transform_static(transform,'onboard_equipment_mount')
                    self._mount_frames.add(transform.child_frame_id)
                else:
                    remaining.append(transform)
            if len(remaining)==len(pending):break
            pending=remaining

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
        return gap

    def _strips(self, bearings, ranges):
        """Every candidate heading against every return, for _find_gap.

        Rows are headings -n..n in 2 degree steps, columns are returns. The
        caller always uses the full passage width.
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
        hw = np.full_like(r_1, half_w)
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

    def _reset_drift(self):
        self._drift = 0.0
        self._response_known = False
        self._expect_world = np.zeros(2)
        self._wind_world = np.zeros(2)
        self._wind_rate_world = np.zeros(2)
        self._wind_innovation_world = np.zeros(2)
        self._ground_world = np.zeros(2)
        self._drift_prev = None
        self._drift_prev_t = None
        self._model_t = None
        self._last_command_world = np.zeros(2)
        self._command_history = deque(maxlen=400)
        self._sent_heading = {}
        self._yaw_command_history = deque(maxlen=400)

    def _heading_at(self, stamp):
        """Observed past heading or a queued-command forecast, from FCU only."""
        history = self._pose_history
        if not history:
            return self._yaw or 0.0
        latest = history[-1]
        if stamp <= latest[0]:
            right = latest
            for left in reversed(history):
                if left[0] <= stamp:
                    if right[0] <= left[0]:
                        return left[3]
                    delta = math.atan2(math.sin(right[3]-left[3]),
                                       math.cos(right[3]-left[3]))
                    return left[3]+delta*(stamp-left[0])/(right[0]-left[0])
                right = left
            return history[0][3]
        rate = 0.0
        for previous in reversed(history):
            span = latest[0]-previous[0]
            if span > 1e-6:
                rate = math.atan2(math.sin(latest[3]-previous[3]),
                                  math.cos(latest[3]-previous[3]))/span
                break
        heading, at = latest[3], latest[0]
        delay = float(self._g('vel_command_delay_s'))
        for sent, target_rate in self._yaw_command_history:
            arrival = sent+delay
            if latest[0] < arrival <= stamp:
                heading += rate*(arrival-at)
                at, rate = arrival, target_rate
        return heading+rate*(stamp-at)

    def _command_known(self, stamp):
        """Has an own velocity packet reached the FCU under the declared delay?"""
        return bool(self._command_history) and self._command_history[0][0]+float(
            self._g('vel_command_delay_s'))<=stamp+1e-9

    def _effective_command(self, stamp):
        """Previously transmitted world command that reaches the FCU now."""
        cutoff = stamp-float(self._g('vel_command_delay_s'))
        for sent,command in reversed(self._command_history):
            if sent<=cutoff+1e-9:
                # The stored earth vector uses heading at transmission.
                # ArduPilot instead converts its body vector at receipt.
                issued_heading = self._sent_heading.get(sent)
                if issued_heading is None:
                    issued_heading = self._heading_at(sent)
                delta = self._heading_at(sent+float(self._g('vel_command_delay_s')))-issued_heading
                c,s = math.cos(delta),math.sin(delta)
                return np.array([c*command[0]-s*command[1],
                                 s*command[0]+c*command[1]])
        return np.zeros(2)

    def _drift_comp(self, vy):
        # Kept for callers that inspect lateral compensation.
        return self._compensated(0.0,vy)[1]

    def _compensated(self, vx, vy):
        if self._pos is None or self._yaw is None:
            return vx,vy
        now=self._now()
        use_velocity = self._velocity_fresh()
        measured_t=self._fcu_velocity_t if use_velocity else self._pose_t if self._pose_t is not None else now
        # The EKF sample describes motion at its header stamp. Comparing it
        # with a response already advanced to receipt time invents a wind
        # pulse during every acceleration and reverses its sign when braking.
        # Repeated samples must not advance the observer's response state.
        dt=0.0 if self._model_t is None else max(0,measured_t-self._model_t)
        # The preceding position controller retains its target until our
        # first packet arrives. Its target response is not a zero command.
        # Start residual assessment after a measured seed at that boundary.
        response_known=self._model_t is not None and self._command_known(self._model_t)
        self._response_known=response_known
        # Integrate delayed commands at their actual arrival boundaries.
        # Comparing current motion with the newest transmitted command makes
        # actuator lag look like wind and closes an unstable correction loop.
        if response_known and dt>0:
            delay=float(self._g('vel_command_delay_s'))
            boundaries=[self._model_t]+[sent+delay for sent,_ in self._command_history
                if self._model_t<sent+delay<measured_t]+[measured_t]
            for start,end in zip(boundaries,boundaries[1:]):
                command=self._effective_command((start+end)/2)
                fraction=min(1.0,(end-start)/float(self._g('vel_response_s')))
                self._expect_world += (command-self._expect_world)*fraction
        self._model_t=measured_t
        if use_velocity:
            self._ground_world = self._fcu_velocity.copy()
        if self._drift_prev is None:
            self._drift_prev,self._drift_prev_t = self._pos,measured_t
        elif measured_t>self._drift_prev_t:
            span = measured_t-self._drift_prev_t
            if not use_velocity:
                self._ground_world = (np.array(self._pos)-self._drift_prev)/span
            if response_known:
                residual = self._ground_world-self._expect_world
                f = min(1.0,span/float(self._g('drift_filter_s')))
                previous = self._wind_world.copy()
                self._wind_world += (residual-self._wind_world)*f
                self._wind_innovation_world = residual-self._wind_world
                rate = (self._wind_world-previous)/span
                rate *= min(1.0,.8/max(1e-6,float(np.linalg.norm(rate))))
                self._wind_rate_world += (rate-self._wind_rate_world)*f
            self._drift_prev,self._drift_prev_t = self._pos,measured_t
        if not response_known:
            self._expect_world=self._ground_world.copy()
            self._wind_world=np.zeros(2);self._wind_rate_world=np.zeros(2)
            self._wind_innovation_world=np.zeros(2)
        c,s = math.cos(self._yaw),math.sin(self._yaw)
        wind = np.array([[c,s],[-s,c]]) @ self._wind_world
        self._drift = float(wind[1])
        rate = np.array([[c,s],[-s,c]]) @ self._wind_rate_world
        prediction = float(self._g('vel_response_s'))
        compensation = float(self._g('drift_comp_gain'))*(wind+prediction*rate)
        cap = float(self._g('max_drift_comp'))
        compensation *= min(1.0,cap/max(1e-6,float(np.linalg.norm(compensation))))
        desired = np.array([vx,vy])
        # Act only on the disturbance innovation, not the ordinary command
        # response lag. Startup must retain the configured cruise speed.
        feedback = -.7*(np.array([[c,s],[-s,c]]) @ self._wind_innovation_world)
        feedback *= min(1.0,.8/max(1e-6,float(np.linalg.norm(feedback))))
        output = desired-compensation+feedback
        return float(output[0]),float(output[1])

    def _publish(self, vx, vy, front, bearing, extra):
        computation_started = self._now()
        budget = float(self._g('motion_computation_budget_s'))
        if not math.isfinite(budget) or budget < 0:
            raise ValueError('motion computation budget must be finite and nonnegative')
        extra = dict(extra)
        if 'fault' not in extra and (self.scan_stale() or
                self._pose_t is None or self._now()-self._pose_t > float(self._g('stale_scan_s')) or
                self._velocity_fault or (self._fcu_velocity_t is not None and not self._velocity_fresh())):
            extra['fault'] = 'post-planning flight observations stale'
        fresh = 'fault' not in extra
        desired = (vx,vy)
        yaw_rate = float(self._yaw_rate_for(bearing)) if fresh and abs(vx)+abs(vy)>1e-6 else 0.0
        if getattr(self, 'intent_enabled', False) and fresh and self._intent is not None:
            yaw_rate = max(-float(self._g('max_yaw_rate')), min(float(self._g('max_yaw_rate')), self._intent.z))
        if fresh:
            vx,vy = self._compensated(vx,vy)
            c,s = math.cos(self._yaw or 0.0),math.sin(self._yaw or 0.0)
            rotation = np.array([[c,s],[-s,c]])
            ground = rotation @ self._ground_world
            wind = rotation @ self._wind_world
            wind_rate = rotation @ self._wind_rate_world
            delay = float(self._g('vel_command_delay_s'))
            receipt_turn = self._heading_at(self._now()+budget+delay)-(self._yaw or 0.0)
            def disturbance_motion_clear(command, wind, wind_rate):
                forecast_now = self._now()
                # Preserve the supported FCU-pose finite-difference fallback.
                # Once an EKF velocity stream exists, stale captures still stop.
                sample_epoch = min(self._pose_t, self._fcu_velocity_t) if self._fcu_velocity_t is not None else self._pose_t
                capture_age = max(0., forecast_now-sample_epoch)
                packet_wait = capture_age+budget+float(self._g('vel_command_delay_s'))
                # Predict velocity-loop lag from measured ground velocity,
                # followed by braking. Motor compensation is not travel.
                target = np.array(command)+wind
                velocity = ground.copy()
                point = np.zeros(2)
                trajectory = [point.copy()]
                dt = .05
                tau = float(self._g('vel_response_s'))
                delay = float(self._g('vel_command_delay_s'))
                # Check the lagged braking tail too. One response time leaves
                # 37% of incoming velocity, which can still cross a surface.
                braking = float(self._g('braking_accel_mps2'))
                duration = packet_wait+6*tau+max(float(np.linalg.norm(target)),
                                    float(np.linalg.norm(ground)))/braking
                for t in np.arange(0,max(.1,duration),dt):
                    if t<packet_wait:
                        # A stop cannot cancel commands already in transit.
                        if self._command_known(sample_epoch+t):
                            goal = rotation @ self._effective_command(sample_epoch+t)+wind+t*wind_rate
                        else:
                            # Retain observed incoming motion while the prior
                            # position target is unknown. Zero would shorten
                            # the predicted stopping distance at handover.
                            goal = ground.copy()
                    else:
                        # Forecast compensation counters a changing measured
                        # disturbance. Testing that command against constant
                        # wind rejects it as overshoot and loses cancellation
                        # during a rising gust. Bound the forecast to the same
                        # command/response interval, then certify the full
                        # braking tail as before.
                        if t<packet_wait+tau:
                            # ArduPilot converts each body velocity to earth
                            # coordinates at receipt. Turning during transport
                            # changes that conversion; the received vector
                            # then stays fixed during its response interval.
                            angle=receipt_turn
                            co,si=math.cos(angle),math.sin(angle)
                            turning=np.array([[co,-si],[si,co]]) @ np.array(command)
                            goal = turning+wind+t*wind_rate
                        else:
                            goal = np.zeros(2)
                    change = (goal-velocity)*min(1.0,dt/tau)
                    if t>=packet_wait+tau:
                        # The declared braking acceleration is a bound, not
                        # just a horizon. A fast exponential response must
                        # not shorten the required stopping distance.
                        change *= min(1.0,braking*dt/max(1e-6,float(np.linalg.norm(change))))
                    velocity += change
                    nxt = point+velocity*dt
                    trajectory.append(nxt.copy())
                    point = nxt
                return self._planner.polyline_clear(trajectory)
            def motion_clear(command):
                # A response mismatch may masquerade as helpful disturbance.
                # Require clearance for the zero-disturbance prediction too.
                return (disturbance_motion_clear(command, wind, wind_rate)
                        and disturbance_motion_clear(command, np.zeros(2), np.zeros(2)))
            if not motion_clear((vx,vy)):
                # Reduce travel while retaining the disturbance cancellation.
                accepted = False
                for scale in (.75,.5,.25,0.0):
                    candidate = (vx-desired[0]*(1-scale),vy-desired[1]*(1-scale))
                    if motion_clear(candidate):
                        vx,vy = candidate
                        accepted = True
                        break
                if not accepted:
                    # A rising-gust forecast can over-correct toward the
                    # opposite surface. Retain cancellation of the currently
                    # measured disturbance only if its entire ground-motion
                    # prediction is certified. Raw zero loses that cancellation.
                    cap = float(self._g('max_drift_comp'))
                    for hold,source in (
                        (-wind.copy(),'certified current disturbance cancellation'),
                        (-wind-np.array([0,ground[1]]),'certified active disturbance braking'),
                        (-wind-np.array([ground[0],0]),'certified active disturbance braking'),
                        (-wind-ground,'certified active disturbance braking'),
                    ):
                        hold *= min(1.0,cap/max(1e-6,float(np.linalg.norm(hold))))
                        if motion_clear(hold):
                            vx,vy = map(float,hold)
                            accepted = True
                            extra['hold_source']=source
                            break
                if not accepted:
                    # A blocked planner still sends hold compensation. It must
                    # pass exactly the same motion check as a travel command.
                    # If no compensated hold is certified, request FCU braking
                    # rather than publishing an unchecked drift correction.
                    vx=vy=0.0
                    extra['braking_clearance_available']=motion_clear((0,0))
                    yaw_rate=0.0
                extra['recovery_reason'] = 'travel limited by observed braking clearance'
            # Update the actuator model with the command actually published.
            command_world = np.array([[c,-s],[s,c]]) @ np.array([vx,vy])
            self._last_command_world=command_world
        else:
            vx=vy=0.0
            pending=self._command_history.copy()
            headings=self._sent_heading.copy()
            yaw_pending=self._yaw_command_history.copy()
            # A lidar fault prohibits motion, but it does not invalidate
            # independently fresh FCU motion samples. Erasing the observer
            # here makes the next scan treat measured wind as a new handover.
            flight_fresh = (self._pose_t is not None and
                self._now()-self._pose_t <= float(self._g('stale_scan_s')) and
                not self._velocity_fault and self._velocity_fresh())
            if flight_fresh:
                self._compensated(0.0,0.0)
            else:
                self._reset_drift()
            self._command_history=pending
            self._sent_heading=headings
            self._yaw_command_history=yaw_pending
        computation_elapsed = max(0.,self._now()-computation_started)
        if fresh and computation_elapsed > budget+1e-9:
            vx=vy=yaw_rate=0.0
            fresh=False
            extra['fault']='motion certificate computation deadline exceeded'
        elif fresh and (self.scan_stale() or self._pose_t is None or
                self._now()-self._pose_t > float(self._g('stale_scan_s')) or
                self._velocity_fault or (self._fcu_velocity_t is not None and not self._velocity_fresh())):
            vx=vy=yaw_rate=0.0
            fresh=False
            extra['fault']='post-certificate flight observations stale'
        extra['motion_computation_elapsed_s']=computation_elapsed
        extra['motion_computation_budget_s']=budget
        # Include holds in the delayed actuator model, including stale-data
        # braking. Resetting estimation must not erase commands in transit.
        c,s=math.cos(self._yaw or 0),math.sin(self._yaw or 0)
        sent_at=self._now()
        if len(self._command_history)==self._command_history.maxlen:
            self._sent_heading.pop(self._command_history[0][0],None)
        self._command_history.append((sent_at,np.array([c*vx-s*vy,s*vx+c*vy])))
        self._sent_heading[sent_at]=self._yaw or 0.0
        self._yaw_command_history.append((sent_at,yaw_rate))
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = FRAME_BODY_OFFSET_NED
        sp.type_mask = VEL_YAWRATE_MASK
        sp.velocity.x = float(vx)      # forward
        sp.velocity.y = float(vy)      # LEFT (MAVROS converts FLU -> FRD)
        vz = float(self._alt_correction())
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
                  "altitude_motion_basis": ('FCU vertical displacement from handover'
                      if self._alt_motion_origin is not None else 'pose-only; no FCU motion anchor'),
                  "altitude_motion_displacement_m": self._alt_motion_displacement,
                  "altitude_motion_fault": self._alt_motion_fault,
                  "backoffs": self._backoffs_done,
                  "disturbance_estimate_valid": bool(fresh and self._response_known),
                  "disturbance_world_mps": self._wind_world.tolist() if fresh and self._response_known else None,
                  "disturbance_rate_world_mps2": self._wind_rate_world.tolist() if fresh and self._response_known else None,
                  "command_response_state": 'observed own velocity response' if fresh and self._response_known else 'unknown prior control response',
                  "ground_velocity_world_mps": self._ground_world.tolist() if fresh else None,
                  "ground_velocity_basis": 'unavailable during sensor fault' if not fresh else 'fresh FCU EKF velocity_local' if self._velocity_fresh() else 'FCU pose finite difference estimate',
                  "fcu_velocity_age_s": None if self._fcu_velocity_t is None else self._now()-self._fcu_velocity_t,
                  "command_delay_s": float(self._g('vel_command_delay_s')),
                  "commanded_yaw_rate_rad_s": yaw_rate,
                  "command_receipt_turn_rad": float(receipt_turn) if fresh else None,
                  "drift_left_mps": round(self._drift, 2)}
        detail.update(planner_state=self._planner.reason,
                      required_clearance_m=self._planner.clearance,
                      observed_clearance_m=float(self._planner.distances([(0,0)])[0]) if self._planner.tree is not None else None,
                      scan_motion_basis=getattr(self,'_scan_motion_basis','unavailable'),
                      scan_motion_world_m=getattr(self,'_scan_motion_world',None),
                      scan_age_s=None if self._scan_t is None else round(self._now()-self._scan_t,3),
                      pose_age_s=None if self._pose_t is None else round(self._now()-self._pose_t,3),
                      planner_path_body_m=self._planner.path,
                      axis_progress_m=round(self._axis_progress(),3))
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

        if getattr(self, 'intent_enabled', False):
            m = self._intent
            if m is None or self._intent_t is None or self._intent_steady is None or self._now()-self._intent_t > .5 or time.monotonic()-self._intent_steady > .5 or not all(math.isfinite(x) for x in (m.x,m.y,m.z)):
                self._publish(0,0,0,0,{'fault':'pilot intent stale or invalid'})
                return
            self.cruise = min(self._intent_speed_cap, math.hypot(m.x,m.y))
            self._axis = self._yaw + math.atan2(m.y,m.x)

        if self.scan is None or self.scan_stale():
            # Never coast on stale data: an unknown world is a stop, not a
            # continuation of the last command.
            self._publish(0.0, 0.0, 0.0, 0.0,
                          {"fault": "scan stale or missing"})
            return

        if self._pose_t is None or self._now()-self._pose_t > float(self._g('stale_scan_s')):
            self._publish(0.0,0.0,0.0,0.0,{'fault':'pose stale or missing'})
            return

        if self._velocity_fault or (self._fcu_velocity_t is not None and not self._velocity_fresh()):
            self._publish(0.0,0.0,0.0,0.0,{'fault':self._velocity_fault or 'FCU velocity stale'})
            return

        bearings, ranges = self.conditioned(self.scan)
        if not len(ranges):
            self._publish(0.0, 0.0, 0.0, 0.0, {"fault": "no valid returns"})
            return
        if any(math.isnan(r) or r <= 0 for r in self.scan.ranges):
            self._publish(0.0,0.0,0.0,0.0,{'fault':'scan contains unknown or invalid ranges'})
            return

        front = self._front_min(bearings, ranges)
        hits = ranges < float(self._g('range_max_valid'))-0.001
        try:
            sensor_origin,bearings,ranges = self._registered_scan(bearings,ranges)
        except (ValueError, TransformException) as error:
            self._publish(0,0,front,0,{'fault':str(error)})
            return
        front = self._front_min(bearings,ranges)
        if self._axis is None:
            self._axis = self._yaw
        if self._origin is None:
            self._origin = self._pos
        self._planner.observe(bearings,ranges,float(self._g('range_max_valid')),
                              sensor_origin=sensor_origin,hits=hits)
        if self.state == 'STUCK':
            self._publish(0.0,0.0,front,0.0,{'recovery_reason':'recovery exhausted'})
            return
        axis_bearing = math.atan2(math.sin(self._axis-self._yaw),math.cos(self._axis-self._yaw))
        path = self._local_path(axis_bearing)
        gap = None
        if len(path)>1:
            x,y = steer_point(path, self._planner.segment_clear,
                              float(self._g('steer_min_step_m')))
            gap = (math.atan2(y,x),0.0,math.hypot(x,y)+float(self._g('stop_dist')))

        is_open, left_m, right_m = self.corridor_open(bearings, ranges)
        self._open_depth, self._open_width = self.open_extent(
            bearings, ranges, (left_m, right_m))
        self._open_ticks = self._open_ticks + 1 if is_open else 0
        # Entry must be observed before exit can mean anything.
        self._enclosed_ticks = 0 if is_open else self._enclosed_ticks + 1
        if self._enclosed_ticks >= int(self._g('corridor_enter_ticks')):
            self._entered = True
            if self._entered_progress is None:
                self._entered_progress = self._axis_progress()
        progress = self._axis_progress()
        self._max_progress = max(self._max_progress,progress)
        exited = (self._entered
                  and self._open_ticks >= int(self._g('corridor_open_ticks'))
                  and self._entered_progress is not None
                  and progress > self._entered_progress+float(self._g('min_progress_m'))
                  and progress >= self._max_progress-0.2)
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
                # Give this bounded recovery attempt a new progress window.
                # The attempt count stays cumulative; STUCK stays terminal.
                self._progress_ref,self._progress_t = self._pos,self._now()
            reverse = -float(self._g('backoff_speed'))
            if not self._planner.segment_clear((0,0),(-0.8,0)):
                reverse = 0.0
            self._publish(reverse, 0.0, front, 0.0,
                          {"recovery_reason": "observed reverse path" if reverse else 'reverse path blocked or unobserved'})
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
        # LaserScan convention, so the sign is direct â€” asserted in tests.
        # Travel along the chosen heading. The old extra `gain * bearing`
        # push is gone: with a physical heading it only overshot the strip.
        vy = speed * math.sin(bearing)
        vy = max(-max_lat, min(max_lat, vy))

        self._publish(vx, vy, front, bearing,
                      {"gap_width_deg": round(math.degrees(width), 1),
                       "gap_depth_m": round(depth, 2),
                       "reach_m": round(reach, 2)})

    def _axis_progress(self):
        if self._pos is None or self._origin is None or self._axis is None:
            return 0.0
        return ((self._pos[0]-self._origin[0])*math.cos(self._axis)
                +(self._pos[1]-self._origin[1])*math.sin(self._axis))

    def _local_path(self, axis_bearing):
        # Reuse only the steering hint, never its old clearance verdict.
        # Fresh observations and braking certification still run every tick.
        now = self._now()
        c,s = math.cos(self._yaw),math.sin(self._yaw)
        px,py = self._pos
        while len(self._path_world)>1 and math.dist(self._pos,self._path_world[0])<0.2:
            self._path_world.pop(0)
        body = [(c*(x-px)+s*(y-py),-s*(x-px)+c*(y-py)) for x,y in self._path_world]
        if (body and self._plan_t is not None and now-self._plan_t<0.5
                and all(self._planner.segment_clear(a,b)
                        for a,b in zip([(0,0)]+body,body))):
            self._planner.path = [(0,0)]+body
            self._planner.reason = 'freshly revalidated lidar path'
            return self._planner.path
        path = self._planner.plan(axis_bearing)
        self._plan_t = now
        self._path_world = [(px+c*x-s*y,py+s*x+c*y) for x,y in path[1:]]
        return path

    @property
    def last_detail(self):
        return self._last_detail


def steer_point(path, segment_clear, min_step_m):
    """The body-frame waypoint to steer at: path[1], unless it is a stub
    nearer than `min_step_m`, then path[2] when the straight leg to it is
    clear. Only the stub is skipped: steering past ordinary waypoints cut
    corners into the walls under gusts (corridor stress tests).

    Steering at path[1] alone stalled the clarified arena's return slalom
    (rendered, p2r10): the planner's first waypoint was 0.1 m straight ahead
    of a half-width obstacle with the turn after it, so the commanded bearing
    was 0 deg and the speed, from 0.1 m of reach, 0.05 m/s -- straight at the
    obstacle -- until "no ground covered" exhausted recovery (STUCK). Braking
    certification still applies to whatever is commanded."""
    first = path[1]
    if len(path) < 3 or math.hypot(*first) >= min_step_m:
        return first
    return path[2] if segment_clear((0.0, 0.0), path[2]) else first


def main():
    import os
    rclpy.init()
    node = None
    clock_driver = capture_driver = trace = None
    try:
        node = VelocityController()
        from avoidance.buffered_scan_pose import install
        install(node)
        from avoidance.independent_ros_clock_candidate import IndependentRosClock
        clock_driver = IndependentRosClock(node)
        from avoidance.capture_inbox import CaptureInbox
        capture_driver = CaptureInbox(node)
        recording_path = os.environ.get('AEROTHON_MOTION_FORECAST_OUT')
        if recording_path:
            from avoidance.motion_forecast_trace import MotionForecastTrace
            trace = MotionForecastTrace(node, recording_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if trace is not None:
            trace.close()
        if capture_driver is not None:
            capture_driver.close()
        if clock_driver is not None:
            clock_driver.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
