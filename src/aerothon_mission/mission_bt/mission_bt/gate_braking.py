"""Candidate gate speed lease; no simulator imports or geometric truth inputs."""
import math

import py_trees
from mission_bt.mission_tree import GateAdvance

RUNNING = py_trees.common.Status.RUNNING
FAILURE = py_trees.common.Status.FAILURE
SUCCESS = py_trees.common.Status.SUCCESS


class BrakedGateAdvance(GateAdvance):
    """Keep position control until fresh measurements show a settled aircraft.

    The fixed cap is a simulation calibration candidate. Native qualification
    and hardware rate/response qualification remain necessary before adoption.
    """

    def __init__(self, *args, gate_speed_mps=.35, arrival_speed_mps=.08,
                 settle_s=.3, stale_s=.3, service_timeout_s=3.,
                 restore_speed_mps=None, bridge_s=None, pause_s=None, **kwargs):
        super().__init__(*args, **kwargs)
        values = (gate_speed_mps, arrival_speed_mps, stale_s, service_timeout_s)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError('Gate braking parameters must be finite and positive')
        if not math.isfinite(settle_s) or settle_s < 0:
            raise ValueError('Invalid gate settling duration')
        if restore_speed_mps is not None and not (
                math.isfinite(restore_speed_mps) and restore_speed_mps > 0):
            raise ValueError('Invalid configured navigation speed')
        self.restore_speed_mps = restore_speed_mps
        self.gate_speed_mps = float(gate_speed_mps)
        self.arrival_speed_mps = float(arrival_speed_mps)
        self.settle_s = float(settle_s)
        self.stale_s = float(stale_s)
        self.service_timeout_s = float(service_timeout_s)
        # Braking captures: a gap up to bridge_s is integrated across, and a
        # transient fault is held out for up to pause_s before failing.
        self.bridge_s = float(3 * self.stale_s if bridge_s is None else bridge_s)
        self.pause_s = float(2 * self.stale_s if pause_s is None else pause_s)
        if not (math.isfinite(self.bridge_s) and self.bridge_s >= self.stale_s
                and math.isfinite(self.pause_s) and self.pause_s >= 0):
            raise ValueError('Invalid gate braking gap bridge or pause')

    def initialise(self):
        super().initialise()
        self._phase = 'ADMIT'
        self._brake_hold = None
        self._brake_yaw = None
        self._brake_motion_previous = None
        self._brake_capture_stamp = None
        self._brake_capture_fault = ''
        self._brake_fault_since = None
        self._brake_gap_fatal = False
        self._brake_displacement = [0., 0.]
        self._entry_speed = getattr(self.mav, 'speed_cmd', None)
        self._restore_basis = 'previous commanded navigation speed'
        if self._entry_speed is None and self.restore_speed_mps is not None:
            self._entry_speed = self.restore_speed_mps
            self._restore_basis = 'configured navigation speed; no previous command'
        self._lease_future = None
        self._lease_sent = None
        self._hold = None
        self._settle_first = None
        self._settle_last = None
        self._settle_count = 0
        self._started = self._now()
        self._capture_pause_started = None
        self._cross_epoch = None
        self._cross_axis = None
        self._cross_motion_previous = None
        self._cross_displacement = [0., 0.]
        self._cross_motion_valid = True

    def _now(self):
        return self.mav.node.get_clock().now().nanoseconds * 1e-9

    def _fresh_motion(self, now):
        inputs = getattr(self.mav, 'delivery_inputs', {})
        pose_stamp = inputs.get('pose_stamp')
        velocity_stamp = inputs.get('velocity_stamp')
        speed = inputs.get('speed')
        scan = getattr(self.mav, '_scan', None)
        if scan is None:
            return None
        scan_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        stamps = (pose_stamp, velocity_stamp, scan_stamp)
        if not all(isinstance(stamp, (int, float)) and math.isfinite(stamp)
                   and 0 <= now-stamp <= self.stale_s for stamp in stamps):
            return None
        if not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed < 0:
            return None
        return float(speed), float(velocity_stamp)

    def _hold_position(self):
        self.mav.goto(*self._hold, self.mav.yaw())

    def _brake_position(self):
        # Hold the physical arrival using measured ground displacement.
        # A fixed estimated XY can pull sideways as the GPS estimate wanders.
        now = self._now()
        previous = self._brake_motion_previous
        capture_stamp = self._brake_capture_stamp
        self._brake_capture_fault = ''
        for sample in getattr(self.mav, 'ground_velocity_samples', ()):
            if (not isinstance(sample, (tuple, list)) or len(sample) != 3
                    or not all(isinstance(value, (int, float))
                    and math.isfinite(value) for value in sample)):
                self._brake_capture_fault = 'invalid vector in capture history'
                return False
            stamp, vx, vy = sample
            if stamp > now or (previous is not None and stamp <= previous[0]):
                continue
            if previous is None:
                previous = sample
                capture_stamp = stamp
                continue
            dt = stamp - previous[0]
            # The ROS clock and capture headers have integer nanosecond
            # precision. Subtracting their float seconds can turn an exact
            # 0.30 s interval into 0.30000000000007 and reject valid motion.
            # A gap up to bridge_s is integrated between the FCU velocities
            # either side of it (braking is near-stationary by then); one
            # longer than that leaves the displacement unknown.
            if round(dt*1e9) > round(self.bridge_s*1e9):
                self._brake_capture_fault = f'capture gap {round(dt*1e9)} ns exceeds {round(self.bridge_s*1e9)} ns'
                self._brake_gap_fatal = True
                return False
            self._brake_displacement[0] += .5 * dt * (vx + previous[1])
            self._brake_displacement[1] += .5 * dt * (vy + previous[2])
            previous = sample
            capture_stamp = stamp
        if previous is None or not 0 <= now-previous[0] <= self.stale_s:
            self._brake_capture_fault = 'latest vector capture stale or unavailable'
            return False
        scalar_stamp = self.mav.delivery_inputs.get('velocity_stamp')
        if capture_stamp != scalar_stamp:
            # A capture stamped ahead of this node's clock (MAVROS timesync, or
            # /clock delivery lagging) is in the scalar copy but not yet in the
            # integrated history. That is a transient, not a disagreement.
            ahead = (isinstance(scalar_stamp, (int, float)) and math.isfinite(scalar_stamp)
                     and scalar_stamp > now)
            self._brake_capture_fault = ('newest capture stamped ahead of the clock' if ahead
                                         else 'scalar speed capture differs from vector history')
            return False
        self._brake_motion_previous = previous
        self._brake_capture_stamp = capture_stamp
        x, y, _ = self.mav.pos()
        if not all(math.isfinite(value) for value in self.mav.pos()):
            self._brake_capture_fault = 'nonfinite braking position'
            return False
        dx, dy = self._brake_displacement
        self._brake_hold = (x-dx, y-dy, self._flight_alt)
        self.mav.goto(*self._brake_hold, self._brake_yaw)
        return True

    def _brake_or_fail(self):
        """SUCCESS while braking from consistent captures; RUNNING while a
        transient fault is held out at the last braking anchor (up to
        pause_s); FAILURE for invalid data, an unbridgeable gap, or a fault
        that outlasts the pause. One late or skewed capture used to abort the
        mission on the spot (16 Oct campaign, h030L5_both calm s3)."""
        self._brake_gap_fatal = False
        if self._brake_position():
            self._brake_fault_since = None
            return SUCCESS
        fault = self._brake_capture_fault
        invalid = fault in ('invalid vector in capture history', 'nonfinite braking position')
        now = self._now()
        if not invalid and not self._brake_gap_fatal:
            if self._brake_fault_since is None:
                self._brake_fault_since = now
            if now - self._brake_fault_since <= self.pause_s:
                if self._brake_hold is not None:
                    self.mav.goto(*self._brake_hold, self._brake_yaw)
                else:
                    self._hold_position()
                self.feedback_message = 'holding for consistent braking captures: ' + fault
                return RUNNING
        self._hold_position()
        self._fail('continuous fresh ground velocity required for gate braking: ' + fault)
        return FAILURE

    def _distance_covered_by_motion(self, now):
        """Confirm signed forward travel from fresh FCU ground velocities.

        A drifting GPS position can leave the latched crossing waypoint
        beyond the first obstacle even after the measured distance was flown.
        Integrate only captures since speed admission, retaining the original
        crossing distance, heading and arrival tolerance.
        """
        if (not self._cross_motion_valid or self._cross_epoch is None
                or self._cross_axis is None):
            return False
        for sample in getattr(self.mav, 'ground_velocity_samples', ()):
            if (len(sample) != 3 or not all(isinstance(value, (int, float))
                    and math.isfinite(value) for value in sample)):
                self._cross_motion_valid = False
                return False
            stamp, vx, vy = sample
            if stamp < self._cross_epoch or stamp > now:
                continue
            previous = self._cross_motion_previous
            if previous is not None:
                dt = stamp - previous[0]
                if dt <= 0:
                    continue
                if dt > self.stale_s:
                    self._cross_motion_valid = False
                    return False
                self._cross_displacement[0] += .5 * dt * (vx + previous[1])
                self._cross_displacement[1] += .5 * dt * (vy + previous[2])
            self._cross_motion_previous = sample
        previous = self._cross_motion_previous
        if previous is None or not 0 <= now-previous[0] <= self.stale_s:
            return False
        progress = sum(a*b for a,b in zip(self._cross_displacement,self._cross_axis))
        return (self._flight_distance is not None
                and progress >= self._flight_distance-self.tol)

    def _fail(self, reason):
        self.feedback_message = self.mav.abort_reason = 'GateAdvance: ' + reason
        return FAILURE

    def _speed_ack(self, speed, now):
        if self._lease_future is None:
            try:
                future = self.mav.set_speed(speed)
            except Exception:
                return self._fail('ground-speed command service failed')
            if future is None:
                return self._fail('ground-speed command service unavailable')
            self._lease_future = future
            self._lease_sent = now
        if not self._lease_future.done():
            if now-self._lease_sent > self.service_timeout_s:
                return self._fail('ground-speed command acknowledgement timed out')
            return RUNNING
        try:
            accepted = bool(self._lease_future.result().success)
        except Exception:
            accepted = False
        if not accepted:
            return self._fail('flight controller rejected the ground-speed command')
        self._lease_future = None
        self._lease_sent = None
        return SUCCESS

    def update(self):
        now = self._now()
        if self._hold is None:
            self._hold = self.mav.pos()
        # Refresh the stop anchor from an actual fresh pose even if another
        # observation has failed. A stale scan must not command a return to
        # the original gate approach after the aircraft has crossed it.
        pose_stamp = getattr(self.mav, 'delivery_inputs', {}).get('pose_stamp')
        if isinstance(pose_stamp, (int, float)) and math.isfinite(pose_stamp) and (
                0 <= now-pose_stamp <= self.stale_s):
            position = self.mav.pos()
            if all(math.isfinite(value) for value in position):
                self._hold = position
        if now-self._started > 60.:
            self._hold_position()
            return self._fail('gate crossing or braking did not settle in 60 seconds')
        motion = self._fresh_motion(now)
        if motion is None:
            self._hold_position()
            self._settle_first = self._settle_last = None
            self._settle_count = 0
            if self._capture_pause_started is None:
                self._capture_pause_started = now
            # No crossing or handover uses data beyond stale_s. A missed
            # capture pauses at the last fresh measured position; pause_s
            # without recovery still fails safely.
            if now-self._capture_pause_started > self.pause_s:
                return self._fail('fresh pose, velocity and lidar capture times required')
            self.feedback_message = 'holding for fresh gate capture times'
            return RUNNING
        self._capture_pause_started = None
        speed, velocity_stamp = motion

        if self._phase == 'ADMIT':
            self._hold_position()
            if not isinstance(self._entry_speed, (int, float)) or not (
                    math.isfinite(self._entry_speed) and self._entry_speed > 0):
                return self._fail('previous commanded navigation speed is unavailable')
            status = self._speed_ack(self.gate_speed_mps, now)
            if status != SUCCESS:
                return status
            self._phase = 'CROSS'
            self._cross_epoch = now
            heading = self.mav.yaw()
            self._cross_axis = (math.cos(heading), math.sin(heading))

        if self._phase == 'CROSS':
            status = (SUCCESS if self._distance_covered_by_motion(now)
                      else super().update())
            if status != SUCCESS:
                return status
            # Arrival has already passed the crossing's distance check.
            # Continuing the final target can fight the keep-out guard and
            # prevent settling beside the first slalom obstacle. Brake at
            # the fresh measured horizontal arrival, retaining the altitude
            # measured safe by DuckUnderBoard, then verify actual motion.
            x,y,_ = self.mav.pos()
            self._brake_hold = (x,y,self._flight_alt)
            self._brake_yaw = self.mav.yaw()
            samples = getattr(self.mav, 'ground_velocity_samples', ())
            latest = next((s for s in reversed(samples) if isinstance(s, (tuple, list))
                           and len(s) == 3
                           and all(isinstance(v, (int, float)) and math.isfinite(v)
                                   for v in s)
                           and 0 <= now-s[0] <= self.stale_s), None)
            if latest is None:
                self._hold_position()
                return self._fail('fresh ground velocity required at gate arrival')
            self._brake_motion_previous = (now, latest[1], latest[2])
            self._brake_capture_stamp = latest[0]
            self._brake_displacement = [0., 0.]
            self._phase = 'BRAKE'

        if self._phase == 'BRAKE':
            status = self._brake_or_fail()
            if status is not SUCCESS:
                return status
            if (speed > self.arrival_speed_mps or
                    math.dist(self.mav.pos(),self._brake_hold) > self.tol):
                self._settle_first = self._settle_last = None
                self._settle_count = 0
                self.feedback_message = 'waiting for measured gate braking'
                return RUNNING
            if self._settle_last != velocity_stamp:
                if self._settle_first is None:
                    self._settle_first = velocity_stamp
                self._settle_last = velocity_stamp
                self._settle_count += 1
            if self._settle_count < 3 or velocity_stamp-self._settle_first < self.settle_s:
                self.feedback_message = 'confirming gate braking with distinct FCU samples'
                return RUNNING
            self._hold = self.mav.pos()
            self._phase = 'RESTORE'

        if self._phase == 'RESTORE':
            status = self._brake_or_fail()
            if status is not SUCCESS:
                return status
            status = self._speed_ack(float(self._entry_speed), now)
            if status != SUCCESS:
                return status
            self._phase = 'READY'

        status = self._brake_or_fail()
        if status is not SUCCESS:
            return status
        if (speed > self.arrival_speed_mps or
                math.dist(self.mav.pos(),self._brake_hold) > self.tol):
            self.feedback_message = 'waiting for the restored-speed position hold to settle'
            return RUNNING
        return SUCCESS
