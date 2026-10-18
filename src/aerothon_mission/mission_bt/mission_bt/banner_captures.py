"""Associate distinct banner captures with recorded onboard poses.

No simulator state is accepted. Missing, late, unpaired or replayed captures
cannot supply an identified banner. Pose interpolation never extrapolates.
"""
import bisect
import math
from collections import deque


class BannerCaptures:
    def __init__(self, max_age_s=0.7, max_pose_gap_s=0.3, history_s=3.0):
        self.max_age_s = max_age_s
        self.max_pose_gap_s = max_pose_gap_s
        self.history_s = history_s
        self.poses = deque()
        self.last_capture = -math.inf
        self.observation = None
        self.observations = deque(maxlen=64)
        self.sequence = 0
        self.reason = 'no timestamped banner capture'

    def pose(self, stamp, xyz, yaw):
        values = (float(stamp), *map(float, xyz), float(yaw))
        if not all(map(math.isfinite, values)) or values[0] <= 0:
            return False
        if self.poses and values[0] <= self.poses[-1][0]:
            return False
        self.poses.append(values)
        while self.poses and self.poses[0][0] < stamp-self.history_s:
            self.poses.popleft()
        return True

    def at(self, stamp):
        if not self.poses:
            return None
        records = list(self.poses)
        index = bisect.bisect_left([row[0] for row in records], stamp)
        if index < len(records) and abs(records[index][0]-stamp) < 1e-8:
            row = records[index]
            return dict(stamp=stamp, xyz=list(row[1:4]), yaw=row[4])
        if not 0 < index < len(records):
            return None
        left, right = records[index-1:index+1]
        gap = right[0]-left[0]
        if gap > self.max_pose_gap_s:
            return None
        fraction = (stamp-left[0])/gap
        delta = math.atan2(math.sin(right[4]-left[4]), math.cos(right[4]-left[4]))
        return dict(stamp=stamp,
                    xyz=[a+fraction*(b-a) for a,b in zip(left[1:4],right[1:4])],
                    yaw=math.atan2(math.sin(left[4]+fraction*delta),
                                   math.cos(left[4]+fraction*delta)))

    def receive(self, detail, now):
        try:
            stamp = float(detail['capture_stamp'])
            processed = float(detail['processed_stamp'])
            bearing = float(detail.get('bearing', 0.0))
            identified = detail.get('identified') is True
        except (KeyError, ValueError, TypeError):
            self.observation = None
            self.reason = 'invalid banner capture metadata'
            return False
        if not all(map(math.isfinite, (stamp, processed, bearing, now))) or not (
                stamp > 0 and stamp <= processed <= now+1e-6 and
                0 <= now-stamp <= self.max_age_s):
            self.observation = None
            self.reason = 'stale or invalid banner capture time'
            return False
        if stamp <= self.last_capture:
            self.reason = 'duplicate or out-of-order banner capture'
            return False
        self.last_capture = stamp
        self.sequence += 1
        self.observation = dict(detail, capture_stamp=stamp, bearing=bearing,
                                identified=identified)
        self.observations.append(dict(self.observation, sequence=self.sequence))
        self.reason = 'awaiting capture-time onboard pose'
        return True

    def current(self, now, not_before=None):
        observation = self.observation
        if observation is None or not 0 <= now-observation['capture_stamp'] <= self.max_age_s:
            return None
        stamp = observation['capture_stamp']
        if not_before is not None and stamp < not_before:
            return None
        pose = self.at(stamp)
        if pose is None:
            return None
        return dict(observation, pose=pose, sequence=self.sequence)

    def recent(self, now, not_before=None):
        if self.current(now, not_before) is None:
            return []
        result = []
        for observation in list(self.observations):
            stamp = observation['capture_stamp']
            if not 0 <= now-stamp <= self.max_age_s:
                continue
            if not_before is not None and stamp < not_before:
                continue
            pose = self.at(stamp)
            if pose is not None:
                result.append(dict(observation, pose=pose))
        return result

    def bearing_now(self, now, current_yaw, hfov_rad):
        return self.bearing_at(self.current(now), current_yaw, hfov_rad)

    @staticmethod
    def bearing_at(observation, current_yaw, hfov_rad):
        if observation is None or not observation['identified']:
            return None
        # Image right is negative body yaw. Preserve the captured world ray
        # while the aircraft rotates during detector processing.
        image_angle = -math.atan(observation['bearing']*math.tan(hfov_rad/2))
        world_ray = observation['pose']['yaw']+image_angle
        current_angle = math.atan2(math.sin(world_ray-current_yaw),
                                   math.cos(world_ray-current_yaw))
        if abs(current_angle) > hfov_rad/2:
            return None
        return -math.tan(current_angle)/math.tan(hfov_rad/2)
