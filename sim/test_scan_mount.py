#!/usr/bin/env python3
"""The LD06 is wherever its bracket put it; the stack reads beams off the nose.

scripts/check_sensors.py measures the mount from two boxes (ahead, to port)
in the raw scan, and mission_bringup's scan_mount rewrites the scan's angles
with the result. These pin the round trip: for any mount yaw, either scan
direction, what was measured is what comes out on the nose and to port.

    python3 -m pytest sim/test_scan_mount.py -v
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_mission", "mission_bringup"))

from sensor_msgs.msg import LaserScan                       # noqa: E402

from check_sensors import nearest_bearing, solve_mount      # noqa: E402
from mission_bringup.scan_mount import device_frame, mount  # noqa: E402

N = 450


def sllidar_scan(device_cw_deg, n=720):
    """sllidar_ros2 publish_scan (angle_compensate, inverted false) of one box
    1 m away, `device_cw_deg` clockwise from the C1's marked 0 deg. The driver
    writes device beam i to slot n-1-i over angle_min = pi - 2pi = -pi."""
    s = LaserScan()
    s.range_min, s.range_max = 0.05, 12.0
    s.angle_min, s.angle_max = math.pi - 2 * math.pi, math.pi
    s.angle_increment = (s.angle_max - s.angle_min) / (n - 1)
    s.ranges = [float("inf")] * n
    i = round(device_cw_deg % 360.0 * n / 360.0) % n
    s.ranges[n - 1 - i] = 1.0
    return s


def raw_scan(body_bearing_deg, yaw_deg, mirrored):
    """The driver's scan of one box 1 m away at `body_bearing_deg`."""
    s = LaserScan()
    s.range_min, s.range_max = 0.02, 12.0
    s.angle_increment = (-1 if mirrored else 1) * 2 * math.pi / N
    s.angle_min = 0.0
    s.angle_max = s.angle_min + (N - 1) * s.angle_increment
    raw = (yaw_deg - body_bearing_deg) if mirrored else (body_bearing_deg - yaw_deg)
    s.ranges = [float("inf")] * N
    i = round(math.radians(raw) / s.angle_increment) % N
    s.ranges[i] = 1.0
    return s


class ScanMountTests(unittest.TestCase):

    def test_what_is_measured_comes_out_on_the_nose_and_to_port(self):
        for yaw in (0.0, 90.0, -135.0, 180.0, 37.0):
            for mirrored in (False, True):
                with self.subTest(yaw=yaw, mirrored=mirrored):
                    front = nearest_bearing(raw_scan(0.0, yaw, mirrored))[0]
                    port = nearest_bearing(raw_scan(90.0, yaw, mirrored))[0]
                    got_yaw, got_mirrored, err = solve_mount(front, port)
                    self.assertEqual(got_mirrored, mirrored)
                    self.assertLess(err, 1.0)
                    for body in (0.0, 90.0, -60.0):
                        s = mount(raw_scan(body, yaw, mirrored),
                                  math.radians(got_yaw), got_mirrored)
                        self.assertAlmostEqual(nearest_bearing(s)[0], body, delta=1.0)

    def test_c1_front_is_bearing_zero_before_any_calibration(self):
        """18 Oct: an object at the C1's nose drew behind the GCS arrow.
        sllidar_ros2 puts the C1's 0 deg at pi; the driver correction undoes
        that, so yaw 0 means the lidar's marked front faces the airframe nose."""
        for device_cw, body_ccw in ((0.0, 0.0), (90.0, -90.0), (180.0, 180.0),
                                    (270.0, 90.0), (30.0, -30.0)):
            with self.subTest(device_cw=device_cw):
                raw = device_frame(sllidar_scan(device_cw), math.pi)
                got = nearest_bearing(raw)[0]
                self.assertAlmostEqual((got - body_ccw + 180) % 360 - 180, 0.0, delta=1.0)
                body = mount(raw, 0.0, False)
                got = nearest_bearing(body)[0]
                self.assertAlmostEqual((got - body_ccw + 180) % 360 - 180, 0.0, delta=1.0)

    def test_uncorrected_c1_scan_is_reversed(self):
        """The fault itself: without the correction the nose reads 180 deg."""
        got = nearest_bearing(sllidar_scan(0.0))[0]
        self.assertAlmostEqual(abs((got + 180) % 360 - 180), 180.0, delta=1.0)


if __name__ == "__main__":
    unittest.main()
