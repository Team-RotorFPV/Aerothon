#!/usr/bin/env python3
"""Phase 2 — automated assertions on ENU/NED and body-frame signs.

CURRENT_PROGRESS_HANDOFF.md repair step 2: "Verify ENU/NED and body-frame signs
with automated assertions." This is that.

The defect these tests exist to prevent (found live, VERIFICATION.md 2.3):
`velocity_controller` documented and implemented MAVLink's FRD convention
("y right") for /mavros/setpoint_raw/local. But MAVROS takes body-frame
setpoints in ROS FLU and runs transform_frame_baselink_aircraft itself, so
positive y on that topic means LEFT. Every lateral correction was applied to
the wrong side and the aircraft steered into the wall it was avoiding.

A sign error is invisible in a symmetric corridor and catastrophic in an
asymmetric one, so it must be asserted, not eyeballed.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_frame_conventions.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import PositionTarget

from avoidance.velocity_controller import (
    VelocityController,
    VEL_YAWRATE_MASK,
    FRAME_BODY_OFFSET_NED,
)


def make_scan(left_m, right_m, front_m, n=360, rmax=12.0):
    """A CORRIDOR, not three isolated sectors.

    Rays are traced against two parallel walls (left at `left_m`, right at
    `right_m`) and a frontal obstacle at `front_m`, so the range varies
    smoothly with bearing the way a real scan does. The previous fixture put
    max range everywhere except three narrow sectors, which cannot exercise a
    gap-following controller: the "widest gap" was always the empty rest of the
    arc.

    ROS LaserScan is counter-clockwise with 0 straight ahead, so +pi/2 is the
    vehicle's LEFT and -pi/2 its RIGHT.
    """
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.05
    scan.range_max = rmax
    ranges = []
    for i in range(n):
        ang = scan.angle_min + i * scan.angle_increment
        cands = [rmax]
        s_, c_ = math.sin(ang), math.cos(ang)
        if s_ > 1e-6:                      # ray points left
            cands.append(left_m / s_)
        if s_ < -1e-6:                     # ray points right
            cands.append(right_m / -s_)
        if c_ > 1e-6:                      # ray points forward
            cands.append(front_m / c_)
        ranges.append(max(0.05, min(rmax, min(cands))))
    scan.ranges = ranges
    return scan


class FrameConventionTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = VelocityController()
        self.sent = []
        self.node.pub_sp.publish = self.sent.append
        self.node.enabled = True

    def tearDown(self):
        self.node.destroy_node()

    def tick_with(self, scan):
        # Go through the real callback so the scan timestamp is set; assigning
        # .scan directly leaves the stale-scan failsafe correctly firing and
        # every command zeroed.
        self.node._on_scan(scan)
        self.node._tick()
        self.assertTrue(self.sent, "controller published nothing")
        return self.sent[-1]

    # ---- the sign that caused the crash ---- #

    def test_room_on_the_left_commands_positive_y(self):
        """Roomier left => move LEFT => positive vy on this topic."""
        sp = self.tick_with(make_scan(left_m=3.0, right_m=0.5, front_m=10.0))
        self.assertGreater(
            sp.velocity.y, 0.0,
            "left is roomier, so the drone must move left (+y in MAVROS FLU); "
            "a negative value steers it into the near right wall")

    def test_room_on_the_right_commands_negative_y(self):
        sp = self.tick_with(make_scan(left_m=0.5, right_m=3.0, front_m=10.0))
        self.assertLess(
            sp.velocity.y, 0.0,
            "right is roomier, so the drone must move right (-y in MAVROS FLU)")

    def test_centred_corridor_commands_negligible_lateral(self):
        """Symmetric corridor => essentially straight ahead.

        Not exactly zero: the gap midpoint lands on a scan sample, so a 1 deg
        angular resolution leaves up to half a sample of bias. What matters is
        that it is negligible compared with the clamp, not that it is 0.000.
        """
        sp = self.tick_with(make_scan(left_m=1.75, right_m=1.75, front_m=10.0))
        limit = self.node.get_parameter('max_lateral').value
        self.assertLess(abs(sp.velocity.y), 0.1 * limit,
                        f"symmetric corridor should not steer: {sp.velocity.y}")

    def test_lateral_is_clamped_both_ways(self):
        limit = self.node.get_parameter('max_lateral').value
        sp = self.tick_with(make_scan(left_m=12.0, right_m=0.2, front_m=10.0))
        self.assertLessEqual(sp.velocity.y, limit + 1e-6)
        sp = self.tick_with(make_scan(left_m=0.2, right_m=12.0, front_m=10.0))
        self.assertGreaterEqual(sp.velocity.y, -limit - 1e-6)

    # ---- forward / vertical ---- #

    def test_clear_front_commands_full_cruise_forward(self):
        """Essentially cruise, not exactly: the controller travels ALONG the
        chosen gap, so vx = speed * cos(bearing) and a gap half a degree off
        centre costs 0.004%. Asserting exact equality would be asserting that
        the aircraft never steers."""
        sp = self.tick_with(make_scan(left_m=1.75, right_m=1.75, front_m=12.0))
        self.assertGreater(sp.velocity.x, 0.0, "+x must be FORWARD")
        self.assertAlmostEqual(sp.velocity.x / self.node.cruise, 1.0, places=3)

    def test_obstacle_inside_stop_distance_halts(self):
        stop = self.node.get_parameter('stop_dist').value
        sp = self.tick_with(make_scan(1.75, 1.75, front_m=stop * 0.5))
        self.assertAlmostEqual(sp.velocity.x, 0.0, places=6)

    def test_braking_ramp_is_monotonic(self):
        stop = self.node.get_parameter('stop_dist').value
        brake = self.node.get_parameter('brake_dist').value
        speeds = []
        for d in [stop + (brake - stop) * f for f in (0.1, 0.4, 0.7, 1.0)]:
            speeds.append(self.tick_with(make_scan(1.75, 1.75, d)).velocity.x)
        for a, b in zip(speeds, speeds[1:]):
            self.assertLessEqual(a, b + 1e-9,
                                 f"speed must not fall as clearance grows: {speeds}")

    def test_no_vertical_command_without_a_target_altitude(self):
        """This used to read `test_altitude_is_held` and assert velocity.z == 0.

        That is what the controller DID, and it is not what the name claimed:
        zero vertical velocity is zero vertical RATE, and the aircraft sank
        1.9 m in eight seconds in the corridor while this test was green. The
        test asserted the defect, exactly as the search planner's
        `test_never_sweeps_below_decode_altitude` did.

        What is genuinely true here is narrower: with no altitude to hold,
        there must be no vertical command. A confident correction toward an
        unknown target is worse than none. The hold itself is proven in
        sim/test_altitude_hold.py, against a simulated sink.
        """
        self.node._hold_alt = None
        sp = self.tick_with(make_scan(1.75, 1.75, 10.0))
        self.assertAlmostEqual(sp.velocity.z, 0.0, places=6)

    def test_a_target_altitude_DOES_produce_a_vertical_command(self):
        """The half the old test could not have caught."""
        self.node._hold_alt = 3.0
        self.node._alt = 1.5
        sp = self.tick_with(make_scan(1.75, 1.75, 10.0))
        self.assertGreater(sp.velocity.z, 0.0,
                           "1.5 m below the corridor altitude and the setpoint "
                           "commands no climb")

    # ---- message shape ---- #

    def test_frame_and_mask_are_body_velocity(self):
        sp = self.tick_with(make_scan(1.75, 1.75, 10.0))
        self.assertEqual(sp.coordinate_frame, FRAME_BODY_OFFSET_NED)
        self.assertEqual(sp.type_mask, VEL_YAWRATE_MASK)

    def test_mask_ignores_position_and_uses_velocity(self):
        """Guard the literal bits: a wrong mask silently changes meaning."""
        IGN_PX, IGN_PY, IGN_PZ = 1, 2, 4
        IGN_VX, IGN_VY, IGN_VZ = 8, 16, 32
        IGN_YAW, IGN_YAWRATE = 1024, 2048
        m = VEL_YAWRATE_MASK
        for bit, name in ((IGN_PX, "px"), (IGN_PY, "py"), (IGN_PZ, "pz")):
            self.assertTrue(m & bit, f"position {name} must be ignored")
        for bit, name in ((IGN_VX, "vx"), (IGN_VY, "vy"), (IGN_VZ, "vz")):
            self.assertFalse(m & bit, f"velocity {name} must be USED")
        self.assertTrue(m & IGN_YAW, "yaw must be ignored")
        self.assertFalse(m & IGN_YAWRATE, "yaw_rate must be USED")

    def test_disabled_controller_publishes_nothing(self):
        self.node.enabled = False
        self.node._on_scan(make_scan(1.75, 1.75, 10.0))
        self.node._tick()
        self.assertEqual(self.sent, [],
                         "controller must stay silent until the mission enables it")


if __name__ == "__main__":
    unittest.main(verbosity=2)
