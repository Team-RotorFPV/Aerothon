#!/usr/bin/env python3
"""The navigator must be able to TURN to follow a rotated corridor.

WHAT WAS WRONG

    velocity_controller._publish() contained:

        sp.yaw_rate = float(self._g('max_yaw_rate'))

    That publishes a PARAMETER as the command. The parameter was 0.0,
    commented "heading is owned by the mission", so the aircraft could never
    yaw. The navigator measured a gap bearing every tick and discarded it,
    leaving only sideways translation to cross a rotated corridor.

    The shipped arena's corridor is axis-aligned, so entering on the banner
    heading is already correct and nothing shows. Phase 11 rotates it, and
    the outcome tracks the rotation exactly:

        seed 1001   10.4 deg   FAILED
        seed 1002  -11.2 deg   FAILED
        seed 1003    8.9 deg   FAILED
        seed 1004    4.5 deg   completed
        seed 1005    1.5 deg   completed

    Recorded flight, seed 1002: clean cruise at 3.00 m and 0.8 m/s while
    drifting y = -1.8 -> -2.9, then wedged with 0.3 m ahead, backoff ladder
    exhausted, tipped to 47.8 degrees. Commanded velocity at that instant was
    0.00, 0.00, 0.03 — the pitch-over is a COLLISION, not a manoeuvre.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_corridor_yaw_alignment.py -v
"""

import math
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))

PARAMS = {"max_yaw_rate": 0.5, "yaw_align_gain": 1.2,
          "alt_hold_gain": 0.8, "max_vz": 0.6,
          "rate_hz": 20.0, "scan_topic": "/scan"}


def make_controller():
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.create_timer'), \
         patch('rclpy.node.Node.declare_parameter'), \
         patch('rclpy.node.Node.get_logger'), \
         patch('rclpy.node.Node.get_parameter',
               side_effect=lambda n: MagicMock(value=PARAMS.get(n, 0.0))):
        from avoidance.velocity_controller import VelocityController
        c = VelocityController()
    c._g = lambda n: PARAMS.get(n, 0.0)
    return c


class YawFollowsTheGapTests(unittest.TestCase):

    def setUp(self):
        self.c = make_controller()

    def test_a_gap_to_the_LEFT_turns_left(self):
        self.assertGreater(self.c._yaw_rate_for(0.30), 0.0)

    def test_a_gap_to_the_RIGHT_turns_right(self):
        self.assertLess(self.c._yaw_rate_for(-0.30), 0.0)

    def test_a_centred_gap_commands_no_turn(self):
        self.assertAlmostEqual(self.c._yaw_rate_for(0.0), 0.0)

    def test_the_turn_rate_is_CAPPED(self):
        self.assertLessEqual(self.c._yaw_rate_for(3.0), 0.5 + 1e-9)
        self.assertGreaterEqual(self.c._yaw_rate_for(-3.0), -0.5 - 1e-9)

    def test_a_bigger_bearing_turns_faster(self):
        self.assertGreater(self.c._yaw_rate_for(0.20),
                           self.c._yaw_rate_for(0.05))

    def test_the_rate_is_NOT_the_parameter(self):
        """The defect stated directly: publishing the cap as the command means
        every bearing yields the same turn."""
        rates = {self.c._yaw_rate_for(b) for b in (0.0, 0.1, -0.1, 0.25)}
        self.assertGreater(len(rates), 1,
                           "yaw_rate does not vary with the gap bearing — it "
                           "is still being published as a constant")


class ThePublishedSetpointTurnsTests(unittest.TestCase):
    """A correct calculation the setpoint ignores is worth nothing — the
    lesson from the altitude-hold fix, applied up front this time."""

    def setUp(self):
        self.c = make_controller()
        self.sent = []
        self.c.pub_sp = MagicMock()
        self.c.pub_sp.publish = self.sent.append
        self.c.pub_status = MagicMock()
        self.c.pub_detail = MagicMock()
        self.c.get_clock = MagicMock()
        self.c.state = "CRUISE"
        self.c._left_m = self.c._right_m = 2.0
        self.c._hold_alt = self.c._alt = 3.0

    def publish(self, bearing):
        self.c._publish(0.8, 0.0, 3.0, bearing, {})
        return self.sent[-1]

    def test_the_setpoint_yaw_rate_follows_the_bearing(self):
        sp = self.publish(0.30)
        self.assertAlmostEqual(sp.yaw_rate, self.c._yaw_rate_for(0.30))
        self.assertGreater(sp.yaw_rate, 0.0)

    def test_opposite_bearings_give_opposite_setpoint_turns(self):
        left = self.publish(0.30).yaw_rate
        right = self.publish(-0.30).yaw_rate
        self.assertGreater(left, 0.0)
        self.assertLess(right, 0.0)

    def test_a_rotated_corridor_is_turned_INTO_not_slid_across(self):
        """Seed 1002's corridor is rotated -11.2 degrees. Entering it on the
        banner heading puts the gap off-centre; the aircraft must turn to
        face it rather than translate the whole way."""
        bearing = math.radians(-11.2)
        sp = self.publish(bearing)
        self.assertLess(sp.yaw_rate, 0.0,
                        "no turn commanded for an 11 degree corridor offset")
        self.assertAlmostEqual(sp.velocity.x, 0.8)

    def test_forward_and_lateral_commands_are_untouched(self):
        sp = self.publish(0.25)
        self.assertAlmostEqual(sp.velocity.x, 0.8)
        self.assertAlmostEqual(sp.velocity.y, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
