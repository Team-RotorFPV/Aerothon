#!/usr/bin/env python3
"""The corridor navigator must actually HOLD altitude, not just say it does.

WHAT WAS WRONG

    velocity_controller._publish() contained:

        sp.velocity.z = 0.0            # hold altitude

    Zero vertical velocity is a command for zero vertical RATE. It is not a
    held altitude: any thrust bias, downwash or disturbance integrates, and
    nothing corrects it. The comment asserted an outcome the command could
    not produce.

    Worse, `_on_pose` stored only (x, y) and discarded z, so the controller
    did not know its own altitude and COULD NOT have held it.

    Arena regression seed 1002, corridor navigation:

        t32  CORRIDOR_NAV  [9.8 -2.3 2.8]
        t40  ABORT         [9.7 -1.8 0.9]

    Forward progress stopped and the aircraft sank 1.9 m in eight seconds.
    The mission's altitude-band guard caught it and aborted — the guard did
    its job, but it exists to catch what should not happen.

    This is the same defect as the very first live failure recorded in this
    project (CORRIDOR_NAV reported while dragging along the ground at
    z = 0.117 m). The guard was added then; the cause was not.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_altitude_hold.py -v
"""

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))


def make_controller():
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.create_timer'), \
         patch('rclpy.node.Node.declare_parameter'), \
         patch('rclpy.node.Node.get_logger'):
        from avoidance.velocity_controller import VelocityController
        params = {
            'alt_hold_gain': 0.8, 'max_vz': 0.6, 'rate_hz': 20.0,
            'scan_topic': '/scan',
        }
        with patch('rclpy.node.Node.get_parameter',
                   side_effect=lambda n: MagicMock(
                       value=params.get(n, 0.0))):
            c = VelocityController()
    c._g = lambda n: {'alt_hold_gain': 0.8, 'max_vz': 0.6}.get(n, 0.0)
    return c


class AltitudeIsKnownTests(unittest.TestCase):
    """It cannot hold what it does not measure."""

    def setUp(self):
        self.c = make_controller()

    def test_the_pose_handler_keeps_the_ALTITUDE(self):
        m = MagicMock()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 1.0, 2.0, 3.4
        self.c._on_pose(m)
        self.assertAlmostEqual(self.c._alt, 3.4,
                               msg="z is discarded; altitude hold is impossible")

    def test_an_unknown_altitude_produces_NO_correction(self):
        """A confident correction from an unknown altitude is worse than none."""
        self.c._alt = None
        self.c._hold_alt = 3.0
        self.assertEqual(self.c._alt_correction(), 0.0)

    def test_no_target_produces_no_correction(self):
        self.c._alt = 3.0
        self.c._hold_alt = None
        self.assertEqual(self.c._alt_correction(), 0.0)


class CorrectionTests(unittest.TestCase):

    def setUp(self):
        self.c = make_controller()
        self.c._hold_alt = 3.0

    def test_below_target_commands_a_CLIMB(self):
        """The seed 1002 case: 0.9 m when it should be at 3.0 m."""
        self.c._alt = 0.9
        self.assertGreater(self.c._alt_correction(), 0.0)

    def test_above_target_commands_a_DESCENT(self):
        self.c._alt = 4.5
        self.assertLess(self.c._alt_correction(), 0.0)

    def test_on_target_commands_nothing(self):
        self.c._alt = 3.0
        self.assertAlmostEqual(self.c._alt_correction(), 0.0)

    def test_the_correction_is_CAPPED(self):
        """A large error must not command a violent climb."""
        self.c._alt = 0.0
        self.assertLessEqual(self.c._alt_correction(), 0.6 + 1e-9)
        self.c._alt = 40.0
        self.assertGreaterEqual(self.c._alt_correction(), -0.6 - 1e-9)

    def test_the_correction_is_proportional_to_the_error(self):
        self.c._alt = 2.9
        small = self.c._alt_correction()
        self.c._alt = 2.5
        big = self.c._alt_correction()
        self.assertGreater(big, small)

    def test_a_sink_is_ARRESTED_before_the_band_guard_fires(self):
        """Simulate the seed 1002 sink with a constant downward disturbance
        and check the loop actually recovers, rather than that its sign is
        right. `velocity.z = 0.0` cannot pass this."""
        alt, dt, sink = 2.8, 0.05, -0.35        # m/s of unmodelled sink
        for _ in range(400):                     # 20 s
            self.c._alt = alt
            alt += (sink + self.c._alt_correction()) * dt
        self.assertGreater(alt, 3.0 - 1.5,
                           f"settled at {alt:.2f} m, outside the 3.0 +/- 1.5 band")

    def test_zero_vz_would_FAIL_that_same_sink(self):
        """The negative control: the old behaviour, same disturbance."""
        alt, dt, sink = 2.8, 0.05, -0.35
        for _ in range(400):
            alt += (sink + 0.0) * dt
        self.assertLess(alt, 3.0 - 1.5)


class ThePublishedSetpointCarriesItTests(unittest.TestCase):
    """`_alt_correction()` being right is worthless if the setpoint ignores it.

    Found by mutation: reverting `_publish` to `velocity.z = 0.0` while
    leaving `_alt_correction()` intact broke NOTHING in this file, because
    every other test called the method directly. The tests were checking a
    calculation, not the command that reaches the flight controller.
    """

    def setUp(self):
        self.c = make_controller()
        self.sent = []
        self.c.pub_sp = MagicMock()
        self.c.pub_sp.publish = self.sent.append
        self.c.pub_status = MagicMock()
        self.c.pub_detail = MagicMock()
        self.c.state = "CRUISE"
        self.c._left_m = self.c._right_m = 2.0
        self.c.get_clock = MagicMock()      # no real node behind this

    def _publish_once(self):
        self.c._publish(0.5, 0.0, 3.0, 0.0, {})
        return self.sent[-1]

    def test_the_setpoint_z_IS_the_correction(self):
        self.c._hold_alt, self.c._alt = 3.0, 1.5
        sp = self._publish_once()
        self.assertAlmostEqual(sp.velocity.z, self.c._alt_correction())
        self.assertGreater(sp.velocity.z, 0.0,
                           "the setpoint commands no climb while 1.5 m low")

    def test_a_sinking_aircraft_is_commanded_UP_in_the_setpoint(self):
        """Seed 1002's numbers, end to end through the published command."""
        self.c._hold_alt, self.c._alt = 3.0, 0.9
        self.assertGreater(self._publish_once().velocity.z, 0.0)

    def test_on_target_the_setpoint_z_is_zero(self):
        self.c._hold_alt, self.c._alt = 3.0, 3.0
        self.assertAlmostEqual(self._publish_once().velocity.z, 0.0)

    def test_horizontal_commands_are_untouched(self):
        """The altitude loop must not disturb follow-the-gap steering."""
        self.c._hold_alt, self.c._alt = 3.0, 1.0
        sp = self._publish_once()
        self.assertAlmostEqual(sp.velocity.x, 0.5)
        self.assertAlmostEqual(sp.velocity.y, 0.0)


class HandoverTests(unittest.TestCase):

    def setUp(self):
        self.c = make_controller()

    def test_the_mission_can_STATE_the_altitude_to_hold(self):
        """Better than latching whatever the aircraft happened to be at: the
        corridor stage knows it wants the rulebook's 3 m."""
        self.c._on_hold_alt(MagicMock(data=3.0))
        self.assertAlmostEqual(self.c._hold_alt, 3.0)

    def test_a_non_positive_hold_altitude_clears_it(self):
        self.c._hold_alt = 3.0
        self.c._on_hold_alt(MagicMock(data=0.0))
        self.assertIsNone(self.c._hold_alt)

    def test_enabling_latches_the_current_altitude_as_a_default(self):
        self.c._alt = 2.9
        self.c.enabled = False
        self.c._on_enable(MagicMock(data=True))
        self.assertAlmostEqual(self.c._hold_alt, 2.9)

    def test_disabling_clears_the_hold(self):
        """Otherwise a stale target from the outbound corridor would steer the
        next stage."""
        self.c._alt = 3.0
        self.c.enabled = False
        self.c._on_enable(MagicMock(data=True))
        self.c._on_enable(MagicMock(data=False))
        self.assertIsNone(self.c._hold_alt)


if __name__ == "__main__":
    unittest.main(verbosity=2)


class SafetyStopsStillHoldAltitudeTests(unittest.TestCase):
    """Stopping horizontally must not mean giving up vertically.

    The stale-scan and missing-scan paths are asserted to command
    vx = vy = 0 (sim/test_corridor_recovery.py), which is right: a lidar that
    has gone quiet must not be coasted on. Nothing asserted anything about the
    vertical axis, and "the lidar went stale" must not become "the aircraft
    descends into whatever it could no longer see".
    """

    def setUp(self):
        self.c = make_controller()
        self.sent = []
        self.c.pub_sp = MagicMock()
        self.c.pub_sp.publish = self.sent.append
        self.c.pub_status = MagicMock()
        self.c.pub_detail = MagicMock()
        self.c.get_clock = MagicMock()
        self.c.state = "BLOCKED"
        self.c._left_m = self.c._right_m = 2.0
        self.c._hold_alt, self.c._alt = 3.0, 1.6

    def test_a_full_stop_still_commands_the_altitude_correction(self):
        self.c._publish(0.0, 0.0, 0.0, 0.0, {"fault": "scan stale"})
        sp = self.sent[-1]
        self.assertAlmostEqual(sp.velocity.x, 0.0)
        self.assertAlmostEqual(sp.velocity.y, 0.0)
        self.assertGreater(sp.velocity.z, 0.0,
                           "stopped horizontally AND sinking is not a stop")

    def test_at_the_held_altitude_a_stop_is_a_full_stop(self):
        self.c._alt = 3.0
        self.c._publish(0.0, 0.0, 0.0, 0.0, {})
        sp = self.sent[-1]
        for v in (sp.velocity.x, sp.velocity.y, sp.velocity.z):
            self.assertAlmostEqual(v, 0.0)
