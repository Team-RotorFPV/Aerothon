#!/usr/bin/env python3
"""Winch controller: real subscriber, real feedback, gated release.

Before this existed, /winch/cmd had two publishers and zero subscribers, and
WinchDrop "delivered" after a fixed 20-tick timer whether or not anything had
moved. Payload delivery is what Mission 2 scores.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_winch.py -v
"""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_payload", "winch_ctrl"))

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String

from winch_ctrl.winch_node import WinchNode


def pose(z):
    m = PoseStamped()
    m.pose.position.z = float(z)
    return m


def vel(vx, vy=0.0):
    m = TwistStamped()
    m.twist.linear.x = float(vx)
    m.twist.linear.y = float(vy)
    return m


class WinchTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = WinchNode()
        self.status = []
        self.node.pub_status.publish = self.status.append

    def tearDown(self):
        self.node.destroy_node()

    def cmd(self, text):
        self.node._on_cmd(String(data=text))

    def last(self):
        self.node.publish_status()
        return json.loads(self.status[-1].data)

    def advance(self, seconds, step=0.1):
        n = max(1, int(seconds / step))
        for _ in range(n):
            self.node.integrate(step)

    def stabilise(self, alt=5.0, n=10):
        self.node._on_pose(pose(alt))
        for _ in range(n):
            self.node._on_vel(vel(0.05))

    def lower_to_ground(self, alt=5.0):
        self.stabilise(alt)
        self.cmd("lower")
        for _ in range(400):
            self.node.integrate(0.1)
            if self.node.at_ground() or self.node.at_limit():
                break

    # ---- the interface exists at all ---- #
    def test_publishes_status(self):
        st = self.last()
        self.assertIn("state", st)
        self.assertIn("payout_m", st)
        self.assertEqual(st["state"], "IDLE")

    def test_unknown_command_is_ignored(self):
        self.cmd("banana")
        self.assertEqual(self.node.state, "IDLE")

    # ---- lowering ---- #
    def test_lower_pays_out_line(self):
        self.stabilise(5.0)
        self.cmd("lower")
        before = self.node.payout
        self.advance(2.0)
        self.assertGreater(self.node.payout, before)

    def test_lowering_reaches_ground_state(self):
        self.lower_to_ground(4.0)
        self.assertIn(self.node.state, ("AT_GROUND",))
        self.assertTrue(self.node.at_ground() or self.node.at_limit())

    def test_payout_never_exceeds_spool(self):
        self.stabilise(50.0)          # unreachable ground
        self.cmd("lower")
        self.advance(120.0)
        self.assertLessEqual(self.node.payout,
                             self.node.get_parameter("max_payout_m").value + 1e-6)

    # ---- the interlocks: the point of the node ---- #
    def test_release_refused_before_payload_is_down(self):
        self.stabilise(5.0)
        self.cmd("release")
        self.assertFalse(self.node.released,
                         "released with the payload still hanging")
        self.assertIn("not down", self.node.fault)

    def test_release_refused_while_drifting(self):
        self.lower_to_ground(4.0)
        for _ in range(10):
            self.node._on_vel(vel(3.0))      # moving fast
        ok, blockers = self.node.release_ok()
        self.assertFalse(ok)
        self.assertTrue(any("hover" in b for b in blockers), blockers)

    def test_release_refused_when_too_high(self):
        self.stabilise(50.0)
        self.cmd("lower")
        self.advance(120.0)
        ok, blockers = self.node.release_ok()
        self.assertFalse(ok)
        self.assertTrue(any("altitude" in b for b in blockers), blockers)

    def test_release_succeeds_when_every_condition_holds(self):
        self.lower_to_ground(4.0)
        self.stabilise(4.0)
        ok, blockers = self.node.release_ok()
        self.assertTrue(ok, f"unexpected blockers: {blockers}")
        self.cmd("release")
        self.assertTrue(self.node.released)
        self.assertEqual(self.node.state, "RELEASED")

    def test_cannot_lower_after_release(self):
        self.lower_to_ground(4.0)
        self.stabilise(4.0)
        self.cmd("release")
        self.cmd("lower")
        self.assertIn("already released", self.node.fault)

    def test_delivered_payload_reports_delivery_not_a_fault(self):
        """A successful delivery must not read as a malfunction.

        The first live run left the GCS showing
        `fault: release refused: already released` for the entire return leg,
        after a delivery that had actually succeeded.
        """
        self.lower_to_ground(4.0)
        self.stabilise(4.0)
        self.cmd("release")
        self.assertTrue(self.node.released)
        ok, blockers = self.node.release_ok()
        self.assertFalse(ok)
        self.assertEqual(blockers, ["payload already delivered"])
        self.assertNotIn("fault", " ".join(blockers))

    def test_second_release_is_ignored_without_raising_a_fault(self):
        """Re-commanding release on a delivered payload is redundant, not broken.

        Raising a FAULT here left the GCS reporting an error for the entire
        return leg of a mission whose delivery had succeeded.
        """
        self.lower_to_ground(4.0)
        self.stabilise(4.0)
        self.cmd("release")
        self.assertTrue(self.node.released)
        self.node.fault = ""
        self.cmd("release")
        self.assertEqual(self.node.fault, "",
                         "a redundant release must not raise a fault")
        self.assertEqual(self.node.state, "RELEASED")
        self.assertTrue(self.node.released)

    def test_transient_refusal_clears_once_conditions_are_met(self):
        """A refusal is a condition, not a latched failure."""
        self.stabilise(4.0)
        self.cmd("release")                      # too early: payload not down
        self.assertEqual(self.node.state, "FAULT")
        self.cmd("lower")
        for _ in range(400):
            self.node.integrate(0.1)
            if self.node.at_ground():
                break
        self.stabilise(4.0)
        self.node.integrate(0.1)
        self.assertEqual(self.node.fault, "",
                         "stale refusal still reported after it was resolved")

    def test_blockers_are_reported_not_just_the_first(self):
        """The operator should see everything wrong at once."""
        self.node._on_pose(pose(40.0))
        for _ in range(10):
            self.node._on_vel(vel(5.0))
        ok, blockers = self.node.release_ok()
        self.assertFalse(ok)
        self.assertGreaterEqual(len(blockers), 2, blockers)

    # ---- stow ---- #
    def test_stow_retracts_and_finishes(self):
        self.lower_to_ground(4.0)
        self.cmd("stow")
        for _ in range(1000):
            self.node.integrate(0.1)
            if self.node.payout <= 0.0:
                break
        self.assertEqual(self.node.payout, 0.0)
        self.assertIn(self.node.state, ("IDLE", "RELEASED"))

    def test_status_reports_release_ok_and_blockers(self):
        st = self.last()
        self.assertIn("release_ok", st)
        self.assertIn("blockers", st)
        self.assertFalse(st["release_ok"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
