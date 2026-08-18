#!/usr/bin/env python3
"""Phase 3 — the QR loop closes: offset is used, plausibility is checked.

Three things this guards:

  1. The image offset is published for ANY visible marker, not only a matched
     target. Before Phase 3 it was populated only on a match, so during the
     start scan — when the target is by definition unknown — the mission could
     not tell the marker was off to one side.

  2. Centring moves the aircraft the RIGHT WAY. Phase 2 put the drone into a
     wall through an inverted body-frame sign; the same class of error here
     would walk it away from the marker. Asserted, not eyeballed.

  3. A decode whose apparent size is impossible at the current altitude is
     rejected rather than used to set the delivery target.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_qr_closed_loop.py -v
"""

import math
import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "src", "aerothon_mission", "mission_bt"))
sys.path.insert(0, os.path.join(_ROOT, "src", "aerothon_perception", "perception_qr"))

import numpy as np
import py_trees
import rclpy
from geometry_msgs.msg import Vector3

from mission_bt.mission_tree import CenterOnQR, FindStartQR, ScanStartQR
from perception_qr.qr_node import QrNode


class FakeMav:
    def __init__(self):
        self.qr_offset = Vector3(x=0.0, y=0.0, z=0.0)
        self.qr_decoded = ""
        self.qr_streak = 0
        self.target_override = ""
        self.target_set = None
        self.abort_reason = ""
        self._pos = (10.0, 5.0, 5.0)
        self._yaw = 0.0
        self.goto_calls = []

    def pos(self):
        return self._pos

    def alt(self):
        return self._pos[2]

    def yaw(self):
        return self._yaw

    def goto(self, x, y, z, yaw=0.0):
        self.goto_calls.append((x, y, z, yaw))

    def set_target(self, s):
        self.target_set = s

    def qr_confident(self, frames):
        return bool(self.qr_decoded) and self.qr_streak >= frames

    def qr_visible(self):
        return self.qr_offset.z > 0.0

    def qr_centred(self, tol):
        return (self.qr_visible()
                and abs(self.qr_offset.x) <= tol
                and abs(self.qr_offset.y) <= tol)


def offset(x, y, z=0.5):
    return Vector3(x=float(x), y=float(y), z=float(z))


# --------------------------------------------------------------------------- #
class CenteringTests(unittest.TestCase):

    def setUp(self):
        self.mav = FakeMav()
        self.leaf = CenterOnQR("Center", self.mav, tol=0.1, gain=1.0,
                               max_step=2.0, timeout_ticks=50)

    def last_goto(self):
        self.assertTrue(self.mav.goto_calls, "no motion commanded")
        return self.mav.goto_calls[-1]

    def test_marker_below_frame_centre_moves_aircraft_BACK(self):
        """Image +y is DOWN. A marker low in frame is BEHIND the aircraft."""
        self.mav.qr_offset = offset(0.0, 0.6)
        self.leaf.tick_once()
        x, y, z, _ = self.last_goto()
        self.assertLess(x, self.mav.pos()[0],
                        "marker below centre must move the aircraft backwards")

    def test_marker_above_frame_centre_moves_aircraft_FORWARD(self):
        self.mav.qr_offset = offset(0.0, -0.6)
        self.leaf.tick_once()
        x, _, _, _ = self.last_goto()
        self.assertGreater(x, self.mav.pos()[0])

    def test_marker_right_of_centre_moves_aircraft_RIGHT(self):
        """At yaw 0 (facing +x in ENU), the aircraft's right is -y."""
        self.mav.qr_offset = offset(0.6, 0.0)
        self.leaf.tick_once()
        _, y, _, _ = self.last_goto()
        self.assertLess(y, self.mav.pos()[1],
                        "marker right of centre must move the aircraft right (-y at yaw 0)")

    def test_marker_left_of_centre_moves_aircraft_LEFT(self):
        self.mav.qr_offset = offset(-0.6, 0.0)
        self.leaf.tick_once()
        _, y, _, _ = self.last_goto()
        self.assertGreater(y, self.mav.pos()[1])

    def test_correction_respects_yaw(self):
        """Rotated 90 deg, 'forward' is +y in the local frame, not +x."""
        self.mav._yaw = math.pi / 2
        self.mav.qr_offset = offset(0.0, -0.6)      # marker ahead
        self.leaf.tick_once()
        x, y, _, _ = self.last_goto()
        self.assertAlmostEqual(x, self.mav.pos()[0], places=3)
        self.assertGreater(y, self.mav.pos()[1])

    def test_step_is_clamped(self):
        self.leaf.max_step = 0.5
        self.mav.qr_offset = offset(1.0, 1.0)
        self.leaf.tick_once()
        x, y, _, _ = self.last_goto()
        self.assertLessEqual(abs(x - self.mav.pos()[0]), 0.75)
        self.assertLessEqual(abs(y - self.mav.pos()[1]), 0.75)

    def test_centred_marker_succeeds(self):
        self.mav.qr_offset = offset(0.02, -0.03)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_no_marker_eventually_FAILS(self):
        self.mav.qr_offset = offset(0.0, 0.0, 0.0)
        for _ in range(60):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("no marker", self.mav.abort_reason)

    def test_uncentrable_marker_FAILS_rather_than_hunting_forever(self):
        self.mav.qr_offset = offset(0.9, 0.9)       # never improves
        for _ in range(60):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("failed to centre", self.mav.abort_reason)


# --------------------------------------------------------------------------- #
class FindStartQRTests(unittest.TestCase):

    def setUp(self):
        self.mav = FakeMav()
        self.leaf = FindStartQR(self.mav, start_alt=5.0, floor_alt=2.0,
                                step=1.0, dwell_ticks=5)

    def test_visible_marker_succeeds_immediately(self):
        self.mav.qr_offset = offset(0.3, 0.2)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_descends_when_nothing_visible(self):
        self.mav.qr_offset = offset(0, 0, 0.0)
        for _ in range(7):
            self.leaf.tick_once()
        alts = [c[2] for c in self.mav.goto_calls]
        self.assertLess(min(alts), 5.0, f"never descended: {alts}")

    def test_ladder_bottoms_out_and_FAILS(self):
        self.mav.qr_offset = offset(0, 0, 0.0)
        for _ in range(200):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("no start marker", self.mav.abort_reason)

    def test_never_descends_below_the_floor(self):
        self.mav.qr_offset = offset(0, 0, 0.0)
        for _ in range(200):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        alts = [c[2] for c in self.mav.goto_calls]
        self.assertGreaterEqual(min(alts), 2.0, f"descended past the floor: {alts}")


# --------------------------------------------------------------------------- #
class ScanHoldsWhereCentringLeftItTests(unittest.TestCase):

    def test_scan_holds_current_position_not_a_hardcoded_pose(self):
        mav = FakeMav()
        mav._pos = (1.0, 0.0, 5.0)            # where centring put us
        leaf = ScanStartQR(mav, pose=None, timeout_ticks=50, confirm_frames=2)
        leaf.tick_once()
        x, y, z, _ = mav.goto_calls[-1]
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertNotEqual((x, y), (0.0, 0.0),
                            "scan must not fly back to a hardcoded (0,0)")


# --------------------------------------------------------------------------- #
class RealCommanderOffsetSemanticsTests(unittest.TestCase):
    """Against the REAL Mav, not a fake.

    The behaviour tests above use a FakeMav, so they cannot catch a regression
    in the commander's own reading of the offset — a mutation that restored the
    pre-Phase-3 "matched targets only" rule passed all of them. This closes
    that hole: qr_node encodes z = 0.5 for "some marker" and 1.0 for "the
    target", and the mission must act on both.
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        from rclpy.node import Node
        from mission_bt.mav_commander import Mav
        self.node = Node("qr_offset_semantics_test")
        self.mav = Mav(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_unmatched_marker_still_counts_as_visible(self):
        """z=0.5 means 'a marker, not the target' — the start-scan case."""
        self.mav._on_off(offset(0.3, 0.1, 0.5))
        self.assertTrue(self.mav.qr_visible(),
                        "an unmatched marker must still be visible to the "
                        "mission; during the start scan no target is known yet")

    def test_matched_marker_is_visible(self):
        self.mav._on_off(offset(0.0, 0.0, 1.0))
        self.assertTrue(self.mav.qr_visible())

    def test_nothing_visible_when_z_is_zero(self):
        self.mav._on_off(offset(0.0, 0.0, 0.0))
        self.assertFalse(self.mav.qr_visible())

    def test_centred_requires_visibility(self):
        self.mav._on_off(offset(0.0, 0.0, 0.0))
        self.assertFalse(self.mav.qr_centred(0.1),
                         "a zeroed offset must not read as perfectly centred")

    def test_centred_within_tolerance(self):
        self.mav._on_off(offset(0.05, -0.05, 0.5))
        self.assertTrue(self.mav.qr_centred(0.1))
        self.assertFalse(self.mav.qr_centred(0.01))


class PlausibilityTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = QrNode()
        self.node._fx = 554.3

    def tearDown(self):
        self.node.destroy_node()

    def test_gate_abstains_without_altitude(self):
        self.node._alt = None
        ok, why = self.node.plausible(100.0)
        self.assertTrue(ok, "gate must abstain, not reject, when altitude is unknown")
        self.assertIn("abstains", why)

    def test_plausible_marker_accepted(self):
        self.node._alt = 5.0
        # 0.5 m marker at 5 m -> 554.3 * 0.5 / 5 = 55 px
        ok, _ = self.node.plausible(55.0)
        self.assertTrue(ok)

    def test_absurdly_large_detection_rejected(self):
        """A QR filling the frame at 20 m is not a competition marker."""
        self.node._alt = 20.0
        ok, why = self.node.plausible(600.0)
        self.assertFalse(ok)
        self.assertIn("too large", why)

    def test_absurdly_small_detection_rejected(self):
        self.node._alt = 3.0
        ok, why = self.node.plausible(2.0)
        self.assertFalse(ok)
        self.assertIn("too small", why)

    def test_expected_range_scales_inversely_with_altitude(self):
        self.node._alt = 5.0
        lo5, hi5 = self.node.expected_px_range()
        self.node._alt = 10.0
        lo10, hi10 = self.node.expected_px_range()
        self.assertAlmostEqual(lo5 / lo10, 2.0, places=3)
        self.assertAlmostEqual(hi5 / hi10, 2.0, places=3)

    def test_camera_info_with_numpy_intrinsics_does_not_crash(self):
        """CameraInfo.k is a numpy array.

        Writing `if m.k and m.k[0] > 0` raises "truth value of an array with
        more than one element is ambiguous" and killed the node on its first
        CameraInfo message. Unit tests did not catch it because they set _fx
        directly; the live run did, immediately.
        """
        from sensor_msgs.msg import CameraInfo
        info = CameraInfo()
        info.k = np.array([554.3, 0.0, 320.0,
                           0.0, 554.3, 240.0,
                           0.0, 0.0, 1.0], dtype=np.float64)
        self.node._fx = None
        self.node._on_info(info)              # must not raise
        self.assertAlmostEqual(self.node._fx, 554.3, places=3)

    def test_camera_info_with_zero_intrinsics_is_ignored(self):
        from sensor_msgs.msg import CameraInfo
        info = CameraInfo()
        self.node._fx = None
        self.node._on_info(info)
        self.assertIsNone(self.node._fx)

    def test_marker_px_uses_the_longest_side(self):
        quad = np.array([[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]])
        self.assertAlmostEqual(QrNode._marker_px(quad), 40.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
