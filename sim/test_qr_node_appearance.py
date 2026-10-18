#!/usr/bin/env python3
"""The QR node's pad decision, on rendered codes, as the mission sees it.

The start marker is captured while no target exists; the mission then names
the target -- a decoded payload, or APPEARANCE_ONLY when the codes carry
nothing -- and each pad is matched by its pattern first. These pin that the
right pad matches, a one-character-different pad never does, a shared
identifier cannot outvote two plainly different patterns, and each START
drops the previous flight's reference.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_qr_node_appearance.py -v
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in (("aerothon_perception", "camera_ctrl"),
            ("aerothon_perception", "perception_qr")):
    sys.path.insert(0, os.path.join(ROOT, "src", *pkg))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402
import rclpy                                                # noqa: E402
from cv_bridge import CvBridge                              # noqa: E402
from rclpy.parameter import Parameter                       # noqa: E402
from std_msgs.msg import String                             # noqa: E402

from perception_qr.qr_match import APPEARANCE_ONLY         # noqa: E402
from perception_qr.qr_node import QrNode                    # noqa: E402


def frame(text, px, turns=0):
    code = cv2.QRCodeEncoder.create().encode(text)
    code = cv2.resize(code, (px, px), interpolation=cv2.INTER_NEAREST)
    border = max(2, int(px * 0.0762))
    sheet = np.rot90(cv2.copyMakeBorder(code, border, border, border, border,
                                        cv2.BORDER_CONSTANT, value=255), turns)
    img = np.full((720, 1280), 100, np.uint8)
    y, x = 360 - sheet.shape[0] // 2, 640 - sheet.shape[1] // 2
    img[y:y + sheet.shape[0], x:x + sheet.shape[1]] = sheet
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


@unittest.skipUnless(hasattr(cv2, "QRCodeEncoder"), "OpenCV without QRCodeEncoder")
class QrNodeAppearanceTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.bridge = CvBridge()
        self.node = QrNode()
        self.detail = []
        self.node.pub_detail.publish = lambda m: self.detail.append(json.loads(m.data))
        for pub in (self.node.pub_matched, self.node.pub_offset,
                    self.node.pub_decoded, self.node.pub_annot):
            pub.publish = lambda m: None
        self.stamp = 1.0

    def tearDown(self):
        self.node.destroy_node()

    def see(self, text, px=200, turns=0):
        message = self.bridge.cv2_to_imgmsg(frame(text, px, turns), "bgr8")
        message.header.stamp.sec = int(self.stamp)
        self.stamp += 1.0
        self.node.on_image(message)
        return self.detail[-1]

    def target(self, value):
        self.node.on_target(String(data=value))

    def test_a_decoded_start_matches_only_its_own_pad(self):
        self.target("")
        start = self.see("B")
        self.assertTrue(start["reference_captured"])
        self.assertEqual(start["reference_kind"], "modules")
        self.assertFalse(start["matched"], "the start marker cannot be its own target")
        self.target("B")
        self.assertTrue(self.see("B", px=140, turns=2)["matched"])
        for other in ("A", "C", "D"):
            self.assertFalse(self.see(other, px=140, turns=1)["matched"], other)

    def test_codes_that_carry_nothing_match_by_pattern_alone(self):
        self.node.set_parameters([Parameter("use_payloads", value=False)])
        self.target("")
        start = self.see("TARGET B")
        self.assertEqual(start["accepted"], "")
        self.assertTrue(start["reference_captured"])
        self.target(APPEARANCE_ONLY)
        pad = self.see("TARGET B", px=150, turns=3)
        self.assertTrue(pad["matched"])
        self.assertEqual(pad["match_basis"], ["appearance"])
        for other in ("TARGET A", "TARGET C", "B"):
            self.assertFalse(self.see(other, px=150)["matched"], other)

    def test_a_shared_identifier_does_not_outvote_a_different_pattern(self):
        self.target("")
        self.see("ZONE 2 PAD 1")
        self.target("ZONE 2 PAD 1")
        self.assertFalse(self.see("2", px=150)["matched"])
        self.assertTrue(self.see("ZONE 2 PAD 1", px=150)["matched"])

    def test_each_start_captures_its_own_marker(self):
        self.target("")
        self.see("A")
        self.target("A")
        self.target("")                 # the next flight's START
        self.assertIsNone(self.node._reference)
        self.see("C")
        self.target(APPEARANCE_ONLY)
        self.assertTrue(self.see("C", px=150)["matched"])
        self.assertFalse(self.see("A", px=150)["matched"])


if __name__ == "__main__":
    unittest.main()
