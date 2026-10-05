#!/usr/bin/env python3
"""Detectors work only while the camera points where they have something to see.

The banner detector ran over the nadir view of the grass for the whole
delivery search and the QR reader into the sky through the corridor: most of
a Pi 5 core spent measuring nothing. camera_ctrl.gate.CameraGate follows the
camera's REQUESTED pose. These pin that it closes only on a known, fresh
pose; that a closed detector says "nothing" rather than falling silent (the
mission keeps the last message, so silence would keep a stale detection
alive); and that it opens again the moment the camera turns back.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_camera_gate.py -v
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in (("aerothon_perception", "camera_ctrl"),
            ("aerothon_perception", "perception_banner"),
            ("aerothon_perception", "perception_qr")):
    sys.path.insert(0, os.path.join(ROOT, "src", *pkg))

import cv2                                                  # noqa: E402
import rclpy                                                # noqa: E402
from cv_bridge import CvBridge                              # noqa: E402
from std_msgs.msg import String                             # noqa: E402

from perception_banner.banner_node import BannerNode        # noqa: E402
from perception_qr.qr_node import QrNode                    # noqa: E402

BANNER = cv2.imread(os.path.join(ROOT, "tests", "fixtures", "banner_sim_ambient_3m.png"))


def pose_state(requested):
    return String(data=json.dumps({"requested": requested, "settled": True}))


class CameraGateTests(unittest.TestCase):

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
        self.banner = BannerNode()
        self.vec, self.detail = [], []
        self.banner.pub.publish = self.vec.append
        self.banner.pub_detail.publish = self.detail.append
        self.banner.pub_annot.publish = lambda m: None

    def tearDown(self):
        self.banner.destroy_node()

    def feed(self):
        self.banner.on_image(self.bridge.cv2_to_imgmsg(BANNER, "bgr8"))
        return json.loads(self.detail[-1].data)

    def test_with_no_camera_state_the_detector_runs(self):
        """Never blind because the camera controller is."""
        self.assertTrue(self.feed()["identified"])

    def test_looking_down_the_banner_detector_says_nothing_is_there(self):
        self.banner.gate._on_state(pose_state("NADIR"))
        d = self.feed()
        self.assertFalse(d["identified"])
        self.assertEqual(self.vec[-1].z, 0.0, "a stale identification survived")

    def test_it_opens_again_when_the_camera_turns_back(self):
        self.banner.gate._on_state(pose_state("NADIR"))
        self.feed()
        self.banner.gate._on_state(pose_state("BANNER"))
        self.assertTrue(self.feed()["identified"])

    def test_a_stale_camera_state_does_not_keep_it_closed(self):
        self.banner.gate._on_state(pose_state("NADIR"))
        self.banner.gate._t -= 10.0
        self.assertTrue(self.feed()["identified"])

    def test_the_qr_reader_clears_its_outputs_while_looking_ahead(self):
        qr = QrNode()
        try:
            sent = []
            qr.pub_matched.publish = sent.append
            qr.pub_offset.publish = sent.append
            qr.pub_decoded.publish = sent.append
            qr.gate._on_state(pose_state("FORWARD"))
            qr.on_image(self.bridge.cv2_to_imgmsg(BANNER, "bgr8"))
            self.assertEqual([getattr(m, "data", getattr(m, "z", None)) for m in sent],
                             [0.0, "", False])
        finally:
            qr.destroy_node()


if __name__ == "__main__":
    unittest.main()
