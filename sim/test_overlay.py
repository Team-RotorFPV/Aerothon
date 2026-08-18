#!/usr/bin/env python3
"""The GCS camera pane shows the live frame with everything detected on it.

WHAT WAS WRONG

    App.tsx hardcoded one stream:

        u.pathname = "/stream"; u.search = "?topic=/percep/qr/annotated"

    so the operator saw the QR detector's private copy and nothing else.
    During banner alignment -- the stage where watching the camera matters
    most -- the pane showed a nadir QR view with no banner box on it, because
    every detector drew onto its own image and published its own topic.

    perception_overlay composites once, server-side, from the boxes the
    detectors REPORT. It never re-detects: a node that re-ran detection could
    disagree with the detector the mission is actually flying on, and an
    overlay that contradicts the mission is worse than none.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_overlay.py -v
"""

import json
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_perception", "perception_overlay"))

PARAMS = {"image_topic": "/camera/image", "max_box_age_s": 0.7,
          "draw_status_bar": True}


def make_overlay():
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.declare_parameter'), \
         patch('rclpy.node.Node.get_logger'), \
         patch('rclpy.node.Node.get_parameter',
               side_effect=lambda n: MagicMock(value=PARAMS.get(n))):
        from perception_overlay.overlay_node import Overlay
        node = Overlay()
    node.get_parameter = lambda n: MagicMock(value=PARAMS.get(n))
    return node


def blank(w=320, h=240):
    return np.zeros((h, w, 3), dtype=np.uint8)


def msg(payload):
    m = MagicMock()
    m.data = json.dumps(payload)
    return m


class BoxIngestTests(unittest.TestCase):

    def setUp(self):
        self.n = make_overlay()

    def test_banner_boxes_are_taken_from_the_detector(self):
        self.n._on_detail("banner", msg({
            "identified": True, "text": "AEROTHON",
            "boxes": [{"rect": [10, 20, 60, 30], "label": "BANNER", "ok": True}]}))
        self.assertEqual(len(self.n.fresh("banner")), 1)

    def test_qr_quads_are_taken_from_the_detector(self):
        self.n._on_detail("qr", msg({
            "matched": True, "accepted": "TARGET_C",
            "boxes": [{"quad": [[1, 1], [9, 1], [9, 9], [1, 9]],
                       "label": "TARGET_C", "ok": True}]}))
        self.assertEqual(len(self.n.fresh("qr")), 1)

    def test_malformed_json_is_ignored_not_fatal(self):
        bad = MagicMock()
        bad.data = "{not json"
        self.n._on_detail("qr", bad)
        self.assertEqual(self.n.fresh("qr"), [])

    def test_a_detector_reporting_no_boxes_draws_none(self):
        self.n._on_detail("banner", msg({"identified": False}))
        self.assertEqual(self.n.fresh("banner"), [])

    def test_STALE_boxes_are_not_drawn(self):
        """A box from the last frame the detector managed, sitting on the
        picture after the object has left it, is exactly the lie this node
        exists to avoid."""
        self.n._on_detail("banner", msg({
            "boxes": [{"rect": [1, 1, 5, 5], "label": "B", "ok": True}]}))
        self.n.boxes["banner"] = (0.0, self.n.boxes["banner"][1])
        self.assertEqual(self.n.fresh("banner"), [])


class DrawingTests(unittest.TestCase):

    def setUp(self):
        self.n = make_overlay()

    def painted(self, frame):
        return int(np.count_nonzero(frame))

    def test_nothing_detected_leaves_the_frame_alone(self):
        f = blank()
        PARAMS["draw_status_bar"] = False
        try:
            self.n.draw(f)
            self.assertEqual(self.painted(f), 0)
        finally:
            PARAMS["draw_status_bar"] = True

    def test_a_banner_box_is_actually_DRAWN(self):
        self.n._on_detail("banner", msg({
            "identified": True,
            "boxes": [{"rect": [40, 60, 80, 40], "label": "BANNER", "ok": True}]}))
        f = blank()
        self.n.draw(f)
        self.assertGreater(self.painted(f), 0, "the box was never drawn")

    def test_a_qr_quad_is_actually_DRAWN(self):
        self.n._on_detail("qr", msg({
            "matched": True,
            "boxes": [{"quad": [[20, 20], [80, 20], [80, 80], [20, 80]],
                       "label": "T", "ok": True}]}))
        f = blank()
        self.n.draw(f)
        self.assertGreater(self.painted(f), 0)

    def test_detections_from_DIFFERENT_detectors_appear_together(self):
        """The whole point: one pane showing whatever is currently detected,
        not one detector's private view."""
        self.n._on_detail("banner", msg({
            "boxes": [{"rect": [10, 100, 40, 30], "label": "BANNER", "ok": True}]}))
        self.n._on_detail("qr", msg({
            "boxes": [{"quad": [[200, 30], [260, 30], [260, 90], [200, 90]],
                       "label": "QR", "ok": True}]}))
        f = blank()
        self.n.draw(f)
        left = self.painted(f[:, :160])
        right = self.painted(f[:, 160:])
        self.assertGreater(left, 0, "banner box missing")
        self.assertGreater(right, 0, "QR box missing")

    def test_a_rejected_detection_is_drawn_in_a_DIFFERENT_colour(self):
        self.n._on_detail("banner", msg({
            "boxes": [{"rect": [40, 60, 80, 40], "label": "GREEN, NOT BANNER",
                       "ok": False}]}))
        f = blank()
        self.n.draw(f)
        # Accepted draws pure green; rejected must not.
        self.assertEqual(int(np.count_nonzero(
            (f[:, :, 1] > 200) & (f[:, :, 0] < 50) & (f[:, :, 2] < 50))), 0)

    def test_the_status_bar_names_what_is_seen(self):
        self.n._on_detail("banner", msg({"identified": True,
                                         "text": "AEROTHON", "boxes": []}))
        f = blank()
        self.n.draw(f)
        self.assertGreater(self.painted(f[:22, :]), 0, "no status bar drawn")

    def test_the_decoded_banner_TEXT_reaches_the_operator(self):
        self.n._on_detail("banner", msg({"identified": True,
                                         "text": "AEROTHON", "boxes": []}))
        self.assertIn("AEROTHON", self.n.status["banner"])

    def test_the_camera_POSE_is_shown(self):
        """Which way the camera is pointing is half of reading the picture."""
        self.n._on_pose(msg({"named": "BANNER"}))
        self.assertEqual(self.n.camera_pose, "BANNER")


class NeverReDetectsTests(unittest.TestCase):
    """An overlay that disagrees with the mission is worse than none."""

    def test_the_node_does_not_import_a_detector(self):
        import ast
        import perception_overlay.overlay_node as m
        tree = ast.parse(open(m.__file__).read())
        names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names += [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names.append(node.module or "")
        for n in names:
            self.assertNotIn("perception_qr", n)
            self.assertNotIn("perception_banner", n)
            self.assertNotIn("perception_redzone", n)


if __name__ == "__main__":
    unittest.main(verbosity=2)
