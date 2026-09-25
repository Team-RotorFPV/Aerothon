"""The payload is physically dropped in Gazebo, and the camera confirms it.

    Gazebo      a 100 g yellow box on the Iris's winch hook (a prismatic joint
                driven to winch_ctrl's payout), held by a DetachableJoint that
                winch_ctrl's release lets go of.
    Camera      perception_redzone.payload finds it; WinchDrop only calls the
                drop done when the nadir camera sees it ON THE GROUND at the
                size the altitude predicts, after the hook has wound back up.
    Grading     check_track reads the payload's true final position.
"""

import json
import math
import os
import sys
import unittest
from pathlib import Path

import numpy as np
import py_trees

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src/aerothon_perception/perception_redzone"))
sys.path.insert(0, str(ROOT / "src/aerothon_payload/winch_ctrl"))
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "sim"))

from perception_redzone.payload import detect_payload  # noqa: E402
from test_fail_closed_stages import FakeMav  # noqa: E402

HFOV = 1.0472


def frame(w=1280, h=720, bg=(120, 120, 120)):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = bg
    return img


def payload_px(alt, size_m=0.12, w=1280):
    return size_m / (2 * alt * math.tan(HFOV / 2)) * w


class DetectorTests(unittest.TestCase):

    def test_a_yellow_payload_is_found_and_located(self):
        img = frame()
        img[400:430, 700:730] = (10, 215, 255)          # BGR yellow
        d = detect_payload(img)
        self.assertTrue(d["visible"])
        self.assertAlmostEqual(d["x"], (715 - 640) / 640, places=2)
        self.assertAlmostEqual(d["y"], (415 - 360) / 360, places=2)
        self.assertEqual(d["w_px"], 30)

    def test_the_arena_colours_are_not_the_payload(self):
        img = frame()
        img[100:300, 100:400] = (13, 107, 255)          # return lane orange
        img[400:600, 500:800] = (5, 5, 240)             # red zone
        img[50:200, 900:1200] = (97, 194, 128)          # field green
        img[300:350, 1000:1100] = (255, 255, 255)       # QR white
        self.assertFalse(detect_payload(img)["visible"])

    def test_a_speck_is_not_a_payload(self):
        img = frame()
        img[10:13, 10:13] = (10, 215, 255)
        self.assertFalse(detect_payload(img)["visible"])


class WinchGazeboBackendTests(unittest.TestCase):
    """backend:=gazebo drives the hook and lets the payload go."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from winch_ctrl.winch_node import WinchNode
        self.String = String
        self.node = WinchNode()
        self.node._enable_gazebo()
        self.payout, self.detach = [], []
        self.node.pub_gz_payout.publish = lambda m: self.payout.append(m.data)
        self.node.pub_gz_detach.publish = lambda m: self.detach.append(m)
        self.node.pub_status.publish = lambda m: None
        p = PoseStamped()
        p.pose.position.z = 5.0
        self.node._on_pose(p)
        for _ in range(10):
            self.node._on_vel(TwistStamped())

    def tearDown(self):
        self.node.destroy_node()

    def tick(self, n, dt=0.1):
        for _ in range(n):
            self.node._last_tick = self.node._now() - dt
            self.node._tick()

    def test_lowering_drives_the_hook_down(self):
        self.node._on_cmd(self.String(data="lower"))
        self.tick(30)
        self.assertGreater(self.payout[-1], self.payout[0])
        self.assertAlmostEqual(self.payout[-1], self.node.payout, places=6)
        self.assertEqual(self.detach, [])

    def test_release_detaches_the_payload_and_resends(self):
        self.node._on_cmd(self.String(data="lower"))
        for _ in range(400):
            self.node.integrate(0.1)
            if self.node.at_ground():
                break
        self.node._on_cmd(self.String(data="release"))
        self.assertTrue(self.node.released)
        self.tick(8)
        self.assertEqual(len(self.detach), 5)


class CameraConfirmationTests(unittest.TestCase):
    """WinchDrop phases 3-5: stow over the pad, confirm with the camera, climb."""

    def build(self, det, alt=5.0, pad_in_frame=True):
        from mission_bt.mission_tree import WinchDrop
        mav = FakeMav()
        mav._pos = (20.0, 3.0, alt)
        mav._alt = alt
        mav.qr_off = (0.0, 0.0) if pad_in_frame else None
        mav.winch = lambda c: None
        mav.winch_status = {"released": True, "payout_m": 0.0}
        mav.payload_seen = (lambda: det) if not callable(det) else det
        leaf = WinchDrop(mav, 5.0, 10.0, hfov_rad=HFOV, image_w_px=1280,
                         image_h_px=720, confirm_timeout_ticks=40)
        leaf.initialise()
        leaf.phase = 3
        leaf.drop_x, leaf.drop_y = 20.0, 3.0
        return leaf, mav

    def run_until(self, leaf, phase, n=100):
        for _ in range(n):
            leaf.update()
            if leaf.phase >= phase:
                return

    def test_a_payload_on_the_ground_is_confirmed_and_measured_against_the_pad(self):
        px = payload_px(5.0)
        det = {"visible": True, "x": 0.1, "y": 0.0, "w_px": round(px),
               "h_px": round(px), "img_w": 1280}
        leaf, mav = self.build(det)
        self.run_until(leaf, 5)
        self.assertIs(mav.delivery_confirmed, True)
        half_w = 5.0 * math.tan(HFOV / 2)
        self.assertAlmostEqual(mav.delivery_offset_m, 0.1 * half_w, places=3)
        self.assertIn("confirmed on the ground by the camera", mav.delivery_note)

    def test_it_waits_for_the_hook_to_wind_up_first(self):
        px = payload_px(5.0)
        det = {"visible": True, "x": 0.0, "y": 0.0, "w_px": round(px), "h_px": round(px)}
        leaf, mav = self.build(det)
        mav.winch_status = {"released": True, "payout_m": 3.0}   # still paying in
        for _ in range(20):
            leaf.update()
        self.assertEqual(leaf.phase, 3)
        self.assertIsNone(mav.delivery_confirmed)

    def test_a_blob_of_the_wrong_size_is_not_a_payload_on_the_ground(self):
        """Still on the hook just under the camera it would look huge."""
        det = {"visible": True, "x": 0.0, "y": 0.0, "w_px": 250, "h_px": 250}
        leaf, mav = self.build(det)
        self.run_until(leaf, 5)
        self.assertIs(mav.delivery_confirmed, False)
        self.assertIn("NOT confirmed", mav.delivery_note)
        self.assertIn("px", mav.delivery_note)

    def test_no_payload_in_view_is_not_confirmed(self):
        leaf, mav = self.build({"visible": False})
        self.run_until(leaf, 5)
        self.assertIs(mav.delivery_confirmed, False)

    def test_one_frame_is_not_enough(self):
        px = payload_px(5.0)
        good = {"visible": True, "x": 0.0, "y": 0.0, "w_px": round(px), "h_px": round(px)}
        seq = iter([good] + [{"visible": False}] * 200)
        leaf, mav = self.build(lambda: next(seq))
        self.run_until(leaf, 5)
        self.assertIs(mav.delivery_confirmed, False)

    def test_after_confirming_it_climbs_back(self):
        px = payload_px(5.0)
        det = {"visible": True, "x": 0.0, "y": 0.0, "w_px": round(px), "h_px": round(px)}
        leaf, mav = self.build(det)
        self.run_until(leaf, 5)
        mav._pos = (20.0, 3.0, 10.0)
        self.assertEqual(leaf.update(), py_trees.common.Status.SUCCESS)


class LandOutcomeTests(unittest.TestCase):

    def land(self, confirmed):
        from mission_bt.mission_tree import Land
        mav = FakeMav()
        mav.results = []
        mav.delivery_confirmed = confirmed
        mav.delivery_offset_m = 0.2
        mav.state = type("S", (), {"armed": False, "connected": True})()
        mav.connected = lambda: True
        mav.land = lambda: None
        mav.expect_disarm = lambda v=True: None
        leaf = Land(mav)
        leaf.initialise()
        leaf.update()
        return mav.results[-1]

    def test_an_unconfirmed_drop_is_not_reported_completed(self):
        state, reason = self.land(False)
        self.assertEqual(state, "DELIVERY_UNCONFIRMED")
        self.assertIn("NOT confirmed", reason)

    def test_a_confirmed_drop_is_completed(self):
        state, reason = self.land(True)
        self.assertEqual(state, "COMPLETED")
        self.assertIn("seen on the ground by the camera", reason)


class GroundTruthGradingTests(unittest.TestCase):

    LAYOUT = {"home_world": [-2.0, 2.0], "geofence_rect": [-30, 80, -40, 40],
              "delivery_zone_rect": [34.0, -2.0, 40.0, 30.0],
              "gate": [2.0, 2.0, 0.0], "red_zones": {},
              "pads": {"c": [23.0, 1.0]}}

    def track(self, pay_final):
        rows = []
        for i in range(20):
            rows.append({"t": i, "x": 25.0, "y": -1.0, "z": 5.0, "armed": True,
                         "state": "WINCH_DROP", "pay": (-2.0, 2.0, 0.12)})
        rows.append({"t": 21, "x": 0.1, "y": 0.1, "z": 0.0, "armed": False,
                     "state": "COMPLETED", "pay": pay_final})
        return rows

    def test_a_payload_on_the_pad_passes(self):
        from check_track import grade
        g = grade(self.track((23.2, 1.1, 0.04)), self.LAYOUT, "c")
        self.assertTrue(g["checks"]["payload released (on the ground, off the aircraft)"])
        self.assertTrue(g["checks"]["payload on the target pad (within 1 m)"])
        self.assertAlmostEqual(g["payload_error_m"], math.hypot(0.2, 0.1), places=2)

    def test_a_payload_that_came_home_on_the_hook_fails(self):
        from check_track import grade
        # Final payload at the aircraft (home), just under it.
        g = grade(self.track((-1.9, 2.1, 0.12)), self.LAYOUT, "c")
        self.assertFalse(g["checks"]["payload released (on the ground, off the aircraft)"])
        self.assertFalse(g["pass"])

    def test_a_payload_off_the_pad_fails(self):
        from check_track import grade
        g = grade(self.track((26.0, 1.0, 0.04)), self.LAYOUT, "c")
        self.assertFalse(g["checks"]["payload on the target pad (within 1 m)"])


class GazeboWiringTests(unittest.TestCase):

    def test_the_vehicle_carries_a_winch_and_a_detachable_joint(self):
        import materialize_vehicle_model as mv
        hw = mv.COMPETITION_HARDWARE
        self.assertIn('<joint name="winch_joint" type="prismatic">', hw)
        # The line must not be a rigid rod on the airframe: a rod put 1.3 kg m^2
        # on the attitude loop at 3.6 m of payout and the aircraft rolled over.
        self.assertIn('<joint name="winch_swing" type="universal">', hw)
        self.assertIn("<parent>winch_pulley</parent><child>winch_hook</child>", hw)
        self.assertIn("<child_model>aerothon_payload</child_model>", hw)
        self.assertIn("<topic>/aerothon/winch/payout</topic>", hw)
        self.assertIn("<detach_topic>/aerothon/payload/detach</detach_topic>", hw)
        # And it is still XML: a "--" in a comment made gz refuse the model.
        import xml.etree.ElementTree as ET
        for k in ("@CAMERA_W@", "@CAMERA_H@", "@CAMERA_HFOV@", "@LIDAR_SAMPLES@"):
            hw = hw.replace(k, "1")
        ET.fromstring(hw)

    def test_the_world_has_the_payload_and_the_spec_moves_it_with_the_spawn(self):
        import world_spec as W
        from test_world_spec import build
        s = W.default_spec()
        s["takeoff"] = {"x": -3.0, "y": 0.0, "yaw_deg": 30.0}
        root, lay, _ = build(s)
        for m in root.iter("model"):
            if m.get("name") == "aerothon_payload":
                x, y = (float(v) for v in m.find("pose").text.split()[:2])
        sx, sy = W.spawn_point(s)
        self.assertAlmostEqual(x, sx, places=2)
        self.assertAlmostEqual(y, sy, places=2)

    def test_the_bridge_carries_payout_detach_and_ground_truth(self):
        cfg = (ROOT / "src/aerothon_sim/sim_gazebo/config/gz_bridge.yaml").read_text()
        for topic in ("/aerothon/winch/payout", "/aerothon/payload/detach",
                      "/model/aerothon_payload/pose"):
            self.assertIn(topic, cfg)


if __name__ == "__main__":
    unittest.main()
