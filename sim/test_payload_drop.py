"""The payload is physically dropped in Gazebo, and the camera confirms it.

    Gazebo      a 100 g yellow box on the Iris's winch hook (a prismatic joint
                driven to winch_ctrl's payout), held by a DetachableJoint that
                winch_ctrl's release lets go of.
    Camera      perception_redzone.payload finds it; WinchDrop only calls the
                drop done when the nadir camera sees it ON THE GROUND at the
                size the altitude predicts, after the hook has wound back up.
    Grading     check_track reads the payload's true final position.
"""

import math
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

    def test_the_colour_band_is_the_real_payloads(self):
        """The rulebook gives no colour: a blue payload needs its own band."""
        img = frame()
        img[400:430, 700:760] = (200, 90, 20)           # BGR blue, 10 x 5 cm
        self.assertFalse(detect_payload(img)["visible"])
        d = detect_payload(img, hsv_lo=(100, 120, 80), hsv_hi=(125, 255, 255))
        self.assertTrue(d["visible"])
        self.assertEqual((d["w_px"], d["h_px"]), (60, 30))


class WinchGazeboBackendTests(unittest.TestCase):
    """The winch commands travel; it never receives simulator contact truth."""
    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok(): rclpy.init()

    def setUp(self):
        from winch_ctrl.winch_node import WinchNode
        self.node = WinchNode()
        self.node._enable_gazebo()

    def tearDown(self): self.node.destroy_node()

    def test_lowering_drives_commanded_travel(self):
        from std_msgs.msg import String
        self.node._on_cmd(String(data='lower'))
        self.node.integrate(1)
        self.assertGreater(self.node.payout, 0)

    def test_altitude_does_not_stop_lowering_as_if_contact_was_measured(self):
        self.node._alt = 2
        self.node.state = 'LOWERING'
        self.node.integrate(10)
        self.assertEqual(self.node.state, 'LOWERING')
        self.node.integrate(20)
        self.assertEqual(self.node.state, 'AT_LIMIT')

    def test_no_simulator_truth_subscription_or_detach_publisher(self):
        import inspect
        from winch_ctrl.winch_node import WinchNode
        code = inspect.getsource(WinchNode)
        self.assertNotIn('/sim/payload_pose', code)
        self.assertNotIn('/winch/gz/detach', code)
        self.assertNotIn('_gravity_hook', code)

    def test_status_reports_estimates_and_unknown_physical_state(self):
        capture = []
        self.node.pub_status.publish = lambda m: capture.append(m.data)
        self.node.publish_status()
        import json
        status = json.loads(capture[-1])
        self.assertIsNone(status['ground'])
        self.assertIsNone(status['released'])
        self.assertIn('no encoder', status['payout_basis'])

    def test_repeated_intents_are_idempotent_and_stop_reaches_backend(self):
        from std_msgs.msg import String
        actions = []
        self.node._send_backend = lambda action: actions.append(action)
        for _ in range(5): self.node._on_cmd(String(data='lower'))
        self.assertEqual(actions, ['lower'])
        self.node._on_cmd(String(data='stop'))
        self.assertEqual(actions, ['lower', 'stop'])
        for _ in range(5): self.node._on_cmd(String(data='stow'))
        self.assertEqual(actions, ['lower', 'stop', 'stow'])


class CameraConfirmationTests(unittest.TestCase):
    def test_exhaustion_stows_before_continuing_without_claiming_delivery(self):
        from mission_bt.mission_tree import WinchDrop
        from mission_bt.delivery_observer import calibration
        mav = FakeMav()
        commands = []
        mav.delivery_load_profile = calibration([.26]*40, [.24]*40)
        mav.winch_status = {'payout_estimate_m': 6, 'fault': ''}
        mav.winch = commands.append
        leaf = WinchDrop(mav)
        leaf.initialise(); leaf.phase = 1
        leaf.observer.started = 0
        leaf.observer.attempt = 3
        leaf.observer._retry(0, 'camera inconclusive')
        self.assertEqual(leaf.update(), py_trees.common.Status.RUNNING)
        self.assertEqual(commands[-1], 'stow')
        self.assertEqual(leaf.phase, 1)
        self.assertEqual(mav.modes, [])
        mav.winch_status['payout_estimate_m'] = 0
        self.assertEqual(leaf.update(), py_trees.common.Status.RUNNING)
        self.assertEqual(leaf.phase, 5)
        self.assertFalse(mav.delivery_confirmed)
        mav._pos = (leaf.drop_x, leaf.drop_y, leaf.cruise_alt)
        self.assertEqual(leaf.update(), py_trees.common.Status.SUCCESS)
        self.assertEqual(mav.modes, [])

    def test_stale_winch_status_cannot_certify_recovery_completion(self):
        from mission_bt.mission_tree import WinchDrop
        mav = FakeMav()
        mav.winch_status = {'payout_estimate_m': 0, 'fault': ''}
        mav.winch_status_fresh = lambda: False
        mav.winch = lambda cmd: None
        leaf = WinchDrop(mav)
        leaf.initialise(); leaf.phase = 1
        leaf.observer.attempt = 3
        leaf.observer._retry(0, 'camera inconclusive')
        self.assertEqual(leaf.update(), py_trees.common.Status.FAILURE)
        self.assertFalse(mav.delivery_confirmed)

    def test_commanded_release_cannot_confirm_delivery(self):
        from mission_bt.mission_tree import WinchDrop
        mav = FakeMav()
        mav.winch_status = {'released': True, 'ground': True, 'payout_m': 0}
        mav.winch = lambda cmd: None
        mav.payload_seen = lambda: {'visible': True, 'stamp': 1, 'w_px': 22, 'h_px': 11}
        leaf = WinchDrop(mav)
        leaf.initialise(); leaf.phase = 1
        self.assertEqual(leaf.update(), py_trees.common.Status.FAILURE)
        self.assertFalse(mav.delivery_confirmed)
        self.assertIn('calibration', mav.delivery_note)


class LandOutcomeTests(unittest.TestCase):

    def land(self, confirmed, basis=None):
        from mission_bt.mission_tree import Land
        mav = FakeMav()
        mav.results = []
        mav.delivery_confirmed = confirmed
        mav.delivery_confirmation_basis = basis
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

    def test_motor_confirmation_is_not_misreported_as_visual_proof(self):
        state, reason = self.land(True, 'calibrated_motor_output_after_retraction')
        self.assertEqual(state, 'COMPLETED')
        self.assertIn('unloaded motor effort', reason)
        self.assertNotIn('seen on the ground by the camera', reason)


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
        for topic in ("/aerothon/winch/payout",
                      "/model/aerothon_payload/pose"):
            self.assertIn(topic, cfg)


if __name__ == "__main__":
    unittest.main()
