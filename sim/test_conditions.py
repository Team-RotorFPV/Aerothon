#!/usr/bin/env python3
"""The day's conditions: from a world spec to what the stack actually receives.

world_spec.conditions() resolves a preset (calm / field / worst / random);
materialize_world.py turns it into Gazebo wind, worn markers and SITL fault
parameters; sim_gazebo/degrade_node.py applies the camera, lidar and GPS
faults in flight. Each hop is pinned here, so a "worst" run cannot quietly
be a calm one with a different name.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_conditions.py -v
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, LaserScan

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src" / "aerothon_sim" / "sim_gazebo"))

import materialize_world as MW                              # noqa: E402
import world_spec as W                                      # noqa: E402
from sim_gazebo import degrade_node                         # noqa: E402

TEMPLATE = ROOT / "src/aerothon_sim/sim_gazebo/worlds/mission2.sdf"
ASSETS = ROOT / "src/aerothon_sim/sim_gazebo/materials"


def spec_with(conditions):
    s = W.default_spec()
    s["conditions"] = conditions
    return s


class ResolutionTests(unittest.TestCase):

    def test_no_conditions_is_calm(self):
        c = W.conditions(W.default_spec())
        self.assertEqual(c["wind"]["speed"], 0.0)
        self.assertFalse(any(c["camera"][k] for k in W.CAMERA_KEYS))
        self.assertEqual(c["fcu"]["gps_glitch_m"], 0.0)

    def test_a_preset_fills_every_field_and_overrides_win(self):
        c = W.conditions(spec_with({"preset": "worst", "wind": {"dir_deg": 45}}))
        self.assertEqual(c["wind"]["speed"], 8.0)
        self.assertEqual(c["wind"]["dir_deg"], 45)
        self.assertEqual(set(c), set(W.conditions(W.default_spec())))

    def test_random_is_repeatable_per_seed_and_differs_between_seeds(self):
        s = spec_with({"preset": "random"})
        self.assertEqual(W.conditions(s, 7), W.conditions(s, 7))
        self.assertNotEqual(W.conditions(s, 7), W.conditions(s, 8))

    def test_random_stays_between_calm_and_worst(self):
        worst = W.conditions(spec_with({"preset": "worst"}))
        for seed in range(50):
            c = W.conditions(spec_with({"preset": "random"}), seed)
            self.assertFalse(W.validate(spec_with({"preset": "random"}))[0])
            for group in ("camera", "lidar", "wear"):
                for k, v in c[group].items():
                    self.assertLessEqual(abs(v), abs(worst[group][k]) + 1e-9, (group, k))
            self.assertLessEqual(c["wind"]["speed"], worst["wind"]["speed"])

    def test_nonsense_is_refused(self):
        for bad in ({"preset": "hurricane"}, {"wind": {"speedd": 3}},
                    {"camera": {"noise": 9}}, {"lidar": {"dropout": 2.0}},
                    {"weather": {}}):
            with self.subTest(bad=bad):
                self.assertTrue(W.validate(spec_with(bad))[0])


class MaterialiseTests(unittest.TestCase):

    def build(self, conditions):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            sp = d / "spec.json"
            sp.write_text(json.dumps(spec_with(conditions)))
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith("AEROTHON_")}
            subprocess.run(
                [sys.executable, str(ROOT / "scripts/materialize_world.py"),
                 "--source", str(TEMPLATE), "--assets", str(ASSETS),
                 "--output", str(d / "w.sdf"), "--world-spec", str(sp),
                 "--conditions-out", str(d / "c.json"),
                 "--sitl-params-out", str(d / "c.parm")],
                check=True, capture_output=True, env=env)
            return (ET.parse(d / "w.sdf").getroot().find("world"),
                    json.loads((d / "c.json").read_text()),
                    (d / "c.parm").read_text())

    def test_calm_has_no_wind_system(self):
        world, _, _ = self.build({"preset": "calm"})
        self.assertIsNone(world.find("wind"))
        self.assertFalse([p for p in world.findall("plugin")
                          if p.get("name").endswith("WindEffects")])

    def test_wind_is_the_heading_and_speed_asked_for(self):
        world, _, _ = self.build({"preset": "worst", "wind": {"dir_deg": 90}})
        v = [float(x) for x in world.findtext("wind/linear_velocity").split()]
        self.assertAlmostEqual(v[0], 0.0, places=2)
        self.assertAlmostEqual(v[1], 8.0, places=2)
        plugin = [p for p in world.findall("plugin")
                  if p.get("name").endswith("WindEffects")][0]
        self.assertAlmostEqual(float(plugin.findtext(
            "horizontal/magnitude/sin/amplitude_percent")), 0.5)

    def test_the_wind_push_is_the_airframes_drag_at_the_gust_peak(self):
        """WindEffects: F = m k v_rel. At 12 m/s that must be the quadratic
        drag of 0.05 m^2 of airframe, 0.5 rho CdA v^2."""
        m = MW.airframe_body_kg()
        sdf = MW.wind_sdf({"speed": 8.0, "dir_deg": 0.0, "gust": 4.0,
                           "gust_period_s": 6.0, "veer_deg": 0.0}, m)
        k = float(ET.fromstring(f"<w>{sdf}</w>").findtext(
            "plugin/force_approximation_scaling_factor"))
        self.assertAlmostEqual(m * k * 12.0, 0.5 * 1.2 * 0.05 * 12.0 ** 2, places=2)

    def test_worn_markers_are_faded_towards_dust(self):
        fresh, _, _ = self.build({"preset": "calm"})
        worn, _, _ = self.build({"preset": "calm", "wear": {"banner": 4, "qr": 4}})

        def diffuse(world, visual):
            for v in world.iter("visual"):
                if v.get("name") == visual:
                    return [float(x) for x in v.findtext("material/diffuse").split()[:3]]
        g0, g1 = diffuse(fresh, "banner_board"), diffuse(worn, "banner_board")
        self.assertLess(g1[1] - g1[0], g0[1] - g0[0], "green no less saturated")
        self.assertLess(diffuse(worn, "target_a_white_plate")[2],
                        diffuse(fresh, "target_a_white_plate")[2])

    def test_sensor_faults_become_sitl_parameters(self):
        _, cond, parm = self.build({"preset": "worst"})
        params = dict(line.split() for line in parm.splitlines()
                      if line and not line.startswith("#"))
        self.assertEqual(float(params["SIM_GPS_NOISE"]), cond["fcu"]["gps_noise_m"])
        self.assertEqual(float(params["SIM_BATT_VOLTAGE"]), cond["fcu"]["battery_v"])
        self.assertIn("SIM_BARO_DRIFT", params)
        self.assertNotIn("SIM_GPS_GLITCH_X", params, "a glitch from boot is an offset")

    def test_the_team_airframe_feels_the_wind(self):
        src = (ROOT / "scripts/build_cad_vehicle.py").read_text()
        self.assertIn("<link name='base_link'><enable_wind>true</enable_wind>", src)


class DegradeNodeTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def node(self, conditions):
        path = Path(tempfile.mkdtemp()) / "c.json"
        path.write_text(json.dumps(W.conditions(spec_with(conditions))))
        n = degrade_node.DegradeNode(parameter_overrides=[
            Parameter("conditions_file", value=str(path))])
        n.sent_images, n.sent_scans = [], []
        n.pub_image.publish = n.sent_images.append
        n.pub_scan.publish = n.sent_scans.append
        self.addCleanup(n.destroy_node)
        return n

    def image(self):
        m = Image()
        m.height, m.width, m.encoding, m.step = 72, 128, "bgr8", 128 * 3
        m.data = (np.random.default_rng(0).integers(0, 255, 72 * 128 * 3)
                  .astype(np.uint8).tobytes())
        m.header.stamp.sec = 42
        return m

    def scan(self):
        s = LaserScan()
        s.range_min, s.range_max = 0.05, 12.0
        s.ranges = [3.0] * 360
        return s

    def test_calm_passes_the_very_same_messages_through(self):
        n = self.node({"preset": "calm"})
        img, sc = self.image(), self.scan()
        n.on_image(img)
        n.on_scan(sc)
        self.assertIs(n.sent_images[0], img)
        self.assertEqual(list(n.sent_scans[0].ranges), [3.0] * 360)

    def test_the_worst_day_changes_the_frame_but_keeps_its_capture_stamp(self):
        n = self.node({"preset": "worst", "camera": {"frame_drop": 0.0,
                                                     "latency_ms": 0}})
        img = self.image()
        n.on_image(img)
        out = n.sent_images[0]
        self.assertNotEqual(bytes(out.data), bytes(img.data))
        self.assertEqual(out.header.stamp.sec, 42)

    def test_frames_are_dropped_at_the_rate_asked_for(self):
        n = self.node({"camera": {"frame_drop": 0.3}})
        for _ in range(1000):
            n.on_image(self.image())
        self.assertAlmostEqual(len(n.sent_images) / 1000, 0.7, delta=0.05)

    def test_late_frames_are_held_until_their_time(self):
        n = self.node({"camera": {"latency_ms": 200}})
        n.on_image(self.image())
        n.release()
        self.assertEqual(n.sent_images, [])
        n.pending[0] = (0.0, n.pending[0][1])
        n.release()
        self.assertEqual(len(n.sent_images), 1)

    def test_the_lidar_loses_and_invents_returns(self):
        n = self.node({"lidar": {"dropout": 0.2, "spurious": 0.05}})
        n.on_scan(self.scan())
        r = np.array(n.sent_scans[0].ranges)
        self.assertGreater(np.isinf(r).mean(), 0.1)
        self.assertGreater((r < 2.0).mean(), 0.02)

    def test_the_gps_glitch_is_injected_after_arming_then_cleared(self):
        n = self.node({"fcu": {"gps_glitch_m": 5.0, "glitch_at_s": 10.0,
                               "glitch_s": 3.0}})
        calls = []
        n._set_glitch = lambda x, y: calls.append(round(math.hypot(x, y), 3)) or True
        clock = [100.0]
        n._now = lambda: clock[0]
        n.glitch_tick()
        self.assertEqual(calls, [], "glitched before arming")
        n.armed_at = 100.0
        clock[0] = 109.0
        n.glitch_tick()
        self.assertEqual(calls, [])
        clock[0] = 110.5
        n.glitch_tick()
        self.assertEqual(calls, [5.0])
        clock[0] = 113.6
        n.glitch_tick()
        self.assertEqual(calls, [5.0, 0.0])
        n.glitch_tick()
        self.assertEqual(calls, [5.0, 0.0], "glitched twice")


if __name__ == "__main__":
    unittest.main()
