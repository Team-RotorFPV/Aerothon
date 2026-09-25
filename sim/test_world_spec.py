"""User-built arenas: the spec, the world it materialises, the checks.

The editor (tools/world_editor) writes a spec; materialize_world.py places it
into mission2.sdf. These tests pin that every position, size and heading in
the spec reaches the world and the organiser inputs, and that the default
spec rebuilds the shipped arena exactly -- so a custom run and a shipped run
differ only in what the person changed.
"""

import copy
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "sim"))

import world_spec as W  # noqa: E402
import materialize_world as MW  # noqa: E402

TEMPLATE = ROOT / "src/aerothon_sim/sim_gazebo/worlds/mission2.sdf"
ASSETS = ROOT / "src/aerothon_sim/sim_gazebo/materials"


def build(spec):
    """Run the real materialiser on a spec; return (world XML root, layout)."""
    with tempfile.TemporaryDirectory() as d:
        sp = Path(d) / "spec.json"
        sp.write_text(json.dumps(spec))
        out = Path(d) / "world.sdf"
        lay = Path(d) / "layout.json"
        env = dict(os.environ)
        env.pop("AEROTHON_WORLD_SPEC", None)
        env.pop("AEROTHON_RANDOM_ARENA", None)
        r = subprocess.run(
            [sys.executable, str(ROOT / "scripts/materialize_world.py"),
             "--source", str(TEMPLATE), "--assets", str(ASSETS),
             "--output", str(out), "--layout-out", str(lay),
             "--world-spec", str(sp)],
            capture_output=True, text=True, env=env)
        if r.returncode != 0:
            raise AssertionError(r.stdout + r.stderr)
        return ET.fromstring(out.read_text()), json.loads(lay.read_text()), r.stdout


def model_pose(root, name):
    for m in root.iter("model"):
        if m.get("name") == name:
            return [float(v) for v in m.find("pose").text.split()]
    raise KeyError(name)


def custom_spec():
    """Nothing where the shipped arena has it: turned, resized, re-counted."""
    s = W.default_spec()
    s["name"] = "test_custom"
    s["takeoff"] = {"x": 5.0, "y": -20.0, "yaw_deg": 30.0}
    s["corridor"] = {"x": 12.0, "y": -14.0, "yaw_deg": 25.0}
    s["delivery_zone"] = {"x": 45.0, "y": 5.0, "w": 34.0, "h": 26.0}
    s["red_zones"] = [
        {"x": 50.0, "y": 12.0, "w": 5.0, "h": 3.0, "yaw_deg": 40.0},
        {"x": 40.0, "y": -3.0, "w": 4.0, "h": 4.0, "yaw_deg": 0.0},
        {"x": 56.0, "y": -2.0, "w": 3.0, "h": 6.0, "yaw_deg": -15.0},
        {"x": 38.0, "y": 13.0, "w": 2.5, "h": 2.5, "yaw_deg": 10.0},
    ]
    s["pads"] = {"a": {"x": 33.0, "y": 0.0}, "b": {"x": 45.0, "y": 6.0},
                 "c": {"x": 57.0, "y": 10.0}, "d": {"x": 48.0, "y": -5.0},
                 "e": {"x": 34.0, "y": 14.0, "yaw_deg": 20.0}}
    s["decoys"] = [{"x": 25.0, "y": 20.0, "yaw_deg": 90.0}]
    s["geofence"] = {"mode": "polygon", "vertices": [
        [-5.0, -32.0], [70.0, -32.0], [70.0, 26.0], [20.0, 26.0], [-5.0, 0.0]]}
    s["start_target"] = "d"
    s["qr"] = {"start_m": 1.5, "target_m": 2.5}
    return s


class DefaultSpecTests(unittest.TestCase):

    def test_the_default_spec_is_valid(self):
        errs, _ = W.validate(W.default_spec())
        self.assertEqual(errs, [])

    def test_the_default_spec_rebuilds_the_shipped_arena(self):
        root, lay, _ = build(W.default_spec())
        self.assertEqual(lay["gate"], [2.0, 2.0, 0.0])
        self.assertEqual(lay["home_world"], [-2.0, 2.0])
        shipped = MW.shipped_layout()
        self.assertEqual(lay["pads"], {k: [float(a), float(b)]
                                       for k, (a, b) in shipped["pads"].items()})
        # The lanes are rebuilt from the spec; the default rebuilds the
        # shipped geometry: outbound from its banner at (2, 2), the linked
        # return lane from (12, -2) pointing back, obstacles where they were.
        self.assertEqual(model_pose(root, "corridor_outbound")[:2], [2.0, 2.0])
        p = model_pose(root, "corridor_return")
        self.assertEqual(p[:2], [12.0, -2.0])
        self.assertAlmostEqual(abs(p[5]), math.pi, places=3)
        self.assertEqual(model_pose(root, "return_aerothon_banner")[:2], [12.0, -2.0])
        centres = sorted((round(sum(x for x, _ in o["poly"]) / 4, 2),
                          round(sum(y for _, y in o["poly"]) / 4, 2))
                         for o in lay["obstacles"])
        self.assertEqual(centres, [(3.4, -0.95), (5.8, -3.05), (8.2, -0.95),
                                   (10.6, -3.05)])
        self.assertEqual(model_pose(root, "takeoff_landing_zone")[:2], [-1.0, 0.0])
        self.assertEqual(model_pose(root, "start_qr_target_a")[:2], [-1.0, 2.0])
        reds = sorted((v[0], v[1], v[2], v[3]) for v in lay["red_zones"].values())
        self.assertEqual(reds, sorted([(38.0, 5.0, 10.0, 7.0), (29.0, 10.0, 6.0, 4.0),
                                       (40.0, -11.0, 7.0, 4.0)]))


class CustomSpecTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.spec = custom_spec()
        errs, cls.warns = W.validate(cls.spec)
        assert errs == [], errs
        cls.root, cls.lay, cls.stdout = build(cls.spec)

    def test_the_spawn_moves_and_turns_with_the_take_off_area(self):
        inc = [i for i in self.root.iter("include")
               if i.find("name").text == "aerothon_iris"][0]
        x, y, _, _, _, yaw = (float(v) for v in inc.find("pose").text.split())
        sx, sy = W.spawn_point(self.spec)
        self.assertAlmostEqual(x, sx, places=3)
        self.assertAlmostEqual(y, sy, places=3)
        self.assertAlmostEqual(yaw, math.radians(30.0), places=3)
        self.assertEqual(self.lay["home_world"], [round(sx, 3), round(sy, 3)])

    def test_the_corridor_moves_as_one_rigid_body(self):
        yaw = math.radians(25.0)
        # Return banner is (10, -4) from the forward one in corridor frame.
        ex = 12.0 + 10 * math.cos(yaw) + 4 * math.sin(yaw)
        ey = -14.0 + 10 * math.sin(yaw) - 4 * math.cos(yaw)
        p = model_pose(self.root, "return_aerothon_banner")
        self.assertAlmostEqual(p[0], ex, places=2)
        self.assertAlmostEqual(p[1], ey, places=2)
        self.assertAlmostEqual(p[5], yaw, places=3)
        self.assertEqual(self.lay["gate"][:2], [12.0, -14.0])

    def test_every_red_zone_exists_at_its_size_and_heading(self):
        names = [m.get("name") for m in self.root.iter("model")
                 if m.get("name", "").startswith("restricted_red_zone")]
        self.assertEqual(len(names), 4)
        for n, r in enumerate(self.spec["red_zones"], 1):
            p = model_pose(self.root, f"restricted_red_zone_{n}")
            self.assertAlmostEqual(p[0], r["x"], places=3)
            self.assertAlmostEqual(p[5], math.radians(r["yaw_deg"]), places=3)
            self.assertEqual(self.lay["red_zones"][f"restricted_red_zone_{n}"][2:4],
                             [r["w"], r["h"]])

    def test_the_zone_is_resized(self):
        for m in self.root.iter("model"):
            if m.get("name") == "delivery_zone_40x30":
                size = m.find(".//visual/geometry/box/size").text.split()
        self.assertEqual([float(v) for v in size[:2]], [34.0, 26.0])
        hx, hy = self.lay["home_world"]
        self.assertEqual(self.lay["delivery_zone_rect"],
                         [round(45.0 - hx, 3), round(5.0 - hy, 3), 34.0, 26.0])

    def test_the_polygon_fence_is_published_home_local(self):
        hx, hy = self.lay["home_world"]
        self.assertEqual(len(self.lay["geofence_poly"]), 5)
        want = W.fence_polygon(self.spec)
        for (x, y), (wx, wy) in zip(self.lay["geofence_poly"], want):
            self.assertAlmostEqual(x, wx - hx, places=2)
            self.assertAlmostEqual(y, wy - hy, places=2)

    def test_the_publisher_reads_the_polygon(self):
        import publish_delivery_zone as pub
        env = {"AEROTHON_GEOFENCE": ";".join(
            f"{x:.3f},{y:.3f}" for x, y in self.lay["geofence_poly"])}
        verts = pub.geofence_from_env(env)
        self.assertEqual(len(verts), 5)
        payload = json.loads(pub.geofence_payload(verts, -35.36, 149.16))
        self.assertEqual(len(payload["vertices"]), 5)

    def test_pads_decoys_and_qr_settings_come_from_the_spec(self):
        p = model_pose(self.root, "delivery_qr_target_e")
        self.assertEqual(p[:2], [34.0, 14.0])
        self.assertAlmostEqual(p[5], math.radians(20.0), places=3)
        decoys = [m.get("name") for m in self.root.iter("model")
                  if m.get("name", "").startswith("green_decoy")]
        self.assertEqual(decoys, ["green_decoy_1"])
        self.assertIn("start pad names delivery target D", self.stdout)

    def test_the_ground_covers_the_fence(self):
        for m in self.root.iter("model"):
            if m.get("name") == "site_ground":
                sx, sy = (float(v) for v in m.find(".//visual/geometry/box/size").text.split()[:2])
                cx, cy = (float(v) for v in m.find(".//visual/pose").text.split()[:2])
        for x, y in W.fence_polygon(self.spec):
            self.assertLessEqual(abs(x - cx), sx / 2)
            self.assertLessEqual(abs(y - cy), sy / 2)

    def test_the_grader_reads_sized_headed_zones_and_the_polygon(self):
        from check_track import fence_polygon, red_polygons, rect_gap
        reds = dict(red_polygons(self.lay))
        hx, hy = self.lay["home_world"]
        # The first zone's centre is inside it; a point 1 m beyond its
        # rotated long edge is 1 m away.
        r = self.spec["red_zones"][0]
        poly = reds["restricted_red_zone_1"]
        self.assertEqual(rect_gap(poly, r["x"] - hx, r["y"] - hy), 0.0)
        yaw = math.radians(r["yaw_deg"])
        nx, ny = -math.sin(yaw), math.cos(yaw)          # long edge's normal
        px = r["x"] - hx + nx * (r["h"] / 2 + 1.0)
        py = r["y"] - hy + ny * (r["h"] / 2 + 1.0)
        self.assertAlmostEqual(rect_gap(poly, px, py), 1.0, places=3)
        self.assertEqual(len(fence_polygon(self.lay)), 5)


class SeparateCorridorTests(unittest.TestCase):
    """The return corridor placed on its own, with obstacles of the user's."""

    @classmethod
    def setUpClass(cls):
        s = W.default_spec()
        s["name"] = "split"
        s["corridor"].update({"length": 12.0, "width": 4.0, "wall_height": 4.0})
        s["return_corridor"] = {
            "linked": False, "x": 13.0, "y": -9.0, "yaw_deg": 200.0,
            "length": 9.0, "width": 3.0, "wall_height": 3.4,
            "obstacles": [{"u": 3.0, "v": 0.6, "w": 0.4, "d": 1.2, "h": 3.5},
                          {"u": 6.0, "v": -0.5, "w": 0.5, "d": 1.0, "h": 3.4,
                           "yaw_deg": 20.0}]}
        s["red_zones"] = [{"x": 40.0, "y": 8.0, "w": 5.0, "h": 4.0, "yaw_deg": 0.0}]
        cls.spec = s
        errs, _ = W.validate(s)
        assert errs == [], errs
        cls.root, cls.lay, _ = build(s)

    def test_the_return_corridor_is_where_it_was_put(self):
        p = model_pose(self.root, "corridor_return")
        self.assertEqual(p[:2], [13.0, -9.0])
        self.assertAlmostEqual(p[5], math.radians(200.0), places=3)
        self.assertEqual(model_pose(self.root, "return_aerothon_banner")[:2], [13.0, -9.0])
        self.assertEqual(self.lay["corridors"]["return"][3:], [9.0, 3.0, 3.4])
        self.assertEqual(self.lay["corridors"]["outbound"][3:], [12.0, 4.0, 4.0])

    def test_the_obstacles_are_the_users(self):
        names = []
        for m in self.root.iter("model"):
            if m.get("name") == "corridor_return":
                names = [v.get("name") for v in m.iter("visual")]
        self.assertEqual(sorted(n for n in names if n.startswith("obstacle")),
                         ["obstacle_1", "obstacle_2"])
        self.assertEqual(len(self.lay["obstacles"]), 2)
        self.assertEqual(self.lay["obstacles"][0]["h"], 3.5)

    def _track_along(self, lane, across, z=3.0):
        """A straight flight down a lane's centre (offset `across`)."""
        from check_track import to_lane  # noqa: F401
        x0, y0, yaw, L = lane[:4]
        hx, hy = self.lay["home_world"]
        rows = []
        for i in range(0, 60):
            u = -1.0 + (L + 2.0) * i / 59
            wx = x0 + u * math.cos(yaw) - across * math.sin(yaw)
            wy = y0 + u * math.sin(yaw) + across * math.cos(yaw)
            rows.append({"t": float(i), "x": wx - hx, "y": wy - hy, "z": z,
                         "armed": True, "state": "X"})
        return rows

    def test_the_grader_follows_the_moved_return_lane(self):
        from check_track import traversal
        lane = self.lay["corridors"]["return"]
        through, _ = traversal(self._track_along(lane, 0.0), self.lay, 0, False)
        self.assertTrue(through["found"])
        self.assertEqual(through["n_bad"], 0)
        beside, _ = traversal(self._track_along(lane, 2.5), self.lay, 0, False)
        self.assertTrue(beside.get("n_bad", 1) > 0 or not beside["found"])

    def test_touching_an_obstacle_fails_the_grade(self):
        from check_track import grade
        lane = self.lay["corridors"]["return"]
        # Straight down the centre: obstacle 2 (v -0.5, 1.0 m across) is hit.
        g = grade(self._track_along(lane, 0.0), self.lay)
        self.assertLess(g["closest_obstacle_m"], 0.4)
        self.assertFalse(g["checks"]["no obstacle contact"])


class ValidationTests(unittest.TestCase):

    def test_a_blocking_obstacle_is_refused(self):
        s = W.default_spec()
        s["return_corridor"]["obstacles"].append(
            {"u": 5.0, "v": 0.0, "w": 0.3, "d": 3.2, "h": 3.0})
        self.assertRefused(s, "blocks the return corridor")

    def test_an_obstacle_under_the_lidar_plane_but_in_the_way_is_refused(self):
        """Live: a 3.2 m block under a 3.23 m scan plane, flown into at 3 m."""
        s = W.default_spec()
        s["return_corridor"]["obstacles"][2]["h"] = 3.2
        self.assertRefused(s, "cannot see it")

    def test_overlapping_corridors_are_refused(self):
        s = W.default_spec()
        s["return_corridor"].update({"linked": False, "x": 12.0, "y": 2.0,
                                     "yaw_deg": 180.0})
        self.assertRefused(s, "corridors overlap")

    def test_a_linked_return_corridor_follows_the_outbound_one(self):
        s = W.normalise({"corridor": {"x": 5.0, "y": 1.0, "yaw_deg": 90.0}})
        r = s["return_corridor"]
        # 10 m up the outbound lane (+y) and 4 m to its starboard (+x).
        self.assertAlmostEqual(r["x"], 9.0, places=3)
        self.assertAlmostEqual(r["y"], 11.0, places=3)
        self.assertAlmostEqual(r["yaw_deg"], -90.0, places=3)

    def assertRefused(self, spec, fragment):
        errs, _ = W.validate(spec)
        self.assertTrue(any(fragment in e for e in errs), errs)

    def test_a_pad_on_red_ground_is_refused(self):
        s = W.default_spec()
        s["pads"]["a"] = {"x": 38.0, "y": 5.0}
        self.assertRefused(s, "pad A is within 1 m of red zone 1")

    def test_home_outside_the_fence_is_refused(self):
        s = W.default_spec()
        s["geofence"] = {"mode": "polygon",
                         "vertices": [[5, -20], [60, -20], [60, 20], [5, 20]]}
        self.assertRefused(s, "take-off point (FCU home) is outside")

    def test_home_too_close_to_the_fence_is_refused(self):
        s = W.default_spec()
        s["geofence"] = {"mode": "polygon",
                         "vertices": [[-3.0, -20], [60, -20], [60, 20], [-3.0, 20]]}
        self.assertRefused(s, "within 2 m of the geofence")

    def test_a_self_crossing_fence_is_refused(self):
        s = W.default_spec()
        s["geofence"] = {"mode": "polygon",
                         "vertices": [[-10, -25], [60, 25], [60, -25], [-10, 25]]}
        self.assertRefused(s, "crosses itself")

    def test_a_corridor_on_the_take_off_pad_is_refused(self):
        s = W.default_spec()
        s["corridor"] = {"x": -3.0, "y": 3.0, "yaw_deg": 0.0}
        self.assertRefused(s, "corridor overlaps the take-off pad")

    def test_a_pad_outside_the_zone_is_refused(self):
        s = W.default_spec()
        s["pads"]["b"] = {"x": 70.0, "y": 0.0}
        self.assertRefused(s, "pad B is not entirely inside")

    def test_the_materialiser_refuses_an_invalid_spec(self):
        s = W.default_spec()
        s["pads"]["a"] = {"x": 38.0, "y": 5.0}
        with self.assertRaises(AssertionError) as cm:
            build(s)
        self.assertIn("WORLD SPEC refused", str(cm.exception))

    def test_a_partial_spec_is_filled_from_the_default(self):
        s = W.normalise({"corridor": {"yaw_deg": 10.0}})
        self.assertEqual(s["corridor"]["x"], 2.0)
        self.assertEqual(len(s["pads"]), 5)


class EditorServerTests(unittest.TestCase):
    """tools/world_editor/serve.py: save, list, load, validate."""

    @classmethod
    def setUpClass(cls):
        import importlib.util
        import threading
        from http.server import ThreadingHTTPServer
        spec = importlib.util.spec_from_file_location(
            "world_editor_serve", ROOT / "tools/world_editor/serve.py")
        cls.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)
        cls.tmp = tempfile.TemporaryDirectory()
        cls.mod.WORLDS = Path(cls.tmp.name)
        cls.mod.ROOT = Path(cls.tmp.name).parent
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), cls.mod.Handler)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.tmp.cleanup()

    def call(self, method, path, body=None):
        import urllib.request
        import urllib.error
        req = urllib.request.Request(self.base + path, method=method,
                                     data=json.dumps(body).encode() if body is not None else None,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_save_list_load(self):
        code, j = self.call("POST", "/api/worlds/unit_world", custom_spec())
        self.assertEqual(code, 200, j)
        self.assertEqual(j["errors"], [])
        code, j = self.call("GET", "/api/worlds")
        self.assertIn("unit_world", j["worlds"])
        code, j = self.call("GET", "/api/worlds/unit_world")
        self.assertEqual(j["corridor"]["yaw_deg"], 25.0)
        self.assertEqual(j["name"], "unit_world")

    def test_validate_reports_errors(self):
        s = W.default_spec()
        s["pads"]["a"] = {"x": 38.0, "y": 5.0}
        code, j = self.call("POST", "/api/validate", s)
        self.assertTrue(any("pad A" in e for e in j["errors"]))

    def test_bad_names_are_refused(self):
        code, _ = self.call("POST", "/api/worlds/..%2Fescape", W.default_spec())
        self.assertEqual(code, 400)

    def test_an_invalid_world_is_not_flown(self):
        s = W.default_spec()
        s["pads"]["a"] = {"x": 38.0, "y": 5.0}
        self.call("POST", "/api/worlds/bad_world", s)
        code, j = self.call("POST", "/api/run/bad_world")
        self.assertEqual(code, 400)
        self.assertIn("errors", j["error"])


if __name__ == "__main__":
    unittest.main()
