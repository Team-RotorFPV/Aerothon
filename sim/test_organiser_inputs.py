#!/usr/bin/env python3
"""The organiser's inputs decide where the aircraft may go.

RULEBOOK, Mission 2 Operation:

    "Delivery zone geo-fence coordinates will be provided to teams during
    Phase 2."
    "Geo-fencing: Coordinates for the geo-fence boundary will be provided.
    Teams must program these into the ground station software to ensure the
    UAS stays within the designated area."

THE FAILURE THIS IS FOR

    After the corridor the aircraft spiralled out of the area. The search
    zone was a 12 m lidar glance from the corridor mouth, grown forward and
    sideways by a 60 m budget until something matched, and the geofence was
    drawn round that same guess, never enabled. Nothing held the aircraft in.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_organiser_inputs.py -v
"""

import importlib.util
import json
import math
import os
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "aerothon_mission" / "mission_bt"))
sys.path.insert(0, str(ROOT / "scripts"))

import py_trees  # noqa: E402

from mission_bt.delivery_zone import (parse_polygon, point_in_polygon,  # noqa: E402
                                      point_inside_with_margin,
                                      polygon_to_local)
from mission_bt.geofence import (FENCE_POLYGON_INCLUSION,  # noqa: E402
                                 global_to_local, local_to_global)
from mission_bt.mission_tree import (LawnmowerSearch, UploadArenaFence,  # noqa: E402
                                     WinchDrop)

HOME = (-35.3632621, 149.1652374)
SQUARE = [(-9.5, -21.0), (58.0, -21.0), (58.0, 21.0), (-9.5, 21.0)]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Done:
    def __init__(self, ok=True):
        self._r = type("R", (), {"success": ok})()

    def done(self):
        return True

    def result(self):
        return self._r


class _Wp:
    """Stand-in for mavros_msgs/Waypoint."""
    frame = command = 0
    is_current = autocontinue = False
    param1 = param2 = param3 = param4 = 0.0
    x_lat = y_long = z_alt = 0.0


class FenceMav:
    def __init__(self, fence=SQUARE, zone=(12.0, 52.0, -15.0, 15.0),
                 home_xy=(0.0, 0.0), echo=True, refuse=None):
        self.geofence_local = fence
        self.geofence_reason = "missing" if fence is None else ""
        self._zone = zone
        self._home_xy = home_xy
        self.fence_readback = None
        self.fence_verified = False
        self.fence_reason = ""
        self.abort_reason = ""
        self.params = []
        self._echo = echo
        self._refuse = refuse
        self.pushed = None

    def home_global(self):
        return HOME

    def home_local_xy(self):
        return self._home_xy

    def delivery_search_zone(self, c):
        if self._zone is None:
            return None
        x0, x1, y0, y1 = self._zone
        return (x0 + c, x1 - c, y0 + c, y1 - c)

    def push_fence(self, items):
        self.pushed = list(items)
        if self._echo:
            self.fence_readback = list(items)
        return _Done()

    def set_param(self, name, value):
        self.params.append((name, value))
        return _Done(ok=(name != self._refuse))

    def log(self, msg, warn=False):
        pass


def _run(leaf, n=20):
    for _ in range(n):
        leaf.tick_once()
        if leaf.status != py_trees.common.Status.RUNNING:
            break
    return leaf.status


# Waypoint class is imported lazily inside the stage; give it one here.
sys.modules.setdefault("mavros_msgs", type(sys)("mavros_msgs"))
if not hasattr(sys.modules["mavros_msgs"], "msg"):
    _m = type(sys)("mavros_msgs.msg")
    _m.Waypoint = _Wp
    sys.modules["mavros_msgs"].msg = _m
    sys.modules["mavros_msgs.msg"] = _m


class UploadArenaFenceTests(unittest.TestCase):

    def test_supplied_polygon_is_uploaded_verified_and_ENFORCED(self):
        mav = FenceMav()
        st = _run(UploadArenaFence(mav))
        self.assertEqual(st, py_trees.common.Status.SUCCESS, mav.abort_reason)
        self.assertTrue(mav.fence_verified)
        names = [n for n, _ in mav.params]
        self.assertEqual(names[-1], "FENCE_ENABLE",
                         "enforcement must be the LAST thing set")
        self.assertIn(("FENCE_TYPE", 5), mav.params)
        self.assertIn(("FENCE_ACTION", 1), mav.params)
        # The upload is the organiser's polygon, inclusion, vertex for vertex.
        self.assertEqual(len(mav.pushed), 4)
        self.assertTrue(all(w.command == FENCE_POLYGON_INCLUSION
                            for w in mav.pushed))
        for w, (x, y) in zip(mav.pushed, SQUARE):
            gx, gy = global_to_local(w.x_lat, w.y_long, *HOME)
            self.assertAlmostEqual(gx, x, places=2)
            self.assertAlmostEqual(gy, y, places=2)

    def test_no_geofence_means_no_arming(self):
        mav = FenceMav(fence=None)
        st = _run(UploadArenaFence(mav, timeout_ticks=5))
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertFalse(mav.fence_verified)
        self.assertIn("UploadArenaFence", mav.abort_reason)

    def test_a_refused_parameter_is_retried_until_accepted(self):
        """MAVROS refuses sets until its parameter download has finished."""
        mav = FenceMav()
        refusals = {"n": 2}
        base = mav.set_param

        def flaky(name, value):
            if name == "FENCE_TYPE" and refusals["n"] > 0:
                refusals["n"] -= 1
                mav.params.append((name, value))
                return _Done(ok=False)
            return base(name, value)

        mav.set_param = flaky
        st = _run(UploadArenaFence(mav), n=40)
        self.assertEqual(st, py_trees.common.Status.SUCCESS, mav.abort_reason)
        self.assertEqual([n for n, _ in mav.params].count("FENCE_TYPE"), 3)

    def test_home_outside_the_fence_is_refused(self):
        mav = FenceMav(home_xy=(-30.0, 0.0))
        st = _run(UploadArenaFence(mav))
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertIn("home", mav.abort_reason)
        self.assertIsNone(mav.pushed, "uploaded a fence that excludes home")

    def test_delivery_zone_outside_the_fence_is_refused(self):
        mav = FenceMav(zone=(12.0, 80.0, -15.0, 15.0))
        st = _run(UploadArenaFence(mav))
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertIn("delivery-zone corner", mav.abort_reason)

    def test_a_fence_that_never_reads_back_is_not_enforced(self):
        mav = FenceMav(echo=False)
        st = _run(UploadArenaFence(mav, timeout_ticks=6), n=30)
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertNotIn("FENCE_ENABLE", [n for n, _ in mav.params])

    def test_a_refused_parameter_fails_closed(self):
        mav = FenceMav(refuse="FENCE_ENABLE")
        st = _run(UploadArenaFence(mav, timeout_ticks=8), n=30)
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertFalse(mav.fence_verified)


class PolygonTests(unittest.TestCase):

    def test_parse_and_localise_round_trip(self):
        verts = [dict(zip(("lat", "lon"), local_to_global(x, y, *HOME)))
                 for x, y in SQUARE]
        parsed, why = parse_polygon(json.dumps({"vertices": verts}))
        self.assertEqual(why, "")
        pts, why = polygon_to_local(parsed, *HOME)
        for (ax, ay), (bx, by) in zip(pts, SQUARE):
            self.assertAlmostEqual(ax, bx, places=2)
            self.assertAlmostEqual(ay, by, places=2)

    def test_parse_rejects_bad_input(self):
        for payload in ("", "nope", json.dumps({"vertices": [{"lat": 1, "lon": 2}]}),
                        json.dumps({"vertices": [{"lat": 99, "lon": 2}] * 3})):
            parsed, why = parse_polygon(payload)
            self.assertIsNone(parsed)
            self.assertTrue(why)

    def test_containment_and_margin(self):
        self.assertTrue(point_in_polygon(0.0, 0.0, SQUARE))
        self.assertFalse(point_in_polygon(60.0, 0.0, SQUARE))
        self.assertTrue(point_inside_with_margin(0.0, 0.0, SQUARE, 5.0))
        self.assertFalse(point_inside_with_margin(-8.0, 0.0, SQUARE, 5.0))
        # Non-rectangular (triangle) fence.
        tri = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
        self.assertTrue(point_in_polygon(2.0, 2.0, tri))
        self.assertFalse(point_in_polygon(8.0, 8.0, tri))


class SearchMav:
    """Just enough for LawnmowerSearch to fly its plan."""

    def __init__(self, pos=(14.0, 2.0, 10.0)):
        self._pos = pos
        self.gotos = []
        self.logs = []
        self.qr_matched = False
        self.qr_decoded = ""
        self.exclusions = []
        self.abort_reason = ""
        self.corridor_exit_pose = (14.0, 2.0, 3.0, 0.0)

    def pos(self):
        return self._pos

    def yaw(self):
        return 0.0

    def alt(self):
        return self._pos[2]

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z))
        self._pos = (x, y, z)

    def reached(self, x, y, z, tol=0.6):
        return math.dist(self._pos, (x, y, z)) < tol

    def log(self, msg, warn=False):
        self.logs.append(msg)


class BottomLeftLawnmowerTests(unittest.TestCase):
    ZONE = (13.0, 51.0, -14.0, 14.0)

    def _leaf(self, mav, zone=None):
        z = zone or self.ZONE
        return LawnmowerSearch(mav, lambda: z, 10.0, marker_m=3.0,
                               exclusions=lambda: mav.exclusions,
                               search_budget_m=0.0)

    def test_first_waypoint_is_the_bottom_left_corner(self):
        # Wherever the corridor lets the aircraft out -- here the far NE.
        for start in ((50.0, 13.0, 10.0), (14.0, 2.0, 10.0), (30.0, -13.0, 10.0)):
            mav = SearchMav(pos=start)
            leaf = self._leaf(mav)
            leaf.initialise()
            x0, x1, y0, y1 = self.ZONE
            wx, wy = leaf.wps[0]
            self.assertAlmostEqual(wx, x0, delta=0.01,
                                   msg=f"start {start}: first wp {leaf.wps[0]}")
            self.assertLess(wy - y0, leaf.plan["lane_spacing_m"],
                            f"start {start}: first wp {leaf.wps[0]} not at the south edge")

    def test_lanes_advance_from_south_to_north(self):
        mav = SearchMav()
        leaf = self._leaf(mav)
        leaf.initialise()
        lane_ys = [leaf.wps[i][1] for i in range(0, len(leaf.wps), 2)]
        self.assertEqual(lane_ys, sorted(lane_ys))

    def test_the_search_never_leaves_the_boundary(self):
        mav = SearchMav()
        leaf = self._leaf(mav)
        for _ in range(4000):
            leaf.tick_once()
            if leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(leaf.status, py_trees.common.Status.FAILURE)
        self.assertEqual(leaf.expansions, 0)
        x0, x1, y0, y1 = self.ZONE
        for x, y, _ in mav.gotos:
            self.assertTrue(x0 - 0.01 <= x <= x1 + 0.01 and y0 - 0.01 <= y <= y1 + 0.01,
                            f"commanded ({x:.1f}, {y:.1f}) outside the boundary")
        self.assertTrue(any("search pass 2" in m for m in mav.logs),
                        "no cross-grid second pass after a miss")


class RedZoneLookaheadTests(unittest.TestCase):
    """The sweep must see red ground before it arrives on it."""
    ZONE = (13.0, 51.0, -14.0, 14.0)

    def _leaf(self, mav, **kw):
        kw.setdefault("crab", True)
        kw.setdefault("image_height_px", 720)
        kw.setdefault("search_speed_mps", 2.5)
        return LawnmowerSearch(mav, lambda: self.ZONE, 10.0, marker_m=3.0,
                               exclusions=lambda: mav.exclusions,
                               search_budget_m=0.0, **kw)

    def test_x_lanes_are_flown_crabbed_north(self):
        mav = SearchMav()
        mav.yaws = []
        orig = mav.goto
        leaf = self._leaf(mav)
        mav.goto = lambda x, y, z, yaw=0.0: (
            mav.yaws.append((leaf.plan["lane_axis"], yaw)), orig(x, y, z, yaw))
        leaf.initialise()
        for _ in range(400):
            leaf.tick_once()
        # Every commanded heading is perpendicular to the leg being flown:
        # check the lane legs, which are the long ones.
        lane = [(a, y) for a, y in mav.yaws]
        self.assertTrue(lane)
        x_yaws = {round(abs(math.sin(y)), 3) for a, y in lane if a == "x"}
        # x-lanes: nose north or south (|sin| = 1), long axis along the lane.
        self.assertIn(1.0, x_yaws)

    def test_the_crab_heading_is_perpendicular_to_travel(self):
        mav = SearchMav(pos=(20.0, 0.0, 10.0))
        mav._yaw_now = 0.0
        mav.yaw = lambda: mav._yaw_now
        leaf = self._leaf(mav)
        leaf.initialise()
        for target in ((40.0, 0.0), (20.0, -15.0), (5.0, 0.0)):
            yaw = leaf._yaw(target)
            d = math.atan2(target[1] - 0.0, target[0] - 20.0)
            self.assertAlmostEqual(abs(math.cos(yaw - d)), 0.0, places=6)

    def test_crabbed_lanes_are_spaced_for_the_narrow_swath(self):
        wide = self._leaf(SearchMav(), crab=False)
        wide.initialise()
        crab = self._leaf(SearchMav())
        crab.initialise()
        self.assertLess(crab.plan["lane_spacing_m"],
                        0.6 * wide.plan["lane_spacing_m"])
        self.assertGreaterEqual(crab.plan["coverage"], 0.99)

    def test_the_search_speed_is_capped(self):
        mav = SearchMav()
        mav.speeds = []
        mav.set_speed = lambda v: (mav.speeds.append(v), object())[1]
        leaf = self._leaf(mav)
        leaf.initialise()
        for _ in range(5):
            leaf.tick_once()
        self.assertEqual(mav.speeds, [2.5])

    def test_a_replan_resumes_at_the_current_lane_not_waypoint_zero(self):
        mav = SearchMav()
        leaf = self._leaf(mav)
        leaf.initialise()
        # Fly until the third lane is under way.
        lanes = sorted({leaf._lane_coord(w) for w in leaf.wps})
        for _ in range(4000):
            leaf.tick_once()
            if leaf.i < len(leaf.wps) and leaf._lane_coord(leaf.wps[leaf.i]) >= lanes[2]:
                break
        current = leaf._lane_coord(leaf.wps[leaf.i])
        # A red zone is confirmed on the far side of the field.
        mav.exclusions = [(40.0, 44.0, 10.0, 13.0)]
        leaf.tick_once()
        self.assertGreater(leaf.i, 0, "re-plan restarted the whole pattern")
        self.assertGreaterEqual(leaf._lane_coord(leaf.wps[leaf.i]),
                                current - 0.5 * leaf.plan["lane_spacing_m"])


class TargetFixTests(unittest.TestCase):
    """Seed 1002: matched on one frame mid-lane, then CenterOnTarget held a
    spot the pad had already slid out of view from, and timed out."""

    def test_centring_flies_back_to_the_pad_fixed_during_the_sweep(self):
        from geometry_msgs.msg import Vector3
        from mission_bt.mission_tree import CenterOnQR, note_target
        mav = SearchMav(pos=(20.0, 0.0, 10.0))
        mav.abort_reason = ""
        # Matched pad seen dead ahead-right: yaw 0 (east), offset right/up.
        mav.qr_offset = Vector3(x=0.2, y=-0.3, z=1.0)
        note_target(mav)
        fix = mav.target_xy
        self.assertIsNotNone(fix)
        self.assertGreater(fix[0], 20.0)       # ahead (east)
        self.assertLess(fix[1], 0.0)           # right of an east heading
        # The aircraft moves on; the pad leaves the frame.
        mav._pos = (26.0, 0.0, 10.0)
        mav.qr_offset = Vector3(x=0.0, y=0.0, z=0.0)
        stage = CenterOnQR("CenterOnTarget", mav, require_match=True,
                           settle_ticks=3, timeout_ticks=50)
        stage.initialise()
        stage.update()
        gx, gy, _ = mav.gotos[-1]
        self.assertAlmostEqual(gx, fix[0], places=6)
        self.assertAlmostEqual(gy, fix[1], places=6)


class DropServoTests(unittest.TestCase):

    def test_drop_point_walks_onto_the_matched_pad(self):
        from geometry_msgs.msg import Vector3

        class M(SearchMav):
            winch_status = {}

            def winch(self, cmd):
                pass

        mav = M(pos=(20.0, 3.0, 10.0))
        # Matched pad to the RIGHT of frame at yaw 0 (east): i.e. to the south.
        mav.qr_offset = Vector3(x=0.3, y=0.0, z=1.0)
        leaf = WinchDrop(mav, drop_alt=5.0, cruise_alt=10.0, marker_m=3.0)
        leaf.initialise()
        leaf.update()
        self.assertLess(leaf.drop_y, 3.0, "moved away from the pad")
        self.assertAlmostEqual(leaf.drop_x, 20.0, delta=0.05)
        self.assertEqual(leaf.phase, 0)


class ArenaLayoutTests(unittest.TestCase):
    """Every randomised arena is one the rulebook describes."""

    @classmethod
    def setUpClass(cls):
        cls.mw = _load("materialize_world", ROOT / "scripts" / "materialize_world.py")
        cls.world = (ROOT / "src" / "aerothon_sim" / "sim_gazebo" / "worlds"
                     / "mission2.sdf").read_text(encoding="utf-8")

    def test_zone_is_beyond_the_corridor_and_contains_its_mouth(self):
        mw = self.mw
        for seed in range(1001, 1041):
            lay = mw.randomise_arena(self.world, random.Random(seed))
            zx, zy, w, h = lay["delivery_zone_rect"]
            gx, gy, gyaw = lay["gate"]
            foot = mw.corridor_footprint(gx, gy, gyaw)
            self.assertGreaterEqual(zx - w / 2, max(p[0] for p in foot) - 1e-6,
                                    f"seed {seed}: zone overlaps the corridor")
            for ex, ey in foot[1:3]:
                self.assertTrue(zy - h / 2 + 2.9 <= ey <= zy + h / 2 - 2.9,
                                f"seed {seed}: corridor mouth not inside zone")

    def test_pads_are_deliverable_and_inside_the_zone(self):
        mw = self.mw
        for seed in range(1001, 1041):
            lay = mw.randomise_arena(self.world, random.Random(seed))
            zx, zy, w, h = lay["delivery_zone_rect"]
            reds = []
            for name, (cx, cy) in lay["red_zones"].items():
                rw, rh = mw.RED_ZONE_SIZES[name]
                reds.append((cx - rw / 2, cx + rw / 2, cy - rh / 2, cy + rh / 2))
            pads = list(lay["pads"].values())
            for px, py in pads:
                self.assertTrue(abs(px - zx) <= w / 2 - 3 and abs(py - zy) <= h / 2 - 3,
                                f"seed {seed}: pad ({px}, {py}) at the zone edge")
                for r in reds:
                    self.assertGreaterEqual(mw._rect_point_gap(r, (px, py)), 3.4,
                                            f"seed {seed}: pad under red ground")
            for i in range(len(pads)):
                for j in range(i + 1, len(pads)):
                    self.assertGreaterEqual(math.dist(pads[i], pads[j]), 6.9)

    def test_geofence_contains_takeoff_corridor_and_zone(self):
        mw = self.mw
        for seed in range(1001, 1041):
            lay = mw.randomise_arena(self.world, random.Random(seed))
            fx0, fx1, fy0, fy1 = lay["geofence_rect"]
            fence = [(fx0, fy0), (fx1, fy0), (fx1, fy1), (fx0, fy1)]
            zx, zy, w, h = lay["delivery_zone_rect"]
            gx, gy, gyaw = lay["gate"]
            pts = (list(mw.TAKEOFF_CORNERS) + mw.corridor_footprint(gx, gy, gyaw)
                   + [(zx - w / 2, zy - h / 2), (zx + w / 2, zy + h / 2)])
            for p in pts:
                self.assertTrue(point_inside_with_margin(p[0], p[1], fence, 5.9),
                                f"seed {seed}: {p} not well inside the fence")

    def test_shipped_default_matches_the_publisher(self):
        pub = _load("publish_delivery_zone", ROOT / "scripts" / "publish_delivery_zone.py")
        home = self.mw.spawn_xy(self.world)
        self.assertEqual(home, (-2.0, 2.0))
        lay = self.mw.to_home_frame(self.mw.shipped_layout(), home)
        self.assertEqual(tuple(lay["geofence_rect"]), pub.DEFAULT_GEOFENCE)
        self.assertEqual(tuple(lay["delivery_zone_rect"]), pub.DEFAULT_ZONE)

    def test_the_published_rectangles_are_HOME_local(self):
        """World coordinates published as local put the field 2 m off."""
        lay = self.mw.to_home_frame(
            {"delivery_zone_rect": [32.0, 0.0, 40.0, 30.0],
             "geofence_rect": [-9.5, 58.0, -21.0, 21.0]}, (-2.0, 2.0))
        self.assertEqual(lay["delivery_zone_rect"], [34.0, -2.0, 40.0, 30.0])
        self.assertEqual(lay["geofence_rect"], [-7.5, 60.0, -23.0, 19.0])
        self.assertEqual(lay["home_world"], [-2.0, 2.0])


if __name__ == "__main__":
    unittest.main()
