#!/usr/bin/env python3
"""A banner the start position sees only edge-on is still found.

THE CASE

    A custom arena put the take-off pad 10.5 m north of the corridor mouth,
    with the banner facing west down the lane. From the pad the board was a
    green sliver with no lettering; AlignToBanner swept a full turn, relocated
    along its entry heading (parallel to the board), and never saw its face.

    These tests put a board where only its edge is visible from the start and
    require the stage to go round it and identify it -- for the outbound gate
    (AlignToBanner) and the return gate (FindReturnBanner) -- plus the
    geometry the orbit is built from.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_banner_orbit.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import py_trees                                              # noqa: E402
from mission_bt.banner_orbit import (green_fix, leg_clear,   # noqa: E402
                                     orbit_plan)
from mission_bt.mission_tree import (AlignToBanner,          # noqa: E402
                                     FindReturnBanner)
from test_banner_sweep import Clock, SweepMav                # noqa: E402

HFOV = 1.0472
W, H = 1280, 720
FOCAL = 0.5 * W / math.tan(HFOV / 2)
BOARD_W, BOARD_H = 3.7, 1.15


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BoardMav(SweepMav):
    """A board at `board` whose lettering faces `normal`.

    Readable only from within `read_cone` of its face and `read_range` of it,
    and only when it is inside the camera's field of view; from anywhere else
    in view it is green the detector cannot identify -- reported the way
    perception_banner reports it, as a bounding box whose height encodes the
    range. The vehicle goes where it is told at once: these tests are about
    WHERE the search goes, not how it gets there.
    """

    def __init__(self, start, board, normal, read_cone=math.radians(50.0),
                 read_range=9.0, two_sided=False, zone=None):
        super().__init__()
        self._pos = start
        self._alt = start[2]
        self._yaw = 0.0
        self.board = board
        self.normal = normal
        self.read_cone = read_cone
        self.read_range = read_range
        self.banner_green = None
        self.camera_state = {}
        self.geofence_local = None
        self.exclusions = []
        self.two_sided = two_sided
        self.zone = zone                  # (x0, x1, y0, y1) delivery zone

    def delivery_search_zone(self, clearance_m):
        if self.zone is None:
            return None
        x0, x1, y0, y1 = self.zone
        c = clearance_m
        return (x0 + c, x1 - c, y0 + c, y1 - c)

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z, yaw))
        self.commanded_yaws.append(yaw)
        self._pos = (x, y, z)
        self._alt = z
        self._yaw = wrap(yaw)

    def reached(self, x, y, z, tol=0.6):
        return math.dist(self._pos, (x, y, z)) < tol

    def _view(self):
        px, py = self._pos[:2]
        bx, by = self.board
        rng = math.hypot(bx - px, by - py)
        off = wrap(math.atan2(by - py, bx - px) - self._yaw)
        if abs(off) > HFOV / 2 - 0.02:
            return None
        bearing = -math.tan(off) / math.tan(HFOV / 2)     # inverse of bearing_to_angle
        facing = math.atan2(py - by, px - bx)
        off_face = abs(wrap(facing - self.normal))
        if self.two_sided:
            off_face = min(off_face, math.pi - off_face)
        readable = rng <= self.read_range and off_face <= self.read_cone
        return bearing, rng, readable

    def _refresh(self):
        v = self._view()
        if v is None:
            self.banner_green = None
            return None
        bearing, rng, readable = v
        h = FOCAL * BOARD_H / rng
        self.banner_green = {"px": [int(W / 2 + bearing * W / 2 - 5), 300, 10, h],
                             "area": 10.0 * h, "bearing": bearing, "wh": [W, H]}
        if readable:
            self.banner_board_area = BOARD_W * BOARD_H * (FOCAL / rng) ** 2
        return v

    def banner_identified(self):
        v = self._refresh()
        return bool(v and v[2])

    def banner_bearing(self):
        v = self._refresh()
        return v[0] if v and v[2] else 0.0


def run(stage, clock, until, ticks=20000, dt=0.1):
    for _ in range(ticks):
        status = stage.update()
        if until() or status is not py_trees.common.Status.RUNNING:
            return status
        clock.advance(dt)
    return py_trees.common.Status.RUNNING


class OrbitGeometryTests(unittest.TestCase):

    def test_vantages_sit_on_the_circle_and_face_the_centre(self):
        plan = orbit_plan((0.0, 0.0), (0.0, 10.0), 6.0, lambda x, y: True, n=7)
        self.assertEqual(len(plan), 7)
        for v in plan:
            x, y = v["at"]
            self.assertAlmostEqual(math.hypot(x, y), 6.0, places=6)
            self.assertAlmostEqual(wrap(math.atan2(-y, -x) - v["face"]), 0.0, places=6)

    def test_no_leg_cuts_across_the_structure(self):
        """Arc waypoints, not chords: every waypoint stays on the circle and
        consecutive ones are at most 30 degrees apart."""
        plan = orbit_plan((0.0, 0.0), (0.0, 10.0), 6.0, lambda x, y: True, n=7)
        prev = None
        for v in plan:
            for p in v["path"]:
                self.assertAlmostEqual(math.hypot(*p), 6.0, places=6)
                if prev is not None:
                    gap = abs(wrap(math.atan2(p[1], p[0]) - math.atan2(prev[1], prev[0])))
                    self.assertLessEqual(gap, math.radians(30.0) + 1e-9)
                prev = p

    def test_a_fence_turns_the_orbit_round_the_other_way(self):
        """Nothing west of x = -2 may be flown: the orbit goes east first
        and never places a waypoint over the line."""
        ok = lambda x, y: x > -2.0                                  # noqa: E731
        plan = orbit_plan((0.0, 0.0), (0.0, 10.0), 6.0, ok, n=7)
        self.assertTrue(plan)
        for v in plan:
            for p in v["path"] + [v["at"]]:
                self.assertGreater(p[0], -2.0)
        self.assertGreater(plan[0]["at"][0], 0.0)

    def test_nothing_flyable_is_no_plan(self):
        self.assertEqual(orbit_plan((0, 0), (0, 10), 6.0, lambda x, y: False), [])


class GreenFixTests(unittest.TestCase):

    def mav(self):
        return BoardMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=math.pi)

    def test_height_gives_range_when_the_ground_contact_is_unknown(self):
        m = self.mav()
        m._yaw = -math.pi / 2                    # facing the board
        m._refresh()
        f = green_fix(m, HFOV)
        self.assertAlmostEqual(f["range"], 10.0, delta=0.3)
        self.assertAlmostEqual(f["x"], 0.0, delta=0.3)
        self.assertAlmostEqual(f["y"], 0.0, delta=0.3)

    def test_ground_contact_is_preferred_when_the_camera_pitch_is_known(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        # Pitched 20 deg down, 5 m up: a region whose bottom row sits on the
        # optical axis meets the ground 5 / tan(20 deg) = 13.7 m away.
        m.camera_state = {"actual_rad": -math.radians(20.0)}
        m.banner_green["px"] = [635, 300, 10, H / 2 - 300]
        f = green_fix(m, HFOV)
        self.assertAlmostEqual(f["range"], 5.0 / math.tan(math.radians(20.0)), delta=0.05)

    def test_the_outbound_gate_is_not_a_lead_on_the_way_back(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        f = green_fix(m, HFOV, exclude=[((0.0, 0.0), (10.0, 0.0))])
        self.assertIsNone(f)

    def test_no_green_no_fix(self):
        m = self.mav()
        m._yaw = math.pi / 2                     # facing away
        m._refresh()
        self.assertIsNone(green_fix(m, HFOV))


class LegClearTests(unittest.TestCase):

    def test_the_lidar_vetoes_a_leg_into_something_close(self):
        class Scan:
            angle_min, angle_increment = -math.pi, 2 * math.pi / 360
            range_min, range_max = 0.1, 12.0
            ranges = [12.0] * 360
        s = Scan()
        s.ranges = list(s.ranges)
        s.ranges[180] = 1.2                      # dead ahead
        m = BoardMav((0.0, 0.0, 5.0), board=(50, 50), normal=0.0)
        m._scan = s
        self.assertFalse(leg_clear(m, 5.0, 0.0))
        self.assertTrue(leg_clear(m, -5.0, 0.0))


class EdgeOnOutboundTests(unittest.TestCase):
    """AlignToBanner from a start that sees the board edge-on."""

    def test_it_goes_round_and_identifies_the_board(self):
        clock = Clock()
        # Board at the origin facing WEST; aircraft 10 m NORTH of it.
        mav = BoardMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        stage = AlignToBanner(mav, clock=clock, dwell_s=1.0)
        stage.initialise()
        run(stage, clock, until=lambda: stage.phase in (stage.CENTRE, stage.SQUARE))
        self.assertIn(stage.phase, (stage.CENTRE, stage.SQUARE),
                      f"never identified; last feedback: {stage.feedback_message}")
        logs = " ".join(str(line) for line in mav.logs)
        self.assertIn("Orbiting", logs)
        x, y = mav.pos()[:2]
        # It identified the board from its WEST side, where the face is.
        self.assertLess(x, -2.0, f"identified from ({x:.1f}, {y:.1f})")
        # ...and got there below the wall tops: never above a corridor.
        self.assertAlmostEqual(mav.pos()[2], stage.alt_floor_m, places=6)

    def test_a_face_on_start_does_not_orbit(self):
        clock = Clock()
        mav = BoardMav((-8.0, 0.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        stage = AlignToBanner(mav, clock=clock, dwell_s=1.0)
        stage.initialise()
        run(stage, clock, until=lambda: stage.phase in (stage.CENTRE, stage.SQUARE))
        self.assertIn(stage.phase, (stage.CENTRE, stage.SQUARE))
        self.assertNotIn("Orbiting", " ".join(str(line) for line in mav.logs))


class EdgeOnReturnTests(unittest.TestCase):
    """FindReturnBanner from a stand-off that sees the return board edge-on."""

    def test_the_back_of_the_board_is_not_a_stand_off(self):
        """Watched live: the orbit read the return board from INSIDE the
        return lane, stood off there and squared up facing the wrong way. A
        board readable from both sides, the delivery zone to its east: the
        stand-off must be on the zone side, and the orbit at corridor
        altitude."""
        clock = Clock()
        mav = BoardMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=0.0,
                       two_sided=True, zone=(2.0, 40.0, -20.0, 20.0))
        stage = FindReturnBanner(mav, alt=5.0, clock=clock, dwell_s=1.0,
                                 orbit_alt_m=3.0)
        stage.initialise()
        status = run(stage, clock, until=lambda: False)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      f"not found; {stage.feedback_message}")
        x, y, z = mav.pos()
        self.assertGreater(x, -1.0, f"stood off behind the board at ({x:.1f}, {y:.1f})")
        zs = [round(g[2], 2) for g in mav.gotos]
        self.assertIn(3.0, zs, "orbit not flown at corridor altitude")
        self.assertEqual(set(zs[zs.index(3.0):]), {3.0},
                         "climbed back above the wall tops mid-orbit")

    def test_it_goes_round_and_stands_off_in_front_of_the_board(self):
        clock = Clock()
        mav = BoardMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        stage = FindReturnBanner(mav, alt=5.0, clock=clock, dwell_s=1.0)
        stage.initialise()
        status = run(stage, clock, until=lambda: False)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      f"not found; {stage.feedback_message}")
        logs = " ".join(str(line) for line in mav.logs)
        self.assertIn("Orbiting", logs)
        x, y = mav.pos()[:2]
        self.assertLess(x, -2.0, f"stood off at ({x:.1f}, {y:.1f})")


if __name__ == "__main__":
    unittest.main()
