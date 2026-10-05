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
from mission_bt.banner_orbit import (better_green, green_fix,  # noqa: E402
                                     lidar_refutes,
                                     leg_clear, orbit_plan)
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


def scan_with(returns, n=360):
    """A 360-sample scan, angle 0 at index n/2, with `returns` {index: m}."""
    class Scan:
        angle_min, angle_increment = -math.pi, 2 * math.pi / n
        range_min, range_max = 0.1, 12.0
    s = Scan()
    s.ranges = [float("inf")] * n
    for i, r in returns.items():
        s.ranges[i] = r
    return s


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

    def test_a_raised_board_is_ranged_by_its_height_not_the_ground_beyond_it(self):
        """my_world: the edge-on board 11 m off, 5 m up. The ray under its
        2.8 m bottom edge meets the ground ~25 m off, and the board was
        dropped as farther than any gate could be."""
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        pitch = math.radians(20.0)
        m.camera_state = {"actual_rad": -pitch}
        rng, alt, z0 = 11.0, 5.0, 2.805

        def row(z):
            return H / 2 + FOCAL * math.tan(math.atan((alt - z) / rng) - pitch)
        top, bottom = row(z0 + BOARD_H), row(z0)
        m.banner_green["px"] = [635, top, 10, bottom - top]
        f = green_fix(m, HFOV)
        self.assertEqual(f["source"], "height")
        self.assertAlmostEqual(f["range"], rng, delta=0.6)

    def test_green_wider_than_the_view_is_not_a_lead(self):
        """The grassed delivery zone, 13 m off, across the whole frame."""
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        m.banner_green["px"] = [0, 380, W, 150]
        self.assertIsNone(green_fix(m, HFOV))

    def test_green_off_the_bottom_and_a_side_is_ground_round_the_aircraft(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        m.banner_green["px"] = [0, 480, 1180, H - 480]      # rb_rotated, return
        self.assertIsNone(green_fix(m, HFOV))
        m.banner_green["px"] = [600, 200, 80, H - 200]      # corridor floor: kept
        self.assertTrue(green_fix(m, HFOV)["cut"])

    def test_a_whole_board_beats_a_bigger_region_the_frame_cuts(self):
        """my_world: the board a sliver beside a chunk of grass running off
        the side of the frame. The grass is bigger; the board is the lead."""
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        board = m.banner_green["px"]
        m.banner_green["regions"] = [
            {"px": [1100, 380, W - 1100, 150], "area": 180.0 * 150},
            {"px": board, "area": m.banner_green["area"]}]
        f = green_fix(m, HFOV)
        self.assertFalse(f["edge"])
        self.assertAlmostEqual(f["x"], 0.0, delta=0.3)
        self.assertAlmostEqual(f["y"], 0.0, delta=0.3)

    def test_green_the_lidar_should_see_and_does_not_is_ground(self):
        """my_world from 5 m up: the grass read as a board 4.2 m off. From
        gate height the lidar would see posts there; it sees nothing."""
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        m.banner_green["px"] = [600, 300, 80, FOCAL * BOARD_H / 4.0]
        m._scan = scan_with({})
        self.assertIsNotNone(green_fix(m, HFOV))
        self.assertIsNone(green_fix(m, HFOV, refute=True))
        m._scan = scan_with({180: 4.4})                    # posts, dead ahead
        self.assertEqual(green_fix(m, HFOV, refute=True)["source"], "lidar")

    def test_green_beyond_the_lidar_is_not_refuted_by_it(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()                                     # the board, 10 m off
        m._scan = scan_with({})
        f = green_fix(m, HFOV, refute=True)
        self.assertIsNotNone(f)
        self.assertFalse(lidar_refutes(m, f))

    def test_ground_contact_is_used_when_it_is_the_nearer_range(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        # Pitched 20 deg down, 5 m up: a region whose bottom row sits on the
        # optical axis meets the ground 5 / tan(20 deg) = 13.7 m away.
        m.camera_state = {"actual_rad": -math.radians(20.0)}
        m.banner_green["px"] = [635, 300, 10, H / 2 - 300]
        f = green_fix(m, HFOV)
        self.assertAlmostEqual(f["range"], 5.0 / math.tan(math.radians(20.0)), delta=0.05)

    def test_green_running_off_the_frame_is_a_bearing_not_a_range(self):
        """my_world: the corridor's green floor, from the frame bottom up to
        the board, read as a 3.2 m fix for a board 11 m off."""
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        m.banner_green["px"] = [600, 200, 80, H - 200]      # to the bottom row
        f = green_fix(m, HFOV)
        self.assertTrue(f["cut"])
        self.assertEqual(f["source"], "height")

    def test_the_lidar_ranges_the_green_when_it_can(self):
        m = self.mav()
        m._yaw = -math.pi / 2
        m._refresh()
        m.banner_green["px"] = [600, 200, 80, H - 200]
        m._scan = scan_with({180: 11.0, 185: 11.4})         # dead ahead
        f = green_fix(m, HFOV)
        self.assertEqual(f["source"], "lidar")
        self.assertFalse(f["cut"])
        self.assertAlmostEqual(f["range"], 11.0, places=6)
        self.assertAlmostEqual(f["y"], -1.0, delta=0.05)

    def test_a_ranged_fix_beats_a_bigger_bearing(self):
        cut = {"area": 9000.0, "cut": True}
        ranged = {"area": 400.0, "cut": False}
        self.assertTrue(better_green(ranged, cut))
        self.assertFalse(better_green(cut, ranged))
        self.assertTrue(better_green(cut, None))

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
        s.ranges[0] = 0.23                       # the aircraft's own GPS mast
        s.ranges[295] = 0.35                     # its rear arm, +151 deg
        s.ranges[250] = 0.50                     # its gear, banked into a leg
        m = BoardMav((0.0, 0.0, 5.0), board=(50, 50), normal=0.0)
        m._scan = s
        self.assertFalse(leg_clear(m, 5.0, 0.0))
        self.assertTrue(leg_clear(m, -5.0, 0.0), "blocked by its own mast")


class FloorMav(BoardMav):
    """my_world's outbound lane: a green corridor FLOOR runs from under the
    aircraft's view up to the board. From above the walls the largest green
    is that floor, cut off by the bottom of the frame and ~500 px tall; the
    lidar slice passes over everything. At gate height the lidar sees the
    board on its bearing."""

    WALL_TOP = 3.6

    def _refresh(self):
        v = super()._refresh()
        if v is not None and self._alt > self.WALL_TOP:
            self.banner_green["px"] = [600, 220, 80, H - 220]
            self.banner_green["area"] = 80.0 * (H - 220)
        return v

    @property
    def _scan(self):
        if self._alt > self.WALL_TOP:
            return scan_with({})
        px, py = self._pos[:2]
        bx, by = self.board
        rel = wrap(math.atan2(by - py, bx - px) - self._yaw)
        i = int(round((rel + math.pi) / (2 * math.pi / 360))) % 360
        return scan_with({i: math.hypot(bx - px, by - py)})

    @_scan.setter
    def _scan(self, _):
        pass


class EdgeOnOutboundTests(unittest.TestCase):
    """AlignToBanner from a start that sees the board edge-on."""

    def test_a_green_floor_is_ranged_on_the_lidar_before_it_is_orbited(self):
        clock = Clock()
        mav = FloorMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        stage = AlignToBanner(mav, clock=clock, dwell_s=1.0)
        stage.initialise()
        run(stage, clock, until=lambda: stage.phase in (stage.CENTRE, stage.SQUARE))
        logs = " ".join(str(line) for line in mav.logs)
        self.assertIn("range it on the lidar", logs)
        self.assertIn("(lidar)", logs)
        self.assertIn(stage.phase, (stage.CENTRE, stage.SQUARE),
                      f"never identified; last feedback: {stage.feedback_message}")
        x, y = mav.pos()[:2]
        self.assertLess(x, -2.0, f"identified from ({x:.1f}, {y:.1f})")

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

    def test_orbit_legs_fly_slow_enough_to_stop_inside_the_lidar_look(self):
        """At WPNAV_SPEED a checked leg reached 3.8 m/s, needed 2.9 m to stop
        and flew into the board; the cap comes off when the stage ends."""
        clock = Clock()
        mav = BoardMav((0.0, 10.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        speeds = []
        mav.set_speed = lambda v: speeds.append(v) or True
        stage = AlignToBanner(mav, clock=clock, dwell_s=1.0)
        stage.initialise()
        run(stage, clock, until=lambda: stage.phase in (stage.CENTRE, stage.SQUARE))
        self.assertEqual(speeds[:1], [stage.guard_speed_mps])
        self.assertLessEqual(stage.guard_speed_mps ** 2 / (2 * 2.5), 0.5)
        stage.terminate(None)
        self.assertEqual(speeds[-1], stage.free_speed_mps)

    def test_green_beyond_the_near_gate_is_no_lead(self):
        """my_world: a delivery-zone pad ranged 25 m became the orbit centre
        and the orbit crossed red ground."""
        clock = Clock()
        mav = BoardMav((0.0, 30.0, 5.0), board=(0.0, 0.0), normal=math.pi)
        stage = AlignToBanner(mav, clock=clock, dwell_s=1.0)
        stage.initialise()
        logs = lambda: " ".join(str(line) for line in mav.logs)   # noqa: E731
        run(stage, clock, until=lambda: "Orbiting" in logs(), ticks=6000)
        self.assertIn("relocation", logs(), "the search never moved on")
        self.assertNotIn("Orbiting", logs())

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
