"""FindReturnBanner: the return corridor is found with the camera, anywhere.

The fake aircraft turns and moves where it is told. The camera "identifies" a
banner when it is within range, inside the field of view, and seen from in
FRONT of the board (the lettering side). Bearing and board area are what the
real detector reports, so the stage's range and position estimate is tested
against the same numbers it gets live.
"""

import math
import sys
import unittest
from pathlib import Path

import py_trees

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from test_fail_closed_stages import FakeMav  # noqa: E402

HFOV = 1.0472
W_PX = 1280
FOCAL = 0.5 * W_PX / math.tan(HFOV / 2)
AREA_M2 = 3.7 * 1.15


class World(FakeMav):
    def __init__(self, banners, zone=(10.0, 50.0, -15.0, 15.0), start=(14.0, -2.0, 0.0)):
        super().__init__()
        self.banners = banners              # [(x, y, facing_rad)]
        self.zone = zone
        self._pos = (start[0], start[1], 5.0)
        self._alt = 5.0
        self._yaw = start[2]
        self.exclusions = []
        self.outbound_banner_xy = None
        self.corridor_exit_pose = (12.0, 2.0, 3.0, 0.0)
        self.banner_board_area = 0.0
        self.path = []

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z, yaw))
        self._pos = (x, y, z)
        self._yaw = yaw
        self.path.append((x, y))
        self._look()

    def delivery_search_zone(self, inset):
        x0, x1, y0, y1 = self.zone
        return (x0 + inset, x1 - inset, y0 + inset, y1 - inset)

    def _look(self):
        self.banner_z, self.banner_x, self.banner_board_area = 0.0, 0.0, 0.0
        x, y = self._pos[:2]
        best = None
        for bx, by, facing in self.banners:
            dx, dy = x - bx, y - by
            r = math.hypot(dx, dy)
            # In front of the board, within 12 m, inside the FOV.
            if r > 12.0 or dx * math.cos(facing) + dy * math.sin(facing) <= 0:
                continue
            ang = math.atan2(by - y, bx - x) - self._yaw
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) > HFOV / 2:
                continue
            if best is None or r < best[0]:
                best = (r, ang)
        if best:
            r, ang = best
            self.banner_z = 1.0
            self.banner_x = -math.tan(ang) / math.tan(HFOV / 2)   # +x right
            self.banner_board_area = FOCAL ** 2 * AREA_M2 / r ** 2


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.1
        return self.t


def run(mav, n=5000):
    from mission_bt.mission_tree import FindReturnBanner
    leaf = FindReturnBanner(mav, alt=5.0, clock=Clock(), hfov_rad=HFOV,
                            image_width_px=W_PX, near_range_m=9.6)
    leaf.initialise()
    mav._look()
    st = py_trees.common.Status.RUNNING
    for _ in range(n):
        st = leaf.update()
        if st != py_trees.common.Status.RUNNING:
            break
    return leaf, st


class ReturnGateSearchTests(unittest.TestCase):

    def test_beside_the_outbound_lane_it_is_found_on_the_first_look(self):
        """The rulebook layout: the banner is dead ahead of the stand-off."""
        mav = World([(10.0, -2.0, 0.0)], start=(14.5, -2.0, math.pi))
        leaf, st = run(mav)
        self.assertEqual(st, py_trees.common.Status.SUCCESS)
        self.assertEqual(leaf.vi, -1, "it went searching when it did not need to")
        self.assertTrue(any("identified here" in m for m, _ in mav.logs))

    def test_a_return_corridor_elsewhere_is_found_from_the_zone_edge(self):
        """Return banner on the zone's south edge, 18 m from the stand-off,
        facing north into the zone: nothing to see from the stand-off."""
        mav = World([(26.0, -18.0, math.pi / 2)], start=(14.5, 2.0, math.pi))
        leaf, st = run(mav)
        self.assertEqual(st, py_trees.common.Status.SUCCESS, mav.logs[-3:])
        self.assertGreaterEqual(leaf.vi, 0)
        # It ends in front of the banner, near the stand-off distance, facing it.
        x, y = mav.pos()[:2]
        d = math.hypot(x - 26.0, y + 18.0)
        self.assertLess(abs(d - 5.0), 1.5)
        self.assertGreater(y, -18.0, "stood off behind the board")

    def test_the_outbound_banner_is_not_taken_for_the_return_one(self):
        mav = World([(2.0, 2.0, math.pi)], start=(1.0, 2.0, 0.0))
        mav.outbound_banner_xy = (2.0, 2.0)
        mav.banners = [(2.0, 2.0, math.pi)]
        mav._pos = (-3.0, 2.0, 5.0)           # in front of the outbound board
        leaf, st = run(mav)
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertIn("no return banner identified", leaf.feedback_message)

    def test_a_banner_beyond_the_near_range_is_not_accepted(self):
        mav = World([(14.5 - 11.0, -2.0, 0.0)], start=(14.5, -2.0, math.pi),
                    zone=(14.0, 16.0, -3.0, -1.0))
        leaf, st = run(mav)
        self.assertEqual(st, py_trees.common.Status.FAILURE)

    def test_it_never_flies_outside_the_zone_edge_it_searches(self):
        mav = World([(26.0, -18.0, math.pi / 2)], start=(14.5, 2.0, math.pi))
        run(mav)
        x0, x1, y0, y1 = mav.zone
        for x, y in mav.path[1:]:
            self.assertTrue(x0 - 0.1 <= x <= x1 + 0.1 and y0 - 0.1 <= y <= y1 + 0.1,
                            (x, y))

    def test_it_is_in_the_tree_before_the_return_alignment(self):
        from mission_bt.mission_tree import build_root
        import test_behavior_tree as tb
        t = tb.TestMissionBT()
        t.setUp()
        root = build_root(t.mav, t.node, t.defaults)
        names = [n.name for n in root.iterate()]
        i = names.index("FindReturnBanner")
        self.assertLess(names.index("DescendToReturnIdent"), i)
        later = [j for j, n in enumerate(names) if n == "AlignToBanner"]
        self.assertTrue(any(j > i for j in later))


if __name__ == "__main__":
    unittest.main()
