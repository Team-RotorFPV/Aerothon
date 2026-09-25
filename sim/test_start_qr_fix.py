#!/usr/bin/env python3
"""A start marker glimpsed at the frame edge is flown to, not waited for.

Third flight on the team airframe: from above the take-off point the C270
(48.8 deg across, 28.6 deg front-to-back) had the 2.2 m start marker 0.9 m
ahead, half out of frame. FindStartQR saw it for a frame and succeeded;
CenterStartQR then found nothing in view, held where it was -- where the
marker cannot be seen -- and failed "no marker to centre on".

    python3 -m pytest sim/test_start_qr_fix.py -v
"""

import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

import py_trees                                              # noqa: E402
from mission_bt.mission_tree import CenterOnQR, FindStartQR  # noqa: E402

HFOV = 0.851919


class NadirMav:
    """A marker at `marker` seen by a nadir C270; the aircraft goes where told."""

    def __init__(self, marker, pos=(0.0, 0.0, 5.0), visible_ticks=None):
        self.marker = marker
        self._pos = pos
        self.gotos = []
        self.logs = []
        self.abort_reason = ""
        self.qr_offset = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.visible_ticks = visible_ticks       # force a glimpse then loss
        self._n = 0

    def pos(self):
        return self._pos

    def alt(self):
        return self._pos[2]

    def yaw(self):
        return 0.0

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z))
        self._pos = (x, y, z)

    def log(self, *a, **k):
        self.logs.append(a)

    def _see(self):
        x, y, z = self._pos
        half_w = z * math.tan(HFOV / 2)
        half_h = half_w * 720 / 1280
        fwd, right = self.marker[0] - x, -(self.marker[1] - y)
        self._n += 1
        glimpse_over = self.visible_ticks is not None and self._n > self.visible_ticks
        near_edge = abs(fwd) > 0.7 * half_h
        if abs(fwd) > half_h or abs(right) > half_w or (glimpse_over and near_edge):
            self.qr_offset = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        else:
            self.qr_offset = SimpleNamespace(x=right / half_w, y=-fwd / half_h, z=0.5)

    def qr_visible(self):
        self._see()
        return self.qr_offset.z > 0.0

    def qr_centred(self, tol):
        return self.qr_visible() and abs(self.qr_offset.x) <= tol and abs(self.qr_offset.y) <= tol


class StartMarkerTests(unittest.TestCase):

    def test_a_glimpsed_marker_is_flown_to_and_centred(self):
        mav = NadirMav(marker=(0.94, 0.0), visible_ticks=1)
        find = FindStartQR(mav, 5.0, hfov_rad=HFOV)
        find.initialise()
        self.assertIs(find.update(), py_trees.common.Status.SUCCESS)
        self.assertIsNotNone(getattr(mav, "marker_xy", None))
        self.assertAlmostEqual(mav.marker_xy[0], 0.94, delta=0.05)
        centre = CenterOnQR("CenterStartQR", mav, hfov_rad=HFOV)
        centre.initialise()
        status = py_trees.common.Status.RUNNING
        for _ in range(200):
            status = centre.update()
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.SUCCESS, mav.abort_reason)
        self.assertAlmostEqual(mav.pos()[0], 0.94, delta=0.35)


if __name__ == "__main__":
    unittest.main()


class SweepMatchTests(unittest.TestCase):
    """Flight 4 on the team airframe: the sweep matched pad D at the frame
    edge and ended before the offset that places it arrived, so there was no
    ground fix; CenterOnTarget held where it stopped, the pad just out of
    frame, and timed out."""

    def _search(self, mav):
        from mission_bt.mission_tree import LawnmowerSearch
        mav.exclusions = []
        mav.reached = lambda *a, **k: False
        mav.corridor_exit_pose = (17.0, 0.0, 10.0, 0.0)
        mav.avoid_detail = {"open_depth_m": 12.0, "open_width_m": 16.6}
        s = LawnmowerSearch(mav, (16.5, 27.9, -8.1, 6.9), 10.0, hfov_rad=HFOV,
                            image_height_px=720)
        s.initialise()
        return s

    def test_a_match_waits_for_the_pads_position(self):
        mav = NadirMav(marker=(20.0, 2.0), pos=(20.0, 0.0, 10.0))
        mav.target_xy = None
        stage = self._search(mav)
        mav.qr_matched = True                         # the Bool arrived first
        mav.qr_offset = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING)
        held = mav.gotos[-1]
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING)
        self.assertEqual(mav.gotos[-1], held, "it did not stop where it matched")
        mav.qr_offset = SimpleNamespace(x=-0.4, y=0.0, z=1.0)   # ...then the offset
        self.assertIs(stage.update(), py_trees.common.Status.SUCCESS)
        self.assertIsNotNone(mav.target_xy)

    def test_a_fix_that_never_comes_does_not_stall_the_mission(self):
        mav = NadirMav(marker=(20.0, 2.0), pos=(20.0, 0.0, 10.0))
        mav.target_xy = None
        stage = self._search(mav)
        mav.qr_matched = True
        mav.qr_offset = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        status = [stage.update() for _ in range(stage.fix_wait_ticks + 1)]
        self.assertIs(status[-1], py_trees.common.Status.SUCCESS)


class HomePadLandingTests(unittest.TestCase):
    """Flight 5: the whole mission flew, then the precision landing started at
    5 m with the pad 1 m off and a commit altitude of 4.96 m -- 4 cm of room
    to lock -- never locked, and landed 2.7 m from the pad."""

    def test_the_landing_goes_to_the_pad_and_locks(self):
        from mission_bt.mission_tree import PrecisionDescent
        mav = NadirMav(marker=(0.94, 0.0), pos=(0.0, 0.0, 5.0), visible_ticks=1)
        mav.home_marker_xy = (0.94, 0.0)
        mav.landing_precision = None
        stage = PrecisionDescent(mav, start_alt=5.0, marker_m=2.2, hfov_rad=HFOV)
        self.assertGreaterEqual(stage.start_alt, stage.commit_alt + 1.5)
        stage.initialise()
        status = py_trees.common.Status.RUNNING
        for _ in range(800):
            status = stage.update()
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertTrue(str(mav.landing_precision).startswith("PRECISE"),
                        mav.landing_precision)
        self.assertAlmostEqual(mav.pos()[0], 0.94, delta=0.3)
