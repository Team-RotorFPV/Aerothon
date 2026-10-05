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


class LatchedMav(NadirMav):
    """The offset is latched: it holds the last frame's value, and a new
    frame arrives only every third tick. It starts holding a sighting from
    before the stage -- taken while the camera swung to nadir."""

    def __init__(self, marker, stale, **kw):
        super().__init__(marker, **kw)
        self.qr_offset = SimpleNamespace(x=stale[0], y=stale[1], z=0.5)
        self.qr_offset_seq = 7
        self._calls = 0

    def qr_visible(self):
        self._calls += 1
        if self._calls % 3 == 0:
            self._see()
            self.qr_offset_seq += 1
        return self.qr_offset.z > 0.0


class WholeMarkerMav(NadirMav):
    """A detector that needs the WHOLE marker in frame, as a QR decoder does."""

    SIZE = 2.2

    def _see(self):
        x, y, z = self._pos
        half_w = z * math.tan(HFOV / 2)
        half_h = half_w * 720 / 1280
        fwd, right = self.marker[0] - x, -(self.marker[1] - y)
        h = self.SIZE / 2
        if abs(fwd) + h > half_h or abs(right) + h > half_w:
            self.qr_offset = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        else:
            self.qr_offset = SimpleNamespace(x=right / half_w, y=-fwd / half_h, z=0.5)


class SlowFrameMav(NadirMav):
    """What the real loop sees: a frame every `every` ticks, showing where the
    aircraft was `lag` ticks before, latched between frames; and a vehicle
    with momentum -- a position loop over an acceleration-limited velocity,
    as ArduPilot's GUIDED position target flies (dt 0.1 s)."""

    def __init__(self, marker, pos, every=4, lag=3, kp=1.0, vmax=2.0,
                 accel=2.5, dt=0.1):
        super().__init__(marker, pos=pos)
        self.every, self.lag = every, lag
        self.kp, self.vmax, self.accel, self.dt = kp, vmax, accel, dt
        self.qr_offset_seq = 0
        self._tick = 0
        self._goal = pos
        self._v = (0.0, 0.0)
        self._hist = [pos] * (lag + 1)
        self._see()

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z))
        self._goal = (x, y, z)

    def _see(self):
        now = self._pos
        self._pos = self._hist[0]                   # the frame is `lag` old
        super()._see()
        self._pos = now

    def step(self):
        (px, py, _), (gx, gy, gz) = self._pos, self._goal
        wx, wy = self.kp * (gx - px), self.kp * (gy - py)
        w = math.hypot(wx, wy)
        if w > self.vmax:
            wx, wy = wx * self.vmax / w, wy * self.vmax / w
        dvx, dvy = wx - self._v[0], wy - self._v[1]
        dv = math.hypot(dvx, dvy)
        lim = self.accel * self.dt
        if dv > lim:
            dvx, dvy = dvx * lim / dv, dvy * lim / dv
        self._v = (self._v[0] + dvx, self._v[1] + dvy)
        self._pos = (px + self._v[0] * self.dt, py + self._v[1] * self.dt, gz)
        self._hist = self._hist[1:] + [self._pos]
        self._tick += 1
        if self._tick % self.every == 0:
            self._see()
            self.qr_offset_seq += 1

    def qr_visible(self):
        return self.qr_offset.z > 0.0

    def qr_centred(self, tol):
        return (self.qr_visible() and abs(self.qr_offset.x) <= tol
                and abs(self.qr_offset.y) <= tol)


class TargetCentringTests(unittest.TestCase):

    def test_slow_frames_do_not_make_the_centring_swing(self):
        """my_world, pad B: the correction was applied every tick from a
        latched offset and the aircraft swung +-0.6 m for the whole stage."""
        mav = SlowFrameMav(marker=(1.8, -1.2), pos=(0.0, 0.0, 10.0))
        stage = CenterOnQR("CenterOnTarget", mav, tol=0.08, settle_ticks=3,
                           hfov_rad=HFOV)
        stage.initialise()
        status, worst = py_trees.common.Status.RUNNING, 0.0
        for _ in range(120):
            status = stage.update()
            if status is not py_trees.common.Status.RUNNING:
                break
            mav.step()
            worst = max(worst, mav.pos()[0] - 1.8)
        self.assertIs(status, py_trees.common.Status.SUCCESS, mav.abort_reason)
        self.assertLess(worst, 0.3, f"overshot the pad by {worst:.2f} m")


    def test_a_frame_taken_banked_does_not_move_the_goal(self):
        """At 10 m a 10 deg bank moves the image 1.8 m: pad B's swing."""
        mav = SlowFrameMav(marker=(1.0, 0.0), pos=(0.0, 0.0, 10.0))
        mav.roll_deg, mav.pitch_deg = 0.0, 0.0
        stage = CenterOnQR("CenterOnTarget", mav, hfov_rad=HFOV)
        stage.initialise()
        for _ in range(6):                          # level: a goal is set
            stage.update()
        goal = stage._goal
        mav.roll_deg = 10.0
        mav.marker = (2.0, 0.0)                     # what the tilt makes it look like
        mav._hist = [mav._pos] * len(mav._hist)
        mav._see()
        mav.qr_offset_seq += 1
        stage.update()
        self.assertEqual(stage._goal, goal)
        mav.roll_deg = 0.0
        for _ in range(6):
            mav._see()
            mav.qr_offset_seq += 1
            stage.update()
        self.assertNotEqual(stage._goal, goal, "level again, it looks again")


class StartMarkerTests(unittest.TestCase):

    def test_a_marker_cut_by_the_frame_edge_is_climbed_above(self):
        """my_world, twice: at 4.85 m the C270 covers 2.4 m front-to-back.
        At the fix, 0.38 m off the 2.2 m marker, it was never seen whole."""
        mav = WholeMarkerMav(marker=(1.0, 0.0), pos=(1.38, 0.07, 4.85))
        mav.marker_xy = (1.38, 0.07)
        centre = CenterOnQR("CenterStartQR", mav, hfov_rad=HFOV,
                            climb_step_m=1.0, max_alt_m=9.0)
        centre.initialise()
        status = py_trees.common.Status.RUNNING
        for _ in range(400):
            status = centre.update()
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.SUCCESS, mav.abort_reason)
        self.assertGreater(mav.pos()[2], 5.0)
        self.assertAlmostEqual(mav.pos()[0], 1.0, delta=0.35)
        self.assertTrue(any("climbing" in str(l) for l in mav.logs))

    def test_a_sighting_from_before_the_stage_is_not_one(self):
        """my_world: the first offset FindStartQR read was from the camera's
        swing to nadir; the fix was 1.5 m out and CenterStartQR failed."""
        mav = LatchedMav(marker=(0.94, 0.0), stale=(0.0, 0.9))
        find = FindStartQR(mav, 5.0, hfov_rad=HFOV)
        find.initialise()
        status = py_trees.common.Status.RUNNING
        for _ in range(30):
            status = find.update()
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertAlmostEqual(mav.marker_xy[0], 0.94, delta=0.05)

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

    def _land(self, marker_now):
        """Fixed at (0.94, 0) at the start; seen at `marker_now` on return."""
        from mission_bt.mission_tree import PrecisionDescent
        mav = NadirMav(marker=marker_now, pos=(0.0, 0.0, 5.0), visible_ticks=1)
        mav.home_marker_xy = (0.94, 0.0)
        mav.home_local_xy = lambda: (0.0, 0.0)
        mav.landing_precision = None
        stage = PrecisionDescent(mav, start_alt=5.0, marker_m=2.2, hfov_rad=HFOV)
        stage.initialise()
        for _ in range(800):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
        return mav

    def test_it_lands_where_it_took_off_not_on_the_marker(self):
        """The start QR stands forward of the take-off point."""
        mav = self._land((0.94, 0.0))
        self.assertTrue(str(mav.landing_precision).startswith("PRECISE"))
        self.assertLess(math.hypot(*mav.pos()[:2]), 0.4)

    def test_estimate_drift_is_taken_out_by_the_marker(self):
        """A metre of drift since take-off: the marker reads 1 m further on,
        and so must the take-off point."""
        mav = self._land((1.94, 0.0))
        self.assertAlmostEqual(mav.pos()[0], 1.0, delta=0.4)

    def test_the_landing_goes_to_the_pad_and_locks(self):
        from mission_bt.mission_tree import PrecisionDescent
        mav = NadirMav(marker=(0.94, 0.0), pos=(0.0, 0.0, 5.0), visible_ticks=1)
        mav.home_marker_xy = (0.94, 0.0)
        mav.home_local_xy = lambda: (0.0, 0.0)
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
        self.assertLess(abs(mav.pos()[0]), 0.4, "did not return to take-off")
