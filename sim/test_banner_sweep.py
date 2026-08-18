#!/usr/bin/env python3
"""The banner sweep stops and stares instead of yawing continuously.

WHAT THIS REPLACES

    AlignToBanner yawed a little further on every tick while the banner was
    unidentified, and switched to a bearing-following yaw the instant it was.
    Watched live, that reads as an oscillation: the aircraft turns one way,
    the detector confirms for a frame, the stage reverses, the detector drops,
    the stage turns back. It never commits.

    The operator's description was exact -- "it yaws to the right, it detects
    the banner, but as soon as it detects the banner it yaws left".

WHAT THESE TESTS ARE CAREFUL ABOUT

    The clock is injected. A test that really slept five seconds per step
    would take a minute per case, and nobody would run it. The tests advance
    the clock explicitly, which also lets them assert on what happens at the
    boundaries of a dwell rather than near them.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_banner_sweep.py -v
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
from mission_bt.mission_tree import AlignToBanner            # noqa: E402
from test_fail_closed_stages import FakeMav                  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class SweepMav(FakeMav):
    """A vehicle whose heading actually follows the commanded yaw.

    The old stage counted the yaw it had COMMANDED as the yaw it had turned,
    and reported 180 degrees of sweep while the airframe had physically
    rotated 23. Anything asserting on sweep progress has to move a heading.
    """

    def __init__(self, banner_at=None, banner_arc=math.radians(20.0)):
        super().__init__()
        self._pos = (0.0, 0.0, 3.0)
        self._alt = 3.0
        self._yaw = 0.0
        self.banner_at = banner_at        # heading the banner is visible from
        self.banner_arc = banner_arc
        self.commanded_yaws = []
        self.lag = 0                      # ticks of heading lag to simulate

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z, yaw))
        self.commanded_yaws.append(yaw)
        if self.lag > 0:
            self.lag -= 1
            return
        self._yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    def _bearing_now(self):
        if self.banner_at is None:
            return None
        err = math.atan2(math.sin(self.banner_at - self._yaw),
                         math.cos(self.banner_at - self._yaw))
        if abs(err) > self.banner_arc:
            return None
        # Image +x is to the RIGHT of frame centre; a banner to the aircraft's
        # left (positive yaw error) appears at negative image x.
        return -err / self.banner_arc

    def banner_identified(self):
        return self._bearing_now() is not None

    def banner_bearing(self):
        b = self._bearing_now()
        return 0.0 if b is None else b


def run(stage, mav, clock, ticks=4000, dt=0.1):
    """Tick to a terminal status, advancing the clock like the tree would."""
    for _ in range(ticks):
        status = stage.update()
        if status is not py_trees.common.Status.RUNNING:
            return status
        clock.advance(dt)
    return py_trees.common.Status.RUNNING


class DwellTests(unittest.TestCase):
    """The oscillation fix: the aircraft is stationary while it decides."""

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 5.0)
        kw.setdefault("step_rad", math.radians(30.0))
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_the_commanded_yaw_is_held_for_the_whole_dwell(self):
        """The heart of it. A yaw that changes every tick is the oscillation."""
        mav = SweepMav(banner_at=None)
        stage = self._stage(mav)
        for _ in range(40):                      # 4.0 s at 10 Hz
            stage.update()
            self.clock.advance(0.1)
        held = mav.commanded_yaws[1:]
        self.assertTrue(held, "nothing was commanded")
        self.assertEqual(len(set(round(y, 6) for y in held)), 1,
                         f"yaw moved during the dwell: {sorted(set(held))}")

    def test_the_dwell_lasts_the_configured_time(self):
        mav = SweepMav(banner_at=None)
        stage = self._stage(mav, dwell_s=5.0)
        for _ in range(49):                      # 4.9 s
            stage.update()
            self.clock.advance(0.1)
        self.assertEqual(stage.step_index, 0)
        for _ in range(4):
            stage.update()
            self.clock.advance(0.1)
        self.assertEqual(stage.step_index, 1)

    def test_the_next_step_is_one_step_further_round(self):
        mav = SweepMav(banner_at=None)
        stage = self._stage(mav, step_rad=math.radians(30.0))
        first = None
        for _ in range(80):
            stage.update()
            self.clock.advance(0.1)
            if first is None and mav.commanded_yaws:
                first = mav.commanded_yaws[-1]
            if stage.step_index == 1:
                stage.update()          # the tick that commands the new heading
                break
        delta = abs(math.atan2(math.sin(mav.commanded_yaws[-1] - first),
                               math.cos(mav.commanded_yaws[-1] - first)))
        self.assertAlmostEqual(delta, math.radians(30.0), places=2)

    def test_the_sweep_turns_one_way_only(self):
        """A sweep that reverses has covered less than it thinks it has."""
        mav = SweepMav(banner_at=None)
        stage = self._stage(mav)
        seen = []
        for _ in range(400):
            stage.update()
            self.clock.advance(0.1)
            if not seen or seen[-1] != stage.step_index:
                seen.append(stage.step_index)
            if stage.step_index >= 5:
                break
        self.assertEqual(seen, sorted(seen), f"steps went backwards: {seen}")

    def test_position_and_altitude_are_held_throughout(self):
        """Only the heading changes. A sweep that drifts is a sweep that has
        left the place it was told to look from."""
        mav = SweepMav(banner_at=None)
        stage = self._stage(mav)
        for _ in range(60):
            stage.update()
            self.clock.advance(0.1)
        self.assertTrue(all(g[:3] == (0.0, 0.0, 3.0) for g in mav.gotos),
                        "the sweep moved the aircraft")

    def test_it_waits_for_the_heading_to_actually_settle(self):
        """MEASURED, not assumed. Counting commanded yaw as achieved yaw is
        the defect that reported 180 degrees swept for 23 degrees turned.

        The first step's heading is wherever the aircraft already points, so
        it settles instantly; the airframe is then frozen and must not be
        credited with the second step."""
        free = SweepMav(banner_at=None)
        loose = self._stage(free, dwell_s=1.0, settle_timeout_s=60.0)
        for _ in range(40):
            loose.update()
            self.clock.advance(0.1)
        self.assertGreaterEqual(free.commanded_yaws and loose.step_index, 3,
                                "the control case never got moving")

        self.clock.t = 0.0
        mav = SweepMav(banner_at=None)
        mav.lag = 10 ** 6                        # heading refuses to follow
        stage = self._stage(mav, dwell_s=1.0, settle_timeout_s=60.0)
        for _ in range(40):
            stage.update()
            self.clock.advance(0.1)
        self.assertEqual(stage.step_index, 1,
                         "advanced while the airframe was still turning")

    def test_a_heading_that_never_settles_does_not_hang_the_mission(self):
        mav = SweepMav(banner_at=None)
        mav.lag = 10 ** 6
        stage = self._stage(mav, dwell_s=1.0, settle_timeout_s=2.0)
        status = run(stage, mav, self.clock, ticks=2000)
        self.assertIs(status, py_trees.common.Status.FAILURE)


class DecisionTests(unittest.TestCase):

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_a_confident_dwell_ends_the_sweep(self):
        mav = SweepMav(banner_at=math.radians(60.0))
        stage = self._stage(mav)
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.SUCCESS)

    def test_it_ends_facing_the_banner(self):
        mav = SweepMav(banner_at=math.radians(60.0))
        stage = self._stage(mav)
        run(stage, mav, self.clock)
        err = abs(math.atan2(math.sin(mav._yaw - math.radians(60.0)),
                             math.cos(mav._yaw - math.radians(60.0))))
        self.assertLess(err, math.radians(10.0),
                        f"finished {math.degrees(err):.0f} deg off the banner")

    def test_one_stray_frame_does_not_end_the_sweep(self):
        """A single identified frame is what the old stage acted on. The dwell
        exists so a flicker cannot decide anything."""
        class Flicker(SweepMav):
            def __init__(self):
                super().__init__(banner_at=None)
                self.n = 0

            def banner_identified(self):
                self.n += 1
                return self.n == 3        # exactly one frame, very early

            def banner_bearing(self):
                return 0.0

        mav = Flicker()
        stage = self._stage(mav, dwell_s=5.0)
        for _ in range(60):
            stage.update()
            self.clock.advance(0.1)
        self.assertGreaterEqual(stage.step_index, 1,
                                "one frame ended the sweep")

    def test_a_dropped_frame_does_not_restart_the_dwell(self):
        """Losing the banner for one frame mid-dwell used to send the stage
        straight back to searching, which is the reversal the operator saw."""
        class Blinker(SweepMav):
            def __init__(self):
                super().__init__(banner_at=math.radians(0.0))
                self.n = 0

            def banner_identified(self):
                self.n += 1
                return self.n % 4 != 0     # 75% of frames

        mav = Blinker()
        stage = self._stage(mav, dwell_s=2.0, min_hit_ratio=0.6)
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertEqual(stage.step_index, 0, "left the step it had found it on")

    def test_a_marginal_dwell_keeps_looking(self):
        """Below the confidence floor is not a find. The 271-degree lock onto
        something to the south came from acting on weak evidence, not from
        sweeping too far."""
        class Weak(SweepMav):
            def __init__(self):
                super().__init__(banner_at=None)
                self.n = 0

            def banner_identified(self):
                self.n += 1
                return self.n % 5 == 0     # 20% of frames

        mav = Weak()
        stage = self._stage(mav, dwell_s=2.0, min_hit_ratio=0.6)
        for _ in range(60):
            stage.update()
            self.clock.advance(0.1)
        self.assertGreaterEqual(stage.step_index, 1)


class CentringTests(unittest.TestCase):
    """Found in review: the centring timeout measured the wrong interval."""

    def setUp(self):
        self.clock = Clock()

    class Slow(SweepMav):
        """Identified throughout, converging slowly -- an ordinary approach."""

        def __init__(self):
            super().__init__(banner_at=None)
            self.visible = True
            self.bear = 0.9

        def banner_identified(self):
            return self.visible

        def banner_bearing(self):
            self.bear *= 0.995
            return self.bear

    def test_one_dropped_frame_late_in_centring_is_not_a_lost_banner(self):
        """The timeout ran from the moment centring STARTED, so after eight
        seconds of perfectly normal centring a single dropped frame aborted
        the mission -- and said "not seen again within 8 s" about a banner
        that had been in frame a tenth of a second earlier.

        A stage that fails is bad. A stage that fails while describing
        something that did not happen is worse: it sends the next hour of
        debugging somewhere else entirely.
        """
        mav = self.Slow()
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0,
                              settle_timeout_s=8.0)
        stage.initialise()
        for _ in range(400):
            if stage.update() is not py_trees.common.Status.RUNNING:
                self.fail("centring ended before the test could drop a frame")
            self.clock.advance(0.1)
            if stage.phase is stage.CENTRE and self.clock.t > 11.0:
                break
        mav.visible = False
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING,
                      "one dropped frame ended a healthy centring")

    def test_a_banner_that_really_is_gone_still_fails_closed(self):
        """The timeout has to still exist, or centring hangs for ever."""
        mav = self.Slow()
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0,
                              settle_timeout_s=8.0)
        stage.initialise()
        for _ in range(400):
            stage.update()
            self.clock.advance(0.1)
            if stage.phase is stage.CENTRE:
                break
        mav.visible = False
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.FAILURE)

    def test_the_hold_during_a_dropped_frame_keeps_the_chosen_heading(self):
        mav = self.Slow()
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0,
                              settle_timeout_s=8.0)
        stage.initialise()
        for _ in range(400):
            stage.update()
            self.clock.advance(0.1)
            if stage.phase is stage.CENTRE:
                break
        mav.visible = False
        stage.update()
        self.assertAlmostEqual(mav.commanded_yaws[-1], stage._target_yaw)


class FailClosedTests(unittest.TestCase):

    def setUp(self):
        self.clock = Clock()

    def test_a_full_turn_with_nothing_found_fails(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0)
        stage.initialise()
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.FAILURE)

    def test_the_sweep_covers_a_FULL_turn_before_giving_up(self):
        """A half turn was the old limit. A banner behind the start heading was
        unfindable, which is what stranded the return leg of seed 1001."""
        mav = SweepMav(banner_at=math.radians(-150.0))
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0)
        stage.initialise()
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.SUCCESS)

    def test_the_failure_says_how_many_steps_were_stared_at(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0,
                              step_rad=math.radians(30.0))
        stage.initialise()
        run(stage, mav, self.clock)
        self.assertIn("12", mav.abort_reason)
        self.assertIn("360", mav.abort_reason)

    def test_the_failure_carries_the_detector_reason(self):
        mav = SweepMav(banner_at=None)
        mav.banner_reject = "GREEN, NOT BANNER (no lettering read)"
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0)
        stage.initialise()
        run(stage, mav, self.clock)
        self.assertIn("no lettering read", mav.abort_reason)

    def test_every_step_is_recorded_with_what_it_saw(self):
        """'Nothing found' is not a diagnosis. Which twelve headings were
        stared at, and what did each one see?"""
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0)
        stage.initialise()
        run(stage, mav, self.clock)
        self.assertEqual(len(stage.step_reports), 12)
        for r in stage.step_reports:
            self.assertIn("heading_deg", r)
            self.assertIn("hit_ratio", r)
            self.assertIn("samples", r)

    def test_the_step_reports_are_published_for_the_operator(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=1.0)
        stage.initialise()
        run(stage, mav, self.clock)
        swept = [m for m, _ in mav.logs if "stared" in m]
        self.assertTrue(swept, mav.logs)


if __name__ == "__main__":
    unittest.main()
