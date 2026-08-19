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


class SlewMav(SweepMav):
    """A heading that SLEWS toward the command instead of teleporting to it.

    This is the whole point of the class. With a fake that reaches the
    commanded yaw instantly, a per-tick proportional correction converges
    beautifully and the tests pass -- which is exactly why the live aircraft
    oscillated for a hundred seconds while the suite was green.

    A real airframe is still turning when the next setpoint is computed. If
    that setpoint is recomputed from the CURRENT heading every tick, the
    target runs away from the aircraft at the same speed the aircraft chases
    it. The stack has met this before, under the name "receding carrot", in
    ApproachBanner.
    """

    def __init__(self, banner_at, banner_arc=math.radians(30.0),
                 slew_rad_per_tick=math.radians(3.0)):
        super().__init__(banner_at=banner_at, banner_arc=banner_arc)
        self.slew = float(slew_rad_per_tick)

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z, yaw))
        self.commanded_yaws.append(yaw)
        err = math.atan2(math.sin(yaw - self._yaw), math.cos(yaw - self._yaw))
        step = max(-self.slew, min(self.slew, err))
        self._yaw = math.atan2(math.sin(self._yaw + step),
                               math.cos(self._yaw + step))


class AlignmentConvergesTests(unittest.TestCase):
    """The live regression: the sweep found the banner and then hunted.

    MEASURED IN FLIGHT, seed 1001, arena regression with the GUI up. The
    stop-and-stare sweep worked -- "banner identified at -0 deg after staring
    at 1 heading(s) (11/11 frames)" -- and the aircraft then swung between
    -1 and -31 degrees on a five-second cycle and never converged:

        t      yaw     bearing  identified
        6.5    -1.1     0.82    yes
        9.6   -15.7     0.79    yes
        10.7  -26.7     0.72    yes
        11.2  -31.4     0.00    NO      <- detector drops it
        13.7   -1.3     0.00    NO      <- snapped back to the dwell heading
        14.2   -0.5     0.81    yes     <- re-acquired, starts over

    Two defects. The correction was recomputed from the current heading on
    every tick, so it receded; and when the banner dropped, the hold used the
    DWELL heading rather than the alignment target, throwing away every degree
    of progress made since.
    """

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.5)
        kw.setdefault("hfov_rad", math.radians(60.0))
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_a_banner_off_to_one_side_is_actually_centred(self):
        """The headline regression. The banner sits 24 degrees off; the stage
        has to end up pointing at it, not hunting around it."""
        mav = SlewMav(banner_at=math.radians(24.0))
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=3000)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      "alignment never converged")
        err = abs(math.degrees(math.atan2(
            math.sin(mav._yaw - math.radians(24.0)),
            math.cos(mav._yaw - math.radians(24.0)))))
        self.assertLess(err, 8.0, f"finished {err:.0f} deg off the banner")

    def test_the_commanded_heading_does_not_recede_while_the_aircraft_turns(self):
        """A setpoint recomputed from the current heading every tick moves
        away as fast as the aircraft closes on it."""
        mav = SlewMav(banner_at=math.radians(24.0))
        stage = self._stage(mav)
        for _ in range(3000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage.phase is stage.CENTRE and len(mav.commanded_yaws) > 4:
                break
        held = mav.commanded_yaws[-3:]
        self.assertEqual(len(set(round(y, 6) for y in held)), 1,
                         f"the alignment target moved every tick: {held}")

    def test_it_does_not_oscillate(self):
        """Stated as the operator sees it: the heading must settle, not swing.

        The live cycle spanned 30 degrees and repeated every five seconds.
        """
        mav = SlewMav(banner_at=math.radians(24.0))
        stage = self._stage(mav)
        run(stage, mav, self.clock, ticks=3000)
        tail = [math.degrees(y) for y in mav.commanded_yaws[-12:]]
        self.assertLess(max(tail) - min(tail), 10.0,
                        f"still hunting: commanded yaw spanned "
                        f"{max(tail) - min(tail):.0f} deg at the end")

    def test_a_dropped_frame_holds_the_ALIGNMENT_target_not_the_dwell_heading(self):
        """The snap-back. Holding the heading the dwell chose discards every
        degree of alignment achieved since, which is what made the cycle
        repeat rather than merely wobble."""
        mav = SlewMav(banner_at=math.radians(24.0))
        stage = self._stage(mav)
        dwell_heading = None
        for _ in range(3000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                self.fail("aligned before the test could drop a frame")
            self.clock.advance(0.1)
            if stage.phase is stage.CENTRE:
                if dwell_heading is None:
                    dwell_heading = stage._target_yaw
                if abs(mav._yaw - dwell_heading) > math.radians(8.0):
                    break
        mav.banner_at = None                        # detector drops it
        stage.update()
        commanded = mav.commanded_yaws[-1]
        self.assertGreater(
            abs(math.degrees(commanded - dwell_heading)), 4.0,
            "snapped back to the heading the dwell chose, discarding the "
            "alignment achieved since")

    def test_the_correction_is_derived_from_the_CAMERA_not_a_constant(self):
        """A bearing is a fraction of the half-FOV. Turning it into an angle
        with a magic gain is a hidden assumption about the lens; the stack has
        one field of view and it is already known."""
        narrow = SlewMav(banner_at=math.radians(10.0))
        wide = SlewMav(banner_at=math.radians(10.0))
        s_narrow = self._stage(narrow, hfov_rad=math.radians(40.0))
        s_wide = self._stage(wide, hfov_rad=math.radians(90.0))
        targets = []
        for st, mv in ((s_narrow, narrow), (s_wide, wide)):
            for _ in range(3000):
                if st.update() is not py_trees.common.Status.RUNNING:
                    break
                self.clock.advance(0.1)
                # Wait for a correction to actually be COMMANDED -- until then
                # both stages are still holding the heading the dwell chose.
                if st._corrections >= 1:
                    break
            targets.append(st._align_target)
        self.assertNotAlmostEqual(targets[0], targets[1], places=3,
                                  msg="the same bearing produced the same "
                                      "correction at two different fields of "
                                      "view, so the lens is not being used")

    def test_a_banner_already_centred_succeeds_without_hunting(self):
        mav = SlewMav(banner_at=0.0)
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=3000)
        self.assertIs(status, py_trees.common.Status.SUCCESS)


class ZigzagSweepTests(unittest.TestCase):
    """The sweep searches OUTWARD from where it started, not round in a circle.

    THE OPERATOR'S INSTRUCTION: "instead of rotating i want it to oscillate
    like 30 to the right then 60 to the left then 90 to the right and then 120
    to the left and so on".

    WHY IT IS BETTER, not merely different. The gate is in front of the
    aircraft far more often than it is behind it -- the start pad faces it.
    A one-way rotation gives the most likely headings no priority at all, so
    a banner 30 degrees to the right costs one step if you happen to turn
    right and eleven if you turn left. Expanding alternately means the nearest
    headings are always tried first, and the worst case is still one full
    turn.
    """

    def setUp(self):
        self.clock = Clock()

    def _headings(self, mav, stage, limit=12):
        seen = []
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            h = round(math.degrees(stage._target_yaw))
            if not seen or seen[-1] != h:
                seen.append(h)
            if len(seen) >= limit:
                break
        return seen

    def test_it_alternates_outward_from_the_start_heading(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        seen = self._headings(mav, stage, limit=7)
        self.assertEqual(seen[:7], [0, -30, 30, -60, 60, -90, 90],
                         f"not an expanding zigzag: {seen}")

    def test_the_first_move_is_to_the_RIGHT(self):
        """"30 to the right" first, as asked."""
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        seen = self._headings(mav, stage, limit=2)
        self.assertLess(seen[1], seen[0])

    def test_it_still_covers_a_full_turn(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        run(stage, mav, self.clock)
        self.assertEqual(len(stage.step_reports), 12)
        covered = sorted(round(r["heading_deg"]) for r in stage.step_reports)
        self.assertEqual(len(set(covered)), 12, f"repeated a heading: {covered}")

    def test_a_banner_just_to_the_right_is_found_in_ONE_step(self):
        """The whole point: nearest first."""
        mav = SweepMav(banner_at=math.radians(-30.0),
                       banner_arc=math.radians(20.0))
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertLessEqual(len(stage.step_reports), 2,
                             "took the long way round to a banner 30 deg away")

    def test_a_banner_just_to_the_LEFT_is_found_in_two_steps(self):
        mav = SweepMav(banner_at=math.radians(30.0),
                       banner_arc=math.radians(20.0))
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        status = run(stage, mav, self.clock)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertLessEqual(len(stage.step_reports), 3)

    def test_headings_are_reported_as_offsets_that_make_sense(self):
        mav = SweepMav(banner_at=None)
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.5,
                              step_rad=math.radians(30.0))
        stage.initialise()
        run(stage, mav, self.clock)
        for r in stage.step_reports:
            self.assertGreaterEqual(r["heading_deg"], -180.5)
            self.assertLessEqual(r["heading_deg"], 180.5)


class StrafeWhenYawStallsTests(unittest.TestCase):
    """Yaw cannot centre a long structure. Translation can.

    MEASURED, seed 1001 run 9, with the banner locked on 12 frames of 12:

        correction 1: bearing +0.28 at -30 deg -> commanding -37
        correction 2: bearing +0.29 at -37 deg -> commanding -44

    Seven degrees of yaw toward a point target should cut a 0.28 bearing by
    about 0.23. It moved +0.01, the wrong way -- because the gate runs away
    from the aircraft, so yawing toward it brings more of it into frame and
    the centroid slides right by as much as the rotation moved it left.

    The operator called this before the measurement did: "move the drone left
    and right in a horizontal way so that the banner comes in front".
    """

    def setUp(self):
        self.clock = Clock()

    class Stubborn(SweepMav):
        """A banner whose bearing does not respond to yaw, as measured."""

        def __init__(self, bearing=0.28, yields_to_strafe=True):
            super().__init__(banner_at=0.0, banner_arc=math.radians(30.0))
            self._fixed = bearing
            self.yields = yields_to_strafe
            self.strafed = 0

        def goto(self, x, y, z, yaw=0.0):
            if (x, y) != self._pos[:2]:
                self.strafed += 1
                if self.yields:
                    self._fixed *= 0.45      # coming into line with it
            self._pos = (x, y, z)            # arrives at the commanded point
            super().goto(x, y, z, yaw)

        def reached(self, x, y, z, tol=0.6):
            return math.dist(self._pos, (x, y, z)) < max(tol, 0.75)

        def banner_identified(self):
            return True

        def banner_bearing(self):
            return self._fixed

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.4)
        kw.setdefault("align_dwell_s", 0.2)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_it_strafes_once_yaw_stops_closing(self):
        mav = self.Stubborn()
        stage = self._stage(mav)
        run(stage, mav, self.clock, ticks=2000)
        self.assertGreater(mav.strafed, 0,
                           "kept yawing at a bearing that never improved")

    def test_a_strafe_STEP_does_not_also_rotate(self):
        """Yaw and roll both belong here -- the operator asked for exactly
        that -- but not in the same command. Rotating while translating
        reintroduces the coupling that made yaw useless: the centroid shift
        from the rotation would be indistinguishable from the one the
        translation is trying to measure."""
        mav = self.Stubborn()
        stage = self._stage(mav)
        prev = None
        for _ in range(2000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if len(mav.gotos) >= 2:
                a, b = mav.gotos[-2], mav.gotos[-1]
                moved = (a[0], a[1]) != (b[0], b[1])
                turned = abs(a[3] - b[3]) > 1e-6
                self.assertFalse(moved and turned,
                                 f"translated and rotated at once: {a} -> {b}")
            if mav.strafed >= 2:
                break

    def test_it_strafes_toward_the_side_the_banner_is_on(self):
        """An object off to starboard comes into line as you move starboard."""
        mav = self.Stubborn(bearing=0.28)
        stage = self._stage(mav)
        start = mav.pos()[:2]
        for _ in range(2000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if mav.strafed:
                break
        moved = (mav.pos()[0] - start[0], mav.pos()[1] - start[1])
        # heading ~0 (east), banner to starboard -> -y in ENU
        self.assertLess(moved[1], -0.1, f"strafed the wrong way: {moved}")

    def test_a_strafe_that_works_ends_in_alignment(self):
        mav = self.Stubborn(yields_to_strafe=True)
        status = run(self._stage(mav), mav, self.clock, ticks=4000)
        self.assertIs(status, py_trees.common.Status.SUCCESS)

    def test_a_strafe_that_never_helps_FAILS_CLOSED(self):
        """Sliding sideways for ever is not better than hunting for ever."""
        mav = self.Stubborn(yields_to_strafe=False)
        status = run(self._stage(mav), mav, self.clock, ticks=6000)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertIn("sideways", mav.abort_reason)

    def test_a_banner_that_yaw_CAN_centre_never_strafes(self):
        """The strafe is a fallback, not the normal path: it costs mission
        time and moves the aircraft near the gate."""
        mav = SweepMav(banner_at=math.radians(12.0),
                       banner_arc=math.radians(30.0))
        stage = self._stage(mav)
        run(stage, mav, self.clock, ticks=3000)
        moved = [g for g in mav.gotos if g[:2] != (0.0, 0.0)]
        self.assertEqual(moved, [], "strafed when yaw was working")


class SquareOnBeforeAdvancingTests(unittest.TestCase):
    """Centred is not in front. The waypoint may only be set from square-on.

    THE OPERATOR'S REPORT: "rn it is just flying out of the world ... it should
    only set the waypoint 10 metres ahead of it when it is completely in front
    of the banner while it maintains some distance".

    Facing the gate from off to one side and then committing to a waypoint
    through it drives at the board rather than through the opening. A board is
    widest seen face-on, so its apparent aspect answers "am I in front of it"
    directly -- and orbiting (sideways step, then yaw back on to it) walks an
    arc around it at constant range without ever closing the distance.
    """

    def setUp(self):
        self.clock = Clock()

    class Oblique(SweepMav):
        """Centred immediately, but only square after a few orbit steps."""

        def __init__(self, steps_to_square=3, ceiling=3.6, start=1.1):
            super().__init__(banner_at=0.0, banner_arc=math.radians(30.0))
            self.aspect = start
            self.steps = 0
            self.steps_to_square = steps_to_square
            self.ceiling = ceiling

        def goto(self, x, y, z, yaw=0.0):
            if (x, y) != self._pos[:2]:
                self.steps += 1
                if self.steps <= self.steps_to_square:
                    self.aspect = min(self.ceiling, self.aspect * 1.35)
            self._pos = (x, y, z)
            super().goto(x, y, z, yaw)

        def reached(self, x, y, z, tol=0.6):
            return math.dist(self._pos, (x, y, z)) < max(tol, 0.75)

        def banner_identified(self):
            return True

        def banner_bearing(self):
            return 0.0

        def banner_aspect(self):
            return self.aspect

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.4)
        kw.setdefault("align_dwell_s", 0.2)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_a_centred_but_OBLIQUE_banner_does_not_finish_alignment(self):
        mav = self.Oblique()
        stage = self._stage(mav)
        for _ in range(6):
            self.assertIs(stage.update(), py_trees.common.Status.RUNNING)
            self.clock.advance(0.1)

    def test_it_orbits_until_the_board_looks_like_a_BANNER(self):
        """Not "until it stops widening" -- a board that never widens at all
        satisfies that, which is how the aircraft sat at aspect 0.99 (edge on)
        and advanced into the gate."""
        mav = self.Oblique(steps_to_square=3, ceiling=3.6)
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=3000)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertGreaterEqual(mav.steps, 2, "did not orbit to come square")
        self.assertGreaterEqual(mav.aspect, 2.0, "finished before it was square")

    def test_the_orbit_never_closes_the_DISTANCE(self):
        """"while it maintains some distance" -- the advance is a separate
        stage and comes afterwards. Every step here is perpendicular to the
        heading, so range to the board is preserved."""
        mav = self.Oblique()
        stage = self._stage(mav)
        run(stage, mav, self.clock, ticks=3000)
        for a, b in zip(mav.gotos, mav.gotos[1:]):
            if (a[0], a[1]) == (b[0], b[1]):
                continue
            step = math.atan2(b[1] - a[1], b[0] - a[0])
            off = abs(math.atan2(math.sin(step - a[3]), math.cos(step - a[3])))
            self.assertAlmostEqual(off, math.pi / 2, places=1,
                                   msg="moved along the heading, not across it")

    def test_a_banner_already_square_finishes_without_orbiting(self):
        mav = self.Oblique(steps_to_square=0, ceiling=3.6, start=3.6)
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=3000)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertEqual(mav.steps, 0, "orbited when already in front of it")

    def test_a_board_that_never_comes_square_REFUSES_to_advance(self):
        """The failure the operator watched: edge-on at 0.99, and it pitched
        in anyway. Refusing is the only safe answer -- advancing at a board
        seen edge-on drives into it."""
        mav = self.Oblique(steps_to_square=0, ceiling=1.0, start=1.0)
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=6000)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertIn("in front", mav.abort_reason)

    def test_the_orbit_keeps_ONE_direction(self):
        """Direction used to come from the sign of the bearing, which flips
        about zero once the banner is centred -- so the aircraft rolled left,
        right, left, going nowhere. Watched live."""
        mav = self.Oblique(steps_to_square=3, ceiling=3.6)
        stage = self._stage(mav)
        seen = []
        for _ in range(3000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if len(mav.gotos) >= 2:
                a, b = mav.gotos[-2], mav.gotos[-1]
                if (a[0], a[1]) != (b[0], b[1]):
                    seen.append(math.atan2(b[1] - a[1], b[0] - a[0]))
        if len(seen) >= 2:
            for d in seen[1:]:
                self.assertLess(
                    abs(math.atan2(math.sin(d - seen[0]),
                                   math.cos(d - seen[0]))), 0.3,
                    f"orbit reversed direction: {seen}")
