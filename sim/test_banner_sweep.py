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

import inspect
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
from mission_bt.scan_geometry import (                       # noqa: E402
    bearing_to_angle, no_surface)
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
        # A COOPERATIVE LIDAR, because these tests are about the SWEEP.
        #
        # The sweep's job ends when the banner is centred; squaring up is the
        # next phase and has its own fake, `GateMav`, which models the gate's
        # geometry rather than answering with a constant. Without something
        # here every sweep test would end in the square-up refusing, and would
        # then be reporting on a phase it is not about.
        self.surface = {"ok": True, "angle_rad": 0.0, "range_m": 5.0,
                        "points": 40, "residual_m": 0.004, "extent_m": 3.6,
                        "reason": ""}

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


class LagMav(SweepMav):
    """A vehicle that ARRIVES LATE -- in heading AND in position.

    THE BLIND SPOT THIS CLOSES

        Every fake vehicle in this repo followed setpoints instantly. Against
        one of those, a per-tick proportional correction converges beautifully
        and the suite goes green; against a real airframe it oscillates,
        because the aircraft is still moving when the next setpoint is
        computed and the target recedes at the rate the aircraft closes on it.
        The team calls that the receding carrot. It has now appeared three
        times -- in the banner approach, in centring, and in the orbit -- and
        each time the suite was green when it flew.

        The lag is FIRST ORDER with a rate cap, on both axes:

            first order   the aircraft never quite arrives, so anything that
                          waits for exact arrival hangs rather than passing
            rate cap      finite authority, so a large command does not
                          produce a large single-tick response

        A stage that is correct against this fake is correct about the thing
        that actually flies. A stage that needs the instant fake was never
        being tested for what its name claims.

    Position is what makes it new. `SlewMav` lagged the heading only, so every
    orbit and strafe test in this file measured a vehicle that teleported
    sideways and re-measured the banner from a place it had reached in zero
    time -- which is precisely the defect the live orbit had.
    """

    def __init__(self, banner_at=None, banner_arc=math.radians(30.0),
                 yaw_gain=0.25, yaw_rate=math.radians(4.0),
                 pos_gain=0.22, speed_m=0.30):
        super().__init__(banner_at=banner_at, banner_arc=banner_arc)
        self.yaw_gain = float(yaw_gain)
        self.yaw_rate = float(yaw_rate)
        self.pos_gain = float(pos_gain)
        self.speed_m = float(speed_m)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _fly(self, x, y, z, yaw):
        err = self._wrap(yaw - self._yaw)
        step = max(-self.yaw_rate, min(self.yaw_rate, self.yaw_gain * err))
        self._yaw = self._wrap(self._yaw + step)

        px, py, pz = self._pos
        d = math.dist((px, py, pz), (x, y, z))
        if d > 1e-9:
            f = min(self.pos_gain * d, self.speed_m) / d
            self._pos = (px + (x - px) * f, py + (y - py) * f,
                         pz + (z - pz) * f)
            self._alt = self._pos[2]

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((x, y, z, yaw))
        self.commanded_yaws.append(yaw)
        self._fly(x, y, z, yaw)

    def reached(self, x, y, z, tol=0.6):
        return math.dist(self._pos, (x, y, z)) < tol


class SlewMav(LagMav):
    """Heading lag only, for the tests that are about yaw convergence."""

    def __init__(self, banner_at, banner_arc=math.radians(30.0),
                 slew_rad_per_tick=math.radians(3.0)):
        super().__init__(banner_at=banner_at, banner_arc=banner_arc,
                         yaw_rate=float(slew_rad_per_tick))


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

    class Slow(LagMav):
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


class GateMav(LagMav):
    """A vehicle in a world that has a GATE in it, with a real face.

    WHY THIS IS NOT A CONSTANT

        The fake it replaces answered "the board looks 1.4 wide" from a
        counter, so the orbit tests graded a number the fixture had decided in
        advance. This one holds a gate at a position with a facing, and works
        out what the camera and the lidar would each report from wherever the
        aircraft has actually got to. Moving changes the answers because the
        geometry changes, which is the only way an orbit test can mean
        anything.

    THE SCAN PLANE IS MODELLED TOO. The C1 sweeps one horizontal slice, so a
    gate shorter than the aircraft's altitude is invisible to it however
    perfectly the aircraft is positioned -- measured on seed 1001 as 0 finite
    returns of 720 at 5.0 m, and 289 at 3.0 m. A fake without that cannot
    exercise the descent, and the descent is what makes the whole measurement
    reachable.
    """

    LIDAR_OFFSET_M = 0.235          # sensor height above the vehicle origin

    def __init__(self, gate=(6.0, 0.0), face_rad=math.pi, start=(0.0, 0.0, 3.0),
                 gate_top_m=4.0, hfov=math.radians(60.0),
                 visible=True, lidar_blind=False,
                 max_ident_range_m=None, min_ident_alt_m=None,
                 only_within_m=None, **kw):
        super().__init__(banner_at=None, **kw)
        self.gate = (float(gate[0]), float(gate[1]))
        # Which way the face POINTS. The aircraft squares up when its nose is
        # anti-parallel to this.
        self.face_rad = float(face_rad)
        self.gate_top_m = float(gate_top_m)
        self.hfov = float(hfov)
        self.visible = visible          # the camera can see it
        self.lidar_blind = lidar_blind  # the lidar never finds a face
        # HOW THE CAMERA ACTUALLY LOSES IT, both watched live.
        #
        # `max_ident_range_m` is "green region too small (3416 px, need
        # 8533)": too far away and the board subtends too few pixels to
        # identify, whatever the heading. `min_ident_alt_m` is the sighting
        # lost on the descent. Neither is a bearing error, so neither can be
        # recovered by yawing -- which is the whole point of the tests below.
        self.max_ident_range_m = max_ident_range_m
        self.min_ident_alt_m = min_ident_alt_m
        self.only_within_m = only_within_m       # visible only near one spot
        self.good_spot = None
        self._pos = (float(start[0]), float(start[1]), float(start[2]))
        self._alt = self._pos[2]
        self._yaw = 0.0
        self.surface = None
        self.sector_vs_camera = []

    # ---- what the geometry actually is ---- #
    def _to_gate(self):
        return (self.gate[0] - self._pos[0], self.gate[1] - self._pos[1])

    def _bearing_angle(self):
        vx, vy = self._to_gate()
        return self._wrap(math.atan2(vy, vx) - self._yaw)

    def standoff(self):
        """Perpendicular distance from the aircraft to the gate's face."""
        vx, vy = self._to_gate()
        return -(vx * math.cos(self.face_rad) + vy * math.sin(self.face_rad))

    def off_centreline(self):
        """How far along the face the aircraft is from the gate's centre.

        Positive is to the side `face_rad + 90 degrees` points.
        """
        vx, vy = self._to_gate()
        ux, uy = -math.sin(self.face_rad), math.cos(self.face_rad)
        return -(vx * ux + vy * uy)

    # ---- what the camera reports ---- #
    def banner_identified(self):
        if not self.visible:
            return False
        vx, vy = self._to_gate()
        if self.max_ident_range_m is not None \
                and math.hypot(vx, vy) > self.max_ident_range_m:
            return False
        if self.min_ident_alt_m is not None \
                and self._pos[2] < self.min_ident_alt_m:
            return False
        if self.only_within_m is not None and self.good_spot is not None \
                and math.dist(self._pos[:2], self.good_spot) > self.only_within_m:
            return False
        return abs(self._bearing_angle()) < self.hfov / 2.0

    def banner_bearing(self):
        if not self.banner_identified():
            return 0.0
        return -math.tan(self._bearing_angle()) / math.tan(self.hfov / 2.0)

    # ---- what the lidar reports ---- #
    def surface_ahead(self, bearing_rad, half_width_rad,
                      expected_range_m=None, **kw):
        self.surface_calls.append((bearing_rad, half_width_rad,
                                   expected_range_m))
        self.sector_vs_camera.append((bearing_rad, self.banner_bearing()))
        if self.lidar_blind:
            return no_surface("no flat face in the sector")
        if self._pos[2] + self.LIDAR_OFFSET_M > self.gate_top_m:
            return no_surface(
                f"only 0 lidar return(s) inside the sector; the scan plane at "
                f"{self._pos[2] + self.LIDAR_OFFSET_M:.2f} m is above "
                f"everything in the arena")
        if self.standoff() <= 0.0:
            return no_surface("the aircraft is behind the face")
        # The FOOT of the perpendicular, in the aircraft frame.
        alpha = self._wrap(self.face_rad + math.pi - self._yaw)
        if abs(self._wrap(alpha - bearing_rad)) > half_width_rad:
            return no_surface(
                f"the face lies {math.degrees(alpha):+.0f} deg off the nose, "
                f"outside the sector the camera named")
        return {"ok": True, "angle_rad": alpha, "range_m": self.standoff(),
                "points": 44, "residual_m": 0.005, "extent_m": 3.6,
                "reason": ""}


class SquareOnWithTheLidarTests(unittest.TestCase):
    """Centred is not square on, and the lidar is what tells them apart.

    THE OPERATOR'S REPORT: "rn it is just flying out of the world ... it should
    only set the waypoint 10 metres ahead of it when it is completely in front
    of the banner while it maintains some distance".

    The aspect ratio this replaces plateaued at 1.88-1.91 because the derived
    box includes the gate posts, so no threshold above that was reachable and
    every threshold below it fired on noise. Fourteen watched runs.
    """

    class Sticky(GateMav):
        """A face the aircraft can never come square to.

        The measurement is always 30 degrees off however the aircraft turns,
        which is a geometry no manoeuvre resolves. The point is that the stage
        gives up on its own step budget rather than hovering until the battery
        runs down -- so the fake keeps the surface visible throughout, instead
        of letting the aircraft spin itself out of its own search sector and
        fail for a different reason.
        """

        def surface_ahead(self, bearing_rad, half_width_rad, **kw):
            self.surface_calls.append((bearing_rad, half_width_rad, None))
            self.sector_vs_camera.append((bearing_rad, self.banner_bearing()))
            return {"ok": True, "angle_rad": math.radians(30.0),
                    "range_m": self.standoff(), "points": 44,
                    "residual_m": 0.005, "extent_m": 3.6, "reason": ""}

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.4)
        kw.setdefault("align_dwell_s", 0.2)
        kw.setdefault("hfov_rad", mav.hfov)
        kw.setdefault("alt_floor_m", 2.0)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def _fly(self, mav, ticks=4000, **kw):
        stage = self._stage(mav, **kw)
        return stage, run(stage, mav, self.clock, ticks=ticks)

    # ---- the measurement drives the manoeuvre ---- #
    def test_an_aircraft_already_square_finishes_without_moving(self):
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi)
        start = mav.pos()[:2]
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertLess(math.dist(mav.pos()[:2], start), 0.5,
                        "moved when it was already in front of the gate")

    def test_it_squares_up_from_the_PORT_side(self):
        """The gate faces west; the aircraft sits north of its centreline."""
        mav = GateMav(gate=(5.0, -3.0), face_rad=math.pi)
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        self.assertLess(abs(math.degrees(mav._wrap(mav.face_rad + math.pi
                                                   - mav._yaw))), 8.0,
                        "finished pointing somewhere other than at the face")
        self.assertLess(abs(mav.off_centreline()), 1.0,
                        f"finished {mav.off_centreline():+.1f} m off the "
                        f"gate's centreline")

    def test_it_squares_up_from_the_STARBOARD_side(self):
        """The mirror image. A sign error passes one of these and not both."""
        mav = GateMav(gate=(5.0, 3.0), face_rad=math.pi)
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        self.assertLess(abs(mav.off_centreline()), 1.0,
                        f"finished {mav.off_centreline():+.1f} m off the "
                        f"gate's centreline")

    def test_a_gate_turned_away_from_the_approach_is_still_squared_up_to(self):
        """The gate's face is 25 degrees off the direction the aircraft came
        from, which is the case the aspect ratio could never distinguish from
        being square."""
        mav = GateMav(gate=(6.0, 1.0), face_rad=math.pi - math.radians(25.0))
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        off = math.degrees(mav._wrap(mav.face_rad + math.pi - mav._yaw))
        self.assertLess(abs(off), 8.0,
                        f"finished {off:+.0f} deg off perpendicular")

    def test_the_STANDOFF_is_held_while_the_aircraft_comes_round(self):
        """"while it maintains some distance". An orbit that closes the
        distance is an approach, and the approach is a later stage."""
        mav = GateMav(gate=(5.0, 3.0), face_rad=math.pi)
        opening = mav.standoff()
        stage = self._stage(mav)
        seen = []
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage.phase is stage.SQUARE:
                seen.append(mav.standoff())
        self.assertTrue(seen)
        self.assertGreater(min(seen), opening - 2.0,
                           "closed the distance to the gate while squaring up")

    def test_a_standoff_outside_the_sensors_band_is_corrected(self):
        """Drifting out to the edge of the lidar's range loses the very
        measurement the stage depends on."""
        mav = GateMav(gate=(11.0, 0.0), face_rad=math.pi)
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        self.assertLessEqual(mav.standoff(), 6.5,
                             "never closed to a range the lidar works at")

    # ---- the refusal ---- #
    def test_a_CONFIDENT_CAMERA_does_not_advance_a_blind_lidar(self):
        """The headline rule: the lidar wins. A camera reporting the banner
        dead ahead is not evidence of perpendicularity, and acting on it is
        what put the aircraft outside the arena."""
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi, lidar_blind=True)
        stage, status = self._fly(mav, ticks=6000)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertTrue(mav.banner_identified(),
                        "the camera was meant to be confident throughout")

    def test_the_refusal_names_where_it_stood_and_what_it_saw(self):
        """"No banner found" is not a diagnosis, and neither is a list of
        headings when the problem was the position. The give-up message names
        every vantage point tried and the best frame at each."""
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi, lidar_blind=True)
        self._fly(mav, ticks=9000)
        self.assertIn("vantage point", mav.abort_reason)
        self.assertIn("no flat face", mav.abort_reason)
        self.assertRegex(mav.abort_reason, r"\(-?\d+\.\d+, -?\d+\.\d+",
                         "the message does not say where the aircraft stood")

    def test_a_momentarily_lost_camera_does_not_stop_the_lidar(self):
        """The camera says WHERE to look and the lidar says what is there. A
        dropped frame or two must not throw away a measurement the lidar is
        making perfectly well."""
        mav = GateMav(gate=(5.0, 1.5), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage.phase is stage.SQUARE and stage._sq_steps >= 1:
                mav.visible = False       # detector drops it mid-square-up
                break
        status = run(stage, mav, self.clock, ticks=200)
        self.assertIsNot(status, py_trees.common.Status.FAILURE,
                         "a dropped camera frame ended a working measurement")

    def test_the_sector_searched_FOLLOWS_the_camera_bearing(self):
        """A fixed forward sector would measure the corridor wall behind an
        open gate rather than the gate. The sector the stage asks for has to
        be the one the camera is pointing at, every time it asks."""
        mav = GateMav(gate=(5.0, 4.0), face_rad=math.pi)
        self._fly(mav)
        self.assertTrue(mav.sector_vs_camera)
        off = [(s, b) for s, b in mav.sector_vs_camera
               if abs(s - bearing_to_angle(b, mav.hfov)) > 1e-6 and b]
        self.assertEqual(off, [], f"the sector did not follow the camera: "
                                  f"{off[:3]}")

    def test_an_OFF_CENTRE_banner_is_searched_for_off_the_nose(self):
        """The invariant above is satisfied trivially if the bearing is always
        zero. This is the case where it is not."""
        mav = GateMav(gate=(5.0, 4.0), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage.phase is stage.SQUARE:
                break
        mav._yaw = mav._wrap(mav._yaw + math.radians(20.0))   # knocked off
        mav.sector_vs_camera = []
        run(stage, mav, self.clock, ticks=400)
        self.assertTrue(mav.sector_vs_camera)
        self.assertGreater(max(abs(s) for s, _ in mav.sector_vs_camera),
                           math.radians(5.0),
                           "kept searching straight ahead with the banner "
                           "20 degrees off the nose")

    def test_the_measurement_is_taken_from_a_STOPPED_aircraft(self):
        """Three orbit steps in a row once reported an identical aspect: the
        stage moved its target and re-read the view in the same breath, so it
        was measuring the old position every time."""
        mav = GateMav(gate=(5.0, 3.0), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage.phase is stage.SQUARE and stage._sq_phase is stage.MEASURE:
                ax, ay, az = stage._anchor
                self.assertTrue(
                    mav.reached(ax, ay, az, 1.2),
                    f"measured from {mav.pos()} while still flying to "
                    f"{(ax, ay, az)}")

    # ---- the scan plane ---- #
    def test_it_DESCENDS_when_the_scan_plane_is_above_the_gate(self):
        """MEASURED on seed 1001: 51 consecutive samples in BANNER_ALIGN at
        5.0 m returned 0 finite ranges out of 720, and the same sensor at
        3.0 m returned 289. The lidar was not broken and the gate was not
        missing -- the scan plane was over the top of it."""
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 5.0), gate_top_m=4.0)
        stage, status = self._fly(mav, ticks=6000)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        self.assertLess(mav.pos()[2], 4.0 - GateMav.LIDAR_OFFSET_M,
                        "never got the scan plane below the top of the gate")

    def test_the_camera_is_POINTED_FORWARD_on_the_way_down(self):
        """MEASURED, and it is the root cause of run 16 rather than the
        altitude everyone blamed.

        The BANNER pose looks 20 degrees below the horizon because from 5 m a
        gate a few metres ahead sits under a level camera. Once the aircraft
        has descended to gate height that same pose puts the board out of the
        TOP of the frame -- the detector went from 10/10 frames at 5.0 m to
        refusing almost every frame at 3.0 m, and the stage was left with a
        working lidar and no idea which sector to search.

        Taken with the camera FORWARD, every cell of a nine-point grid from
        2.5 to 3.5 m altitude and 3.5 to 6.9 m standoff identifies at 100%.
        The band was never narrow; the pointing was wrong.
        """
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 5.0), gate_top_m=4.0)
        self._fly(mav, ticks=6000)
        self.assertIn("FORWARD", mav.camera_poses,
                      "descended to gate height with the camera still "
                      "pitched for a search from 5 m")

    def test_the_camera_is_left_alone_when_no_descent_was_needed(self):
        """A stage that re-points the camera it was handed, unprompted, is a
        stage that will fight the leaf that pointed it."""
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 3.0), gate_top_m=4.0)
        self._fly(mav)
        self.assertEqual(mav.camera_poses, [],
                         "re-pointed the camera without descending")

    def test_the_descent_stops_at_the_floor_and_then_FAILS_CLOSED(self):
        """The ladder is not a licence to fly into the ground looking for a
        surface that is not there."""
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 5.0), gate_top_m=0.5)
        stage, status = self._fly(mav, ticks=8000, alt_floor_m=3.0)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertGreaterEqual(mav.pos()[2], 2.8,
                                "descended through its own floor")

    def test_it_does_not_descend_when_the_lidar_can_already_see(self):
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 3.0), gate_top_m=4.0)
        stage, status = self._fly(mav)
        self.assertIs(status, py_trees.common.Status.SUCCESS)
        self.assertAlmostEqual(mav.pos()[2], 3.0, delta=0.2,
                               msg="descended for no reason")

    # ---- one action at a time ---- #
    def test_it_never_TURNS_and_TRANSLATES_in_the_same_command(self):
        """Rotating while translating reintroduces the coupling that made yaw
        useless: the centroid shift from the rotation is indistinguishable
        from the one the translation is trying to measure."""
        mav = GateMav(gate=(6.0, 3.0), face_rad=math.pi - math.radians(20.0))
        stage = self._stage(mav)
        for _ in range(4000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if len(mav.gotos) >= 2:
                a, b = mav.gotos[-2], mav.gotos[-1]
                moved = (a[0], a[1]) != (b[0], b[1])
                turned = abs(a[3] - b[3]) > 1e-9
                self.assertFalse(moved and turned,
                                 f"translated and rotated at once: {a} -> {b}")

    def test_a_hopeless_geometry_gives_up_within_its_step_budget(self):
        """A stage that hovers until the battery runs down has not failed
        safely, it has just failed later."""
        mav = self.Sticky(gate=(5.0, 0.0), face_rad=math.pi)
        stage, status = self._fly(mav, ticks=8000, max_square_steps=6)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertIn("square", mav.abort_reason)

    def test_the_give_up_message_states_the_angle_AND_the_range(self):
        mav = self.Sticky(gate=(5.0, 0.0), face_rad=math.pi)
        self._fly(mav, ticks=8000, max_square_steps=4)
        self.assertIn("deg off perpendicular", mav.abort_reason)
        self.assertIn("m standoff", mav.abort_reason)

    # ---- what the operator sees ---- #
    def test_the_angle_and_standoff_are_reported_while_it_converges(self):
        mav = GateMav(gate=(5.0, 3.0), face_rad=math.pi)
        self._fly(mav)
        self.assertTrue(mav.square_on, "nothing was published for the GCS")
        good = [p for p in mav.square_on if p["ok"]]
        self.assertTrue(good)
        for p in good:
            self.assertIsNotNone(p["angle_deg"])
            self.assertIsNotNone(p["standoff_m"])

    def test_the_success_line_carries_the_measured_angle(self):
        """A run artifact has to say how square it thought it was."""
        mav = GateMav(gate=(5.0, 2.0), face_rad=math.pi)
        self._fly(mav)
        lines = [m for m, _ in mav.logs if "SQUARE ON" in m]
        self.assertTrue(lines, mav.logs)
        self.assertIn("off perpendicular", lines[-1])
        self.assertIn("standoff", lines[-1])


class CentringIsACameraJobTests(unittest.TestCase):
    """Turning to face the banner must never wait on the lidar.

    WATCHED LIVE, run 16 onward: the aircraft held station, CONTINUING TO
    DETECT the banner the whole time, and never turned toward it. Squareness
    had been made a precondition of everything behind it, so a lidar that
    could not confirm perpendicularity also stopped the aircraft doing the one
    thing it plainly could do -- point its nose at a board it could see.

    Fail-closed is right for committing a waypoint through the gate. It is
    wrong for turning to look at something.

        Which way do I turn?   CAMERA. Box midpoint against frame midpoint.
        Am I perpendicular?    LIDAR.
        May I advance?         Perpendicular within tolerance AND in range.
    """

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.4)
        kw.setdefault("align_dwell_s", 0.2)
        kw.setdefault("hfov_rad", mav.hfov)
        kw.setdefault("alt_floor_m", 2.0)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_a_DEAD_LIDAR_does_not_stop_the_aircraft_facing_the_banner(self):
        """The defect, stated as the operator saw it. The camera is confident
        and the banner is well off to one side; the aircraft must turn."""
        mav = GateMav(gate=(6.0, 3.5), face_rad=math.pi, lidar_blind=True)
        opening = abs(mav._bearing_angle())
        stage = self._stage(mav)
        for _ in range(1500):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if abs(mav._bearing_angle()) < math.radians(6.0):
                break
        self.assertLess(abs(mav._bearing_angle()), opening / 2.0,
                        f"held still while looking at a banner "
                        f"{math.degrees(opening):.0f} deg off the nose")

    def test_a_dead_lidar_still_REFUSES_to_advance(self):
        """Both halves matter. Centring without the lidar is required;
        advancing without it is forbidden."""
        mav = GateMav(gate=(6.0, 3.5), face_rad=math.pi, lidar_blind=True)
        stage = self._stage(mav)
        self.assertIs(run(stage, mav, self.clock, ticks=9000),
                      py_trees.common.Status.FAILURE)

    def test_a_banner_ALREADY_in_frame_is_not_swept_for(self):
        """The unexplained yaw and roll at entry was the zigzag searching for
        a board that was already there. Look first."""
        mav = GateMav(gate=(6.0, 0.0), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(20):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
        self.assertIsNot(stage.phase, stage.SETTLE,
                         "swept for a banner that was already identified")
        self.assertIsNot(stage.phase, stage.DWELL)
        turns = {round(g[3], 6) for g in mav.gotos}
        self.assertLessEqual(len(turns), 2,
                             f"commanded a sweep of headings {turns}")

    def test_the_SAME_state_always_produces_the_SAME_command(self):
        """"as soon as it detects the banner it should not be confused and it
        should not try to do different things." One behaviour per state."""
        commands = []
        for _ in range(3):
            clock = Clock()
            mav = GateMav(gate=(6.0, 2.5), face_rad=math.pi)
            stage = self._stage(mav, clock=clock)
            for _ in range(12):
                stage.update()
                clock.advance(0.1)
            commands.append([tuple(round(v, 6) for v in g) for g in mav.gotos])
        self.assertEqual(commands[0], commands[1])
        self.assertEqual(commands[1], commands[2])


class RecoverTheBannerByMovingTests(unittest.TestCase):
    """A yaw sweep cannot fix a position error.

    WATCHED LIVE, run 16: the aircraft descended to 3.0 m, lost the board, and
    sat at [0.3, 0.0, 3.0] sweeping headings. The detector's refusals name the
    reason it could not be found from there:

        green region too small (5262 px, need 8533)
        green region too small (3416 px, need 8533)
        green region too small (1930 px, need 8533)

    From that position no heading revealed the banner, so rotating through all
    of them could not have worked. The zigzag is the inner loop; the outer
    loop moves.
    """

    def setUp(self):
        self.clock = Clock()

    def _stage(self, mav, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.3)
        kw.setdefault("align_dwell_s", 0.2)
        kw.setdefault("hfov_rad", mav.hfov)
        kw.setdefault("alt_floor_m", 2.0)
        stage = AlignToBanner(mav, **kw)
        stage.initialise()
        return stage

    def test_when_no_heading_reveals_the_banner_the_aircraft_TRANSLATES(self):
        """The case that pins the defect. A yaw-only implementation fails it,
        which is the point of writing it."""
        mav = GateMav(gate=(9.0, 0.0), face_rad=math.pi,
                      max_ident_range_m=6.0)
        start = mav.pos()[:2]
        stage = self._stage(mav)
        moved = 0.0
        for _ in range(6000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            moved = max(moved, math.dist(mav.pos()[:2], start))
            if moved > 1.5:
                break
        self.assertGreater(moved, 1.5,
                           "swept every heading from one spot and never moved")

    def test_closing_range_recovers_a_board_that_was_too_small(self):
        mav = GateMav(gate=(9.0, 0.0), face_rad=math.pi,
                      max_ident_range_m=6.5)
        stage = self._stage(mav)
        status = run(stage, mav, self.clock, ticks=9000)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)

    def test_climbing_recovers_a_board_lost_on_the_way_down(self):
        mav = GateMav(gate=(6.0, 0.0), face_rad=math.pi,
                      start=(0.0, 0.0, 2.2), min_ident_alt_m=3.0,
                      gate_top_m=4.0)
        stage = self._stage(mav, alt_floor_m=2.0)
        status = run(stage, mav, self.clock, ticks=9000)
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      stage.feedback_message)
        self.assertGreaterEqual(mav.pos()[2], 2.9,
                                "never climbed back to where it could see")

    def test_it_returns_to_the_pose_the_banner_was_last_seen_from(self):
        """A position that demonstrably worked beats any search pattern, and
        it is one setpoint away.

        Driven directly rather than through a scenario: the priority rule is
        "the remembered pose FIRST", and a scenario test can only show that
        some recovery happened, not that the right one was tried first.
        """
        mav = GateMav(gate=(6.0, 0.0), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(3000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage._good_vantage is not None:
                break
        self.assertIsNotNone(stage._good_vantage,
                             "never recorded where it saw the banner from")
        good = stage._good_vantage

        stage._relocate("the banner went out of view")
        self.assertAlmostEqual(stage._anchor[0], good[0], places=3)
        self.assertAlmostEqual(stage._anchor[1], good[1], places=3)
        self.assertAlmostEqual(stage._anchor[2], good[2], places=3)
        self.assertTrue([m for m, _ in mav.logs
                         if "where the banner was last identified" in m],
                        "the return was not reported")

    def test_the_remembered_pose_is_only_tried_ONCE(self):
        """If going back there did not work, going back there again will not
        either. The pattern has to take over."""
        mav = GateMav(gate=(6.0, 0.0), face_rad=math.pi)
        stage = self._stage(mav)
        for _ in range(3000):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            self.clock.advance(0.1)
            if stage._good_vantage is not None:
                break
        good = stage._good_vantage
        stage._relocate("first loss")
        stage._relocate("second loss")
        self.assertNotEqual(
            (round(stage._anchor[0], 3), round(stage._anchor[1], 3)),
            (round(good[0], 3), round(good[1], 3)),
            "went back to the same pose twice instead of searching")

    def test_it_gives_up_inside_its_bound_and_says_where_it_stood(self):
        mav = GateMav(gate=(60.0, 0.0), face_rad=math.pi,
                      max_ident_range_m=1.0, lidar_blind=True)
        stage = self._stage(mav, max_relocations=3)
        status = run(stage, mav, self.clock, ticks=20000)
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertIn("vantage point", mav.abort_reason)
        self.assertLessEqual(stage._relocations, 3)


class TheAspectGateIsGoneTests(unittest.TestCase):
    """It is DELETED, not demoted.

    A fallback that fires on bad data is how the aircraft flew out of the
    world: a single noisy narrowing at aspect 1.4 satisfied the peak
    detector and the aircraft latched a 10 m waypoint while badly off-axis.
    Keeping the old path "just in case" keeps that failure reachable.
    """

    def setUp(self):
        self.clock = Clock()

    def test_a_perfect_aspect_ratio_cannot_substitute_for_a_measurement(self):
        mav = GateMav(gate=(5.0, 0.0), face_rad=math.pi, lidar_blind=True)
        mav.banner_board_aspect = 3.6         # as square as a board ever looks
        mav.banner_aspect = lambda: 3.6
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.4,
                              align_dwell_s=0.2, hfov_rad=mav.hfov,
                              alt_floor_m=2.0)
        stage.initialise()
        self.assertIs(run(stage, mav, self.clock, ticks=6000),
                      py_trees.common.Status.FAILURE)

    def test_the_stage_has_no_aspect_knobs_left_to_turn(self):
        """These parameters named a measurement that could not answer the
        question. A stage that still accepts them still has the code."""
        import inspect
        args = inspect.signature(AlignToBanner.__init__).parameters
        for gone in ("min_square_aspect", "peak_min_aspect", "square_gain",
                     "max_strafes", "max_strafes_square", "stall_before_strafe"):
            self.assertNotIn(gone, args, f"{gone} survived the rewrite")

    def test_the_aspect_may_be_REPORTED_but_never_changes_the_outcome(self):
        """The box aspect is kept as a cross-check on the lidar -- a board is
        genuinely widest seen face-on -- and is logged beside the measured
        angle so a persistent disagreement between the two instruments is
        visible. What it must never do is change what the aircraft does.

        A source grep used to stand in for this. It could not tell reporting
        from steering, and it forbade the cross-check the operator asked for.
        """
        outcomes = []
        for aspect in (0.4, 3.6):
            clock = Clock()
            mav = GateMav(gate=(5.0, 2.0), face_rad=math.pi)
            mav.banner_aspect = lambda a=aspect: a
            stage = AlignToBanner(mav, clock=clock, dwell_s=0.4,
                                  align_dwell_s=0.2, hfov_rad=mav.hfov,
                                  alt_floor_m=2.0)
            stage.initialise()
            outcomes.append((run(stage, mav, clock, ticks=4000),
                             round(mav.off_centreline(), 1)))
        self.assertEqual(outcomes[0], outcomes[1],
                         f"the board aspect changed the outcome: {outcomes}")

    def test_the_aspect_is_carried_in_the_report_for_comparison(self):
        mav = GateMav(gate=(5.0, 2.0), face_rad=math.pi)
        mav.banner_aspect = lambda: 1.9
        stage = AlignToBanner(mav, clock=self.clock, dwell_s=0.4,
                              align_dwell_s=0.2, hfov_rad=mav.hfov,
                              alt_floor_m=2.0)
        stage.initialise()
        run(stage, mav, self.clock, ticks=4000)
        self.assertTrue(mav.square_on)
        self.assertIn("box_aspect", mav.square_on[-1],
                      "the camera cross-check is not reported beside the "
                      "lidar angle, so a disagreement would be silent")


if __name__ == "__main__":
    unittest.main()
