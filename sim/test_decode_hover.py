#!/usr/bin/env python3
"""Every distinct decoded marker gets a visible pause over it.

WHAT THIS REPLACES

    Nothing. The mission decoded a marker and moved on in the same tick, so a
    decode was something you found in a log afterwards rather than something
    you watched happen. The operator asked to "hover on the qr for 5 seconds
    even after scanning it", on every marker decoded.

WHAT THESE TESTS ARE CAREFUL ABOUT

    They assert on the setpoints commanded during the hover, not on a flag
    saying a hover is in progress. A hover that sets a flag and keeps flying
    is the same class of defect as an altitude hold that computes a correction
    and publishes zero.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_decode_hover.py -v
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
from mission_bt.decode_hover import DecodeHover              # noqa: E402
from test_fail_closed_stages import FakeMav                  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class HoverMav(FakeMav):
    def __init__(self, at=(4.0, -2.0, 8.0), yaw=0.7):
        super().__init__()
        self._pos = at
        self._alt = at[2]
        self._yaw = yaw

    def goto(self, *a, **k):
        self.gotos.append(a)


class DecodeHoverTests(unittest.TestCase):

    def setUp(self):
        self.clock = Clock()
        self.mav = HoverMav()
        self.h = DecodeHover(hover_s=5.0, clock=self.clock)

    def test_no_decode_means_no_hover(self):
        self.assertFalse(self.h.tick(self.mav))
        self.assertEqual(self.mav.gotos, [])

    def test_a_new_payload_starts_a_hover(self):
        self.mav.qr_decoded = "PAD_A"
        self.assertTrue(self.h.tick(self.mav))

    def test_the_hover_actually_commands_the_current_position(self):
        """The assertion that matters. A hover that returns True and commands
        nothing lets the caller's own setpoint keep flying."""
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.assertEqual(self.mav.gotos[-1][:3], (4.0, -2.0, 8.0))

    def test_EVERY_tick_of_the_hover_commands_the_hold(self):
        """Not just the first one.

        A mutation that commanded the hold on the opening tick and nothing
        afterwards passed every other test in this file. The continuation is
        where a hover actually lives -- one setpoint at the start decays into
        whatever the position controller drifts to over five seconds, and the
        caller's own setpoint from the previous tick is still the last thing
        the vehicle was told.
        """
        self.mav.qr_decoded = "PAD_A"
        for _ in range(30):
            self.assertTrue(self.h.tick(self.mav))
            self.assertEqual(self.mav.gotos[-1][:3], (4.0, -2.0, 8.0))
            self.clock.advance(0.1)
        self.assertEqual(len(self.mav.gotos), 30,
                         "the hover skipped ticks it claimed to be holding on")

    def test_the_hover_holds_the_heading_too(self):
        """Yawing during a hover swings the marker out of frame, which is the
        opposite of letting the operator look at it."""
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.assertAlmostEqual(self.mav.gotos[-1][3], 0.7)

    def test_it_holds_for_the_configured_time(self):
        self.mav.qr_decoded = "PAD_A"
        for _ in range(49):
            self.assertTrue(self.h.tick(self.mav))
            self.clock.advance(0.1)
        self.clock.advance(0.2)
        self.assertFalse(self.h.tick(self.mav))

    def test_the_hold_point_does_not_drift_with_the_aircraft(self):
        """Latched at the start. Re-reading pos() each tick would make the
        hold chase whatever error the position controller has left."""
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.mav._pos = (9.0, 9.0, 9.0)
        self.clock.advance(1.0)
        self.h.tick(self.mav)
        self.assertEqual(self.mav.gotos[-1][:3], (4.0, -2.0, 8.0))

    # ---- once per DISTINCT payload ---- #
    def test_the_same_payload_does_not_hover_twice(self):
        """A marker in frame decodes every frame. Hovering per decode would
        hold the aircraft there until the mission clock ran out."""
        self.mav.qr_decoded = "PAD_A"
        for _ in range(60):
            self.h.tick(self.mav)
            self.clock.advance(0.2)
        self.assertFalse(self.h.tick(self.mav))

    def test_a_different_payload_hovers_again(self):
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.clock.advance(6.0)
        self.h.tick(self.mav)
        self.mav.qr_decoded = "PAD_B"
        self.assertTrue(self.h.tick(self.mav))

    def test_every_distinct_payload_is_hovered_on(self):
        """"every qr decoded", not only the one that matches."""
        hovered = []
        for p in ("PAD_A", "PAD_B", "PAD_C"):
            self.mav.qr_decoded = p
            if self.h.tick(self.mav):
                hovered.append(self.h.payload)
            self.clock.advance(6.0)
            self.h.tick(self.mav)
        self.assertEqual(hovered, ["PAD_A", "PAD_B", "PAD_C"])

    def test_resetting_a_stage_does_not_re_hover_old_markers(self):
        """The sweep is re-entered after a descent. Hovering again on the
        marker that caused the descent would loop."""
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.clock.advance(6.0)
        self.h.tick(self.mav)
        self.h.reset()
        self.assertFalse(self.h.tick(self.mav))

    def test_a_new_mission_hovers_again(self):
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.h.forget()
        self.assertTrue(self.h.tick(self.mav))

    def test_it_can_be_switched_off(self):
        off = DecodeHover(hover_s=5.0, clock=self.clock, enabled=False)
        self.mav.qr_decoded = "PAD_A"
        self.assertFalse(off.tick(self.mav))
        self.assertEqual(self.mav.gotos, [])

    def test_the_hover_is_logged_so_it_is_not_mistaken_for_a_hang(self):
        self.mav.qr_decoded = "PAD_A"
        self.h.tick(self.mav)
        self.assertTrue([m for m, _ in self.mav.logs if "PAD_A" in m])


class StageWiringTests(unittest.TestCase):
    """The hover has to happen inside the stages that read markers."""

    def setUp(self):
        self.clock = Clock()

    def test_ScanStartQR_holds_over_the_start_marker_after_decoding(self):
        """"it should hover on the qr for 5 second even after scanning it"."""
        from mission_bt.mission_tree import ScanStartQR
        mav = HoverMav()
        mav.qr_decoded = "TARGET_C"
        mav.qr_streak = 9
        stage = ScanStartQR(mav, confirm_frames=3, hover_s=5.0,
                            clock=self.clock)
        stage.initialise()
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING,
                      "moved on the instant it decoded")
        self.clock.advance(5.5)
        self.assertIs(stage.update(), py_trees.common.Status.SUCCESS)

    def test_ScanStartQR_still_sets_the_target_it_read(self):
        from mission_bt.mission_tree import ScanStartQR
        mav = HoverMav()
        mav.qr_decoded = "TARGET_C"
        mav.qr_streak = 9
        stage = ScanStartQR(mav, confirm_frames=3, hover_s=5.0,
                            clock=self.clock)
        stage.initialise()
        stage.update()
        self.clock.advance(5.5)
        stage.update()
        self.assertEqual(mav.target_set, "TARGET_C")

    def test_a_confirmed_decode_survives_a_blurred_frame_during_the_hover(self):
        """A regression the hover itself introduced, found in review.

        Before the hover, a confident decode returned SUCCESS in the same
        tick, so there was no window in which it could be un-made. Holding
        station for five seconds opens one: the streak resets on a blurred
        frame, the stage falls back to "still waiting", and the timeout then
        fails the mission with "start QR not decoded" -- about a QR it had
        decoded five seconds earlier and was at that moment hovering over.
        """
        from mission_bt.mission_tree import ScanStartQR
        mav = HoverMav()
        mav.qr_decoded = "TARGET_C"
        mav.qr_streak = 9
        stage = ScanStartQR(mav, timeout_ticks=20, confirm_frames=3,
                            hover_s=5.0, clock=self.clock)
        stage.initialise()
        stage.update()
        self.clock.advance(0.1)
        mav.qr_streak = 0                       # one blurred frame
        status = None
        for _ in range(40):
            status = stage.update()
            self.clock.advance(0.2)
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.SUCCESS,
                      f"lost a confirmed decode: {mav.abort_reason}")
        self.assertEqual(mav.target_set, "TARGET_C")

    def test_an_unconfirmed_decode_still_times_out(self):
        """The latch must not make the fail-closed timeout unreachable --
        that timeout is the headline defect this whole stage exists for."""
        from mission_bt.mission_tree import ScanStartQR
        mav = HoverMav()
        mav.qr_decoded = ""
        stage = ScanStartQR(mav, timeout_ticks=5, confirm_frames=3,
                            hover_s=5.0, clock=self.clock)
        stage.initialise()
        status = None
        for _ in range(20):
            status = stage.update()
            self.clock.advance(0.1)
            if status is not py_trees.common.Status.RUNNING:
                break
        self.assertIs(status, py_trees.common.Status.FAILURE)

    def test_ScanStartQR_does_not_hover_on_an_operator_override(self):
        """There is no marker to look at; the operator typed it."""
        from mission_bt.mission_tree import ScanStartQR
        mav = HoverMav()
        mav.target_override = "MANUAL_B"
        stage = ScanStartQR(mav, hover_s=5.0, clock=self.clock)
        stage.initialise()
        self.assertIs(stage.update(), py_trees.common.Status.SUCCESS)

    def test_the_sweep_pauses_over_a_marker_it_passes_over(self):
        from mission_bt.mission_tree import LawnmowerSearch
        mav = HoverMav(at=(0.0, 0.0, 10.0), yaw=0.0)
        mav.observed_zone = (-5.0, 40.0, -20.0, 20.0)
        mav.corridor_exit_pose = (0.0, 0.0, 10.0, 0.0)
        stage = LawnmowerSearch(mav, zone=lambda: mav.observed_zone,
                                image_width_px=1280, hfov_rad=math.radians(60),
                                marker_m=2.2, alt=10.0, hover_s=5.0,
                                clock=self.clock)
        stage.initialise()
        stage.update()
        n = len(mav.gotos)
        mav.qr_decoded = "PAD_B"
        stage.update()
        self.assertEqual(mav.gotos[-1][:3], mav.pos(),
                         "the sweep flew on past a marker it had just read")
        self.assertGreater(len(mav.gotos), n)

    def test_the_sweep_resumes_after_the_pause(self):
        from mission_bt.mission_tree import LawnmowerSearch
        mav = HoverMav(at=(0.0, 0.0, 10.0), yaw=0.0)
        mav.observed_zone = (-5.0, 40.0, -20.0, 20.0)
        mav.corridor_exit_pose = (0.0, 0.0, 10.0, 0.0)
        stage = LawnmowerSearch(mav, zone=lambda: mav.observed_zone,
                                image_width_px=1280, hfov_rad=math.radians(60),
                                marker_m=2.2, alt=10.0, hover_s=5.0,
                                clock=self.clock)
        stage.initialise()
        mav.qr_decoded = "PAD_B"
        stage.update()
        self.clock.advance(5.5)
        stage.update()
        self.assertNotEqual(mav.gotos[-1][:3], mav.pos(),
                            "still parked after the hover expired")


if __name__ == "__main__":
    unittest.main()
