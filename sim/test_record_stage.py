#!/usr/bin/env python3
"""The stage gate on the in-flight recorder, which is the whole point of it.

An ungated probe of this exact subsystem produced a confident wrong answer:
it sampled while the aircraft was on the pad, read "altitude too low to
project", and very nearly became a fourth retracted diagnosis. The gate is
what makes a recording evidence about a stage rather than about whenever the
probe happened to look.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_record_stage.py -v
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_mission", "mission_bt"))

from record_stage import Recorder, _clearance_breaches   # noqa: E402


class _Gate(Recorder):
    """Just the gating logic, without a ROS node underneath it."""

    def __init__(self, stage):
        self.stage = stage
        self.s = {}


class StageGateTests(unittest.TestCase):

    def test_a_sample_outside_the_stage_is_refused(self):
        g = _Gate("SEARCH_QR")
        g.s["state"] = "LANDED"
        self.assertFalse(g.in_stage())

    def test_a_sample_inside_the_stage_is_taken(self):
        g = _Gate("SEARCH_QR")
        g.s["state"] = "SEARCH_QR"
        self.assertTrue(g.in_stage())

    def test_an_unknown_state_is_refused_not_assumed(self):
        """No mission state yet is not "probably the right one"."""
        self.assertFalse(_Gate("SEARCH_QR").in_stage())

    def test_ANY_is_an_explicit_opt_out_not_the_default(self):
        g = _Gate("ANY")
        g.s["state"] = "LANDED"
        self.assertTrue(g.in_stage())

    def test_the_stage_argument_has_no_default(self):
        """It has to be impossible to run this without saying what state the
        answer is about. A default would restore the ungated probe."""
        import inspect
        import record_stage
        src = inspect.getsource(record_stage.main)
        self.assertIn('"--stage", required=True', src)


class BreachDetectionTests(unittest.TestCase):
    """What the red-zone claim actually rests on: did the AIRFRAME enter one."""

    EX = [(10.0, 20.0, -5.0, 5.0)]

    def test_a_track_through_the_zone_is_reported(self):
        track = [(0.0, 0.0, 10.0), (15.0, 0.0, 10.0), (30.0, 0.0, 10.0)]
        self.assertEqual(len(_clearance_breaches(track, self.EX, 1.5)), 1)

    def test_a_track_around_the_zone_is_clean(self):
        track = [(0.0, 0.0, 10.0), (15.0, 9.0, 10.0), (30.0, 0.0, 10.0)]
        self.assertEqual(_clearance_breaches(track, self.EX, 1.5), [])

    def test_the_clearance_band_counts_as_a_breach(self):
        """Skimming the edge with a 1.5 m airframe is not staying out."""
        track = [(15.0, 5.8, 10.0)]
        self.assertTrue(_clearance_breaches(track, self.EX, 1.5))

    def test_altitude_does_not_excuse_a_breach(self):
        """A restricted zone is ground the aircraft may not fly OVER."""
        track = [(15.0, 0.0, 40.0)]
        self.assertTrue(_clearance_breaches(track, self.EX, 1.5))

    def test_no_exclusions_means_no_breaches(self):
        self.assertEqual(_clearance_breaches([(15.0, 0.0, 10.0)], [], 1.5), [])


if __name__ == "__main__":
    unittest.main()
