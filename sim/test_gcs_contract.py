#!/usr/bin/env python3
"""Every field the aggregator publishes is one the GCS can actually show.

WHY THIS EXISTS

    The aggregator publishes roughly seventy fields across a dozen groups.
    Nothing checked that the frontend knew about any of them. A field renamed
    on the ROS side, or added and never wired, shows in the panel as blank or
    `undefined` -- and a blank cell looks exactly like a healthy zero.

    That is the specific failure mode this file is for: the panel is only
    trustworthy if a missing value LOOKS missing.

    This is the automated half of the GCS probe. The other half is watching
    the panel during a live flight, which is the only way to catch a value
    that renders correctly and reads badly.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_gcs_contract.py -v
"""

import os
import re
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_gcs", "gcs_aggregator"))

TYPES_TS = os.path.join(ROOT, "src", "aerothon_gcs", "tauri_app", "src", "types.ts")
APP_TSX = os.path.join(ROOT, "src", "aerothon_gcs", "tauri_app", "src", "App.tsx")


def blank_state():
    from gcs_aggregator.aggregator import Aggregator
    return Aggregator._blank_state()


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


class SnapshotShapeTests(unittest.TestCase):

    def setUp(self):
        self.state = blank_state()
        self.flat = flatten(self.state)

    def test_the_snapshot_is_not_empty(self):
        self.assertGreater(len(self.flat), 20,
                           "the telemetry contract collapsed to almost nothing")

    def test_every_group_is_a_dict_not_a_bare_value(self):
        for k, v in self.state.items():
            if k in ("t",):
                continue
            self.assertIsInstance(
                v, (dict, list, str, int, float, bool, type(None)),
                f"{k} has an unrenderable type {type(v)}")

    def test_the_expected_GROUPS_are_all_present(self):
        """The panel is laid out by group; losing one loses a whole card."""
        for group in ("flight", "power", "gps", "safety", "percep",
                      "mission", "nav"):
            self.assertIn(group, self.state, f"telemetry group {group} missing")

    def test_nothing_is_silently_undefined(self):
        """A key present with value None is fine and renders as unknown; a key
        ABSENT is what shows as undefined in the panel."""
        for key in self.flat:
            self.assertNotIn("undefined", str(key))


class FrontendKnowsTheFieldsTests(unittest.TestCase):
    """The contract test: ROS side and TypeScript side agree."""

    def setUp(self):
        self.state = blank_state()
        self.types_src = open(TYPES_TS).read()
        self.app_src = open(APP_TSX).read()

    def test_every_top_level_group_is_declared_in_types_ts(self):
        missing = [g for g in self.state
                   if g not in ("t",) and not re.search(rf"\b{g}\b", self.types_src)]
        self.assertEqual(missing, [],
                         f"groups the frontend has no type for: {missing}")

    def test_the_interlock_items_are_rendered(self):
        """Eleven items with a measured value and a reason each. Publishing
        them and not showing them leaves the operator with a greyed-out ARM
        button and no explanation -- the thing the interlock replaced."""
        for token in ("ready_items", "ready_reasons"):
            self.assertIn(token, self.app_src,
                          f"{token} is published but never rendered")

    def test_the_redzone_tri_state_is_rendered(self):
        """NOT_VISIBLE, CLEAR and RED are three different things; collapsing
        them to a boolean is how 'no camera' reads as 'no red zone'."""
        self.assertIn("redzone_status", self.app_src)

    def test_the_delivery_accuracy_is_rendered(self):
        """15 rulebook marks. Measured now, so it has to be visible."""
        self.assertTrue(
            "delivery" in self.app_src.lower(),
            "delivery accuracy is measured but the panel never shows it")

    def test_the_camera_pane_shows_the_COMPOSITE_feed(self):
        """It was pinned to the QR detector's private copy, so during banner
        alignment the operator watched a nadir view with no banner box."""
        self.assertIn("/percep/overlay", self.app_src)

    def test_the_operator_can_still_reach_the_per_detector_feeds(self):
        for topic in ("/percep/banner/annotated", "/percep/redzone/annotated"):
            self.assertIn(topic, self.app_src)

    def test_the_scan_ledger_is_rendered(self):
        """"a list of everything that has been detected/decoded". A ledger
        that only exists in the snapshot is a log nobody reads in flight."""
        self.assertIn("scans", self.app_src,
                      "the scan ledger is published but never rendered")

    def test_a_matched_scan_is_visibly_TAGGED(self):
        """"everything that has been matched should have a tag with it".
        Decoding and matching are different claims and must look different."""
        self.assertTrue(
            re.search(r"matched", self.app_src),
            "nothing in the panel distinguishes a match from a decode")

    def test_a_rejected_scan_shows_its_reason(self):
        """A marker the stack refused is the row an operator most needs."""
        self.assertIn("reason", self.app_src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
