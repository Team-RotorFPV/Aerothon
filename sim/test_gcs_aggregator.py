#!/usr/bin/env python3
"""Unit test suite for GCS Aggregator telemetry snapshot and command dispatch."""

import sys
import unittest
import json
from unittest.mock import MagicMock

# Substitute mavros_msgs ONLY when it is genuinely unavailable. See the same
# note in sim/test_behavior_tree.py: the previous `not in sys.modules` guard
# poisoned sys.modules for every subsequently collected test file.
try:
    import mavros_msgs.msg  # noqa: F401
    import mavros_msgs.srv  # noqa: F401
except ImportError:
    m_msgs = MagicMock()
    sys.modules['mavros_msgs'] = m_msgs
    sys.modules['mavros_msgs.msg'] = m_msgs
    sys.modules['mavros_msgs.srv'] = m_msgs

from unittest.mock import patch, MagicMock

class TestGCSAggregator(unittest.TestCase):
    def setUp(self):
        with patch('rclpy.node.Node.__init__', return_value=None), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_client'), \
             patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.get_parameter', return_value=MagicMock(value="test-token")), \
             patch('rclpy.node.Node.get_logger'):
            from gcs_aggregator.aggregator import Aggregator, _euler_deg
            global _euler_deg_fn
            _euler_deg_fn = _euler_deg
            self.node = Aggregator()
            self.node.cli_arm = MagicMock()
            self.node.cli_mode = MagicMock()
            self.node.cli_takeoff = MagicMock()
            self.node.cli_land = MagicMock()
            self.node.pub_winch = MagicMock()
            self.node.pub_abort = MagicMock()
            self.node.pub_target = MagicMock()
            self.node.pub_start = MagicMock()
            self.node.state["safety"]["ready"] = True

    def test_euler_deg_conversion(self):
        """Test quaternion to euler degrees conversion."""
        q = MagicMock(x=0.0, y=0.0, z=0.0, w=1.0)
        roll, pitch, yaw = _euler_deg_fn(q)
        self.assertAlmostEqual(roll, 0.0, places=2)
        self.assertAlmostEqual(pitch, 0.0, places=2)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_command_dispatch_takeoff(self):
        """Test takeoff command triggers cli_takeoff."""
        res, reason = self.node._dispatch("takeoff", {"alt": 6.0})
        self.assertEqual(res, "accepted")

    def test_command_dispatch_land(self):
        """Test land command triggers land service."""
        res, reason = self.node._dispatch("land", {})
        self.assertEqual(res, "accepted")

    def test_command_dispatch_winch(self):
        """Test winch command publishes action."""
        res, reason = self.node._dispatch("winch", {"action": "lower"})
        self.assertEqual(res, "accepted")

    def test_command_dispatch_invalid(self):
        """Test unknown command returns rejected."""
        res, reason = self.node._dispatch("unknown_fly_command", {})
        self.assertEqual(res, "rejected")
        self.assertEqual(reason, "unknown cmd")

    def test_battery_telemetry_safety_flag(self):
        """Test battery low voltage correctly updates safety state."""
        batt = MagicMock(voltage=12.5, percentage=0.10)
        self.node._on_batt(batt)
        self.assertFalse(self.node.state["safety"]["battery_ok"])

        batt_ok = MagicMock(voltage=15.5, percentage=0.85)
        self.node._on_batt(batt_ok)
        self.assertTrue(self.node.state["safety"]["battery_ok"])

    # ---- Phase 10: the panel gets the REASONS, not just a Bool ---- #
    def test_interlock_detail_reaches_the_snapshot(self):
        """/mission_ready alone cannot tell an operator why ARM is blocked."""
        self.node._on_ready_detail(MagicMock(data=json.dumps({
            "ready": False,
            "items": [{"key": "gps_sats", "label": "GPS satellites",
                       "ok": False, "value": 7,
                       "reason": "7 satellites, need 12"}],
            "reasons": ["GPS satellites: 7 satellites, need 12"],
            "waived": [],
        })))
        sa = self.node.state["safety"]
        self.assertFalse(sa["ready"])
        self.assertEqual(sa["ready_items"][0]["value"], 7)
        self.assertIn("need 12", sa["ready_reasons"][0])

    def test_waived_items_stay_visible(self):
        """Relaxing a check is a decision; it must not vanish from the panel."""
        self.node._on_ready_detail(MagicMock(data=json.dumps({
            "ready": True, "items": [], "reasons": [],
            "waived": ["rc_failsafe"]})))
        self.assertEqual(self.node.state["safety"]["ready_waived"],
                         ["rc_failsafe"])

    def test_malformed_interlock_json_is_ignored_not_crashing(self):
        before = dict(self.node.state["safety"])
        self.node._on_ready_detail(MagicMock(data="not json"))
        self.assertEqual(self.node.state["safety"]["ready"], before["ready"])

    # ---- Phase 7: red zone is a tri-state ---- #
    def test_redzone_status_starts_UNKNOWN_not_clear(self):
        """A detector that has never published must not read as safe."""
        self.assertEqual(self.node._blank_state()["percep"]["redzone_status"],
                         "UNKNOWN")

    def test_redzone_detail_carries_status_and_exclusions(self):
        self.node._on_red_detail(MagicMock(data=json.dumps({
            "status": "RED", "reason": "confirmed red ground",
            "exclusions": [[38.0, 43.0, 2.0, 8.0]],
            "confirmed_area_m2": 34.6})))
        p = self.node.state["percep"]
        self.assertEqual(p["redzone_status"], "RED")
        self.assertEqual(len(p["redzone_exclusions"]), 1)
        self.assertAlmostEqual(p["redzone_area_m2"], 34.6)

    def test_NOT_VISIBLE_is_distinguished_from_CLEAR(self):
        """The boolean reported both as 'no red zone'. Only one of them is a
        statement about the ground."""
        self.node._on_red_detail(MagicMock(data=json.dumps(
            {"status": "NOT_VISIBLE", "reason": "camera above the horizon"})))
        self.assertEqual(self.node.state["percep"]["redzone_status"],
                         "NOT_VISIBLE")
        self.node._on_red_detail(MagicMock(data=json.dumps(
            {"status": "CLEAR", "reason": ""})))
        self.assertEqual(self.node.state["percep"]["redzone_status"], "CLEAR")


if __name__ == '__main__':
    unittest.main()


class ScanLedgerTests(unittest.TestCase):
    """Everything detected or decoded, kept, with matches tagged.

    THE OPERATOR'S REQUEST, verbatim: "it should maintain a list of everything
    that has been detected/decoded and everything that has been matched should
    have a tag with it".

    WHY THE AGGREGATOR OWNS THE DE-DUPLICATION

        A marker sits in frame for two hundred frames. Shipping two hundred
        rows to the browser and de-duplicating them there means the list is
        computed twice, in two languages, and the two can disagree about what
        the run contained. The aggregator already holds the one snapshot
        everything else agrees on; the ledger belongs in it.
    """

    def setUp(self):
        with patch('rclpy.node.Node.__init__', return_value=None), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_client'), \
             patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.get_parameter',
                   return_value=MagicMock(value="test-token")), \
             patch('rclpy.node.Node.get_logger'):
            from gcs_aggregator.aggregator import Aggregator
            self.node = Aggregator()

    def _qr(self, **kw):
        d = {"accepted": "", "matched": False, "target": "PAD_C",
             "rejected": [], "marker_px": 40.0}
        d.update(kw)
        self.node._on_qr_detail(MagicMock(data=json.dumps(d)))

    def _banner(self, **kw):
        d = {"identified": False, "text": "", "reason": "", "green": True}
        d.update(kw)
        self.node._on_banner_detail(MagicMock(data=json.dumps(d)))

    def ledger(self):
        return self.node.state["scans"]

    # ---- it records ---- #
    def test_it_starts_empty(self):
        self.assertEqual(self.ledger(), [])

    def test_a_decoded_payload_is_recorded(self):
        self._qr(accepted="PAD_A")
        self.assertEqual(len(self.ledger()), 1)
        self.assertEqual(self.ledger()[0]["payload"], "PAD_A")
        self.assertEqual(self.ledger()[0]["kind"], "qr")

    def test_every_distinct_payload_gets_its_own_entry(self):
        """"every qr decoded", not just the one that matched."""
        for p in ("PAD_A", "PAD_B", "PAD_C"):
            self._qr(accepted=p)
        self.assertEqual([e["payload"] for e in self.ledger()],
                         ["PAD_A", "PAD_B", "PAD_C"])

    def test_the_same_payload_seen_again_does_not_add_a_row(self):
        for _ in range(200):
            self._qr(accepted="PAD_A")
        self.assertEqual(len(self.ledger()), 1)

    def test_the_same_payload_seen_again_counts_the_frames(self):
        """The count is what tells an operator a read was solid, not a blip."""
        for _ in range(37):
            self._qr(accepted="PAD_A")
        self.assertEqual(self.ledger()[0]["count"], 37)

    # ---- it tags matches ---- #
    def test_a_matched_payload_is_tagged(self):
        self._qr(accepted="PAD_C", matched=True)
        self.assertTrue(self.ledger()[0]["matched"])

    def test_matching_upgrades_the_existing_row_rather_than_adding_one(self):
        self._qr(accepted="PAD_C", matched=False)
        self._qr(accepted="PAD_C", matched=True)
        self.assertEqual(len(self.ledger()), 1)
        self.assertTrue(self.ledger()[0]["matched"])

    def test_a_match_is_never_downgraded_by_a_later_frame(self):
        """Losing the marker for a frame does not un-match the mission."""
        self._qr(accepted="PAD_C", matched=True)
        self._qr(accepted="PAD_C", matched=False)
        self.assertTrue(self.ledger()[0]["matched"])

    def test_decoding_is_not_matching(self):
        self._qr(accepted="PAD_A", matched=False)
        self.assertFalse(self.ledger()[0]["matched"])
        self.assertEqual(self.ledger()[0]["status"], "DECODED")

    # ---- it keeps rejections ---- #
    def test_a_rejection_is_recorded_with_its_reason(self):
        self._qr(rejected=[{"reason": "marker 12 px below expected 30-90 px"}])
        self.assertEqual(len(self.ledger()), 1)
        e = self.ledger()[0]
        self.assertEqual(e["status"], "REJECTED")
        self.assertIn("12 px", e["reason"])

    def test_a_banner_rejection_carries_the_detectors_reason(self):
        self._banner(reason="board aspect 0.73 outside 1.2-8.0")
        self.assertEqual(self.ledger()[0]["kind"], "banner")
        self.assertIn("aspect 0.73", self.ledger()[0]["reason"])

    def test_an_identified_banner_records_the_text_it_read(self):
        """"the text it READ" -- so the reader has to have confirmed it. An
        unconfirmed field is a failed read, not a quieter one."""
        self._banner(identified=True, text="AEROTHON", text_confirmed=True)
        e = self.ledger()[0]
        self.assertEqual(e["status"], "IDENTIFIED")
        self.assertEqual(e["payload"], "AEROTHON")

    def test_the_reading_path_is_recorded_so_a_rescue_is_visible(self):
        self._banner(identified=True, text="AEROTHON", lettering_path="stroke")
        self.assertEqual(self.ledger()[0]["via"], "stroke")

    def test_an_UNREADABLE_reading_is_not_shown_as_a_reading(self):
        """Watched live, the panel filled with rows like

            BANNER  ??N?E??????N?   ID   via brightness
            BANNER  ???AER????????????  ID  via stroke

        The glyph classifier marks anything it cannot name '?'. Those rows are
        legitimately identified -- identity passes on STRUCTURE, and the text
        is only ever a confirming check -- but printing the failed read as the
        payload presents noise as data. The operator called it what it is:
        absurd.
        """
        self._banner(identified=True, text="??N?E??????N?")
        self.assertEqual(self.ledger()[0]["payload"], "BANNER")

    def test_a_partial_reading_is_not_shown_either(self):
        self._banner(identified=True, text="?N?ER????N?")
        self.assertNotIn("?", self.ledger()[0]["payload"])

    def test_a_REAL_reading_is_still_shown(self):
        self._banner(identified=True, text="AEROTHON", text_confirmed=True)
        self.assertEqual(self.ledger()[0]["payload"], "AEROTHON")

    def test_a_confirmed_reading_is_marked_as_actually_READ(self):
        """A structural identification and one that read the lettering are
        different claims, and the panel has to keep them apart."""
        self._banner(identified=True, text="AEROTHON", text_confirmed=True)
        self.assertTrue(self.ledger()[0]["read"])
        self._banner(identified=True, text="??N?E??")
        rows = [r for r in self.ledger() if r["payload"] == "BANNER"]
        self.assertTrue(rows)
        self.assertFalse(rows[0]["read"])

    def test_unreadable_rows_collapse_into_ONE_row(self):
        """Every distinct garbage string was its own row, so a single banner
        produced dozens of them and pushed everything else out of the list."""
        for t in ("?????", "??N?E??", "???AER??????", "?N?E????N?"):
            self._banner(identified=True, text=t)
        self.assertEqual(len(self.ledger()), 1)
        self.assertEqual(self.ledger()[0]["count"], 4)

    def test_repeated_rejections_for_the_same_reason_collapse(self):
        for _ in range(50):
            self._banner(reason="board is not green enough (0.11)")
        self.assertEqual(len(self.ledger()), 1)
        self.assertEqual(self.ledger()[0]["count"], 50)

    def test_different_reasons_are_different_rows(self):
        self._banner(reason="board aspect 0.73 outside 1.2-8.0")
        self._banner(reason="board is not green enough (0.11)")
        self.assertEqual(len(self.ledger()), 2)

    # ---- it stays usable ---- #
    def test_the_ledger_is_bounded(self):
        for i in range(400):
            self._qr(accepted=f"PAD_{i}")
        self.assertLessEqual(len(self.ledger()), self.node.MAX_SCANS)

    def test_the_bound_drops_the_OLDEST_not_the_newest(self):
        for i in range(self.node.MAX_SCANS + 5):
            self._qr(accepted=f"PAD_{i}")
        payloads = [e["payload"] for e in self.ledger()]
        self.assertNotIn("PAD_0", payloads)
        self.assertIn(f"PAD_{self.node.MAX_SCANS + 4}", payloads)

    def test_a_matched_row_survives_the_bound(self):
        """The match is the one row of the whole run that matters."""
        self._qr(accepted="TARGET", matched=True)
        for i in range(self.node.MAX_SCANS + 20):
            self._qr(accepted=f"PAD_{i}")
        self.assertIn("TARGET", [e["payload"] for e in self.ledger()])

    def test_each_row_records_the_mission_stage_it_happened_in(self):
        self.node.state["mission"]["state"] = "SEARCH_QR"
        self._qr(accepted="PAD_A")
        self.assertEqual(self.ledger()[0]["stage"], "SEARCH_QR")

    def test_rows_are_ordered_oldest_first(self):
        self._qr(accepted="FIRST")
        self._qr(accepted="SECOND")
        self.assertLess(self.ledger()[0]["seq"], self.ledger()[1]["seq"])

    def test_empty_reads_are_not_recorded(self):
        """A frame with nothing in it is not an observation."""
        for _ in range(10):
            self._qr()
        self._banner()
        self.assertEqual(self.ledger(), [])

    def test_starting_a_new_mission_clears_the_ledger(self):
        """Two runs' markers in one list, with nothing to say which was which,
        is worse than no list."""
        self.node.state["safety"]["ready"] = True
        self.node.pub_start = MagicMock()
        self._qr(accepted="PAD_A")
        self.node._dispatch("start_mission", {})
        self.assertEqual(self.ledger(), [])

    def test_the_sequence_numbers_restart_with_the_new_run(self):
        self.node.state["safety"]["ready"] = True
        self.node.pub_start = MagicMock()
        self._qr(accepted="PAD_A")
        self.node._dispatch("start_mission", {})
        self._qr(accepted="PAD_B")
        self.assertEqual(self.ledger()[0]["seq"], 1)

    def test_malformed_detail_does_not_break_the_ledger(self):
        self.node._on_qr_detail(MagicMock(data="{not json"))
        self.node._on_banner_detail(MagicMock(data=""))
        self.assertEqual(self.ledger(), [])
