#!/usr/bin/env python3
"""Phase 10 — every interlock item can individually block arming, with a reason.

WHAT WAS WRONG
    /mission_ready was four checks (FC connected, GPS "fix", a LaserScan in
    the last 2 s, an Image in the last 2 s) collapsed into one Bool with no
    reason attached. `status >= 0` is true for a three-satellite fix with HDOP
    9. Battery, EKF, MAVLink latency, camera pose, detectors, winch and RC
    failsafe were not checked at all.

    The Phase 10 acceptance criterion is that EACH item can be forced to fail
    on its own and produce the right reason — which is what this file is.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_readiness_interlock.py -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_gcs", "gcs_aggregator"))

from gcs_aggregator.readiness import (
    ITEMS,
    blocking_reasons,
    evaluate,
    is_ready,
)
from gcs_aggregator.readiness_node import RateMeter


def healthy():
    """An observation where every single item passes."""
    return {
        "connected": True,
        "sats": 14,
        "hdop": 0.8,
        "battery_v": 16.2,
        "ekf_ok": True,
        "lidar_hz": 12.0,
        "latency_ms": 25.0,
        "camera_settled": True,
        "camera_stale": False,
        "detectors": {"qr": 0.2, "banner": 0.3, "redzone": 0.4},
        "winch_fault": "",
        "rc_failsafe": False,
    }


def item(items, key):
    return next(i for i in items if i["key"] == key)


class BaselineTests(unittest.TestCase):

    def test_a_healthy_aircraft_is_ready(self):
        items = evaluate(healthy())
        self.assertTrue(is_ready(items), blocking_reasons(items))

    def test_every_declared_item_is_evaluated(self):
        keys = [i["key"] for i in evaluate(healthy())]
        self.assertEqual(keys, [k for k, _ in ITEMS])

    def test_an_empty_observation_is_NOT_ready(self):
        """Unknown is not the same as fine. Defaulting unknown to fine is how
        an interlock becomes decoration."""
        items = evaluate({})
        self.assertFalse(is_ready(items))
        self.assertEqual(len(blocking_reasons(items)), len(ITEMS))

    def test_no_items_is_not_ready(self):
        self.assertFalse(is_ready([]))


class EachItemBlocksTests(unittest.TestCase):
    """The acceptance criterion: force each one, arming is blocked, reason
    names that item and nothing else."""

    def force(self, **overrides):
        obs = healthy()
        obs.update(overrides)
        items = evaluate(obs)
        self.assertFalse(is_ready(items),
                         f"interlock passed with {overrides}")
        failing = [i["key"] for i in items if not i["ok"]]
        return items, failing

    def test_disconnected_fcu_blocks(self):
        items, failing = self.force(connected=False)
        self.assertEqual(failing, ["fcu_link"])
        self.assertIn("not connected", item(items, "fcu_link")["reason"])

    def test_too_few_satellites_blocks(self):
        """The old check passed an 11-satellite fix. Q27 wants 12."""
        items, failing = self.force(sats=11)
        self.assertEqual(failing, ["gps_sats"])
        self.assertIn("11", item(items, "gps_sats")["reason"])

    def test_the_old_three_satellite_fix_is_now_REJECTED(self):
        items, failing = self.force(sats=3, hdop=9.0)
        self.assertIn("gps_sats", failing)
        self.assertIn("gps_hdop", failing)

    def test_poor_hdop_blocks_even_with_many_satellites(self):
        items, failing = self.force(hdop=1.5)
        self.assertEqual(failing, ["gps_hdop"])

    def test_hdop_exactly_at_the_limit_blocks(self):
        _, failing = self.force(hdop=1.2)
        self.assertEqual(failing, ["gps_hdop"])

    def test_low_battery_blocks(self):
        items, failing = self.force(battery_v=14.6)
        self.assertEqual(failing, ["battery"])
        self.assertIn("14.6", item(items, "battery")["reason"])

    def test_unhealthy_ekf_blocks_with_its_own_reason(self):
        items, failing = self.force(ekf_ok=False,
                                    ekf_reason="compass variance high")
        self.assertEqual(failing, ["ekf"])
        self.assertIn("compass", item(items, "ekf")["reason"])

    def test_slow_lidar_blocks(self):
        """A lidar stuttering at 1 Hz passed the old "seen in the last 2 s"."""
        items, failing = self.force(lidar_hz=1.0)
        self.assertEqual(failing, ["lidar_rate"])

    def test_high_mavlink_latency_blocks(self):
        items, failing = self.force(latency_ms=250.0)
        self.assertEqual(failing, ["mavlink_latency"])

    def test_unsettled_camera_blocks(self):
        items, failing = self.force(camera_settled=False)
        self.assertEqual(failing, ["camera_pose"])
        self.assertIn("settled", item(items, "camera_pose")["reason"])

    def test_stale_camera_feedback_blocks_even_when_settled(self):
        items, failing = self.force(camera_settled=True, camera_stale=True)
        self.assertEqual(failing, ["camera_pose"])
        self.assertIn("stale", item(items, "camera_pose")["reason"])

    def test_a_stale_detector_blocks_and_is_NAMED(self):
        items, failing = self.force(
            detectors={"qr": 0.2, "banner": 9.0, "redzone": 0.4})
        self.assertEqual(failing, ["detectors"])
        self.assertIn("banner", item(items, "detectors")["reason"])
        self.assertNotIn("qr", item(items, "detectors")["reason"])

    def test_no_detectors_at_all_blocks(self):
        items, failing = self.force(detectors={})
        self.assertEqual(failing, ["detectors"])

    def test_a_winch_fault_blocks(self):
        items, failing = self.force(winch_fault="jam")
        self.assertEqual(failing, ["actuator"])
        self.assertIn("jam", item(items, "actuator")["reason"])

    def test_rc_failsafe_blocks(self):
        items, failing = self.force(rc_failsafe=True)
        self.assertEqual(failing, ["rc_failsafe"])


class UnknownIsNotOkTests(unittest.TestCase):
    """Every item independently: a missing input must not read as healthy."""

    def test_each_missing_input_blocks_its_own_item(self):
        mapping = {
            "connected": "fcu_link",
            "sats": "gps_sats",
            "hdop": "gps_hdop",
            "battery_v": "battery",
            "ekf_ok": "ekf",
            "lidar_hz": "lidar_rate",
            "latency_ms": "mavlink_latency",
            "camera_settled": "camera_pose",
            "detectors": "detectors",
            "winch_fault": "actuator",
            "rc_failsafe": "rc_failsafe",
        }
        for obs_key, item_key in mapping.items():
            obs = healthy()
            del obs[obs_key]
            items = evaluate(obs)
            failing = [i["key"] for i in items if not i["ok"]]
            self.assertEqual(failing, [item_key],
                             f"dropping {obs_key} did not block {item_key}")
            self.assertIn("no data", item(items, item_key)["reason"])


class ReasonReportingTests(unittest.TestCase):

    def test_a_ready_aircraft_reports_no_reasons(self):
        self.assertEqual(blocking_reasons(evaluate(healthy())), [])

    def test_reasons_name_the_item(self):
        obs = healthy()
        obs["battery_v"] = 12.0
        reasons = blocking_reasons(evaluate(obs))
        self.assertEqual(len(reasons), 1)
        self.assertIn("Battery voltage", reasons[0])

    def test_multiple_failures_are_all_reported(self):
        obs = healthy()
        obs.update(battery_v=12.0, sats=4, rc_failsafe=True)
        self.assertEqual(len(blocking_reasons(evaluate(obs))), 3)

    def test_limits_are_configurable_not_baked_in(self):
        obs = healthy()
        obs["sats"] = 8
        self.assertFalse(is_ready(evaluate(obs)))
        self.assertTrue(is_ready(evaluate(obs, limits={"min_sats": 6})))

    def test_every_item_carries_a_measured_value(self):
        """The panel shows what was measured, not just pass/fail."""
        for i in evaluate(healthy()):
            self.assertIsNotNone(i["value"], f"{i['key']} reported no value")


class RateMeterTests(unittest.TestCase):
    """"A message arrived recently" cannot tell 12 Hz from 1 Hz, and the
    obstacle avoidance is only as good as the scan rate."""

    def test_a_healthy_stream_reports_its_rate(self):
        r = RateMeter(window_s=2.0)
        t = 1000.0
        for i in range(20):
            r.touch(t + i * 0.1)
        self.assertAlmostEqual(r.hz(t + 1.9), 10.0, delta=1.0)

    def test_a_stuttering_stream_is_DISTINGUISHED_from_a_healthy_one(self):
        """The exact case the old freshness check waved through."""
        r = RateMeter(window_s=3.0)
        t = 1000.0
        for i in range(3):
            r.touch(t + i * 1.0)
        self.assertLess(r.hz(t + 2.5), 2.0)

    def test_a_silent_stream_reports_zero(self):
        self.assertEqual(RateMeter().hz(), 0.0)

    def test_a_single_message_is_not_a_rate(self):
        r = RateMeter(window_s=3.0)
        r.touch(1000.0)
        self.assertEqual(r.hz(1000.5), 0.0)

    def test_old_samples_leave_the_window(self):
        r = RateMeter(window_s=1.0)
        t = 1000.0
        for i in range(10):
            r.touch(t + i * 0.05)
        self.assertEqual(r.hz(t + 60.0), 0.0)

    def test_age_is_none_before_anything_arrives(self):
        self.assertIsNone(RateMeter().age())


class WaiverTests(unittest.TestCase):
    """Relaxing an item is a decision, and must stay visible."""

    def test_a_waived_item_is_marked_not_silently_passed(self):
        obs = healthy()
        obs["rc_failsafe"] = True
        items = evaluate(obs)
        for i in items:
            if i["key"] == "rc_failsafe" and not i["ok"]:
                i["ok"] = True
                i["reason"] = f"WAIVED ({i['reason']})"
        self.assertTrue(is_ready(items))
        self.assertIn("WAIVED", item(items, "rc_failsafe")["reason"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
