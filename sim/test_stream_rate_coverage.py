#!/usr/bin/env python3
"""Every interlock input needs a MAVLink message that something asks for.

The Q27 readiness interlock reads `/mavros/estimator_status`, which MAVROS
fills only from EKF_STATUS_REPORT (193). Nothing in either stream table
requested that message, so the "EKF health" item reported "no data received"
for the life of the process. `is_ready()` requires EVERY item to pass, so the
interlock could never go true and the GCS "Start Mission 2" button stayed
blocked -- in simulation and on the aircraft alike. Nothing logged a cause:
the item simply sat at "no data received" among eleven that passed.

Two separate tables request streams -- `scripts/set_stream_rates.py` for the
one-shot pass and `stream_rate_keeper.py` for the node that re-asserts them --
and they are edited independently, so this pins them to each other too.

The tables are read with `ast` rather than imported: both modules pull in rclpy
and mavros_msgs, which are not importable on a development machine without ROS.
"""

import ast
import os
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TABLES = {
    "set_stream_rates.py": os.path.join(_ROOT, "scripts", "set_stream_rates.py"),
    "stream_rate_keeper.py": os.path.join(
        _ROOT, "src", "aerothon_mission", "mission_bringup",
        "mission_bringup", "stream_rate_keeper.py"),
}

# Each MAVLink message an interlock item cannot be evaluated without, and the
# readiness item that goes dark when it stops arriving.
REQUIRED = {
    24: ("GPS_RAW_INT", "GPS satellites / GPS HDOP"),
    1: ("SYS_STATUS", "Battery voltage"),
    193: ("EKF_STATUS_REPORT", "EKF health"),
    65: ("RC_CHANNELS", "RC failsafe"),
}

# The setpoint loop's rail, not an interlock item: the keeper warns below 10 Hz.
POSE_MESSAGE = 32
POSE_MIN_HZ = 10.0


def desired_table(path):
    """Return the module's DESIRED dict without importing the module."""
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=path)
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "DESIRED":
                return ast.literal_eval(node.value)
    raise AssertionError(f"no DESIRED table in {path}")


class StreamRateCoverage(unittest.TestCase):
    def test_every_interlock_message_is_requested(self):
        for label, path in TABLES.items():
            table = desired_table(path)
            for msg_id, (name, item) in REQUIRED.items():
                with self.subTest(f"{label}:{name}"):
                    self.assertIn(
                        msg_id, table,
                        f"{label} never asks for {name} ({msg_id}), so the "
                        f"readiness item '{item}' reports 'no data received' "
                        "forever. is_ready() requires every item, so the "
                        "interlock can never go true and the mission cannot "
                        "arm — on the aircraft as much as in simulation")

    def test_the_two_tables_agree(self):
        """They are edited separately and drifted apart once already."""
        rates = {label: desired_table(path) for label, path in TABLES.items()}
        (a_label, a), (b_label, b) = rates.items()
        self.assertEqual(
            set(a), set(b),
            f"{a_label} and {b_label} request different messages; the keeper "
            "re-asserts a different set than the one-shot pass established, so "
            "which rates survive depends on which ran last")
        for msg_id in a:
            self.assertEqual(
                a[msg_id], b[msg_id],
                f"message {msg_id} is requested at {a[msg_id]} Hz by {a_label} "
                f"and {b[msg_id]} Hz by {b_label}")

    def test_the_setpoint_rail_is_requested_fast_enough(self):
        """LOCAL_POSITION_NED feeds the position setpoint loop."""
        for label, path in TABLES.items():
            table = desired_table(path)
            with self.subTest(label):
                self.assertIn(POSE_MESSAGE, table,
                              f"{label} does not request LOCAL_POSITION_NED")
                self.assertGreaterEqual(
                    table[POSE_MESSAGE], POSE_MIN_HZ,
                    f"{label} asks for LOCAL_POSITION_NED at "
                    f"{table[POSE_MESSAGE]} Hz; stream_rate_keeper warns below "
                    f"{POSE_MIN_HZ} Hz because the setpoint loop starves")


if __name__ == "__main__":
    unittest.main()
