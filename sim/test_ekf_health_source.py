#!/usr/bin/env python3
"""EKF health has to come from a message ArduPilot actually sends.

WHAT WAS WRONG
    The interlock read `/mavros/estimator_status`. MAVROS fills that topic from
    `SystemStatusPlugin::handle_estimator_status`, whose handler is typed
    `mavlink::common::msg::ESTIMATOR_STATUS` -- message **230**.

    ArduPilot never sends 230. It sends EKF_STATUS_REPORT, message **193**, an
    ardupilotmega message MAVROS has no handler for at all: the string
    "EKF_STATUS_REPORT" does not appear anywhere in libmavros_plugins.so.

    So the topic was advertised, the node was subscribed, and nothing was ever
    published on it. "EKF health" read "no data received" for the life of every
    run and `is_ready()` -- which requires EVERY item -- could never go true.

    A previous fix added 193 to both stream tables on the premise that MAVROS
    filled the topic from 193. That got the data onto the wire (confirmed at
    0.94 Hz with pymavlink) and it still had nowhere to go. Necessary, not
    sufficient.

    **This is a defect on the aircraft, not only in simulation.** The message
    ID is a property of the firmware.

WHAT THIS PINS
    That EKF health is derived from SYS_STATUS's AHRS bit, that an absent AHRS
    does not read as a healthy one, and that a real PX4 ESTIMATOR_STATUS still
    wins when it arrives.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_ekf_health_source.py -v
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_gcs", "gcs_aggregator"))

from gcs_aggregator.readiness import evaluate
from gcs_aggregator.readiness_node import Readiness

# MAV_SYS_STATUS_AHRS
AHRS = 0x0400

# Measured on the live FCU this session: ArduPilot Copter-4.5 in SITL, healthy.
REAL_HEALTHY = 0x57719C2F


def node():
    n = Readiness.__new__(Readiness)
    n.obs = {}
    return n


def sys_status(present=AHRS, health=AHRS):
    return SimpleNamespace(sensors_present=present, sensors_health=health)


def ekf_item(obs):
    return next(i for i in evaluate(obs) if i["key"] == "ekf")


class EkfHealthFromSysStatus(unittest.TestCase):
    def test_the_defect_no_estimator_status_means_no_ekf_item(self):
        """With only the PX4 path wired, ArduPilot leaves the item dark."""
        item = ekf_item(node().obs)
        self.assertFalse(item["ok"])
        self.assertEqual(item["reason"], "no data received")

    def test_a_healthy_ardupilot_fcu_passes(self):
        n = node()
        n._on_sys_status(sys_status(health=REAL_HEALTHY, present=REAL_HEALTHY))
        self.assertTrue(n.obs["ekf_ok"])
        self.assertTrue(ekf_item(n.obs)["ok"])

    def test_an_unhealthy_ahrs_blocks_with_a_reason(self):
        n = node()
        n._on_sys_status(sys_status(present=AHRS, health=0))
        self.assertFalse(n.obs["ekf_ok"])
        self.assertIn("unhealthy", n.obs["ekf_reason"])
        self.assertFalse(ekf_item(n.obs)["ok"])

    def test_an_absent_ahrs_is_not_a_healthy_one(self):
        """Unknown must not read as fine -- the whole point of the interlock."""
        n = node()
        n._on_sys_status(sys_status(present=0, health=0))
        self.assertFalse(n.obs["ekf_ok"])
        self.assertIn("no AHRS", n.obs["ekf_reason"])

    def test_the_bit_is_read_not_the_whole_mask(self):
        """A mask with other sensors healthy but AHRS dark must still block."""
        n = node()
        n._on_sys_status(sys_status(present=REAL_HEALTHY,
                                    health=REAL_HEALTHY & ~AHRS))
        self.assertFalse(n.obs["ekf_ok"])

    def test_a_real_estimator_status_wins_over_sys_status(self):
        """PX4's report is strictly richer, so it must not be overwritten."""
        n = node()
        n._on_ekf(SimpleNamespace(
            attitude_status_flag=False, velocity_horiz_status_flag=True,
            pos_horiz_abs_status_flag=True, pos_vert_abs_status_flag=True,
            const_pos_mode_status_flag=False))
        self.assertFalse(n.obs["ekf_ok"])
        n._on_sys_status(sys_status(health=REAL_HEALTHY, present=REAL_HEALTHY))
        self.assertFalse(n.obs["ekf_ok"], "sys_status overwrote a real EKF report")
        self.assertIn("attitude", n.obs["ekf_reason"])


class TheTopicItSubscribesTo(unittest.TestCase):
    def test_it_subscribes_to_sys_status(self):
        """Guards against the subscription being dropped in a refactor."""
        import inspect
        src = inspect.getsource(Readiness.__init__)
        self.assertIn("/mavros/sys_status", src)


if __name__ == "__main__":
    unittest.main()
