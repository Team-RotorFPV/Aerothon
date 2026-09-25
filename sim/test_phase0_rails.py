#!/usr/bin/env python3
"""Phase 0 — fail-closed rail tests (PHASE_PLAN.md).

These run against a REAL rclpy node and the REAL Mav commander, not a mock.
That is deliberate: the previous test suite passed 13 assertions against a
MockMav while the actual mission streamed setpoints at a disarmed aircraft
and reported GOTO_CORRIDOR from the ground. Mocks cannot catch that class of
defect, so every rail added in Phase 0 is asserted through the real object.

Run:
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=src/aerothon_mission/mission_bt python3 -m pytest sim/test_phase0_rails.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State

from mission_bt.mav_commander import Mav
from mission_bt.mission_tree import (
    CheckAbortTriggered,
    apply_pending_reset,
    build_root,
)


# Mirrors declare_mission_params(); the asserted arena coordinates it used
# to carry (scan_pose, corridor_entry/exit_x, zone_entry, zone, home) are
# gone -- perception supplies them now.
DEFAULT_PARAMS = {
    'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
    'image_width_px': 1280, 'camera_hfov': 1.0472,
    'target_marker_m': 2.2, 'qr_modules': 33,
    'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
    'zone_margin': 1.0, 'corridor_alt': 3.0,
    'waypoint_tol': 0.8, 'drop_tol': 0.5, 'scan_floor_alt': 2.0,
}


def quat_from_rpy(roll_deg, pitch_deg, yaw_deg=0.0):
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def pose_with(roll_deg=0.0, pitch_deg=0.0, z=5.0):
    m = PoseStamped()
    m.pose.position.z = z
    m.pose.orientation = quat_from_rpy(roll_deg, pitch_deg)
    return m


def state_msg(armed=False, mode="GUIDED", connected=True):
    s = State()
    s.armed = armed
    s.mode = mode
    s.connected = connected
    return s


class RailTestCase(unittest.TestCase):
    """Base: one fresh rclpy node (and therefore one fresh Mav) per test."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = Node(f"phase0_test_{self.id().split('.')[-1]}")
        self.mav = Mav(self.node)
        self.published_setpoints = []
        self.published_results = []
        self.mav.pub_sp.publish = self.published_setpoints.append
        self.mav.pub_result.publish = self.published_results.append
        self.supply_organiser_inputs()

    def supply_organiser_inputs(self):
        """What the organisers hand over pre-flight, and an FC that accepts it.

        The tree will not arm without a delivery-zone boundary and a
        verified, enforced arena geofence. These rails are about what happens
        after arming, so the inputs are supplied and the fence service echoes
        the upload back intact, as a healthy FC does.
        """
        from mavros_msgs.msg import HomePosition
        home = HomePosition()
        home.geo.latitude = -35.3632621
        home.geo.longitude = 149.1652374
        self.mav.home = home
        self.mav.delivery_zone_local = (12.0, 52.0, -15.0, 15.0)
        self.mav.geofence_local = [(-9.5, -21.0), (58.0, -21.0),
                                   (58.0, 21.0), (-9.5, 21.0)]

        class _Done:
            def __init__(self, ok=True):
                self._r = type("R", (), {"success": ok})()

            def done(self):
                return True

            def result(self):
                return self._r

        def push(items):
            self.mav.fence_readback = list(items)
            return _Done()

        self.mav.push_fence = push
        self.mav.set_param = lambda name, value: _Done()

    def tearDown(self):
        self.node.destroy_node()

    def arm(self, mode="GUIDED"):
        self.mav.state = state_msg(armed=True, mode=mode)


# --------------------------------------------------------------------------- #
# Rail 1: no setpoints while disarmed
# --------------------------------------------------------------------------- #
class TestSetpointGate(RailTestCase):

    def test_disarmed_setpoints_are_suppressed(self):
        self.mav.state = state_msg(armed=False)
        self.mav.goto(10.0, 0.0, 5.0)
        for _ in range(10):
            self.mav._stream()
        self.assertEqual(self.published_setpoints, [],
                         "setpoints were published to a DISARMED aircraft")
        self.assertEqual(self.mav.setpoints_suppressed, 10)
        self.assertEqual(self.mav.setpoint_block_reason, "disarmed")

    def test_armed_setpoints_flow(self):
        self.arm()
        self.mav.goto(10.0, 0.0, 5.0)
        for _ in range(5):
            self.mav._stream()
        self.assertEqual(len(self.published_setpoints), 5)
        self.assertEqual(self.mav.setpoint_block_reason, "")
        sp = self.published_setpoints[-1]
        self.assertAlmostEqual(sp.pose.position.x, 10.0)
        self.assertEqual(sp.header.frame_id, "map")

    def test_gate_closes_again_on_disarm_mid_flight(self):
        self.arm()
        self.mav.goto(10.0, 0.0, 5.0)
        self.mav._stream()
        self.assertEqual(len(self.published_setpoints), 1)
        self.mav.state = state_msg(armed=False)
        for _ in range(5):
            self.mav._stream()
        self.assertEqual(len(self.published_setpoints), 1,
                         "setpoints continued after the aircraft disarmed")

    def test_no_setpoint_when_none_requested(self):
        self.arm()
        for _ in range(5):
            self.mav._stream()
        self.assertEqual(self.published_setpoints, [])


# --------------------------------------------------------------------------- #
# Rail 2: external intervention resets the mission
# --------------------------------------------------------------------------- #
class TestExternalIntervention(RailTestCase):

    def test_external_disarm_requests_reset(self):
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False))
        self.assertTrue(self.mav.reset_pending())
        self.assertEqual(self.mav.consume_reset(), "external disarm")

    def test_gate_heading_is_latched_by_the_outbound_gate(self):
        """The return gate's advance must not overwrite the corridor axis."""
        self.assertEqual(self.mav.record_gate_heading(0.08), 0.08)
        self.assertEqual(self.mav.record_gate_heading(3.2), 0.08)

    def test_a_new_start_forgets_the_last_corridor(self):
        from std_msgs.msg import Bool
        self.mav.record_gate_heading(0.08)
        self.mav.corridor_exit_pose = (18.4, 2.95, 3.0, 0.08)
        self.mav._on_start(Bool(data=True))
        self.assertIsNone(self.mav.gate_heading)
        self.assertIsNone(self.mav.corridor_exit_pose)

    def test_lost_link_is_not_an_external_disarm(self):
        """On heartbeat loss MAVROS publishes connected=False with armed
        defaulted to False. Arena 1001 (batch E) ended "external disarm" on
        exactly that while the aircraft hovered armed in GUIDED."""
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False, mode="", connected=False))
        self.assertFalse(self.mav.reset_pending())
        self.mav._on_state(state_msg(armed=True))       # link back, still flying
        self.assertFalse(self.mav.reset_pending())

    def test_disarm_during_an_outage_is_caught_when_the_link_returns(self):
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False, mode="", connected=False))
        self.mav._on_state(state_msg(armed=False))      # FCU: really disarmed
        self.assertTrue(self.mav.reset_pending())
        self.assertEqual(self.mav.consume_reset(), "external disarm")

    def test_commanded_landing_disarm_is_not_an_intervention(self):
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav.expect_disarm(True)          # the Land leaf sets this
        self.mav._on_state(state_msg(armed=False))
        self.assertFalse(self.mav.reset_pending())

    def test_external_mode_change_requests_reset(self):
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self.mav._on_state(state_msg(armed=True, mode="RTL"))
        self.assertTrue(self.mav.reset_pending())
        self.assertIn("RTL", self.mav.consume_reset())

    def test_our_own_rtl_is_not_an_intervention(self):
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self.mav.set_mode("RTL")              # abort path commands this itself
        self.mav._on_state(state_msg(armed=True, mode="RTL"))
        self.assertFalse(self.mav.reset_pending())

    def test_idle_aircraft_does_not_reset(self):
        self.mav.mission_started = False
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False))
        self.assertFalse(self.mav.reset_pending())

    def test_new_start_clears_stale_expect_disarm(self):
        """A completed mission must not deafen the watchdog on the next run.

        The Land leaf sets expect_disarm so its own touchdown is not read as an
        intervention. If that flag survived into the next mission, a genuine
        external disarm would be silently swallowed.
        """
        from std_msgs.msg import Bool
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav.expect_disarm(True)            # previous run landed
        self.mav._on_state(state_msg(armed=False))
        self.assertFalse(self.mav.reset_pending())

        # Operator starts a fresh mission.
        self.mav._on_start(Bool(data=True))
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False))
        self.assertTrue(self.mav.reset_pending(),
                        "stale expect_disarm swallowed a real intervention")

    def test_new_start_clears_stale_attitude_violations(self):
        from std_msgs.msg import Bool
        for _ in range(self.mav.attitude_limit_samples):
            self.mav._on_pose(pose_with(roll_deg=70.0))
        self.assertTrue(self.mav.attitude_excessive())
        self.mav._on_start(Bool(data=True))
        self.assertFalse(self.mav.attitude_excessive(),
                         "previous run's attitude violations leaked into a new mission")

    def test_consume_reset_clears_mission_state(self):
        self.mav.mission_started = True
        self.mav.abort_requested = True
        self.mav.goto(5.0, 0.0, 3.0)
        self.mav.state = state_msg(armed=True)
        self.mav._on_state(state_msg(armed=False))
        self.mav.consume_reset()
        self.assertFalse(self.mav.mission_started)
        self.assertFalse(self.mav.abort_requested)
        self.assertIsNone(self.mav._sp, "stale setpoint survived the reset")
        self.assertIsNone(self.mav.consume_reset(), "reset fired twice")


# --------------------------------------------------------------------------- #
# Rail 3: excessive attitude aborts
# --------------------------------------------------------------------------- #
class TestAttitudeAbort(RailTestCase):

    def test_level_flight_is_not_excessive(self):
        for _ in range(20):
            self.mav._on_pose(pose_with(roll_deg=3.0, pitch_deg=-2.0))
        self.assertFalse(self.mav.attitude_excessive())

    def test_sustained_tilt_trips_after_n_samples(self):
        limit_samples = self.mav.attitude_limit_samples
        for _ in range(limit_samples - 1):
            self.mav._on_pose(pose_with(roll_deg=54.0))
        self.assertFalse(self.mav.attitude_excessive(),
                         "tripped before the debounce count")
        self.mav._on_pose(pose_with(roll_deg=54.0))
        self.assertTrue(self.mav.attitude_excessive())

    def test_single_noisy_sample_does_not_trip(self):
        self.mav._on_pose(pose_with(roll_deg=80.0))
        self.mav._on_pose(pose_with(roll_deg=1.0))
        for _ in range(10):
            self.mav._on_pose(pose_with(roll_deg=1.0))
        self.assertFalse(self.mav.attitude_excessive())

    def test_roll_pitch_recovered_from_quaternion(self):
        self.mav._on_pose(pose_with(roll_deg=-53.8, pitch_deg=5.9))
        self.assertAlmostEqual(self.mav.roll_deg, -53.8, places=1)
        self.assertAlmostEqual(self.mav.pitch_deg, 5.9, places=1)

    def test_abort_latches_and_does_not_resume(self):
        """An abort that un-aborts itself is not an abort.

        Every guard condition is level-triggered: attitude recovers once RTL
        levels the aircraft, battery voltage recovers under reduced load, the
        FCU reconnects. Observed live — the guard fired on a 46 deg pitch,
        commanded RTL, then released as soon as the aircraft levelled, and the
        mission resumed its previous leg mid-flight.
        """
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        guard = CheckAbortTriggered(self.mav, self.node)

        for _ in range(self.mav.attitude_limit_samples):
            self.mav._on_pose(pose_with(roll_deg=54.0))
        self.assertEqual(guard.update(), py_trees.common.Status.SUCCESS)
        self.assertTrue(self.mav.abort_latched)
        latched_reason = self.mav.abort_reason

        # Aircraft levels out under RTL: the raw condition is no longer true.
        for _ in range(20):
            self.mav._on_pose(pose_with(roll_deg=0.5))
        self.assertFalse(self.mav.attitude_excessive())

        self.assertEqual(guard.update(), py_trees.common.Status.SUCCESS,
                         "abort released once the condition cleared")
        self.assertEqual(self.mav.abort_reason, latched_reason,
                         "latched reason was overwritten after the fact")

    def test_abort_latch_clears_only_on_new_start(self):
        from std_msgs.msg import Bool
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        self.mav.abort_requested = True
        guard = CheckAbortTriggered(self.mav, self.node)
        guard.update()
        self.assertTrue(self.mav.abort_latched)

        self.mav.abort_requested = False
        self.assertEqual(guard.update(), py_trees.common.Status.SUCCESS)

        self.mav._on_start(Bool(data=True))
        self.assertFalse(self.mav.abort_latched)
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)

    def test_guard_trips_on_excessive_attitude(self):
        """The exact failure from the last live run: 54 deg against a wall."""
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True)
        guard = CheckAbortTriggered(self.mav, self.node)
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        for _ in range(self.mav.attitude_limit_samples):
            self.mav._on_pose(pose_with(roll_deg=-53.8, pitch_deg=5.9))
        self.assertEqual(guard.update(), py_trees.common.Status.SUCCESS)
        self.assertIn("Excessive attitude", self.mav.abort_reason)


# --------------------------------------------------------------------------- #
# Rail 4: the tree actually returns to WAITING
# --------------------------------------------------------------------------- #
class TestTreeReset(RailTestCase):

    def _tick(self, root, n=1):
        for _ in range(n):
            root.tick_once()
        return root.tip()

    def test_tree_advances_then_resets_to_waiting(self):
        root = build_root(self.mav, self.node, DEFAULT_PARAMS)

        # Idle: parked at WaitForMissionStart, no setpoints.
        self.assertEqual(self._tick(root).name, "WaitForMissionStart")

        # Start the mission and let it reach Takeoff.
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self.mav._on_pose(pose_with(z=0.2))
        tip = self._tick(root, 3)
        self.assertEqual(tip.name, "Takeoff",
                         f"expected Takeoff, tree sat at {tip.name}")

        # Somebody disarms the aircraft from Mission Planner / the safety pilot.
        self.mav._on_state(state_msg(armed=False, mode="GUIDED"))
        reason = apply_pending_reset(root, self.mav)
        self.assertEqual(reason, "external disarm")

        # The memory Sequence must NOT resume at Takeoff.
        tip = self._tick(root)
        self.assertEqual(tip.name, "WaitForMissionStart",
                         f"tree retained progress after intervention: {tip.name}")

    def test_reset_latches_interrupted_result(self):
        root = build_root(self.mav, self.node, DEFAULT_PARAMS)
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self._tick(root, 3)
        self.mav._on_state(state_msg(armed=False, mode="GUIDED"))
        apply_pending_reset(root, self.mav)
        self.assertEqual(len(self.published_results), 1)
        self.assertIn("INTERRUPTED", self.published_results[0].data)
        self.assertIn("external disarm", self.published_results[0].data)

    def test_no_reset_means_no_interference(self):
        root = build_root(self.mav, self.node, DEFAULT_PARAMS)
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self._tick(root, 3)
        self.assertIsNone(apply_pending_reset(root, self.mav))
        self.assertEqual(root.tip().name, "Takeoff")

    def test_restart_after_reset_is_deterministic(self):
        """Second run must begin at ARMING, not resume mid-mission."""
        root = build_root(self.mav, self.node, DEFAULT_PARAMS)
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=True, mode="GUIDED")
        self._tick(root, 3)
        self.mav._on_state(state_msg(armed=False, mode="GUIDED"))
        apply_pending_reset(root, self.mav)
        self._tick(root)

        # Operator presses START again.
        self.mav.mission_started = True
        self.mav.state = state_msg(armed=False, mode="STABILIZE")
        tip = self._tick(root, 2)
        self.assertEqual(tip.name, "SetModeArm",
                         f"restart resumed at {tip.name} instead of arming")


# --------------------------------------------------------------------------- #
# Rail 5: mission outcome is latched and honest
# --------------------------------------------------------------------------- #
class TestMissionResult(RailTestCase):

    def test_result_is_latched_first_writer_wins(self):
        self.mav.publish_result("ABORTED_RTL", "battery critical")
        self.mav.publish_result("COMPLETED", "landed and disarmed")
        self.assertEqual(len(self.published_results), 1)
        self.assertIn("ABORTED_RTL", self.published_results[0].data)
        self.assertEqual(self.mav.result, ("ABORTED_RTL", "battery critical"))

    def test_result_payload_is_json_with_reason(self):
        import json
        self.mav.publish_result("INTERRUPTED", "external disarm")
        payload = json.loads(self.published_results[0].data)
        self.assertEqual(payload["state"], "INTERRUPTED")
        self.assertEqual(payload["reason"], "external disarm")
        self.assertIn("t", payload)

    def test_new_start_clears_previous_result(self):
        from std_msgs.msg import Bool
        self.mav.publish_result("COMPLETED", "landed and disarmed")
        self.mav._on_start(Bool(data=True))
        self.mav.publish_result("ABORTED_RTL", "battery critical")
        self.assertEqual(len(self.published_results), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
