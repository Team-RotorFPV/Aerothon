#!/usr/bin/env python3
"""Phase 2 — camera pointing as a commanded, CONFIRMED state.

The defect being closed (CURRENT_PROGRESS_HANDOFF.md): the mission never
commanded the camera downward, so the ground QR was outside the field of view
and `ScanStartQR` "ran" against a picture of a wall and the sky.

The rule these tests enforce: `settled` is true only when the joint has been
MEASURED at the requested angle. Sending a command proves nothing.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_camera_ctrl.py -v
"""

import json
import math
import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "src", "aerothon_perception", "camera_ctrl"))
sys.path.insert(0, os.path.join(_ROOT, "src", "aerothon_mission", "mission_bt"))

import rclpy
from rclpy.node import Node
import py_trees
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from camera_ctrl.camera_ctrl_node import CameraCtrl, NAMED_POSES_DEG
from mission_bt.mission_tree import SetCameraPose


def joint_state(angle_rad, name="webcam_pitch_joint"):
    js = JointState()
    js.name = ["imu_joint", "rotor_0_joint", name]
    js.position = [0.0, 0.0, float(angle_rad)]
    return js


class CameraCtrlTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = CameraCtrl()
        self.commands = []
        self.states = []
        self.node.pub_cmd.publish = self.commands.append
        self.node.pub_state.publish = self.states.append
        # These tests feed joint samples in a tight loop, so no wall-clock time
        # passes between them. Drop the hold requirement here and cover it
        # explicitly in test_settle_hold_time_is_enforced below.
        self.set_hold(0.0)

    def set_hold(self, seconds):
        from rclpy.parameter import Parameter
        self.node.set_parameters(
            [Parameter("settle_hold_s", Parameter.Type.DOUBLE, float(seconds))])

    def tearDown(self):
        self.node.destroy_node()

    def last_state(self):
        self.node._tick()
        return json.loads(self.states[-1].data)

    # ---- named poses ---- #

    def test_named_poses_match_goal_md(self):
        self.assertEqual(NAMED_POSES_DEG["FORWARD"], 0.0)
        self.assertEqual(NAMED_POSES_DEG["NADIR"], -90.0)
        self.assertEqual(NAMED_POSES_DEG["ALIGN"], -45.0)

    def test_request_publishes_radians(self):
        self.commands.clear()
        self.node.request("NADIR")
        self.assertTrue(self.commands)
        self.assertAlmostEqual(self.commands[-1].data, -math.pi / 2, places=6)

    def test_raw_angle_accepted(self):
        self.node.request("-30")
        self.assertAlmostEqual(self.node.requested_rad, math.radians(-30), places=6)

    def test_unknown_pose_is_ignored_not_guessed(self):
        self.node.request("NADIR")
        before = self.node.requested_rad
        self.node.request("SIDEWAYS")
        self.assertEqual(self.node.requested_rad, before,
                         "an unrecognised pose must not silently move the camera")

    # ---- the core rail: settled requires MEASUREMENT ---- #

    def test_not_settled_before_any_readback(self):
        self.node.request("NADIR")
        st = self.last_state()
        self.assertFalse(st["settled"],
                         "settled must not be true merely because a command was sent")
        self.assertTrue(st["stale"])
        self.assertIsNone(st["actual_rad"])

    def test_not_settled_while_joint_is_elsewhere(self):
        """The exact handoff failure: commanded down, still pointing forward."""
        self.node.request("NADIR")
        for _ in range(50):
            self.node._on_joint_states(joint_state(0.0))    # stuck FORWARD
        st = self.last_state()
        self.assertFalse(st["settled"])
        self.assertAlmostEqual(st["error_deg"], 90.0, places=1)

    def test_settles_once_joint_reaches_target_and_holds(self):
        self.node.request("NADIR")
        need = int(self.node.get_parameter("settle_samples").value)
        for _ in range(need + 5):
            self.node._on_joint_states(joint_state(-math.pi / 2))
        st = self.last_state()
        self.assertTrue(st["settled"], f"should have settled: {st}")
        self.assertLess(st["error_deg"], 1.0)

    def test_single_in_tolerance_sample_is_not_enough(self):
        self.node.request("NADIR")
        self.node._on_joint_states(joint_state(-math.pi / 2))
        self.assertFalse(self.node.is_settled(),
                         "one sample must not count as settled")

    def test_settle_hold_time_is_enforced(self):
        """Sample count alone is not enough; the pose must HOLD.

        A servo sweeping through the target angle can satisfy N consecutive
        in-tolerance samples in a few milliseconds without ever stopping there.
        """
        import time
        self.set_hold(0.30)
        need = int(self.node.get_parameter("settle_samples").value)
        self.node.request("NADIR")
        for _ in range(need + 5):
            self.node._on_joint_states(joint_state(-math.pi / 2))
        self.assertFalse(self.node.is_settled(),
                         "settled before the hold time elapsed")
        time.sleep(0.35)
        self.node._on_joint_states(joint_state(-math.pi / 2))
        self.assertTrue(self.node.is_settled(),
                        "should settle once the pose has been held")

    def test_tolerance_boundary(self):
        tol = float(self.node.get_parameter("tolerance_deg").value)
        need = int(self.node.get_parameter("settle_samples").value)
        self.node.request("FORWARD")
        for _ in range(need + 3):
            self.node._on_joint_states(joint_state(math.radians(tol * 0.5)))
        self.assertTrue(self.node.is_settled(), "inside tolerance should settle")

        self.node.request("NADIR")
        for _ in range(need + 3):
            self.node._on_joint_states(
                joint_state(-math.pi / 2 + math.radians(tol * 3)))
        self.assertFalse(self.node.is_settled(), "outside tolerance must not settle")

    def test_new_request_clears_previous_settle(self):
        need = int(self.node.get_parameter("settle_samples").value)
        self.node.request("FORWARD")
        for _ in range(need + 3):
            self.node._on_joint_states(joint_state(0.0))
        self.assertTrue(self.node.is_settled())

        self.node.request("NADIR")
        self.assertFalse(self.node.is_settled(),
                         "settled leaked across a pose change")

    def test_drift_out_of_tolerance_unsettles(self):
        need = int(self.node.get_parameter("settle_samples").value)
        self.node.request("NADIR")
        for _ in range(need + 3):
            self.node._on_joint_states(joint_state(-math.pi / 2))
        self.assertTrue(self.node.is_settled())
        self.node._on_joint_states(joint_state(-math.pi / 4))   # servo slipped
        self.assertFalse(self.node.is_settled())

    def test_wrong_joint_name_is_not_read(self):
        self.node.request("NADIR")
        for _ in range(20):
            self.node._on_joint_states(joint_state(-math.pi / 2, name="some_other"))
        self.assertFalse(self.node.is_settled())
        self.assertIsNone(self.node.actual_rad)

    @staticmethod
    def _joint_axis_and_limits(text):
        """Extract (axis_y, lower, upper) from an SDF or URDF joint block."""
        import re
        sdf = re.search(r"<xyz>\s*0\s+(-?1)\s+0\s*</xyz>", text)
        urdf = re.search(r'axis xyz="0 (-?1) 0"', text)
        axis_y = float((sdf or urdf).group(1))
        lower = float(re.search(r"(?:<lower>|lower=\")(-?[0-9.]+)", text).group(1))
        upper = float(re.search(r"(?:<upper>|upper=\")(-?[0-9.]+)", text).group(1))
        return axis_y, lower, upper

    @staticmethod
    def _forward_after_pitch(axis_y, angle_rad):
        """Camera forward (+x) after rotating by angle about (0, axis_y, 0).

        Rotation about +Y by t maps x_hat -> (cos t, 0, -sin t) in FLU.
        A -Y axis is equivalent to negating the angle.
        """
        t = angle_rad * axis_y
        return (math.cos(t), 0.0, -math.sin(t))

    def test_NADIR_actually_points_at_the_ground(self):
        """The bug Phase 2 missed: the joint reached -90 deg and looked at SKY.

        Phase 2 verified that the commanded angle was achieved. It never
        verified what the camera was consequently LOOKING AT. With the axis as
        +Y, -90 deg rotates the camera's forward vector to +Z (up), so a nadir
        command aimed it at the sky — confirmed by capturing a frame, which was
        blank blue. Assert the resulting direction, not the joint number.
        """
        for path in (os.path.join(_ROOT, "scripts", "materialize_vehicle_model.py"),
                     os.path.join(_ROOT, "src", "aerothon_mission",
                                  "uav_description", "urdf", "uav.urdf.xacro")):
            text = open(path).read()
            axis_y, lower, upper = self._joint_axis_and_limits(text)
            nadir = math.radians(NAMED_POSES_DEG["NADIR"])
            fwd = self._forward_after_pitch(axis_y, nadir)
            self.assertLess(
                fwd[2], -0.9,
                f"{os.path.basename(path)}: at NADIR the camera forward vector "
                f"is {fwd} — z must be strongly NEGATIVE (down). "
                f"axis_y={axis_y}")

    def test_FORWARD_points_at_the_horizon(self):
        for path in (os.path.join(_ROOT, "scripts", "materialize_vehicle_model.py"),
                     os.path.join(_ROOT, "src", "aerothon_mission",
                                  "uav_description", "urdf", "uav.urdf.xacro")):
            axis_y, _, _ = self._joint_axis_and_limits(open(path).read())
            fwd = self._forward_after_pitch(axis_y, 0.0)
            self.assertGreater(fwd[0], 0.9, "FORWARD must look along +x")
            self.assertAlmostEqual(fwd[2], 0.0, places=6)

    def test_sdf_and_urdf_axes_agree(self):
        a = self._joint_axis_and_limits(
            open(os.path.join(_ROOT, "scripts", "materialize_vehicle_model.py")).read())
        b = self._joint_axis_and_limits(
            open(os.path.join(_ROOT, "src", "aerothon_mission", "uav_description",
                              "urdf", "uav.urdf.xacro")).read())
        self.assertEqual(a, b,
                         "Gazebo model and RViz URDF disagree on the camera "
                         "joint axis/limits; RViz would show a different "
                         "camera orientation from the one actually flown")

    def test_nadir_is_inside_the_joint_limit(self):
        """-90 deg must be reachable, not sitting on the hard stop.

        The SDF/URDF lower limit was widened to -1.65 rad precisely so a NADIR
        command can settle. If someone narrows it back to -1.570796 the joint
        can never read back inside tolerance.
        """
        import re
        sdf = open(os.path.join(_ROOT, "scripts",
                                "materialize_vehicle_model.py")).read()
        m = re.search(r"<lower>(-?[0-9.]+)</lower>", sdf)
        self.assertIsNotNone(m, "could not find joint lower limit")
        lower = float(m.group(1))
        self.assertLess(lower, -math.pi / 2,
                        f"lower limit {lower} does not leave room for a -90 deg NADIR")


class MockCameraMav:
    def __init__(self):
        self.camera_state = {}
        self.requested = []
        self.abort_reason = ""

    def set_camera_pose(self, pose):
        self.requested.append(pose)

    def camera_settled(self, pose=None):
        st = self.camera_state
        if not st or not st.get("settled") or st.get("stale"):
            return False
        if pose is not None and st.get("requested") != pose:
            return False
        return True

    def camera_state_summary(self):
        return str(self.camera_state)


class SetCameraPoseLeafTests(unittest.TestCase):
    """The behaviour-tree gate: a stage must block on an unconfirmed camera."""

    def setUp(self):
        self.mav = MockCameraMav()
        self.leaf = SetCameraPose("CameraNadir", self.mav, "NADIR",
                                  timeout_ticks=10)

    def tick(self, n=1):
        """Tick up to n times, stopping at the first terminal status.

        py_trees re-initialises a leaf whose status is not RUNNING, so ticking
        past a FAILURE resets the leaf and hides it. A parent Sequence would
        have propagated that FAILURE on the tick it occurred.
        """
        for _ in range(n):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        return self.leaf.status

    def test_commands_the_pose_on_entry(self):
        self.tick()
        self.assertIn("NADIR", self.mav.requested)

    def test_blocks_while_not_settled(self):
        self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)

    def test_succeeds_once_settled_at_that_pose(self):
        self.mav.camera_state = {"requested": "NADIR", "settled": True,
                                 "stale": False}
        self.assertEqual(self.tick(), py_trees.common.Status.SUCCESS)

    def test_settled_at_a_DIFFERENT_pose_does_not_count(self):
        self.mav.camera_state = {"requested": "FORWARD", "settled": True,
                                 "stale": False}
        self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)

    def test_stale_readback_does_not_count(self):
        self.mav.camera_state = {"requested": "NADIR", "settled": True,
                                 "stale": True}
        self.assertEqual(self.tick(), py_trees.common.Status.RUNNING)

    def test_fails_closed_on_timeout(self):
        """Must FAIL, not proceed — the ScanStartQR mistake, not repeated."""
        status = self.tick(12)
        self.assertEqual(status, py_trees.common.Status.FAILURE)
        self.assertIn("camera did not reach NADIR", self.mav.abort_reason)


if __name__ == "__main__":
    unittest.main(verbosity=2)
