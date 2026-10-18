#!/usr/bin/env python3
"""Wiring that only the real aircraft exercises.

Each of these passed every simulated run and failed on the Pi (16 Oct 2026):

* The runtime launched the LD06 driver for a fitted RPLIDAR C1. The C1 spins
  only after its driver's start-scan command, so it stayed still and SLAM,
  readiness and avoidance received nothing.
* Readiness subscribed to /scan RELIABLE while scan_mount publishes it
  BEST_EFFORT, so "Lidar rate" read 0.0 Hz forever (Gazebo hid it).
* QR read /camera/camera_info; usb_cam on the aircraft publishes /camera_info.
* The GCS camera buttons sent "0.0"/"-90.0", which camera_ctrl named
  "0.0deg"; no detector gate accepts that name.
* An Abort pressed while idle stayed latched and failed the next flight.
* Set target published only /mission/target; the mission reads
  /mission/target_override.
* /mission/state kept the last stage after a run, locking GCS controls and
  refusing arena dimensions until reconnect.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_real_path_wiring.py -v
"""

import importlib.util
import json
import os
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parent.parent
for pkg in ("aerothon_perception/camera_ctrl", "aerothon_mission/mission_bt",
            "aerothon_gcs/gcs_aggregator", "aerothon_runtime"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

LAUNCH = ROOT / "src/aerothon_mission/mission_bringup/launch/mission2.launch.py"

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node as LaunchNode
except ImportError:                                   # no ROS on this host
    LaunchContext = None


def started(**args):
    """([started launch nodes], context) for mission2.launch.py with args."""
    spec = importlib.util.spec_from_file_location("mission2_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    try:
        ld = mod.generate_launch_description()
    except Exception as e:                            # noqa: BLE001
        raise unittest.SkipTest(f"workspace not built/sourced: {e}")
    ctx = LaunchContext()
    for a in ld.entities:
        if isinstance(a, DeclareLaunchArgument):
            ctx.launch_configurations[a.name] = args.get(
                a.name, perform_substitutions(ctx, a.default_value or []))
    return [a for a in ld.entities if isinstance(a, LaunchNode)
            and (a.condition is None or a.condition.evaluate(ctx))], ctx


def launched(**args):
    """{package: node} for the nodes mission2.launch.py starts with args."""
    nodes, ctx = started(**args)
    return {a.node_package: a for a in nodes}, ctx


def launched_executable(executable, **args):
    """The started node running `executable`, for packages with several."""
    nodes, ctx = started(**args)
    for a in nodes:
        if a.node_executable == executable:
            return a, ctx
    raise AssertionError(f"{executable} not started")


def param(node, ctx, name):
    for group in node._Node__parameters:
        if isinstance(group, dict):
            for key, value in group.items():
                if perform_substitutions(ctx, key) == name:
                    if hasattr(value, 'value_type'):          # ParameterValue
                        value = value.value
                    if isinstance(value, (list, tuple)) or hasattr(value, 'perform'):
                        # launch_ros hands strings back YAML-encoded.
                        import yaml
                        return yaml.safe_load(perform_substitutions(ctx, value))
                    return value
    raise KeyError(name)


@unittest.skipIf(LaunchContext is None, "ROS 2 launch not installed")
class LaunchSelectsTheFittedHardware(unittest.TestCase):

    def test_the_aircraft_runs_the_c1_driver_by_default(self):
        nodes, _ = launched(use_sim="false")
        self.assertIn("sllidar_ros2", nodes)
        self.assertNotIn("ldlidar_stl_ros2", nodes)

    def test_the_ld06_remains_selectable(self):
        nodes, _ = launched(use_sim="false", lidar_model="ld06")
        self.assertIn("ldlidar_stl_ros2", nodes)
        self.assertNotIn("sllidar_ros2", nodes)

    def test_simulation_starts_no_lidar_driver(self):
        nodes, _ = launched(use_sim="true")
        self.assertNotIn("sllidar_ros2", nodes)
        self.assertNotIn("ldlidar_stl_ros2", nodes)

    def test_c1_publishes_the_raw_scan_scan_mount_reads(self):
        nodes, ctx = launched(use_sim="false")
        c1 = nodes["sllidar_ros2"]
        self.assertEqual(param(c1, ctx, "serial_baudrate"), 460800)
        self.assertEqual(param(c1, ctx, "frame_id"), "base_scan")
        remaps = [(perform_substitutions(ctx, a), perform_substitutions(ctx, b))
                  for a, b in c1._Node__remappings]
        self.assertIn(("scan", "scan_driver"), remaps)

    def test_scan_mount_corrects_each_drivers_frame(self):
        """sllidar_ros2 publishes the C1's marked front at pi; ldlidar's LD06
        frame is used as published. scan_mount turns /scan_driver into
        /scan_raw with the lidar's own front at bearing 0."""
        for model, driver_yaw in (("c1", 180.0), ("ld06", 0.0)):
            with self.subTest(model=model):
                mount, ctx = launched_executable("scan_mount", use_sim="false",
                                                 lidar_model=model)
                self.assertEqual(param(mount, ctx, "driver_yaw_deg"), driver_yaw)
        nodes, ctx = launched(use_sim="false", lidar_model="ld06")
        self.assertEqual(param(nodes["ldlidar_stl_ros2"], ctx, "topic_name"), "scan_driver")

    def test_qr_reads_the_camera_info_each_path_publishes(self):
        nodes, ctx = launched(use_sim="false")
        self.assertEqual(param(nodes["perception_qr"], ctx, "camera_info_topic"), "/camera_info")
        nodes, ctx = launched(use_sim="true")
        self.assertEqual(param(nodes["perception_qr"], ctx, "camera_info_topic"),
                         "/camera/camera_info")

    def test_aircraft_images_carry_a_frame_the_urdf_defines(self):
        nodes, ctx = launched(use_sim="false")
        frame = param(nodes["usb_cam"], ctx, "frame_id")
        urdf = (ROOT / "src/aerothon_mission/uav_description/urdf/uav.urdf.xacro").read_text()
        self.assertIn(f'name="{frame}"', urdf)


class ReadinessHearsTheBestEffortScan(unittest.TestCase):

    def test_scan_subscription_is_sensor_data_qos(self):
        from rclpy.qos import ReliabilityPolicy
        subs = []
        with patch('rclpy.node.Node.__init__', return_value=None), \
             patch('rclpy.node.Node.create_subscription',
                   side_effect=lambda *a, **k: subs.append(a)), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.get_parameter', return_value=MagicMock(value=1.0)), \
             patch('rclpy.node.Node.get_logger'), \
             patch('rclpy.node.Node.get_clock'):
            from gcs_aggregator.readiness_node import Readiness
            try:
                Readiness()
            except Exception:                         # noqa: BLE001
                pass                                  # only the subscriptions matter
        scan = [a for a in subs if a[1] == "/scan"]
        self.assertTrue(scan, "readiness must subscribe to /scan")
        qos = scan[0][3]
        self.assertFalse(isinstance(qos, int), "a depth means RELIABLE: no data from scan_mount")
        self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)


class CameraButtonsUseNamedPoses(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.owns_context = not rclpy.ok()
        if cls.owns_context:
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Later modules call rclpy.init() themselves; a context left open
        # here fails their setup with "must only be called once".
        import rclpy
        if cls.owns_context and rclpy.ok():
            rclpy.shutdown()

    def test_degrees_matching_a_named_pose_carry_its_name(self):
        from camera_ctrl.camera_ctrl_node import CameraCtrl
        node = CameraCtrl()
        try:
            node.pub_cmd.publish = lambda *_: None
            node.pub_state.publish = lambda *_: None
            node.request("-90.0")
            self.assertEqual(node.requested_name, "NADIR")
            node.request("0.0")
            self.assertEqual(node.requested_name, "FORWARD")
            node.request("-10")
            self.assertEqual(node.requested_name, "-10.0deg")
        finally:
            node.destroy_node()


def commander():
    from mission_bt.mav_commander import Mav
    mav = Mav.__new__(Mav)
    mav.node = MagicMock()
    mav.mission_started = False
    mav.mission_seq = 0
    mav.clear_airborne_floor = MagicMock()
    mav.set_target = MagicMock()
    return mav


class AbortDoesNotPoisonTheNextFlight(unittest.TestCase):

    def test_an_old_idle_abort_is_cleared_by_start(self):
        mav = commander()
        mav._on_abort(NS(data=True))
        mav._abort_t = time.monotonic() - 5.0
        mav._on_start(NS(data=True))
        self.assertFalse(mav.abort_requested)

    def test_an_abort_racing_the_start_still_stands(self):
        mav = commander()
        mav._on_abort(NS(data=True))
        mav._on_start(NS(data=True))
        self.assertTrue(mav.abort_requested)


class SetTargetReachesTheMission(unittest.TestCase):

    def test_operator_target_is_published_as_the_override(self):
        from sim import test_gcs_aggregator as support
        case = support.TestGCSAggregator()
        case.setUp()
        node = case.node
        node.pub_target, node.pub_target_override = MagicMock(), MagicMock()
        node._dispatch("set_target", {"target": "AEROTHON2026:M2:TARGET_E"})
        self.assertEqual(node.pub_target_override.publish.call_args.args[0].data,
                         "AEROTHON2026:M2:TARGET_E")


class MissionStateEndsWithTheRun(unittest.TestCase):

    def execution(self, state, last):
        from aerothon_runtime.execution_node import ExecutionNode
        node = ExecutionNode.__new__(ExecutionNode)
        node.run = {'state': state}
        node.pending_stop_id = None
        node.status_pub, node.state_pub = MagicMock(), MagicMock()
        node.last_state = {'value': last}
        node.publish()
        return node

    def test_a_finished_run_publishes_its_terminal_state(self):
        for state, final in (('Completed', 'COMPLETED'), ('Failed', 'FAILED')):
            node = self.execution(state, 'RETURN')
            self.assertEqual(node.last_state['value'], final)
            self.assertEqual(node.state_pub.publish.call_args.args[0].data, final)

    def test_running_and_unknown_outcomes_keep_the_stage(self):
        for state in ('Running', 'Outcome unknown'):
            node = self.execution(state, 'RETURN')
            self.assertEqual(node.last_state['value'], 'RETURN')
            node.state_pub.publish.assert_not_called()


def respawns(node):
    # Jazzy keeps it on ExecuteLocal; older launch on ExecuteProcess.
    for name in ("_ExecuteLocal__respawn", "_ExecuteProcess__respawn"):
        if hasattr(node, name):
            return bool(getattr(node, name))
    raise AttributeError("respawn flag not found")


@unittest.skipIf(LaunchContext is None, "ROS 2 launch not installed")
class SensorDriversComeBack(unittest.TestCase):
    """16 Oct: the C1 driver exited when the lidar was plugged in after
    Connect (or USB reset after a brown-out) and stayed dead: no scan, and the
    C1 does not spin without its driver."""

    def test_lidar_and_camera_drivers_respawn(self):
        for model in ("c1", "ld06"):
            nodes, _ = launched(use_sim="false", lidar_model=model)
            pkg = "sllidar_ros2" if model == "c1" else "ldlidar_stl_ros2"
            self.assertTrue(respawns(nodes[pkg]), pkg)
        self.assertTrue(respawns(nodes["usb_cam"]))


if __name__ == "__main__":
    unittest.main()
