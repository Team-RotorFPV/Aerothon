"""mission2.launch.py's hardware-facing defaults follow use_sim.

With fixed defaults, `use_sim:=false` alone drove the Gazebo winch joint and
camera joint while the aircraft's winch and tilt servo got nothing, and
started SLAM and RViz on the Pi.
"""

import importlib.util
import unittest
from pathlib import Path

LAUNCH = (Path(__file__).resolve().parent.parent
          / "src/aerothon_mission/mission_bringup/launch/mission2.launch.py")

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
except ImportError:                                   # no ROS on this host
    LaunchContext = None


def defaults(use_sim):
    spec = importlib.util.spec_from_file_location("mission2_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    try:
        ld = mod.generate_launch_description()
    except Exception as e:                            # noqa: BLE001
        raise unittest.SkipTest(f"workspace not built/sourced: {e}")
    ctx = LaunchContext()
    ctx.launch_configurations["use_sim"] = use_sim
    return {a.name: perform_substitutions(ctx, a.default_value)
            for a in ld.entities if isinstance(a, DeclareLaunchArgument)
            and a.name in ("rviz", "slam", "winch_backend", "camera_backend")}


@unittest.skipIf(LaunchContext is None, "ROS 2 launch not installed")
class LaunchDefaultsTests(unittest.TestCase):

    def test_the_aircraft_drives_real_hardware_and_skips_the_desktop(self):
        self.assertEqual(defaults("false"), {
            "rviz": "false", "slam": "false",
            "winch_backend": "mavlink", "camera_backend": "mavlink"})

    def test_the_simulator_keeps_its_backends(self):
        self.assertEqual(defaults("true"), {
            "rviz": "true", "slam": "true",
            "winch_backend": "gazebo", "camera_backend": "sim"})


class VenueFileTests(unittest.TestCase):
    """Every venue.yaml key is a parameter its node declares: a misspelt one
    is silently ignored, and the node flies on its default."""

    NODES = {
        "mission_bt": "src/aerothon_mission/mission_bt/mission_bt/mission_tree.py",
        "winch_ctrl": "src/aerothon_payload/winch_ctrl/winch_ctrl/winch_node.py",
        "perception_payload": ("src/aerothon_perception/perception_redzone/"
                               "perception_redzone/payload_node.py"),
    }

    def test_every_key_is_declared(self):
        import re
        import yaml
        root = LAUNCH.parents[4]
        venue = yaml.safe_load((LAUNCH.parents[1] / "config/venue.yaml").read_text())
        self.assertEqual(set(venue), set(self.NODES))
        for node, src in self.NODES.items():
            text = (root / src).read_text()
            declared = set(re.findall(
                r"""(?:\bd|\bp|declare_parameter)\(\s*["'](\w+)["']""", text))
            keys = set(venue[node]["ros__parameters"])
            self.assertEqual(sorted(keys - declared), [], node)


if __name__ == "__main__":
    unittest.main()
