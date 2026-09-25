#!/usr/bin/env python3
"""The generated vehicle must attach its sensor head to the real airframe link.

Upstream ships `iris_with_gimbal` in two shapes. One merges the airframe links
into the model, so `base_link` is a direct child. The one installed on the WSL
development machine `<include>`s iris_with_standoffs, so the link is addressed
as `iris_with_standoffs::base_link`.

The materialiser emitted `<parent>base_link</parent>` unconditionally. Against
the nested shape those joints resolve to nothing, Gazebo silently drops the
lidar and webcam mounts, and the world comes up with no `/scan` and no
`/camera/image` — with no error that names the cause. It was patched by hand in
`.scratch/open_gazebo.sh`, which repaired the viewing model only and left the
model the launcher actually flies broken.

The same nesting scopes the gimbal joints as `gimbal::roll_joint`, so removing
plugins by the bare names roll/pitch/yaw_joint left stale PID controllers
driving joints that had just been removed.
"""

import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SCRIPT = os.path.join(_ROOT, "scripts", "materialize_vehicle_model.py")

NESTED = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="iris_with_gimbal">
    <include><uri>model://iris_with_standoffs</uri><name>iris_with_standoffs</name></include>
    <include><uri>model://gimbal_small_3d</uri><name>gimbal</name></include>
    <include><uri>model://lidar_2d_v2</uri><name>lidar_2d</name></include>
    <joint name="gimbal_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent><child>gimbal::base_link</child>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="c1">
      <joint_name>gimbal::roll_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system" name="c2">
      <joint_name>gimbal::pitch_joint</joint_name>
    </plugin>
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <control channel="0"><jointName>rotor_0_joint</jointName></control>
      <control channel="8"><jointName>gimbal::roll_joint</jointName></control>
    </plugin>
  </model>
</sdf>
"""

MERGED = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="iris_with_gimbal">
    <link name="base_link"/>
    <include><uri>model://gimbal_small_3d</uri><name>gimbal</name></include>
    <joint name="gimbal_joint" type="fixed">
      <parent>base_link</parent><child>gimbal::base_link</child>
    </joint>
    <plugin filename="x" name="c1"><joint_name>roll_joint</joint_name></plugin>
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <control channel="0"><jointName>rotor_0_joint</jointName></control>
      <control channel="9"><jointName>yaw_joint</jointName></control>
    </plugin>
  </model>
</sdf>
"""

MERGE_INCLUDE = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="iris_with_gimbal">
    <include merge="true">
      <uri>package://ardupilot_gazebo/models/iris_with_standoffs</uri><name>iris</name>
    </include>
    <include merge="true">
      <uri>package://ardupilot_gazebo/models/gimbal_small_3d</uri><name>gimbal</name>
    </include>
    <joint name="gimbal_joint" type="revolute">
      <parent>base_link</parent><child>gimbal_link</child>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="c1">
      <joint_name>roll_joint</joint_name>
    </plugin>
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <control channel="0"><jointName>rotor_0_joint</jointName></control>
      <control channel="8"><jointName>roll_joint</jointName></control>
    </plugin>
  </model>
</sdf>
"""

MOUNTS = {"rplidar_c1_mount", "webcam_servo_mount"}


def materialise(source_xml):
    tmp = tempfile.mkdtemp(prefix="aerothon_vehicle_")
    source = os.path.join(tmp, "model.sdf")
    with open(source, "w", encoding="utf-8") as handle:
        handle.write(source_xml)
    subprocess.run([sys.executable, _SCRIPT, "--source", source,
                    "--output-root", os.path.join(tmp, "out")],
                   check=True, capture_output=True)
    out = os.path.join(tmp, "out", "aerothon_iris_c1_webcam", "model.sdf")
    return ET.parse(out).find("model")


class VehicleModelMaterialisation(unittest.TestCase):
    def _mount_parents(self, model):
        return {j.get("name"): j.findtext("parent")
                for j in model.findall("joint") if j.get("name") in MOUNTS}

    def test_nested_airframe_gets_the_scoped_base_link(self):
        parents = self._mount_parents(materialise(NESTED))
        self.assertEqual(set(parents), MOUNTS, "sensor mounts are missing")
        for name, parent in parents.items():
            self.assertEqual(
                parent, "iris_with_standoffs::base_link",
                f"{name} hangs off '{parent}', which does not exist in the "
                "nested upstream model — Gazebo drops the joint and the run "
                "comes up with no /scan and no /camera/image")

    def test_merged_airframe_keeps_the_plain_base_link(self):
        parents = self._mount_parents(materialise(MERGED))
        self.assertEqual(set(parents), MOUNTS, "sensor mounts are missing")
        for name, parent in parents.items():
            self.assertEqual(parent, "base_link", f"{name} lost its parent link")

    def test_a_merge_include_keeps_the_plain_base_link(self):
        """A merge-include's <name> is not a frame prefix."""
        parents = self._mount_parents(materialise(MERGE_INCLUDE))
        self.assertEqual(set(parents), MOUNTS, "sensor mounts are missing")
        for name, parent in parents.items():
            self.assertEqual(
                parent, "base_link",
                f"{name} hangs off '{parent}'. <include merge=\"true\"> splices "
                "the included links into this model's scope, so that scoped "
                "frame does not exist; gz-sim then refuses the whole world "
                "with Error Code 21, never advertises /gazebo/worlds, and the "
                "launcher times out waiting for it")

    def test_no_plugin_drives_a_removed_gimbal_joint(self):
        for label, source in (("nested", NESTED), ("merged", MERGED),
                                  ("merge-include", MERGE_INCLUDE)):
            with self.subTest(label):
                model = materialise(source)
                joints = {j.get("name") for j in model.findall("joint")}
                for plugin in model.findall("plugin"):
                    if plugin.get("name") == "ArduPilotPlugin":
                        continue
                    for node in plugin.iter("joint_name"):
                        self.assertIn(
                            node.text, joints,
                            f"{label}: a plugin still drives '{node.text}', "
                            "a joint this variant no longer has")

    def test_gimbal_control_channels_are_dropped_from_ardupilot(self):
        for label, source in (("nested", NESTED), ("merged", MERGED),
                                  ("merge-include", MERGE_INCLUDE)):
            with self.subTest(label):
                model = materialise(source)
                plugin = [p for p in model.findall("plugin")
                          if p.get("name") == "ArduPilotPlugin"][0]
                channels = [int(c.get("channel")) for c in plugin.findall("control")]
                self.assertEqual(channels, [0],
                                 f"{label}: gimbal servo channels survived")

    def test_the_camera_pitch_joint_is_still_driven(self):
        """The removal pass must not take the competition webcam with it."""
        for label, source in (("nested", NESTED), ("merged", MERGED),
                                  ("merge-include", MERGE_INCLUDE)):
            with self.subTest(label):
                model = materialise(source)
                driven = {n.text for p in model.findall("plugin")
                          for n in p.iter("joint_name")}
                self.assertIn("webcam_pitch_joint", driven,
                              f"{label}: nothing drives the webcam pitch joint")


if __name__ == "__main__":
    unittest.main()
