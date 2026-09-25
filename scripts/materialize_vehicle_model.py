#!/usr/bin/env python3
"""Build the competition Iris variant from the maintained ArduPilot model.

The flight dynamics and rotor plugins remain upstream ArduPilot. Only the
stock three-axis gimbal is removed and replaced by the actual competition
layout: a fixed top RPLidar C1 and a front Logitech-style webcam on one pitch
servo.
"""

from __future__ import annotations

import argparse
import os
import xml.etree.ElementTree as ET
from pathlib import Path


COMPETITION_HARDWARE = """
<hardware>
  <link name="base_scan">
    <pose>0 0 0.19 0 0 0</pose>
    <inertial>
      <mass>0.12</mass>
      <inertia><ixx>0.00012</ixx><iyy>0.00012</iyy><izz>0.00018</izz></inertia>
    </inertial>
    <collision name="rplidar_collision">
      <pose>0 0 0.022 0 0 0</pose>
      <geometry><cylinder><radius>0.060</radius><length>0.050</length></cylinder></geometry>
    </collision>
    <visual name="rplidar_c1_white_body">
      <pose>0 0 0.018 0 0 0</pose>
      <geometry><cylinder><radius>0.060</radius><length>0.036</length></cylinder></geometry>
      <material><ambient>0.86 0.88 0.90 1</ambient><diffuse>0.96 0.97 0.98 1</diffuse></material>
    </visual>
    <visual name="rplidar_c1_black_scan_head">
      <pose>0 0 0.043 0 0 0</pose>
      <geometry><cylinder><radius>0.044</radius><length>0.018</length></cylinder></geometry>
      <material><ambient>0.015 0.018 0.022 1</ambient><diffuse>0.025 0.030 0.036 1</diffuse></material>
    </visual>
    <visual name="rplidar_c1_blue_band">
      <pose>0 0 0.037 0 0 0</pose>
      <geometry><cylinder><radius>0.050</radius><length>0.008</length></cylinder></geometry>
      <material><ambient>0.02 0.40 0.75 1</ambient><diffuse>0.02 0.58 0.95 1</diffuse></material>
    </visual>
    <visual name="rplidar_c1_front_marker">
      <pose>0.059 0 0.023 0 0 0</pose>
      <geometry><box><size>0.008 0.035 0.022</size></box></geometry>
      <material><ambient>0.9 0.08 0.03 1</ambient><diffuse>1 0.12 0.04 1</diffuse></material>
    </visual>
    <sensor name="rplidar_c1_scan" type="gpu_lidar">
      <gz_frame_id>base_scan</gz_frame_id>
      <pose>0 0 0.045 0 0 0</pose>
      <topic>/lidar</topic>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <lidar>
        <scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan>
        <range><min>0.05</min><max>12.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
      </lidar>
    </sensor>
  </link>
  <joint name="rplidar_c1_mount" type="fixed">
    <parent>base_link</parent><child>base_scan</child>
  </joint>

  <link name="webcam_servo_base">
    <pose>0.145 0 0.035 0 0 0</pose>
    <inertial><mass>0.035</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00002</izz></inertia></inertial>
    <collision name="servo_collision"><geometry><box><size>0.050 0.075 0.055</size></box></geometry></collision>
    <visual name="front_pitch_servo">
      <geometry><box><size>0.050 0.075 0.055</size></box></geometry>
      <material><ambient>0.04 0.04 0.05 1</ambient><diffuse>0.08 0.08 0.10 1</diffuse></material>
    </visual>
    <visual name="servo_horn">
      <pose>0.029 0 0 0 1.570796 0</pose>
      <geometry><cylinder><radius>0.018</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.85 0.85 0.88 1</ambient><diffuse>0.95 0.95 0.98 1</diffuse></material>
    </visual>
  </link>
  <joint name="webcam_servo_mount" type="fixed">
    <parent>base_link</parent><child>webcam_servo_base</child>
  </joint>

  <link name="webcam_link">
    <pose>0.190 0 0.035 0 0 0</pose>
    <inertial><mass>0.080</mass><inertia><ixx>0.00007</ixx><iyy>0.00005</iyy><izz>0.00008</izz></inertia></inertial>
    <collision name="webcam_collision">
      <pose>0.040 0 0 0 0 0</pose><geometry><box><size>0.080 0.125 0.045</size></box></geometry>
    </collision>
    <visual name="logitech_webcam_body">
      <pose>0.040 0 0 0 0 0</pose><geometry><box><size>0.080 0.125 0.045</size></box></geometry>
      <material><ambient>0.025 0.028 0.032 1</ambient><diffuse>0.045 0.050 0.060 1</diffuse></material>
    </visual>
    <visual name="logitech_blue_face">
      <pose>0.082 0 0 0 1.570796 0</pose><geometry><cylinder><radius>0.020</radius><length>0.008</length></cylinder></geometry>
      <material><ambient>0.02 0.28 0.58 1</ambient><diffuse>0.02 0.48 0.90 1</diffuse></material>
    </visual>
    <visual name="logitech_lens">
      <pose>0.087 0 0 0 1.570796 0</pose><geometry><cylinder><radius>0.012</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.005 0.008 0.012 1</ambient><diffuse>0.01 0.03 0.06 1</diffuse></material>
    </visual>
    <sensor name="logitech_front_camera" type="camera">
      <gz_frame_id>camera_optical_frame</gz_frame_id>
      <pose>0.093 0 0 0 0 0</pose>
      <topic>/camera/image</topic>
      <always_on>true</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <camera>
        <!-- Resolution is substituted at materialize time; see the
             camera_width / camera_height options and the AEROTHON_CAMERA_W and
             AEROTHON_CAMERA_H environment variables.

             WHY IT IS A PARAMETER. Pixels per QR module scales linearly with
             image width, so rendering at 640x480 makes every perception
             measurement roughly 3x pessimistic against goal.md Q14's 1080p
             Brio stream: the Phase 1 envelope put a 0.4 m marker at 1.3 m max
             stand-off, which is an artefact of render size, not a property of
             the aircraft. But 1080p costs real time. Measured here, real time
             factor fell from 0.55 to 0.32 and /camera/image fell from 7.5 Hz
             to 1.45 Hz, too slow to close a perception loop against.

             So: high resolution for MEASUREMENT runs (decode envelopes, static
             characterisation), low resolution for CLOSED LOOP runs (mission
             rehearsal, avoidance) where frame rate matters more than fidelity.
             Convert between them with the px/module formula in
             docs/QR_DECODE_ENVELOPE.md. -->
        <horizontal_fov>@CAMERA_HFOV@</horizontal_fov>
        <image><width>@CAMERA_W@</width><height>@CAMERA_H@</height><format>R8G8B8</format></image>
        <clip><near>0.04</near><far>120</far></clip>
      </camera>
    </sensor>
  </link>
  <joint name="webcam_pitch_joint" type="revolute">
    <pose>0.190 0 0.035 0 0 0</pose>
    <parent>webcam_servo_base</parent><child>webcam_link</child>
    <!-- Lower limit is -1.65 rad (-94.5 deg), not -1.570796 (-90 deg). NADIR is
         a -90 deg command, and a position controller asked to hold exactly at
         its own hard stop settles slightly short of it, so the joint would
         never read back inside tolerance and /camera/pose_state.settled would
         stay false forever. A few degrees of headroom past the useful travel
         costs nothing and makes -90 deg an ordinary, reachable setpoint. -->
    <!-- AXIS SIGN. Rotating about +Y maps the camera's forward vector to
         (cos t, 0, -sin t) in FLU, so with an axis of +Y a POSITIVE angle
         looks down and a NEGATIVE one looks UP.

         goal.md Q18, MAVLink DO_MOUNT_CONTROL and the limits below all use the
         aerospace convention where pitch-down is NEGATIVE. With the axis at +Y
         those disagreed: the model could look only 30 deg down but 94 deg up,
         and a "-90 deg NADIR" command aimed the camera at the sky. Confirmed by
         capturing a frame at NADIR over the start pad — it was blank blue.

         The axis is therefore -Y, which negates the angle and makes -90 deg
         genuinely nadir. Guarded by
         sim/test_camera_ctrl.py::test_NADIR_actually_points_at_the_ground,
         which asserts the resulting VIEW DIRECTION rather than the joint
         angle — the distinction that let this bug through Phase 2. -->
    <axis><xyz>0 -1 0</xyz><limit><lower>-1.65</lower><upper>0.523599</upper><effort>2</effort><velocity>2</velocity></limit><dynamics><damping>0.08</damping></dynamics></axis>
  </joint>
  <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>webcam_pitch_joint</joint_name><topic>/gimbal/direct_pitch</topic><p_gain>8</p_gain><i_gain>0.1</i_gain><d_gain>0.3</d_gain>
  </plugin>
</hardware>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    # Camera resolution trades perception fidelity against simulator speed.
    # Measured on this machine: 640x480 -> RTF 0.55, camera 7.5 Hz;
    #                           1920x1080 -> RTF 0.32, camera 1.45 Hz.
    parser.add_argument("--camera-width", type=int,
                        default=int(os.environ.get("AEROTHON_CAMERA_W", 1280)))
    parser.add_argument("--camera-height", type=int,
                        default=int(os.environ.get("AEROTHON_CAMERA_H", 720)))
    parser.add_argument("--camera-hfov", type=float,
                        default=float(os.environ.get("AEROTHON_CAMERA_HFOV", 1.0472)))
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"No <model> in {args.source}")
    model.set("name", "aerothon_iris_c1_webcam")

    for include in list(model.findall("include")):
        uri = include.findtext("uri", "")
        if "gimbal_small_3d" in uri or "lidar_2d" in uri:
            model.remove(include)
    for joint in list(model.findall("joint")):
        if joint.get("name") == "gimbal_joint":
            model.remove(joint)
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "ArduPilotPlugin":
            for control in list(plugin.findall("control")):
                if int(control.get("channel", "0")) >= 8:
                    plugin.remove(control)
        if plugin.findtext("joint_name", "") in {"roll_joint", "pitch_joint", "yaw_joint"}:
            model.remove(plugin)

    hardware_xml = (COMPETITION_HARDWARE
                    .replace("@CAMERA_W@", str(args.camera_width))
                    .replace("@CAMERA_H@", str(args.camera_height))
                    .replace("@CAMERA_HFOV@", f"{args.camera_hfov:.6f}"))
    hardware = ET.fromstring(hardware_xml)
    for element in list(hardware):
        model.append(element)
    ET.indent(tree, space="  ")

    model_dir = args.output_root / "aerothon_iris_c1_webcam"
    model_dir.mkdir(parents=True, exist_ok=True)
    tree.write(model_dir / "model.sdf", encoding="utf-8", xml_declaration=True)
    (model_dir / "model.config").write_text(
        """<?xml version="1.0"?>
<model><name>AeroTHON Iris C1 Webcam</name><version>1.0</version>
<sdf version="1.9">model.sdf</sdf>
<description>ArduPilot Iris with top RPLidar C1 and front servo webcam.</description></model>
""",
        encoding="utf-8",
    )
    print(model_dir)


if __name__ == "__main__":
    main()
