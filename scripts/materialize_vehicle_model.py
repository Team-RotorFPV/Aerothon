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
        <!-- Sample count is substituted at materialize time; see the
             lidar_samples option and AEROTHON_LIDAR_SAMPLES.

             WHY IT IS A PARAMETER. A gpu_lidar sweep is many small render
             passes, and this one was the dominant per-step cost in the
             simulator: it is the term that sets real-time factor, and the
             Q27 interlock measures /scan in WALL time, so a slow host reads
             as a sick lidar (~10 sim-Hz x RTF).

             THE DEFAULT IS THE HARDWARE FIGURE. The RPLidar C1 samples at
             5000 Hz and spins at 10 Hz, so it delivers 500 points per
             revolution. The model asked for 720, which is finer than the
             sensor it simulates, paying for angular detail the C1 cannot
             resolve. 500 is both cheaper and more faithful. -->
        <scan><horizontal><samples>@LIDAR_SAMPLES@</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan>
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

  <!-- PAYLOAD WINCH. A hook on a vertical prismatic joint under the airframe
       centre, driven to the payout winch_ctrl commands (backend:=gazebo,
       /aerothon/winch/payout, metres of line). The payload is a separate
       model (aerothon_payload in the world) held on the hook by a
       DetachableJoint; /aerothon/payload/detach lets it go.

       THE LINE HANGS FROM A UNIVERSAL JOINT, like a rope from a pulley. It
       was first a prismatic joint fixed to the airframe: a rigid rod. With
       3.6 m of line out that put 0.1 kg at 3.6 m on a stick the flight
       controller had to rotate with the aircraft (~1.3 kg m^2 against the
       Iris's ~0.03), its attitude loop went unstable, and the aircraft rolled
       over and crashed mid-delivery. A line cannot push or twist; through the
       damped universal joint it only pulls, and the payload swings under the
       aircraft as a real slung load does.

       Placement: the legs reach 0.195 m below base_link and the webcam sits
       0.23 m ahead of centre, so a payload hung at -0.125 m clears the ground
       at spawn and is ~55 deg off the nadir camera's axis in flight (out of
       frame) while lowered it is almost straight below it. -->
  <link name="winch_pulley">
    <pose>0 0 -0.06 0 0 0</pose>
    <inertial><mass>0.005</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
  </link>
  <joint name="winch_swing" type="universal">
    <parent>base_link</parent><child>winch_pulley</child>
    <axis><xyz>1 0 0</xyz><dynamics><damping>0.02</damping></dynamics></axis>
    <axis2><xyz>0 1 0</xyz><dynamics><damping>0.02</damping></dynamics></axis2>
  </joint>
  <link name="winch_hook">
    <pose>0 0 -0.075 0 0 0</pose>
    <inertial><mass>0.01</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="hook"><geometry><sphere><radius>0.012</radius></sphere></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
  </link>
  <joint name="winch_joint" type="prismatic">
    <parent>winch_pulley</parent><child>winch_hook</child>
    <axis><xyz>0 0 -1</xyz><limit><lower>0</lower><upper>8</upper><effort>50</effort><velocity>2</velocity></limit><dynamics><damping>0.5</damping></dynamics></axis>
  </joint>
  <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>winch_joint</joint_name><topic>/aerothon/winch/payout</topic>
    <use_velocity_commands>true</use_velocity_commands><p_gain>4</p_gain><cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min>
  </plugin>
  <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
    <parent_link>winch_hook</parent_link>
    <child_model>aerothon_payload</child_model>
    <child_link>body</child_link>
    <detach_topic>/aerothon/payload/detach</detach_topic>
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
    # Points per revolution. 500 is the RPLidar C1's own figure (5000 Hz
    # sampling / 10 Hz rotation) and the dominant simulator cost; see the
    # comment on the <scan> block above.
    parser.add_argument("--lidar-samples", type=int,
                        default=int(os.environ.get("AEROTHON_LIDAR_SAMPLES", 500)))
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"No <model> in {args.source}")
    model.set("name", "aerothon_iris_c1_webcam")

    removed_includes = set()
    for include in list(model.findall("include")):
        uri = include.findtext("uri", "")
        if "gimbal_small_3d" in uri or "lidar_2d" in uri:
            name = include.findtext("name", "") or uri.rsplit("/", 1)[-1]
            removed_includes.add(name)
            model.remove(include)
    for joint in list(model.findall("joint")):
        if joint.get("name") == "gimbal_joint":
            model.remove(joint)

    # Where the airframe's base_link actually lives.
    #
    # Upstream ships iris_with_gimbal in three shapes. In one, the airframe
    # links sit directly in this model and `base_link` is a plain child. In the
    # second, the airframe is an ordinary <include> of iris_with_standoffs, so
    # the link is addressed as `iris_with_standoffs::base_link` and a joint
    # whose <parent> says plain `base_link` resolves to nothing: Gazebo drops
    # the lidar and webcam mounts and the world comes up with no /scan and no
    # /camera/image, with no error that names the cause.
    #
    # The third is an <include merge="true">, which splices the included links
    # into THIS model's scope. It still carries a <name>, but that name is not
    # a frame prefix, so the reference is a plain `base_link` again. Reading
    # the name there produced `iris::base_link`, a frame no graph contains, and
    # gz-sim refused the entire world with Error Code 21 rather than dropping
    # a joint quietly -- Gazebo never advertised /gazebo/worlds and the
    # launcher timed out. This is the shape the built ArduPilot overlay ships.
    #
    # This was previously patched by hand in .scratch/open_gazebo.sh, which
    # fixed the viewing model only and left the launcher's model broken. Detect
    # it here instead so both paths get an attached sensor head.
    base_link_ref = "base_link"
    if model.find("link[@name='base_link']") is None:
        for include in model.findall("include"):
            uri = include.findtext("uri", "")
            name = include.findtext("name", "") or uri.rsplit("/", 1)[-1]
            if not name or name in removed_includes:
                continue
            if include.get("merge", "").lower() == "true":
                base_link_ref = "base_link"
            else:
                base_link_ref = f"{name}::base_link"
            break

    # Drop every plugin still driving a joint this variant no longer has.
    #
    # Matching the bare names roll_joint/pitch_joint/yaw_joint missed the
    # installed model, which scopes them as `gimbal::roll_joint`. The stale
    # PID controllers then spun up against joints that had just been removed.
    surviving_joints = {j.get("name") for j in model.findall("joint")}
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "ArduPilotPlugin":
            for control in list(plugin.findall("control")):
                if int(control.get("channel", "0")) >= 8:
                    plugin.remove(control)
            continue
        joint_names = [(n.text or "") for n in plugin.iter("joint_name")]
        if not joint_names:
            continue
        if any(n.split("::")[-1] in {"roll_joint", "pitch_joint", "yaw_joint"}
               or n.split("::")[0] in removed_includes
               or (n not in surviving_joints and "::" in n)
               for n in joint_names):
            model.remove(plugin)

    hardware_xml = (COMPETITION_HARDWARE
                    .replace("@CAMERA_W@", str(args.camera_width))
                    .replace("@CAMERA_H@", str(args.camera_height))
                    .replace("@CAMERA_HFOV@", f"{args.camera_hfov:.6f}")
                    .replace("@LIDAR_SAMPLES@", str(args.lidar_samples)))
    hardware = ET.fromstring(hardware_xml)
    for element in list(hardware):
        if element.tag == "joint":
            parent = element.find("parent")
            if parent is not None and parent.text == "base_link":
                parent.text = base_link_ref
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
