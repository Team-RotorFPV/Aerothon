#!/usr/bin/env python3
"""Winch bench in Gazebo: the airframe held at the drop height, one drop, filmed.

    python3 sim/winch_bench.py                     # load from a stand, 5 m drop
    python3 sim/winch_bench.py --no-loading        # payload starts in the claw
    python3 sim/winch_bench.py --claw latch        # a claw that stays open

The team airframe is welded to the world at --alt (no flight controller: only
the winch is under test). Its dropping mechanism is the CAD's: the spool on
the motor's axle and the scissor claw on the line (sim_gazebo/claw.py). The
rulebook payload (10 x 5 x 5 cm, 100 g) carries a flat wire ring sized to the
claw's hook pockets (see WIRE_D and docs/hook_physics.md). The REAL winch_ctrl
(backend:=gazebo) runs lower -> release -> stow.

LOADING (default). The payload stands on a stand. The claw, held open, comes
down over the ring, closes round it, and the line takes in 5 mm: only contact
can raise the payload. The stand slides away and the bench checks for 3 s
that the payload's height follows the line. If it does not, the drop is not
flown and the verdict is NOT_HELD_AFTER_LOADING.

THE CLAW. Gazebo cannot hold the closed linkage, so the bench commands the
jaw angle from the CAD linkage geometry: shut when the line is taut, opening
as the line goes slack once the payload rests. No joint or command attaches
or detaches the payload; contact between the CAD jaw meshes and the ring
decides. On the way back up:

    --claw as_drawn   no latch: taking up the slack shuts the jaws again
    --claw latch      the jaws stay open until the claw is stowed

Captions show what winch_ctrl BELIEVES (it infers release from slack) and
what was MEASURED. assessment.json grades the payload's measured height
against the measured line length. Cameras: contact and oblique (riding on the
payload, looking at the ring), claw (at the ground), mechanism, wide, nadir.
"""

import argparse
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src/aerothon_sim/sim_gazebo"))
from sim_gazebo.claw import Claw                                  # noqa: E402

MODEL = "aerothon_iris_c1_webcam"
VEHICLE = "aerothon_quad"
W, H, FPS = 960, 540, 20
VIEWS = ("contact", "oblique", "claw", "wide", "mechanism", "nadir")
# THE PAYLOAD'S LIFTING EYE, sized from the claw's hook pockets (airframe.json
# "pockets", measured on the CAD by cad_to_gazebo.py). The claw's jaws are
# plates in the x-z plane, each ending in a hook pocket 3.26 mm across that
# opens toward the claw's centre through a 2.87 mm slot. What they can hold is
# a round wire crossing both plates, one side of it in each pocket: a flat,
# horizontal ring whose centreline passes through both pocket centres. A
# vertical plate with a hole cannot be held and released by this claw at any
# hole size: through the hole, the claw's arms and links trap it for good;
# beside it, the shut fingertips stop 1.6 mm short of the hole.
# Ring wire. It must pass the 2.87 mm slot to be let go, and leave room to
# close the jaws around it: at 2.2 mm the claw has to be within 0.05 mm of one
# height to close without striking the wire; at 1.6 mm, anywhere in 0.95 mm.
WIRE_D = 0.0016
LEG_H = 0.006                # ring centre above the payload's top, m
LOAD_TRAVEL = 0.020          # the open claw comes down this far over the ring
LOAD_OPEN_DEG = 35.0         # held open (as it rests, slack) while loading
LIFT_OFF = 0.005             # line taken in after closing: lifts the payload
STAND_TRAVEL = 0.25          # the stand slides this far out from under it
CLAW_TAU_S = 0.12            # the jaws swing open or shut, first order


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


# --------------------------------------------------------------------------- #
# The rig
# --------------------------------------------------------------------------- #
def use_cad_jaw_collision(model):
    """Make collision and rendering use the same jaw mesh, in the same place.

    The mission model's jaw collision is a small box at the fingertip, posed
    at the tip. Swapping in the mesh must drop that pose too: left in, it
    moved every collision mesh 6.5 mm down and 5.5 mm sideways of the jaw
    that is drawn, and the pockets seen closing on the ring were not the
    ones colliding.
    """
    for side in ("a", "b"):
        link = model.find(f"link[@name='claw_jaw_{side}']")
        collision = link.find("collision")
        for pose in collision.findall("pose"):
            collision.remove(pose)
        geometry = collision.find("geometry")
        geometry.clear()
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = link.findtext("visual/geometry/mesh/uri")


def build_vehicle(out):
    """The team airframe, welded to the world, no ArduPilot, no lidar."""
    prefix = sh("ros2 pkg prefix ardupilot_gazebo")
    if not prefix:
        sys.exit("ardupilot_gazebo not found: source ~/aerothon_stack/install/setup.bash")
    models = out / "models"
    subprocess.run([sys.executable, str(ROOT / "scripts/materialize_vehicle_model.py"),
                    "--source", f"{prefix}/share/ardupilot_gazebo/models/iris_with_gimbal/model.sdf",
                    "--output-root", str(models), "--airframe", "cad"], check=True)
    path = models / MODEL / "model.sdf"
    tree = ET.parse(path)
    model = tree.getroot().find("model")
    if model.find("link[@name='claw_pivot']") is None:
        sys.exit("the vehicle has no claw: regenerate airframe.json with scripts/cad_to_gazebo.py")
    for plugin in model.findall("plugin"):
        if ("ArduPilot" in plugin.get("filename", "") or
                "DetachableJoint" in plugin.get("name", "")):
            model.remove(plugin)
    use_cad_jaw_collision(model)
    for plugin in model.findall("plugin"):
        if plugin.findtext("joint_name") == "winch_joint":
            # The sim winch is velocity-driven at P 4: it trails the command
            # by 2.5 mm at 10 mm/s, more than the loading window.
            plugin.find("p_gain").text = "40"
    for link in model.findall("link"):
        for sensor in link.findall("sensor"):
            if sensor.get("type") == "gpu_lidar":
                link.remove(sensor)         # rendering it costs, nothing reads it
    rig = ET.SubElement(model, "joint", name="bench_rig", type="fixed")
    ET.SubElement(rig, "parent").text = "world"
    ET.SubElement(rig, "child").text = "base_link"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    airframe = json.loads((ROOT / "src/aerothon_sim/sim_gazebo/models/aerothon_quad/"
                           "airframe.json").read_text())
    return models, f"{prefix}/share/ardupilot_gazebo/models", airframe


class Cam:
    """A static pinhole camera: its SDF pose, and world points to pixels."""

    def __init__(self, eye, target, hfov):
        self.eye, self.hfov = eye, hfov
        dx, dy, dz = (t - e for t, e in zip(target, eye))
        self.yaw = math.atan2(dy, dx)
        self.pitch = -math.atan2(dz, math.hypot(dx, dy))    # +pitch looks down
        self.f = (W / 2.0) / math.tan(hfov / 2.0)

    def pose(self):
        e = self.eye
        return f"{e[0]:.4f} {e[1]:.4f} {e[2]:.4f} 0 {self.pitch:.4f} {self.yaw:.4f}"

    def project(self, p):
        dx, dy, dz = (a - b for a, b in zip(p, self.eye))
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        x1, y1 = cy * dx + sy * dy, -sy * dx + cy * dy
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        x, z = cp * x1 - sp * dz, sp * x1 + cp * dz
        if x <= 0.005:
            return None
        return (int(round(W / 2 - self.f * y1 / x)), int(round(H / 2 - self.f * z / x)))


def camera_model(name, cam):
    return f"""
    <model name="cam_{name}"><static>true</static>
      <pose>{cam.pose()}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <update_rate>{FPS}</update_rate><always_on>1</always_on>
          <topic>/bench/{name}</topic>
          <camera><horizontal_fov>{cam.hfov}</horizontal_fov>
            <image><width>{W}</width><height>{H}</height></image>
            <clip><near>0.005</near><far>60</far></clip></camera>
        </sensor>
      </link>
    </model>"""


def ring_geometry(claw):
    """The lifting ring in the vehicle frame, loaded: its centreline through
    both pocket centres, the wire resting on the pocket floors."""
    a, b = claw["pockets"]["a"], claw["pockets"]["b"]
    r_pocket = min(a["radius"], b["radius"])
    return {
        "R": (a["centre"][0] - b["centre"][0]) / 2.0,
        "x": (a["centre"][0] + b["centre"][0]) / 2.0,
        # Where the two jaw plates meet: each pocket holds one side of the ring.
        "y": (a["plate_y"][1] + b["plate_y"][0]) / 2.0,
        "z": (a["centre"][1] + b["centre"][1]) / 2.0 - (r_pocket - WIRE_D / 2.0),
        "wire": WIRE_D, "slot": min(a["slot"], b["slot"]), "pocket": 2 * r_pocket,
    }


MODEL_DIR = ROOT / "src/aerothon_sim/sim_gazebo/models/aerothon_quad"


def _glb_vertices(path):
    """Triangle vertices of a GLB written by cad_to_gazebo.write_glb."""
    import struct
    import numpy as np
    b = path.read_bytes()
    n = struct.unpack("<I", b[12:16])[0]
    js, blob = json.loads(b[20:20 + n]), b[20 + n + 8:]
    out = []
    for prim in js["meshes"][0]["primitives"]:
        view = js["bufferViews"][js["accessors"][prim["attributes"]["POSITION"]]["bufferView"]]
        out.append(np.frombuffer(blob[view["byteOffset"]:view["byteOffset"] + view["byteLength"]],
                                 np.float32).reshape(-1, 3))
    return np.vstack(out).astype(float)


def ring_clearance(claw, phi, ring_dz=0.0):
    """Smallest gap (m) between the ring's wire and the jaw meshes Gazebo
    collides, with the jaws opened by phi about a fixed centre pin and the
    ring ring_dz above its loaded height. Negative: they overlap."""
    import numpy as np
    ring, ctr = ring_geometry(claw), np.array(claw["centre_pin"])
    worst = float("inf")
    for side, sgn in (("a", 1), ("b", -1)):
        v = _glb_vertices(MODEL_DIR / "meshes" / claw["meshes"][f"claw_jaw_{side}"])
        a = sgn * phi
        x, z = v[:, 0] * math.cos(a) - v[:, 2] * math.sin(a), v[:, 0] * math.sin(a) + v[:, 2] * math.cos(a)
        rho = np.hypot(x + ctr[0] - ring["x"], v[:, 1] + ctr[1] - ring["y"])
        d = np.hypot(rho - ring["R"], z + ctr[2] - ring["z"] - ring_dz) - ring["wire"] / 2
        worst = min(worst, float(d.min()))
    return worst


def loading_window(claw, clearance=0.0001):
    """Heights (ring above its loaded position, m) at which the open claw
    comes down over the ring and closes round it without touching it: the
    claw has at least `clearance` of air throughout. Returns (lo, hi), or
    None when there is no such height."""
    phi = math.radians(LOAD_OPEN_DEG)
    ok = []
    for k in range(-40, 41):
        dz = k * 5e-5
        sweep = min(ring_clearance(claw, phi * j / 35, dz) for j in range(36))
        down = min(ring_clearance(claw, phi, dz - LOAD_TRAVEL * j / 40) for j in range(41))
        if sweep > clearance and down > clearance:
            ok.append(dz)
    return (min(ok), max(ok)) if ok else None


def rig_geometry(alt, payload, claw, loading=False):
    """Where everything is, in the world, with the airframe at `alt`.

    loading=False: the payload hangs from the shut claw (payout 0).
    loading=True:  it stands LOAD_TRAVEL lower, on the loading stand.
    """
    top, ctr = claw["top_pin"], claw["centre_pin"]
    ring = ring_geometry(claw)
    x, y = ring["x"], ring["y"]
    ptop = alt + ring["z"] - LEG_H
    rest_ctr = payload[2] + LEG_H + ctr[2] - ring["z"]
    return {
        "x": x, "y": y, "top": top, "exit": claw["line_exit"], "ring": ring,
        "payload_z0": ptop - payload[2] / 2.0 - (LOAD_TRAVEL if loading else 0),
        "loading": loading,
        "cams": {
            "claw": Cam((x, y - 0.16, rest_ctr + 0.004), (x, y, rest_ctr + 0.002), 0.45),
            "mechanism": Cam((x, y - 0.42, alt - 0.13), (x, y, alt - 0.14), 0.62),
            "wide": Cam((x + 6.5, y - 3.8, alt / 2 + 0.4), (x, y, alt / 2 + 0.1), 1.3),
        },
    }


def lifting_ring_sdf(payload_height, ring, segments=24):
    """A flat ring of round wire on two legs, collidable and visible.

    Built from short cylinders around the centreline; the ones at 0 and 180
    degrees lie along y, across the jaw plates, where the pockets hold them.
    """
    top = payload_height / 2
    R, r = ring["R"], ring["wire"] / 2
    z = top + LEG_H
    seg = 1.15 * 2 * math.pi * R / segments
    pieces = []
    for i in range(segments):
        th = 2 * math.pi * i / segments
        # A cylinder's axis is z; roll 90 deg lays it along y, yaw turns it
        # tangent to the ring at th.
        pieces.append((f"ring_{i}", f"{R * math.cos(th):.6f} {R * math.sin(th):.6f} {z:.6f} "
                                    f"1.5707963 0 {th:.6f}", r, seg))
    for side, sgn in (("leg_near", -1), ("leg_far", 1)):
        pieces.append((side, f"0 {sgn * R:.6f} {top + LEG_H / 2:.6f} 0 0 0", r, LEG_H))
    out = []
    for label, pose, radius, length in pieces:
        shape = (f"<geometry><cylinder><radius>{radius}</radius><length>{length:.6f}</length>"
                 "</cylinder></geometry>")
        out.append(
            f'<collision name="lifting_{label}"><pose>{pose}</pose>{shape}'
            '<surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode>'
            '</friction></surface></collision>'
            f'<visual name="lifting_{label}"><pose>{pose}</pose>{shape}'
            '<material><ambient>0.72 0.72 0.76 1</ambient>'
            '<diffuse>0.82 0.82 0.86 1</diffuse><specular>0.6 0.6 0.6 1</specular>'
            '</material></visual>')
    return "".join(out)


def loading_stand_sdf(geo, payload_height):
    """A stand under the payload while the claw takes it; it slides away in x."""
    bottom = geo["payload_z0"] - payload_height / 2
    return f"""
    <model name="loading_stand"><pose>{geo['x']} {geo['y']} {bottom - 0.005} 0 0 0</pose>
      <link name="stage"><inertial><mass>1</mass><inertia>
        <ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="stage"><geometry><box><size>0.12 0.07 0.01</size></box>
          </geometry></collision>
        <visual name="stage"><geometry><box><size>0.12 0.07 0.01</size></box>
          </geometry><material><ambient>0.15 0.4 0.35 1</ambient>
          <diffuse>0.15 0.4 0.35 1</diffuse></material></visual>
        <visual name="post"><pose>0 0 {-(bottom - 0.005) / 2:.4f} 0 0 0</pose>
          <geometry><box><size>0.02 0.02 {bottom - 0.005:.4f}</size></box></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse>
          </material></visual>
      </link>
      <joint name="stage_slide" type="prismatic"><parent>world</parent>
        <child>stage</child><axis><xyz>1 0 0</xyz>
          <limit><lower>0</lower><upper>{STAND_TRAVEL}</upper><effort>1000</effort>
            <velocity>0.5</velocity></limit></axis></joint>
      <plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
        <joint_name>stage_slide</joint_name><topic>/bench/stage_target</topic>
        <use_velocity_commands>true</use_velocity_commands>
        <p_gain>3</p_gain><cmd_max>0.25</cmd_max><cmd_min>-0.25</cmd_min>
      </plugin>
      <plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher"/>
    </model>"""


def write_world(out, alt, payload, geo):
    x, y = geo["x"], geo["y"]
    px, py, pz = payload
    tab = lifting_ring_sdf(pz, geo["ring"])
    stand = loading_stand_sdf(geo, pz) if geo["loading"] else ""
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="winch_bench">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.62 0.75 0.9 1</background>
      <grid>false</grid></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose><diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>

    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.30 0.45 0.22 1</ambient><diffuse>0.33 0.5 0.25 1</diffuse></material></visual>
    </link></model>

    <!-- The target pad under the drop point: white, 1.2 m, a black cross. -->
    <model name="pad"><static>true</static><pose>{x} {y} 0.002 0 0 0</pose><link name="link">
      <visual name="w"><geometry><box><size>1.2 1.2 0.004</size></box></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      <visual name="x"><pose>0 0 0.003 0 0 0</pose><geometry><box><size>0.6 0.04 0.002</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
      <visual name="y"><pose>0 0 0.003 0 0 0</pose><geometry><box><size>0.04 0.6 0.002</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
    </link></model>

    <!-- Rulebook Figure 1: 10 x 5 x 5 cm, 100 g. Its lifting eye here is a
         flat wire ring sized to the claw's pockets (ring_geometry). -->
    {stand}
    <model name="aerothon_payload"><pose>{x} {y} {geo["payload_z0"]:.5f} 0 0 0</pose>
      <link name="body">
        <inertial><mass>0.10</mass><inertia>
          <ixx>{0.1 * (py**2 + pz**2) / 12:.3e}</ixx><iyy>{0.1 * (px**2 + pz**2) / 12:.3e}</iyy>
          <izz>{0.1 * (px**2 + py**2) / 12:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{px} {py} {pz}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{px} {py} {pz}</size></box></geometry>
          <material><ambient>0.35 0.5 0.85 1</ambient><diffuse>0.45 0.6 0.95 1</diffuse></material></visual>
        {tab}
        <sensor name="contact" type="camera">
          <pose>0 -0.12 {pz / 2 + LEG_H + 0.006:.4f} 0 0.05 1.5707963</pose>
          <update_rate>{FPS}</update_rate><always_on>1</always_on>
          <topic>/bench/contact</topic>
          <camera><horizontal_fov>0.45</horizontal_fov>
            <image><width>{W}</width><height>{H}</height></image>
            <clip><near>0.005</near><far>60</far></clip></camera>
        </sensor>
        <sensor name="oblique" type="camera">
          <pose>0.085 -0.10 {pz / 2 + LEG_H + 0.008:.4f} 0 0.06 2.27</pose>
          <update_rate>{FPS}</update_rate><always_on>1</always_on>
          <topic>/bench/oblique</topic>
          <camera><horizontal_fov>0.55</horizontal_fov>
            <image><width>{W}</width><height>{H}</height></image>
            <clip><near>0.005</near><far>60</far></clip></camera>
        </sensor>
      </link>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg><update_frequency>50</update_frequency>
      </plugin>
    </model>

    <include><uri>model://{MODEL}</uri><name>{VEHICLE}</name>
      <pose>0 0 {alt} 0 0 0</pose></include>
    {"".join(camera_model(n, c) for n, c in geo["cams"].items())}
  </world>
</sdf>
"""
    (out / "world.sdf").write_text(world)


def write_bridge(out):
    to_gz = [("/bench/line", "/aerothon/winch/payout", "std_msgs/msg/Float64", "gz.msgs.Double"),
             ("/bench/spool", "/aerothon/winch/spool", "std_msgs/msg/Float64", "gz.msgs.Double"),
             ("/gimbal/direct_pitch", "/gimbal/direct_pitch", "std_msgs/msg/Float64",
              "gz.msgs.Double")]
    to_gz.append(("/bench/stage_target", "/bench/stage_target", "std_msgs/msg/Float64",
                  "gz.msgs.Double"))
    to_gz += [(f"/bench/claw/{j}", f"/aerothon/claw/{j}", "std_msgs/msg/Float64",
               "gz.msgs.Double") for j in ("link_a", "link_b", "pivot", "jaw_a", "jaw_b")]
    to_ros = [("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock"),
              ("/sim/payload_pose", "/model/aerothon_payload/pose", "geometry_msgs/msg/Pose",
               "gz.msgs.Pose"),
              ("/bench/joints", f"/world/winch_bench/model/{VEHICLE}/joint_state",
               "sensor_msgs/msg/JointState", "gz.msgs.Model"),
              ("/bench/nadir", "/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
              ("/bench/stand_joints", "/world/winch_bench/model/loading_stand/joint_state",
               "sensor_msgs/msg/JointState", "gz.msgs.Model")]
    to_ros += [(f"/bench/{v}", f"/bench/{v}", "sensor_msgs/msg/Image", "gz.msgs.Image")
               for v in VIEWS if v != "nadir"]
    text = "".join(
        f"- ros_topic_name: \"{r}\"\n  gz_topic_name: \"{g}\"\n  ros_type_name: \"{rt}\"\n"
        f"  gz_type_name: \"{gt}\"\n  direction: {d}\n"
        for entries, d in ((to_gz, "ROS_TO_GZ"), (to_ros, "GZ_TO_ROS"))
        for r, g, rt, gt in entries)
    (out / "bridge.yaml").write_text(text)


# --------------------------------------------------------------------------- #
# The claw's mechanics, the drop, and the recording
# --------------------------------------------------------------------------- #
def run_drop(out, alt, payload, geo, claw_geo, mode, max_sim_s, loading=False, load_dz=0.0,
             lift_s=2.0):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Float64, String

    rclpy.init()
    node = Node("winch_bench", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    bridge = CvBridge()
    claw = Claw(claw_geo)
    rest_z = payload[2] / 2.0
    st = {"status": {}, "payload": None, "q": {}, "payout": 0.0, "phase": "hanging",
          "t_written": None, "t_mech": None,
          # the claw
          "resting": False, "rest_line": None, "phi": 0.0,
          "claw_note": "jaws shut", "t_open": None, "lift_seen": False,
          "early_fall_seen": False, "start_payload_z": None,
          # Loading: the line and jaws scripted until the payload hangs; then
          # winch_ctrl's payout plus the line already out (offset).
          "load": {"line": 0.0, "phi": 0.0} if loading else None, "offset": 0.0,
          "hang_ref": None, "hang_ok": None, "not_held": False}
    if loading:
        st["phase"] = "loading: claw held open"
    phi_load = math.radians(LOAD_OPEN_DEG)
    # The claw closes load_dz lower than the loaded pose: the middle of the
    # loading window, where it clears the ring on the way down and round it.
    close_line = LOAD_TRAVEL + load_dz
    open_line = close_line + claw.pose(phi_load)["drop"]
    latest, writers, frames = {}, {}, {}
    events, rows = [], []

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def event(what):
        t = now()
        events.append((t, what))
        node.get_logger().info(f"[t={t:6.2f}] {what}")

    pub = {k: node.create_publisher(Float64, f"/bench/claw/{k}", 10)
           for k in ("link_a", "link_b", "pivot", "jaw_a", "jaw_b")}
    pub_line = node.create_publisher(Float64, "/bench/line", 10)
    pub_spool = node.create_publisher(Float64, "/bench/spool", 10)
    pub_pose = node.create_publisher(PoseStamped, "/mavros/local_position/pose", 10)
    pub_vel = node.create_publisher(TwistStamped, "/mavros/local_position/velocity_local", 10)
    pub_cmd = node.create_publisher(String, "/winch/cmd", 10)
    pub_tilt = node.create_publisher(Float64, "/gimbal/direct_pitch", 10)
    pub_stage = node.create_publisher(Float64, "/bench/stage_target", 10)

    def on_status(m):
        s = json.loads(m.data)
        old = st["status"]
        if s.get("hook_open") and not old.get("hook_open"):
            event("winch_ctrl infers the release (line slack)")
        if s.get("state") != old.get("state"):
            event(f"winch {old.get('state', '-')} -> {s.get('state')}")
        st["status"] = s

    def on_payout(m):
        st["payout"] = float(m.data)

    def on_payload(m):
        st["payload"] = (m.position.x, m.position.y, m.position.z)
        o = m.orientation
        # Tilt of the payload's up axis from vertical, degrees.
        st["tilt"] = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (o.x * o.x + o.y * o.y)))))
        if st["start_payload_z"] is None:
            st["start_payload_z"] = m.position.z

    def on_joints(m):
        st["q"] = dict(zip(m.name, m.position))

    def on_stand(m):
        st["stand"] = dict(zip(m.name, m.position)).get("stage_slide")

    def on_image(view):
        def cb(m):
            try:
                img = bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
            except Exception:                           # noqa: BLE001
                return
            latest[view] = img if img.shape[1] == W else cv2.resize(img, (W, H))
        return cb

    # ---- the claw -------------------------------------------------------- #
    def mechanism():
        """The line and the claw, from winch_ctrl's payout (50 Hz, sim time).

        A line can pull but not push: while the claw rests, payout beyond
        the resting length is slack, and the claw uses the first few mm of
        it to open.
        """
        t = now()
        dt = 0.0 if st["t_mech"] is None else t - st["t_mech"]
        st["t_mech"] = t
        if st["load"] is not None:
            # Loading: the line and the jaw angle follow the loading script;
            # the ring goes wherever contact puts it.
            L = st["load"]
            st["phi"] = L["phi"]
            pose = claw.pose(L["phi"])
            for k in pub:
                pub[k].publish(Float64(data=float(pose[k])))
            pub_line.publish(Float64(data=float(L["line"])))
            pub_spool.publish(Float64(data=-L["line"] / claw_geo["spool_line_r"]))
            st["claw_note"] = f"jaw angle {math.degrees(L['phi']):.0f} deg"
            return
        payout = st["payout"] + st["offset"]
        q = st["q"].get("winch_joint", 0.0)
        p = st["payload"]
        on_ground = p is not None and p[2] <= rest_z + 0.002
        paying_out = payout > st.get("last_payout", 0.0)
        st["last_payout"] = payout
        latched = mode == "latch" and st["t_open"] is not None
        if latched and payout < 0.05:
            st["phi"] = max(0.0, st["phi"] - dt / CLAW_TAU_S * claw.open_max)
        if not st["resting"]:
            line = payout
            if on_ground and paying_out:
                st["resting"], st["rest_line"] = True, q
                event(f"payload reached ground; {q:.3f} m of line out")
            elif not latched:
                # Hanging free, the jaws' own weight shuts the tongs.
                st["phi"] *= math.exp(-dt / CLAW_TAU_S) if dt > 0 else 1.0
        else:
            rest = st["rest_line"]
            slack = payout - rest
            if latched:
                # A latch holds the jaws open; the claw lifts off with them
                # open as soon as the line takes up the top pin's travel.
                st["phi"] += (claw.open_max - st["phi"]) * (
                    1.0 - math.exp(-dt / CLAW_TAU_S) if dt > 0 else 0.0)
                drop = claw.pose(st["phi"])["drop"]
                if slack < drop:
                    st["resting"] = False
                line = max(payout, rest) if slack >= drop else payout
            else:
                # The linkage. Slack lets gravity swing the jaws open (not
                # instantly); taking the slack in pulls them shut in lockstep
                # with the top pin.
                geo_phi = claw.phi_for_drop(max(slack, 0.0))
                if geo_phi < st["phi"]:
                    st["phi"] = geo_phi
                elif dt > 0:
                    st["phi"] += (geo_phi - st["phi"]) * (1.0 - math.exp(-dt / CLAW_TAU_S))
                if slack < 0.0:
                    st["resting"] = False            # shut, and lifting off
                    line = payout
                else:
                    line = rest + claw.pose(st["phi"])["drop"]

        if st["t_open"] is None and st["phi"] >= claw.release:
            st["t_open"] = t
            event(f"JAWS OPEN {math.degrees(st['phi']):.0f} deg; contact decides release")

        deg = math.degrees(st["phi"])
        st["claw_note"] = f"jaw angle {deg:.0f} deg; release measured by payload motion"
        pose = claw.pose(st["phi"])
        for k in pub:
            pub[k].publish(Float64(data=float(pose[k])))
        pub_line.publish(Float64(data=float(line)))
        # The spool turns with the motor: the line it pays out, slack or not.
        pub_spool.publish(Float64(data=-payout / claw_geo["spool_line_r"]))

    def expected_z():
        """Where the payload is if it still hangs from the claw: it moves
        with the line's measured length from the hang check onwards."""
        ref = st["hang_ref"]
        if ref is None:
            return None
        return ref[1] - (st["q"].get("winch_joint", 0.0) - ref[2])

    # ---- captions and frames -------------------------------------------- #
    def draw_line(img, view):
        cam = geo["cams"].get(view)
        if cam is None:
            return
        q = st["q"].get("winch_joint", 0.0)
        ex = geo["exit"]
        a = cam.project((ex[0], ex[1], alt + ex[2]))
        b = cam.project((geo["top"][0], geo["top"][1], alt + geo["top"][2] - q + 0.004))
        if a and b:
            cv2.line(img, a, b, (35, 35, 35), 1 if view == "wide" else 2, cv2.LINE_AA)

    def caption(img, view):
        s, p = st["status"], st["payload"]
        believes = "released" if s.get("hook_open") else "holding"
        observed = ("MEASURED: payload NOT HELD when the stand left" if st["not_held"]
                    else "MEASURED: payload fell before reaching the ground" if st["early_fall_seen"]
                    else "MEASURED: payload lifted again after touchdown" if st["lift_seen"]
                    else "MEASURED: payload hangs from the claw by contact" if st["hang_ok"]
                    else "measured: -")
        lines = [f"{view}   t = {now():6.2f} s (sim)   claw: {mode.replace('_', ' ')}",
                 f"winch {s.get('state', '-')}   line out {s.get('payout_m', 0):.3f} m",
                 f"winch_ctrl believes: {believes}",
                 f"claw: {st['claw_note']}",
                 (f"payload bottom {p[2] - rest_z:+.3f} m" if p else ""),
                 observed,
                 st["phase"]]
        for i, t in enumerate(lines):
            y = 24 + 22 * i
            cv2.putText(img, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 4,
                        cv2.LINE_AA)
            colour = ((80, 230, 255) if i == 0 else
                      (0, 60, 255) if i == 5 and (st["lift_seen"] or st["not_held"] or
                                                   st["early_fall_seen"]) else
                      (255, 255, 255))
            cv2.putText(img, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.56, colour, 1,
                        cv2.LINE_AA)
        return img

    def write_frames():
        t = now()
        if st["t_written"] is not None and t - st["t_written"] < 1.0 / FPS - 1e-3:
            return
        if not all(v in latest for v in VIEWS):
            return
        st["t_written"] = t
        st.setdefault("t_first_frame", t)
        for v in VIEWS:
            if v not in writers:
                writers[v] = cv2.VideoWriter(str(out / f"{v}.avi"),
                                             cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
            img = latest[v].copy()
            draw_line(img, v)
            writers[v].write(caption(img, v))
            frames[v] = frames.get(v, 0) + 1

    node.create_subscription(String, "/winch/status", on_status, 10)
    node.create_subscription(Float64, "/winch/gz/payout", on_payout, 10)
    node.create_subscription(Pose, "/sim/payload_pose", on_payload, qos_profile_sensor_data)
    node.create_subscription(JointState, "/bench/joints", on_joints, qos_profile_sensor_data)
    node.create_subscription(JointState, "/bench/stand_joints", on_stand, qos_profile_sensor_data)
    for view in VIEWS:
        node.create_subscription(Image, f"/bench/{view}", on_image(view),
                                 qos_profile_sensor_data)

    def tick():
        # The flight controller's view of the hovering aircraft: the winch
        # reads its altitude and speed from these.
        p = PoseStamped()
        p.header.stamp = node.get_clock().now().to_msg()
        p.pose.position.z = float(alt)
        p.pose.orientation.w = 1.0
        pub_pose.publish(p)
        v = TwistStamped()
        v.header.stamp = p.header.stamp
        pub_vel.publish(v)
        pub_tilt.publish(Float64(data=-math.pi / 2))    # C270 straight down
        write_frames()
        s, t, pl = st["status"], now(), st["payload"]
        ph = st["phase"]
        q = st["q"].get("winch_joint", 0.0)
        rows.append((round(t, 3), ph.split(":")[0], s.get("state"), s.get("payout_m"),
                     round(q, 5), s.get("hook_open"), round(math.degrees(st["phi"]), 2),
                     round(pl[2], 5) if pl else None, round(pl[0], 5) if pl else None,
                     round(pl[1], 5) if pl else None, round(st.get("tilt", 0.0), 2)))
        if st["load"] is not None:
            L, tl = st["load"], st.setdefault("t_phase", t)
            if ph == "loading: claw held open":
                L["phi"] = phi_load * min(1.0, (t - tl) / 1.0)
                if t - tl > 1.5:
                    st["phase"], st["t_phase"] = "loading: the open claw comes down over the ring", t
                    event(f"loading: jaws held open {LOAD_OPEN_DEG:.0f} deg; lowering {open_line * 1000:.1f} mm")
            elif ph.startswith("loading: the open claw"):
                L["line"] = min(open_line, 0.01 * (t - tl))
                if L["line"] >= open_line and t - tl > open_line / 0.01 + 1.5:
                    st["phase"], st["t_phase"] = "loading: line taken up, the jaws close on the ring", t
                    event("loading: taking up the slack; the pockets close on the ring")
            elif ph.startswith("loading: line taken up"):
                f = min(1.0, (t - tl) / 3.0)
                L["phi"] = phi_load * (1.0 - f)
                L["line"] = close_line + claw.pose(L["phi"])["drop"]
                if t - tl > 4.0:
                    st["phase"], st["t_phase"] = "loading: winch lifts the payload off the stand", t
                    st["pre_lift_z"] = pl[2] if pl else None
                    event("loading: lifting 5 mm -- only contact can raise the payload")
            elif ph.startswith("loading: winch lifts"):
                L["line"] = close_line - LIFT_OFF * min(1.0, (t - tl) / lift_s)
                if pl and st["pre_lift_z"]:
                    st["lift_peak"] = max(st.get("lift_peak", 0.0), pl[2] - st["pre_lift_z"])
                if t - tl > lift_s + 1.0:
                    rise = (pl[2] - st["pre_lift_z"]) if pl and st["pre_lift_z"] else 0.0
                    event(f"MEASURED payload rose up to {st.get('lift_peak', 0.0) * 1000:.1f} mm "
                          f"during the lift and is {rise * 1000:.1f} mm up at its end "
                          f"({LIFT_OFF * 1000:.0f} mm of line taken in)")
                    if rise < 0.6 * LIFT_OFF:
                        # The claw came up without it: whatever holds it, it
                        # is not the claw. Carry on so the video shows it.
                        st["lift_failed"] = True
                        event("MEASURED the payload is back on the stand: the claw did not "
                              "keep hold of it")
                    st["phase"], st["t_phase"] = "loading: the stand slides away", t
                    pub_stage.publish(Float64(data=STAND_TRAVEL))
            elif ph.startswith("loading: the stand"):
                pub_stage.publish(Float64(data=STAND_TRAVEL))
                if t - tl > 2.0:
                    event(f"MEASURED stand at {st.get('stand') or 0.0:.3f} m of "
                          f"{STAND_TRAVEL} m travel")
                    st["phase"], st["t_phase"] = "hang check: the stand is gone", t
                    st["hang_ref"] = (t, pl[2], q)
                    event(f"hang check starts: payload z {pl[2]:.4f} m, line {q:.4f} m")
            elif ph.startswith("hang check"):
                ez = expected_z()
                stand_gone = (st.get("stand") or 0.0) > 0.9 * STAND_TRAVEL
                if (pl and ez is not None and pl[2] < ez - 0.01) or st.get("lift_failed") \
                        or not stand_gone:
                    st["not_held"] = True
                    why = ("the claw lifted it, then lost it" if st.get("lift_failed") else
                           "the stand never moved away" if not stand_gone else
                           f"it fell {ez - pl[2]:.3f} m below the claw's hold")
                    event(f"MEASURED payload NOT HELD: {why}")
                    st["phase"], st["t_done"] = "stowed: payload not held after loading", t
                    st["load"] = None
                elif t - tl > 3.0:
                    st["hang_ok"] = True
                    dev = abs(pl[2] - ez) if pl and ez is not None else float("nan")
                    event(f"MEASURED payload hangs from the claw by contact for 3 s "
                          f"(within {dev * 1000:.1f} mm of the line)")
                    st["offset"] = L["line"]
                    st["load"] = None
                    st["phase"] = "hanging"
            return
        ez = expected_z()
        if (pl and ph.startswith("lowering") and ez is not None
                and pl[2] > ez + 0.02 and not st.get("held_up_seen")):
            st["held_up_seen"] = True
            event(f"MEASURED payload not following the line down: {pl[2] - ez:.3f} m above "
                  f"the claw's hold")
        if (pl and ph.startswith("lowering") and ez is not None
                and pl[2] < ez - 0.02 and not st["early_fall_seen"]):
            st["early_fall_seen"] = True
            event(f"MEASURED payload separated from claw during descent "
                  f"({ez - pl[2]:.3f} m below the claw's hold)")
        if pl and ph.startswith("stowing") and pl[2] > rest_z + 0.10 and not st["lift_seen"]:
            st["lift_seen"] = True
            event(f"MEASURED payload lifted off ground: centre z={pl[2]:.3f} m")
        if ph == "hanging" and (t > 3.0 or st["hang_ok"]) and s:
            if st["hang_ref"] is None and pl:
                st["hang_ref"] = (t, pl[2], q)       # no stand: it starts on the claw
            pub_cmd.publish(String(data="lower"))
            st["phase"] = "lowering: the motor pays out line"
            event("command: lower")
        elif ph.startswith("lowering") and s.get("state") == "AT_GROUND":
            pub_cmd.publish(String(data="release"))
            st["phase"] = "down: winch_ctrl records the release"
            st["t_down"] = t
            event("command: release (a gravity hook: bookkeeping only)")
        elif ph.startswith("down") and t - st["t_down"] > 2.0:
            pub_cmd.publish(String(data="stow"))
            st["phase"] = "stowing: the motor winds the claw back up"
            event("command: stow")
        elif ph.startswith("stowing") and s.get("payout_m", 1) <= 1e-3:
            left = pl is not None and pl[2] <= rest_z + 0.01
            st["phase"] = (
                "stowed: payload fell early" if left and st["early_fall_seen"] else
                "stowed: payload REGRABBED then fell again" if left and st["lift_seen"] else
                "stowed: payload stayed on ground" if left else
                "stowed: payload came back up with claw")
            st["t_done"] = t
            event(st["phase"])
        elif ph.startswith("stowed") and t - st.get("t_done", t) > 3.0:
            st["phase"] = "done"

    node.create_timer(0.02, mechanism)
    node.create_timer(1.0 / FPS, tick)
    wall0 = time.time()
    while rclpy.ok() and st["phase"] != "done":
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - wall0 > 60 and (now() == 0.0 or not st["status"]):
            # Fail loudly rather than wait out the wall-clock limit.
            raise SystemExit("no /clock or /winch/status after 60 s: the simulator, "
                             "the bridge or winch_ctrl is not being heard")
        if now() > max_sim_s or time.time() - wall0 > 60 * 40:
            event("timed out")
            break
    for w in writers.values():
        w.release()
    with open(out / "timeline.csv", "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["sim_t", "phase", "winch_state", "payout_m", "line_m",
                     "ctrl_believes_open", "claw_open_deg", "payload_z", "payload_x",
                     "payload_y", "payload_tilt_deg"])
        cw.writerows(rows)
    (out / "events.txt").write_text("".join(f"{t:7.2f}  {w}\n" for t, w in events))
    node.destroy_node()
    rclpy.shutdown()
    release = st["t_open"]
    return frames, release, st.get("t_first_frame", 0.0), events


def encode(out, release_t, first_frame_t, events=()):
    for v in VIEWS:
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(out / f"{v}.avi"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                        str(out / f"{v}.mp4")], check=True)
        (out / f"{v}.avi").unlink()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error",
                    *sum((["-i", str(out / f"{v}.mp4")] for v in VIEWS[:4]), []),
                    "-filter_complex",
                    "[0:v][1:v]hstack=inputs=2[top];[2:v][3:v]hstack=inputs=2[bot];"
                    "[top][bot]vstack=inputs=2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "22", str(out / "drop.mp4")], check=True)
    # The loading, close up: front and oblique side by side, half speed.
    hung = next((t for t, w in events if w.startswith(("MEASURED payload hangs",
                                                        "MEASURED payload NOT HELD"))), None)
    if hung is not None:
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-t", f"{hung - first_frame_t + 1:.2f}",
                        "-i", str(out / "contact.mp4"), "-t", f"{hung - first_frame_t + 1:.2f}",
                        "-i", str(out / "oblique.mp4"), "-filter_complex",
                        "[0:v][1:v]hstack=inputs=2,setpts=2.0*PTS", "-r", str(FPS),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                        str(out / "loading_closeup_2x_slow.mp4")], check=True)
    if release_t is not None:
        start = max(0.0, release_t - first_frame_t - 3.0)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{start:.2f}",
                        "-t", "9", "-i", str(out / "claw.mp4"), "-vf", "setpts=4.0*PTS",
                        "-r", str(FPS), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                        str(out / "claw_release_4x_slow.mp4")], check=True)


def assess_drop(out, payload_height):
    """Grade the drop from measured payload motion, never from winch state.

    With a loading stand, the payload's height is compared with the line's
    measured length from the moment the stand is gone: if it still hangs from
    the claw, it moves with the line. Without one (the payload starts on the
    claw), the first sample is the reference.
    """
    with (out / "timeline.csv").open(newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["payload_z"]]
    rest_z = payload_height / 2
    num = lambda r, k: float(r[k]) if r[k] not in ("", "None") else None  # noqa: E731
    hang = [r for r in rows if r["phase"] == "hang check"]
    ref = hang[0] if hang else next((r for r in rows if r["phase"] == "hanging"), None)
    verdict = {"basis": "Gazebo payload pose against the measured line length "
                        "(winch joint); the controller's belief is not used"}
    if ref is None:
        verdict["result"] = "INCOMPLETE"
        verdict["reason"] = "the payload never reached the hanging phase"
    else:
        z0, q0 = num(ref, "payload_z"), num(ref, "line_m")
        excess = lambda r: z0 - (num(r, "line_m") - q0) - num(r, "payload_z")  # noqa: E731
        if hang:
            worst = max(hang, key=excess)
            verdict["hang_check_max_drop_below_hold_m"] = round(excess(worst), 4)
        lowering = [r for r in rows if r["phase"] == "lowering"]
        touch = next((r for r in lowering if num(r, "payload_z") <= rest_z + 0.004), None)
        # Up to and including first ground contact: an early fall also ends on
        # the ground, but with the line still holding the claw high above it.
        before = [r for r in lowering if touch is None or float(r["sim_t"]) <= float(touch["sim_t"])]
        worst_d = max(before, key=excess) if before else None
        stow = [r for r in rows if r["phase"] == "stowing" or r["phase"].startswith("stowed")]
        high = min(before, key=excess) if before else None
        if hang and verdict["hang_check_max_drop_below_hold_m"] > 0.01:
            verdict["result"] = "NOT_HELD_AFTER_LOADING"
        elif not lowering and any(r["phase"] == "stowed" for r in rows):
            verdict["result"] = "NOT_HELD_AFTER_LOADING"
            verdict["reason"] = "the run stopped at loading (see events.txt)"
        elif worst_d is not None and excess(worst_d) > 0.02:
            verdict["result"] = "DROPPED_EARLY"
        elif high is not None and excess(high) < -0.02:
            verdict["result"] = "NOT_FOLLOWING_LINE"
            verdict["reason"] = "the payload stayed above the claw's hold: held by something else"
            verdict["max_above_hold_m"] = round(-excess(high), 4)
        elif touch is None or not stow:
            verdict["result"] = "INCOMPLETE"
            verdict["reason"] = "no touchdown or no stow samples"
        else:
            peak = max(stow, key=lambda r: num(r, "payload_z"))
            lift = num(peak, "payload_z") - rest_z
            verdict["result"] = "REGRABBED" if lift > 0.05 else "RELEASED_ON_GROUND"
            verdict.update({
                "touchdown_time_s": round(float(touch["sim_t"]), 3),
                "touchdown_line_m": round(num(touch, "line_m"), 3),
                "max_lift_after_touchdown_m": round(lift, 3),
                "max_lift_time_s": round(float(peak["sim_t"]), 3),
                "final_payload_centre_z_m": round(num(stow[-1], "payload_z"), 4),
            })
        if worst_d is not None:
            verdict["max_drop_below_hold_during_descent_m"] = round(excess(worst_d), 4)
            verdict["max_drop_time_s"] = round(float(worst_d["sim_t"]), 3)
    (out / "assessment.json").write_text(json.dumps(verdict, indent=2) + "\n")
    return verdict


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--alt", type=float, default=5.0, help="airframe height, m")
    ap.add_argument("--payload", type=float, nargs=3, default=(0.10, 0.05, 0.05),
                    metavar=("X", "Y", "Z"), help="payload box, m (rulebook Fig. 1)")
    ap.add_argument("--claw", choices=("as_drawn", "latch"), default="as_drawn")
    ap.add_argument("--out", type=Path, default=ROOT / "logs" / "winch_bench")
    ap.add_argument("--max-sim-s", type=float, default=110.0)
    ap.add_argument("--load-at", type=float, default=0.5,
                    help="where in the loading window to close the claw, 0 (the "
                         "lowest ring position, closest to the pocket floors) to 1")
    ap.add_argument("--lift-s", type=float, default=2.0,
                    help="seconds to take in the 5 mm lift")
    ap.add_argument("--no-loading", dest="loading", action="store_false",
                    help="start with the payload on the claw instead of loading it "
                         "from a stand")
    args = ap.parse_args()

    out = args.out.resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    models, upstream, airframe = build_vehicle(out)
    # Refuse a start that has the claw inside the ring, or a loading path
    # that runs through it: contact would begin by ejecting the payload.
    start = ring_clearance(airframe["claw"], 0.0)
    window = loading_window(airframe["claw"])
    load_dz = (window[0] + args.load_at * (window[1] - window[0])) if window else 0.0
    print(f"ring clearance loaded {start * 1000:+.3f} mm; loading window "
          + (f"{window[0] * 1000:+.2f}..{window[1] * 1000:+.2f} mm, closing at "
             f"{load_dz * 1000:+.2f} mm" if window else "NONE"))
    if start < 0 or (args.loading and window is None):
        sys.exit("the claw's jaws overlap the lifting ring, or cannot close round it: "
                 "fix the geometry before simulating")
    geo = rig_geometry(args.alt, args.payload, airframe["claw"], loading=args.loading)
    write_world(out, args.alt, args.payload, geo)
    write_bridge(out)

    env = dict(os.environ,
               GZ_SIM_RESOURCE_PATH=os.pathsep.join(
                   [str(models), upstream, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]),
               GZ_PARTITION=f"winch_bench_{os.getpid()}", GZ_IP="127.0.0.1",
               ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "61"),
               # Everything runs on this host: shared memory only. When WSL
               # falls back to its "None" networking mode, UDP discovery on
               # the loopback stops working and every node runs deaf.
               FASTDDS_BUILTIN_TRANSPORTS="SHM")
    env.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.update({k: env[k] for k in ("GZ_PARTITION", "GZ_IP", "ROS_DOMAIN_ID",
                                           "FASTDDS_BUILTIN_TRANSPORTS")})
    procs = []

    def start(cmd, log):
        procs.append(subprocess.Popen(cmd, env=env, stdout=open(out / log, "w"),
                                      stderr=subprocess.STDOUT, start_new_session=True))

    try:
        start(["gz", "sim", "-s", "-r", "--headless-rendering", str(out / "world.sdf")],
              "gz.log")
        start(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
               "-p", f"config_file:={out / 'bridge.yaml'}"], "bridge.log")
        # winch_ctrl's payout goes to the bench (the claw's mechanics), not
        # straight to the joint; its own detach is not bridged: the claw lets go.
        start([sys.executable, "-c", "from winch_ctrl.winch_node import main; main()",
               "--ros-args", "-p", "backend:=gazebo", "-p", "use_sim_time:=true",
               # 5 Hz is 9 cm steps of line at stow speed: too coarse to film.
               "-p", "publish_rate_hz:=25.0"],
              "winch.log")
        frames, release_t, first_t, events = run_drop(
            out, args.alt, args.payload, geo, airframe["claw"], args.claw, args.max_sim_s,
            loading=args.loading, load_dz=load_dz, lift_s=args.lift_s)
    finally:
        for sig in (signal.SIGINT, signal.SIGKILL):
            for p in procs:
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    pass
            time.sleep(2)

    print(f"frames: {frames}")
    encode(out, release_t, first_t, events)
    print(f"observed outcome: {assess_drop(out, args.payload[2])}")
    print((out / "events.txt").read_text())
    print(f"outputs in {out}")


if __name__ == "__main__":
    main()
