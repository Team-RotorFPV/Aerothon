#!/usr/bin/env python3
"""The team's real airframe as a Gazebo vehicle, flown by ArduPilot SITL.

Built from what scripts/cad_to_gazebo.py measured off the CAD (airframe.json
and meshes/) plus the upstream ArduPilot Iris for the parts that are physics
convention rather than geometry: the IMU link, the rotor joints and the
LiftDrag / ApplyJointForce / ArduPilotPlugin systems. Everything the airframe
decides is re-derived here:

  rotors      at the CAD motor hubs, in the prop plane; motor order and spin
              are ArduPilot's quad-X (1 FR ccw, 2 BL ccw, 3 FL cw, 4 BR cw),
              the same as the Iris's rotor_0..3
  thrust      two LiftDrag blades per rotor, scaled to the flown prop (9.45
              in): centre of pressure and blade area by radius, and the
              rotor's top speed set so the maximum thrust matches a 2312
              980 KV motor on a 9450 at 4S (~1.17 kg). The LiftDrag model
              gives about 1.7x the lift of a real prop at a given rpm -- the
              Iris's own tuning has the same bias -- so the sim's rotor rpm
              reads low while the force is right.
  mass        2.0 kg all-up; centre of mass and inertia from the CAD parts
  collision   the core stack, the arms and the two landing skids as boxes
  sensors     LD06 lidar (450 points / 10 Hz / 12 m) and the C270 camera
              (1280x720, 48.8 deg HFOV) at their CAD mounts, the camera on the
              tilt servo; the lidar does not see the props (see LD06_MASK)
  payload     the winch hook at the dropping mechanism, where a 0.08 m
              payload clears the ground with the aircraft on its gear
"""

import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

MODEL = "aerothon_iris_c1_webcam"      # the world's vehicle slot; kept stable
# 4S2P Li-ion at 4S, a 9450 on a 2312 980 KV: ~1.17 kg of thrust per motor.
MAX_THRUST_N = 11.5
PROP_A0, PROP_CLA = 0.3, 4.25        # the Iris's blade section, unchanged
# The props render to the camera and the GUI, NOT to the lidar: the LD06's
# scan plane is 5 mm above the prop disc in the CAD, and a degree of pitch
# would put the blades in the scan. Visual bit 1 is "prop"; the lidar's mask
# excludes it.
PROP_FLAG = 0x2
LD06_MASK = 0xFFFFFFFF & ~PROP_FLAG
C270_HFOV = 2 * math.atan(math.tan(math.radians(55.0) / 2) * 16 / math.hypot(16, 9))


def _f(v):
    return f"{v:.6f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def pose(x=0.0, y=0.0, z=0.0, r=0.0, p=0.0, yw=0.0):
    return " ".join(_f(float(v)) for v in (x, y, z, r, p, yw))


def box_inertia(m, a, b, c):
    return m / 12 * (b * b + c * c), m / 12 * (a * a + c * c), m / 12 * (a * a + b * b)


def inertial(m, ixx, iyy, izz, at=(0, 0, 0), ixy=0.0, ixz=0.0, iyz=0.0):
    return (f"<inertial><pose>{pose(*at)}</pose><mass>{_f(float(m))}</mass><inertia>"
            f"<ixx>{ixx:.8g}</ixx><ixy>{ixy:.8g}</ixy><ixz>{ixz:.8g}</ixz>"
            f"<iyy>{iyy:.8g}</iyy><iyz>{iyz:.8g}</iyz><izz>{izz:.8g}</izz>"
            f"</inertia></inertial>")


def build(airframe_dir, upstream_models, out_root, camera_w=1280, camera_h=720,
          camera_hfov=None, lidar_samples=450, lidar_raise_m=0.0,
          lidar_forward_m=0.0):
    af = json.loads((airframe_dir / "airframe.json").read_text())
    hfov = C270_HFOV if camera_hfov is None else float(camera_hfov)
    stand = ET.parse(upstream_models / "iris_with_standoffs" / "model.sdf").getroot().find("model")
    ardu = ET.parse(upstream_models / "iris_with_ardupilot" / "model.sdf").getroot().find("model")
    uri = f"model://{MODEL}/meshes"

    # ---- the airframe ----
    cam_mass = 0.075
    body_m = af["body_mass_kg"] - cam_mass
    I = af["inertia"]
    col = af["collision"]
    boxes = []
    c, s = col["core"]
    boxes.append(("core", c, s))
    for i, (c, s) in enumerate(col["arms"]):
        boxes.append((f"arm_{i}", c, [s[0], s[1], max(s[2], 0.012)]))
    for i, (c, s) in enumerate(col["skids"]):
        # The skid tube at the bottom of each gear frame, not its whole outline.
        boxes.append((f"skid_{i}", [c[0], c[1], af["gear_bottom_z"] + 0.01],
                      [s[0], 0.03, 0.02]))
    collisions = "".join(
        f"<collision name='{n}'><pose>{pose(*c)}</pose><geometry><box><size>"
        f"{' '.join(_f(float(v)) for v in s)}</size></box></geometry>"
        + ("<surface><contact><ode><max_vel>10</max_vel><min_depth>0.001</min_depth>"
           "</ode></contact><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>"
           "</surface>" if n.startswith("skid") else "")
        + "</collision>" for n, c, s in boxes)
    base = ET.fromstring(
        f"<link name='base_link'>"
        f"{inertial(body_m, I['ixx'], I['iyy'], I['izz'], af['com'], I['ixy'], I['ixz'], I['iyz'])}"
        f"{collisions}"
        f"<visual name='airframe'><geometry><mesh><uri>{uri}/body.glb</uri></mesh></geometry></visual>"
        f"</link>")
    # The FCU's other sensors, exactly as the Iris carries them.
    for sensor in stand.find("link[@name='base_link']").findall("sensor"):
        base.append(copy.deepcopy(sensor))

    model = ET.Element("model", name=MODEL)
    ET.SubElement(model, "pose").text = pose()
    model.append(base)
    # The IMU is inside the Pixhawk, whose mass the airframe already counts;
    # the Iris's 0.15 kg placeholder would make this a 2.15 kg aircraft.
    imu = copy.deepcopy(stand.find("link[@name='imu_link']"))
    imu.find("inertial/mass").text = "0.005"
    model.append(imu)
    model.append(copy.deepcopy(stand.find("joint[@name='imu_joint']")))

    # ---- rotors: rotor_i is ArduPilot motor i+1 ----
    order = ["Front Right", "Back Left", "Front Left", "Back Right"]
    r = af["prop_diameter_m"] / 2
    rotor_m = af["rotor_mass_kg"]
    k_r = r / 0.127                                  # vs the Iris's 10 in
    cp = 0.084 * k_r
    area = 0.002 * k_r * k_r
    # lift of one blade at the blade's section = 0.5 rho (w cp)^2 area cla a0
    w_max = math.sqrt(MAX_THRUST_N / (2 * 0.5 * 1.2041 * area * PROP_CLA * PROP_A0 * cp * cp))
    for i, corner in enumerate(order):
        m = af["motors"][corner]
        up_rotor = stand.find(f"link[@name='rotor_{i}']")
        link = copy.deepcopy(up_rotor)
        link.find("pose").text = pose(m["hub"][0], m["hub"][1], af["prop_plane_z"])
        ine = link.find("inertial")
        ine.find("mass").text = _f(rotor_m)
        ixx, iyy, izz = rotor_m * 0.004 ** 2, rotor_m * r * r / 3, rotor_m * r * r / 3
        for tag, v in (("ixx", ixx), ("iyy", iyy), ("izz", izz)):
            ine.find(f"inertia/{tag}").text = f"{v:.8g}"
        link.find("collision/geometry/cylinder/radius").text = _f(r)
        vis = link.find("visual")
        vis.find("geometry/mesh/uri").text = f"{uri}/{af['prop_meshes'][corner]}"
        vis.find("geometry/mesh/scale").text = "1 1 1"
        for tag in ("material",):
            e = vis.find(tag)
            if e is not None:
                vis.remove(e)
        ET.SubElement(vis, "visibility_flags").text = str(PROP_FLAG)
        model.append(link)
        model.append(copy.deepcopy(stand.find(f"joint[@name='rotor_{i}_joint']")))

    # ---- physics systems, from the Iris, re-pointed and re-scaled ----
    for plugin in ardu.findall("plugin"):
        p = copy.deepcopy(plugin)
        for e in p.iter():
            if e.text and "iris_with_standoffs::" in e.text:
                e.text = e.text.replace("iris_with_standoffs::", "")
        if "lift-drag" in p.get("filename", ""):
            x = float(p.find("cp").text.split()[0])
            p.find("cp").text = f"{math.copysign(cp, x):.5f} 0 0"
            p.find("area").text = f"{area:.6f}"
        if p.get("name") == "ArduPilotPlugin":
            p.find("imuName").text = "imu_link::imu_sensor"
            for ctl in list(p.findall("control")):
                if int(ctl.get("channel", "0")) >= 4:
                    p.remove(ctl)
                    continue
                mul = ctl.find("multiplier")
                mul.text = _f(round(math.copysign(w_max, float(mul.text)), 1))
        model.append(p)

    # ---- the LD06, on its raised mount on the front plate ----
    (lx, ly, lz), lsize = af["lidar"]
    lx += lidar_forward_m
    scan_z = af["lidar_scan_z"] + lidar_raise_m
    model.append(ET.fromstring(
        f"<link name='base_scan'><pose>{pose(lx, ly, scan_z)}</pose>"
        f"{inertial(0.001, 1e-7, 1e-7, 1e-7)}"
        f"<sensor name='ld06_scan' type='gpu_lidar'><gz_frame_id>base_scan</gz_frame_id>"
        f"<topic>/lidar</topic><always_on>true</always_on><update_rate>10</update_rate>"
        f"<visualize>true</visualize><lidar>"
        f"<scan><horizontal><samples>{lidar_samples}</samples><resolution>1</resolution>"
        f"<min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan>"
        f"<range><min>0.02</min><max>12.0</max><resolution>0.01</resolution></range>"
        f"<noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>"
        f"<visibility_mask>{LD06_MASK}</visibility_mask></lidar></sensor></link>"))
    model.append(ET.fromstring(
        "<joint name='ld06_mount' type='fixed'><parent>base_link</parent>"
        "<child>base_scan</child></joint>"))

    # ---- the C270 on its tilt servo, pivoting at its own centre ----
    (cx, cy, cz), csize = af["camera"]
    lens = csize[0] / 2
    model.append(ET.fromstring(
        f"<link name='webcam_servo_base'><pose>{pose(cx, cy, cz)}</pose>"
        f"{inertial(0.01, 1e-6, 1e-6, 1e-6)}</link>"))
    model.append(ET.fromstring(
        "<joint name='webcam_servo_mount' type='fixed'><parent>base_link</parent>"
        "<child>webcam_servo_base</child></joint>"))
    ixx, iyy, izz = box_inertia(cam_mass, *csize)
    model.append(ET.fromstring(
        f"<link name='webcam_link'><pose>{pose(cx, cy, cz)}</pose>"
        f"{inertial(cam_mass, ixx, iyy, izz)}"
        f"<visual name='logitech_c270'><geometry><mesh><uri>{uri}/camera.glb</uri></mesh></geometry></visual>"
        f"<sensor name='logitech_front_camera' type='camera'>"
        f"<gz_frame_id>camera_optical_frame</gz_frame_id><pose>{pose(lens)}</pose>"
        f"<topic>/camera/image</topic><always_on>true</always_on><update_rate>20</update_rate>"
        f"<visualize>true</visualize><camera><horizontal_fov>{hfov:.6f}</horizontal_fov>"
        f"<image><width>{camera_w}</width><height>{camera_h}</height><format>R8G8B8</format></image>"
        f"<clip><near>0.04</near><far>120</far></clip></camera></sensor></link>"))
    # Axis -Y so a NEGATIVE angle looks down: see the Iris variant's note in
    # materialize_vehicle_model.py; the camera stack assumes it.
    model.append(ET.fromstring(
        f"<joint name='webcam_pitch_joint' type='revolute'><pose>0 0 0 0 0 0</pose>"
        f"<parent>webcam_servo_base</parent><child>webcam_link</child>"
        f"<axis><xyz>0 -1 0</xyz><limit><lower>-1.65</lower><upper>0.523599</upper>"
        f"<effort>2</effort><velocity>2</velocity></limit><dynamics><damping>0.08</damping>"
        f"</dynamics></axis></joint>"))
    model.append(ET.fromstring(
        "<plugin filename='gz-sim-joint-position-controller-system' "
        "name='gz::sim::systems::JointPositionController'><joint_name>webcam_pitch_joint</joint_name>"
        "<topic>/gimbal/direct_pitch</topic><p_gain>40</p_gain><i_gain>4</i_gain>"
        "<d_gain>0.6</d_gain><i_max>1</i_max><i_min>-1</i_min></plugin>"))

    # ---- the hook, under the dropping mechanism ----
    (dx, dy, _), _ = af["drop_mechanism"]
    hz = af["hook_z"]
    model.append(ET.fromstring(
        f"<link name='winch_pulley'><pose>{pose(dx, dy, hz + 0.015)}</pose>"
        f"{inertial(0.005, 1e-7, 1e-7, 1e-7)}</link>"))
    model.append(ET.fromstring(
        "<joint name='winch_swing' type='universal'><parent>base_link</parent>"
        "<child>winch_pulley</child><axis><xyz>1 0 0</xyz><dynamics><damping>0.02</damping>"
        "</dynamics></axis><axis2><xyz>0 1 0</xyz><dynamics><damping>0.02</damping>"
        "</dynamics></axis2></joint>"))
    model.append(ET.fromstring(
        f"<link name='winch_hook'><pose>{pose(dx, dy, hz)}</pose>"
        f"{inertial(0.01, 1e-6, 1e-6, 1e-6)}"
        f"<visual name='hook'><geometry><sphere><radius>0.012</radius></sphere></geometry>"
        f"<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>"
        f"</visual></link>"))
    model.append(ET.fromstring(
        "<joint name='winch_joint' type='prismatic'><parent>winch_pulley</parent>"
        "<child>winch_hook</child><axis><xyz>0 0 -1</xyz><limit><lower>0</lower>"
        "<upper>8</upper><effort>50</effort><velocity>2</velocity></limit>"
        "<dynamics><damping>0.5</damping></dynamics></axis></joint>"))
    model.append(ET.fromstring(
        "<plugin filename='gz-sim-joint-position-controller-system' "
        "name='gz::sim::systems::JointPositionController'><joint_name>winch_joint</joint_name>"
        "<topic>/aerothon/winch/payout</topic><use_velocity_commands>true</use_velocity_commands>"
        "<p_gain>4</p_gain><cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min></plugin>"))
    model.append(ET.fromstring(
        "<plugin filename='gz-sim-detachable-joint-system' "
        "name='gz::sim::systems::DetachableJoint'><parent_link>winch_hook</parent_link>"
        "<child_model>aerothon_payload</child_model><child_link>body</child_link>"
        "<detach_topic>/aerothon/payload/detach</detach_topic></plugin>"))

    sdf = ET.Element("sdf", version="1.9")
    sdf.append(model)
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space="  ")
    mdir = out_root / MODEL
    (mdir / "meshes").mkdir(parents=True, exist_ok=True)
    tree.write(mdir / "model.sdf", encoding="utf-8", xml_declaration=True)
    for f in (airframe_dir / "meshes").iterdir():
        (mdir / "meshes" / f.name).write_bytes(f.read_bytes())
    (mdir / "model.config").write_text(
        "<?xml version=\"1.0\"?>\n<model><name>AeroTHON quad (team airframe)</name>"
        "<version>1.0</version><sdf version=\"1.9\">model.sdf</sdf>"
        "<description>The team's airframe from CAD: 2312 980 KV on 9450, 4S2P Li-ion, "
        "LD06 lidar, C270 on a tilt servo, gravity-hook winch.</description></model>\n")
    return mdir, {"w_max_rad_s": w_max, "cp": cp, "area": area, "hfov": hfov,
                  "lidar_pose": (lx, ly, scan_z), "camera_pose": (cx, cy, cz)}
