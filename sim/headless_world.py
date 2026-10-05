#!/usr/bin/env python3
"""A headless Mission 2 world: everything but the stack, at object level.

WHY THIS EXISTS

    A Gazebo + ArduPilot SITL run of the full mission takes ~45 minutes of a
    workstation's wall time, and it is the only thing that could say whether
    a change survives a gusty crosswind, a GPS glitch, a dirty lens or a
    banner hung somewhere the rulebook drawing did not put it. That is one
    sample of one condition per hour. Industry practice for exactly this gap
    is a layered test pyramid: a fast, lower-fidelity closed loop for broad
    Monte Carlo coverage, and the high-fidelity simulator (then hardware in
    the loop, then flight) to confirm what it finds.

    This is the fast layer. The REAL mission tree, corridor navigator, camera
    controller and winch controller run unmodified; this node plays every
    other part -- the flight controller behind MAVROS, the airframe in the
    wind, the LD06, the camera's detectors, the payload on its line -- on
    simulated time, as fast as the host allows.

WHAT IT MODELS, AND AT WHAT FIDELITY

    flight controller  ArduPilot Copter in GUIDED: position targets with a
                       speed cap and a braking profile, body-frame velocity
                       targets with their 3 s timeout, TAKEOFF, LAND, RTL,
                       the inclusion geofence (breach -> RTL), parameters,
                       fence upload with read-back. The controller acts on
                       its ESTIMATE, so GPS and baro errors move the real
                       airframe exactly as they do in flight.
    airframe           first-order velocity response (vel_response_s), the
                       residual of wind drag the loop has not yet rejected
                       (test_corridor_stress.WindDrift), tilt from
                       acceleration, contact with any structure.
    LD06 lidar         450 beams ray-cast in the plane 0.08 m above the body
                       against every box whose height spans that plane --
                       walls, obstacles, banner posts, the banner board,
                       decoys -- then sim_gazebo.corruptions.LidarCorruptor.
    camera             not images: the detectors' OUTPUTS, derived from the
                       projected geometry the way the envelope tests measured
                       them (sim/test_perception_corruption.py): a pad reads
                       above 5.3 px/module, is only located below it or when
                       blurred, and each corruption severity takes its share
                       off the read rate. Banner identity needs the lettering
                       large and face-on enough. Red ground is projected
                       through the footprint and accumulated in the node's
                       own GroundGrid, so confirmation lags as it does live.
    payload            hangs under the airframe on the paid-out line, blown
                       downwind by its own drag, rests when it reaches the
                       ground, stays where the hook lets it go.

    Not modelled: rotor aerodynamics, the EKF's internals, rolling shutter as
    pixels, anything a real image would show that geometry does not. Those
    are what the Gazebo run is for.

OUTPUTS (in --out)

    track.csv    ground truth, in sim/check_track.py's format
    layout.json  the arena's ground truth, as materialize_world writes it
    run.json     result, contacts, fence breaches, what was injected

    python3 sim/fly_headless.py sim/worlds/my_world.json --conditions worst
"""

import argparse
import json
import math
import random
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Vector3
from mavros_msgs.msg import HomePosition, PositionTarget, State, WaypointList
from mavros_msgs.srv import (CommandBool, CommandLong, CommandTOL, ParamSetV2,
                             SetMode, WaypointPush)
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from std_msgs.msg import Bool, Empty, Float64, String

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "sim"))
for pkg in (("aerothon_sim", "sim_gazebo"), ("aerothon_mission", "mission_bt"),
            ("aerothon_perception", "perception_qr"),
            ("aerothon_perception", "perception_redzone")):
    sys.path.insert(0, str(ROOT / "src" / pkg[0] / pkg[1]))

import materialize_world as MW                              # noqa: E402
import world_spec as W                                      # noqa: E402
from mission_bt.geofence import (FENCE_POLYGON_INCLUSION,   # noqa: E402
                                 global_to_local)
from perception_qr.qr_match import matches                  # noqa: E402
from perception_redzone.georef import GroundGrid            # noqa: E402
from sim_gazebo.corruptions import CAMERA_KEYS, LidarCorruptor  # noqa: E402
from test_corridor_stress import WindDrift                  # noqa: E402

HOME_LATLON = (12.9692, 79.1559)       # VIT Vellore: any real place will do
DT = 0.01                              # physics step, s
G = 9.81

# The team airframe (airframe.json, build_cad_vehicle.py).
LIDAR_ABOVE_BODY_M = 0.0815
BODY_BELOW_M, BODY_ABOVE_M = 0.17, 0.16      # skids / prop tops, from base_link
AIRFRAME_R = 0.4
HOOK_BELOW_M = 0.20
IMAGE_W, IMAGE_H = 1280, 720
QR_MODULES = 33

# ArduPilot Copter defaults the mission meets (m, m/s, m/s^2, rad/s).
WPNAV_SPEED, WPNAV_ACCEL = 5.0, 2.5
SPEED_UP, SPEED_DN, LAND_SPEED = 2.5, 1.5, 0.5
YAW_RATE_MAX = math.radians(90.0)
ACCEL_MAX = WPNAV_ACCEL
GUIDED_VEL_TIMEOUT_S = 3.0
RTL_ALT = 15.0
# ArduPilot EKF3 GPS handling: innovations past EK3_POS_I_GATE (5 sigma of a
# ~0.6 m GPS) are rejected and the estimate dead-reckons, drifting; rejected
# for long enough it resets to the GPS.
EKF_GATE_M, EKF_RESET_S, EKF_COAST_MPS = 3.0, 10.0, 0.1

# The banner gate (mission2.sdf, materialize_world.banner_geometry), in the
# banner's own frame: board across y, posts either side.
BOARD_W, BOARD_H, BOARD_T = 3.7, 1.15, 0.12
POST_Y, POST_SIZE, POST_H = 1.92, 0.18, 4.0

# Detector envelope (sim/test_perception_corruption.py, docs/QR_DECODE_ENVELOPE.md).
PX_PER_MODULE_READ = 5.3        # reliable read at or above
PX_PER_MODULE_LOCATE = 2.5      # finder patterns still found at or above
BLUR_PX_BY_SEVERITY = (0, 4, 8, 14, 22, 32)
QR_SEV_50, QR_SEV_50_PER_PPM, QR_SEV_SCALE = 17.6, 0.79, 1.5   # _read_factor
EXPOSURE_S = 0.01               # the flight camera's capped exposure
BANNER_MIN_BOARD_PX = 20        # lettering legible from this board height
BANNER_MAX_INCIDENCE = math.radians(65.0)
# Banner identity vs total camera severity (sum over the corruptions, plus
# print wear): logistic, fitted to the REAL detector on the 3 m sim frame,
# 30 seeds per point -- severity 17 (the worst preset) 30/30, 26 27/30,
# 32 12/30 (tests/fixtures/banner_sim_ambient_3m.png).
BANNER_SEV_50 = 31.0
BANNER_SEV_SCALE = 2.5


def latched(depth=1):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def place(local_xy, origin, yaw):
    dx, dy = rot(local_xy[0], local_xy[1], yaw)
    return origin[0] + dx, origin[1] + dy


def rect(cx, cy, w, h, yaw):
    return [place(p, (cx, cy), yaw)
            for p in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]


def clip_convex(subject, clip):
    """Sutherland-Hodgman: `subject` polygon clipped to convex CCW `clip`."""
    def inside(p, a, b):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0

    def cut(p, q, a, b):
        x1, y1, x2, y2 = p[0], p[1], q[0], q[1]
        x3, y3, x4, y4 = a[0], a[1], b[0], b[1]
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-12:
            return q
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    out = list(subject)
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        src, out = out, []
        for j in range(len(src)):
            p, q = src[j], src[(j + 1) % len(src)]
            if inside(q, a, b):
                if not inside(p, a, b):
                    out.append(cut(p, q, a, b))
                out.append(q)
            elif inside(p, a, b):
                out.append(cut(p, q, a, b))
        if not out:
            return []
    return out


def ccw(poly):
    return poly if W.polygon_area(poly) > 0 else poly[::-1]


# --------------------------------------------------------------------------- #
# The arena, in the mission's home-local frame
# --------------------------------------------------------------------------- #

class Arena:
    """Everything the aircraft can hit or see, from a world spec."""

    def __init__(self, spec, start_letter):
        self.spec = spec
        self.home_world = W.spawn_point(spec)
        hx, hy = self.home_world
        self.home_yaw = math.radians(spec["takeoff"]["yaw_deg"])
        loc = lambda p: (p[0] - hx, p[1] - hy)              # noqa: E731
        self.to_local = loc

        self.boxes = []          # (poly, zmin, zmax, kind)
        lane = lambda c, obstacles: self._lane(c, obstacles, loc)  # noqa: E731
        lane(spec["corridor"], [])
        lane(spec["return_corridor"], W.obstacle_polygons(spec))
        bottom = float(spec["banner"]["board_bottom_m"])
        c, r = spec["corridor"], spec["return_corridor"]
        self.banners = [self._gate(loc((c["x"], c["y"])), math.radians(c["yaw_deg"]),
                                   bottom, "outbound"),
                        self._gate(loc((r["x"], r["y"])),
                                   math.radians(r["yaw_deg"]) + math.pi, bottom, "return")]
        # Decoys: a banner-shaped tarp and a smaller panel (green_decoy_visuals).
        self.green = []          # (centre xy, facing yaw, width, z0, z1) for the camera
        for d in spec["decoys"]:
            o, yaw = loc((d["x"], d["y"])), math.radians(d.get("yaw_deg", 0.0))
            for (lx, ly), (w, z0, z1) in (((0.0, 0.0), (3.4, 0.775, 1.825)),
                                           ((0.0, 2.6), (1.2, 0.30, 1.50))):
                cx, cy = place((lx, ly), o, yaw)
                self.boxes.append((ccw(rect(cx, cy, 0.10, w, yaw)), z0, z1, "decoy"))
                self.green.append(((cx, cy), yaw, w, z0, z1))

        payloads = {}
        for line in (ROOT / "src/aerothon_sim/sim_gazebo/materials/qr_payloads.txt"
                     ).read_text().splitlines():
            if line.startswith("qr_target_"):
                payloads[line[10]] = line.split(":", 1)[1].strip()
        size = float(spec["qr"]["target_m"])
        self.markers = [(loc((p["x"], p["y"])), size, math.radians(p.get("yaw_deg", 0.0)),
                         payloads[k]) for k, p in spec["pads"].items()]
        self.start_payload = payloads[start_letter]
        self.markers.append((loc(W.start_qr_point(spec)), float(spec["qr"]["start_m"]),
                             self.home_yaw, self.start_payload))
        self.target_xy = loc((spec["pads"][start_letter]["x"], spec["pads"][start_letter]["y"]))
        self.red = [ccw([loc(p) for p in W.red_polygon(rz)]) for rz in spec["red_zones"]]
        # Green GROUND the banner detector's mask takes in: the delivery zone
        # is grassed (mission2.sdf, 0.50 0.76 0.38 -- inside the detector's
        # hue/saturation band). In Gazebo it was the largest green in view
        # from my_world's take-off pad, and the search orbited it.
        zx0, zx1, zy0, zy1 = W.zone_rect(spec)
        self.green_ground = [ccw([loc(p) for p in
                                  ((zx0, zy0), (zx1, zy0), (zx1, zy1), (zx0, zy1))])]
        self.fence_local = [loc(p) for p in W.fence_polygon(spec)]
        self._edges()

    def _lane(self, c, obstacles, loc):
        L, Wd, H = float(c["length"]), float(c["width"]), float(c["wall_height"])
        yaw = math.radians(c["yaw_deg"])
        o = loc((c["x"], c["y"]))
        wy = Wd / 2 + W.WALL_T / 2
        for side in (1, -1):
            cx, cy = place((L / 2, side * wy), o, yaw)
            self.boxes.append((ccw(rect(cx, cy, L + 0.2, W.WALL_T, yaw)), 0.0, H, "wall"))
        for poly, ob in zip(obstacles, c.get("obstacles") or []):
            self.boxes.append((ccw([loc(p) for p in poly]), 0.0, float(ob["h"]), "obstacle"))

    def _gate(self, origin, facing, bottom, name):
        for side in (1, -1):
            cx, cy = place((0.0, side * POST_Y), origin, facing)
            self.boxes.append((ccw(rect(cx, cy, POST_SIZE, POST_SIZE, facing)),
                               0.0, max(POST_H, bottom + BOARD_H), "post"))
        self.boxes.append((ccw(rect(origin[0], origin[1], BOARD_T, BOARD_W, facing)),
                           bottom, bottom + BOARD_H, "board"))
        return {"xy": origin, "facing": facing, "z0": bottom,
                "z1": bottom + BOARD_H, "name": name}

    def _edges(self):
        self.edge_a, self.edge_b, self.edge_z0, self.edge_z1 = [], [], [], []
        for poly, z0, z1, _ in self.boxes:
            for i in range(len(poly)):
                self.edge_a.append(poly[i])
                self.edge_b.append(poly[(i + 1) % len(poly)])
                self.edge_z0.append(z0)
                self.edge_z1.append(z1)
        self.edge_a = np.array(self.edge_a)
        self.edge_b = np.array(self.edge_b)
        self.edge_z0 = np.array(self.edge_z0)
        self.edge_z1 = np.array(self.edge_z1)

    def clear_los(self, p, q, stop_short_m=0.2):
        """True when nothing solid stands between 3-D points p and q. The
        segment ends `stop_short_m` before q, so the thing looked at does not
        hide itself. A crossing of a box's side below its top is a hit."""
        p, q = np.asarray(p, float), np.asarray(q, float)
        d = q[:2] - p[:2]
        length = math.hypot(*d)
        if length <= stop_short_m:
            return True
        e = self.edge_b - self.edge_a
        w0 = self.edge_a - p[:2]
        den = d[0] * e[:, 1] - d[1] * e[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (w0[:, 0] * e[:, 1] - w0[:, 1] * e[:, 0]) / den
            u = (w0[:, 0] * d[1] - w0[:, 1] * d[0]) / den
        cross = (np.abs(den) > 1e-12) & (t > 0.0) & (t < 1.0 - stop_short_m / length) \
            & (u >= 0.0) & (u <= 1.0)
        z = p[2] + t * (q[2] - p[2])
        return not np.any(cross & (z >= self.edge_z0) & (z <= self.edge_z1))

    def raycast(self, x, y, z_plane, yaw, n=450, rmax=12.0):
        """LD06 ranges, counter-clockwise from -pi, inf where nothing is hit."""
        live = (self.edge_z0 <= z_plane) & (self.edge_z1 >= z_plane)
        a, b = self.edge_a[live], self.edge_b[live]
        ang = yaw - math.pi + np.arange(n) * (2 * math.pi / n)
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)               # (n,2)
        if not len(a):
            return np.full(n, np.inf)
        e = b - a                                                       # (m,2)
        w0 = a - np.array([x, y])                                       # (m,2)
        den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (w0[None, :, 0] * e[None, :, 1] - w0[None, :, 1] * e[None, :, 0]) / den
            u = (w0[None, :, 0] * d[:, None, 1] - w0[None, :, 1] * d[:, None, 0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        r = np.where(hit, t, np.inf).min(axis=1)
        r[r > rmax] = np.inf
        return r

    def contact(self, x, y, z):
        """The first structure the airframe's disc overlaps, or None."""
        for poly, z0, z1, kind in self.boxes:
            if z1 < z - BODY_BELOW_M or z0 > z + BODY_ABOVE_M:
                continue
            if W.point_in_polygon((x, y), poly) or \
                    W.distance_to_edge((x, y), poly) < AIRFRAME_R:
                return kind
        return None


# --------------------------------------------------------------------------- #
# The camera: projection and the detectors' envelope
# --------------------------------------------------------------------------- #

class Camera:
    """Pinhole C270 on a pitch servo; `pitch` is radians DOWN from level."""

    def __init__(self, hfov):
        self.hfov = hfov
        self.f = (IMAGE_W / 2) / math.tan(hfov / 2)

    def axes(self, yaw, pitch):
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        optical = fwd * math.cos(pitch) - up * math.sin(pitch)
        down = -up * math.cos(pitch) - fwd * math.sin(pitch)
        return optical, -left, down

    def project(self, pts, eye, yaw, pitch):
        """(N,3) world points -> (N,2) pixels and a mask of those in front."""
        optical, right, down = self.axes(yaw, pitch)
        d = np.asarray(pts, float) - np.asarray(eye, float)
        zc = d @ optical
        front = zc > 0.2
        zc = np.where(front, zc, 1.0)
        u = IMAGE_W / 2 + self.f * (d @ right) / zc
        v = IMAGE_H / 2 + self.f * (d @ down) / zc
        return np.stack([u, v], axis=1), front

    def ground_view(self, eye, yaw, pitch, rmax=150.0, n=8):
        """The ground the frame covers out to `rmax`, as a polygon. The rays
        through the frame's border that pass above the horizon are cut off at
        `rmax`, so this works for a camera looking ahead as well as down."""
        optical, right, down = self.axes(yaw, pitch)
        border = ([(IMAGE_W * i / n, 0) for i in range(n)]
                  + [(IMAGE_W, IMAGE_H * i / n) for i in range(n)]
                  + [(IMAGE_W * (n - i) / n, IMAGE_H) for i in range(n)]
                  + [(0, IMAGE_H * (n - i) / n) for i in range(n)])
        out = []
        for u, v in border:
            ray = optical + right * (u - IMAGE_W / 2) / self.f + down * (v - IMAGE_H / 2) / self.f
            flat = math.hypot(ray[0], ray[1])
            t = -eye[2] / ray[2] if ray[2] < -1e-6 else math.inf
            t = min(t, rmax / max(flat, 1e-6))
            out.append((eye[0] + t * ray[0], eye[1] + t * ray[1]))
        return out

    def footprint(self, eye, yaw, pitch):
        """Ground polygon the frame covers, or [] when it reaches the horizon."""
        optical, right, down = self.axes(yaw, pitch)
        out = []
        for u, v in ((0, 0), (IMAGE_W, 0), (IMAGE_W, IMAGE_H), (0, IMAGE_H)):
            ray = optical + right * (u - IMAGE_W / 2) / self.f + down * (v - IMAGE_H / 2) / self.f
            if ray[2] > -0.05:
                return []
            t = -eye[2] / ray[2]
            out.append((eye[0] + t * ray[0], eye[1] + t * ray[1]))
        return ccw(out)


def in_frame(px, margin=0.0):
    return ((px[:, 0] >= -margin) & (px[:, 0] <= IMAGE_W + margin)
            & (px[:, 1] >= -margin) & (px[:, 1] <= IMAGE_H + margin))


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #

class HeadlessWorld(Node):

    def __init__(self, args):
        super().__init__("headless_world")
        self.args = args
        spec = W.load(args.spec)
        cond_spec = {"conditions": {"preset": args.conditions}} if args.conditions \
            else {"conditions": spec["conditions"]}
        self.cond = W.conditions(cond_spec, args.seed)
        rng = random.Random(args.seed)
        start = spec["start_target"]
        self.start_letter = start if start in W.PAD_LETTERS else rng.choice(W.PAD_LETTERS)
        self.arena = Arena(spec, self.start_letter)
        self.camera = Camera(args.hfov)
        self.np_rng = np.random.default_rng(args.seed)
        self.rng = rng
        self.lidar = LidarCorruptor(self.cond["lidar"], args.seed + 1)
        cam = self.cond["camera"]
        self.cam_sev = {k: int(cam[k]) for k in CAMERA_KEYS}
        self.frame_drop = float(cam["frame_drop"])
        self.latency = float(cam["latency_ms"]) / 1000.0

        w = self.cond["wind"]
        self.wind = None
        if w["speed"] > 0:
            self.wind = WindDrift(mean=w["speed"], dir_rad=math.radians(w["dir_deg"]),
                                  gust=w["gust"], period=w["gust_period_s"],
                                  turb=0.5 * w["veer_deg"] / 20.0 * w["speed"] / 8.0,
                                  seed=args.seed, dt=DT)
        self.fcu = self.cond["fcu"]

        # ---- truth ----
        self.t = 0.0
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a_prev = np.zeros(3)
        self.yaw = self.arena.home_yaw
        self.roll = self.pitch = 0.0
        # ---- flight controller ----
        self.armed = False
        self.mode = "STABILIZE"
        self.sp_pos = None           # (x, y, z, yaw) estimate frame
        self.sp_vel = None           # (vx, vy, vz body FLU, yaw_rate, t)
        self.takeoff_alt = None
        self.speed_cap = WPNAV_SPEED
        self.params = {}
        self.fence_items = []
        self.fence_poly = None
        self.landed_t = None
        # ---- estimator error ----
        self.gps_err = np.zeros(2)
        self.glitch = np.zeros(2)            # the glitch's share of the estimate
        self.glitch_gps = np.zeros(2)        # the glitch in the GPS itself
        self.glitch_phase = 0
        self.rejected_s = 0.0
        self.coast_v = np.zeros(2)
        self.armed_at = None
        self.baro_err = 0.0
        self.baro_noise = 0.0
        # ---- camera servo, winch, payload ----
        self.cam_cmd = 0.0           # joint rad, negative = down
        self.cam_joint = 0.0
        self.payout = 0.0
        self.detached = False
        self.payload = np.array([0.0, 0.0, 0.04])
        self.payload_resting = True
        self.target_str = ""
        self.grid = GroundGrid(cell_m=1.0, confirm_hits=3)
        self.pending_frames = []     # (release_t, outputs)
        # ---- bookkeeping ----
        self.track = []
        self.events = []
        self.contacts = []
        self.fence_breaches = 0
        self.result = None
        self.mission_state = ""
        self.started = False
        self.lock = threading.Lock()

        self._advertise()
        self._serve()
        self.get_logger().info(
            f"headless world: {spec.get('name')} start->{self.start_letter.upper()} "
            f"conditions {json.dumps(self.cond, sort_keys=True)}")

    # ---- ROS plumbing --------------------------------------------------------
    def _advertise(self):
        P = self.create_publisher
        self.pub_clock = P(Clock, "/clock", 10)
        self.pub_state = P(State, "/mavros/state", 10)
        self.pub_pose = P(PoseStamped, "/mavros/local_position/pose", qos_profile_sensor_data)
        self.pub_vel = P(TwistStamped, "/mavros/local_position/velocity_local",
                         qos_profile_sensor_data)
        self.pub_batt = P(BatteryState, "/mavros/battery", qos_profile_sensor_data)
        self.pub_home = P(HomePosition, "/mavros/home_position/home", latched())
        self.pub_fences = P(WaypointList, "/mavros/geofence/fences", latched())
        self.pub_scan = P(LaserScan, "/scan", qos_profile_sensor_data)
        self.pub_joint = P(JointState, "/joint_states", qos_profile_sensor_data)
        self.pub_qr = P(String, "/percep/qr/decoded", 10)
        self.pub_match = P(Bool, "/percep/qr/matched", 10)
        self.pub_off = P(Vector3, "/percep/qr/target_offset", 10)
        self.pub_qr_detail = P(String, "/percep/qr/detail", 10)
        self.pub_banner = P(Vector3, "/percep/banner", 10)
        self.pub_banner_detail = P(String, "/percep/banner/detail", 10)
        self.pub_red = P(String, "/percep/redzone/detail", 10)
        self.pub_payload = P(String, "/percep/payload", 10)
        self.pub_payload_pose = P(Pose, "/sim/payload_pose", qos_profile_sensor_data)
        self.pub_start = P(Bool, "/mission/start", 10)
        S = self.create_subscription
        S(PoseStamped, "/mavros/setpoint_position/local", self._on_sp_pos, 10)
        S(PositionTarget, "/mavros/setpoint_raw/local", self._on_sp_raw, 10)
        S(Float64, "/gimbal/cmd_pitch", lambda m: setattr(self, "cam_cmd", float(m.data)), 10)
        S(Float64, "/winch/gz/payout", lambda m: setattr(self, "payout", max(0.0, m.data)), 10)
        S(Empty, "/winch/gz/detach", self._on_detach, 10)
        S(String, "/mission/target", lambda m: setattr(self, "target_str", m.data), 10)
        S(String, "/mission/result", self._on_result, latched())
        S(String, "/mission/state", lambda m: setattr(self, "mission_state", m.data), 10)

    def _serve(self):
        C = self.create_service
        C(CommandBool, "/mavros/cmd/arming", self._srv_arm)
        C(SetMode, "/mavros/set_mode", self._srv_mode)
        C(CommandTOL, "/mavros/cmd/takeoff", self._srv_takeoff)
        C(CommandTOL, "/mavros/cmd/land", self._srv_land)
        C(CommandLong, "/mavros/cmd/command", self._srv_command)
        C(ParamSetV2, "/mavros/param/set", self._srv_param)
        C(WaypointPush, "/mavros/geofence/push", self._srv_fence)

    def stamp(self):
        return TimeMsg(sec=int(self.t), nanosec=int((self.t % 1.0) * 1e9))

    # ---- flight controller services ----------------------------------------
    def _event(self, what):
        self.events.append((round(self.t, 2), what))
        self.get_logger().info(f"[t={self.t:7.2f}] {what}")

    def _srv_arm(self, req, res):
        with self.lock:
            # A plain bool: self.p is numpy, and the message bindings assert
            # PyBool_Check on the reply (apt ROS builds keep asserts on).
            ok = bool((not req.value)
                      or (self.mode == "GUIDED" and self.p[2] < 0.2))
            if ok:
                self.armed = bool(req.value)
                if self.armed and self.armed_at is None:
                    self.armed_at = self.t
                self._event(f"{'armed' if self.armed else 'disarmed'} by command")
        res.success, res.result = ok, 0 if ok else 4
        return res

    def _srv_mode(self, req, res):
        with self.lock:
            self.mode = req.custom_mode
            if self.mode == "GUIDED":
                self.sp_pos = self.sp_vel = None
            self._event(f"mode {self.mode}")
        res.mode_sent = True
        return res

    def _srv_takeoff(self, req, res):
        with self.lock:
            ok = bool(self.armed and self.mode == "GUIDED")
            if ok:
                self.takeoff_alt = float(req.altitude)
                self.sp_pos = self.sp_vel = None
        res.success, res.result = ok, 0 if ok else 4
        return res

    def _srv_land(self, req, res):
        with self.lock:
            self.mode = "LAND"
            self._event("mode LAND")
        res.success, res.result = True, 0
        return res

    def _srv_command(self, req, res):
        with self.lock:
            if int(req.command) == 178 and req.param2 > 0:      # DO_CHANGE_SPEED
                self.speed_cap = float(req.param2)
        res.success, res.result = True, 0
        return res

    def _srv_param(self, req, res):
        v = req.value
        value = v.integer_value if v.type == 2 else v.double_value
        with self.lock:
            self.params[req.param_id] = value
        res.success, res.value = True, req.value
        return res

    def _srv_fence(self, req, res):
        with self.lock:
            self.fence_items = list(req.waypoints)
            incl = [w for w in self.fence_items if int(w.command) == FENCE_POLYGON_INCLUSION]
            self.fence_poly = [global_to_local(w.x_lat, w.y_long, *HOME_LATLON)
                               for w in incl] or None
        self.pub_fences.publish(WaypointList(current_seq=0, waypoints=self.fence_items))
        res.success, res.wp_transfered = True, len(req.waypoints)
        return res

    def _on_sp_pos(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            if self.mode == "GUIDED" and self.armed:
                p = m.pose.position
                self.sp_pos = (p.x, p.y, p.z, yaw)
                self.sp_vel = None
                self.takeoff_alt = None

    def _on_sp_raw(self, m):
        with self.lock:
            if self.mode == "GUIDED" and self.armed:
                self.sp_vel = (m.velocity.x, m.velocity.y, m.velocity.z, m.yaw_rate, self.t)
                self.sp_pos = None
                self.takeoff_alt = None

    def _on_detach(self, _):
        with self.lock:
            if not self.detached:
                self.detached = True
                self._event(f"payload released at ({self.payload[0]:.2f}, "
                            f"{self.payload[1]:.2f}, {self.payload[2]:.2f})")

    def _on_result(self, m):
        if self.result is None:
            self.result = json.loads(m.data)
            self._event(f"mission result {self.result.get('state')}: "
                        f"{self.result.get('reason')}")

    # ---- estimator ------------------------------------------------------------
    def est(self):
        """What the flight controller believes: truth plus GPS and baro error."""
        return np.array([self.p[0] + self.gps_err[0] + self.glitch[0],
                         self.p[1] + self.gps_err[1] + self.glitch[1],
                         self.p[2] + self.baro_err + self.baro_noise])

    def _step_errors(self):
        # GPS: a slowly wandering error (the receiver's), sigma = noise / 2.
        s = 0.5 * self.fcu["gps_noise_m"]
        self.gps_err += (-self.gps_err / 20.0) * DT + s * math.sqrt(2 * DT / 20.0) \
            * self.np_rng.normal(size=2)
        self.baro_err += self.fcu["baro_drift_mps"] * DT
        # The EKF blends the baro with the accelerometers: its height carries
        # about half the sensor's noise, correlated over ~1 s -- and it is the
        # one estimate both the position controller and MAVROS see.
        s = 0.5 * self.fcu["baro_noise_m"]
        self.baro_noise += -self.baro_noise * DT + s * math.sqrt(2 * DT) \
            * self.np_rng.normal()
        g = self.fcu["gps_glitch_m"]
        if g > 0 and self.armed_at is not None:
            since = self.t - self.armed_at
            if self.glitch_phase == 0 and since >= self.fcu["glitch_at_s"]:
                a = self.rng.uniform(0, 2 * math.pi)
                self.glitch_gps = np.array([g * math.cos(a), g * math.sin(a)])
                b = self.rng.uniform(0, 2 * math.pi)
                self.coast_v = EKF_COAST_MPS * np.array([math.cos(b), math.sin(b)])
                self.glitch_phase = 1
                self._event(f"GPS glitch {g:.1f} m")
            elif self.glitch_phase == 1 and since >= self.fcu["glitch_at_s"] + self.fcu["glitch_s"]:
                self.glitch_gps = np.zeros(2)
                self.glitch_phase = 2
                self._event("GPS glitch cleared")
        # The EKF's innovation gate: a GPS jump beyond it is rejected and the
        # estimate coasts on the IMU; still rejected after EKF_RESET_S, it
        # resets onto the GPS. Within the gate the GPS is fused (~1 s).
        off = self.glitch_gps - self.glitch
        if float(np.hypot(*off)) <= EKF_GATE_M:
            self.glitch += off * min(1.0, DT / 1.0)
            self.rejected_s = 0.0
        elif self.rejected_s >= EKF_RESET_S:
            self.glitch = self.glitch_gps.copy()
            self.rejected_s = 0.0
            self._event("EKF reset onto the GPS")
        else:
            self.rejected_s += DT
            self.glitch += self.coast_v * DT

    # ---- flight controller + airframe --------------------------------------
    def _desired_velocity(self, est):
        """What ArduPilot's position/velocity controllers ask of the airframe."""
        if not self.armed:
            return np.zeros(3), 0.0
        if self.mode == "LAND":
            return np.array([0.0, 0.0, -LAND_SPEED if est[2] > 1.0 else -0.3]), 0.0
        if self.mode == "RTL":
            if est[2] < RTL_ALT - 0.3 and math.hypot(*est[:2]) > 1.0:
                return np.array([0.0, 0.0, SPEED_UP]), 0.0
            return self._to_point(est, (0.0, 0.0, RTL_ALT), None)[0] \
                if math.hypot(*est[:2]) > 1.0 else np.array([0.0, 0.0, -LAND_SPEED]), 0.0
        if self.mode != "GUIDED":
            return np.zeros(3), 0.0
        if self.takeoff_alt is not None:
            return self._to_point(est, (est[0], est[1], self.takeoff_alt), None)
        if self.sp_vel is not None:
            vx, vy, vz, yr, t0 = self.sp_vel
            if self.t - t0 > GUIDED_VEL_TIMEOUT_S:
                return np.zeros(3), 0.0
            wx, wy = rot(vx, vy, self.yaw)
            return np.array([wx, wy, vz]), yr
        if self.sp_pos is not None:
            x, y, z, yaw = self.sp_pos
            return self._to_point(est, (x, y, z), yaw)
        return np.zeros(3), 0.0

    def _to_point(self, est, target, yaw):
        d = np.array(target) - est
        h = math.hypot(d[0], d[1])
        cap = min(self.speed_cap, math.sqrt(2 * WPNAV_ACCEL * h))
        v = np.zeros(3)
        if h > 1e-3:
            v[:2] = d[:2] / h * min(cap, 1.2 * h)
        v[2] = max(-SPEED_DN, min(SPEED_UP, 1.2 * d[2]))
        yr = 0.0
        if yaw is not None:
            err = (yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            yr = max(-YAW_RATE_MAX, min(YAW_RATE_MAX, 2.0 * err))
        return v, yr

    def _step_airframe(self):
        est = self.est()
        v_des, yr = self._desired_velocity(est)
        on_ground = self.p[2] <= 0.0 and v_des[2] <= 0.0
        a = (v_des - self.v) / np.array([self.args.vel_response_s] * 2 + [0.4])
        # ANGLE_MAX: the attitude controller will not lean further than this
        # to accelerate, whatever the velocity loop asks.
        ah = math.hypot(a[0], a[1])
        if ah > ACCEL_MAX:
            a[:2] *= ACCEL_MAX / ah
        if not self.armed or on_ground:
            a[:] = 0.0
            self.v[:] = 0.0
        self.v += a * DT
        drift = np.zeros(3)
        if self.wind is not None and self.p[2] > 0.3:
            dx, dy = self.wind(self.t)
            drift[:2] = (dx, dy)
        self.p += (self.v + drift) * DT
        self.yaw = (self.yaw + yr * DT + math.pi) % (2 * math.pi) - math.pi
        if self.p[2] < 0.0:
            self.p[2] = 0.0
            self.v[2] = max(0.0, self.v[2])
        # Tilt from horizontal acceleration, in the body frame.
        ab = rot(a[0], a[1], -self.yaw)
        self.pitch = math.degrees(math.atan2(ab[0], G))
        self.roll = math.degrees(math.atan2(-ab[1], G))

        hit = self.arena.contact(self.p[0], self.p[1], self.p[2]) if self.p[2] > 0.3 else None
        if hit is not None:
            if not self.contacts or self.t - self.contacts[-1][0] > 1.0:
                self.contacts.append((round(self.t, 2), hit, round(self.p[0], 2),
                                      round(self.p[1], 2), round(self.p[2], 2)))
                self._event(f"CONTACT with {hit} at ({self.p[0]:.2f}, {self.p[1]:.2f}, "
                            f"{self.p[2]:.2f})")
            # A strike: pushed back, knocked about.
            self.p[:2] -= self.v[:2] * 0.05
            self.v[:2] *= -0.3
            self.pitch = 50.0

        if self.fence_poly and float(self.params.get("FENCE_ENABLE", 0)) >= 1 \
                and self.armed and self.mode == "GUIDED":
            if not W.point_in_polygon((est[0], est[1]), self.fence_poly):
                self.fence_breaches += 1
                self.mode = "RTL"
                self._event(f"GEOFENCE BREACH at ({est[0]:.1f}, {est[1]:.1f}): RTL")

        if self.armed and self.mode in ("LAND", "RTL") and self.p[2] <= 0.02:
            self.landed_t = self.landed_t or self.t
            if self.t - self.landed_t > 1.0:
                self.armed = False
                self.mode = "LAND"
                self._event(f"landed and disarmed at ({self.p[0]:.2f}, {self.p[1]:.2f})")
        else:
            self.landed_t = None

    def _step_payload(self):
        if self.detached:
            if self.payload[2] > 0.04:
                self.payload[2] = max(0.04, self.payload[2] - 3.0 * DT)
            return
        hang = self.p[2] - HOOK_BELOW_M - self.payout
        off = np.zeros(2)
        if self.wind is not None and self.payout > 0.3:
            # 10 x 5 cm face, Cd 1, 100 g: the line leans downwind by drag/weight.
            wmean = self.wind.u * self.wind.mean
            drag = 0.5 * 1.2 * 0.005 * float(np.linalg.norm(wmean)) ** 2
            off = self.wind.u * self.payout * math.sin(math.atan2(drag, 0.1 * G))
        if hang <= 0.04:
            if not self.payload_resting:
                self._event(f"payload touched down at ({self.payload[0]:.2f}, "
                            f"{self.payload[1]:.2f})")
            self.payload_resting = True
            self.payload[2] = 0.04
            return
        self.payload_resting = False
        self.payload[:] = (self.p[0] + off[0], self.p[1] + off[1], hang)

    # ---- sensors ---------------------------------------------------------------
    def _publish_fcu(self, est):
        stamp = self.stamp()
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = stamp, "map"
        m.pose.position.x, m.pose.position.y = float(est[0]), float(est[1])
        m.pose.position.z = float(est[2])
        r, p, y = math.radians(self.roll), math.radians(self.pitch), self.yaw
        cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                                  math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
        m.pose.orientation.w = cr * cp * cy + sr * sp * sy
        m.pose.orientation.x = sr * cp * cy - cr * sp * sy
        m.pose.orientation.y = cr * sp * cy + sr * cp * sy
        m.pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.pub_pose.publish(m)
        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = (float(v) for v in self.v)
        self.pub_vel.publish(tw)

    def _publish_slow(self):
        self.pub_state.publish(State(connected=True, armed=self.armed,
                                     guided=self.mode == "GUIDED", mode=self.mode,
                                     system_status=4 if self.armed else 3))
        b = BatteryState()
        flown = max(0.0, self.t - (self.armed_at or self.t))
        b.voltage = float(self.fcu["battery_v"] - 0.15 * flown / 60.0
                          - (0.6 if self.armed and self.p[2] > 0.2 else 0.0))
        b.percentage = float(max(0.0, 1.0 - flown / 1200.0))
        self.pub_batt.publish(b)
        if abs(self.t % 1.0) < 0.1:          # MAVROS sends home at ~1 Hz
            h = HomePosition()
            h.header.stamp = self.stamp()
            h.geo.latitude, h.geo.longitude = HOME_LATLON
            self.pub_home.publish(h)
        self.pub_payload_pose.publish(self._payload_pose())
        js = JointState()
        js.header.stamp = self.stamp()
        js.name, js.position = ["webcam_pitch_joint"], [float(self.cam_joint)]
        self.pub_joint.publish(js)

    def _payload_pose(self):
        hx, hy = self.arena.home_world
        p = Pose()
        p.position.x = float(self.payload[0] + hx)
        p.position.y = float(self.payload[1] + hy)
        p.position.z = float(self.payload[2])
        return p

    def _publish_scan(self):
        r = self.arena.raycast(self.p[0], self.p[1], self.p[2] + LIDAR_ABOVE_BODY_M,
                               self.yaw)
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.stamp(), "base_scan"
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / len(r)
        s.angle_max = s.angle_min + s.angle_increment * (len(r) - 1)
        s.range_min, s.range_max = 0.05, 12.0
        s.scan_time, s.time_increment = 0.1, 0.1 / len(r)
        s.ranges = [float(x) for x in self.lidar.apply(r, 0.05, 12.0)]
        self.pub_scan.publish(s)

    # ---- camera: the detectors' outputs ---------------------------------------
    def _read_factor(self, ppm):
        """Share of frames a marker at `ppm` px/module still reads through
        the camera corruptions: logistic in their total severity, with a
        midpoint that rises with resolution. Fitted to the REAL decoder
        (sim/test_perception_corruption.render_pad, 24 seeds each): at the
        worst preset's total 17, 22/24 at 12.8 px/module and 17/24 at 8.5;
        at 26, none. Worn print costs contrast, which the reader measured
        almost indifferent to (haze and dust read at every severity)."""
        sev = sum(abs(v) for v in self.cam_sev.values())
        mid = QR_SEV_50 + QR_SEV_50_PER_PPM * (ppm - 8.5)
        p = 1.0 / (1.0 + math.exp((sev - mid) / QR_SEV_SCALE))
        return p * (1.0 - 0.02 * self.cond["wear"]["qr"])

    def _camera_frame(self):
        eye = self.p + np.array([0.16 * math.cos(self.yaw), 0.16 * math.sin(self.yaw), 0.0])
        pitch = -self.cam_joint
        out = {"qr": self._qr(eye, pitch), "banner": self._banner(eye, pitch),
               "red": self._red(eye, pitch), "payload": self._payload_det(eye, pitch)}
        return out

    def _qr(self, eye, pitch):
        speed = float(np.linalg.norm(self.v))
        best, boxes = None, []
        for (xy, size, yaw, payload) in self.arena.markers:
            corners = [(x, y, 0.1) for x, y in rect(xy[0], xy[1], size, size, yaw)]
            px, front = self.camera.project(corners, eye, self.yaw, pitch)
            if not front.all() or not in_frame(px).all():
                continue
            span = float(max(np.ptp(px[:, 0]), np.ptp(px[:, 1])))
            ppm = span / QR_MODULES
            rng = math.dist(eye, (xy[0], xy[1], 0.1))
            blur_px = speed * EXPOSURE_S * self.camera.f / max(rng, 0.5)
            sev_blur = max(self.cam_sev["motion_blur"],
                           sum(1 for b in BLUR_PX_BY_SEVERITY[1:] if blur_px >= b))
            p_read = 0.0
            if ppm >= 4.0:
                p_read = min(1.0, 0.3 + 0.65 * (ppm - 4.0) / (PX_PER_MODULE_READ - 4.0))
                p_read *= (1.0, 1.0, 1.0, 0.5, 0.1, 0.0)[sev_blur] * self._read_factor(ppm)
            cx, cy = px.mean(axis=0)
            read = self.rng.random() < p_read
            located = (not read) and ppm >= PX_PER_MODULE_LOCATE and self.rng.random() < 0.9
            if not (read or located):
                continue
            is_match = read and matches(self.target_str, payload)
            cand = (is_match, payload if read else "", float(cx), float(cy), span, read)
            boxes.append({"label": payload[:24] if read else "UNREAD", "ok": is_match})
            key = (cand[0], cand[5], cand[4])
            if best is None or key > (best[0], best[5], best[4]):
                best = cand
        off = Vector3()
        accepted, matched = "", False
        if best is not None:
            matched, accepted, cx, cy, span, read = best
            off.x = (cx - IMAGE_W / 2) / (IMAGE_W / 2)
            off.y = (cy - IMAGE_H / 2) / (IMAGE_H / 2)
            off.z = (1.0 if matched else 0.5) if read else 0.25
        return {"off": off, "decoded": accepted, "matched": matched,
                "detail": {"accepted": accepted, "matched": matched,
                           "target": self.target_str, "boxes": boxes,
                           "unread": bool(best and not best[5])}}

    def _board_view(self, xy, facing, width, z0, z1, eye, pitch):
        """(bbox, centre px, incidence rad, px height) of a vertical board,
        or None when it is out of view or mostly hidden behind something."""
        zm = (z0 + z1) / 2
        # Sighted 0.15 m in front of the face the eye is on. Sighted on the
        # board's own centre plane, every line of sight to an edge-on board
        # ran inside its 0.12 m box and hid it, where Gazebo draws the green
        # sliver (58 px from my_world's take-off pad) the search steers by.
        normal = np.array([math.cos(facing), math.sin(facing)])
        side = 0.15 * (1.0 if normal @ (np.asarray(eye[:2]) - xy) >= 0 else -1.0)
        seen = sum(self.arena.clear_los(
            eye, (*(np.array(place((0.0, s * width / 3), xy, facing)) + side * normal), zm))
            for s in (-1, 0, 1))
        if seen < 2:
            return None
        corners = []
        for side in (-0.5, 0.5):
            bx, by = place((0.0, side * width), xy, facing)
            corners += [(bx, by, z0), (bx, by, z1)]
        px, front = self.camera.project(corners, eye, self.yaw, pitch)
        if not front.all():
            return None
        x0, y0 = px.min(axis=0)
        x1, y1 = px.max(axis=0)
        if x1 < 0 or x0 > IMAGE_W or y1 < 0 or y0 > IMAGE_H:
            return None
        normal = np.array([math.cos(facing), math.sin(facing)])
        view = np.array(xy) - eye[:2]
        dist = float(np.linalg.norm(view))
        inc = math.acos(min(1.0, abs(float(normal @ view)) / max(dist, 1e-6)))
        cx, cy = x0 + x1, y0 + y1
        frac_in = (min(x1, IMAGE_W) - max(x0, 0)) / max(x1 - x0, 1e-6)
        return ([int(max(0, x0)), int(max(0, y0)), int(min(IMAGE_W, x1) - max(0, x0)),
                 int(min(IMAGE_H, y1) - max(0, y0))], (cx / 2, cy / 2), inc,
                float(y1 - y0), frac_in, dist)

    def _ground_green_view(self, poly, eye, pitch):
        """(bbox, centre px) of green ground `poly` in the frame, or None."""
        part = clip_convex(self.camera.ground_view(eye, self.yaw, pitch), poly)
        if len(part) < 3:
            return None
        px, front = self.camera.project([(x, y, 0.0) for x, y in part], eye, self.yaw, pitch)
        px = px[front]
        if len(px) < 3:
            return None
        x0, y0 = np.clip(px.min(axis=0), 0, [IMAGE_W, IMAGE_H])
        x1, y1 = np.clip(px.max(axis=0), 0, [IMAGE_W, IMAGE_H])
        if x1 - x0 < 2 or y1 - y0 < 2:
            return None
        return ([int(x0), int(y0), int(x1 - x0), int(y1 - y0)],
                ((x0 + x1) / 2, (y0 + y1) / 2))

    def _banner(self, eye, pitch):
        detail = {"identified": False, "reason": "", "candidates": 0,
                  "image_wh": [IMAGE_W, IMAGE_H]}
        out = Vector3()
        views = []
        for b in self.arena.banners:
            v = self._board_view(b["xy"], b["facing"], BOARD_W, b["z0"], b["z1"], eye, pitch)
            if v is not None:
                views.append(("banner", v))
        for (xy, yaw, w, z0, z1) in self.arena.green:
            v = self._board_view(xy, yaw, w, z0, z1, eye, pitch)
            if v is not None:
                views.append(("decoy", v))
        for poly in self.arena.green_ground:
            v = self._ground_green_view(poly, eye, pitch)
            if v is not None:
                views.append(("ground", v))
        if not views:
            return {"vec": out, "detail": detail}
        detail["candidates"] = len(views)
        ranked = sorted(views, key=lambda kv: -kv[1][0][2] * kv[1][0][3])
        big = ranked[0][1]
        detail["green_px"] = big[0]
        detail["green_area_px"] = big[0][2] * big[0][3]
        detail["green_bearing"] = round((big[1][0] - IMAGE_W / 2) / (IMAGE_W / 2), 3)
        detail["green_regions"] = [[*v[0], v[0][2] * v[0][3]] for _, v in ranked[:4]]
        ident = [v for kind, v in views if kind == "banner"
                 and v[3] >= BANNER_MIN_BOARD_PX and v[2] <= BANNER_MAX_INCIDENCE
                 and v[4] >= 0.7 and v[5] <= 15.0]
        sev = sum(abs(s) for s in self.cam_sev.values()) + self.cond["wear"]["banner"]
        p_ident = 1.0 / (1.0 + math.exp((sev - BANNER_SEV_50) / BANNER_SEV_SCALE))
        if ident and self.rng.random() < p_ident:
            bbox, (cx, cy), _, _, _, _ = max(ident, key=lambda v: v[3])
            out.x = (cx - IMAGE_W / 2) / (IMAGE_W / 2)
            out.y = (cy - IMAGE_H / 2) / (IMAGE_H / 2)
            out.z = 1.0
            detail.update(identified=True, board_px=bbox, bearing=round(out.x, 3),
                          board_area_px=bbox[2] * bbox[3],
                          board_aspect=round(bbox[2] / max(1, bbox[3]), 2),
                          text_confirmed=True)
        else:
            out.z = 0.5
            detail["reason"] = "no lettering legible (edge-on, distant or not the banner)"
        return {"vec": out, "detail": detail}

    def _red(self, eye, pitch):
        detail = {"status": "NOT_VISIBLE", "reason": "", "exclusions": []}
        if abs(pitch - math.pi / 2) <= 0.35 and self.p[2] >= 1.5:
            fp = self.camera.footprint(eye, self.yaw, pitch)
            if fp:
                shift = self.est()[:2] - self.p[:2]
                cells = set()
                area = W.polygon_area(fp)
                for red in self.arena.red:
                    part = clip_convex(red, fp)
                    if len(part) >= 3 and W.polygon_area(part) >= 0.002 * area:
                        cells |= self.grid.polygon_cells([(x + shift[0], y + shift[1])
                                                          for x, y in part])
                self.grid.add_cells(cells)
                detail["status"] = "RED" if cells else "CLEAR"
        detail["exclusions"] = [[round(v, 2) for v in ex]
                                for ex in self.grid.exclusions(inflate_m=1.0)]
        return detail

    def _payload_det(self, eye, pitch):
        det = {"visible": False, "img_w": IMAGE_W, "img_h": IMAGE_H}
        s = 0.06
        corners = [(self.payload[0] + dx, self.payload[1] + dy, self.payload[2])
                   for dx, dy in ((-s, -s), (s, -s), (s, s), (-s, s))]
        px, front = self.camera.project(corners, eye, self.yaw, pitch)
        if front.all() and in_frame(px).all():
            w, h = float(np.ptp(px[:, 0])), float(np.ptp(px[:, 1]))
            if w * h >= 20:
                cx, cy = px.mean(axis=0)
                det.update(visible=True, x=(cx - IMAGE_W / 2) / (IMAGE_W / 2),
                           y=(cy - IMAGE_H / 2) / (IMAGE_H / 2), area_px=w * h,
                           w_px=int(w), h_px=int(h))
        return det

    def _deliver(self, frame):
        q = frame["qr"]
        self.pub_off.publish(q["off"])
        self.pub_qr.publish(String(data=q["decoded"]))
        self.pub_match.publish(Bool(data=q["matched"]))
        self.pub_qr_detail.publish(String(data=json.dumps(q["detail"])))
        self.pub_banner.publish(frame["banner"]["vec"])
        self.pub_banner_detail.publish(String(data=json.dumps(frame["banner"]["detail"])))
        self.pub_red.publish(String(data=json.dumps(frame["red"])))
        pay = dict(frame["payload"], stamp=self.t)
        self.pub_payload.publish(String(data=json.dumps(pay)))

    # ---- the loop ---------------------------------------------------------------
    def run(self):
        steps = 0
        t_wall0, t_sim0 = time.monotonic(), 0.0
        while rclpy.ok():
            with self.lock:
                self._step_errors()
                self._step_airframe()
                self._step_payload()
                slew = 2.0 * DT
                self.cam_joint += max(-slew, min(slew, self.cam_cmd - self.cam_joint))
                est = self.est()
                self.t += DT
                steps += 1
                self.pub_clock.publish(Clock(clock=self.stamp()))
                if steps % 3 == 0:
                    self._publish_fcu(est)
                if steps % 10 == 0:
                    self._publish_scan()
                    if self.rng.random() >= self.frame_drop:
                        self.pending_frames.append((self.t + self.latency,
                                                    self._camera_frame()))
                    self._record()
                while self.pending_frames and self.pending_frames[0][0] <= self.t:
                    self._deliver(self.pending_frames.pop(0)[1])
                if steps % 20 == 0:
                    self._publish_slow()
                self._start_and_stop()
                if self.done:
                    break
            # Real-time factor cap, so the stack's own timers keep up.
            ahead = (self.t - t_sim0) / self.args.rtf - (time.monotonic() - t_wall0)
            if ahead > 0:
                time.sleep(ahead)

    def _record(self):
        hx, hy = self.arena.home_world
        self.track.append((round(self.t, 2), self.p[0], self.p[1], self.p[2],
                           int(self.armed), self.mission_state, self.roll, self.pitch,
                           self.mode, self.payload[0] + hx, self.payload[1] + hy,
                           self.payload[2]))

    done = False

    def _start_and_stop(self):
        # Once a second until the tree takes it: every START resets it.
        if not self.started and self.t >= self.args.start_after_s:
            if self.mission_state not in ("", "WAITING", "PREFLIGHT"):
                self.started = True
                self._event(f"mission started ({self.mission_state})")
            elif abs(self.t % 1.0) < DT / 2:
                self.pub_start.publish(Bool(data=True))
        finished = self.result is not None and not self.armed
        if finished and self.p[2] <= 0.05:
            self.done = True
        if self.t >= self.args.max_sim_s:
            self._event("sim time limit reached")
            self.done = True

    def write(self, out):
        out.mkdir(parents=True, exist_ok=True)
        with open(out / "track.csv", "w", encoding="utf-8") as f:
            f.write("t_sim,x,y,z,armed,state,roll_deg,pitch_deg,mode,"
                    "payload_wx,payload_wy,payload_wz\n")
            for r in self.track:
                f.write(",".join(f"{v:.3f}" if isinstance(v, float) else str(v)
                                 for v in r) + "\n")
        template = (ROOT / "src/aerothon_sim/sim_gazebo/worlds/mission2.sdf").read_text()
        _, layout = MW.apply_spec(template, self.arena.spec)
        layout = MW.to_home_frame(layout, self.arena.home_world)
        (out / "layout.json").write_text(json.dumps(layout, sort_keys=True))
        (out / "run.json").write_text(json.dumps({
            "spec": self.arena.spec.get("name"), "seed": self.args.seed,
            "start_target": self.start_letter, "conditions": self.cond,
            "result": self.result, "sim_s": round(self.t, 1),
            "contacts": self.contacts, "fence_breaches": self.fence_breaches,
            "events": self.events}, indent=1))


def organiser_inputs(spec):
    """AEROTHON_DELIVERY_ZONE / AEROTHON_GEOFENCE for publish_delivery_zone.py,
    exactly as launch_level6_sim.sh derives them from the layout."""
    template = (ROOT / "src/aerothon_sim/sim_gazebo/worlds/mission2.sdf").read_text()
    _, layout = MW.apply_spec(template, spec)
    layout = MW.to_home_frame(layout, W.spawn_point(spec))
    zone = ",".join(f"{v:.3f}" for v in layout["delivery_zone_rect"])
    fence = ";".join(f"{x:.3f},{y:.3f}" for x, y in layout["geofence_poly"])
    return zone, fence


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spec", type=Path)
    ap.add_argument("--conditions", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--hfov", type=float, default=0.851919)
    ap.add_argument("--vel-response-s", type=float, default=0.6)
    ap.add_argument("--rtf", type=float, default=2.0)
    ap.add_argument("--start-after-s", type=float, default=8.0)
    ap.add_argument("--max-sim-s", type=float, default=1200.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = HeadlessWorld(args)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        node.run()
    finally:
        node.write(args.out)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
