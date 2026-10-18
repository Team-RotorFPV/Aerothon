#!/usr/bin/env python3
"""GCS aggregator — RUNNABLE.

Bridges the ROS 2 graph to the Tauri control GCS over WebSocket:
  outbound  kind=telemetry (10 Hz snapshot) / kind=event / kind=ack
  inbound   kind=command   -> arm/disarm, set_mode, abort, set target

MAVROS supplies flight data as ROS topics, so this node speaks only ROS.
Camera is a SEPARATE MJPEG stream (web_video_server).
"""
import asyncio
import json
import math
import time
import threading
from numbers import Real

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
# One definition of a valid colour, shared with the detectors.
from perception_banner.colours import validate as validate_colours
# ...and of the arena's dimensions, shared with mission_bt.
from mission_bt import arena_dimensions
from mission_bt.delivery_zone import parse_polygon
import os
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       QoSDurabilityPolicy, QoSHistoryPolicy)
from mavros_msgs.msg import State, EstimatorStatus, WaypointList, Vibration, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import BatteryState, NavSatFix, LaserScan, MagneticField, Image
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool, Float64, UInt32

import websockets

from gcs_aggregator import lidar_calibration
from gcs_aggregator.sensor_metrics import SensorMetric


def _euler_deg(q):
    """Quaternion -> (roll, pitch, yaw) in degrees."""
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), (math.degrees(yaw) + 360) % 360

def _finite_json(value):
    """value with every non-finite float replaced by None, recursively."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(v) for v in value]
    return value


def _finite_number(value, nonnegative=False):
    if not isinstance(value, Real) or not math.isfinite(value):
        return None
    if nonnegative and value < 0:
        return None
    return float(value)

SCHEMA_VERSION = 1
# Several headless simulations run side by side, each with its own GCS;
# the GCS app and the field keep 8765.
WS_HOST, WS_PORT = "0.0.0.0", int(os.environ.get("AEROTHON_GCS_WS_PORT", "8765"))
TELEM_HZ = 10.0

# The mode the aircraft flies itself in. See the note in _dispatch: every
# autonomous command the stack issues is an external setpoint, which
# ArduCopter acts on in GUIDED and ignores in AUTO.
REQUIRED_FLIGHT_MODE = "GUIDED"
# The full mission and each test part it can be cut down to. A part runs the
# same production nodes; it just stops the mission short of the rest.
TEST_PARTS = ("mission", "start_qr", "corridor_out", "search",
              "delivery", "return_corridor")
# Commands that hand control to the autonomy. These need the interlock AND
# the flight mode; everything else, recovery included, does not.
STARTS_FLIGHT = frozenset({"arm", "takeoff", "start_mission", "start_test"})


def max_pool_grid(data, w, h, step):
    """(ow, oh, cells): an occupancy grid max-pooled in step x step blocks.

    Max, so an obstacle cell survives the downsampling; -1 (unknown) only
    where a block holds nothing else. Blocks at the right and bottom edges
    are truncated, not padded with anything that could win. Vectorised: the
    per-cell Python loop it replaced cost ~0.1 s a second on a Pi-sized CPU
    for a 400 x 400 map.
    """
    ow, oh = (w + step - 1) // step, (h + step - 1) // step
    g = np.full((oh * step, ow * step), -1, dtype=np.int16)
    g[:h, :w] = np.asarray(data, dtype=np.int16).reshape(h, w)
    return ow, oh, g.reshape(oh, step, ow, step).max(axis=(1, 3)).ravel().tolist()


def pack_grid(data):
    """Lossless run lengths keep large, mostly-free maps small on the LAN."""
    values = np.asarray(data, dtype=np.int16)
    if not len(values):
        return {'data': []}
    starts = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1]
    counts = np.diff(np.r_[starts, len(values)])
    if 2*len(starts) < len(values):
        return {'encoding': 'rle', 'rle': np.column_stack((values[starts], counts)).ravel().tolist()}
    return {'data': data}


class Aggregator(Node):
    def __init__(self):
        super().__init__("gcs_aggregator")
        self._ws_clients = set()
        self._loop = None
        self.state = self._blank_state()

        self._arm_t = None
        self._detector_stamps = {}
        self._steady_seen = {}
        self._command_receipts = {}
        self._sensor_metrics = {key: SensorMetric() for key in ("camera", "lidar")}
        self.managed = self.declare_parameter('managed', False).value is True
        self.ws_host = self.declare_parameter('ws_host', WS_HOST).value
        self._device_status = None
        self._runtime_seen = None
        q = 10
        qos_sensor = qos_profile_sensor_data
        self.declare_parameter("image_topic", "/image_raw")
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._on_image, qos_sensor, raw=True)
        self.create_subscription(State, "/mavros/state", self._on_state, qos_sensor)
        self.create_subscription(Vibration, "/mavros/vibration/raw/vibration", self._on_vibration, qos_sensor)
        self.create_subscription(MagneticField, "/mavros/imu/mag", self._on_magnetic, qos_sensor)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended_state, qos_sensor)
        self.create_subscription(BatteryState, "/mavros/battery", self._on_batt, qos_sensor)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose, qos_sensor)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._on_vel, qos_sensor)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_sensor)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_gps, qos_sensor)
        self.create_subscription(String, "/percep/qr/decoded", self._on_qr, q)
        self.create_subscription(Bool, "/percep/qr/matched", self._on_match, q)
        self.create_subscription(String, "/mission/target",
                                 lambda m: self.state["percep"].update(mission_target=m.data), q)
        self.create_subscription(Vector3, "/percep/banner", self._on_banner, q)
        self.create_subscription(String, "/percep/qr/detail",
                                 self._on_qr_detail, q)
        self.create_subscription(String, "/percep/banner/detail",
                                 self._on_banner_detail, q)
        self.create_subscription(Bool, "/percep/redzone", self._on_red, q)
        self.create_subscription(String, "/percep/redzone/detail",
                                 self._on_red_detail, q)
        self.create_subscription(Vector3, "/avoidance/status", self._on_avoid, q)
        self.create_subscription(Bool, "/mission_ready", self._on_ready, q)
        # The Bool alone cannot tell an operator WHY arming is blocked. The
        # interlock evaluates eleven items with a measured value and a reason
        # each (gcs_aggregator.readiness); the panel gets all of it.
        self.create_subscription(String, "/mission_ready/detail",
                                 self._on_ready_detail, q)
        self.create_subscription(String, "/mission/state", self._on_mission_state, q)
        self.create_subscription(String, "/mission/square_on",
                                 self._on_square_on, q)
        # The terminal outcome, including the delivery accuracy as a NUMBER.
        self.create_subscription(String, "/mission/result", self._on_mission_result, q)
        # Real slam_toolbox occupancy grid (throttled + downsampled to the GCS).
        self._last_map = 0.0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self.create_subscription(EstimatorStatus, "/mavros/estimator_status",
                                 self._on_estimator, qos_sensor)
        self.create_subscription(UInt32, "/mavros/global_position/raw/satellites",
                                 self._on_sats, qos_sensor)
        self.create_subscription(WaypointList, "/mavros/geofence/fences",
                                 self._on_fences, q)
        self.create_subscription(String, "/winch/status", self._on_winch, q)
        self._seen = {}

        self.pub_abort = self.create_publisher(Bool, "/mission/abort", q)
        self.pub_runtime = self.create_publisher(String, '/runtime/request', q)
        self.pub_camera_pose = self.create_publisher(String, '/camera/set_pose', q)
        self.create_subscription(String, '/runtime/status', self._on_execution, q)
        self.create_subscription(String, '/runtime/receipt', self._on_receipt, q)
        self.create_subscription(String, '/runtime/health', self._on_runtime_health, q)
        self.create_subscription(String, '/devices/status', self._on_devices, q)
        self.pub_start = self.create_publisher(Bool, "/mission/start", q)
        self.pub_target = self.create_publisher(String, "/mission/target", q)
        # The mission reads the operator's target here; /mission/target is
        # its own output to perception and is cleared at every start.
        self.pub_target_override = self.create_publisher(String, "/mission/target_override", q)
        self.pub_winch = self.create_publisher(String, "/winch/cmd", q)
        self.pub_gimbal_pitch = self.create_publisher(Float64, "/gimbal/cmd_pitch", q)
        zone_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_geofence = self.create_publisher(String, "/mission/geofence", zone_qos)
        self.pub_delivery_zone = self.create_publisher(
            String, "/mission/delivery_zone", zone_qos)
        # The fence in force, whoever supplied it, for the map's polygon.
        self.create_subscription(String, "/mission/geofence",
                                 self._on_geofence_supplied, zone_qos)
        # Latched like the boundary: a detector that starts later still gets
        # the shades the operator typed before it came up.
        self.pub_banner_colours = self.create_publisher(
            String, "/mission/banner_colours", zone_qos)
        self.pub_test_part = self.create_publisher(
            String, "/mission/test_part", zone_qos)
        # Every dimension the algorithms use, entered here instead of fixed at
        # launch; mission_bt rebuilds with them on the ground and says so.
        self.pub_arena_dimensions = self.create_publisher(
            String, "/mission/arena_dimensions", zone_qos)
        self.create_subscription(String, "/mission/arena_dimensions/applied",
                                 self._on_dimensions_applied, zone_qos)
        # LIDAR CALIBRATION. The driver's raw scan (only on the aircraft:
        # the simulator publishes /scan in the body frame and has no
        # /scan_raw), the mount the GCS measures from it, and what
        # scan_mount reports it is applying.
        self._scan_raw = None          # (LaserScan, monotonic time)
        self._scan_body = None
        self._lidar_session = lidar_calibration.CalibrationSession()
        self.create_subscription(String, '/slam/status', self._on_slam_status, 10)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan_raw, qos_sensor)
        self.pub_lidar_mount = self.create_publisher(
            String, "/mission/lidar_mount", zone_qos)
        self.create_subscription(String, "/mission/lidar_mount/applied",
                                 self._on_lidar_mount_applied, zone_qos)
        self.declare_parameter("dimension_presets_file", os.path.expanduser(
            "~/.aerothon/dimension_presets.json"))
        self._presets_path = self.get_parameter("dimension_presets_file").value
        self.state["dimension_presets"] = self._load_presets()
        self.state["dimension_fields"] = [
            dict(group=g, key=k, label=label, default=default, min=lo, max=hi,
                 used_by=used) for g, k, label, default, lo, hi, used
            in arena_dimensions.FIELDS]

        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.create_timer(1.0 / TELEM_HZ, self._publish_snapshot,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"aggregator up; ws://{WS_HOST}:{WS_PORT}")

    @staticmethod
    def _blank_state():
        return {
            "mission": {"selected": None, "part": "mission", "state": "idle", "armed": False,
                        "mode": "", "elapsed": 0.0,
                        # 15 rulebook marks. Measured at release from the QR
                        # offset and the altitude; None until it happens.
                        "delivery_offset_m": None, "landing_precision": None,
                        "result": None, "result_reason": None},
            "flight": {"x": 0, "y": 0, "alt": 0, "gs": 0,
                       "roll_deg": 0, "pitch_deg": 0, "yaw_deg": 0},
            # sats is None until a real count arrives; the GCS must render
            # "UNKNOWN", not a confident zero.
            "gps": {"lat": 0, "lon": 0, "sats": None, "fix": "NO"},
            "power": {"volt": None, "pct": None, "current_a": None},
            "sensor_diagnostics": {"vibration_mps2": None, "clipping": None,
                "magnetic_ut": None, "magnetic_strength_ut": None, "magnetic_units_basis": None,
                "magnetic_frame": None, "magnetic_capture_stamp": None},
            "nav": {"front_m": 0, "centering_err": 0, "cmd_vx": 0},
            # redzone_status starts UNKNOWN rather than CLEAR: "no detection
            # yet" and "looked and saw clear ground" are not the same claim.
            "percep": {"start_qr": "", "target_match": False, "banner": False,
                       # What the mission is delivering to, and how the pad
                       # is being identified (pattern, payload or both).
                       "mission_target": "", "qr_reference": False,
                       "qr_match_basis": [], "qr_appearance_score": None,
                       "redzone_visible": False, "redzone_status": "UNKNOWN",
                       "redzone_reason": "", "redzone_exclusions": [],
                       "redzone_area_m2": 0.0,
                       # SQUARE ON, measured with the lidar. "Aligned" says
                       # only where the nose points; this says how far off
                       # perpendicular to the banner's face the aircraft is
                       # and how far off it is standing. None until the stage
                       # has actually measured -- a zero here would read as
                       # perfectly square.
                       "square_angle_deg": None, "square_standoff_m": None,
                       "square_ok": None, "square_reason": ""},
            # ekf and geofence were hardcoded True / "INSIDE" and never
            # updated, so the panel asserted the two things an operator most
            # needs to trust. They now start UNKNOWN and only ever show what
            # has actually been measured.
            "safety": {"ready": False, "fcu_connected": False,
                       "ekf": "UNKNOWN", "ekf_flags": {},
                       "geofence": "UNKNOWN", "fence_count": None,
                       "battery_ok": True,
                       "stale": [],
                       "ready_items": [], "ready_reasons": [],
                       "ready_waived": []},
            "checklist": {k: False for k in
                          ("takeoff", "start_qr", "banner", "corridor",
                           "target_id", "drop", "return", "land")},
            "scan": {"yaw_deg": 0, "ranges": []},
            # The GCS's lidar calibration mode: live nearest return in the
            # raw and the corrected scan, the two captures, the solved mount
            # and the mount scan_mount says is in force.
            "lidar_cal": {"raw_fresh": False, "raw_nearest": None,
                          "body_nearest": None,
                          "captures": {"ahead": None, "port": None},
                          "result": None, "applied": None},
            # Everything scanned, decoded, identified or refused, in the order
            # it first happened. The operator asked for "a list of everything
            # that has been detected/decoded", with matches tagged; rejections
            # are kept too, because a marker the stack REFUSED is the thing an
            # operator most needs to see and the thing a log hides best.
            "scans": [],
            "gimbal": {"pitch_deg": 0.0},
            # None until an operator supplies them; the mission flies the
            # launch values meanwhile, and the panel says so.
            "arena_dimensions": None,
        }

    STALE_AFTER_S = 5.0

    # Long enough to hold a whole run's distinct observations, short enough
    # that the panel stays readable and the websocket payload stays small.
    MAX_SCANS = 120

    # ------------------------------------------------------------------ #
    # Scan ledger
    # ------------------------------------------------------------------ #
    def _record_scan(self, kind, payload="", matched=False, reason="",
                     status="", via="", read=False):
        """Add or update one row. De-duplicated HERE, not in the browser.

        The identity of a row is what it says, not when it was said: the same
        marker read on two hundred consecutive frames is one observation seen
        two hundred times, and that count is itself evidence -- it separates a
        solid read from a single-frame blip.
        """
        if not payload and not reason:
            return                        # a frame with nothing in it
        key = "|".join((kind, payload, "" if payload else reason))
        rows = self.state["scans"]
        for e in rows:
            if e["key"] == key:
                e["count"] += 1
                e["t_last"] = round(self._uptime(), 1)
                if via:
                    e["via"] = via
                if read and not e["read"]:
                    e["read"] = True
                # A match is never withdrawn by a later frame: losing the
                # marker for one frame does not un-match the mission.
                if matched and not e["matched"]:
                    e["matched"] = True
                    e["status"] = "MATCHED"
                return
        self._scan_seq = getattr(self, "_scan_seq", 0) + 1
        rows.append({
            "key": key,
            "seq": self._scan_seq,
            "kind": kind,
            "payload": payload,
            "matched": bool(matched),
            "status": status or ("MATCHED" if matched else
                                 ("DECODED" if payload else "REJECTED")),
            "reason": reason,
            "via": via,
            # Did the lettering actually READ, or did this pass on structure
            # alone? Different claims, and the panel must not blur them.
            "read": bool(read),
            "stage": self.state["mission"].get("state", ""),
            "t": round(self._uptime(), 1),
            "t_last": round(self._uptime(), 1),
            "count": 1,
        })
        if len(rows) > self.MAX_SCANS:
            # Drop the oldest UNMATCHED row. The matched one is the single row
            # of the whole run that the score depends on; a long tail of pad
            # reads must not push it out of the list.
            for i, e in enumerate(rows):
                if not e["matched"]:
                    rows.pop(i)
                    break
            else:
                rows.pop(0)

    def _uptime(self):
        return float(self.state["mission"].get("elapsed", 0.0) or 0.0)

    def _detector_rate(self, key, d):
        stamp = d.get('capture_stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):
            return
        samples = self._detector_stamps.setdefault(key, [])
        if samples and stamp <= samples[-1]:
            if stamp < samples[-1]:
                samples.clear()
            else:
                return
        samples.append(stamp)
        samples[:] = samples[-50:]
        hz = (len(samples)-1)/(samples[-1]-samples[0]) if len(samples)>1 else None
        self.state.setdefault('detector_rates',{})[key] = {'hz':hz,'capture_stamp':stamp,
            'processed_stamp':d.get('processed_stamp'),'received':time.time()}

    def _on_geofence_supplied(self, m):
        vertices, _why = parse_polygon(m.data, what="geofence")
        if vertices is not None:
            self.state["geofence_vertices"] = [[lat, lon] for lat, lon in vertices]

    def _on_qr_detail(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict):
            return
        self._detector_rate("qr", d)
        percep = self.state["percep"]
        percep["qr_reference"] = d.get("reference_captured") is True
        percep["qr_match_basis"] = [str(b) for b in (d.get("match_basis") or [])][:2]
        score = d.get("appearance_score")
        percep["qr_appearance_score"] = (round(float(score), 3)
                                         if isinstance(score, (int, float)) else None)
        accepted = str(d.get("accepted") or "")
        if accepted:
            self._record_scan("qr", payload=accepted,
                              matched=bool(d.get("matched")))
        for r in (d.get("rejected") or []):
            reason = r.get("reason") if isinstance(r, dict) else str(r)
            if reason:
                self._record_scan("qr", reason=str(reason))

    def _on_square_on(self, m):
        self._mark("square")
        """How square the aircraft is to the banner, while it converges.

        The operator watching a live flight could see the aircraft moving and
        not what it believed about its angle to the gate, which is the one
        number the stage decides on.
        """
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        pc = self.state["percep"]
        pc["square_ok"] = d.get("ok")
        pc["square_angle_deg"] = d.get("angle_deg")
        pc["square_standoff_m"] = d.get("standoff_m")
        pc["square_reason"] = str(d.get("reason") or "")

    def _on_banner_detail(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict):
            return
        self._detector_rate("banner", d)
        if d.get("identified"):
            # The glyph classifier writes '?' for anything it cannot name, and
            # the identity gate passes on STRUCTURE -- so a legitimately
            # identified banner routinely carries a text field like
            # "??N?E??????N?". Printing that as the payload presents a failed
            # read as data, and every distinct garbage string became its own
            # row, burying everything else in the list.
            confirmed = bool(d.get("text_confirmed"))
            text = str(d.get("text") or "")
            payload = text if (confirmed and "?" not in text) else "BANNER"
            self._record_scan("banner", payload=payload, status="IDENTIFIED",
                              read=confirmed and "?" not in text,
                              via=str(d.get("lettering_path") or ""))
        elif d.get("reason"):
            self._record_scan("banner", reason=str(d.get("reason")))

    def _staleness(self):
        """Which measured safety fields have gone quiet, or never arrived."""
        now = time.time()
        stale = []
        for key in ("ekf", "sats", "fence"):
            t = self._seen.get(key)
            if t is None:
                stale.append(f"{key}:never")
            elif (now - t) > self.STALE_AFTER_S:
                stale.append(f"{key}:{now - t:.0f}s")
        return stale

    # ---- subscription callbacks ---- #
    def _mark(self, key):
        self._seen[key] = time.time()
        self._steady_seen[key] = time.monotonic()

    def _on_estimator(self, m):
        """Real EKF health from MAVROS, replacing a hardcoded True.

        ArduPilot reports per-axis estimator flags; treat the estimator as
        healthy only when the states guidance depends on are all good.
        """
        flags = {
            "attitude": bool(m.attitude_status_flag),
            "velocity_horiz": bool(m.velocity_horiz_status_flag),
            "pos_horiz_rel": bool(m.pos_horiz_rel_status_flag),
            "pos_horiz_abs": bool(m.pos_horiz_abs_status_flag),
            "pos_vert_abs": bool(m.pos_vert_abs_status_flag),
            "const_pos_mode": bool(m.const_pos_mode_status_flag),
        }
        required = ("attitude", "velocity_horiz", "pos_horiz_abs", "pos_vert_abs")
        healthy = all(flags[k] for k in required) and not flags["const_pos_mode"]
        self.state["safety"]["ekf"] = "OK" if healthy else "DEGRADED"
        self.state["safety"]["ekf_flags"] = flags
        self._mark("ekf")

    def _on_sats(self, m):
        self.state["gps"]["sats"] = int(m.data)
        self._mark("sats")

    def _on_fences(self, m):
        """What MAVROS can actually tell us about the fence.

        It publishes the loaded fence LIST. It does not publish breach state --
        ArduPilot's FENCE_STATUS is not exposed -- so claiming "INSIDE" was
        never measurable. Report what is knowable: whether a fence is loaded.
        """
        n = len(m.waypoints)
        self.state["safety"]["fence_count"] = n
        self.state["safety"]["geofence"] = "LOADED" if n else "NONE"
        self._mark("fence")

    def _on_winch(self, m):
        try:
            self.state["winch"] = json.loads(m.data)
        except json.JSONDecodeError:
            pass
        self._mark("winch")
    def _on_state(self, m):
        self._mark("state")
        self.state["safety"]["fcu_connected"] = bool(m.connected)
        if m.armed and self._arm_t is None:
            self._arm_t = time.monotonic()
        elif not m.armed:
            self._arm_t = None
        self.state["mission"]["armed"] = m.armed
        self.state["mission"]["mode"] = m.mode
        self.state["mission"]["elapsed"] = round(time.monotonic() - self._arm_t, 1) if self._arm_t else 0.0

    def _on_batt(self, m):
        self._mark("power")
        voltage = _finite_number(m.voltage, True)
        pct = _finite_number(m.percentage, True)
        self.state["power"]["volt"] = None if voltage is None else round(voltage, 2)
        self.state["power"]["pct"] = None if pct is None else round(pct * 100 if pct <= 1 else pct)
        self.state["power"]["current_a"] = _finite_number(m.current)
        self.state["safety"]["battery_ok"] = not (0.0 < m.voltage < 10.5 or 0.0 < m.percentage < 0.15)

    def _on_vibration(self, m):
        self._mark("vibration")
        diag = self.state["sensor_diagnostics"]
        diag["vibration_mps2"] = [_finite_number(v, True) for v in
                                  (m.vibration.x, m.vibration.y, m.vibration.z)]
        diag["clipping"] = [_finite_number(v, True) for v in m.clipping]

    def _on_magnetic(self, m):
        self._mark("magnetic")
        diag = self.state["sensor_diagnostics"]
        field = [_finite_number(v) for v in (m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z)]
        # MAVROS fills this from ArduPilot's RAW_IMU/SCALED_IMU milligauss
        # multiplied by its MILLIT_TO_TESLA constant (1000), not by 1e-7, so
        # Earth's ~0.5 gauss arrives as ~5e5 "tesla". No FC magnetometer reads
        # 0.01 T; above that the value is that encoding: uT = mG * 0.1.
        raw = [v for v in field if v is not None]
        scaled = bool(raw) and max(abs(v) for v in raw) > 0.01
        factor = 1e-4 if scaled else 1e6
        field = [None if v is None else _finite_number(v * factor) for v in field]
        diag["magnetic_units_basis"] = "mavros_milligauss_x1000" if scaled else "tesla"
        diag["magnetic_ut"] = field
        diag["magnetic_strength_ut"] = _finite_number(math.hypot(*field)) if all(v is not None for v in field) else None
        diag["magnetic_frame"] = str(m.header.frame_id)
        diag["magnetic_capture_stamp"] = m.header.stamp.sec + m.header.stamp.nanosec / 1e9

    def _on_extended_state(self, m):
        self._mark("extended_state")
        self.state["safety"]["on_ground"] = m.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def _on_pose(self, m):
        self._mark("pose")
        p = m.pose.position
        roll, pitch, yaw = _euler_deg(m.pose.orientation)
        fl = self.state["flight"]
        fl["x"], fl["y"], fl["alt"] = round(p.x, 2), round(p.y, 2), round(p.z, 2)
        fl["roll_deg"], fl["pitch_deg"], fl["yaw_deg"] = round(roll, 1), round(pitch, 1), round(yaw, 1)
        self.state["scan"]["yaw_deg"] = round(yaw, 1)

    def _on_vel(self, m):
        self._mark("velocity")
        self.state["flight"]["gs"] = round(math.hypot(m.twist.linear.x, m.twist.linear.y), 2)

    def _on_scan_raw(self, m):
        self._scan_raw = (m, time.monotonic())
        self._lidar_session.observe(m)

    def _on_slam_status(self, m):
        try:
            data = json.loads(m.data)
            if isinstance(data, dict):
                self.state['slam_status'] = data
                self._mark('slam_status')
        except (ValueError, TypeError):
            pass

    def _on_lidar_mount_applied(self, m):
        try:
            self.state["lidar_cal"]["applied"] = json.loads(m.data)
        except ValueError:
            pass

    RAW_STALE_S = 1.0

    def _fresh_raw(self):
        if self._scan_raw is None:
            return None
        scan, t = self._scan_raw
        return scan if time.monotonic() - t <= self.RAW_STALE_S else None

    def _lidar_cal_refresh(self):
        """Live readouts for the calibration panel, from the latest scans."""
        cal = self.state["lidar_cal"]
        raw = self._fresh_raw()
        cal["raw_fresh"] = raw is not None
        near = lidar_calibration.nearest_bearing(raw) if raw is not None else None
        cal["raw_nearest"] = None if near is None else [round(near[0], 1), round(near[1], 2)]
        cal['geometry'] = self._lidar_session.geometry
        cal['background_ready'] = self._lidar_session.baseline is not None
        cal['samples'] = len(self._lidar_session.history)
        cal['capture_details'] = self._lidar_session.captures
        cal['result'] = self._lidar_session.result()
        if raw is not None:
            cal['scan'] = dict(ranges=[round(float(r), 3) if math.isfinite(r) and raw.range_min < r < raw.range_max else None for r in raw.ranges],
                               angle_min=float(raw.angle_min), angle_step=float(raw.angle_increment),
                               range_min=float(raw.range_min), range_max=float(raw.range_max))
        body = self._scan_body
        near = lidar_calibration.nearest_bearing(body) if body is not None else None
        cal["body_nearest"] = None if near is None else [round(near[0], 1), round(near[1], 2)]

    def _on_scan(self, m):
        self._sensor_metrics["lidar"].observe(m.header)
        self._scan_body = m
        self._mark("lidar")
        n = len(m.ranges)
        if not n:
            return
        # The C1's 720 beams fit comfortably in telemetry. Taking every tenth
        # beam erased thin obstacles and made straight walls look disconnected.
        step = max(1, math.ceil(n / 4096))
        out = []
        for i in range(0, n, step):
            r = m.ranges[i]
            out.append(round(r, 2) if (math.isfinite(r) and m.range_min < r < m.range_max) else None)
        self.state["scan"]["ranges"] = out
        # Body frame (scan_mount): bearing of out[i] = angle_min + i*angle_step,
        # CCW from the nose. The SLAM tab draws it before any map exists.
        self.state["scan"]["angle_min"] = float(m.angle_min)
        self.state["scan"]["angle_step"] = float(m.angle_increment) * step
        self.state["scan"]["range_min"] = float(m.range_min)
        self.state["scan"]["range_max"] = float(m.range_max)
        self.state["scan"]["frame"] = m.header.frame_id
        self.state["scan"]["stamp"] = m.header.stamp.sec + m.header.stamp.nanosec / 1e9

    def _on_image(self, m):
        # Subscribed raw: only the stamp is needed, not the pixels.
        if isinstance(m, (bytes, bytearray, memoryview)):
            self._sensor_metrics["camera"].observe_raw(m)
        else:
            self._sensor_metrics["camera"].observe(m.header)

    def _on_gps(self, m):
        self._mark("gps")
        self.state["gps"]["lat"] = m.latitude
        self.state["gps"]["lon"] = m.longitude
        self.state["gps"]["fix"] = "3D" if m.status.status >= 0 else "NO"

    def _on_qr(self, m):
        self.state["percep"]["start_qr"] = m.data
        if m.data:
            self.state["checklist"]["start_qr"] = True

    def _on_match(self, m):
        self.state["percep"]["target_match"] = m.data
        if m.data:
            self.state["checklist"]["target_id"] = True

    def _on_banner(self, m):
        det = m.z > 0.5
        self.state["percep"]["banner"] = det
        if det:
            self.state["checklist"]["banner"] = True

    def _on_red(self, m):
        self.state["percep"]["redzone_visible"] = m.data

    def _on_red_detail(self, m):
        """NOT_VISIBLE / CLEAR / RED, plus the georeferenced exclusions.

        `redzone_visible: false` collapsed two very different situations: the
        camera is looking at clear ground, and the camera cannot see the
        ground at all. Only one of those is reassuring.
        """
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict):
            return
        self._detector_rate("red", d)
        p = self.state["percep"]
        p["redzone_status"] = d.get("status", "UNKNOWN")
        p["redzone_reason"] = d.get("reason", "")
        p["redzone_exclusions"] = d.get("exclusions", [])
        p["redzone_area_m2"] = round(float(d.get("confirmed_area_m2", 0.0)), 1)

    def _on_avoid(self, m):
        self._mark("avoidance")
        self.state["nav"] = {"front_m": round(m.x, 2),
                             "centering_err": round(m.y, 2), "cmd_vx": round(m.z, 2)}

    def _on_ready(self, m):
        self._mark("readiness")
        self.state["safety"]["ready"] = m.data

    def _on_ready_detail(self, m):
        """The eleven interlock items, each with its measured value + reason.

        An operator looking at a blocked ARM button needs to know which check
        is holding and what it measured, not just that something is.
        """
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict):
            return
        self._mark("readiness")
        s = self.state["safety"]
        s["ready"] = bool(d.get("ready", False))
        s["ready_items"] = d.get("items", [])
        s["ready_reasons"] = d.get("reasons", [])
        s["ready_waived"] = d.get("waived", [])

    def _on_mission_result(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        mis = self.state["mission"]
        mis["result"] = d.get("state")
        mis["result_reason"] = d.get("reason")
        mis["delivery_offset_m"] = d.get("delivery_offset_m")
        mis["landing_precision"] = d.get("landing_precision")

    def _on_mission_state(self, m):
        self._mark("mission")
        self.state["mission"]["state"] = m.data
        if m.data == "TAKEOFF":
            self.state["checklist"]["takeoff"] = True
        elif m.data == "CORRIDOR_NAV":
            self.state["checklist"]["corridor"] = True
        elif m.data in {"RETURN", "RETURN_CORRIDOR"}:
            self.state["checklist"]["return"] = True
        elif m.data == "LAND":
            self.state["checklist"]["land"] = True

    def _on_map(self, m):
        """Forward the REAL slam_toolbox occupancy grid (throttled + downsampled)."""
        if not self._ws_clients:
            return                  # nobody to forward it to
        now = time.monotonic()
        if now - self._last_map < 1.0:
            return
        self._last_map = now
        w, h, data = m.info.width, m.info.height, m.data
        if not w or not h:
            return
        step = max(1, math.ceil(max(w, h) / 1024))
        ow, oh, out = max_pool_grid(data, w, h, step)
        grid = {"res": round(m.info.resolution * step, 3), "w": ow, "h": oh,
                "ox": round(m.info.origin.position.x, 3),
                "oy": round(m.info.origin.position.y, 3),
                "yaw": math.radians(_euler_deg(m.info.origin.orientation)[2]),
                "frame": m.header.frame_id,
                "stamp": m.header.stamp.sec + m.header.stamp.nanosec / 1e9, **pack_grid(out)}
        self._broadcast(self._env("map", grid))

    # ---- outbound ---- #
    def _env(self, kind, data):
        # Browsers' JSON.parse rejects Infinity/NaN, so one non-finite value
        # (front_m is inf with nothing ahead, normal outdoors) dropped every
        # telemetry frame and the GCS read "stale". Non-finite -> null.
        return json.dumps({"v": SCHEMA_VERSION, "kind": kind, "t": time.time(),
                           "data": _finite_json(data)}, allow_nan=False, separators=(',', ':'))

    # An exception in a callback ends rclpy.spin; the launch does not respawn
    # the aggregator, and the web host then refuses even Abort/Land/RTL as
    # stale. Runtime messages are parsed defensively and a bad one dropped.
    def _runtime_json(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warning('dropped malformed runtime message')
            return None
        return data if isinstance(data, dict) else None

    def _on_execution(self, msg):
        data = self._runtime_json(msg)
        if data is None:
            return
        self.state['execution'] = data
        self._broadcast(self._env('execution', data))

    def _on_receipt(self, msg):
        data = self._runtime_json(msg)
        if data is not None:
            self._broadcast(self._env('receipt', data))

    def _on_runtime_health(self, msg):
        data = self._runtime_json(msg)
        if data is None:
            return
        self._runtime_seen = time.monotonic()
        self.state['runtime_health'] = data

    def _on_devices(self, msg):
        data = self._runtime_json(msg)
        if data is not None:
            self._device_status = (data, time.monotonic())

    def _publish_snapshot(self):
        if not self._ws_clients:
            return                  # no GCS connected: nothing to serialise for
        # Refresh staleness on every snapshot so a field that stops updating
        # is visibly stale rather than frozen at its last confident value.
        self.state["safety"]["stale"] = self._staleness()
        self._lidar_cal_refresh()
        now = time.monotonic()
        self.state["sample_age_s"] = {k: round(max(0.0, now - stamp), 3) for k, stamp in self._steady_seen.items()}
        nanos = _finite_number(self.get_clock().now().nanoseconds)
        clock = None if nanos is None else nanos / 1e9
        self.state["devices"] = {key: metric.snapshot(clock, now) for key, metric in self._sensor_metrics.items()}
        if self._device_status:
            devices, received = self._device_status
            self.state['devices'] = {k: dict(v, ok=v.get('ok') is True and now-received < 1)
                                     for k, v in devices.items() if isinstance(v, dict)}
        self.state.setdefault('runtime_health', {})['fresh'] = self._runtime_seen is not None and now-self._runtime_seen < 1
        self.state["sample_age_s"]["devices"] = 0.0
        self._broadcast(self._env("telemetry", self.state))

    def _broadcast(self, payload):
        if self._loop and self._ws_clients:
            asyncio.run_coroutine_threadsafe(self._bcast(payload), self._loop)

    async def _bcast(self, msg):
        dead = set()
        # A copy: a client can disconnect during an await below.
        for ws in list(self._ws_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ---- inbound commands ---- #
    async def _handler(self, ws):
        self._ws_clients.add(ws)
        try:
            async for raw in ws:
                await self._on_command(ws, raw)
        finally:
            self._ws_clients.discard(ws)

    async def _on_command(self, ws, raw):
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(msg, dict) or msg.get("kind") != "command":
            return
        d = msg.get("data", {})
        if not isinstance(d, dict):
            return
        cmd, cid = d.get("cmd"), d.get("cmd_id")
        args = d.get("args", {})
        signature = json.dumps([cmd, args], sort_keys=True)
        previous = self._command_receipts.get(cid) if isinstance(cid, str) else None
        if previous:
            result, reason = previous[1:] if previous[0] == signature else ("rejected", "command ID reused for a different request")
        elif not isinstance(cid, str) or not cid or len(cid) > 128 or not isinstance(args, dict):
            result, reason = "rejected", "command needs a unique ID and an arguments object"
        else:
            result, reason = self._dispatch(cmd, args, cid)
            # Keep receipts for the lifetime of this runtime. Fail closed when
            # full rather than evicting an ID that could repeat a release.
            self._command_receipts[cid] = (signature, result, reason)
        await ws.send(self._env("ack", {"cmd_id": cid, "cmd": cmd,
                                        "result": result, "reason": reason}))

    def _dispatch(self, cmd, args, cid=None):
        try:
            if cmd in {'runtime_start', 'runtime_stop'}:
                if not self.managed or not cid:
                    return 'rejected', 'managed production supervisor required'
                if cmd == 'runtime_start' and (self._runtime_seen is None or time.monotonic()-self._runtime_seen > 1):
                    return 'rejected', 'execution supervisor is stale'
                self.pub_runtime.publish(String(data=json.dumps({'command_id': cid,
                    'cmd': 'start' if cmd == 'runtime_start' else 'stop', 'args': args})))
                return 'accepted', 'submitted to production supervisor'
            if self.managed and cmd in {'start_mission', 'start_test', 'winch'}:
                return 'rejected', 'select a registered execution through the Pi supervisor'
            active = self.state.get('execution') or {}
            if self.managed and active.get('state') in {'Submitted', 'Accepted', 'Running'} and cmd in {'arm', 'takeoff', 'set_mode', 'gimbal_pitch'} and args.get('mode') not in {'RTL', 'LAND'}:
                return 'rejected', 'active execution owns the flight controls'
            if len(self._command_receipts) >= 10000 and cmd not in {"abort", "land", "disarm", "set_mode"}:
                return "rejected", "command receipt capacity reached; recover before restarting the runtime"
            if cmd in {"start_mission", "start_test"} and str(self.state["mission"]["state"]).upper() not in {"IDLE", "WAITING", "COMPLETED", "FAILED", "RESET"}:
                return "rejected", "a production execution already owns movement"
            # Flight-changing commands must never bypass the readiness
            # interlock. Abort / land / RTL remain available when unhealthy.
            if cmd in STARTS_FLIGHT and not self.state["safety"]["ready"]:
                return "rejected", "mission interlock not ready"
            # Nothing autonomous starts unless the FCU is actually in GUIDED.
            #
            # The whole stack flies by streaming external setpoints
            # (/mavros/setpoint_position/local from mav_commander,
            # /mavros/setpoint_raw/local from the avoidance controller).
            # ArduCopter honours those in GUIDED; in AUTO it runs an uploaded
            # waypoint mission and ignores them, so a mission "started" in any
            # other mode would publish setpoints nothing acts on while the
            # operator believes the aircraft is flying itself.
            #
            # Recovery is deliberately NOT gated: abort, land, RTL, disarm and
            # the component commands stay available in any mode. An interlock
            # that blocks ABORT is worse than no interlock.
            if cmd in STARTS_FLIGHT:
                mode = str(self.state["mission"].get("mode") or "")
                if mode != REQUIRED_FLIGHT_MODE:
                    return "rejected", (
                        f"flight mode is {mode or 'unknown'}; "
                        f"{REQUIRED_FLIGHT_MODE} required to start")
            if cmd == "arm":
                self.cli_arm.call_async(CommandBool.Request(value=True))
            elif cmd == "disarm":
                self.cli_arm.call_async(CommandBool.Request(value=False))
            elif cmd == "set_mode":
                r = SetMode.Request(); r.custom_mode = args.get("mode", "GUIDED")
                self.cli_mode.call_async(r)
            elif cmd == "takeoff":
                alt = float(args.get("alt", 5.0))
                self.cli_takeoff.call_async(CommandTOL.Request(altitude=alt))
            elif cmd == "land":
                if self.cli_land.service_is_ready():
                    self.cli_land.call_async(CommandTOL.Request())
                else:
                    self.cli_mode.call_async(SetMode.Request(custom_mode="LAND"))
            elif cmd == "winch":
                action = args.get("action", "stow")
                self.pub_winch.publish(String(data=action))
            elif cmd == "gimbal_pitch":
                pitch_deg = max(-90.0, min(30.0, float(args.get("degrees", 0.0))))
                if self.managed:
                    self.pub_camera_pose.publish(String(data=str(pitch_deg)))
                else:
                    self.pub_gimbal_pitch.publish(Float64(data=math.radians(pitch_deg)))
                self.state["gimbal"]["pitch_deg"] = round(pitch_deg, 1)
            elif cmd == "abort":
                self.pub_abort.publish(Bool(data=True))
            elif cmd in ("start_mission", "start_test"):
                part = "mission"
                if cmd == "start_test":
                    part = str(args.get("part", ""))
                    if part not in TEST_PARTS:
                        return "rejected", (
                            f"unknown test part {part!r}; "
                            f"expected one of {', '.join(TEST_PARTS)}")
                self.state["mission"]["selected"] = "M2" if part == "mission" else part
                self.state["mission"]["part"] = part
                self.state["mission"]["state"] = "STARTING"
                # A new run gets a clean ledger. Carrying the previous run's
                # observations forward would put two flights' markers in one
                # list with nothing to say which was which -- and the operator
                # reads this panel to find out what THIS flight saw.
                self.state["scans"] = []
                self._scan_seq = 0
                # The part goes out BEFORE the start, so the mission reads
                # which part it is running rather than starting the full
                # mission and being told afterwards.
                self.pub_test_part.publish(String(data=part))
                self.pub_start.publish(Bool(data=True))
            elif cmd == "set_banner_colours":
                # The organisers give each banner's shade as a HEX code. The
                # banners carry no text, so this is the evidence that tells
                # the aircraft which corridor it may use -- it is validated
                # here by the same rule the detectors apply, and refused with
                # a reason rather than silently dropped.
                try:
                    colours = validate_colours(args.get("colours", args))
                except (ValueError, TypeError, AttributeError) as error:
                    return "rejected", str(error)
                if self.state['mission'].get('armed'):
                    return 'rejected', 'banner colours can only change while disarmed'
                missing = sorted({"green", "orange", "red"} - set(colours))
                if missing:
                    return "rejected", (
                        f"no shade supplied for the {', '.join(missing)} banner")
                payload = json.dumps({"colours": colours}, separators=(",", ":"))
                self.pub_banner_colours.publish(String(data=payload))
                self.state["banner_colours"] = colours
            elif cmd in ("lidar_cal_capture", "lidar_cal_reset", "lidar_cal_apply",
                         "lidar_cal_geometry", "lidar_cal_background"):
                return self._lidar_cal_command(cmd, args)
            elif cmd == "set_arena_dimensions":
                # Refused in the air here AND in mission_bt: the leaves take
                # their dimensions when the tree is built, on the ground.
                if self.state["mission"].get("armed"):
                    return "rejected", "dimensions can only change while disarmed"
                try:
                    dims = arena_dimensions.validate(args.get("dimensions", args))
                except (ValueError, TypeError, AttributeError) as error:
                    return "rejected", str(error)
                self.pub_arena_dimensions.publish(String(data=json.dumps(
                    {"dimensions": dims}, separators=(",", ":"))))
                self.state["arena_dimensions"] = {"sent": dims, "applied": None,
                                                  "reason": "awaiting mission_bt"}
            elif cmd == "save_dimension_preset":
                name = str(args.get("name", "")).strip()
                if not name or len(name) > 60:
                    return "rejected", "a preset needs a name of 1-60 characters"
                if name in arena_dimensions.BUILTIN_PRESETS:
                    return "rejected", f"{name!r} is a built-in preset; choose another name"
                try:
                    dims = arena_dimensions.validate(args.get("dimensions", {}))
                except (ValueError, TypeError, AttributeError) as error:
                    return "rejected", str(error)
                presets = self._operator_presets()
                presets[name] = dims
                self._write_presets(presets)
            elif cmd == "delete_dimension_preset":
                name = str(args.get("name", ""))
                if name in arena_dimensions.BUILTIN_PRESETS:
                    return "rejected", "built-in presets cannot be deleted"
                presets = self._operator_presets()
                if name not in presets:
                    return "rejected", f"no preset named {name!r}"
                del presets[name]
                self._write_presets(presets)
            elif cmd == "set_target":
                target = String(data=args.get("target", ""))
                self.pub_target.publish(target)
                self.pub_target_override.publish(target)
            elif cmd == "set_geofence":
                if self.state["mission"]["armed"]:
                    return "rejected", "arena fence can only change while disarmed"
                payload = json.dumps({"vertices": args.get("vertices")})
                vertices, why = parse_polygon(payload, what="geofence")
                if vertices is None:
                    return "rejected", why
                self.pub_geofence.publish(String(data=payload))
                self.state["geofence_vertices"] = [[lat, lon] for lat, lon in vertices]
            elif cmd == "set_delivery_zone":
                vertices = args.get("vertices")
                if not isinstance(vertices, list) or len(vertices) != 4:
                    return "rejected", "delivery-zone boundary needs four vertices"
                clean = []
                for point in vertices:
                    try:
                        lat, lon = float(point["lat"]), float(point["lon"])
                    except (KeyError, TypeError, ValueError):
                        return "rejected", "every boundary vertex needs numeric lat and lon"
                    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                        return "rejected", "boundary vertex is outside WGS84 bounds"
                    clean.append({"lat": lat, "lon": lon})
                if len({(p["lat"], p["lon"]) for p in clean}) != 4:
                    return "rejected", "delivery-zone boundary needs four distinct vertices"
                payload = json.dumps({"vertices": clean}, separators=(",", ":"))
                self.pub_delivery_zone.publish(String(data=payload))
            else:
                return "rejected", "unknown cmd"
            return "accepted", ""
        except Exception as e:  # noqa: BLE001
            return "rejected", str(e)

    # ---- lidar calibration ---------------------------------------------------
    def _lidar_cal_command(self, cmd, args):
        """Capture a box ahead / to port, start again, or apply the mount.

        On the ground only: the mount turns every beam the corridor
        navigator, the square-up and the keep-out read, and changing it under
        an aircraft in the air would turn the walls it is avoiding."""
        if self.state["mission"].get("armed"):
            return "rejected", "the lidar can only be calibrated while disarmed"
        cal = self.state["lidar_cal"]
        if cmd == "lidar_cal_reset":
            self._lidar_session = lidar_calibration.CalibrationSession()
            cal["captures"] = {"ahead": None, "port": None}
            cal["result"] = None
            return "accepted", ""
        try:
            if cmd == 'lidar_cal_geometry':
                self._lidar_session.measure_geometry(args)
                cal['captures'] = {}
                cal['result'] = None
                return 'accepted', ''
            if cmd == 'lidar_cal_background':
                self._lidar_session.capture_background()
                cal['captures'] = {}
                cal['result'] = None
                return 'accepted', ''
            if cmd == "lidar_cal_capture":
                where = str(args.get("where", ""))
                self._lidar_session.capture(where, args.get('distance_m'))
                cal['captures'] = {k: [round(v['bearing_deg'], 3), round(v['range_m'], 4)]
                                   for k, v in self._lidar_session.captures.items()}
                cal['result'] = self._lidar_session.result()
                return "accepted", ""
        except (ValueError, TypeError, KeyError) as exc:
            return 'rejected', str(exc)
        # lidar_cal_apply
        res = self._lidar_session.result()
        if not res:
            return "rejected", "complete measured geometry, background, ahead, far, port and starboard checks first"
        if not res.get("ok"):
            return "rejected", res.get("reason") or "the captures do not agree"
        self.pub_lidar_mount.publish(String(data=json.dumps(res, separators=(",", ":"))))
        return "accepted", ""

    # ---- arena-dimension presets ------------------------------------------
    def _operator_presets(self):
        """Saved presets, each re-validated; one that no longer loads is skipped
        with a warning rather than taking the others down with it."""
        try:
            with open(self._presets_path, encoding="utf-8") as f:
                raw = json.load(f)
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as error:
            self.get_logger().warning(f"dimension presets unreadable: {error}")
            return {}
        out = {}
        for name, dims in (raw.items() if isinstance(raw, dict) else ()):
            try:
                out[str(name)] = arena_dimensions.validate(dims)
            except (ValueError, TypeError, AttributeError) as error:
                self.get_logger().warning(f"preset {name!r} skipped: {error}")
        return out

    def _load_presets(self):
        return {"builtin": {k: arena_dimensions.validate(v) for k, v
                            in arena_dimensions.BUILTIN_PRESETS.items()},
                "saved": self._operator_presets()}

    def _write_presets(self, presets):
        os.makedirs(os.path.dirname(self._presets_path) or ".", exist_ok=True)
        tmp = self._presets_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(presets, f, indent=2, sort_keys=True)
        os.replace(tmp, self._presets_path)   # never a half-written file
        self.state["dimension_presets"] = self._load_presets()

    def _on_dimensions_applied(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        current = self.state.get("arena_dimensions") or {}
        self.state["arena_dimensions"] = {
            "sent": current.get("sent"),
            "applied": d.get("dimensions") if d.get("applied") else current.get("applied"),
            "params": d.get("params"), "ok": bool(d.get("applied")),
            "reason": d.get("reason", "")}

    def start_ws(self):
        def run():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(websockets.serve(self._handler, self.ws_host, WS_PORT))
            self._loop.run_forever()
        threading.Thread(target=run, daemon=True).start()


def main():
    rclpy.init()
    node = Aggregator()
    node.start_ws()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
