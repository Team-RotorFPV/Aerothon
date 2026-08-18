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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from mavros_msgs.msg import State, EstimatorStatus, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import BatteryState, NavSatFix, LaserScan
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool, Float32, Float64, UInt32

import websockets


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

SCHEMA_VERSION = 1
WS_HOST, WS_PORT = "0.0.0.0", 8765
TELEM_HZ = 10.0


class Aggregator(Node):
    def __init__(self):
        super().__init__("gcs_aggregator")
        self._ws_clients = set()
        self._loop = None
        self.state = self._blank_state()

        self._arm_t = None
        q = 10
        qos_sensor = qos_profile_sensor_data
        self.create_subscription(State, "/mavros/state", self._on_state, qos_sensor)
        self.create_subscription(BatteryState, "/mavros/battery", self._on_batt, qos_sensor)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose, qos_sensor)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._on_vel, qos_sensor)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_sensor)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_gps, qos_sensor)
        self.create_subscription(String, "/percep/qr/decoded", self._on_qr, q)
        self.create_subscription(Bool, "/percep/qr/matched", self._on_match, q)
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
        self.pub_start = self.create_publisher(Bool, "/mission/start", q)
        self.pub_target = self.create_publisher(String, "/mission/target", q)
        self.pub_winch = self.create_publisher(String, "/winch/cmd", q)
        self.pub_gimbal_pitch = self.create_publisher(Float64, "/gimbal/cmd_pitch", q)

        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.create_timer(1.0 / TELEM_HZ, self._publish_snapshot)
        self.get_logger().info(f"aggregator up; ws://{WS_HOST}:{WS_PORT}")

    @staticmethod
    def _blank_state():
        return {
            "mission": {"selected": None, "state": "idle", "armed": False,
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
            "power": {"volt": 0, "pct": 0},
            "nav": {"front_m": 0, "centering_err": 0, "cmd_vx": 0},
            # redzone_status starts UNKNOWN rather than CLEAR: "no detection
            # yet" and "looked and saw clear ground" are not the same claim.
            "percep": {"start_qr": "", "target_match": False, "banner": False,
                       "redzone_visible": False, "redzone_status": "UNKNOWN",
                       "redzone_reason": "", "redzone_exclusions": [],
                       "redzone_area_m2": 0.0},
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
            # Everything scanned, decoded, identified or refused, in the order
            # it first happened. The operator asked for "a list of everything
            # that has been detected/decoded", with matches tagged; rejections
            # are kept too, because a marker the stack REFUSED is the thing an
            # operator most needs to see and the thing a log hides best.
            "scans": [],
            "gimbal": {"pitch_deg": 0.0},
        }

    STALE_AFTER_S = 5.0

    # Long enough to hold a whole run's distinct observations, short enough
    # that the panel stays readable and the websocket payload stays small.
    MAX_SCANS = 120

    # ------------------------------------------------------------------ #
    # Scan ledger
    # ------------------------------------------------------------------ #
    def _record_scan(self, kind, payload="", matched=False, reason="",
                     status="", via=""):
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

    def _on_qr_detail(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        accepted = str(d.get("accepted") or "")
        if accepted:
            self._record_scan("qr", payload=accepted,
                              matched=bool(d.get("matched")))
        for r in (d.get("rejected") or []):
            reason = r.get("reason") if isinstance(r, dict) else str(r)
            if reason:
                self._record_scan("qr", reason=str(reason))

    def _on_banner_detail(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if d.get("identified"):
            self._record_scan("banner", payload=str(d.get("text") or "BANNER"),
                              status="IDENTIFIED",
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
        self.state["safety"]["fcu_connected"] = bool(m.connected)
        if m.armed and self._arm_t is None:
            self._arm_t = time.time()
        elif not m.armed:
            self._arm_t = None
        self.state["mission"]["armed"] = m.armed
        self.state["mission"]["mode"] = m.mode
        self.state["mission"]["elapsed"] = round(time.time() - self._arm_t, 1) if self._arm_t else 0.0

    def _on_batt(self, m):
        self.state["power"]["volt"] = round(m.voltage, 2)
        self.state["power"]["pct"] = round(m.percentage * 100) if m.percentage <= 1.0 else round(m.percentage)
        self.state["safety"]["battery_ok"] = not (0.0 < m.voltage < 10.5 or 0.0 < m.percentage < 0.15)

    def _on_pose(self, m):
        p = m.pose.position
        roll, pitch, yaw = _euler_deg(m.pose.orientation)
        fl = self.state["flight"]
        fl["x"], fl["y"], fl["alt"] = round(p.x, 2), round(p.y, 2), round(p.z, 2)
        fl["roll_deg"], fl["pitch_deg"], fl["yaw_deg"] = round(roll, 1), round(pitch, 1), round(yaw, 1)
        self.state["scan"]["yaw_deg"] = round(yaw, 1)

    def _on_vel(self, m):
        self.state["flight"]["gs"] = round(math.hypot(m.twist.linear.x, m.twist.linear.y), 2)

    def _on_scan(self, m):
        n = len(m.ranges)
        if not n:
            return
        step = max(1, n // 72)
        out = []
        for i in range(0, n, step):
            r = m.ranges[i]
            out.append(round(r, 2) if (math.isfinite(r) and m.range_min < r < m.range_max) else None)
        self.state["scan"]["ranges"] = out

    def _on_gps(self, m):
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
        p = self.state["percep"]
        p["redzone_status"] = d.get("status", "UNKNOWN")
        p["redzone_reason"] = d.get("reason", "")
        p["redzone_exclusions"] = d.get("exclusions", [])
        p["redzone_area_m2"] = round(float(d.get("confirmed_area_m2", 0.0)), 1)

    def _on_avoid(self, m):
        self.state["nav"] = {"front_m": round(m.x, 2),
                             "centering_err": round(m.y, 2), "cmd_vx": round(m.z, 2)}

    def _on_ready(self, m):
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
        now = time.time()
        if now - self._last_map < 1.0:
            return
        self._last_map = now
        w, h, data = m.info.width, m.info.height, m.data
        if not w or not h:
            return
        step = max(1, max(w, h) // 120)          # cap output ~120 cells/side
        ow, oh = (w + step - 1) // step, (h + step - 1) // step
        out = [-1] * (ow * oh)
        for r in range(0, h, step):
            orow = (r // step) * ow
            for c in range(0, w, step):
                best = -1                         # max-pool so obstacles survive
                for dr in range(step):
                    rr = r + dr
                    if rr >= h:
                        break
                    rb = rr * w
                    for dc in range(step):
                        cc = c + dc
                        if cc >= w:
                            break
                        v = data[rb + cc]
                        if v > best:
                            best = v
                out[orow + c // step] = int(best)
        grid = {"res": round(m.info.resolution * step, 3), "w": ow, "h": oh,
                "ox": round(m.info.origin.position.x, 3),
                "oy": round(m.info.origin.position.y, 3), "data": out}
        self._broadcast(self._env("map", grid))

    # ---- outbound ---- #
    def _env(self, kind, data):
        return json.dumps({"v": SCHEMA_VERSION, "kind": kind, "t": time.time(), "data": data})

    def _publish_snapshot(self):
        # Refresh staleness on every snapshot so a field that stops updating
        # is visibly stale rather than frozen at its last confident value.
        self.state["safety"]["stale"] = self._staleness()
        self._broadcast(self._env("telemetry", self.state))

    def _broadcast(self, payload):
        if self._loop and self._ws_clients:
            asyncio.run_coroutine_threadsafe(self._bcast(payload), self._loop)

    async def _bcast(self, msg):
        dead = set()
        for ws in self._ws_clients:
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
        if msg.get("kind") != "command":
            return
        d = msg.get("data", {})
        cmd, cid = d.get("cmd"), d.get("cmd_id")
        result, reason = self._dispatch(cmd, d.get("args", {}))
        await ws.send(self._env("ack", {"cmd_id": cid, "cmd": cmd,
                                        "result": result, "reason": reason}))

    def _dispatch(self, cmd, args):
        try:
            # Flight-changing commands must never bypass the readiness
            # interlock. Abort / land / RTL remain available when unhealthy.
            if cmd in {"arm", "takeoff", "start_mission"} and not self.state["safety"]["ready"]:
                return "rejected", "mission interlock not ready"
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
                self.pub_gimbal_pitch.publish(Float64(data=math.radians(pitch_deg)))
                self.state["gimbal"]["pitch_deg"] = round(pitch_deg, 1)
            elif cmd == "abort":
                self.pub_abort.publish(Bool(data=True))
            elif cmd == "start_mission":
                self.state["mission"]["selected"] = "M2"
                self.state["mission"]["state"] = "STARTING"
                # A new run gets a clean ledger. Carrying the previous run's
                # observations forward would put two flights' markers in one
                # list with nothing to say which was which -- and the operator
                # reads this panel to find out what THIS flight saw.
                self.state["scans"] = []
                self._scan_seq = 0
                self.pub_start.publish(Bool(data=True))
            elif cmd == "set_target":
                self.pub_target.publish(String(data=args.get("target", "")))
            else:
                return "rejected", "unknown cmd"
            return "accepted", ""
        except Exception as e:  # noqa: BLE001
            return "rejected", str(e)

    def start_ws(self):
        def run():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(websockets.serve(self._handler, WS_HOST, WS_PORT))
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
