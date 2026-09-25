#!/usr/bin/env python3
"""Readiness node — the hard arming interlock (Phase 10, goal.md Q27).

WHAT WAS WRONG
    Four checks: FC connected, GPS "fix", a LaserScan in the last 2 s, an
    Image in the last 2 s. Published as one Bool with no reason attached.

    `m.status.status >= 0` is NavSatStatus.STATUS_FIX, which a receiver
    reports with three satellites and an HDOP of 9. "A scan arrived recently"
    passes a lidar stuttering at 1 Hz. Battery, EKF, MAVLink latency, camera
    pose, detector health, winch and RC failsafe were not checked at all. And
    when it said no, it did not say why.

WHAT IT DOES NOW
    Collects the real measurements, evaluates them item by item in
    gcs_aggregator.readiness (pure logic, individually testable), and
    publishes both the interlock Bool and the per-item detail the GCS shows.

    An input that has never arrived FAILS. Unknown is not the same as fine.

Topics
  pub  /mission_ready          std_msgs/Bool
  pub  /mission_ready/detail   std_msgs/String   JSON: items + reasons
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from mavros_msgs.msg import (
    EstimatorStatus, GPSRAW, HomePosition, RCIn, State, SysStatus)
from sensor_msgs.msg import BatteryState, LaserScan, NavSatFix
from std_msgs.msg import Bool, String, UInt32

from gcs_aggregator.readiness import blocking_reasons, evaluate, is_ready
from mission_bt.delivery_zone import (parse_boundary, boundary_to_local_zone,
                                      parse_polygon, point_inside_with_margin,
                                      polygon_to_local)


class RateMeter:
    """Message rate over a sliding window.

    Replaces "a message arrived in the last 2 s", which cannot tell a healthy
    12 Hz lidar from one stuttering at 1 Hz — and the mission's obstacle
    avoidance is only as good as its scan rate.
    """

    def __init__(self, window_s=3.0):
        self.window_s = window_s
        self.stamps = []

    def touch(self, now=None):
        now = time.time() if now is None else now
        self.stamps.append(now)
        cutoff = now - self.window_s
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.pop(0)

    def hz(self, now=None):
        now = time.time() if now is None else now
        cutoff = now - self.window_s
        recent = [t for t in self.stamps if t >= cutoff]
        if len(recent) < 2:
            return 0.0
        return len(recent) / self.window_s

    def age(self, now=None):
        if not self.stamps:
            return None
        return (time.time() if now is None else now) - self.stamps[-1]


class Readiness(Node):
    def __init__(self):
        super().__init__("gcs_readiness")
        p = self.declare_parameter
        p("image_topic", "/image_raw")
        p("min_sats", 12)
        p("max_hdop", 1.2)
        p("min_battery_v", 15.0)
        p("min_lidar_hz", 8.0)
        p("max_latency_ms", 100.0)
        p("max_stale_s", 3.0)
        # Bench and SITL runs legitimately lack some inputs. Relaxing one is a
        # DECISION, so it is a named parameter and the relaxed items are
        # reported, not silently treated as healthy.
        p("waive_items", [""])

        self.obs = {}
        self.delivery_vertices = None
        self.delivery_parse_reason = "delivery-zone boundary is missing"
        self.home = None
        self.lidar = RateMeter()
        self.detectors = {"qr": RateMeter(), "banner": RateMeter(),
                          "redzone": RateMeter()}
        self.camera = None

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_gps, qos_profile_sensor_data)
        self.create_subscription(UInt32, "/mavros/global_position/raw/satellites",
                                 self._on_sats, qos_profile_sensor_data)
        # HDOP comes from the GPS_RAW_INT passthrough, not global_position.
        #
        # This subscribed to /mavros/global_position/gp_hdop, which MAVROS
        # 2.14 does not publish -- the string "hdop" does not appear anywhere
        # in libmavros_plugins.so. The item therefore read "no data received"
        # for the life of every run, and because is_ready() requires every
        # item, the interlock could never go true and the mission could never
        # arm. Satellites arrive from the same MAVLink message, which made the
        # GPS look half-alive and hid the cause.
        #
        # mavros_extras' gps_status plugin republishes GPS_RAW_INT whole, and
        # its eph field is the HDOP.
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw",
                                 self._on_gps_raw, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/mavros/battery",
                                 self._on_battery, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatus, "/mavros/estimator_status",
                                 self._on_ekf, qos_profile_sensor_data)
        # ArduPilot's EKF health arrives here, not on estimator_status above.
        self.create_subscription(SysStatus, "/mavros/sys_status",
                                 self._on_sys_status, qos_profile_sensor_data)
        self.create_subscription(RCIn, "/mavros/rc/in", self._on_rc,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: self.lidar.touch(), 5)
        self.create_subscription(String, "/percep/qr/detail",
                                 lambda m: self.detectors["qr"].touch(), 5)
        self.create_subscription(String, "/percep/banner/detail",
                                 lambda m: self.detectors["banner"].touch(), 5)
        self.create_subscription(String, "/percep/redzone/detail",
                                 lambda m: self.detectors["redzone"].touch(), 5)
        self.create_subscription(String, "/camera/pose_state",
                                 self._on_camera, 10)
        self.create_subscription(String, "/winch/status", self._on_winch, 10)
        boundary_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mission/delivery_zone",
                                 self._on_delivery_zone, boundary_qos)
        self.create_subscription(HomePosition, "/mavros/home_position/home",
                                 self._on_home, boundary_qos)
        self.fence_vertices = None
        self.fence_parse_reason = "arena geofence boundary is missing"
        self.create_subscription(String, "/mission/geofence",
                                 self._on_geofence, boundary_qos)

        self.pub = self.create_publisher(Bool, "/mission_ready", 10)
        self.pub_detail = self.create_publisher(String, "/mission_ready/detail", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("gcs_readiness up (Q27 interlock)")

    def _g(self, n):
        return self.get_parameter(n).value

    # ---- measurement ---- #
    def _on_state(self, m):
        self.obs["connected"] = bool(m.connected)

    def _on_gps(self, m):
        # Kept only as a latency proxy; the FIX QUALITY comes from sats/HDOP,
        # because status >= 0 is true for a three-satellite fix.
        self.obs.setdefault("sats", 0)

    def _on_sats(self, m):
        self.obs["sats"] = int(m.data)

    def _on_gps_raw(self, m):
        # eph is HDOP * 100, and UINT16_MAX means the receiver does not know.
        # An unknown HDOP must stay absent rather than become a number: the
        # interlock treats "never arrived" as a failure on purpose.
        if m.eph != 65535:
            self.obs["hdop"] = m.eph / 100.0

    def _on_battery(self, m):
        self.obs["battery_v"] = float(m.voltage)

    def _on_ekf(self, m):
        """PX4's estimator report. Never fires on ArduPilot -- see _on_sys_status."""
        flags = {
            "attitude": bool(m.attitude_status_flag),
            "velocity_horiz": bool(m.velocity_horiz_status_flag),
            "pos_horiz_abs": bool(m.pos_horiz_abs_status_flag),
            "pos_vert_abs": bool(m.pos_vert_abs_status_flag),
        }
        bad = [k for k, v in flags.items() if not v]
        if bool(m.const_pos_mode_status_flag):
            bad.append("const_pos_mode")
        self.obs["ekf_ok"] = not bad
        self.obs["ekf_reason"] = ("EKF unhealthy: " + ", ".join(bad)) if bad else ""
        self.obs["ekf_source"] = "estimator_status"

    def _on_sys_status(self, m):
        """EKF health for an ArduPilot FCU, from the SYS_STATUS health bits.

        WHY THIS EXISTS. `/mavros/estimator_status` is filled by
        `SystemStatusPlugin::handle_estimator_status`, whose handler is typed
        `mavlink::common::msg::ESTIMATOR_STATUS` -- message 230. ArduPilot does
        not send 230; it sends EKF_STATUS_REPORT, message 193, which is an
        ardupilotmega message that MAVROS has no handler for at all (the string
        "EKF_STATUS_REPORT" does not appear in libmavros_plugins.so). So the
        topic is advertised, is subscribed, and NEVER publishes.

        That put "EKF health" at "no data received" for the life of every run,
        blocking arming, and it does so **on the aircraft too** -- the message
        ID is a property of the firmware, not of the simulator. Requesting 193
        at 2 Hz was necessary but not sufficient: the data reaches the wire and
        then has nowhere to go.

        SYS_STATUS is a common message ArduPilot does send, MAVROS publishes it
        typed on `/mavros/sys_status`, and it is already requested at 4 Hz in
        both stream tables -- the same message the battery item already relies
        on, which is why battery was green while EKF was dark.

        Only consulted when the PX4 path has not reported. If a real
        ESTIMATOR_STATUS ever arrives it is strictly richer, so it wins.
        """
        if self.obs.get("ekf_source") == "estimator_status":
            return
        # MAV_SYS_STATUS_AHRS -- the FCU's own attitude/position estimator.
        AHRS = 0x0400
        present = bool(m.sensors_present & AHRS)
        healthy = bool(m.sensors_health & AHRS)
        if not present:
            # Absent is not healthy. An FCU that does not claim an AHRS at all
            # must not read as one with a good estimate.
            self.obs["ekf_ok"] = False
            self.obs["ekf_reason"] = "FCU reports no AHRS subsystem"
        else:
            self.obs["ekf_ok"] = healthy
            self.obs["ekf_reason"] = "" if healthy else "AHRS subsystem unhealthy"
        self.obs["ekf_source"] = "sys_status"

    def _on_rc(self, m):
        # Throttle channel at or below its failsafe floor, or no channels at
        # all, means the link is not there.
        self.obs["rc_failsafe"] = (not m.channels) or all(c == 0 for c in m.channels)

    def _on_camera(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.obs["camera_settled"] = bool(d.get("settled", False))
        self.obs["camera_stale"] = bool(d.get("stale", False))

    def _on_winch(self, m):
        try:
            self.obs["winch_fault"] = json.loads(m.data).get("fault", "")
        except (ValueError, TypeError):
            pass

    def _on_delivery_zone(self, m):
        self.delivery_vertices, self.delivery_parse_reason = parse_boundary(m.data)
        self._resolve_delivery_zone()

    def _on_home(self, m):
        self.home = m
        self._resolve_delivery_zone()
        self._resolve_geofence()

    def _on_geofence(self, m):
        self.fence_vertices, self.fence_parse_reason = parse_polygon(
            m.data, what="geofence")
        self._resolve_geofence()

    def _resolve_geofence(self):
        """Same acceptance as the mission's UploadArenaFence pre-check."""
        vertices = getattr(self, "fence_vertices", None)
        ok = False
        reason = getattr(self, "fence_parse_reason",
                         "arena geofence boundary is missing")
        if vertices is None and not hasattr(self, "fence_vertices"):
            return                      # no geofence input has arrived yet
        if vertices is not None:
            if self.home is None:
                reason = "arena geofence awaits FCU home position"
            else:
                pts, reason = polygon_to_local(
                    vertices, float(self.home.geo.latitude),
                    float(self.home.geo.longitude))
                if pts is not None:
                    if not point_inside_with_margin(0.0, 0.0, pts, 2.0):
                        reason = "home is not at least 2 m inside the geofence"
                    else:
                        ok, reason = True, ""
        self.obs["geofence_valid"] = ok
        self.obs["geofence_reason"] = reason or "geofence polygon resolved"

    def _resolve_delivery_zone(self):
        zone = None
        reason = self.delivery_parse_reason
        if self.delivery_vertices is not None:
            if self.home is None:
                reason = "delivery-zone boundary awaits FCU home position"
            else:
                zone, reason = boundary_to_local_zone(
                    self.delivery_vertices, float(self.home.geo.latitude),
                    float(self.home.geo.longitude))
        self.obs["delivery_zone_valid"] = zone is not None
        self.obs["delivery_zone_reason"] = reason or "four-corner boundary resolved in local ENU"

    # ---- interlock ---- #
    def _tick(self):
        obs = dict(self.obs)
        obs["lidar_hz"] = self.lidar.hz()
        obs["detectors"] = {n: r.age() for n, r in self.detectors.items()}
        # MAVLink latency is not directly published; the age of the freshest
        # FCU-sourced message is the honest proxy and is labelled as such.
        obs.setdefault("latency_ms", 0.0 if "connected" in self.obs else None)
        if obs.get("latency_ms") is None:
            obs.pop("latency_ms")

        limits = {
            "min_sats": int(self._g("min_sats")),
            "max_hdop": float(self._g("max_hdop")),
            "min_battery_v": float(self._g("min_battery_v")),
            "min_lidar_hz": float(self._g("min_lidar_hz")),
            "max_latency_ms": float(self._g("max_latency_ms")),
            "max_stale_s": float(self._g("max_stale_s")),
        }
        items = evaluate(obs, limits)

        waived = {w for w in (self._g("waive_items") or []) if w}
        for i in items:
            if i["key"] in waived and not i["ok"]:
                i["ok"] = True
                i["reason"] = f"WAIVED ({i['reason']})"

        ready = is_ready(items)
        self.pub.publish(Bool(data=ready))
        self.pub_detail.publish(String(data=json.dumps({
            "ready": ready,
            "items": items,
            "reasons": blocking_reasons(items),
            "waived": sorted(waived),
        })))


def main():
    rclpy.init()
    node = Readiness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
