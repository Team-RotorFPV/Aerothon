#!/usr/bin/env python3
"""The arming interlock, item by item (Phase 10, goal.md Q27).

WHAT WAS WRONG

    /mission_ready was four checks: FC connected, GPS "fix", a LaserScan seen
    recently, an Image seen recently. It published one Bool.

    Every one of those is weaker than it looks:

      * `m.status.status >= 0` is true for NavSatStatus.STATUS_FIX, which a
        receiver reports with three satellites and an HDOP of 9. That is not a
        fix you fly an autonomous mission on.
      * "a LaserScan arrived in the last 2 s" is not "the lidar is healthy" —
        a sensor stuttering at 1 Hz passes.
      * Nothing checked the battery, the EKF, MAVLink latency, the camera
        pose, the winch, or the RC failsafe state.

    And when it said `false`, it did not say WHY. An operator staring at a
    greyed-out ARM button with no reason is the thing this replaces.

WHAT IT DOES NOW

    Each item is evaluated separately and reports (name, ok, value, reason).
    The interlock is the AND; the GCS shows the list. An item whose input has
    never arrived is NOT ready — unknown is not the same as fine, and defaulting
    it to fine is how an interlock becomes decoration.

Pure logic with no ROS types, so every item can be individually forced to fail
in a test — which is exactly the Phase 10 acceptance criterion.
"""

# Each item: (key, human label). Order is display order.
ITEMS = (
    ("fcu_link", "FCU link"),
    ("gps_sats", "GPS satellites"),
    ("gps_hdop", "GPS HDOP"),
    ("battery", "Battery voltage"),
    ("ekf", "EKF health"),
    ("lidar_rate", "Lidar rate"),
    ("mavlink_latency", "MAVLink latency"),
    ("camera_pose", "Camera pose"),
    ("detectors", "Detector health"),
    ("actuator", "Winch health"),
    ("rc_failsafe", "RC failsafe"),
)

DEFAULT_LIMITS = {
    "min_sats": 12,          # goal.md Q27
    "max_hdop": 1.2,
    "min_battery_v": 15.0,
    "min_lidar_hz": 8.0,
    "max_latency_ms": 100.0,
    "max_stale_s": 3.0,
}


def _unknown(key, label):
    return {"key": key, "label": label, "ok": False, "value": None,
            "reason": "no data received"}


def evaluate(obs, limits=None):
    """Interlock items from an observation dict.

    `obs` keys are all optional; a missing key means that input has never
    arrived, which fails. Values:

        connected        bool
        sats             int
        hdop             float
        battery_v        float
        ekf_ok           bool
        ekf_reason       str
        lidar_hz         float
        latency_ms       float
        camera_settled   bool
        camera_stale     bool
        detectors        {name: age_seconds}
        winch_fault      str      ("" is healthy)
        rc_failsafe      bool     (True = in failsafe, which BLOCKS)

    Returns a list of item dicts, in ITEMS order.
    """
    lim = dict(DEFAULT_LIMITS)
    if limits:
        lim.update(limits)

    out = []

    def item(key, label, ok, value, reason=""):
        out.append({"key": key, "label": label, "ok": bool(ok),
                    "value": value, "reason": reason})

    # ---- FCU link ---- #
    if "connected" not in obs:
        out.append(_unknown("fcu_link", "FCU link"))
    else:
        ok = bool(obs["connected"])
        item("fcu_link", "FCU link", ok, ok,
             "" if ok else "flight controller not connected")

    # ---- GPS ---- #
    if "sats" not in obs:
        out.append(_unknown("gps_sats", "GPS satellites"))
    else:
        sats = int(obs["sats"])
        ok = sats >= lim["min_sats"]
        item("gps_sats", "GPS satellites", ok, sats,
             "" if ok else f"{sats} satellites, need {lim['min_sats']}")

    if "hdop" not in obs:
        out.append(_unknown("gps_hdop", "GPS HDOP"))
    else:
        hdop = float(obs["hdop"])
        ok = hdop < lim["max_hdop"]
        item("gps_hdop", "GPS HDOP", ok, round(hdop, 2),
             "" if ok else f"HDOP {hdop:.2f}, need below {lim['max_hdop']}")

    # ---- battery ---- #
    if "battery_v" not in obs:
        out.append(_unknown("battery", "Battery voltage"))
    else:
        v = float(obs["battery_v"])
        ok = v > lim["min_battery_v"]
        item("battery", "Battery voltage", ok, round(v, 2),
             "" if ok else f"{v:.2f} V, need above {lim['min_battery_v']} V")

    # ---- EKF ---- #
    if "ekf_ok" not in obs:
        out.append(_unknown("ekf", "EKF health"))
    else:
        ok = bool(obs["ekf_ok"])
        item("ekf", "EKF health", ok, ok,
             "" if ok else (obs.get("ekf_reason") or "EKF unhealthy"))

    # ---- lidar ---- #
    if "lidar_hz" not in obs:
        out.append(_unknown("lidar_rate", "Lidar rate"))
    else:
        hz = float(obs["lidar_hz"])
        ok = hz >= lim["min_lidar_hz"]
        item("lidar_rate", "Lidar rate", ok, round(hz, 1),
             "" if ok else f"{hz:.1f} Hz, need {lim['min_lidar_hz']} Hz")

    # ---- MAVLink latency ---- #
    if "latency_ms" not in obs:
        out.append(_unknown("mavlink_latency", "MAVLink latency"))
    else:
        ms = float(obs["latency_ms"])
        ok = ms < lim["max_latency_ms"]
        item("mavlink_latency", "MAVLink latency", ok, round(ms, 1),
             "" if ok else f"{ms:.0f} ms, need below {lim['max_latency_ms']:.0f} ms")

    # ---- camera pose ---- #
    if "camera_settled" not in obs:
        out.append(_unknown("camera_pose", "Camera pose"))
    else:
        settled = bool(obs["camera_settled"])
        stale = bool(obs.get("camera_stale", False))
        ok = settled and not stale
        item("camera_pose", "Camera pose", ok, settled,
             "" if ok else ("camera joint feedback stale" if stale
                            else "camera not settled at a commanded pose"))

    # ---- detectors ---- #
    det = obs.get("detectors")
    if det is None:
        out.append(_unknown("detectors", "Detector health"))
    else:
        stale = sorted(n for n, age in det.items()
                       if age is None or age > lim["max_stale_s"])
        ok = bool(det) and not stale
        item("detectors", "Detector health", ok,
             {n: (None if a is None else round(a, 1)) for n, a in det.items()},
             "" if ok else (f"stale: {', '.join(stale)}" if stale
                            else "no detectors reporting"))

    # ---- winch ---- #
    if "winch_fault" not in obs:
        out.append(_unknown("actuator", "Winch health"))
    else:
        fault = obs["winch_fault"] or ""
        ok = fault == ""
        item("actuator", "Winch health", ok, fault or "healthy",
             "" if ok else f"winch fault: {fault}")

    # ---- RC failsafe ---- #
    if "rc_failsafe" not in obs:
        out.append(_unknown("rc_failsafe", "RC failsafe"))
    else:
        in_fs = bool(obs["rc_failsafe"])
        item("rc_failsafe", "RC failsafe", not in_fs, in_fs,
             "" if not in_fs else "RC failsafe active")

    return out


def is_ready(items):
    """The interlock: every item must pass. Empty is NOT ready."""
    return bool(items) and all(i["ok"] for i in items)


def blocking_reasons(items):
    """Human-readable reasons the interlock is holding, in display order."""
    return [f"{i['label']}: {i['reason']}" for i in items if not i["ok"]]
