#!/usr/bin/env python3
"""What the lidar reports from a KNOWN pose. Ground characterisation.

WHY THIS IS A TOOL AND NOT A FLIGHT

    Squaring up to the banner is decided on one number -- the angle between
    the gate's face and the aircraft's nose -- and that number comes out of a
    line fit through `/scan`. When the fit misbehaves there are at least four
    candidate causes (the sensor rate, the sector, the surface actually being
    hit, the fit itself) and a mission flight distinguishes none of them: it
    reports the composition of all four, once, ten minutes later.

    So: park the aircraft where the geometry is KNOWN and read the fit. On the
    shipped arena the forward gate stands at (2, 2) with its face normal along
    -x, which makes the answer computable in advance. An aircraft holding
    (0, 0) at heading 0 is 1.94 m off that face and exactly square to it, so
    the fit must report about 0 degrees; yawed to +30 it must report about
    -30. Those two readings separate "the geometry is wrong" from "the scan is
    wrong" in about ninety seconds.

WHAT IT REPORTS, PER STATION

    rate        measured /scan Hz against the 10 Hz the model configures. The
                launcher already documents that the Gazebo GUI costs enough
                rendering to starve this; AEROTHON_HEADLESS=1 for measurement.
    returns     finite ranges in the whole scan, and in the forward sector
    fit         the surface `fit_surface` finds, or its refusal
    sweep       the same fit taken across a fan of sectors, which is what
                shows WHERE the surfaces around the aircraft actually are
                rather than only what the sector under test happened to catch

USAGE

    source /opt/ros/jazzy/setup.bash
    python3 sim/probe_surface.py --at 0,0,3,0 --at 0,0,3,30
    python3 sim/probe_surface.py --at 0,0,3,0 --expect-angle 0 --expect-range 1.94
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, __file__.rsplit("/", 2)[0]
                + "/src/aerothon_mission/mission_bt")

from mission_bt.scan_geometry import fit_surface        # noqa: E402


class Probe(Node):
    def __init__(self):
        super().__init__("surface_probe")
        self.state = State()
        self.pose = None
        self.scans = []                 # arrival timestamps, for the rate
        self.scan = None
        self._sp = None
        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.pub_sp = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.create_timer(0.1, self._stream)

    def _on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)

    def _on_scan(self, m):
        self.scan = m
        self.scans.append(time.time())
        del self.scans[:-200]

    def _stream(self):
        if self._sp is not None and self.state.armed:
            self._sp.header.stamp = self.get_clock().now().to_msg()
            self._sp.header.frame_id = "map"
            self.pub_sp.publish(self._sp)

    def goto(self, x, y, z, yaw):
        sp = PoseStamped()
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp = sp

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait(self, pred, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        print(f"    TIMED OUT waiting for {what}")
        return False

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    # ------------------------------------------------------------------ #
    def rate_hz(self):
        if len(self.scans) < 3:
            return 0.0
        span = self.scans[-1] - self.scans[0]
        return (len(self.scans) - 1) / span if span > 0 else 0.0

    def finite(self):
        if self.scan is None:
            return []
        s = self.scan
        return [r for r in s.ranges
                if r is not None and r == r
                and r not in (float("inf"), float("-inf"))
                and s.range_min < r < s.range_max]

    def fit(self, sector_rad, half_rad):
        if self.scan is None:
            return {"ok": False, "reason": "no scan has arrived"}
        s = self.scan
        return fit_surface(s.angle_min, s.angle_increment, s.ranges,
                           sector_rad, half_rad,
                           range_min=float(s.range_min),
                           range_max=float(s.range_max))


def report(p, half_rad, sweep_step_deg):
    print(f"    pose        ({p.pose[0]:+.2f}, {p.pose[1]:+.2f}, "
          f"{p.pose[2]:.2f}) heading {math.degrees(p.pose[3]):+.1f} deg")
    print(f"    scan rate   {p.rate_hz():.1f} Hz "
          f"(model configures 10; the GUI is documented to starve it)")
    fin = p.finite()
    print(f"    returns     {len(fin)} finite of "
          f"{len(p.scan.ranges) if p.scan else 0}"
          + (f", nearest {min(fin):.2f} m" if fin else ""))
    f = p.fit(0.0, half_rad)
    if f["ok"]:
        print(f"    FIT ahead   {math.degrees(f['angle_rad']):+7.1f} deg  "
              f"{f['range_m']:5.2f} m  {f['points']:3d} returns  "
              f"residual {f['residual_m'] * 100:.1f} cm  "
              f"span {f['extent_m']:.2f} m")
    else:
        print(f"    FIT ahead   REFUSED: {f['reason']}")

    print(f"    sweep       what the fit says in each direction "
          f"(+/-{math.degrees(half_rad):.0f} deg sectors):")
    for deg in range(-90, 91, sweep_step_deg):
        f = p.fit(math.radians(deg), half_rad)
        if f["ok"]:
            print(f"      sector {deg:+4d}  face {math.degrees(f['angle_rad']):+7.1f} "
                  f"deg  {f['range_m']:5.2f} m  {f['points']:3d} pts  "
                  f"span {f['extent_m']:.2f} m  "
                  f"res {f['residual_m'] * 100:.1f} cm")
        else:
            print(f"      sector {deg:+4d}  -- {f['reason'][:78]}")
    return f


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--at", action="append", default=[],
                    metavar="X,Y,ALT,YAWDEG",
                    help="a station to hold and measure from; repeatable")
    ap.add_argument("--settle", type=float, default=8.0,
                    help="seconds to hold each station before measuring")
    ap.add_argument("--half-width-deg", type=float, default=35.0)
    ap.add_argument("--sweep-step-deg", type=int, default=15)
    ap.add_argument("--json", default="", help="write the readings here")
    ap.add_argument("--no-fly", action="store_true",
                    help="measure from wherever the aircraft already is")
    a = ap.parse_args()

    stations = []
    for s in a.at:
        x, y, z, yaw = (float(v) for v in s.split(","))
        stations.append((x, y, z, math.radians(yaw)))

    rclpy.init()
    p = Probe()
    half = math.radians(a.half_width_deg)
    rows = []

    print("=" * 74)
    print(" LIDAR SURFACE PROBE -- what the fit reports from a known pose")
    print("=" * 74)
    if not p.wait(lambda: p.state.connected, 60, "FCU connection"):
        return 2
    if not p.wait(lambda: p.pose is not None, 60, "local position"):
        return 2

    if stations and not a.no_fly:
        print("\n[1] GUIDED / ARM / TAKEOFF")
        p.call(p.cli_mode, SetMode.Request(custom_mode="GUIDED"))
        p.wait(lambda: p.state.mode == "GUIDED", 20, "GUIDED")
        for _ in range(10):
            p.call(p.cli_arm, CommandBool.Request(value=True))
            p.spin(1.0)
            if p.state.armed:
                break
        if not p.state.armed:
            print("    FAILED to arm")
            return 1
        alt0 = stations[0][2]
        p.call(p.cli_takeoff, CommandTOL.Request(altitude=float(alt0)))
        p.wait(lambda: p.pose[2] > alt0 - 0.5, 90, "takeoff altitude")

    if not stations:
        stations = [(*p.pose[:3], p.pose[3])]

    for i, (x, y, z, yaw) in enumerate(stations, 1):
        print(f"\n--- station {i}/{len(stations)}: "
              f"({x:+.2f}, {y:+.2f}, {z:.2f}) heading "
              f"{math.degrees(yaw):+.0f} deg ---")
        if not a.no_fly:
            p.goto(x, y, z, yaw)
            p.wait(lambda: (math.dist(p.pose[:3], (x, y, z)) < 0.6
                            and abs(math.atan2(math.sin(p.pose[3] - yaw),
                                               math.cos(p.pose[3] - yaw)))
                            < math.radians(6.0)),
                   45, "the station")
        p.scans.clear()
        p.spin(a.settle)
        f = report(p, half, a.sweep_step_deg)
        rows.append({"station": [x, y, z, math.degrees(yaw)],
                     "pose": list(p.pose), "rate_hz": round(p.rate_hz(), 2),
                     "finite": len(p.finite()),
                     "fit_ahead": p.fit(0.0, half)})

    if a.json:
        with open(a.json, "w") as fh:
            json.dump(rows, fh, indent=2, default=str)
        print(f"\nwrote {a.json}")
    p.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
