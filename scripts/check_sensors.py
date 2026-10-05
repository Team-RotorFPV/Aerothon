#!/usr/bin/env python3
"""Measure the flight sensors on the aircraft before the first flight.

    python3 scripts/check_sensors.py lidar     # LD06 mount yaw and direction
    python3 scripts/check_sensors.py camera    # C270 rate, exposure, sharpness

Run with the bring-up up (use_sim:=false). Nothing in the sim can find these:
Gazebo mounts the lidar exactly on the nose and scans the way the stack
expects, and a real one is wherever the bracket put it.

lidar: asks for a box held 1 m ahead of the nose, then 1 m to port, reads
where each shows up in the driver's raw scan (/scan_raw) and prints the
lidar_yaw_deg / lidar_mirrored launch arguments that put them at 0 and +90.
camera: reports frame rate, resolution, brightness and clipping, and the
Laplacian sharpness of a still frame, so a smeared or dark stream is caught
on the bench rather than over the pads.
"""

import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

SELF_M = 0.3          # nearer than this is the airframe or the bracket


def nearest_bearing(scan):
    """Raw-frame bearing (deg) and range of the nearest return."""
    r = np.asarray(scan.ranges, dtype=float)
    r[~np.isfinite(r) | (r < max(SELF_M, scan.range_min))] = np.inf
    i = int(np.argmin(r))
    if not np.isfinite(r[i]):
        return None
    a = scan.angle_min + i * scan.angle_increment
    return math.degrees(math.atan2(math.sin(a), math.cos(a))), float(r[i])


def solve_mount(front_deg, port_deg):
    """(lidar_yaw_deg, mirrored) that put `front` on the nose and `port` at
    +90: body = raw + yaw, or -raw + yaw when the driver runs clockwise."""
    turn = (port_deg - front_deg + 180.0) % 360.0 - 180.0
    mirrored = turn < 0.0
    yaw = front_deg if mirrored else -front_deg
    return (yaw + 180.0) % 360.0 - 180.0, mirrored, abs(abs(turn) - 90.0)


class Probe(Node):
    def __init__(self, topic, msg_type):
        super().__init__("check_sensors")
        self.msgs, self.stamps = [], []
        self.create_subscription(msg_type, topic, self._on, qos_profile_sensor_data)

    def _on(self, m):
        self.msgs.append(m)
        self.stamps.append(time.monotonic())

    def collect(self, seconds):
        self.msgs.clear()
        self.stamps.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        span = (self.stamps[-1] - self.stamps[0]) if len(self.stamps) > 1 else 0.0
        return list(self.msgs), (len(self.stamps) - 1) / span if span else 0.0


def check_lidar(probe):
    scans, hz = probe.collect(3.0)
    if not scans:
        sys.exit("FAIL: nothing on /scan_raw -- is the LD06 driver running?")
    s = scans[-1]
    valid = np.isfinite(np.asarray(s.ranges, dtype=float)).mean()
    print(f"/scan_raw {hz:.1f} Hz (LD06: ~10), {len(s.ranges)} beams, "
          f"angle_min {math.degrees(s.angle_min):.0f} deg, increment "
          f"{math.degrees(s.angle_increment):+.2f} deg, {valid:.0%} returns")
    seen = []
    for where in ("1 m straight AHEAD of the nose", "1 m to PORT (left)"):
        input(f"Clear everything within 2 m, hold a box {where}, press Enter ")
        near = nearest_bearing(probe.collect(1.0)[0][-1])
        if near is None:
            sys.exit("FAIL: no return -- box too close, or the lidar is blocked")
        print(f"  nearest return {near[1]:.2f} m at raw {near[0]:+.1f} deg")
        seen.append(near[0])
    yaw, mirrored, err = solve_mount(*seen)
    if err > 20.0:
        sys.exit(f"FAIL: port read {err:.0f} deg away from 90 deg off the nose; "
                 "repeat with the box square to the aircraft")
    print(f"\nlaunch with: lidar_yaw_deg:={yaw:.0f} "
          f"lidar_mirrored:={'true' if mirrored else 'false'}")


def check_camera(probe):
    frames, hz = probe.collect(3.0)
    if not frames:
        sys.exit("FAIL: nothing on /image_raw -- is the C270 driver running?")
    f = frames[-1]
    gray = CvBridge().imgmsg_to_cv2(f, "mono8")
    sharp = cv2.Laplacian(gray.astype(np.float32), cv2.CV_32F).var()
    print(f"/image_raw {hz:.1f} Hz (need >= 15), {f.width}x{f.height} {f.encoding}; "
          f"mean {gray.mean():.0f}/255, {np.mean(gray >= 250):.1%} clipped, "
          f"{np.mean(gray <= 5):.1%} black, sharpness {sharp:.0f}")
    if f.width != 1280 or hz < 15.0:
        print("FAIL: the QR envelope was measured at 1280 px and the sweep "
              "assumes >= 15 Hz")
    if gray.mean() < 50:
        print("WARN: dark at this exposure; raise camera_exposure (100 us units), "
              "not auto -- auto runs to 60+ ms and smears at sweep speed")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what not in ("lidar", "camera"):
        sys.exit(__doc__)
    rclpy.init()
    probe = Probe("/scan_raw", LaserScan) if what == "lidar" else Probe("/image_raw", Image)
    try:
        (check_lidar if what == "lidar" else check_camera)(probe)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
