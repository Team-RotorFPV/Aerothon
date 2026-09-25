#!/usr/bin/env python3
"""Block until the SITL stack is genuinely usable, then exit 0.

WHAT THIS REPLACES

    live_mission_test.sh gated on short-lived CLI calls:

        ros2 topic echo --once /mavros/state
        ros2 topic hz /mavros/local_position/pose

    Those lose the DDS discovery race against this stack often enough to be
    useless as a gate. A run was reported as "STACK NEVER BECAME READY" with
    `deaths: 0` while a long-lived rclpy node saw, at the same moment:

        state: (connected=True, armed=False, mode=STABILIZE)
        pose:  194 messages in 12 s -> 16.2 Hz

    The stack was fine. The gate was wrong, and it aborted the run.

    Each `--once`/`hz` invocation is a brand new participant that has to
    discover the graph from scratch inside its own timeout. A node that stays
    alive discovers once and then simply counts messages, which is also the
    only way to measure a RATE rather than the presence of one message.

Exit codes
    0  ready
    1  timed out, with each check reported separately

    python3 sim/wait_ready.py --timeout 240 --require-camera --require-scan
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Image, LaserScan


class Readiness(Node):
    def __init__(self, want_camera, want_scan, min_hz):
        super().__init__("wait_ready")
        self.min_hz = min_hz
        self.connected = False
        self.mode = ""
        self.counts = {"pose": 0, "camera": 0, "scan": 0}
        self.first = {}
        self.want_camera = want_camera
        self.want_scan = want_scan

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: self._tick("pose"),
                                 qos_profile_sensor_data)
        if want_camera:
            self.create_subscription(Image, "/camera/image",
                                     lambda m: self._tick("camera"),
                                     qos_profile_sensor_data)
        if want_scan:
            self.create_subscription(LaserScan, "/scan",
                                     lambda m: self._tick("scan"),
                                     qos_profile_sensor_data)

    def _on_state(self, m):
        self.connected = bool(m.connected)
        self.mode = m.mode

    def _tick(self, key):
        self.counts[key] += 1
        self.first.setdefault(key, time.time())

    def hz(self, key):
        """Measured rate, not 'a message arrived once'."""
        n = self.counts[key]
        if n < 2:
            return 0.0
        elapsed = time.time() - self.first[key]
        return n / elapsed if elapsed > 0 else 0.0

    def missing(self):
        out = []
        if not self.connected:
            out.append("FCU connected            MISSING  <- router/MAVProxy ports")
        if self.hz("pose") < self.min_hz:
            out.append(f"local position flowing   MISSING  "
                       f"({self.hz('pose'):.1f} Hz, need {self.min_hz})  "
                       f"<- EKF may still be initialising")
        if self.want_camera and self.hz("camera") < 1.0:
            out.append(f"camera flowing           MISSING  "
                       f"({self.hz('camera'):.1f} Hz)")
        if self.want_scan and self.hz("scan") < 1.0:
            out.append(f"lidar flowing            MISSING  "
                       f"({self.hz('scan'):.1f} Hz)")
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--min-pose-hz", type=float, default=5.0)
    ap.add_argument("--require-camera", action="store_true")
    ap.add_argument("--require-scan", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Readiness(args.require_camera, args.require_scan, args.min_pose_hz)
    start = time.time()
    try:
        while time.time() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            # Needs a couple of seconds of observation before a rate means
            # anything; a single message is not a rate.
            if time.time() - start > 4.0 and not node.missing():
                print(f"stack ready after ~{time.time() - start:.0f}s "
                      f"(pose {node.hz('pose'):.1f} Hz, mode {node.mode})",
                      flush=True)
                return 0
        print(f"STACK NOT HEALTHY after {args.timeout:.0f}s:", flush=True)
        for line in node.missing():
            print(f"  {line}", flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
