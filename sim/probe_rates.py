#!/usr/bin/env python3
"""Measure the real-time factor and the rates the readiness gate depends on.

    python3 sim/probe_rates.py --seconds 20

RTF is sim seconds per wall second, from /clock. Topic rates are reported
per WALL second (what wait_ready.py and the GCS interlock see) and per SIM
second (what the sensors are configured for). A low wall rate with a normal
sim rate is the host, not the stack.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = Node("probe_rates")
    counts = {"pose": 0, "scan": 0, "camera": 0}
    clock = {"first": None, "last": None}

    def on_clock(m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if clock["first"] is None:
            clock["first"] = (t, time.monotonic())
        clock["last"] = (t, time.monotonic())

    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                             lambda m: counts.__setitem__("pose", counts["pose"] + 1),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/scan",
                             lambda m: counts.__setitem__("scan", counts["scan"] + 1),
                             qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/image",
                             lambda m: counts.__setitem__("camera", counts["camera"] + 1),
                             qos_profile_sensor_data)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.monotonic() - t0
    rtf = None
    if clock["first"] and clock["last"] and clock["last"][1] > clock["first"][1]:
        rtf = ((clock["last"][0] - clock["first"][0])
               / (clock["last"][1] - clock["first"][1]))
    print(f"wall {wall:.1f} s  RTF {rtf if rtf is None else round(rtf, 3)}")
    for k, n in counts.items():
        sim_hz = (n / (wall * rtf)) if rtf else float("nan")
        print(f"  {k:7s} {n / wall:6.1f} Hz wall   {sim_hz:6.1f} Hz sim")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
