#!/usr/bin/env python3
"""How often each perception node actually delivers, in SIM seconds.

    python3 sim/probe_perception_rates.py --seconds 20

The red-zone map is only as fresh as the red-zone node's output rate, and it
needs `confirm_hits` frames on the same cell before a cell counts. At a low
real-time factor a node can fall to a frame every few simulated seconds,
which turns a 5.8 m look-ahead into no look-ahead at all.
"""

import argparse
import time

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

TOPICS = {"camera": ("/camera/image", Image, qos_profile_sensor_data),
          "redzone": ("/percep/redzone/detail", String, 10),
          "qr": ("/percep/qr/detail", String, 10),
          "banner": ("/percep/banner/detail", String, 10)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = Node("probe_perception_rates", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time",
                                  rclpy.parameter.Parameter.Type.BOOL, True)])
    stamps = {k: [] for k in TOPICS}
    for key, (topic, typ, qos) in TOPICS.items():
        node.create_subscription(
            typ, topic,
            lambda m, k=key: stamps[k].append(
                node.get_clock().now().nanoseconds * 1e-9), qos)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for key, ts in stamps.items():
        if len(ts) >= 2 and ts[-1] > ts[0]:
            print(f"  {key:8s} {len(ts) - 1} msgs over {ts[-1] - ts[0]:.1f} sim s"
                  f" -> {(len(ts) - 1) / (ts[-1] - ts[0]):.2f} Hz sim")
        else:
            print(f"  {key:8s} {len(ts)} msgs (too few to rate)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
