#!/usr/bin/env python3
"""Put the LD06's scan into the airframe's frame: beam 0 on the nose, CCW.

Every scan consumer -- the corridor navigator, the square-up, the gate check,
the keep-out -- reads a beam's bearing as angle_min + i * angle_increment,
relative to the nose, counter-clockwise, and wraps it. So a lidar mounted
turned, or a driver scanning clockwise, is corrected here by rewriting those
two numbers; the ranges pass through untouched. The sim publishes /scan in
the body frame already and does not run this.

    lidar_yaw_deg   where the lidar's own 0 deg points, CCW from the nose
    mirrored        the driver's angles run clockwise

Both are MEASURED on the aircraft with scripts/check_sensors.py, not read off
a drawing.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def mount(scan, yaw_rad, mirrored):
    """`scan` re-expressed with beam bearings in the body frame."""
    if mirrored:
        scan.angle_min, scan.angle_max = -scan.angle_min, -scan.angle_max
        scan.angle_increment = -scan.angle_increment
    scan.angle_min += yaw_rad
    scan.angle_max += yaw_rad
    return scan


class ScanMount(Node):
    def __init__(self, **kwargs):
        super().__init__("scan_mount", **kwargs)
        self.yaw = math.radians(self.declare_parameter("lidar_yaw_deg", 0.0).value)
        self.mirrored = bool(self.declare_parameter("mirrored", False).value)
        self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan,
                                 qos_profile_sensor_data)

    def _on_scan(self, m):
        self.pub.publish(mount(m, self.yaw, self.mirrored))


def main():
    rclpy.init()
    node = ScanMount()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
