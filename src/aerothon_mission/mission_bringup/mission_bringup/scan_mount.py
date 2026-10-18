#!/usr/bin/env python3
"""Put the lidar's scan into the airframe's frame: beam 0 on the nose, CCW.

Every scan consumer -- the corridor navigator, the square-up, the gate check,
the keep-out -- reads a beam's bearing as angle_min + i * angle_increment,
relative to the nose, counter-clockwise, and wraps it. So a lidar mounted
turned, or a driver scanning clockwise, is corrected here by rewriting those
two numbers; the ranges pass through untouched. The sim publishes /scan in
the body frame already and does not run this.

Two stages. The driver's /scan_driver becomes /scan_raw, the lidar's own
frame: its marked 0 deg at bearing 0. sllidar_ros2 publishes the C1's 0 deg at
pi (bearing = pi - device angle), so the C1 launches with driver_yaw_deg 180;
the LD06 is used as published. Calibration and scripts/check_sensors.py
measure in /scan_raw. The mount then turns /scan_raw into /scan:

    driver_yaw_deg  where the driver's bearing 0 points in the lidar's frame
    lidar_yaw_deg   where the lidar's marked 0 deg points, CCW from the nose
    mirrored        the driver's angles run clockwise

Both are MEASURED on the aircraft -- from the GCS's LIDAR calibration mode, or
with scripts/check_sensors.py -- not read off a drawing.

The GCS sends a measured mount on /mission/lidar_mount; it applies at once,
with no relaunch, and is saved to `mount_file` so it survives a restart. A
saved mount wins over the launch arguments, which are only the starting
point before anything was measured. What is in force is published, latched,
on /mission/lidar_mount/applied.
"""

import json
import math
import os
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_sensor_data)
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

DEFAULT_MOUNT_FILE = os.path.expanduser("~/.aerothon/lidar_mount.json")


def parse_mount(data):
    """(yaw_deg, mirrored) from a mount message or file, or ValueError."""
    if not isinstance(data, dict):
        raise ValueError("a lidar mount is an object")
    yaw = float(data["yaw_deg"])
    if not math.isfinite(yaw) or not -180.0 <= yaw <= 180.0:
        raise ValueError(f"lidar yaw {yaw} is outside -180..180 deg")
    mirrored = data.get("mirrored")
    if not isinstance(mirrored, bool):
        raise ValueError("mirrored must be true or false")
    return yaw, mirrored


def device_frame(scan, driver_yaw_rad):
    """`scan` with bearings in the lidar's own frame, marked 0 deg at 0."""
    scan.angle_min += driver_yaw_rad
    scan.angle_max += driver_yaw_rad
    return scan


def mount(scan, yaw_rad, mirrored):
    """`scan` re-expressed with beam bearings in the body frame."""
    if mirrored:
        scan.angle_min, scan.angle_max = -scan.angle_min, -scan.angle_max
        scan.angle_increment = -scan.angle_increment
    scan.angle_min += yaw_rad
    scan.angle_max += yaw_rad
    return scan


def measured_mount(data):
    """Validate the full measured record; legacy yaw-only records stay labelled."""
    parse_mount(data)
    keys = ('x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'range_scale', 'range_bias_m')
    if not any(k in data for k in keys):
        return None
    limits = (2., 2., 2., 3., 3., 1.1, .1)
    for key, bound in zip(keys, limits):
        v = data.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > bound:
            raise ValueError(f'invalid measured lidar {key}')
    if data['range_scale'] < .9:
        raise ValueError('invalid measured range scale')
    if data.get('ok') is not True or not isinstance(data.get('quality'), dict):
        raise ValueError('full lidar calibration needs passing independent checks')
    return copy.deepcopy(data)


def mount_quaternion(data):
    """Residual tilt after yaw was moved into LaserScan bearings.

    R = Rz(yaw) Ry(pitch) Rx(roll) Rz(-yaw), so TF * corrected_ray is
    exactly the physical Rz(yaw) Ry(pitch) Rx(roll) * raw_ray.
    """
    def product(a, b):
        x, y, z, w = a
        X, Y, Z, W = b
        return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)
    r, p, y = (math.radians(data[k]) / 2 for k in ('roll_deg', 'pitch_deg', 'yaw_deg'))
    return product(product(product((0, 0, math.sin(y), math.cos(y)),
                                    (0, math.sin(p), 0, math.cos(p))),
                            (math.sin(r), 0, 0, math.cos(r))),
                   (0, 0, -math.sin(y), math.cos(y)))


class ScanMount(Node):
    def __init__(self, **kwargs):
        super().__init__("scan_mount", **kwargs)
        self.driver_yaw = math.radians(self.declare_parameter("driver_yaw_deg", 0.0).value)
        self.yaw = math.radians(self.declare_parameter("lidar_yaw_deg", 0.0).value)
        self.mirrored = bool(self.declare_parameter("mirrored", False).value)
        self.mount_file = self.declare_parameter(
            "mount_file", DEFAULT_MOUNT_FILE).value
        self.source = "launch arguments"
        self.calibration = None
        self.mount_tf = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_applied = self.create_publisher(
            String, "/mission/lidar_mount/applied", latched)
        self._load_saved()
        self.pub_raw = self.create_publisher(LaserScan, "/scan_raw", qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_driver", self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/mission/lidar_mount",
                                 self._on_mount, latched)
        self._report("")

    def _load_saved(self):
        try:
            with open(self.mount_file, encoding="utf-8") as f:
                data = json.load(f)
                yaw, mirrored = parse_mount(data)
                calibration = measured_mount(data)
        except FileNotFoundError:
            return
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f"saved lidar mount ignored: {error}")
            return
        self.yaw, self.mirrored = math.radians(yaw), mirrored
        self.calibration = calibration
        self.source = f"saved calibration ({self.mount_file})"

    def _on_mount(self, msg):
        """A mount measured at the GCS: apply now, and keep it."""
        try:
            data = json.loads(msg.data)
            yaw, mirrored = parse_mount(data)
            calibration = measured_mount(data)
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f"lidar mount refused: {error}")
            self._report(str(error))
            return
        self.yaw, self.mirrored = math.radians(yaw), mirrored
        self.calibration = calibration
        self.source = "GCS calibration"
        try:
            os.makedirs(os.path.dirname(self.mount_file) or ".", exist_ok=True)
            tmp = self.mount_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, allow_nan=False)
            os.replace(tmp, self.mount_file)
        except OSError as error:
            self.get_logger().warning(f"lidar mount applied but not saved: {error}")
            self._report(f"applied, not saved: {error}")
            return
        self.get_logger().info(f"lidar mount -> yaw {yaw:+.1f} deg, "
                               f"{'mirrored' if mirrored else 'counter-clockwise'}")
        self._report("")

    def _report(self, reason):
        calibration = getattr(self, 'calibration', None)
        if calibration and getattr(self, 'mount_tf', None):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'lidar_calibrated'
            t.transform.translation.x = float(calibration['x_m'])
            t.transform.translation.y = float(calibration['y_m'])
            t.transform.translation.z = float(calibration['z_m'])
            q = mount_quaternion(calibration)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            self.mount_tf.sendTransform(t)
        report = dict(calibration or {})
        report.update(yaw_deg=round(math.degrees(self.yaw), 3), mirrored=self.mirrored,
                      source=self.source, reason=reason, geometry_measured=calibration is not None)
        self.pub_applied.publish(String(data=json.dumps(report, separators=(",", ":"))))

    def _on_scan(self, m):
        m = device_frame(m, self.driver_yaw)
        # publish() serialises now, so mounting the same message after is safe.
        self.pub_raw.publish(m)
        s = mount(m, self.yaw, self.mirrored)
        if self.calibration:
            c = self.calibration
            s.header.frame_id = 'lidar_calibrated'
            s.ranges = [float(c['range_scale'] * r + c['range_bias_m'])
                        if math.isfinite(r) and m.range_min <= r <= m.range_max else float('inf')
                        for r in s.ranges]
            s.range_min = max(0., c['range_scale'] * s.range_min + c['range_bias_m'])
            s.range_max = c['range_scale'] * s.range_max + c['range_bias_m']
        self.pub.publish(s)


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
