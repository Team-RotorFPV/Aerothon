"""Independent LiDAR mapping TF tree, including operation without GPS/FCU pose."""
import copy
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener

from .lidar_odometry import ScanOdometry


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    length = math.sqrt(x*x+y*y+z*z+w*w)
    if length < 1e-6:
        raise ValueError('invalid lidar mount quaternion')
    x, y, z, w = x/length, y/length, z/length, w/length
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class MappingOdometry(Node):
    def __init__(self):
        super().__init__('lidar_mapping_odometry')
        self.matcher = ScanOdometry()
        self.mount_signature = None
        self.halted = False
        self.static_sent = False
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_slam', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/slam/status', 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)

    def scan(self, m):
        quality = {}
        try:
            if self.halted:
                raise ValueError('mount changed; reconnect runtime before mapping again')
            mount = self.tf_buffer.lookup_transform('base_link', m.header.frame_id, Time())
            v, q = mount.transform.translation, mount.transform.rotation
            signature = tuple(round(x, 7) for x in (v.x,v.y,v.z,q.x,q.y,q.z,q.w))
            if self.mount_signature is not None and signature != self.mount_signature:
                self.halted = True
                raise ValueError('mount changed; reconnect runtime before mapping again')
            self.mount_signature = signature
            matrix = quaternion_matrix(q)
            ranges = np.asarray(m.ranges, dtype=float)
            valid = np.isfinite(ranges) & (ranges > max(.15,m.range_min)) & (ranges < min(12.,m.range_max))
            bearings = m.angle_min + np.flatnonzero(valid)*m.angle_increment
            points = np.column_stack((np.cos(bearings)*ranges[valid],np.sin(bearings)*ranges[valid],np.zeros(valid.sum())))
            points = (points @ matrix.T + np.array([v.x,v.y,v.z]))[:, :2]
            stamp = m.header.stamp.sec + m.header.stamp.nanosec/1e9
            pose, quality = self.matcher.update(points, stamp)
            static = copy.deepcopy(mount)
            static.header.frame_id = 'lidar_mapping_base'
            static.child_frame_id = 'lidar_mapping_scan'
            if not self.static_sent:
                self.static_tf.sendTransform(static)
                self.static_sent = True
            dynamic = TransformStamped()
            dynamic.header = copy.deepcopy(m.header)
            dynamic.header.frame_id = 'lidar_odom'
            dynamic.child_frame_id = 'lidar_mapping_base'
            dynamic.transform.translation.x, dynamic.transform.translation.y = map(float,pose[:2])
            dynamic.transform.rotation.z = math.sin(pose[2]/2)
            dynamic.transform.rotation.w = math.cos(pose[2]/2)
            self.tf.sendTransform(dynamic)
            scan = copy.deepcopy(m)
            scan.header.frame_id = 'lidar_mapping_scan'
            self.scan_pub.publish(scan)
            state, reason = 'Tracking', ''
        except (TransformException, ValueError, np.linalg.LinAlgError) as exc:
            state, reason = 'Held', str(exc)
        self.status.publish(String(data=json.dumps(dict(source='LiDAR scan matching', state=state, reason=reason,
                                                        frame='lidar_odom', **quality),allow_nan=False)))


def main():
    rclpy.init()
    node = MappingOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
