#!/usr/bin/env python3
"""Publish the Gazebo vehicle odometry as the ROS odom -> base_link TF."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__('gazebo_odom_tf')
        self.declare_parameter('odom_topic', '/odometry')
        topic = self.get_parameter('odom_topic').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odom, 20)
        self.get_logger().info(f'publishing odom -> base_link TF from {topic}')

    def on_odom(self, msg: Odometry):
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
