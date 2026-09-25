#!/usr/bin/env python3
"""Payload detector: is the delivery payload in the camera frame, and where.

    sub  /camera/image       sensor_msgs/Image   (param image_topic)
    pub  /percep/payload     std_msgs/String     JSON, see payload.detect_payload
                                                 plus "stamp" (s, node clock)

The mission (WinchDrop) uses it to CONFIRM a drop: after the release and the
winch winding back up, the nadir camera must still see the payload on the
ground at the size the altitude predicts. A payload still on the hook would
have been reeled up out of frame; one that fell short would not be on the pad.
"""

import json

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception_redzone.payload import detect_payload


class PayloadNode(Node):
    def __init__(self):
        super().__init__("perception_payload")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("max_rate_hz", 5.0)
        self.bridge = CvBridge()
        self._last = 0.0
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/percep/payload", 10)
        self.get_logger().info(f"perception_payload up; image_topic={topic}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, msg):
        now = self._now()
        if now - self._last < 1.0 / float(self.get_parameter("max_rate_hz").value):
            return
        self._last = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warning(f"bad frame: {e}")
            return
        det = detect_payload(frame)
        det["stamp"] = now
        self.pub.publish(String(data=json.dumps(det)))


def main():
    rclpy.init()
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
