#!/usr/bin/env python3
"""One camera feed with every detection drawn on it.

WHAT THIS REPLACES

    The GCS showed exactly one stream, hardcoded:

        u.pathname = "/stream"; u.search = "?topic=/percep/qr/annotated"

    so an operator watching the panel saw the QR detector's private copy of
    the frame and nothing else. During banner alignment -- the stage where
    watching the camera matters most -- the pane showed a nadir QR view with
    no banner box on it, because each detector drew onto its own image and
    published its own annotated topic.

WHY COMPOSITE HERE RATHER THAN IN THE BROWSER

    The alternative was shipping the raw frame plus box coordinates and
    drawing on a canvas in the GCS. That costs nothing on the vehicle, but the
    overlay and the frame arrive on different paths and drift apart under
    load: boxes that lag the picture make an operator distrust a feed that is
    actually fine.

    Drawing server-side means a box is always on the frame it describes. It
    also costs less than what it replaces: three annotated encodes become one,
    with the per-detector streams kept for debugging but throttled and
    published only when something is subscribed.

WHY IT DOES NOT RE-DETECT

    It draws what the detectors reported, in image coordinates, from their
    detail topics. A node that re-ran detection could disagree with the
    detector whose decision the mission is actually flying on -- an overlay
    that contradicts the mission is worse than no overlay.

Topics
  sub  <image_topic>            sensor_msgs/Image
  sub  /percep/qr/detail        std_msgs/String
  sub  /percep/banner/detail    std_msgs/String
  sub  /percep/redzone/detail   std_msgs/String
  sub  /camera/pose_state       std_msgs/String
  pub  /percep/overlay          sensor_msgs/Image
"""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

GREEN = (0, 255, 0)
AMBER = (0, 180, 255)
RED = (0, 0, 255)
GREY = (170, 170, 170)


class Overlay(Node):
    def __init__(self):
        super().__init__("perception_overlay")
        p = self.declare_parameter
        p("image_topic", "/camera/image")
        # A detection older than this is not drawn. Without it a box from the
        # last frame the detector managed to process sits on the picture after
        # the object has left it, which is exactly the lie this node exists to
        # avoid.
        p("max_box_age_s", 0.7)
        p("draw_status_bar", True)

        self.bridge = CvBridge()
        self.boxes = {}          # source -> (stamp, [box, ...])
        self.status = {}         # source -> short text for the status bar
        self.camera_pose = ""

        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/percep/qr/detail",
                                 lambda m: self._on_detail("qr", m), 10)
        self.create_subscription(String, "/percep/banner/detail",
                                 lambda m: self._on_detail("banner", m), 10)
        self.create_subscription(String, "/percep/redzone/detail",
                                 lambda m: self._on_detail("redzone", m), 10)
        self.create_subscription(String, "/camera/pose_state",
                                 self._on_pose, 10)

        self.pub = self.create_publisher(Image, "/percep/overlay", 5)
        self.get_logger().info(f"perception_overlay up; image_topic={topic}")

    # ---- inputs ---- #
    def _on_pose(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.camera_pose = str(d.get("named", d.get("pose", "")))

    def _on_detail(self, source, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.boxes[source] = (time.time(), list(d.get("boxes") or []))
        self.status[source] = self._summarise(source, d)

    @staticmethod
    def _summarise(source, d):
        if source == "qr":
            if d.get("matched"):
                return f"QR {str(d.get('accepted', ''))[-8:]} MATCH"
            if d.get("accepted"):
                return f"QR {str(d.get('accepted', ''))[-8:]}"
            return "QR -"
        if source == "banner":
            if d.get("identified"):
                txt = d.get("text") or ""
                return f"BANNER {txt}" if txt else "BANNER"
            return "banner -"
        if source == "redzone":
            return f"RED {d.get('status', '-')}"
        return ""

    def fresh(self, source, now=None):
        """Boxes from `source` if they are recent enough to still be true."""
        now = time.time() if now is None else now
        stamp, boxes = self.boxes.get(source, (0.0, []))
        if now - stamp > float(self.get_parameter("max_box_age_s").value):
            return []
        return boxes

    # ---- drawing ---- #
    def draw(self, frame, now=None):
        for source in ("redzone", "banner", "qr"):
            for box in self.fresh(source, now):
                colour = GREEN if box.get("ok") else (
                    RED if source == "redzone" else AMBER)
                label = str(box.get("label", ""))[:28]
                if "quad" in box:
                    pts = np.array(box["quad"], dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(frame, [pts], True, colour, 2)
                    origin = tuple(int(v) for v in box["quad"][0])
                elif "rect" in box:
                    x, y, w, h = (int(v) for v in box["rect"])
                    cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)
                    origin = (x, y)
                else:
                    continue
                if label:
                    cv2.putText(frame, label,
                                (origin[0], max(12, origin[1] - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

        if bool(self.get_parameter("draw_status_bar").value):
            bits = [self.status.get(s) for s in ("qr", "banner", "redzone")]
            text = "  |  ".join(b for b in bits if b)
            if self.camera_pose:
                text = f"[{self.camera_pose}]  {text}"
            if text:
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 22), (0, 0, 0), -1)
                cv2.putText(frame, text[:110], (6, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREY, 1)
        return frame

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        out = self.draw(frame.copy())
        try:
            stamped = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            stamped.header = msg.header
            self.pub.publish(stamped)
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = Overlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
