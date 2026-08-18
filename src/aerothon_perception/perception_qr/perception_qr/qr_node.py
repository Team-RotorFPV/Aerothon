#!/usr/bin/env python3
"""QR detection, target matching and plausibility gating (OpenCV QRCodeDetector).

Two jobs in Mission 2:
  1. START QR  — decode the delivery target string during the start scan.
  2. TARGET QR — during the delivery-zone sweep, report whether the currently
     visible QR matches that target, and where it is in frame.

PHASE 3 CHANGES, and why

  * OFFSET FOR ANY MARKER. /percep/qr/target_offset used to be populated only
    when the payload matched the target. During the start scan there IS no
    target yet — that is what the scan is for — so the offset stayed at zero
    and the mission had no way to know the marker was off to one side. It could
    only hover over a hardcoded guess and hope. Offset is now published for the
    best-visible marker, with z encoding what it refers to.

  * PLAUSIBILITY GATING. A decode was previously accepted regardless of how
    big the marker appeared. A reflection, a QR on a passing phone screen, or a
    distant marker in another part of the arena would all be taken at face
    value and could set the delivery target. The apparent size is now checked
    against what any plausible physical marker could subtend at the current
    altitude; implausible reads are reported but not accepted.

    The competition marker size is unconfirmed, so the gate is deliberately a
    RANGE (min_marker_m .. max_marker_m) rather than a single expected size.
    Narrow it once the organisers answer — see docs/QR_DECODE_ENVELOPE.md.

Topics
  sub  <image_topic>              sensor_msgs/Image
  sub  /mission/target            std_msgs/String   set/override target payload
  sub  /mavros/local_position/pose  geometry_msgs/PoseStamped  (altitude)
  sub  <camera_info_topic>        sensor_msgs/CameraInfo     (intrinsics)
  pub  /percep/qr/decoded         std_msgs/String   accepted payload ("" if none)
  pub  /percep/qr/matched         std_msgs/Bool     target currently visible
  pub  /percep/qr/target_offset   geometry_msgs/Vector3
         x,y in [-1,1] from image centre (right+, down+)
         z = 1.0 matched target | 0.5 some marker | 0.0 nothing visible
  pub  /percep/qr/detail          std_msgs/String   JSON diagnostics
  pub  /percep/qr/annotated       sensor_msgs/Image
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Bool


class QrNode(Node):
    def __init__(self):
        super().__init__('perception_qr')
        p = self.declare_parameter
        p('image_topic', '/image_raw')
        p('camera_info_topic', '/camera/camera_info')
        p('target', '')
        p('match_mode', 'exact')
        p('process_every', 1)
        # Plausibility bounds on the PHYSICAL marker. Wide on purpose: the
        # competition size is unconfirmed. This rejects nonsense, not detail.
        p('min_marker_m', 0.15)
        p('max_marker_m', 3.50)
        p('plausibility_tolerance', 1.6)   # multiplicative slack either way
        p('require_plausible', True)

        image_topic = self.get_parameter('image_topic').value
        self.target = self.get_parameter('target').value
        self.match_mode = self.get_parameter('match_mode').value
        self.process_every = max(1, int(self.get_parameter('process_every').value))

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self._frame_i = 0
        self._alt = None
        self._fx = None

        self.create_subscription(Image, image_topic, self.on_image, 5)
        self.create_subscription(String, '/mission/target', self.on_target, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda m: setattr(self, '_alt', m.pose.position.z),
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, qos_profile_sensor_data)

        self.pub_decoded = self.create_publisher(String, '/percep/qr/decoded', 10)
        self.pub_matched = self.create_publisher(Bool, '/percep/qr/matched', 10)
        self.pub_offset = self.create_publisher(Vector3, '/percep/qr/target_offset', 10)
        self.pub_detail = self.create_publisher(String, '/percep/qr/detail', 10)
        self.pub_annot = self.create_publisher(Image, '/percep/qr/annotated', 5)

        self.get_logger().info(
            f"perception_qr up; image_topic={image_topic} target='{self.target}'")

    def _on_info(self, m: CameraInfo):
        # CameraInfo.k is a numpy array, so `if m.k` raises
        # "truth value of an array with more than one element is ambiguous"
        # and kills the node on the first CameraInfo message. Check length.
        if len(m.k) >= 1 and float(m.k[0]) > 0.0:
            self._fx = float(m.k[0])

    def on_target(self, msg: String):
        self.target = msg.data
        self.get_logger().info(f"target set -> '{self.target}'")

    def _matches(self, payload: str) -> bool:
        if not self.target or not payload:
            return False
        if self.match_mode == 'substring':
            return self.target in payload or payload in self.target
        return payload == self.target

    # ------------------------------------------------------------------ #
    def expected_px_range(self):
        """Apparent marker width, in pixels, for any plausible marker size.

        A marker of side S metres, viewed from height h with focal length fx,
        subtends fx * S / h pixels. Returns None when altitude or intrinsics
        are unknown, in which case the gate cannot be applied.
        """
        if self._alt is None or self._fx is None or self._alt <= 0.3:
            return None
        tol = float(self.get_parameter('plausibility_tolerance').value)
        lo = self._fx * float(self.get_parameter('min_marker_m').value) / self._alt
        hi = self._fx * float(self.get_parameter('max_marker_m').value) / self._alt
        return (lo / tol, hi * tol)

    def plausible(self, marker_px):
        """(ok, reason). Unknown altitude means the gate abstains, not fails."""
        rng = self.expected_px_range()
        if rng is None:
            return True, "no altitude/intrinsics; gate abstains"
        lo, hi = rng
        if marker_px < lo:
            return False, (f"marker {marker_px:.0f}px too small for any plausible "
                           f"marker at {self._alt:.1f}m (expect {lo:.0f}-{hi:.0f}px)")
        if marker_px > hi:
            return False, (f"marker {marker_px:.0f}px too large for any plausible "
                           f"marker at {self._alt:.1f}m (expect {lo:.0f}-{hi:.0f}px)")
        return True, ""

    @staticmethod
    def _marker_px(quad):
        xs, ys = quad[:, 0], quad[:, 1]
        return float(max(xs.max() - xs.min(), ys.max() - ys.min()))

    # ------------------------------------------------------------------ #
    def on_image(self, msg: Image):
        self._frame_i += 1
        if self._frame_i % self.process_every:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]
        accepted = ""
        matched = False
        offset = Vector3(x=0.0, y=0.0, z=0.0)
        rejected = []
        best = None            # (is_match, payload, cx, cy, px)

        try:
            ok, infos, points, _ = self.detector.detectAndDecodeMulti(frame)
        except cv2.error:
            ok, infos, points = False, [], None

        require = bool(self.get_parameter('require_plausible').value)

        # Where things are, in IMAGE coordinates, so one overlay stream can
        # draw every detector's findings on the live camera frame without
        # re-running detection and possibly disagreeing with it.
        boxes = []
        if ok and points is not None:
            for info, quad in zip(infos, points):
                if not info:
                    continue
                marker_px = self._marker_px(quad)
                good, why = self.plausible(marker_px)
                is_match = self._matches(info)
                quad_i = quad.astype(int)

                if not good and require:
                    rejected.append({"payload": info[:32], "px": round(marker_px, 1),
                                     "why": why})
                    cv2.polylines(frame, [quad_i], True, (0, 0, 255), 2)
                    cv2.putText(frame, "IMPLAUSIBLE", tuple(quad_i[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    boxes.append({"quad": quad_i.tolist(),
                                  "label": "IMPLAUSIBLE", "ok": False})
                    continue

                colour = (0, 255, 0) if is_match else (0, 180, 255)
                cv2.polylines(frame, [quad_i], True, colour, 2)
                cv2.putText(frame, info[:24], tuple(quad_i[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
                boxes.append({"quad": quad_i.tolist(), "label": info[:24],
                              "ok": bool(is_match)})

                cx, cy = quad.mean(axis=0)
                cand = (is_match, info, float(cx), float(cy), marker_px)
                # Prefer the matching target; otherwise the largest marker.
                if best is None or (is_match and not best[0]) or \
                        (is_match == best[0] and marker_px > best[4]):
                    best = cand

        if best is not None:
            matched, accepted, cx, cy, marker_px = best
            offset.x = float((cx - w / 2) / (w / 2))
            offset.y = float((cy - h / 2) / (h / 2))
            # z tells the consumer WHAT the offset refers to, so a centring
            # controller can act on a marker before the target is known.
            offset.z = 1.0 if matched else 0.5

        self.pub_decoded.publish(String(data=accepted))
        self.pub_matched.publish(Bool(data=matched))
        self.pub_offset.publish(offset)

        rng = self.expected_px_range()
        self.pub_detail.publish(String(data=json.dumps({
            "accepted": accepted,
            "matched": matched,
            "target": self.target,
            "marker_px": round(best[4], 1) if best else 0.0,
            "offset": [round(offset.x, 3), round(offset.y, 3), offset.z],
            "alt": None if self._alt is None else round(self._alt, 2),
            "expect_px": [round(rng[0], 1), round(rng[1], 1)] if rng else None,
            "rejected": rejected,
            "boxes": boxes,
        })))

        try:
            self.pub_annot.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = QrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
