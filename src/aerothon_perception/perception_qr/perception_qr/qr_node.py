#!/usr/bin/env python3
"""QR detection, target matching and plausibility gating (zbar + OpenCV, qr_decode.py).

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
         z = 1.0 matched target | 0.5 some marker | 0.25 a marker located
             but not decoded (blur, vibration) | 0.0 nothing visible
  pub  /percep/qr/detail          std_msgs/String   JSON diagnostics
  pub  /percep/qr/annotated       sensor_msgs/Image
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Bool

from camera_ctrl.gate import CameraGate
from perception_qr.qr_decode import QrDecoder, zbar_available
from perception_qr import qr_appearance
from perception_qr.qr_match import APPEARANCE_ONLY, matches

UNREAD = 0.25      # mission_bt.decode_hover.UNREAD


class QrNode(Node):
    def __init__(self):
        super().__init__('perception_qr')
        p = self.declare_parameter
        p('image_topic', '/image_raw')
        p('camera_info_topic', '/camera/camera_info')
        p('target', '')
        # ---- appearance matching, the PRIMARY evidence ---- #
        # The organisers have not settled whether the codes encode anything,
        # and the requirement is to match the marker by how it looks. The
        # start marker's own pattern becomes the reference; a pad whose
        # pattern agrees with it is the target, decoded or not.
        p('appearance_matching', True)
        p('appearance_threshold', qr_appearance.DEFAULT_THRESHOLD)
        p('appearance_coarse_threshold', qr_appearance.COARSE_THRESHOLD)
        p('match_mode', 'identifiers')   # or 'exact'; see qr_match.py
        # False discards every decoded payload: the organisers' codes may
        # carry nothing, and this flies that case on codes that do.
        p('use_payloads', True)
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
        self.decoder = QrDecoder()
        # The start marker, as a pattern. Captured during the start scan --
        # the only time a marker is in frame before any target exists -- and
        # cleared when a new run starts so one flight's reference cannot
        # identify the next flight's pad.
        self._reference = None
        self._reference_px = 0.0
        self._frame_i = 0
        self._alt = None
        self._fx = None

        self.create_subscription(Image, image_topic, self.on_image, 5)
        # Markers lie on the ground: read them looking down.
        self.gate = CameraGate(self, ("NADIR", "ALIGN"))
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
        if not zbar_available():
            self.get_logger().warn(
                "zbar not found (apt install libzbar0; pip install pyzbar): "
                "decoding with OpenCV alone, which loses markers to motion "
                "blur and vibration that zbar reads")

    def _on_info(self, m: CameraInfo):
        # CameraInfo.k is a numpy array, so `if m.k` raises
        # "truth value of an array with more than one element is ambiguous"
        # and kills the node on the first CameraInfo message. Check length.
        if len(m.k) >= 1 and float(m.k[0]) > 0.0:
            self._fx = float(m.k[0])

    def on_target(self, msg: String):
        # A run that names a new target starts a new search. Keeping
        # the previous flight's reference would let one flight's start
        # marker identify the next flight's pad.
        # The mission publishes an empty target at every START, so each
        # flight captures its own start marker.
        if not msg.data:
            self._reference, self._reference_px = None, 0.0
        self.target = msg.data
        self.get_logger().info(f"target set -> '{self.target}'")

    def _appearance(self, frame, quad, marker_px):
        """This marker's pattern, and the start reference if none is held yet.

        The start scan is the one moment a marker is in frame before any
        target exists, so the largest marker seen then IS the start marker.
        Capturing it there -- rather than being handed a reference -- keeps
        this to what the aircraft actually saw.
        """
        if not bool(self.get_parameter('appearance_matching').value):
            return None
        seen = qr_appearance.signature(frame, quad)
        if seen is None:
            return None
        held = self._reference
        # A module-grid reading beats a coarse one at any size: the two kinds
        # cannot be compared, and pads are read as module grids.
        better = held is None or (
            (seen.kind == "modules") > (held.kind == "modules")) or (
            seen.kind == held.kind and marker_px > self._reference_px)
        if not self.target and better:
            self._reference, self._reference_px = seen, marker_px
            self.get_logger().info(
                f"start marker pattern captured ({marker_px:.0f} px)")
        return seen

    def _looks_like_target(self, seen):
        """(is_match, score) for a pattern against the captured start marker."""
        if seen is None or self._reference is None:
            return False, 0.0
        # During the start scan the reference IS this marker; a marker cannot
        # identify itself as the delivery target.
        if not self.target:
            return False, 0.0
        return qr_appearance.same_marker(
            self._reference, seen,
            float(self.get_parameter('appearance_threshold').value),
            float(self.get_parameter('appearance_coarse_threshold').value))

    def _comparable(self, seen):
        """Both patterns read as module grids of one size: the look decides."""
        ref = self._reference
        return (seen is not None and ref is not None and seen.kind == "modules"
                and ref.kind == "modules" and seen.n == ref.n)

    def _matches(self, payload: str) -> bool:
        if self.target == APPEARANCE_ONLY:
            return False
        if self.match_mode == 'exact':
            return bool(self.target) and payload == self.target
        return matches(self.target, payload)

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
        processing_started = time.perf_counter()
        capture = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if capture>0 and capture<=getattr(self,'_last_capture',-1):
            return
        self._last_capture = capture
        self._frame_i += 1
        if self._frame_i % self.process_every:
            return
        if not self.gate.open():
            # Nothing in view, said explicitly: a stale match or offset must
            # not outlive the view it came from.
            self.pub_offset.publish(Vector3())
            self.pub_decoded.publish(String(data=""))
            self.pub_matched.publish(Bool(data=False))
            self.pub_detail.publish(String(data=json.dumps({
                'capture_stamp': capture,
                'processed_stamp': self.get_clock().now().nanoseconds*1e-9,
                'accepted': '', 'matched': False, 'offset': [0,0,0],
                'status': 'NOT_VISIBLE', 'reason': 'camera pose gate is closed',
                'rejected': [], 'boxes': [], 'unread': False,
                'processing_wall_s': time.perf_counter()-processing_started,
            })))
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]
        accepted = ""
        matched = False
        basis = []
        detail_look = 0.0
        offset = Vector3(x=0.0, y=0.0, z=0.0)
        rejected = []
        best = None            # (is_match, payload, cx, cy, px)

        decoded, located = self.decoder.read(frame)
        if not bool(self.get_parameter('use_payloads').value):
            decoded = [("", quad) for _, quad in decoded]

        require = bool(self.get_parameter('require_plausible').value)
        # The annotated copy is for a GCS debug view; nobody subscribed, no
        # drawing and no 2.7 MB conversion.
        annotate = self.pub_annot.get_subscription_count() > 0

        # Where things are, in IMAGE coordinates, so one overlay stream can
        # draw every detector's findings on the live camera frame without
        # re-running detection and possibly disagreeing with it.
        boxes = []
        for info, quad in decoded:
            marker_px = self._marker_px(quad)
            good, why = self.plausible(marker_px)
            # APPEARANCE FIRST, payload second. A code that encodes nothing
            # still looks like itself, so the pattern is the evidence the
            # rulebook asks for and the payload is the confirmation.
            seen = self._appearance(frame, quad, marker_px)
            by_look, look_score = self._looks_like_target(seen)
            detail_look = max(detail_look, look_score)
            by_payload = self._matches(info)
            # A pad that READS as another place is not the target, however
            # alike it looks: a decoded contradiction vetoes the pattern.
            contradicted = (by_look and not by_payload and bool(info)
                            and bool(self.target)
                            and self.target != APPEARANCE_ONLY)
            # And a payload that only SHARES identifiers with the start code
            # ("ZONE 2 PAD 1" against a pad "2") does not outvote two module
            # grids that plainly differ. An identical payload still counts.
            if (by_payload and not by_look
                    and info.strip() != self.target.strip()
                    and self._comparable(seen)):
                by_payload = False
            is_match = bool((by_look and not contradicted) or by_payload)
            if is_match:
                basis.append("appearance" if by_look else "payload")
            quad_i = quad.astype(int)

            if not good and require:
                rejected.append({"payload": info[:32], "px": round(marker_px, 1),
                                 "why": why})
                if annotate:
                    cv2.polylines(frame, [quad_i], True, (0, 0, 255), 2)
                    cv2.putText(frame, "IMPLAUSIBLE", tuple(quad_i[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                boxes.append({"quad": quad_i.tolist(),
                              "label": "IMPLAUSIBLE", "ok": False})
                continue

            if annotate:
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

        unread = None
        if located is not None:
            quad = located
            marker_px = self._marker_px(quad)
            if self.plausible(marker_px)[0]:
                unread = quad
                # A marker nothing could decode is exactly the case appearance
                # matching exists for: judge its pattern anyway.
                seen = self._appearance(frame, quad, marker_px)
                by_look, look_score = self._looks_like_target(seen)
                detail_look = max(detail_look, look_score)
                if by_look:
                    matched = True
                    basis.append("appearance")
                    cx, cy = quad.mean(axis=0)
                    best = best or (True, "", float(cx), float(cy), marker_px)
                boxes.append({"quad": quad.astype(int).tolist(),
                              "label": "UNREAD", "ok": False})
                if annotate:
                    cv2.polylines(frame, [quad.astype(int)], True, (0, 255, 255), 2)

        if best is not None:
            matched, accepted, cx, cy, marker_px = best
            offset.x = float((cx - w / 2) / (w / 2))
            offset.y = float((cy - h / 2) / (h / 2))
            # z tells the consumer WHAT the offset refers to, so a centring
            # controller can act on a marker before the target is known.
            offset.z = 1.0 if matched else 0.5
        elif unread is not None:
            cx, cy = unread.mean(axis=0)
            offset.x = float((cx - w / 2) / (w / 2))
            offset.y = float((cy - h / 2) / (h / 2))
            offset.z = UNREAD

        # WHERE before WHAT: the mission ends its sweep on `matched`, and a
        # match handled before its own offset left it with no ground fix for
        # the pad (team airframe, flight 4: matched at the frame edge, stopped,
        # never saw the pad again). Cross-topic order is not guaranteed, so the
        # sweep also waits for the fix; this makes the wait the rare case.
        self.pub_offset.publish(offset)
        self.pub_decoded.publish(String(data=accepted))
        self.pub_matched.publish(Bool(data=matched))

        rng = self.expected_px_range()
        self.pub_detail.publish(String(data=json.dumps({
            "capture_stamp": capture,
            "processed_stamp": self.get_clock().now().nanoseconds*1e-9,
            "processing_wall_s": time.perf_counter()-processing_started,
            "processing_latency_s": max(0,self.get_clock().now().nanoseconds*1e-9-capture),
            "accepted": accepted,
            "matched": matched,
            "target": self.target,
            "marker_px": round(best[4], 1) if best else 0.0,
            "offset": [round(offset.x, 3), round(offset.y, 3), offset.z],
            "alt": None if self._alt is None else round(self._alt, 2),
            "expect_px": [round(rng[0], 1), round(rng[1], 1)] if rng else None,
            "rejected": rejected,
            "unread": unread is not None,
            # WHICH evidence identified it. Nothing may claim a decode it did
            # not get: "appearance" means the pattern matched the start
            # marker, "payload" means a decoded string did.
            "match_basis": sorted(set(basis)),
            "appearance_score": round(detail_look, 4),
            "reference_captured": self._reference is not None,
            "reference_px": round(self._reference_px, 1),
            "reference_kind": None if self._reference is None else self._reference.kind,
            "reference_n": None if self._reference is None else self._reference.n,
            "boxes": boxes,
        })))

        if annotate:
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
