#!/usr/bin/env python3
"""Phase 1a — measure the QR decode envelope from the SIMULATED camera.

WHY THIS SHAPE
    The obvious experiment ("does it decode at 10 m?") does not transfer. The
    simulated start pad is 2.2 m across and the delivery pads are 3.0 m; a real
    competition QR is far more likely to be 0.3-0.6 m. An altitude answer
    measured against a 2.2 m marker says nothing useful about the real one.

    What transfers is PIXELS PER QR MODULE:

        ground_width(h) = 2 * h * tan(hfov/2)
        px_per_metre(h) = image_width / ground_width(h)
        px_per_module   = px_per_metre(h) * marker_size / modules_per_side

    Measure the px/module at which decoding falls over, and the maximum
    stand-off for ANY marker size follows:

        h_max = image_width * marker_size
                / (2 * tan(hfov/2) * modules * px_per_module_threshold)

    That number is what Phase 6 needs to replace the hardcoded
    search_alt = 10.0 (see docs/GEOMETRY_AUDIT.md A2).

METHOD
    Fly the aircraft to a series of altitudes directly above the start pad with
    the camera confirmed NADIR (Phase 2's camera_ctrl), sample N frames at each
    altitude, and decode each with the same cv2.QRCodeDetector the mission uses.

    Then, at a fixed altitude, translate laterally so the marker sits off-axis,
    to check oblique/edge-of-frame behaviour.

    Prerequisite: scripts/launch_level6_sim.sh running.

        python3 sim/measure_qr_decode.py
        python3 sim/measure_qr_decode.py --alts 3,5,8,10,12 --frames 25
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

import cv2
from cv_bridge import CvBridge

# World geometry (mission2.sdf). Start pad centre sits 1 m ahead of the spawn
# point, so in the MAVROS local ENU frame (origin = home = spawn) it is +1 x.
START_PAD_LOCAL_XY = (1.0, 0.0)
START_PAD_SIZE_M = 2.2
QR_MODULES = 33                 # version 4, ECC M — from qr_matrices.json
EXPECTED_PAYLOAD = "AEROTHON2026:M2:TARGET_A"

# Real-world marker sizes to project the measured threshold onto.
CANDIDATE_REAL_SIZES_M = (0.30, 0.40, 0.50, 0.60, 1.00, 2.20)


def px_per_module(h, image_w, hfov_rad, marker_m, modules):
    if h <= 0:
        return float("inf")
    ground_w = 2.0 * h * math.tan(hfov_rad / 2.0)
    return (image_w / ground_w) * marker_m / modules


def max_altitude_for(px_thresh, image_w, hfov_rad, marker_m, modules):
    return (image_w * marker_m) / (2.0 * math.tan(hfov_rad / 2.0)
                                   * modules * px_thresh)


class QRMeasure(Node):
    def __init__(self):
        super().__init__("qr_decode_measure")
        self.state = State()
        self.pose = PoseStamped()
        self.cam_state = None
        self.info = None
        self.frame = None
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self._sp = None

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/camera/pose_state",
                                 self._on_cam, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 lambda m: setattr(self, "info", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image", self._on_image,
                                 qos_profile_sensor_data)

        self.pub_sp = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_cam = self.create_publisher(String, "/camera/set_pose", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.create_timer(0.1, self._stream)

    def _on_cam(self, m):
        try:
            self.cam_state = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def _on_image(self, m):
        self.frame = m

    def _stream(self):
        if self._sp is not None and self.state.armed:
            self._sp.header.stamp = self.get_clock().now().to_msg()
            self._sp.header.frame_id = "map"
            self.pub_sp.publish(self._sp)

    def goto(self, x, y, z):
        sp = PoseStamped()
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        sp.pose.orientation.w = 1.0
        self._sp = sp

    def spin(self, s):
        end = time.time() + s
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, pred, timeout, what=""):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        if what:
            print(f"    TIMEOUT waiting for {what}")
        return False

    def call(self, cli, req, timeout=5.0):
        fut = cli.call_async(req)
        end = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def wait_for_subscriber(self, pub, timeout=15.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if pub.get_subscription_count() > 0:
                self.spin(0.2)
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def set_camera(self, pose, timeout=15.0):
        self.wait_for_subscriber(self.pub_cam)
        for _ in range(6):
            self.pub_cam.publish(String(data=pose))
            self.spin(0.1)
        return self.wait_for(
            lambda: bool(self.cam_state
                         and self.cam_state.get("requested") == pose
                         and self.cam_state.get("settled")
                         and not self.cam_state.get("stale")),
            timeout, f"camera {pose}")

    def alt(self):
        return self.pose.pose.position.z

    def save_frame(self, path):
        if self.frame is None:
            return False
        try:
            img = self.bridge.imgmsg_to_cv2(self.frame, desired_encoding="bgr8")
        except Exception:
            return False
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, img)
        return True

    def sample_decodes(self, n_frames, settle_s=1.5):
        """Return (decoded_count, total, mean_marker_px, sample_payload)."""
        self.spin(settle_s)
        ok = 0
        total = 0
        widths = []
        payload_seen = ""
        seen_stamps = set()
        deadline = time.time() + max(20.0, n_frames * 1.5)
        while total < n_frames and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            m = self.frame
            if m is None:
                continue
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            if key in seen_stamps:
                continue
            seen_stamps.add(key)
            total += 1
            try:
                img = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
            except Exception:
                continue
            retval, decoded, points, _ = self.detector.detectAndDecodeMulti(img)
            if retval and points is not None:
                for text, quad in zip(decoded, points):
                    if not text:
                        continue
                    ok += 1
                    payload_seen = text
                    xs = quad[:, 0]
                    ys = quad[:, 1]
                    widths.append(max(xs.max() - xs.min(), ys.max() - ys.min()))
                    break
        mean_px = sum(widths) / len(widths) if widths else 0.0
        return ok, total, mean_px, payload_seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alts", default="3,5,7,10,13,16")
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--offaxis-alt", type=float, default=7.0)
    ap.add_argument("--out", default="docs/qr_decode_envelope.csv")
    ap.add_argument("--marker-size", type=float, default=None,
                    help="start pad edge length in metres; must match the "
                         "AEROTHON_START_QR_M the world was materialised with")
    ap.add_argument("--save-frames", default="",
                    help="directory to save one raw frame per station "
                         "(look at the image before theorising about why a "
                         "detector failed)")
    args = ap.parse_args()
    alts = [float(a) for a in args.alts.split(",")]
    marker_m = args.marker_size if args.marker_size is not None else \
        float(os.environ.get("AEROTHON_START_QR_M", START_PAD_SIZE_M))

    rclpy.init()
    n = QRMeasure()

    print("=" * 78)
    print(" PHASE 1a — QR DECODE ENVELOPE (simulated camera)")
    print("=" * 78)

    if not n.wait_for(lambda: n.state.connected, 60, "FCU"):
        return 2
    if not n.wait_for(lambda: n.info is not None, 30, "camera_info"):
        return 2
    if not n.wait_for(lambda: n.pose.header.stamp.sec != 0, 90, "local position"):
        return 2

    image_w = n.info.width
    fx = n.info.k[0]
    hfov = 2.0 * math.atan(image_w / (2.0 * fx)) if fx else math.radians(60.0)
    print(f"\ncamera: {image_w}x{n.info.height}  fx={fx:.1f}  "
          f"hfov={math.degrees(hfov):.1f} deg")
    print(f"marker: {marker_m} m across, {QR_MODULES} modules/side")

    print("\n[1] GUIDED + arm + takeoff")
    n.call(n.cli_mode, SetMode.Request(custom_mode="GUIDED"))
    n.wait_for(lambda: n.state.mode == "GUIDED", 20, "GUIDED")
    for _ in range(10):
        n.call(n.cli_arm, CommandBool.Request(value=True))
        n.spin(1.0)
        if n.state.armed:
            break
    if not n.state.armed:
        print("  could not arm"); return 1
    n.call(n.cli_takeoff, CommandTOL.Request(altitude=float(alts[0])))
    n.wait_for(lambda: n.alt() > alts[0] - 0.7, 90, "takeoff")

    print("[2] camera NADIR")
    if not n.set_camera("NADIR"):
        print("  camera did not confirm NADIR — aborting measurement")
        n.call(n.cli_land, CommandTOL.Request())
        return 1
    print(f"  confirmed ({n.cam_state.get('error_deg', 0):.2f} deg error)")

    rows = []
    px, py = START_PAD_LOCAL_XY

    print(f"\n[3] Altitude sweep over the start pad, {args.frames} frames each\n")
    print(f"{'alt(m)':>7} {'px/module':>10} {'decoded':>9} {'rate':>7} "
          f"{'marker px':>10} {'payload ok':>11}")
    for h in alts:
        n.goto(px, py, h)
        reached = n.wait_for(
            lambda: abs(n.alt() - h) < 0.6
            and math.dist((n.pose.pose.position.x, n.pose.pose.position.y),
                          (px, py)) < 0.7,
            60, f"altitude {h} m")
        if not reached:
            print(f"{h:7.1f}  (could not reach station)")
            continue
        ok, total, mean_px, payload = n.sample_decodes(args.frames)
        if args.save_frames:
            n.save_frame(os.path.join(args.save_frames, f"nadir_{h:.0f}m.jpg"))
        rate = ok / total if total else 0.0
        theo = px_per_module(h, image_w, hfov, marker_m, QR_MODULES)
        good = "yes" if payload == EXPECTED_PAYLOAD else ("-" if not payload else "WRONG")
        print(f"{h:7.1f} {theo:10.2f} {ok:4d}/{total:<4d} {rate:6.0%} "
              f"{mean_px:10.1f} {good:>11}")
        rows.append({"kind": "altitude", "alt_m": round(h, 2),
                     "px_per_module": round(theo, 3), "decoded": ok,
                     "frames": total, "rate": round(rate, 3),
                     "marker_px": round(mean_px, 1), "payload": payload})

    print(f"\n[4] Off-axis at {args.offaxis_alt:.0f} m "
          f"(marker pushed toward frame edge)\n")
    ground_half = args.offaxis_alt * math.tan(hfov / 2.0)
    print(f"{'offset(m)':>10} {'~frac of half-FOV':>18} {'decoded':>9} {'rate':>7}")
    for frac in (0.0, 0.25, 0.5, 0.7):
        off = ground_half * frac
        n.goto(px + off, py, args.offaxis_alt)
        if not n.wait_for(lambda: math.dist(
                (n.pose.pose.position.x, n.pose.pose.position.y),
                (px + off, py)) < 0.7, 60, f"offset {off:.1f} m"):
            continue
        ok, total, mean_px, payload = n.sample_decodes(args.frames)
        rate = ok / total if total else 0.0
        print(f"{off:10.2f} {frac:18.2f} {ok:4d}/{total:<4d} {rate:6.0%}")
        rows.append({"kind": "offaxis", "alt_m": args.offaxis_alt,
                     "offset_m": round(off, 2), "fov_frac": frac,
                     "decoded": ok, "frames": total, "rate": round(rate, 3),
                     "marker_px": round(mean_px, 1), "payload": payload})

    print("\n[5] Land")
    n._sp = None
    n.call(n.cli_land, CommandTOL.Request())
    n.wait_for(lambda: not n.state.armed, 90, "disarm")

    # ---- analysis -------------------------------------------------------- #
    alt_rows = [r for r in rows if r["kind"] == "altitude"]
    reliable = [r for r in alt_rows if r["rate"] >= 0.9]
    threshold = min((r["px_per_module"] for r in reliable), default=None)

    print("\n" + "=" * 78)
    print(" RESULT")
    print("=" * 78)
    if threshold is None:
        print(" No altitude reached a 90% decode rate — see the table above.")
    else:
        worst = max(alt_rows, key=lambda r: r["alt_m"] if r["rate"] >= 0.9 else -1)
        print(f" Reliable (>=90%) down to {threshold:.2f} px/module "
              f"(highest good altitude {worst['alt_m']:.0f} m at "
              f"{marker_m} m marker)")
        print("\n Maximum nadir stand-off implied for a REAL marker:\n")
        print(f"   {'marker (m)':>12} {'max altitude (m)':>18}")
        for s in CANDIDATE_REAL_SIZES_M:
            hm = max_altitude_for(threshold, image_w, hfov, s, QR_MODULES)
            print(f"   {s:12.2f} {hm:18.1f}")
        print("\n This is the number Phase 6 needs instead of search_alt = 10.0")
        print(" (docs/GEOMETRY_AUDIT.md A2). Confirm the competition marker size")
        print(" with the organisers before committing a search altitude.")

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if rows:
        keys = sorted({k for r in rows for k in r})
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n wrote {out}")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
