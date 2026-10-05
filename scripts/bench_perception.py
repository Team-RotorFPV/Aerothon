#!/usr/bin/env python3
"""Time every detector on the flight computer, per 1280x720 frame.

    python3 scripts/bench_perception.py          # on the Pi 5, stack stopped

Run it on the Pi 5 itself: a laptop core says nothing about an A76. The
camera gate (camera_ctrl.gate) runs only the detectors that can see anything
at the camera's pose, so what has to fit in a frame is one pose's set, and
the table totals each: NADIR runs the QR reader, the red-zone mapper and the
payload finder; FORWARD runs the banner check. They are separate nodes,
so on four cores they overlap: the per-pose total is the one-core bound,
and the slowest single row is what the camera rate actually waits on.

A slower set is not a crash: the sweep sets its speed from the measured
frame rate (red ground has to be confirmed in 4 frames before the aircraft
reaches it), and the banner dwell counts frames. It is a slower mission, so
TARGET_HZ is what the 15-minute budget was planned at; below it, expect
the search to take longer (docs/FIELD_READINESS.md).
"""

import os
import sys
import time

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))
for pkg in ("perception_qr", "perception_banner", "perception_redzone", "camera_ctrl"):
    sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_perception", pkg))

import rclpy                                                # noqa: E402
from cv_bridge import CvBridge                              # noqa: E402

import test_perception_corruption as T                      # noqa: E402
from perception_banner.banner_node import BannerNode        # noqa: E402
from perception_qr.qr_decode import QrDecoder               # noqa: E402
from perception_redzone.payload import detect_payload       # noqa: E402
from perception_redzone.redzone_node import RedZoneNode     # noqa: E402

TARGET_HZ = {"NADIR": 10.0, "FORWARD": 8.0}


def ms_per_frame(fn, img, n):
    fn(img)
    t = time.perf_counter()
    for _ in range(n):
        fn(img)
    return 1000.0 * (time.perf_counter() - t) / n


def main():
    rclpy.init()
    bridge = CvBridge()
    grass = T.ground()
    pad = T.CameraCorruptor(T.WORST, seed=0).apply(T.render_pad(1.0, 5.0))
    banner = cv2.imread(os.path.join(ROOT, "tests", "fixtures",
                                     "banner_sim_ambient_3m.png"))
    qr = QrDecoder()
    bn = BannerNode()
    bn.pub.publish = bn.pub_detail.publish = bn.pub_annot.publish = lambda m: None
    rz = RedZoneNode()
    rz._pose, rz._hfov = (0.0, 0.0, 10.0, 0.0), T.HFOV
    msg = lambda i: bridge.cv2_to_imgmsg(i, "bgr8")        # noqa: E731

    rows = {
        "NADIR": [
            ("image message -> array", lambda i: bridge.imgmsg_to_cv2(msg(i), "bgr8"), grass),
            ("QR, a worn pad in view", qr.read, pad),
            ("QR, bare grass", qr.read, grass),
            ("red-zone mask", rz.red_mask, grass),
            ("payload finder", detect_payload, grass),
        ],
        "FORWARD": [
            ("image message -> array", lambda i: bridge.imgmsg_to_cv2(msg(i), "bgr8"), banner),
            ("banner check, banner in view", lambda i: bn.on_image(msg(i)), banner),
        ],
    }
    worst_ok = True
    for pose, items in rows.items():
        print(f"\n{pose}")
        total = 0.0
        for name, fn, img in items:
            ms = ms_per_frame(fn, img, 10 if "banner" in name else 30)
            # The two QR rows are alternatives: a frame is one or the other.
            if name != "QR, a worn pad in view":
                total += ms
            print(f"  {name:32s} {ms:7.1f} ms")
        hz = 1000.0 / total if total else float("inf")
        ok = hz >= TARGET_HZ[pose]
        worst_ok &= ok
        print(f"  {'one frame, worst case':32s} {total:7.1f} ms -> {hz:5.1f} Hz "
              f"(target {TARGET_HZ[pose]:.0f}) {'OK' if ok else 'SLOW'}")
    bn.destroy_node()
    rz.destroy_node()
    rclpy.try_shutdown()
    return 0 if worst_ok else 1


if __name__ == "__main__":
    sys.exit(main())
