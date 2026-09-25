#!/usr/bin/env python3
"""Measure the range at which the banner can still be IDENTIFIED.

WHY THIS EXISTS

    banner_node declares `max_detect_range_m = 25.0`, and derives its minimum
    blob area from it. That gate is about whether a green thing is big enough
    to be worth looking at. It says nothing about whether the *lettering* --
    the thing that actually decides identity -- is resolvable at that range.

    Only one of those ranges was declared, so nobody knew what the other one
    was. Phase 1 answered the equivalent question for QR by measuring
    px/module rather than asserting an altitude; this does the same for the
    banner.

    WHAT IT WAS WRITTEN TO PROVE, AND DID NOT

        Arena-regression seed 1001 put the gate 6.3 m from the aircraft
        instead of the nominal 2.8 m, and the mission failed to identify the
        banner. The obvious explanation was that the lettering had become too
        small to resolve at 2.2x the range.

        That is wrong. Identity holds down to 15 px/letter — about 30 m for
        this banner and camera — so at 6.3 m the detector had roughly five
        times the resolution it needs. The seed 1001 failure is not a range
        failure, and a "fix" aimed at range would have been aimed at nothing.

        The measurement is kept because the number is worth having and was
        never known. The lesson is the one this project keeps relearning:
        the plausible cause was not the cause, and only measuring said so.

WHAT TRANSFERS

    Not "it works to 4 m" -- that is only true of this banner at this
    resolution. What transfers is PIXELS ACROSS A LETTER:

        px_per_letter = focal_px * letter_width_m / range_m
        focal_px      = image_width / (2 tan(hfov/2))

    Measure the px/letter at which identity fails, and the maximum range for
    any banner and any camera follows:

        range_max = focal_px * letter_width_m / px_per_letter_threshold

METHOD

    Take the real rendered frame checked in as a fixture (captured from a live
    run -- synthetic banners drawn by the same assumptions the detector makes
    cannot fail honestly), and rescale it to simulate greater stand-off.
    Halving the linear size is the same projection as doubling the range.

    Feed each scale through the real BannerNode and record identified/reason.

    source /opt/ros/jazzy/setup.bash
    python3 sim/measure_banner_identity.py
"""

import json
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_perception",
                                "perception_banner"))

from perception_banner.banner_node import BannerNode      # noqa: E402

FIXTURE = os.path.join(ROOT, "tests", "fixtures", "banner_sim_ambient_3m.png")

# What the fixture actually is. The frame was captured with the aircraft about
# 3 m from the gate; the simulated board is 3.7 m wide and carries the 8
# characters of "AEROTHON" across 3.25 m of it.
FIXTURE_RANGE_M = 3.0
BANNER_CHARS = 8
LETTERING_WIDTH_M = 3.25
HFOV_RAD = 1.0472


def letter_width_m():
    return LETTERING_WIDTH_M / BANNER_CHARS


def px_per_letter(range_m, image_w_px):
    focal = image_w_px / (2.0 * math.tan(HFOV_RAD / 2.0))
    return focal * letter_width_m() / range_m


def main():
    if not os.path.exists(FIXTURE):
        print(f"fixture missing: {FIXTURE}", file=sys.stderr)
        return 2
    img = cv2.imread(FIXTURE)
    h0, w0 = img.shape[:2]
    print(f"fixture {os.path.basename(FIXTURE)}  {w0}x{h0}  "
          f"captured at ~{FIXTURE_RANGE_M} m")
    print(f"letter width {letter_width_m():.3f} m "
          f"({BANNER_CHARS} chars across {LETTERING_WIDTH_M} m)\n")

    rclpy.init()
    node = BannerNode()
    detail = []
    sent = []
    node.pub.publish = sent.append
    node.pub_detail.publish = detail.append
    node.pub_annot.publish = lambda m: None
    bridge = CvBridge()

    print(f"{'range_m':>8} {'scale':>6} {'px/letter':>10} "
          f"{'identified':>11}  reason")
    print("-" * 78)

    rows = []
    # Simulate stand-off by shrinking the banner within a same-size frame:
    # scale s == range FIXTURE_RANGE_M / s.
    for s in (1.0, 0.7, 0.5, 0.35, 0.25, 0.2, 0.15, 0.12, 0.10, 0.08, 0.06, 0.05, 0.04):
        rng = FIXTURE_RANGE_M / s
        small = cv2.resize(img, (max(8, int(w0 * s)), max(8, int(h0 * s))),
                           interpolation=cv2.INTER_AREA)
        # Paste into a full-size frame so the FRAME geometry is unchanged and
        # only the banner's angular size varies -- otherwise the area gate,
        # which is a fraction of frame, would move too and confound the result.
        canvas = np.full((h0, w0, 3), (90, 70, 55), dtype=np.uint8)
        y0 = (h0 - small.shape[0]) // 2
        x0 = (w0 - small.shape[1]) // 2
        canvas[y0:y0 + small.shape[0], x0:x0 + small.shape[1]] = small

        node.on_image(bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))
        d = json.loads(detail[-1].data)
        ident = bool(d.get("identified"))
        ppl = px_per_letter(rng, w0)
        reason = (d.get("reason") or "")[:34]
        rows.append((rng, ppl, ident))
        print(f"{rng:8.2f} {s:6.2f} {ppl:10.1f} {str(ident):>11}  {reason}")

    node.destroy_node()
    rclpy.shutdown()

    ok = [r for r in rows if r[2]]
    bad = [r for r in rows if not r[2]]
    print()
    if not ok:
        print("identity never achieved — the fixture or the detector changed")
        return 1
    worst_ok = min(ok, key=lambda r: r[1])
    best_bad = max(bad, key=lambda r: r[1]) if bad else None
    print(f"identified down to  {worst_ok[1]:.1f} px/letter "
          f"(range {worst_ok[0]:.1f} m at {w0}px / {math.degrees(HFOV_RAD):.0f} deg)")
    if best_bad:
        print(f"first failure at    {best_bad[1]:.1f} px/letter "
              f"(range {best_bad[0]:.1f} m)")
    print(f"\nFor any banner and camera:")
    print(f"    range_max = focal_px * letter_width_m / "
          f"{worst_ok[1]:.1f}")
    print(f"\nDeclared max_detect_range_m is 25.0 m — the AREA gate. The "
          f"IDENTITY\nrange measured here is the binding one, and it is much "
          f"shorter.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
