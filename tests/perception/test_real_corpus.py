#!/usr/bin/env python3
"""Phase 1b — regression harness for REAL photographs of the markers.

WHY A REAL CORPUS
    Every perception number measured in Gazebo is measured against a rendered
    marker with no lens, no sensor noise, no motion blur, no sun. Those numbers
    tell you the geometry is right; they do not tell you the Brio will decode
    anything on the day. This corpus is the ground truth the simulator is
    checked against.

    It also decides Phase 6's search altitude. The sim start pad is 2.2 m
    across; the competition marker is very unlikely to be. What transfers is
    pixels-per-module, and only real photographs pin down the px/module at
    which a real camera stops decoding.

HOW TO POPULATE IT
    Photographs go in tests/perception/corpus/, one file per condition, named:

        qr_<size_mm>_<distance_cm>_<angle_deg>_<lighting>_<n>.jpg
        banner_<distance_cm>_<angle_deg>_<lighting>_<n>.jpg

    e.g.  qr_400_300_0_sun_01.jpg     400 mm marker, 3.00 m, head-on, sunlight
          qr_400_500_30_overcast_02.jpg
          banner_800_15_indoor_01.jpg

    lighting is one of: indoor, overcast, sun, shade, dusk

    Expected QR payloads live in corpus/expected.json:
        {"qr_400_300_0_sun_01.jpg": "AEROTHON2026:M2:TARGET_A"}
    A file with no entry is decoded and reported but not asserted.

    See docs/CORPUS_SHOT_LIST.md for exactly which photographs to take.

WHAT IT ASSERTS
    Nothing, until photographs exist — an absent corpus SKIPS rather than
    fails, so this file is safe to have in the tree before the shoot. Once
    populated it asserts per-condition decode rates and reports the envelope.

    python3 -m pytest tests/perception/test_real_corpus.py -v -s
"""

import json
import os
import re
import unittest
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CORPUS = os.path.join(HERE, "corpus")
EXPECTED_JSON = os.path.join(CORPUS, "expected.json")

QR_RE = re.compile(
    r"^qr_(?P<size_mm>\d+)_(?P<dist_cm>\d+)_(?P<angle>\d+)_(?P<light>[a-z]+)_(?P<n>\d+)\.(jpg|jpeg|png)$",
    re.IGNORECASE)
BANNER_RE = re.compile(
    r"^banner_(?P<dist_cm>\d+)_(?P<angle>\d+)_(?P<light>[a-z]+)_(?P<n>\d+)\.(jpg|jpeg|png)$",
    re.IGNORECASE)

# Head-on decode rate we require before trusting a condition in flight.
MIN_DECODE_RATE = 0.9
# Green-banner segmentation must find a plausible blob in this fraction of
# frames per lighting condition.
MIN_BANNER_DETECT_RATE = 0.9

QR_MODULES_DEFAULT = 33


def corpus_files():
    if not os.path.isdir(CORPUS):
        return [], []
    names = sorted(os.listdir(CORPUS))
    qr = [(n, m.groupdict()) for n in names if (m := QR_RE.match(n))]
    banner = [(n, m.groupdict()) for n in names if (m := BANNER_RE.match(n))]
    return qr, banner


def load_expected():
    if os.path.isfile(EXPECTED_JSON):
        with open(EXPECTED_JSON) as f:
            return json.load(f)
    return {}


def px_per_module(marker_px, modules=QR_MODULES_DEFAULT):
    return marker_px / modules if modules else 0.0


def detect_green_banner(img, s_lo=90, v_lo=60):
    """Mirror of perception_banner's gate: HSV green + morphology + area."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([35, s_lo, v_lo]), np.array([85, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    c = max(contours, key=cv2.contourArea)
    area_frac = cv2.contourArea(c) / (img.shape[0] * img.shape[1])
    x, y, w, h = cv2.boundingRect(c)
    return {"area_frac": area_frac, "bbox": (x, y, w, h),
            "aspect": (w / h) if h else 0.0}, mask


class RealCorpusTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.qr_files, cls.banner_files = corpus_files()
        cls.expected = load_expected()
        cls.detector = cv2.QRCodeDetector()

    def _skip_if_empty(self, files, kind):
        if not files:
            self.skipTest(
                f"no {kind} photographs in {CORPUS} yet — see "
                f"docs/CORPUS_SHOT_LIST.md. This SKIP is expected until the "
                f"corpus is shot; it is not a pass.")

    # ------------------------------------------------------------------ #
    def test_qr_decode_rate_by_condition(self):
        self._skip_if_empty(self.qr_files, "QR")

        buckets = defaultdict(list)
        for name, meta in self.qr_files:
            img = cv2.imread(os.path.join(CORPUS, name))
            self.assertIsNotNone(img, f"unreadable image {name}")
            retval, decoded, points, _ = self.detector.detectAndDecodeMulti(img)
            text = ""
            marker_px = 0.0
            if retval and points is not None:
                for t, quad in zip(decoded, points):
                    if t:
                        text = t
                        xs, ys = quad[:, 0], quad[:, 1]
                        marker_px = max(xs.max() - xs.min(), ys.max() - ys.min())
                        break
            want = self.expected.get(name)
            ok = bool(text) and (want is None or text == want)
            key = (meta["size_mm"], meta["dist_cm"], meta["angle"], meta["light"])
            buckets[key].append((ok, marker_px, text, want, name))

        print(f"\n{'size(mm)':>9} {'dist(cm)':>9} {'angle':>6} {'light':>9} "
              f"{'rate':>6} {'px/module':>10}")
        failures = []
        for key in sorted(buckets):
            rows = buckets[key]
            rate = sum(1 for r in rows if r[0]) / len(rows)
            pxm = [px_per_module(r[1]) for r in rows if r[1] > 0]
            mean_pxm = sum(pxm) / len(pxm) if pxm else 0.0
            print(f"{key[0]:>9} {key[1]:>9} {key[2]:>6} {key[3]:>9} "
                  f"{rate:6.0%} {mean_pxm:10.2f}")
            # Only head-on shots are held to the bar; oblique ones are
            # characterisation, not a requirement.
            if key[2] == "0" and rate < MIN_DECODE_RATE:
                failures.append(f"{key}: {rate:.0%} < {MIN_DECODE_RATE:.0%}")

        wrong = [r for rows in buckets.values() for r in rows
                 if r[2] and r[3] and r[2] != r[3]]
        self.assertFalse(wrong, f"decoded the WRONG payload: {wrong[:3]}")
        self.assertFalse(failures,
                         "head-on conditions below the decode bar: " + "; ".join(failures))

    def test_banner_detection_by_lighting(self):
        self._skip_if_empty(self.banner_files, "banner")

        buckets = defaultdict(list)
        for name, meta in self.banner_files:
            img = cv2.imread(os.path.join(CORPUS, name))
            self.assertIsNotNone(img, f"unreadable image {name}")
            det, _ = detect_green_banner(img)
            buckets[meta["light"]].append((det is not None, det, name))

        print(f"\n{'lighting':>10} {'detect rate':>12} {'mean area frac':>15}")
        failures = []
        for light in sorted(buckets):
            rows = buckets[light]
            rate = sum(1 for r in rows if r[0]) / len(rows)
            areas = [r[1]["area_frac"] for r in rows if r[1]]
            mean_area = sum(areas) / len(areas) if areas else 0.0
            print(f"{light:>10} {rate:12.0%} {mean_area:15.4f}")
            if rate < MIN_BANNER_DETECT_RATE:
                failures.append(f"{light}: {rate:.0%}")
        self.assertFalse(failures,
                         "lighting conditions below the banner bar: " + "; ".join(failures))

    def test_report_px_per_module_envelope(self):
        """Report the px/module at which real decoding falls over.

        This is the number that converts to a search altitude for any marker
        size, and the one Phase 6 needs (docs/GEOMETRY_AUDIT.md A2).
        """
        self._skip_if_empty(self.qr_files, "QR")

        good, bad = [], []
        for name, meta in self.qr_files:
            if meta["angle"] != "0":
                continue
            img = cv2.imread(os.path.join(CORPUS, name))
            retval, decoded, points, _ = self.detector.detectAndDecodeMulti(img)
            marker_px = 0.0
            text = ""
            if retval and points is not None:
                for t, quad in zip(decoded, points):
                    if t:
                        text = t
                        xs, ys = quad[:, 0], quad[:, 1]
                        marker_px = max(xs.max() - xs.min(), ys.max() - ys.min())
                        break
            if text and marker_px:
                good.append(px_per_module(marker_px))
            elif not text:
                bad.append(name)

        if good:
            print(f"\nreal-image decode succeeded down to "
                  f"{min(good):.2f} px/module (n={len(good)})")
            print(f"failed frames: {len(bad)}")
            print("\nMax nadir stand-off implied for a real marker at that "
                  "threshold, 640px / 60 deg HFOV / 33 modules:")
            thresh = min(good)
            for size_m in (0.30, 0.40, 0.50, 0.60, 1.00):
                import math
                h = (640 * size_m) / (2 * math.tan(math.radians(30)) * 33 * thresh)
                print(f"   {size_m:.2f} m marker -> {h:5.1f} m")
        else:
            print("\nno successful head-on decodes in the corpus")


if __name__ == "__main__":
    unittest.main(verbosity=2)
