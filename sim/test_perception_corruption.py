#!/usr/bin/env python3
"""Every detector, under what a real camera does to a picture.

Gazebo renders a perfect pinhole image. These tests put the camera
corruptions of sim_gazebo/corruptions.py (ImageNet-C style, severity 1-5)
between the scene and each detector the mission depends on, and hold each
one to the envelope measured when they were written:

    QR (zbar + OpenCV cascade)   every corruption to severity 2, on every
                                 marker geometry flown; the markers it cannot
                                 read at severity 3-5 (motion blur, vibration,
                                 defocus) it can still LOCATE, which is what
                                 makes the sweep stop over them
    banner identity              every corruption at every severity
    red zones                    every corruption to severity 4 (glare washes
                                 out part of a zone, never all); and a QR
                                 code printed in red ink -- the rulebook's own
                                 Figure 3 draws them -- is not a red zone

QR scenes are rendered exactly: the real module matrices, projected through
the C270's field of view from the flight altitudes, supersampled 4x. The
banner scene is a frame the simulated camera actually captured
(tests/fixtures/banner_sim_ambient_3m.png).

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_perception_corruption.py -v
"""

import json
import math
import os
import sys
import unittest

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
for pkg in (("aerothon_sim", "sim_gazebo"), ("aerothon_perception", "perception_qr"),
            ("aerothon_perception", "perception_banner"),
            ("aerothon_perception", "perception_redzone")):
    sys.path.insert(0, os.path.join(ROOT, "src", *pkg))

from perception_banner.banner_node import BannerNode        # noqa: E402
from perception_qr.qr_decode import QrDecoder               # noqa: E402
from perception_redzone.redzone_node import RedZoneNode     # noqa: E402
from sim_gazebo.corruptions import CAMERA_KEYS, CameraCorruptor  # noqa: E402
import world_spec                                           # noqa: E402

# Every camera corruption of the "worst" conditions preset, at once.
WORST = {k: v for k, v in world_spec.conditions(
    {"conditions": {"preset": "worst"}})["camera"].items() if k in CAMERA_KEYS}

W, H, HFOV = 1280, 720, 0.851919          # the team airframe's C270
FX = (W / 2) / math.tan(HFOV / 2)
MATRICES = json.load(open(os.path.join(
    ROOT, "src", "aerothon_sim", "sim_gazebo", "materials", "qr_matrices.json")))
PAYLOAD = "AEROTHON2026:M2:TARGET_C"
MATRIX = MATRICES["qr_target_c.png"]
# (marker edge m, altitude m): the simulated 3 m pad from the 10 m search
# altitude, a 1 m pad from the 5 m drop altitude, a 0.6 m sheet from 3 m.
GEOMETRIES = ((3.0, 10.0), (1.0, 5.0), (0.6, 3.0))
TRIALS = 4
GRASS = (70, 120, 95)
RED_INK = (40, 40, 170)


def ground(seed=1):
    """Grass with low-frequency texture, as the field is not a flat colour."""
    rng = np.random.default_rng(seed)
    tex = cv2.resize(rng.integers(-18, 18, (H // 8, W // 8)).astype(np.int16), (W, H))
    return np.clip(np.full((H, W, 3), GRASS, np.int16) + tex[..., None],
                   0, 255).astype(np.uint8)


def render_pad(size_m, alt_m, off=(0.0, 0.0), rot=0.3, ink=(5, 5, 5), ss=4):
    """A nadir view of one delivery pad, `off` metres from frame centre."""
    n = len(MATRIX)
    img = cv2.resize(ground(), (W * ss, H * ss), interpolation=cv2.INTER_NEAREST)
    cell = size_m * FX / alt_m * ss / n
    cx = W * ss / 2 + off[0] * FX / alt_m * ss
    cy = H * ss / 2 + off[1] * FX / alt_m * ss
    c, s = math.cos(rot), math.sin(rot)

    def quad(u0, v0, u1, v1):
        pts = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        return np.array([(cx + (u - n / 2) * cell * c - (v - n / 2) * cell * s,
                          cy + (u - n / 2) * cell * s + (v - n / 2) * cell * c)
                         for u, v in pts], np.int32)

    cv2.fillPoly(img, [quad(0, 0, n, n)], (250, 250, 250))
    for r, row in enumerate(MATRIX):
        for q, dark in enumerate(row):
            if dark:
                cv2.fillPoly(img, [quad(q, r, q + 1, r + 1)], ink)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def corrupted_pads(key, sev, size_m, alt_m):
    """TRIALS views of the pad, each placed differently, under one condition."""
    for t in range(TRIALS):
        yield CameraCorruptor({key: sev}, seed=t).apply(
            render_pad(size_m, alt_m, off=(0.1 * t, 0.05 * t), rot=0.2 + 0.3 * t))


def conditions(max_sev):
    for key in CAMERA_KEYS:
        for sev in range(1, max_sev + 1):
            yield key, sev
            if key == "exposure":
                yield key, -sev


class QrEnvelopeTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.dec = QrDecoder()

    def reads(self, img):
        return any(p == PAYLOAD for p, _ in self.dec.read(img)[0])

    def test_every_corruption_to_severity_2_reads_on_every_geometry(self):
        for size, alt in GEOMETRIES:
            for key, sev in conditions(2):
                with self.subTest(size=size, alt=alt, key=key, sev=sev):
                    got = sum(self.reads(i) for i in corrupted_pads(key, sev, size, alt))
                    self.assertEqual(got, TRIALS)

    def test_noise_haze_exposure_jpeg_and_dust_read_at_every_severity(self):
        for size, alt in GEOMETRIES:
            for key in ("noise", "haze", "jpeg", "lens_dust"):
                for sev in range(3, 6):
                    with self.subTest(size=size, alt=alt, key=key, sev=sev):
                        got = sum(self.reads(i) for i in corrupted_pads(key, sev, size, alt))
                        self.assertGreaterEqual(got, TRIALS - 1)

    def test_what_blur_defeats_can_still_be_located(self):
        """The sweep stops over a located marker; stopping ends motion blur.

        Severity 4 motion blur is a 22 px smear: at the sweep's 2 m/s that
        is an 80 ms exposure, which the capped exposure of the flight camera
        (docs/FIELD_READINESS.md) never reaches; half the frames still
        locate it."""
        for key, sev, need in (("motion_blur", 3, TRIALS),
                               ("motion_blur", 4, TRIALS // 2),
                               ("vibration", 4, TRIALS), ("vibration", 5, TRIALS),
                               ("defocus", 4, TRIALS), ("defocus", 5, TRIALS)):
            with self.subTest(key=key, sev=sev):
                located = sum(self.dec.locate(i) is not None
                              for i in corrupted_pads(key, sev, 3.0, 10.0))
                self.assertGreaterEqual(located, need)

    def test_bare_ground_is_almost_never_taken_for_a_marker(self):
        """A false 'unreadable marker' costs the sweep one hold. OpenCV's
        finder search alone found one in grass in a frame in five, and in sun
        glare; the squareness and print checks leave well under 1%."""
        frames = located = 0
        for key, sev in conditions(5):
            for seed in range(4):
                img = CameraCorruptor({key: sev}, seed=seed).apply(ground(seed))
                frames += 1
                located += self.dec.locate(img) is not None
                self.assertEqual(self.dec.read(img)[0], [])
        self.assertLess(located / frames, 0.01)

    def test_the_rulebook_ink_colours_read(self):
        """Figure 3 prints the delivery codes in teal, red, purple and blue."""
        for name, ink in (("teal", (150, 140, 20)), ("red", RED_INK),
                          ("purple", (140, 40, 120)), ("blue", (170, 60, 20))):
            for size, alt in GEOMETRIES:
                with self.subTest(ink=name, size=size, alt=alt):
                    img = CameraCorruptor({"noise": 2, "motion_blur": 2}, seed=1).apply(
                        render_pad(size, alt, ink=ink))
                    self.assertTrue(self.reads(img))


class WorstDayQrTests(unittest.TestCase):
    """All of the worst preset's corruptions together, frame by frame. A
    frame that does not read is either read on the next (the sweep holds
    over a located marker) -- so what matters is that nearly every frame
    either reads or locates."""

    def test_each_geometry_reads_or_locates(self):
        dec = QrDecoder()
        for size, alt in GEOMETRIES:
            with self.subTest(size=size, alt=alt):
                read = seen = 0
                for t in range(8):
                    img = CameraCorruptor(WORST, seed=t).apply(render_pad(
                        size, alt, off=(0.1 * t, 0.05 * t), rot=0.2 + 0.3 * t))
                    r = any(p == PAYLOAD for p, _ in dec.read(img)[0])
                    read += r
                    seen += r or dec.locate(img) is not None
                self.assertGreaterEqual(read, 5)
                self.assertGreaterEqual(seen, 7)


class _RosCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()


class BannerEnvelopeTests(_RosCase):

    def setUp(self):
        self.node = BannerNode()
        self.detail = []
        self.node.pub.publish = lambda m: None
        self.node.pub_detail.publish = self.detail.append
        self.node.pub_annot.publish = lambda m: None
        self.bridge = CvBridge()
        self.frame = cv2.imread(os.path.join(ROOT, "tests", "fixtures",
                                             "banner_sim_ambient_3m.png"))

    def tearDown(self):
        self.node.destroy_node()

    def identified(self, img):
        self.detail.clear()
        self.node.on_image(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        return json.loads(self.detail[-1].data)["identified"]

    def test_every_corruption_at_every_severity(self):
        """Haze, half exposure and heavy noise used to lose it: the colour
        gates saw a washed-out or dark board. photometry.normalise fixed it."""
        self.assertTrue(self.identified(self.frame))
        for key, sev in conditions(5):
            with self.subTest(key=key, sev=sev):
                got = sum(self.identified(CameraCorruptor({key: sev}, seed=t).apply(
                    self.frame)) for t in range(3))
                self.assertEqual(got, 3)

    def test_the_worst_day_all_at_once(self):
        got = sum(self.identified(CameraCorruptor(WORST, seed=t).apply(self.frame))
                  for t in range(4))
        self.assertEqual(got, 4)


class RedZoneEnvelopeTests(_RosCase):

    ALT = 10.0

    def setUp(self):
        self.node = RedZoneNode()
        self.node._pose = (0.0, 0.0, self.ALT, 0.0)
        self.node._hfov = HFOV

    def tearDown(self):
        self.node.destroy_node()

    def zone_scene(self, side_m, lettered=True):
        img = ground()
        truth = np.zeros((H, W), np.uint8)
        half = int(side_m * FX / self.ALT / 2)
        box = ((W // 2 - half, H // 2 - half), (W // 2 + half, H // 2 + half))
        cv2.rectangle(img, *box, (12, 12, 230), -1)
        cv2.rectangle(truth, *box, 255, -1)
        if lettered:        # the painted RED ZONE lettering across it
            cv2.rectangle(img, (W // 2 - half + 20, H // 2 - 12),
                          (W // 2 + half - 20, H // 2 + 12), (245, 245, 245), -1)
        return img, truth

    def test_every_corruption_to_severity_4_finds_the_zone_and_nothing_else(self):
        """Glare washes part of a zone out (half of it at severity 5); the
        rest must be found whole. Nothing is found away from the zone --
        blur smearing its own edge outward is the zone, not a false one."""
        img, truth = self.zone_scene(6.0)
        near = cv2.dilate(truth, np.ones((71, 71), np.uint8)) > 0   # 0.5 m ring
        for key, sev in conditions(4):
            with self.subTest(key=key, sev=sev):
                mask = self.node.red_mask(CameraCorruptor({key: sev}, seed=0).apply(img)) > 0
                floor = 0.6 if key == "glare" else 0.8
                self.assertGreaterEqual(mask[truth > 0].mean(), floor)
                self.assertLess(mask[~near].mean(), 0.001)

    def test_the_worst_day_all_at_once(self):
        img, truth = self.zone_scene(6.0)
        for t in range(3):
            mask = self.node.red_mask(CameraCorruptor(WORST, seed=t).apply(img)) > 0
            self.assertGreaterEqual(mask[truth > 0].mean(), 0.8)

    def test_a_small_zone_is_still_a_zone(self):
        for side in (1.0, 2.0):
            with self.subTest(side=side):
                img, truth = self.zone_scene(side, lettered=False)
                self.assertGreaterEqual((self.node.red_mask(img)[truth > 0] > 0).mean(), 0.9)

    def test_a_QR_code_in_red_ink_is_not_a_red_zone(self):
        """Its modules and finder squares were confirmed as restricted ground
        over the very pad the mission had to deliver to."""
        for size, alt in ((3.0, 10.0), (5.0, 10.0), (1.0, 5.0), (1.0, 3.0)):
            with self.subTest(size=size, alt=alt):
                self.node._pose = (0.0, 0.0, alt, 0.0)
                mask = self.node.red_mask(render_pad(size, alt, ink=RED_INK))
                self.assertLess(np.count_nonzero(mask) / mask.size,
                                float(self.node._g("min_area_frac")))


if __name__ == "__main__":
    unittest.main()
