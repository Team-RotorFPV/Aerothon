#!/usr/bin/env python3
"""Phase 4 — the banner gate must reject things that are merely green.

WHAT WAS WRONG
    perception_banner accepted the largest green blob that passed an area and
    aspect gate, and published "banner". Grass, a tarpaulin or a green vehicle
    roof would all have qualified — and the corridor entry heading was to be
    derived from it, while the GCS displayed ALIGNED.

    Green decoys now exist in the simulated world (materialize_world.py,
    green_decoy_visuals) so that "reject non-banners" is a claim that can
    actually fail. These tests synthesise the same situations directly.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_banner_identity.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_perception", "perception_banner"))

import cv2
import numpy as np
import rclpy

from perception_banner.banner_node import BannerNode

GREEN = (60, 170, 60)          # BGR, banner green
WHITE = (245, 245, 245)


def blank(w=640, h=480):
    return np.full((h, w, 3), (90, 70, 55), dtype=np.uint8)


def green_rect(img, x, y, w, h):
    cv2.rectangle(img, (x, y), (x + w, y + h), GREEN, -1)
    return img


def with_lettering(img, x, y, w, h, n_letters=8):
    """Green board with separate white blocks in a horizontal band."""
    cv2.rectangle(img, (x + 6, y + 4), (x + w - 6, y + 10), WHITE, -1)   # frame
    cv2.rectangle(img, (x + 6, y + h - 10), (x + w - 6, y + h - 4), WHITE, -1)
    gap = w // (n_letters + 2)
    for i in range(n_letters):
        lx = x + gap + i * gap
        cv2.rectangle(img, (lx, y + h // 3), (lx + gap // 2, y + 2 * h // 3),
                      WHITE, -1)
    return img


class BannerIdentityTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BannerNode()
        self.sent = []
        self.detail = []
        self.node.pub.publish = self.sent.append
        self.node.pub_detail.publish = self.detail.append
        self.node.pub_annot.publish = lambda m: None

    def tearDown(self):
        self.node.destroy_node()

    def feed(self, img):
        from cv_bridge import CvBridge
        self.node.on_image(CvBridge().cv2_to_imgmsg(img, encoding="bgr8"))
        import json
        return self.sent[-1], json.loads(self.detail[-1].data)

    # ---- the real banner ---- #

    def test_real_banner_is_identified(self):
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        with_lettering(img, 150, 180, 340, 120)
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0, f"real banner rejected: {d}")
        self.assertTrue(d["identified"])

    def test_banner_MOUNTED_ON_A_LARGER_GREEN_STRUCTURE_is_identified(self):
        """THE LIVE FAILURE. The gate and the corridor behind it are both
        green, so in the image the board is one small part of a large
        connected green blob. Measuring white content over the whole blob put
        the lettering far under the 2% floor and the real banner was rejected
        as "no white lettering" -- a run swept 180 degrees without ever
        identifying the gate it was looking straight at."""
        img = blank()
        green_rect(img, 20, 60, 600, 400)          # the whole gate + corridor
        with_lettering(img, 180, 200, 280, 100)    # the board, a small part
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0,
                         f"banner on a large green structure rejected: {d}")

    def test_a_LONG_THIN_green_container_does_not_veto_the_board(self):
        """THE LIVE PROBE. With the aspect gate on the green container the
        detector reported

            "green region aspect 17.40 outside 1.2-8.0"   candidates: 3

        for a frame containing the real banner. Aspect is a statement about
        the BOARD's proportions; the container is the gate plus the corridor
        behind it plus whatever green fence is attached."""
        img = blank()
        green_rect(img, 0, 190, 640, 70)           # aspect ~9.1 container
        with_lettering(img, 210, 195, 220, 60, n_letters=6)
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0,
                         f"a long thin green container vetoed the board: {d}")

    def test_an_ABSURDLY_shaped_board_is_still_rejected(self):
        """Moving the aspect gate must not delete it."""
        img = blank()
        green_rect(img, 300, 100, 40, 300)         # tall narrow board
        for i in range(4):                          # letters stacked vertically
            cv2.rectangle(img, (310, 130 + i * 60), (330, 160 + i * 60),
                          WHITE, -1)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0, f"a tall narrow board was accepted: {d}")

    def test_the_reported_bearing_is_the_BOARD_not_the_whole_green_blob(self):
        """Steering at the centroid of "all the green" aims at the corridor,
        not at the gate."""
        img = blank()
        green_rect(img, 20, 60, 600, 400)
        with_lettering(img, 400, 200, 200, 100)    # board well right of centre
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0, f"not identified: {d}")
        self.assertGreater(out.x, 0.15,
                           "bearing followed the green blob, not the board")

    # ---- decoys: green, banner-shaped, no lettering ---- #

    def test_blank_green_tarp_is_REJECTED(self):
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0,
                            "a blank green rectangle was accepted as the banner")
        self.assertEqual(out.z, 0.5, "should report green-but-not-banner")
        self.assertIn("white", d.get("reason", "").lower())

    def test_green_with_a_single_stripe_is_REJECTED(self):
        """One white stripe is not lettering."""
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        cv2.rectangle(img, (170, 230), (470, 250), WHITE, -1)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0, f"single stripe accepted: {d}")

    def test_grass_like_wide_green_region_is_REJECTED(self):
        img = blank()
        green_rect(img, 0, 300, 640, 180)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0, f"grass accepted as banner: {d}")

    def test_mostly_white_board_is_REJECTED(self):
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        cv2.rectangle(img, (155, 185), (485, 295), WHITE, -1)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0, f"white board accepted: {d}")

    # ---- nothing at all ---- #

    def test_a_large_green_structure_with_NO_lettering_is_still_REJECTED(self):
        """Finding the board inside a bigger blob must not become "accept any
        big green thing"."""
        img = blank()
        green_rect(img, 20, 60, 600, 400)
        out, d = self.feed(img)
        self.assertNotEqual(out.z, 1.0,
                            f"a large blank green structure was accepted: {d}")

    def test_a_rejection_always_says_WHY(self):
        """A candidate dropped on area or aspect used to leave the reason
        empty, so the panel showed "identified: false, reason: ''"."""
        img = blank()
        green_rect(img, 300, 220, 12, 8)          # far too small
        _, d = self.feed(img)
        self.assertTrue(d.get("reason"),
                        "a green region was rejected without a reason")

    def test_no_green_reports_nothing_visible(self):
        out, d = self.feed(blank())
        self.assertEqual(out.z, 0.0)
        self.assertFalse(d["identified"])

    def test_nothing_visible_is_distinct_from_rejected(self):
        """An operator must be able to tell 'no banner in view' from
        'something green in view that is not the banner'."""
        empty, _ = self.feed(blank())
        img = blank(); green_rect(img, 150, 180, 340, 120)
        decoy, _ = self.feed(img)
        self.assertEqual(empty.z, 0.0)
        self.assertEqual(decoy.z, 0.5)
        self.assertNotEqual(empty.z, decoy.z)

    # ---- bearing, for alignment ---- #

    def test_bearing_is_negative_when_banner_is_left_of_centre(self):
        img = blank()
        green_rect(img, 40, 180, 240, 120)
        with_lettering(img, 40, 180, 240, 120)
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0, f"not identified: {d}")
        self.assertLess(out.x, 0.0, "banner left of centre must give negative x")

    def test_bearing_is_positive_when_banner_is_right_of_centre(self):
        img = blank()
        green_rect(img, 380, 180, 240, 120)
        with_lettering(img, 380, 180, 240, 120)
        out, d = self.feed(img)
        self.assertEqual(out.z, 1.0, f"not identified: {d}")
        self.assertGreater(out.x, 0.0)

    def test_centred_banner_has_near_zero_bearing(self):
        img = blank()
        green_rect(img, 200, 180, 240, 120)
        with_lettering(img, 200, 180, 240, 120)
        out, _ = self.feed(img)
        self.assertLess(abs(out.x), 0.08)

    # ---- the gate can be turned off, deliberately and visibly ---- #

    def test_identity_gate_can_be_disabled_for_debugging(self):
        import rclpy.parameter
        self.node.set_parameters([rclpy.parameter.Parameter(
            'require_identity', rclpy.parameter.Parameter.Type.BOOL, False)])
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        out, _ = self.feed(img)
        self.assertEqual(out.z, 1.0,
                         "with the gate off, a green rectangle should pass")


if __name__ == "__main__":
    unittest.main(verbosity=2)


# --------------------------------------------------------------------------- #
class _Clock:
    """Injectable monotonic clock. The sweep dwells for seconds, and a suite
    that really waited five of them per heading would not be run."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class AlignToBannerTests(unittest.TestCase):
    """Alignment must act on an IDENTIFIED banner, and yaw the right way.

    This is what replaces corridor_entry = (5.0, 0.0, 3.0) (audit A5). Getting
    the yaw sign wrong here is the same class of error that flew the aircraft
    into a wall in Phase 2, so the direction is asserted.
    """

    def setUp(self):
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "src", "aerothon_mission", "mission_bt"))
        from mission_bt.mission_tree import AlignToBanner
        from geometry_msgs.msg import Vector3
        import py_trees

        class FakeMav:
            def __init__(self):
                self.banner = Vector3()
                self.abort_reason = ""
                self.gotos = []
                self._yaw = 0.0
                self.logs = []
                self.reject = ""
                # The stop-and-stare sweep waits for the AIRFRAME to reach the
                # heading it asked for, so a fake whose heading never moves
                # would stall in SETTLE forever and prove nothing.
                self.follow_yaw = True

            def pos(self):
                return (1.0, 2.0, 3.0)

            def yaw(self):
                return self._yaw

            def goto(self, x, y, z, yaw=0.0):
                self.gotos.append((x, y, z, yaw))
                if self.follow_yaw:
                    self._yaw = math.atan2(math.sin(yaw), math.cos(yaw))

            def banner_identified(self):
                return self.banner.z >= 1.0

            def banner_bearing(self):
                return self.banner.x if self.banner_identified() else 0.0

            def log(self, msg, warn=False):
                self.logs.append((msg, warn))

            def banner_rejection_summary(self, top=3):
                """What the detector actually said, so a failed sweep is
                diagnosable. See the arena-regression seed 1001 failure."""
                return self.reject or "nothing green ever entered the frame"

        self.py_trees = py_trees
        self.Vector3 = Vector3
        self.mav = FakeMav()
        self.clock = _Clock()
        self.AlignToBanner = AlignToBanner
        self.leaf = self._leaf(tol=0.1, yaw_step=0.5, timeout_ticks=30,
                               stable_frames=3)

    def _leaf(self, **kw):
        kw.setdefault("clock", self.clock)
        kw.setdefault("dwell_s", 0.6)
        return self.AlignToBanner(self.mav, **kw)

    def tick(self, leaf=None, n=1, dt=0.1):
        """One tick of wall clock as well as one tick of the tree.

        The sweep is now timed, not counted. A test that ticked without
        advancing a clock would sit in the first dwell forever.
        """
        leaf = leaf if leaf is not None else self.leaf
        for _ in range(n):
            leaf.tick_once()
            self.clock.advance(dt)
            if leaf.status is not self.py_trees.common.Status.RUNNING:
                break
        return leaf.status

    def test_green_but_not_banner_does_not_count_as_aligned(self):
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.5)   # decoy, centred
        self.tick(n=6)
        self.assertEqual(self.leaf.status, self.py_trees.common.Status.RUNNING,
                         "aligned to a green object that is not the banner")

    def _centre(self, bearing):
        """Get past the dwell so the fine-alignment yaw can be observed."""
        self.mav.banner = self.Vector3(x=bearing, y=0.0, z=1.0)
        self.tick(n=12)
        self.assertEqual(self.leaf.phase, self.leaf.CENTRE,
                         "the dwell never confirmed the banner")
        before = self.mav.yaw()
        self.leaf.tick_once()
        return before, self.mav.gotos[-1][3]

    def test_banner_right_of_centre_yaws_right(self):
        before, yaw = self._centre(0.5)
        self.assertLess(yaw, before,
                        "banner to the right must yaw right (negative in ENU)")

    def test_banner_left_of_centre_yaws_left(self):
        before, yaw = self._centre(-0.5)
        self.assertGreater(yaw, before)

    def test_centred_banner_held_for_several_frames_succeeds(self):
        self.mav.banner = self.Vector3(x=0.02, y=0.0, z=1.0)
        self.tick(n=20)
        self.assertEqual(self.leaf.status, self.py_trees.common.Status.SUCCESS)

    def test_one_centred_frame_is_not_enough(self):
        self.mav.banner = self.Vector3(x=0.02, y=0.0, z=1.0)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, self.py_trees.common.Status.RUNNING)

    def test_no_banner_eventually_FAILS_closed(self):
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.0)
        self.tick(n=600)
        self.assertEqual(self.leaf.status, self.py_trees.common.Status.FAILURE)
        self.assertIn("banner", self.mav.abort_reason.lower())


# --------------------------------------------------------------------------- #
class DistanceAwareAreaGateTests(unittest.TestCase):
    """min_area_frac = 0.01 was a hidden statement about range (audit E1).

    A fixed fraction of the frame means "a 2 m banner at about 21 m" at
    640x480/60deg — a range nobody chose. The gate is derived from the
    projection instead, so changing the camera or the detection range changes
    it correctly.
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BannerNode()

    def tearDown(self):
        self.node.destroy_node()

    def _set(self, **kw):
        import rclpy.parameter as rp
        types = {float: rp.Parameter.Type.DOUBLE, int: rp.Parameter.Type.INTEGER}
        self.node.set_parameters(
            [rp.Parameter(k, types[type(v)], v) for k, v in kw.items()])

    def test_gate_matches_the_projected_banner_area(self):
        """2 m x 1 m banner at 25 m, 640 px, 60 deg HFOV."""
        self._set(max_detect_range_m=25.0, camera_hfov=1.0472,
                  banner_width_m=2.0, banner_height_m=1.0, area_safety=1.0)
        gw = 2 * 25.0 * math.tan(1.0472 / 2)
        px_per_m = 640 / gw
        self.assertAlmostEqual(self.node.min_area_px(640, 480),
                               (2.0 * px_per_m) * (1.0 * px_per_m), places=2)

    def test_a_longer_detection_range_lowers_the_gate(self):
        self._set(max_detect_range_m=10.0, area_safety=1.0)
        near = self.node.min_area_px(640, 480)
        self._set(max_detect_range_m=40.0)
        far = self.node.min_area_px(640, 480)
        self.assertLess(far, near,
                        "wanting to see the banner further away must ACCEPT "
                        "smaller blobs, not larger ones")

    def test_gate_scales_with_resolution(self):
        """The same physical banner at the same range is more pixels at 1080p."""
        self._set(max_detect_range_m=25.0, area_safety=1.0)
        self.assertAlmostEqual(
            self.node.min_area_px(1920, 1080) / self.node.min_area_px(640, 480),
            9.0, places=3)

    def test_a_bigger_banner_raises_the_gate(self):
        self._set(banner_width_m=1.0, area_safety=1.0)
        small = self.node.min_area_px(640, 480)
        self._set(banner_width_m=4.0)
        self.assertAlmostEqual(self.node.min_area_px(640, 480) / small, 4.0,
                               places=3)

    def test_an_explicit_fraction_still_overrides(self):
        """A specific arena can pin it if the derivation disagrees."""
        self._set(min_area_frac=0.02)
        self.assertAlmostEqual(self.node.min_area_px(640, 480),
                               0.02 * 640 * 480, places=3)

    def test_safety_factor_admits_oblique_views(self):
        self._set(area_safety=1.0)
        ideal = self.node.min_area_px(640, 480)
        self._set(area_safety=0.5)
        self.assertAlmostEqual(self.node.min_area_px(640, 480), ideal / 2,
                               places=3)

    def test_the_default_gate_still_accepts_a_real_banner(self):
        """The derivation must not be so strict it rejects the thing itself."""
        img = blank()
        green_rect(img, 150, 180, 340, 120)
        with_lettering(img, 150, 180, 340, 120)
        sent, detail = [], []
        self.node.pub.publish = sent.append
        self.node.pub_detail.publish = detail.append
        self.node.pub_annot.publish = lambda m: None
        from cv_bridge import CvBridge
        self.node.on_image(CvBridge().cv2_to_imgmsg(img, encoding="bgr8"))
        self.assertEqual(sent[-1].z, 1.0,
                         f"derived area gate rejected the banner: {detail[-1].data}")



# --------------------------------------------------------------------------- #
class BannerSweepBoundTests(AlignToBannerTests):
    """The search sweep must be bounded, and it must not lock onto a decoy.

    THE LIVE FAILURE: with an unbounded sweep the aircraft turned 271 degrees
    looking for the gate, locked onto a distant green object to the south and
    flew away from the corridor.

    WHAT CHANGED, AND WHY THE BOUND IS NOW A FULL TURN

        The original reading of that failure was "it searched too far", and
        the fix was a half-turn limit. That reading was wrong, and the cost of
        it showed up later: a banner behind the start heading became
        unfindable, which is where the return leg of seed 1001 kept stranding.

        What actually went wrong at 271 degrees is that a continuous yaw acts
        on the FIRST frame that says yes, and over a long enough sweep
        something greenish always will. The protection is the confidence floor
        on a dwell -- most of a five-second stare has to agree -- not a limit
        on how far the aircraft may look.

        So the bound stays (a sweep that never terminates cannot fail closed),
        but it is now one full turn, and these tests hold both halves: it
        terminates, and it does not accept weak evidence on the way round.
    """

    def test_the_sweep_gives_up_rather_than_turning_for_ever(self):
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.0)
        leaf = self._leaf(yaw_step=0.4, timeout_ticks=10000)
        self.tick(leaf, n=2000)
        self.assertEqual(leaf.status, self.py_trees.common.Status.FAILURE)
        self.assertIn("deg", self.mav.abort_reason)

    def test_it_stops_after_ONE_full_turn_not_two(self):
        """Bounded means bounded. A sweep that laps the horizon twice has
        spent 90 seconds of a 15-minute mission proving the same thing."""
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.0)
        leaf = self._leaf(yaw_step=0.4, timeout_ticks=10000,
                          step_rad=math.radians(30.0))
        self.tick(leaf, n=2000)
        self.assertEqual(len(leaf.step_reports), 12)

    def test_a_rejected_decoy_does_not_stop_the_sweep_early(self):
        """z=0.5 is "green, not the banner" and must keep the search going."""
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.5)
        leaf = self._leaf(yaw_step=0.4, timeout_ticks=10000)
        self.tick(leaf, n=5)
        self.assertEqual(leaf.status, self.py_trees.common.Status.RUNNING)

    def test_a_banner_seen_in_only_a_few_frames_is_not_accepted(self):
        """The 271-degree lock, stated as the rule that now prevents it."""
        class Flickering:
            def __init__(self, mav):
                self.mav = mav
                self.n = 0

            def __call__(self):
                self.n += 1
                return self.n % 6 == 0        # 17% of frames

        leaf = self._leaf(yaw_step=0.4, timeout_ticks=10000, min_hit_ratio=0.6)
        self.mav.banner_identified = Flickering(self.mav)
        self.tick(leaf, n=2000)
        self.assertEqual(leaf.status, self.py_trees.common.Status.FAILURE,
                         "aligned to something seen in one frame in six")

    def test_a_banner_found_early_in_the_sweep_still_aligns(self):
        """The bound must not break the normal case."""
        leaf = self._leaf(yaw_step=0.4, stable_frames=2)
        self.mav.banner = self.Vector3(x=0.0, y=0.0, z=0.0)
        self.tick(leaf, n=3)
        self.mav.banner = self.Vector3(x=0.02, y=0.0, z=1.0)
        self.tick(leaf, n=40)
        self.assertEqual(leaf.status, self.py_trees.common.Status.SUCCESS)


# --------------------------------------------------------------------------- #
class RenderedFrameRegressionTests(unittest.TestCase):
    """A real rendered frame, kept because synthetic fixtures missed this.

    Every synthetic test in this file drew the lettering at (245,245,245) and
    passed while the live detector reported `components: 0, white_frac: 0.0`
    for a banner filling most of the image. Under the simulator's flat ambient
    light the lettering renders at about V=150 -- below the fixed V>=170 floor
    the detector used. The fixtures could not fail because they were drawn by
    the same assumption the detector was making.

    tests/fixtures/banner_sim_ambient_3m.png is the frame from that live run.
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BannerNode()
        self.sent, self.detail = [], []
        self.node.pub.publish = self.sent.append
        self.node.pub_detail.publish = self.detail.append
        self.node.pub_annot.publish = lambda m: None

    def tearDown(self):
        self.node.destroy_node()

    def _feed(self, name):
        import json
        from cv_bridge import CvBridge
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "tests", "fixtures", name)
        img = cv2.imread(path)
        self.assertIsNotNone(img, f"fixture missing: {path}")
        self.node.on_image(CvBridge().cv2_to_imgmsg(img, encoding="bgr8"))
        return self.sent[-1], json.loads(self.detail[-1].data)

    def test_the_real_rendered_banner_is_identified(self):
        out, d = self._feed("banner_sim_ambient_3m.png")
        self.assertEqual(out.z, 1.0, f"live banner frame rejected: {d}")

    def test_the_lettering_is_actually_found_in_it(self):
        """Not just "passed" -- the letters have to be what passed it."""
        _, d = self._feed("banner_sim_ambient_3m.png")
        self.assertGreaterEqual(d["components"], 4,
                                f"lettering not detected: {d}")

    def test_a_frame_with_no_banner_reports_nothing(self):
        out, d = self._feed("banner_absent_horizon.png")
        self.assertNotEqual(out.z, 1.0, f"identified a banner in sky: {d}")

    def test_the_bearing_is_plausible_for_a_roughly_centred_banner(self):
        out, _ = self._feed("banner_sim_ambient_3m.png")
        self.assertLess(abs(out.x), 0.5)


class BannerFailureIsDiagnosableTests(unittest.TestCase):
    """A failed sweep has to say WHY, not just that it failed.

    Arena regression seed 1001 failed with:

        no AEROTHON banner identified within 180 deg of the corridor-entry
        sweep (green objects seen but rejected, or none)

    "or none" is the problem: that message cannot distinguish "nothing green
    was ever in frame" (the aircraft is pointed the wrong way, or the gate is
    out of range) from "green things were found and the identity check threw
    them all out" (the detector's thresholds are wrong). Those need opposite
    fixes, and the detector knew which it was the whole time -- it publishes a
    reason per frame on /percep/banner/detail, which nothing was reading.
    """

    def setUp(self):
        from mission_bt.mission_tree import AlignToBanner
        import py_trees
        from geometry_msgs.msg import Vector3

        class FakeMav:
            def __init__(self):
                self.banner = Vector3()
                self.abort_reason = ""
                self.gotos = []
                self._yaw = 0.0
                self.logs = []
                self.reject = ""
                # The stop-and-stare sweep waits for the AIRFRAME to reach the
                # heading it asked for, so a fake whose heading never moves
                # would stall in SETTLE forever and prove nothing.
                self.follow_yaw = True

            def pos(self):
                return (1.0, 2.0, 3.0)

            def yaw(self):
                return self._yaw

            def goto(self, x, y, z, yaw=0.0):
                self.gotos.append((x, y, z, yaw))
                if self.follow_yaw:
                    self._yaw = math.atan2(math.sin(yaw), math.cos(yaw))

            def banner_identified(self):
                return self.banner.z >= 1.0

            def banner_bearing(self):
                return self.banner.x if self.banner_identified() else 0.0

            def log(self, msg, warn=False):
                self.logs.append((msg, warn))

            def banner_rejection_summary(self, top=3):
                return self.reject or "nothing green ever entered the frame"

        self.py_trees = py_trees
        self.mav = FakeMav()
        self.AlignToBanner = AlignToBanner

    def _sweep_to_failure(self):
        clock = _Clock()
        leaf = self.AlignToBanner(self.mav, tol=0.1, yaw_step=0.5,
                                  timeout_ticks=20, stable_frames=3,
                                  dwell_s=0.6, clock=clock)
        for _ in range(2000):
            leaf.tick_once()
            clock.advance(0.1)
            if leaf.status == self.py_trees.common.Status.FAILURE:
                break
        return leaf

    def test_the_detectors_reason_reaches_the_abort_message(self):
        self.mav.reject = "aspect 17.40 outside 1.5..6.0 (x12)"
        self._sweep_to_failure()
        self.assertIn("aspect 17.40", self.mav.abort_reason)

    def test_nothing_green_is_DISTINGUISHED_from_all_rejected(self):
        """The two failures that need opposite fixes."""
        self._sweep_to_failure()
        self.assertIn("nothing green", self.mav.abort_reason)

        self.mav.abort_reason = ""
        self.mav.reject = "no white lettering band found (x31)"
        self._sweep_to_failure()
        self.assertIn("lettering", self.mav.abort_reason)
        self.assertNotIn("nothing green", self.mav.abort_reason)

    def test_the_failure_is_LOGGED_not_only_stored(self):
        """abort_reason goes to the GCS; the run artifacts need it too, which
        is how the seed 1001 failure came to be undiagnosable after the fact."""
        self.mav.reject = "area 120 px below derived minimum 900 px (x8)"
        self._sweep_to_failure()
        warned = [m for m, w in self.mav.logs if w and "AlignToBanner" in m]
        self.assertTrue(warned, "the failure was never logged")
        self.assertIn("area 120 px", warned[0])


class TextRescueTests(unittest.TestCase):
    """A board the aspect gate rejects, whose lettering plainly reads AEROTHON.

    Seed 1001's return lap fails on `board aspect 0.73 outside 1.2-8.0`: seen
    from the delivery-zone side the board is foreshortened into proportions the
    gate refuses, while the lettering stays legible and is never consulted.

    These exercise the NODE. sim/test_banner_text.py already proves the reader
    reads; what was missing is that anything USES it -- the same gap that let a
    mutation of the altitude-hold fix pass unnoticed earlier.

    Built on the REAL rendered frame, not hand-drawn glyphs. Three attempts at
    drawing letters pixel-by-pixel produced fixtures that fragmented into one
    component per bitmap row and read as nothing at all; the detector was fine
    every time. A fixture that hard to draw correctly is not evidence about the
    detector.
    """

    FIXTURE = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "tests", "fixtures", "banner_sim_ambient_3m.png")

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BannerNode()
        self.detail = []
        self.node.pub.publish = lambda m: None
        self.node.pub_detail.publish = self.detail.append
        self.node.pub_annot.publish = lambda m: None

    def tearDown(self):
        self.node.destroy_node()

    def feed(self, img):
        from cv_bridge import CvBridge
        self.node.on_image(CvBridge().cv2_to_imgmsg(img, encoding="bgr8"))

    def last(self):
        import json
        return json.loads(self.detail[-1].data)

    def banner(self):
        img = cv2.imread(self.FIXTURE)
        self.assertIsNotNone(img, f"fixture missing: {self.FIXTURE}")
        return img

    def set_aspect_ceiling(self, value):
        self.node.set_parameters([rclpy.parameter.Parameter(
            'max_aspect', rclpy.Parameter.Type.DOUBLE, value)])

    def test_the_real_banner_lettering_READS(self):
        self.feed(self.banner())
        d = self.last()
        self.assertEqual(d.get("text"), "AEROTHON")
        self.assertEqual(d.get("text_letters"), 8)

    def test_a_frame_without_a_banner_reads_nothing(self):
        img = cv2.imread(os.path.join(os.path.dirname(self.FIXTURE),
                                      "banner_absent_horizon.png"))
        self.feed(img)
        self.assertFalse(self.last()["identified"])

    def test_the_aspect_gate_really_can_reject_this_board(self):
        """Guards the fixture: if the board passed anyway, the rescue test
        below would be proving nothing."""
        self.set_aspect_ceiling(1.5)          # real board measures 3.59
        self.node.set_parameters([rclpy.parameter.Parameter(
            'min_text_letters', rclpy.Parameter.Type.INTEGER, 99)])
        self.feed(self.banner())
        d = self.last()
        self.assertFalse(d["identified"])
        self.assertIn("aspect", d["reason"])

    def test_readable_lettering_RESCUES_the_rejected_board(self):
        self.set_aspect_ceiling(1.5)
        self.feed(self.banner())
        d = self.last()
        self.assertTrue(d["identified"],
                        f"not rescued; reason={d.get('reason')!r} "
                        f"text={d.get('text')!r}")
        self.assertTrue(d.get("rescued_by_text"))
        self.assertEqual(d.get("text"), "AEROTHON")

    def test_the_rescue_needs_the_RIGHT_word(self):
        """Raising the letter requirement above what any word can supply must
        stop the rescue -- it keys on the text, not on text existing."""
        self.set_aspect_ceiling(1.5)
        self.node.set_parameters([rclpy.parameter.Parameter(
            'min_text_letters', rclpy.Parameter.Type.INTEGER, 99)])
        self.feed(self.banner())
        self.assertFalse(self.last()["identified"])

    def test_the_rescue_never_VETOES_a_good_board(self):
        """One-directional by design: with the gate at its real setting the
        banner is identified, rescue or not."""
        self.feed(self.banner())
        d = self.last()
        self.assertTrue(d["identified"])
        self.assertFalse(d.get("rescued_by_text", False),
                         "a board that passes on its merits was marked rescued")


class ShadowRobustLetteringTests(unittest.TestCase):
    """A banner in shade is still read, because a second path asks a question
    a shadow cannot change.

    THE OPERATOR'S REPORT: "very clearly there is a banner there and it does
    not detect it", on a frame where the board was partly shaded and viewed
    obliquely.

    THE MECHANISM: lettering_mask() asks whether a pixel is brighter than the
    BOARD MEDIAN. Under a shade gradient that median is set by the sunlit half,
    and the letters in the shaded half fall under it. Identification often
    survives on the lit half alone -- but the READING does not, and the reading
    is what rescues the oblique return view whose aspect the gate refuses. The
    two failures compound.

    MEASURED on the real rendered frame under a linear shade ramp, letters
    read out of AEROTHON's eight:

        far edge at   100%   70%   50%   40%   30%   25%   20%   15%
        brightness       8     8     5     3     3     0     0     0
        both paths       8     8     7     6     5     4     4     0

    The rescue needs five. Brightness alone holds to 50% shade; both paths
    hold to 30%.

    THE FIXTURE IS NOT HAND-DRAWN. Four attempts at drawing letters pixel by
    pixel produced fixtures that fragmented into one component per bitmap row
    and proved nothing. This is a photometric transform of a frame the
    simulator actually rendered, and the transform models the physical thing:
    a board half in the shade of the gate post.
    """

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BannerNode()
        self.sent, self.detail = [], []
        self.node.pub.publish = self.sent.append
        self.node.pub_detail.publish = self.detail.append
        self.node.pub_annot.publish = lambda m: None
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "tests", "fixtures", "banner_sim_ambient_3m.png")
        self.src = cv2.imread(path)
        self.assertIsNotNone(self.src, f"fixture missing: {path}")

    def tearDown(self):
        self.node.destroy_node()

    def _shaded(self, far_edge):
        """The rendered banner with a linear illumination ramp across it."""
        w = self.src.shape[1]
        ramp = np.linspace(1.0, far_edge, w, dtype=np.float32)[None, :, None]
        return np.clip(self.src.astype(np.float32) * ramp,
                       0, 255).astype(np.uint8)

    def _feed(self, img):
        import json
        from cv_bridge import CvBridge
        self.node.on_image(CvBridge().cv2_to_imgmsg(img, encoding="bgr8"))
        return self.sent[-1], json.loads(self.detail[-1].data)

    def _stroke(self, on):
        from rclpy.parameter import Parameter
        self.node.set_parameters([Parameter('stroke_path', value=bool(on))])

    # ---- the failure, and the fix ---- #
    def test_the_brightness_path_alone_cannot_read_a_shaded_banner(self):
        """The test has to be capable of failing, so prove the old path does."""
        self._stroke(False)
        _, d = self._feed(self._shaded(0.30))
        self.assertLess(d.get("text_letters", 0), 5,
                        f"the brightness path unexpectedly coped: {d}")

    def test_both_paths_read_the_shaded_banner(self):
        self._stroke(True)
        _, d = self._feed(self._shaded(0.30))
        self.assertGreaterEqual(d.get("text_letters", 0), 5,
                                f"shaded banner still unread: {d}")

    def test_the_shaded_reading_is_actually_AEROTHON(self):
        """More letters is not the point; the right letters are."""
        self._stroke(True)
        _, d = self._feed(self._shaded(0.30))
        self.assertTrue(d.get("text", "").startswith("AERO"),
                        f"read something, but not the banner: {d.get('text')!r}")

    def test_a_deeply_shaded_banner_is_still_identified(self):
        self._stroke(True)
        out, d = self._feed(self._shaded(0.25))
        self.assertEqual(out.z, 1.0, f"lost the banner entirely: {d}")

    # ---- the second path must not displace the first ---- #
    def test_full_light_still_uses_the_brightness_path(self):
        """The stroke path is a rescue, not a replacement. If it started
        winning in good light it would be deciding cases nobody measured it
        on."""
        self._stroke(True)
        _, d = self._feed(self.src)
        self.assertEqual(d.get("lettering_path"), "brightness")
        self.assertEqual(d.get("text"), "AEROTHON")

    def test_the_detail_topic_says_which_path_read_it(self):
        """An operator has to be able to tell a clean read from a rescued one."""
        self._stroke(True)
        _, d = self._feed(self._shaded(0.30))
        self.assertEqual(d.get("lettering_path"), "stroke")

    # ---- widening the reader must not widen acceptance ---- #
    def test_a_blank_green_board_is_still_refused_by_BOTH_paths(self):
        """The whole point of the identity gate. A local-contrast path that
        found lettering on a plain tarpaulin would be worse than no path."""
        board = np.zeros((300, 900, 3), np.uint8)
        board[:, :] = (60, 160, 60)
        frame = np.zeros((720, 1280, 3), np.uint8)
        frame[200:500, 150:1050] = board
        self._stroke(True)
        out, d = self._feed(frame)
        self.assertNotEqual(out.z, 1.0,
                            f"identified a blank green board: {d}")

    def test_a_noisy_green_board_is_still_refused(self):
        """Texture is not lettering. Local contrast alone would say yes."""
        rng = np.random.default_rng(7)
        frame = np.zeros((720, 1280, 3), np.uint8)
        board = np.zeros((300, 900, 3), np.uint8)
        board[:, :] = (60, 160, 60)
        noise = rng.integers(-35, 35, board.shape, dtype=np.int16)
        frame[200:500, 150:1050] = np.clip(
            board.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        self._stroke(True)
        out, d = self._feed(frame)
        self.assertNotEqual(out.z, 1.0, f"identified green noise: {d}")

    def test_open_sky_still_reports_nothing(self):
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "tests", "fixtures", "banner_absent_horizon.png")
        img = cv2.imread(path)
        self._stroke(True)
        out, d = self._feed(img)
        self.assertNotEqual(out.z, 1.0, f"identified a banner in sky: {d}")

    # ---- both paths are graded by the same reader ---- #
    def test_both_paths_go_through_the_same_identity_check(self):
        """Two copies of "what counts as the banner" is two definitions, and
        the looser one would win every disagreement."""
        import inspect
        src = inspect.getsource(BannerNode.identity)
        self.assertIn("_identify_with", src)
        self.assertEqual(src.count("_identify_with("), 1,
                         "identity() should route every path through one check")

    def test_the_second_path_does_not_double_the_frame_cost(self):
        """It runs on every frame, at camera rate, on a Pi 5."""
        import time as _t
        self._stroke(True)
        img = self._shaded(0.30)
        self._feed(img)                                   # warm
        t0 = _t.perf_counter()
        for _ in range(5):
            self._feed(img)
        per_frame = (_t.perf_counter() - t0) / 5.0
        self.assertLess(per_frame, 0.20,
                        f"{per_frame * 1000:.0f} ms per frame")
