#!/usr/bin/env python3
"""Phase 7 — a red zone the mission can actually avoid.

WHAT WAS WRONG
    perception_redzone published one Bool: "red is visible somewhere". That
    has no position, so nothing could route around it, and it collapsed two
    different facts into one value — "the camera cannot see the ground there"
    and "the camera can see it and it is clear" were both `false`.

    Phase 7 projects detections onto the ground plane and accumulates them
    into exclusion rectangles the search planner clips its lanes against.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_redzone_georef.py -v
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_perception",
                                "perception_redzone"))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_mission", "mission_bt"))

from perception_redzone.georef import (
    GroundGrid,
    bbox,
    camera_axes,
    focal_px,
    footprint,
    ground_point,
)
from mission_bt.search_planner import (
    clip_lane,
    coverage_fraction_excluding,
    ground_width,
    lane_spacing,
    plan_intersects_exclusions,
    plan_lawnmower_excluding,
)

HFOV = math.radians(60.0)
WH = (640, 480)
NADIR = math.pi / 2
FORWARD = 0.0


class CameraAxisTests(unittest.TestCase):
    """The signs are derived and checked, not asserted — a wrong sign here is
    the same class of error that flew the aircraft into a wall in Phase 2."""

    def test_forward_pose_looks_along_body_x(self):
        o, _, _ = camera_axes(FORWARD)
        self.assertAlmostEqual(o[0], 1.0)
        self.assertAlmostEqual(o[2], 0.0)

    def test_nadir_pose_looks_straight_down(self):
        o, _, _ = camera_axes(NADIR)
        self.assertAlmostEqual(o[0], 0.0, places=9)
        self.assertAlmostEqual(o[2], -1.0)

    def test_image_right_is_body_right(self):
        """Body +y is LEFT in FLU, so image right must be -y."""
        _, r, _ = camera_axes(NADIR)
        self.assertAlmostEqual(r[1], -1.0)

    def test_image_down_is_world_down_when_looking_forward(self):
        _, _, d = camera_axes(FORWARD)
        self.assertAlmostEqual(d[2], -1.0)

    def test_image_down_is_BEHIND_when_looking_down(self):
        """Flying forward moves the scene UP the image — the mapping
        CenterOnQR depends on. Getting this backwards would send the aircraft
        away from every marker it saw."""
        _, _, d = camera_axes(NADIR)
        self.assertAlmostEqual(d[0], -1.0, places=9)

    def test_axes_are_orthonormal(self):
        for phi in (0.0, 0.4, 1.0, NADIR):
            o, r, d = camera_axes(phi)
            for v in (o, r, d):
                self.assertAlmostEqual(sum(c * c for c in v), 1.0, places=9)
            self.assertAlmostEqual(sum(a * b for a, b in zip(o, r)), 0.0, places=9)
            self.assertAlmostEqual(sum(a * b for a, b in zip(o, d)), 0.0, places=9)
            self.assertAlmostEqual(sum(a * b for a, b in zip(r, d)), 0.0, places=9)


class ProjectionTests(unittest.TestCase):

    def test_image_centre_at_nadir_is_directly_below(self):
        g = ground_point(320, 240, WH, HFOV, 10.0, (5.0, -2.0), 0.0, NADIR)
        self.assertAlmostEqual(g[0], 5.0, places=6)
        self.assertAlmostEqual(g[1], -2.0, places=6)

    def test_frame_edge_matches_the_known_ground_width(self):
        """The same swath figure the search planner uses, arrived at the other
        way round — projection and coverage must agree."""
        g = ground_point(640, 240, WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR)
        half = ground_width(10.0, HFOV) / 2.0
        self.assertAlmostEqual(abs(g[1]), half, places=1)

    def test_scene_below_image_centre_is_BEHIND_at_nadir(self):
        g = ground_point(320, 400, WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR)
        self.assertLess(g[0], 0.0, "down the image must map behind the aircraft")

    def test_right_of_image_centre_is_to_the_right(self):
        """Heading east (yaw 0), right of frame is -y in ENU."""
        g = ground_point(500, 240, WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR)
        self.assertLess(g[1], 0.0)

    def test_projection_rotates_with_yaw(self):
        straight = ground_point(320, 100, WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR)
        turned = ground_point(320, 100, WH, HFOV, 10.0, (0.0, 0.0),
                              math.pi / 2, NADIR)
        self.assertGreater(straight[0], 0.1)
        self.assertAlmostEqual(turned[1], straight[0], places=6)
        self.assertAlmostEqual(turned[0], 0.0, places=6)

    def test_distance_scales_with_altitude(self):
        low = ground_point(320, 100, WH, HFOV, 5.0, (0.0, 0.0), 0.0, NADIR)
        high = ground_point(320, 100, WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR)
        self.assertAlmostEqual(high[0] / low[0], 2.0, places=6)

    def test_a_ray_above_the_horizon_is_UNKNOWN_not_the_origin(self):
        """A forward-looking camera's upper frame never meets the ground. A
        false (0,0) would place a red zone under the aircraft."""
        self.assertIsNone(
            ground_point(320, 0, WH, HFOV, 10.0, (0.0, 0.0), 0.0, FORWARD))

    def test_on_the_ground_projects_nothing(self):
        self.assertIsNone(
            ground_point(320, 240, WH, HFOV, 0.0, (0.0, 0.0), 0.0, NADIR))

    def test_focal_length_matches_the_field_of_view(self):
        f = focal_px(640, HFOV)
        self.assertAlmostEqual(2 * math.atan((640 / 2) / f), HFOV, places=9)


class FootprintTests(unittest.TestCase):
    """What makes CLEAR different from NOT VISIBLE."""

    def test_nadir_footprint_is_centred_on_the_aircraft(self):
        fp = footprint(WH, HFOV, 10.0, (4.0, 3.0), 0.0, NADIR)
        x0, x1, y0, y1 = bbox(fp)
        self.assertAlmostEqual((x0 + x1) / 2, 4.0, places=6)
        self.assertAlmostEqual((y0 + y1) / 2, 3.0, places=6)

    def test_nadir_footprint_width_matches_the_swath(self):
        x0, x1, y0, y1 = bbox(footprint(WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR))
        self.assertAlmostEqual(y1 - y0, ground_width(10.0, HFOV), places=1)

    def test_forward_camera_has_NO_bounded_footprint(self):
        """It sees to the horizon, so "no red in this image" cannot be turned
        into "this area is clear" — which is exactly the distinction the old
        Bool destroyed."""
        self.assertIsNone(footprint(WH, HFOV, 10.0, (0.0, 0.0), 0.0, FORWARD))

    def test_footprint_grows_with_altitude(self):
        low = bbox(footprint(WH, HFOV, 5.0, (0.0, 0.0), 0.0, NADIR))
        high = bbox(footprint(WH, HFOV, 10.0, (0.0, 0.0), 0.0, NADIR))
        self.assertGreater(high[1] - high[0], low[1] - low[0])


class GroundGridTests(unittest.TestCase):

    def test_a_single_sighting_is_not_a_zone(self):
        """HSV picks up a red jacket, a flare, wet clay. One frame is not
        enough to plan around."""
        g = GroundGrid(cell_m=1.0, confirm_hits=3)
        g.add([(10.0, 10.0)])
        self.assertEqual(g.exclusions(), [])

    def test_repeated_sightings_confirm_a_zone(self):
        g = GroundGrid(cell_m=1.0, confirm_hits=3)
        for _ in range(3):
            g.add([(10.5, 10.5)])
        ex = g.exclusions()
        self.assertEqual(len(ex), 1)
        x0, x1, y0, y1 = ex[0]
        self.assertLessEqual(x0, 10.5)
        self.assertGreaterEqual(x1, 10.5)

    def test_nearby_points_merge_into_one_cell(self):
        g = GroundGrid(cell_m=2.0, confirm_hits=2)
        g.add([(4.1, 4.1), (5.9, 5.9)])
        self.assertEqual(len(g.exclusions()), 1)

    def test_separate_zones_stay_separate(self):
        g = GroundGrid(cell_m=1.0, confirm_hits=1)
        g.add([(1.5, 1.5), (30.5, 30.5)])
        self.assertEqual(len(g.exclusions()), 2)

    def test_negative_coordinates_do_not_collapse_onto_zero(self):
        """int() truncates toward zero; -0.5 and +0.5 would share a cell."""
        g = GroundGrid(cell_m=1.0, confirm_hits=1)
        g.add([(-0.5, 0.5), (0.5, 0.5)])
        self.assertEqual(len(g.exclusions()), 2)

    def test_inflation_grows_the_exclusion(self):
        g = GroundGrid(cell_m=1.0, confirm_hits=1)
        g.add([(10.5, 10.5)])
        plain = g.exclusions()[0]
        grown = g.exclusions(inflate_m=2.0)[0]
        self.assertAlmostEqual(grown[0], plain[0] - 2.0)
        self.assertAlmostEqual(grown[1], plain[1] + 2.0)

    def test_zero_cell_size_is_rejected(self):
        with self.assertRaises(ValueError):
            GroundGrid(cell_m=0.0)

    def test_area_reflects_confirmed_cells_only(self):
        g = GroundGrid(cell_m=2.0, confirm_hits=2)
        g.add([(1.0, 1.0)])
        self.assertEqual(g.area_m2(), 0.0)
        g.add([(1.0, 1.0)])
        self.assertEqual(g.area_m2(), 4.0)


class LaneClippingTests(unittest.TestCase):

    def test_an_unobstructed_lane_is_unchanged(self):
        self.assertEqual(clip_lane(0.0, 20.0, 5.0, 1.0, []), [(0.0, 20.0)])

    def test_an_exclusion_splits_the_lane(self):
        segs = clip_lane(0.0, 20.0, 5.0, 1.0, [(8.0, 12.0, 4.0, 6.0)])
        self.assertEqual(segs, [(0.0, 8.0), (12.0, 20.0)])

    def test_an_exclusion_beside_the_lane_is_ignored(self):
        segs = clip_lane(0.0, 20.0, 5.0, 1.0, [(8.0, 12.0, 40.0, 46.0)])
        self.assertEqual(segs, [(0.0, 20.0)])

    def test_the_AIRFRAME_is_what_must_stay_out_not_the_camera(self):
        """A red zone forbids flying over it, not looking at it. Clipping
        against the camera swath (tried first) carved out reachable ground:
        coverage fell to 0.81 where 1.0 was achievable."""
        tight = clip_lane(0.0, 20.0, 5.0, 0.5, [(8.0, 12.0, 7.0, 9.0)])
        wide = clip_lane(0.0, 20.0, 5.0, 3.0, [(8.0, 12.0, 7.0, 9.0)])
        self.assertEqual(tight, [(0.0, 20.0)],
                         "a zone 2 m to the side blocked a lane the aircraft "
                         "would have cleared")
        self.assertNotEqual(wide, [(0.0, 20.0)],
                            "a larger clearance must be more conservative")

    def test_overlapping_exclusions_merge(self):
        segs = clip_lane(0.0, 20.0, 5.0, 1.0,
                         [(8.0, 12.0, 4.0, 6.0), (10.0, 14.0, 4.0, 6.0)])
        self.assertEqual(segs, [(0.0, 8.0), (14.0, 20.0)])

    def test_an_exclusion_covering_everything_leaves_nothing(self):
        self.assertEqual(
            clip_lane(0.0, 20.0, 5.0, 1.0, [(-5.0, 25.0, 0.0, 10.0)]), [])


class ExclusionPlanTests(unittest.TestCase):

    ZONE = (0.0, 40.0, -10.0, 10.0)
    ALT = 8.0

    def spacing(self):
        return lane_spacing(self.ALT, HFOV, overlap=0.30)

    def test_plan_without_exclusions_matches_the_plain_lawnmower(self):
        wps = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT, HFOV)
        self.assertGreater(len(wps), 0)
        self.assertTrue(all(self.ZONE[0] <= w[0] <= self.ZONE[1] for w in wps))

    def test_the_plan_provably_avoids_the_exclusion(self):
        """The Phase 7 acceptance assertion: a red zone sitting inside the
        naive lane plan is routed around."""
        ex = [(15.0, 25.0, -3.0, 3.0)]
        clr = 1.0
        naive = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT,
                                         HFOV, exclusions=[])
        self.assertTrue(plan_intersects_exclusions(naive, clr, ex),
                        "test is vacuous: the naive plan misses the zone anyway")
        safe = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT,
                                        HFOV, exclusions=ex, clearance_m=clr)
        self.assertFalse(plan_intersects_exclusions(safe, clr, ex),
                         "planned path still overflies the red zone")

    def test_coverage_of_reachable_ground_is_preserved(self):
        """A plan is not at fault for missing ground it is forbidden to fly
        over — but it must still cover everything else."""
        ex = [(15.0, 25.0, -3.0, 3.0)]
        cov = coverage_fraction_excluding(self.ZONE, self.spacing(), self.ALT,
                                          HFOV, exclusions=ex)
        self.assertGreater(cov, 0.90, f"clipping lost reachable coverage: {cov:.3f}")

    def test_a_zone_entirely_red_yields_no_waypoints(self):
        ex = [(-10.0, 50.0, -20.0, 20.0)]
        wps = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT,
                                       HFOV, exclusions=ex)
        self.assertEqual(wps, [])

    def test_tiny_stubs_are_dropped(self):
        """A 30 cm segment costs more in settling time than it is worth."""
        ex = [(0.5, 40.0, -20.0, 20.0)]
        wps = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT,
                                       HFOV, exclusions=ex, min_segment_m=1.0)
        self.assertEqual(wps, [])

    def test_lanes_still_alternate_direction(self):
        wps = plan_lawnmower_excluding(self.ZONE, self.spacing(), self.ALT, HFOV)
        self.assertLess(wps[0][0], wps[1][0])
        self.assertGreater(wps[2][0], wps[3][0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
