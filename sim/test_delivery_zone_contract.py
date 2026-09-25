#!/usr/bin/env python3
"""The supplied delivery-zone geofence is the search contract."""

import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_mission", "mission_bt"))

from mission_bt.delivery_zone import (                         # noqa: E402
    boundary_to_local_zone,
    inset_zone,
    parse_boundary,
)
from mission_bt.geofence import local_to_global                # noqa: E402
from mission_bt.search_planner import plan_search              # noqa: E402


HOME = (-35.363262, 149.165237)


def global_rectangle(zone):
    x0, x1, y0, y1 = zone
    return [local_to_global(x, y, *HOME)
            for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


class BoundaryInputTests(unittest.TestCase):

    def test_gcs_json_polygon_becomes_the_same_local_zone(self):
        expected = (19.8, 59.8, -23.5, 6.5)
        payload = json.dumps({
            "vertices": [{"lat": lat, "lon": lon}
                         for lat, lon in global_rectangle(expected)]
        })

        vertices, why = parse_boundary(payload)
        self.assertEqual(why, "")
        got, why = boundary_to_local_zone(vertices, *HOME)

        self.assertEqual(why, "")
        for actual, want in zip(got, expected):
            self.assertAlmostEqual(actual, want, places=2)

    def test_missing_boundary_is_invalid(self):
        vertices, why = parse_boundary("")
        self.assertIsNone(vertices)
        self.assertIn("missing", why)

    def test_nonfinite_home_cannot_produce_a_valid_search_zone(self):
        points = global_rectangle((0.0, 40.0, 0.0, 30.0))
        for home in ((float("nan"), HOME[1]), (HOME[0], float("inf"))):
            zone, why = boundary_to_local_zone(points, *home)
            self.assertIsNone(zone)
            self.assertIn("home", why)

    def test_three_vertices_cannot_masquerade_as_the_rectangular_field(self):
        vertices, why = parse_boundary(json.dumps({
            "vertices": [{"lat": 1.0, "lon": 2.0}] * 3,
        }))
        self.assertIsNone(vertices)
        self.assertIn("four", why)

    def test_duplicate_corner_is_invalid(self):
        points = global_rectangle((0.0, 40.0, 0.0, 30.0))
        points[-1] = points[0]
        vertices, why = parse_boundary(json.dumps({
            "vertices": [{"lat": lat, "lon": lon} for lat, lon in points],
        }))
        self.assertIsNone(vertices)
        self.assertIn("distinct", why)

    def test_non_rectangular_boundary_fails_instead_of_searching_its_bbox(self):
        points = global_rectangle((0.0, 40.0, 0.0, 30.0))
        # Pull one corner 8 m into the field. Its bounding box still looks
        # plausible, but sweeping that box would leave the supplied polygon.
        points[2] = local_to_global(32.0, 30.0, *HOME)
        zone, why = boundary_to_local_zone(points, *HOME)
        self.assertIsNone(zone)
        self.assertIn("rectangle", why)


class SearchEnvelopeTests(unittest.TestCase):

    def test_aircraft_track_is_inset_from_every_boundary(self):
        self.assertEqual(inset_zone((0.0, 40.0, -15.0, 15.0), 1.5),
                         (1.5, 38.5, -13.5, 13.5))

    def test_impossible_inset_fails(self):
        with self.assertRaisesRegex(ValueError, "clearance"):
            inset_zone((0.0, 2.0, 0.0, 2.0), 1.5)

    def test_axis_aligned_planner_chooses_the_long_axis(self):
        wide = plan_search((0.0, 40.0, 0.0, 20.0), 1280, math.radians(60),
                           2.2, 33, max_altitude=10.0, axis="auto")
        tall = plan_search((0.0, 20.0, 0.0, 40.0), 1280, math.radians(60),
                           2.2, 33, max_altitude=10.0, axis="auto")
        self.assertEqual(wide["lane_axis"], "x")
        self.assertEqual(tall["lane_axis"], "y")

    def test_every_search_waypoint_remains_inside_the_inset_zone(self):
        zone = inset_zone((19.8, 59.8, -23.5, 6.5), 1.5)
        plan = plan_search(zone, 1280, math.radians(60), 2.2, 33,
                           max_altitude=10.0, axis="auto")
        x0, x1, y0, y1 = zone
        for x, y, _ in plan["waypoints"]:
            self.assertGreaterEqual(x, x0)
            self.assertLessEqual(x, x1)
            self.assertGreaterEqual(y, y0)
            self.assertLessEqual(y, y1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
