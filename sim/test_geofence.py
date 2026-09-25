#!/usr/bin/env python3
"""Phase 7 — a fence that is verified, not merely uploaded.

WHY THIS MATTERS
    Red-zone avoidance in the search planner is a PLAN, and plans are wrong
    when perception is wrong, when the aircraft drifts, or when a human takes
    control. goal.md Q13/Q24 make the ArduPilot exclusion fence the backstop
    underneath all of that.

    An unverified fence is worse than no fence, because it is believed. These
    tests hold the comparison to being able to FAIL: a swapped axis, a dropped
    vertex, a wrong frame and a truncated polygon each have to be caught.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_geofence.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

from mission_bt.geofence import (
    FENCE_POLYGON_EXCLUSION,
    FENCE_POLYGON_INCLUSION,
    FRAME_GLOBAL_REL_ALT,
    build_fence,
    compare_fences,
    fence_signature,
    global_to_local,
    local_to_global,
    rect_vertices,
)

HOME_LAT, HOME_LON = -35.363262, 149.165237      # SITL default (Canberra)


class FakeWaypoint:
    """Stands in for mavros_msgs/Waypoint so this runs without MAVROS."""

    def __init__(self):
        self.frame = 0
        self.command = 0
        self.is_current = False
        self.autocontinue = False
        self.param1 = self.param2 = self.param3 = self.param4 = 0.0
        self.x_lat = self.y_long = self.z_alt = 0.0


class ProjectionTests(unittest.TestCase):

    def test_round_trip_is_lossless_to_the_millimetre(self):
        for x, y in ((0.0, 0.0), (50.0, -30.0), (-120.0, 240.0)):
            lat, lon = local_to_global(x, y, HOME_LAT, HOME_LON)
            bx, by = global_to_local(lat, lon, HOME_LAT, HOME_LON)
            self.assertAlmostEqual(bx, x, places=3)
            self.assertAlmostEqual(by, y, places=3)

    def test_x_is_EAST_and_y_is_NORTH(self):
        """Swapping these rotates the whole fence 90 degrees about home, and
        the result still looks perfectly plausible on a map."""
        east, _ = local_to_global(100.0, 0.0, HOME_LAT, HOME_LON)
        _, north_lon = local_to_global(0.0, 100.0, HOME_LAT, HOME_LON)
        lat_e, lon_e = local_to_global(100.0, 0.0, HOME_LAT, HOME_LON)
        lat_n, lon_n = local_to_global(0.0, 100.0, HOME_LAT, HOME_LON)
        self.assertAlmostEqual(lat_e, HOME_LAT, places=6)   # east: lat unchanged
        self.assertGreater(lon_e, HOME_LON)
        self.assertGreater(lat_n, HOME_LAT)                 # north: lat rises
        self.assertAlmostEqual(lon_n, HOME_LON, places=6)

    def test_one_hundred_metres_north_is_about_nine_ten_thousandths_of_a_degree(self):
        lat, _ = local_to_global(0.0, 100.0, HOME_LAT, HOME_LON)
        self.assertAlmostEqual(lat - HOME_LAT, 100.0 / 111320.0, places=5)

    def test_longitude_scale_shrinks_with_latitude(self):
        _, near_eq = local_to_global(100.0, 0.0, 0.0, 0.0)
        _, high = local_to_global(100.0, 0.0, 60.0, 0.0)
        self.assertGreater(high, near_eq * 1.9,
                           "longitude degrees must widen away from the equator")


class FenceBuildTests(unittest.TestCase):

    def test_rect_has_four_vertices(self):
        self.assertEqual(len(rect_vertices((0.0, 10.0, 0.0, 5.0))), 4)

    def test_inclusion_only(self):
        items = build_fence((0.0, 40.0, -10.0, 10.0), [], HOME_LAT, HOME_LON,
                            FakeWaypoint)
        self.assertEqual(len(items), 4)
        self.assertTrue(all(w.command == FENCE_POLYGON_INCLUSION for w in items))

    def test_each_exclusion_adds_a_polygon(self):
        items = build_fence((0.0, 40.0, -10.0, 10.0),
                            [(5.0, 8.0, 1.0, 3.0), (20.0, 24.0, -4.0, -1.0)],
                            HOME_LAT, HOME_LON, FakeWaypoint)
        self.assertEqual(len(items), 12)
        self.assertEqual(sum(w.command == FENCE_POLYGON_EXCLUSION
                             for w in items), 8)

    def test_every_vertex_carries_the_polygon_vertex_count(self):
        """ArduPilot delimits polygons by param1. Get it wrong and separate
        zones silently merge into one nonsensical shape."""
        items = build_fence((0.0, 40.0, -10.0, 10.0), [(5.0, 8.0, 1.0, 3.0)],
                            HOME_LAT, HOME_LON, FakeWaypoint)
        self.assertTrue(all(w.param1 == 4.0 for w in items))

    def test_frame_is_global_relative_altitude(self):
        items = build_fence((0.0, 10.0, 0.0, 10.0), [], HOME_LAT, HOME_LON,
                            FakeWaypoint)
        self.assertTrue(all(w.frame == FRAME_GLOBAL_REL_ALT for w in items))

    def test_vertices_land_where_the_local_rectangle_said(self):
        items = build_fence((0.0, 40.0, -10.0, 10.0), [], HOME_LAT, HOME_LON,
                            FakeWaypoint)
        locals_ = [global_to_local(w.x_lat, w.y_long, HOME_LAT, HOME_LON)
                   for w in items]
        xs = sorted({round(p[0]) for p in locals_})
        ys = sorted({round(p[1]) for p in locals_})
        self.assertEqual(xs, [0, 40])
        self.assertEqual(ys, [-10, 10])


class ReadBackVerificationTests(unittest.TestCase):
    """The comparison has to be capable of failing."""

    ZONE = (0.0, 40.0, -10.0, 10.0)
    EX = [(15.0, 25.0, -3.0, 3.0)]

    def sent(self):
        return build_fence(self.ZONE, self.EX, HOME_LAT, HOME_LON, FakeWaypoint)

    def test_an_identical_read_back_passes(self):
        ok, why = compare_fences(self.sent(), self.sent(), HOME_LAT, HOME_LON)
        self.assertTrue(ok, why)

    def test_int32_quantisation_does_not_fail_the_comparison(self):
        """ArduPilot stores lat/lon as int32 at 1e-7 deg, so a byte-identical
        read-back never happens. The check must survive that and nothing more."""
        got = self.sent()
        for w in got:
            w.x_lat = round(w.x_lat, 7)
            w.y_long = round(w.y_long, 7)
        ok, why = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertTrue(ok, why)

    def test_a_dropped_vertex_is_CAUGHT(self):
        got = self.sent()[:-1]
        ok, why = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertFalse(ok)
        self.assertIn("count", why)

    def test_a_moved_vertex_is_CAUGHT(self):
        got = self.sent()
        got[2].x_lat += 0.0005                 # ~55 m
        ok, why = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertFalse(ok)
        self.assertIn("item 2", why)

    def test_a_swapped_axis_is_CAUGHT(self):
        got = self.sent()
        for w in got:
            w.x_lat, w.y_long = w.y_long, w.x_lat
        ok, _ = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertFalse(ok)

    def test_an_exclusion_silently_becoming_an_inclusion_is_CAUGHT(self):
        """The most dangerous corruption: the no-fly zone becomes the only
        place it is allowed to fly."""
        got = self.sent()
        for w in got:
            if w.command == FENCE_POLYGON_EXCLUSION:
                w.command = FENCE_POLYGON_INCLUSION
        ok, _ = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertFalse(ok)

    def test_an_empty_read_back_is_CAUGHT(self):
        ok, why = compare_fences(self.sent(), [], HOME_LAT, HOME_LON)
        self.assertFalse(ok)

    def test_a_sub_metre_difference_is_tolerated(self):
        got = self.sent()
        got[0].x_lat += 1e-6                   # ~11 cm
        ok, why = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertTrue(ok, why)

    def test_signature_is_in_metres_not_degrees(self):
        sig = fence_signature(self.sent(), HOME_LAT, HOME_LON)
        self.assertTrue(any(abs(x) > 1.0 or abs(y) > 1.0 for _, x, y in sig))

    def test_tolerance_does_not_hide_a_real_move(self):
        """0.5 m is below anything that matters for a fence but well above
        int32 quantisation; a 2 m shift must still be caught."""
        got = self.sent()
        got[1].y_long += 2.0 / (6378137.0 * math.cos(math.radians(HOME_LAT))) \
            * (180.0 / math.pi)
        ok, why = compare_fences(self.sent(), got, HOME_LAT, HOME_LON)
        self.assertFalse(ok, "a 2 m vertex move slipped through the tolerance")


if __name__ == "__main__":
    unittest.main(verbosity=2)
