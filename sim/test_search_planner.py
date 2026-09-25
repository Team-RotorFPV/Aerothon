#!/usr/bin/env python3
"""Phase 6 — search geometry is derived, and coverage is proven.

WHAT THIS REPLACES (geometry audit A2, B1)
    search_alt = 10.0   "QR readable from 10 m" — a guess Phase 1 disproved
    spacing    = 6.0    while goal.md Q9 says 5.0; neither derived from the
                        camera, and they contradicted each other

    Phase 1 measured a decode floor of about 5.3 px per QR module, holding
    across a 3x resolution change and a 4.4x marker-size change. These tests
    hold the planner to that measurement and prove the lane plan actually
    covers the zone.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_search_planner.py -v
"""

import math
import time
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

from mission_bt.search_planner import (
    grow_zone,
    coverage_fraction,
    extend_zone,
    frontier_strip,
    ground_width,
    lane_spacing,
    max_decode_altitude,
    max_detect_altitude,
    plan_lawnmower,
    min_track_altitude,
    plan_search,
    px_per_module,
    vfov,
    zone_from_observation,
    leg_hits_exclusion,
    merge_exclusions,
    path_hits_exclusion,
    route_leg,
)

HFOV = math.radians(60.0)
MODULES = 33
ZONE = (20.0, 52.0, -12.0, 12.0)          # the simulated delivery zone


class GeometryTests(unittest.TestCase):

    def test_ground_width_matches_hand_calculation(self):
        # 2 * 10 * tan(30) = 11.547 m
        self.assertAlmostEqual(ground_width(10.0, HFOV), 11.547, places=3)

    def test_px_per_module_reproduces_the_phase1_measurement(self):
        """Phase 1, 640x480, 2.2 m pad at 7 m -> 5.28 px/module (measured)."""
        got = px_per_module(7.0, 640, HFOV, 2.2, MODULES)
        self.assertAlmostEqual(got, 5.28, places=1)

    def test_px_per_module_reproduces_the_1080p_measurement(self):
        """Phase 1, 1920x1080, 0.5 m marker at 5 m -> 5.04 px/module."""
        got = px_per_module(5.0, 1920, HFOV, 0.5, MODULES)
        self.assertAlmostEqual(got, 5.04, places=1)

    def test_decode_altitude_inverts_px_per_module(self):
        alt = max_decode_altitude(1920, HFOV, 0.5, MODULES, 5.04)
        self.assertAlmostEqual(alt, 5.0, places=1)

    def test_decode_altitude_scales_with_marker_size(self):
        small = max_decode_altitude(1920, HFOV, 0.3, MODULES, 5.3)
        large = max_decode_altitude(1920, HFOV, 0.6, MODULES, 5.3)
        self.assertAlmostEqual(large / small, 2.0, places=3)

    def test_the_old_constant_is_shown_to_be_wrong(self):
        """search_alt = 10.0 for a realistic marker at the simulated camera."""
        alt = max_decode_altitude(640, HFOV, 0.4, MODULES, 5.3)
        self.assertLess(alt, 2.0,
                        "a 0.4 m marker should NOT be decodable near 10 m at "
                        "640x480; the old constant assumed it was")

    def test_detecting_a_pad_works_higher_than_decoding_it(self):
        """The premise of sweep-then-descend."""
        decode = max_decode_altitude(1920, HFOV, 0.5, MODULES, 5.3)
        detect = max_detect_altitude(1920, HFOV, 0.5, 25.0)
        self.assertGreater(detect, decode)


class LaneTests(unittest.TestCase):

    def test_spacing_follows_altitude(self):
        self.assertAlmostEqual(lane_spacing(10.0, HFOV, overlap=0.0),
                               ground_width(10.0, HFOV), places=6)
        self.assertLess(lane_spacing(5.0, HFOV), lane_spacing(10.0, HFOV))

    def test_overlap_reduces_spacing(self):
        self.assertLess(lane_spacing(10.0, HFOV, 0.4),
                        lane_spacing(10.0, HFOV, 0.1))

    def test_lanes_are_boustrophedon(self):
        wps = plan_lawnmower(ZONE, 6.0, 10.0)
        self.assertEqual(wps[0][0], ZONE[0])
        self.assertEqual(wps[1][0], ZONE[1])
        self.assertEqual(wps[2][0], ZONE[1])       # reversed on the next lane
        self.assertEqual(wps[3][0], ZONE[0])

    def test_lanes_are_inset_from_the_zone_edges(self):
        """A lane centred on the boundary wastes half its swath outside."""
        wps = plan_lawnmower(ZONE, 6.0, 10.0)
        ys = [w[1] for w in wps]
        self.assertGreater(min(ys), ZONE[2])
        self.assertLess(max(ys), ZONE[3])

    def test_all_waypoints_are_inside_the_zone(self):
        for wp in plan_lawnmower(ZONE, 4.0, 8.0):
            self.assertGreaterEqual(wp[0], ZONE[0])
            self.assertLessEqual(wp[0], ZONE[1])
            self.assertGreaterEqual(wp[1], ZONE[2])
            self.assertLessEqual(wp[1], ZONE[3])

    def test_zero_spacing_is_rejected(self):
        with self.assertRaises(ValueError):
            plan_lawnmower(ZONE, 0.0, 10.0)


class CoverageProofTests(unittest.TestCase):
    """The sweep is only meaningful if every point is actually seen."""

    def test_derived_spacing_gives_full_coverage(self):
        alt = 10.0
        spacing = lane_spacing(alt, HFOV, overlap=0.30)
        cov = coverage_fraction(ZONE, spacing, alt, HFOV)
        self.assertAlmostEqual(cov, 1.0, places=3,
                               msg=f"derived spacing left gaps: {cov:.3f}")

    def test_too_wide_a_spacing_leaves_gaps_and_is_detected(self):
        """The proof has to be capable of failing."""
        alt = 5.0
        cov = coverage_fraction(ZONE, spacing=20.0, altitude_m=alt, hfov_rad=HFOV)
        self.assertLess(cov, 0.95,
                        "a 20 m spacing at 5 m altitude cannot cover the zone")

    def test_the_old_constants_are_checked_against_their_own_altitude(self):
        """spacing=6.0 at the old search_alt=10.0 happens to cover — but only
        because the swath at 10 m is 11.5 m wide. Drop to a realistic decode
        altitude and the same spacing fails."""
        self.assertAlmostEqual(coverage_fraction(ZONE, 6.0, 10.0, HFOV), 1.0,
                               places=3)
        self.assertLess(coverage_fraction(ZONE, 6.0, 2.0, HFOV), 0.8)


class PlanSearchTests(unittest.TestCase):

    def test_plan_is_self_consistent(self):
        plan = plan_search(ZONE, 1920, HFOV, 0.5, MODULES)
        self.assertGreater(plan["sweep_alt_m"], 0)
        self.assertGreaterEqual(plan["sweep_alt_m"], plan["decode_alt_m"])
        self.assertAlmostEqual(plan["coverage"], 1.0, places=2)
        self.assertGreater(plan["n_lanes"], 0)

    def test_plan_requires_descending_for_a_small_marker(self):
        plan = plan_search(ZONE, 1920, HFOV, 0.4, MODULES)
        self.assertTrue(plan["descend_to_decode"],
                        "a small marker must require a descent to decode")

    def test_large_marker_may_not_need_to_descend(self):
        plan = plan_search(ZONE, 1920, HFOV, 2.2, MODULES, max_altitude=6.0)
        self.assertFalse(plan["descend_to_decode"])

    def test_altitude_cap_is_respected(self):
        plan = plan_search(ZONE, 1920, HFOV, 0.5, MODULES, max_altitude=8.0)
        self.assertLessEqual(plan["sweep_alt_m"], 8.0 + 1e-9)

    def test_the_cap_WINS_over_the_decode_altitude(self):
        """The regression from live run 12.

        A large marker is decodable from higher than the rulebook lets the
        aircraft fly. The planner used to answer "then sweep at 13.9 m", which
        is a rulebook violation dressed up as an optimisation. The cap is a
        ceiling, not a suggestion.
        """
        plan = plan_search(ZONE, 1280, HFOV, 2.2, MODULES, max_altitude=10.0)
        self.assertGreater(plan["decode_alt_m"], 10.0,
                           "fixture no longer exercises the cap-vs-decode case")
        self.assertLessEqual(plan["sweep_alt_m"], 10.0 + 1e-9)

    def test_sweeping_below_decode_altitude_needs_no_further_descent(self):
        """Capped below decode_alt is safe: lower only makes decoding easier,
        and the plan must say so rather than ordering a pointless descent."""
        plan = plan_search(ZONE, 1280, HFOV, 2.2, MODULES, max_altitude=10.0)
        self.assertLess(plan["sweep_alt_m"], plan["decode_alt_m"])
        self.assertFalse(plan["descend_to_decode"])

    def test_uncapped_plans_still_never_sweep_below_decode_altitude(self):
        """Without a ceiling the original optimisation still holds."""
        plan = plan_search(ZONE, 1920, HFOV, 0.5, MODULES)
        self.assertGreaterEqual(plan["sweep_alt_m"], plan["decode_alt_m"] - 1e-9)

    def test_marker_size_drives_the_whole_plan(self):
        """The organiser's unknown answer is an INPUT, not a constant."""
        small = plan_search(ZONE, 1920, HFOV, 0.3, MODULES)
        large = plan_search(ZONE, 1920, HFOV, 1.0, MODULES)
        self.assertLess(small["decode_alt_m"], large["decode_alt_m"])
        self.assertLessEqual(small["n_lanes"], large["n_lanes"] * 4)


class FrontierTests(unittest.TestCase):
    """The observed window is bounded by LIDAR RANGE, not by the zone.

    In the reference arena the lidar reports 12 m of open ground for a
    delivery zone that is 40 m deep, so `zone_from_observation` returns a 12 m
    window. Sweeping it once and giving up is only correct if the target
    happens to be inside — which, for every live run to date, it was, because
    they were all started with target C.
    """

    def test_the_observed_window_is_much_smaller_than_the_real_zone(self):
        """The premise, measured rather than asserted."""
        window = zone_from_observation((17.0, 0.0), 0.0, 12.0, 16.6,
                                       margin_m=1.0)
        self.assertLess(window[1], 30.0)
        # Targets B (47, 10), D (33, -10) and E (45, -6) are all outside it.
        for name, (tx, _ty) in {"B": (47, 10), "D": (33, -10),
                                "E": (45, -6)}.items():
            self.assertGreater(tx, window[1],
                               f"target {name} was expected outside the window")

    def test_extend_zone_pushes_the_far_edge_along_the_heading(self):
        self.assertEqual(extend_zone((10.0, 20.0, -5.0, 5.0), 0.0, 15.0),
                         (10.0, 35.0, -5.0, 5.0))

    def test_extend_zone_keeps_the_ground_already_covered(self):
        """A union, not a translation: the aircraft does not un-see the window
        it just swept."""
        z = extend_zone((10.0, 20.0, -5.0, 5.0), math.pi, 8.0)
        for got, want in zip(z, (2.0, 20.0, -5.0, 5.0)):
            self.assertAlmostEqual(got, want, places=6)

    def test_extend_zone_handles_a_diagonal_heading(self):
        z = extend_zone((0.0, 10.0, 0.0, 10.0), math.radians(90.0), 6.0)
        self.assertAlmostEqual(z[3], 16.0, places=6)
        self.assertAlmostEqual(z[1], 10.0, places=6)

    def test_the_strip_is_only_the_NEW_ground(self):
        """Re-sweeping the whole extended zone would re-fly covered ground."""
        strip = frontier_strip((10.0, 22.0, -5.0, 5.0), 0.0, 12.0)
        self.assertEqual(strip, (22.0, 34.0, -5.0, 5.0))

    def test_strips_tile_the_ground_without_gaps(self):
        """Three successive advances must leave no unswept band between them."""
        window = (10.0, 22.0, -5.0, 5.0)
        covered = [window]
        frontier = window
        for _ in range(3):
            frontier = frontier_strip(frontier, 0.0, 12.0)
            covered.append(frontier)
        for a, b in zip(covered, covered[1:]):
            self.assertAlmostEqual(a[1], b[0], places=6,
                                   msg=f"gap between {a} and {b}")
        self.assertAlmostEqual(covered[-1][1], 58.0, places=6)

    def test_enough_advances_reach_the_far_targets(self):
        """The point of the whole mechanism."""
        frontier = zone_from_observation((17.0, 0.0), 0.0, 12.0, 16.6,
                                         margin_m=1.0)
        reach = frontier[1]
        for _ in range(4):
            frontier = frontier_strip(frontier, 0.0, 12.0)
            reach = max(reach, frontier[1])
        for name, tx in {"B": 47.0, "D": 33.0, "E": 45.0}.items():
            self.assertGreater(reach, tx, f"target {name} still unreachable")


if __name__ == "__main__":
    unittest.main(verbosity=2)


class TrackingFloorTests(unittest.TestCase):
    """The bottom of the tracking envelope, opposite max_decode_altitude().

    `land_commit_alt` was a flat 1.5 m. For the 2.2 m pad the marker overflows
    the frame below about 2.5 m, so PrecisionDescent was told to hold lock at
    an altitude where the camera cannot see the whole marker. Live runs 12 and
    14 both oscillated between 3.4 m and 4.1 m until the stage timed out.
    """

    def test_the_2_2_m_pad_cannot_be_tracked_down_to_1_5_m(self):
        """The premise, computed rather than asserted."""
        floor = min_track_altitude(2.2, HFOV, 1280, 720)
        self.assertGreater(floor, 1.5,
                           "if this is below 1.5 m the old constant was fine")
        self.assertLess(floor, 4.0)

    def test_the_limiting_dimension_is_the_SHORT_one(self):
        """A marker that fits across the frame but not down it is clipped."""
        landscape = min_track_altitude(2.2, HFOV, 1280, 720)
        square = min_track_altitude(2.2, HFOV, 1280, 1280)
        self.assertGreater(landscape, square)

    def test_a_bigger_marker_must_be_left_higher(self):
        self.assertGreater(min_track_altitude(3.0, HFOV, 1280, 720),
                           min_track_altitude(1.0, HFOV, 1280, 720))

    def test_a_wider_lens_can_come_lower(self):
        wide = min_track_altitude(2.2, math.radians(90.0), 1280, 720)
        narrow = min_track_altitude(2.2, math.radians(40.0), 1280, 720)
        self.assertLess(wide, narrow)

    def test_the_marker_really_does_fit_at_the_floor(self):
        """Checked against the projection, not against itself: at the floor
        the marker must fit the SHORT frame dimension, with the margin."""
        floor = min_track_altitude(2.2, HFOV, 1280, 720, margin=1.0)
        v = vfov(HFOV, 1280, 720)
        self.assertAlmostEqual(2.0 * floor * math.tan(v / 2.0), 2.2, places=6)

    def test_it_does_NOT_fit_below_the_floor(self):
        floor = min_track_altitude(2.2, HFOV, 1280, 720, margin=1.0)
        v = vfov(HFOV, 1280, 720)
        self.assertLess(2.0 * (floor - 0.3) * math.tan(v / 2.0), 2.2)

    def test_the_tracking_envelope_is_not_empty(self):
        """Decode ceiling above tracking floor, or there is no altitude at
        which the marker can be both seen whole and read."""
        floor = min_track_altitude(2.2, HFOV, 1280, 720)
        ceiling = max_decode_altitude(1280, HFOV, 2.2, MODULES, 5.3)
        self.assertLess(floor, ceiling)


class GrowZoneTests(unittest.TestCase):
    """Growing the search sideways as well as forward.

    frontier_strip() only ever advanced along the corridor heading. Seed 1002
    swept four strips out to x = 74.7 and never found pad E, because the pad
    was not further down the corridor -- it was off to one side.
    """

    Z = (10.0, 30.0, -5.0, 5.0)

    def test_forward_growth_extends_the_far_edge(self):
        grown, band = grow_zone(self.Z, 0.0, 8.0)
        self.assertAlmostEqual(grown[1], 38.0)
        self.assertAlmostEqual(band[0], 30.0)
        self.assertAlmostEqual(band[1], 38.0)

    def test_left_growth_extends_the_y_edge(self):
        grown, band = grow_zone(self.Z, math.pi / 2, 6.0)
        self.assertAlmostEqual(grown[3], 11.0)
        self.assertAlmostEqual(band[2], 5.0)
        self.assertAlmostEqual(band[3], 11.0)

    def test_right_growth_extends_the_other_y_edge(self):
        grown, band = grow_zone(self.Z, -math.pi / 2, 6.0)
        self.assertAlmostEqual(grown[2], -11.0)
        self.assertAlmostEqual(band[2], -11.0)
        self.assertAlmostEqual(band[3], -5.0)

    def test_backward_growth_extends_the_near_edge(self):
        grown, band = grow_zone(self.Z, math.pi, 4.0)
        self.assertAlmostEqual(grown[0], 6.0)
        self.assertAlmostEqual(band[0], 6.0)
        self.assertAlmostEqual(band[1], 10.0)

    def test_the_band_is_NEW_ground_not_a_re_sweep(self):
        """Each expansion must cost one band, not the whole search so far."""
        for direction in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            _, band = grow_zone(self.Z, direction, 7.0)
            overlap_x = max(0.0, min(band[1], self.Z[1]) - max(band[0], self.Z[0]))
            overlap_y = max(0.0, min(band[3], self.Z[3]) - max(band[2], self.Z[2]))
            self.assertAlmostEqual(overlap_x * overlap_y, 0.0, places=6,
                                   msg=f"band overlaps the swept zone at {direction}")

    def test_the_grown_zone_CONTAINS_the_original(self):
        for direction in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            grown, _ = grow_zone(self.Z, direction, 5.0)
            self.assertLessEqual(grown[0], self.Z[0] + 1e-9)
            self.assertGreaterEqual(grown[1], self.Z[1] - 1e-9)
            self.assertLessEqual(grown[2], self.Z[2] + 1e-9)
            self.assertGreaterEqual(grown[3], self.Z[3] - 1e-9)

    def test_growth_follows_a_ROTATED_corridor_heading(self):
        """The corridor is randomised in heading; forward is not always +x."""
        grown, _ = grow_zone(self.Z, math.radians(90), 10.0)
        self.assertAlmostEqual(grown[3], 15.0)
        self.assertAlmostEqual(grown[1], 30.0)

    def test_a_lateral_band_can_reach_ground_forward_growth_never_would(self):
        """Seed 1002's shape of failure, stated as geometry: a pad beside the
        corridor is unreachable by any number of forward steps."""
        pad = (20.0, 14.0)
        zone = self.Z
        for _ in range(6):
            zone, _ = grow_zone(zone, 0.0, 10.0)
        self.assertFalse(zone[2] <= pad[1] <= zone[3],
                         "forward-only growth unexpectedly covered the pad")
        zone, _ = grow_zone(zone, math.pi / 2, 12.0)
        self.assertTrue(zone[2] <= pad[1] <= zone[3],
                        "lateral growth still does not cover the pad")


# --------------------------------------------------------------------------- #
# Exclusion-aware transit legs
# --------------------------------------------------------------------------- #

class LegRoutingTests(unittest.TestCase):
    """Every leg the AIRFRAME flies must clear confirmed red ground.

    WHAT THIS REPLACES

        Only the search sweep clipped its lanes against exclusions. Every
        straight leg -- corridor exit to the observed zone, approach to the
        matched pad, the reposition for descent-to-decode, the return to the
        corridor mouth, go-home -- flew point to point with no exclusion check
        at all. A watched flight had 198 confirmed exclusion cells and the
        aircraft still crossed red ground, which is -5 marks a time.

        The retracted diagnosis is worth remembering: the detector was never
        the problem. Exclusions existed. Nothing downstream of the sweep asked
        about them.
    """

    CLEAR = 1.0
    BOX = [(10.0, 20.0, -5.0, 5.0)]       # one exclusion straddling the x axis

    # ---- the primitive itself ---- #
    def test_a_clear_leg_is_flown_straight(self):
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, [])
        self.assertTrue(r["ok"])
        self.assertFalse(r["detoured"])
        self.assertEqual(r["waypoints"], [(30.0, 0.0)])

    def test_a_leg_that_misses_every_exclusion_is_flown_straight(self):
        r = route_leg((0.0, 30.0), (30.0, 30.0), self.CLEAR, self.BOX)
        self.assertTrue(r["ok"])
        self.assertFalse(r["detoured"])

    def test_a_leg_straight_through_an_exclusion_is_detoured(self):
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, self.BOX)
        self.assertTrue(r["ok"], r["reason"])
        self.assertTrue(r["detoured"])
        self.assertGreater(len(r["waypoints"]), 1)

    def test_the_detour_actually_clears_the_exclusion(self):
        """The test that matters: not 'a detour happened' but 'the path is clean'."""
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, self.BOX)
        path = [(0.0, 0.0)] + r["waypoints"]
        self.assertFalse(path_hits_exclusion(path, self.CLEAR, self.BOX),
                         f"routed path still crosses red ground: {path}")

    def test_the_detour_ends_where_it_was_told_to(self):
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, self.BOX)
        self.assertAlmostEqual(r["waypoints"][-1][0], 30.0)
        self.assertAlmostEqual(r["waypoints"][-1][1], 0.0)

    def test_the_detour_is_not_absurdly_long(self):
        """A correct but 400 m detour is a time-limit failure, 15 marks."""
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, self.BOX)
        self.assertLess(r["length_m"], 3.0 * 30.0)

    def test_it_routes_the_short_way_round(self):
        """Exclusion offset north of the leg -> going south is shorter."""
        box = [(10.0, 20.0, -2.0, 20.0)]
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, box)
        self.assertTrue(r["ok"])
        self.assertTrue(all(y < 0.0 for _, y in r["waypoints"][:-1]),
                        f"went the long way: {r['waypoints']}")

    # ---- clearance is an AIRFRAME clearance ---- #
    def test_a_leg_inside_the_clearance_band_is_detoured(self):
        """Passing 0.5 m from red ground with a 1.0 m airframe clearance is a
        violation even though the flight line itself misses."""
        r = route_leg((0.0, 5.5), (30.0, 5.5), self.CLEAR, self.BOX)
        self.assertTrue(r["detoured"])

    def test_a_leg_outside_the_clearance_band_is_left_alone(self):
        r = route_leg((0.0, 6.5), (30.0, 6.5), self.CLEAR, self.BOX)
        self.assertFalse(r["detoured"])

    def test_clearance_is_not_the_camera_swath(self):
        """A leg 8 m to the side is fine even though the camera sees the zone.

        Clipping against what the camera can see was tried once and measured
        0.81 coverage where 1.0 was reachable. A red zone constrains the
        aircraft, not the lens.
        """
        r = route_leg((0.0, 13.0), (30.0, 13.0), self.CLEAR, self.BOX)
        self.assertFalse(r["detoured"])

    # ---- fail closed ---- #
    def test_a_destination_inside_an_exclusion_is_refused(self):
        r = route_leg((0.0, 0.0), (15.0, 0.0), self.CLEAR, self.BOX)
        self.assertFalse(r["ok"])
        self.assertIn("destination", r["reason"].lower())
        self.assertIsNone(r["waypoints"])

    def test_a_refusal_says_which_leg_and_how_many_zones(self):
        r = route_leg((0.0, 0.0), (15.0, 0.0), self.CLEAR, self.BOX)
        self.assertIn("15.0", r["reason"])
        self.assertIn("1", r["reason"])

    def test_a_walled_off_destination_is_refused_rather_than_flown_at(self):
        """Exclusions across the whole corridor of travel: no way through."""
        wall = [(10.0, 12.0, -400.0, 400.0)]
        r = route_leg((0.0, 0.0), (30.0, 0.0), self.CLEAR, wall)
        self.assertFalse(r["ok"])
        self.assertIsNone(r["waypoints"])

    def test_starting_inside_an_exclusion_still_yields_a_way_out(self):
        """Already over red ground when it is confirmed. Leaving is the fix;
        refusing to move would hold the aircraft over the violation."""
        r = route_leg((15.0, 0.0), (30.0, 0.0), self.CLEAR, self.BOX)
        self.assertTrue(r["ok"], r["reason"])
        self.assertAlmostEqual(r["waypoints"][-1][0], 30.0)

    # ---- shape of the real input ---- #
    def test_it_survives_the_cell_count_a_real_flight_produces(self):
        """198 confirmed cells was the live count. This runs on every tick."""
        cells = []
        for i in range(14):
            for j in range(14):
                x = 12.0 + i * 1.0
                y = -7.0 + j * 1.0
                cells.append((x, x + 1.0, y, y + 1.0))
        self.assertGreaterEqual(len(cells), 196)
        start = time.monotonic()
        r = route_leg((0.0, 0.0), (40.0, 0.0), self.CLEAR, cells)
        elapsed = time.monotonic() - start
        self.assertTrue(r["ok"], r["reason"])
        self.assertLess(elapsed, 0.25, f"routing took {elapsed:.3f} s")
        path = [(0.0, 0.0)] + r["waypoints"]
        self.assertFalse(path_hits_exclusion(path, self.CLEAR, cells))

    def test_adjacent_cells_merge_into_one_obstacle(self):
        cells = [(0.0, 1.0, 0.0, 1.0), (1.0, 2.0, 0.0, 1.0),
                 (2.0, 3.0, 0.0, 1.0), (50.0, 51.0, 50.0, 51.0)]
        merged = merge_exclusions(cells, 0.0)
        self.assertEqual(len(merged), 2)
        self.assertIn((0.0, 3.0, 0.0, 1.0), merged)

    def test_merging_is_conservative_never_smaller_than_its_parts(self):
        """An L-shaped zone merges to its bounding box, which forbids some
        flyable ground. Over-forbidding costs coverage; under-forbidding costs
        marks. Only one of those is recoverable."""
        cells = [(0.0, 10.0, 0.0, 2.0), (0.0, 2.0, 0.0, 10.0)]
        merged = merge_exclusions(cells, 0.0)
        self.assertEqual(len(merged), 1)
        x0, x1, y0, y1 = merged[0]
        for cx0, cx1, cy0, cy1 in cells:
            self.assertLessEqual(x0, cx0)
            self.assertGreaterEqual(x1, cx1)
            self.assertLessEqual(y0, cy0)
            self.assertGreaterEqual(y1, cy1)

    # ---- the segment predicate the stages will use ---- #
    def test_leg_predicate_agrees_with_the_lane_clipper(self):
        self.assertTrue(leg_hits_exclusion((0.0, 0.0), (30.0, 0.0),
                                           self.CLEAR, self.BOX))
        self.assertFalse(leg_hits_exclusion((0.0, 30.0), (30.0, 30.0),
                                            self.CLEAR, self.BOX))

    def test_leg_predicate_catches_a_diagonal_clip_of_a_corner(self):
        """A diagonal that only nicks a corner is still a violation, and is
        exactly what an axis-aligned lane check misses."""
        self.assertTrue(leg_hits_exclusion((0.0, -20.0), (30.0, 20.0),
                                           self.CLEAR, self.BOX))

    def test_a_leg_that_stops_short_of_the_zone_does_not_hit_it(self):
        self.assertFalse(leg_hits_exclusion((0.0, 0.0), (8.0, 0.0),
                                            self.CLEAR, self.BOX))

    def test_no_exclusions_means_no_hits(self):
        self.assertFalse(leg_hits_exclusion((0.0, 0.0), (30.0, 0.0),
                                            self.CLEAR, []))
        self.assertFalse(path_hits_exclusion(
            [(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)], self.CLEAR, []))
