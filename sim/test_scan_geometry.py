#!/usr/bin/env python3
"""Squareness to a flat surface, measured from one lidar scan.

WHAT THIS REPLACES

    Squareness used to be inferred from the aspect ratio of a camera-derived
    bounding box. That box includes the gate posts, so it plateaus at 1.88 to
    1.91 however square the aircraft is; a threshold of 2.00 was unreachable
    and a threshold of 1.75 was satisfied by a single noisy narrowing at 1.4.
    Fourteen watched runs failed on it, twice by flying out of the world.

    "Am I perpendicular to that surface" is a question a lidar answers
    directly. Fit a line through the returns in the sector the camera points
    at; the angle of that line to the nose IS the misalignment, in radians,
    with no proxy and no tuning constant between the measurement and the
    answer.

WHAT THESE TESTS ARE CAREFUL ABOUT

    THE SIGN. A sign error here puts the aircraft on the wrong side of the
    gate, and this project has lost flights to exactly that. Every geometry
    case is asserted with its sign, and there is a test that a wall to port
    and the same wall to starboard do not report the same number.

    THE REFUSAL. The function has to say no. A fit through four noisy returns
    that happen to line up is not a measurement, and the stage above this one
    commits a 10 m waypoint on the answer.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_scan_geometry.py -v
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

from mission_bt.scan_geometry import (                        # noqa: E402
    bearing_to_angle,
    fit_surface,
    gate_opening,
)


# --------------------------------------------------------------------------- #
# Synthetic scans. Everything is built by RAY CASTING against a described
# world, never by writing the answer into the ranges: a fixture that encodes
# the expected geometry directly cannot fail for a geometric reason.
# --------------------------------------------------------------------------- #
SAMPLES = 720
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / SAMPLES
RANGE_MAX = 12.0


def _segment_range(angle, p0, p1):
    """Range to a wall segment p0->p1 along `angle`, or None if it misses."""
    dx, dy = math.cos(angle), math.sin(angle)
    sx, sy = p1[0] - p0[0], p1[1] - p0[1]
    den = dx * sy - dy * sx
    if abs(den) < 1e-12:
        return None
    t = ((p0[0] * sy - p0[1] * sx)) / den          # along the ray
    u = ((p0[0] * dy - p0[1] * dx)) / den          # along the segment
    if t <= 0.0 or not (0.0 <= u <= 1.0):
        return None
    return t


def scan_of(segments, noise_m=0.0, seed=7, samples=SAMPLES):
    """Ray-cast `segments` (each ((x0,y0),(x1,y1))) into a range array.

    Coordinates are in the AIRCRAFT frame: +x out of the nose, +y to port,
    which is the ROS LaserScan convention the avoidance node already uses.
    """
    rng = random.Random(seed)
    inc = 2 * math.pi / samples
    ranges = []
    for i in range(samples):
        a = ANGLE_MIN + i * inc
        hits = [r for r in (_segment_range(a, p0, p1) for p0, p1 in segments)
                if r is not None and r < RANGE_MAX]
        if not hits:
            ranges.append(float("inf"))
            continue
        r = min(hits)
        if noise_m:
            r += rng.gauss(0.0, noise_m)
        ranges.append(r)
    return ranges


def wall(distance_m, tilt_rad, half_length_m=1.85, offset_m=0.0):
    """A flat face whose perpendicular foot lies `tilt_rad` off the nose.

    `tilt_rad` is the answer the fit must recover: the direction from the
    aircraft to the nearest point of the surface. `offset_m` slides the panel
    along its own face without moving that perpendicular, which is what an
    aircraft sitting off the gate's centreline sees.
    """
    fx, fy = distance_m * math.cos(tilt_rad), distance_m * math.sin(tilt_rad)
    ux, uy = -math.sin(tilt_rad), math.cos(tilt_rad)      # along the face
    a = (fx + (offset_m - half_length_m) * ux,
         fy + (offset_m - half_length_m) * uy)
    b = (fx + (offset_m + half_length_m) * ux,
         fy + (offset_m + half_length_m) * uy)
    return (a, b)


def post(distance_m, tilt_rad, along_m, width_m=0.18):
    """One square gate post, `along_m` from the perpendicular foot."""
    return wall(distance_m, tilt_rad, half_length_m=width_m / 2.0,
                offset_m=along_m)


def fit(ranges, bearing=0.0, half_width=math.radians(35.0), **kw):
    return fit_surface(ANGLE_MIN, ANGLE_INC, ranges, bearing, half_width, **kw)


# --------------------------------------------------------------------------- #
class FlatSurfaceTests(unittest.TestCase):
    """The measurement itself, on a solid board."""

    def test_a_wall_dead_ahead_reads_ZERO_misalignment(self):
        f = fit(scan_of([wall(5.0, 0.0)]))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), 0.0, delta=0.5)
        self.assertAlmostEqual(f["range_m"], 5.0, delta=0.05)

    def test_a_wall_thirty_degrees_to_PORT_reads_plus_thirty(self):
        f = fit(scan_of([wall(5.0, math.radians(30.0))]),
                bearing=math.radians(30.0))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), 30.0, delta=1.0)

    def test_a_wall_thirty_degrees_to_STARBOARD_reads_minus_thirty(self):
        """The sign is the whole point. Getting it backwards puts the aircraft
        on the far side of the gate from the one it was trying to reach."""
        f = fit(scan_of([wall(5.0, math.radians(-30.0))]),
                bearing=math.radians(-30.0))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), -30.0, delta=1.0)

    def test_port_and_starboard_do_not_report_the_same_number(self):
        port = fit(scan_of([wall(5.0, math.radians(20.0))]),
                   bearing=math.radians(20.0))
        stbd = fit(scan_of([wall(5.0, math.radians(-20.0))]),
                   bearing=math.radians(-20.0))
        self.assertGreater(port["angle_rad"], 0.0)
        self.assertLess(stbd["angle_rad"], 0.0)

    def test_the_range_is_the_PERPENDICULAR_distance_not_the_nearest_return(self):
        """Sliding along the face changes which return is closest and does not
        change how far off the surface the aircraft is standing."""
        centred = fit(scan_of([wall(6.0, 0.0)]))
        offset = fit(scan_of([wall(6.0, 0.0, offset_m=1.5)]))
        self.assertTrue(offset["ok"], offset["reason"])
        self.assertAlmostEqual(centred["range_m"], offset["range_m"], delta=0.06)

    def test_the_angle_does_not_depend_on_where_along_the_face_we_are(self):
        """Squareness is about HEADING. An aircraft square to the face but off
        to one side of it is still square to the face."""
        a = fit(scan_of([wall(6.0, 0.0, offset_m=0.0)]))
        b = fit(scan_of([wall(6.0, 0.0, offset_m=1.6)]))
        self.assertAlmostEqual(math.degrees(a["angle_rad"]),
                               math.degrees(b["angle_rad"]), delta=1.0)

    def test_it_survives_the_sensors_stated_five_millimetre_noise(self):
        for seed in range(6):
            f = fit(scan_of([wall(5.0, math.radians(12.0))], noise_m=0.005,
                            seed=seed),
                    bearing=math.radians(12.0))
            self.assertTrue(f["ok"], f["reason"])
            self.assertAlmostEqual(math.degrees(f["angle_rad"]), 12.0, delta=2.0)

    def test_the_fit_reports_how_many_returns_it_used(self):
        f = fit(scan_of([wall(5.0, 0.0)]))
        self.assertGreater(f["points"], 10)
        self.assertLess(f["residual_m"], 0.02)


# --------------------------------------------------------------------------- #
class BarePostPairTests(unittest.TestCase):
    """The same algorithm, on a gate with no panel between the posts.

    The simulated board only gained collision geometry with this change; a
    real gate may well be an open frame. A line fit handles both, which is why
    it was chosen over a two-post special case.
    """

    def _posts(self, tilt, distance=5.0):
        return [post(distance, tilt, -1.92), post(distance, tilt, +1.92)]

    def test_two_posts_alone_define_the_same_surface_a_solid_board_does(self):
        solid = fit(scan_of([wall(5.0, math.radians(18.0))]),
                    bearing=math.radians(18.0))
        posts = fit(scan_of(self._posts(math.radians(18.0))),
                    bearing=math.radians(18.0))
        self.assertTrue(posts["ok"], posts["reason"])
        self.assertAlmostEqual(math.degrees(posts["angle_rad"]),
                               math.degrees(solid["angle_rad"]), delta=2.5)

    def test_a_post_pair_dead_ahead_reads_zero(self):
        f = fit(scan_of(self._posts(0.0)))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), 0.0, delta=1.5)

    def test_ONE_post_is_not_a_surface(self):
        """An 18 cm return cannot fix an orientation. Reporting an angle from
        it would be asserting a geometric fact the scan does not contain.

        At the 5 m standoff a single post draws three returns, so this refuses
        on the count. The extent guard below is what catches the same mistake
        from close in, where three returns become fourteen and the span is
        still 18 cm."""
        f = fit(scan_of([post(5.0, 0.0, -1.92)]))
        self.assertFalse(f["ok"])
        self.assertIn("3 lidar return", f["reason"])

    def test_a_SHORT_surface_is_refused_however_many_returns_it_draws(self):
        """From 1.5 m one post draws a dozen returns and still spans 18 cm.
        A count threshold alone would call that a face."""
        f = fit(scan_of([post(1.5, 0.0, 0.0)]))
        self.assertFalse(f["ok"], f)
        self.assertIn("span", f["reason"])

    def test_the_wall_BEHIND_an_open_gate_does_not_join_the_fit(self):
        """A corridor wall visible through the opening sits metres further
        back. Letting it into the fit is how a surface two metres behind the
        gate masquerades as the gate."""
        segs = self._posts(0.0) + [wall(8.0, 0.0, half_length_m=6.0)]
        f = fit(scan_of(segs))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(f["range_m"], 5.0, delta=0.15)


# --------------------------------------------------------------------------- #
class RefusalTests(unittest.TestCase):
    """It has to be able to say no, and to say WHY.

    The stage above commits a 10 m waypoint on this answer. "No surface found"
    on its own is not diagnosable from a run artifact.
    """

    def test_an_empty_sector_is_refused(self):
        f = fit([float("inf")] * SAMPLES)
        self.assertFalse(f["ok"])
        self.assertIn("return", f["reason"])

    def test_a_surface_OUTSIDE_the_sector_is_not_found(self):
        """The sector comes from the camera bearing precisely so that
        something the camera is not looking at cannot be measured instead."""
        f = fit(scan_of([wall(5.0, math.radians(80.0))]),
                bearing=0.0, half_width=math.radians(30.0))
        self.assertFalse(f["ok"], f)

    def test_a_curved_scatter_is_refused_rather_than_averaged(self):
        """Six panels arranged around an arc are not a flat face. A line fit
        through them has a large residual and must be reported as a refusal,
        not as a surface at the average angle."""
        segs = [wall(4.0 + 0.9 * k, math.radians(-24.0 + 9.0 * k),
                     half_length_m=0.35)
                for k in range(6)]
        f = fit(scan_of(segs))
        self.assertFalse(f["ok"], f)

    def test_a_range_far_from_the_cameras_estimate_is_refused(self):
        f = fit(scan_of([wall(9.0, 0.0, half_length_m=6.0)]),
                expected_range_m=4.0, range_tol_m=2.0)
        self.assertFalse(f["ok"])
        self.assertIn("9.0", f["reason"])

    def test_a_range_NEAR_the_cameras_estimate_is_accepted(self):
        f = fit(scan_of([wall(5.0, 0.0)]),
                expected_range_m=4.4, range_tol_m=2.0)
        self.assertTrue(f["ok"], f["reason"])

    def test_every_refusal_names_its_own_reason(self):
        empty = fit([float("inf")] * SAMPLES)
        single = fit(scan_of([post(5.0, 0.0, 0.0)]))
        self.assertNotEqual(empty["reason"], single["reason"])
        for f in (empty, single):
            self.assertTrue(f["reason"], "a refusal with no reason")
            self.assertIsNone(f["angle_rad"])
            self.assertIsNone(f["range_m"])


# --------------------------------------------------------------------------- #
class EdgeOnSurfacesTests(unittest.TestCase):
    """A wall running away ALONGSIDE the aircraft is not a wall it is facing.

    MEASURED on the ground, shipped arena, aircraft parked and the fit asked
    what it saw in each direction in turn:

        sector -75:  face  -90.0 deg  7.74 m  17 pts  "span" 2.24 m
        sector +15:  face  -90.9 deg  3.11 m   8 pts  "span" 2.22 m

    Eight returns do not span two metres of anything. The fit had gone RADIAL,
    running toward the aircraft rather than across it, so the spread along it
    was depth. In flight the same readings came back as an alternating
    +90/-90 measurement and the aircraft turned in circles for fourteen steps
    before its step budget ran out.
    """

    def test_a_wall_running_away_alongside_is_REFUSED_not_reported_as_90(self):
        corridor = ((1.5, 1.6), (11.0, 1.6))
        f = fit(scan_of([corridor]), bearing=math.radians(20.0))
        self.assertFalse(f["ok"], f)

    def test_the_refusal_says_edge_on_rather_than_something_vaguer(self):
        corridor = ((1.5, 1.6), (11.0, 1.6))
        f = fit(scan_of([corridor]), bearing=math.radians(20.0))
        self.assertTrue("edge-on" in f["reason"] or "span" in f["reason"],
                        f["reason"])

    def test_the_EXTENT_is_measured_across_the_line_of_sight(self):
        """Depth must not be able to stand in for width. A post seen from
        close in smears through several samples; it is still 18 cm wide."""
        f = fit(scan_of([post(1.5, 0.0, 0.0)]))
        self.assertFalse(f["ok"], f)
        self.assertIn("span", f["reason"])

    def test_a_gate_seen_at_FIFTY_degrees_is_still_measured(self):
        """The guard has to reject edge-on without rejecting oblique: coming
        round to the face is the whole manoeuvre, and it starts off-axis."""
        f = fit(scan_of([wall(5.0, math.radians(50.0))]),
                bearing=math.radians(50.0))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), 50.0, delta=2.0)


# --------------------------------------------------------------------------- #
class TwoSurfacesTests(unittest.TestCase):
    """The nearest coherent surface wins. The gate stands in front of things."""

    def test_the_NEARER_of_two_parallel_walls_is_the_one_measured(self):
        segs = [wall(4.0, 0.0, half_length_m=1.85),
                wall(7.0, 0.0, half_length_m=6.0)]
        f = fit(scan_of(segs))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(f["range_m"], 4.0, delta=0.1)

    def test_a_wall_behind_at_a_DIFFERENT_angle_does_not_bend_the_fit(self):
        segs = [wall(4.0, 0.0, half_length_m=1.85),
                wall(7.5, math.radians(20.0), half_length_m=6.0)]
        f = fit(scan_of(segs))
        self.assertTrue(f["ok"], f["reason"])
        self.assertAlmostEqual(math.degrees(f["angle_rad"]), 0.0, delta=2.0)


# --------------------------------------------------------------------------- #
class BearingConversionTests(unittest.TestCase):
    """A camera bearing is a fraction of the half-FOV, not an angle."""

    def test_frame_centre_is_straight_ahead(self):
        self.assertAlmostEqual(bearing_to_angle(0.0, math.radians(60.0)), 0.0)

    def test_a_banner_to_the_RIGHT_of_frame_is_a_NEGATIVE_lidar_angle(self):
        """Image +x is to the right; a positive lidar angle is to port. The
        two conventions disagree and the conversion is where that is stated."""
        self.assertLess(bearing_to_angle(0.5, math.radians(60.0)), 0.0)
        self.assertGreater(bearing_to_angle(-0.5, math.radians(60.0)), 0.0)

    def test_the_frame_EDGE_is_the_half_field_of_view(self):
        for hfov in (math.radians(40.0), math.radians(60.0), math.radians(90.0)):
            self.assertAlmostEqual(abs(bearing_to_angle(1.0, hfov)), hfov / 2.0,
                                   places=6)

    def test_the_same_bearing_means_a_different_angle_through_a_different_lens(self):
        narrow = bearing_to_angle(0.6, math.radians(40.0))
        wide = bearing_to_angle(0.6, math.radians(90.0))
        self.assertNotAlmostEqual(narrow, wide, places=3)


# --------------------------------------------------------------------------- #
class PurityTests(unittest.TestCase):
    """No ROS types, no clock. The whole reason this is a separate seam."""

    def test_it_takes_plain_numbers_and_a_plain_list(self):
        f = fit_surface(ANGLE_MIN, ANGLE_INC,
                        tuple(scan_of([wall(5.0, 0.0)])),
                        0.0, math.radians(35.0))
        self.assertTrue(f["ok"], f["reason"])

    def test_it_does_not_mutate_the_ranges_it_was_given(self):
        ranges = scan_of([wall(5.0, 0.0)])
        before = list(ranges)
        fit(ranges)
        self.assertEqual(before, ranges)

    def test_None_entries_are_tolerated_like_infinities(self):
        ranges = [None if r == float("inf") else r
                  for r in scan_of([wall(5.0, 0.0)])]
        f = fit(ranges)
        self.assertTrue(f["ok"], f["reason"])


# --------------------------------------------------------------------------- #
class GateOpeningTests(unittest.TestCase):
    """Where the board stops and the way under it begins.

    Squaring up and passing through are mutually exclusive altitudes. The
    lidar can only measure the board's angle where the scan plane cuts the
    board, which is exactly the height at which the aircraft would fly into
    it -- measured on the shipped arena as a 2.5 to 3.5 m usable band against
    a board spanning 2.805 to 3.955.

    So the aircraft finds the bottom edge and drops below it, and the edge is
    a TRANSITION rather than a number: one wide face becomes two post
    clusters with a hole between them.
    """

    POST_HALF = 1.92

    def _at_board_height(self, distance=5.0, tilt=0.0):
        """One continuous face: board plus both posts, no hole."""
        return [wall(distance, tilt, half_length_m=self.POST_HALF)]

    def _below_the_board(self, distance=5.0, tilt=0.0):
        """Two posts and 3.8 m of nothing between them."""
        return [post(distance, tilt, -self.POST_HALF),
                post(distance, tilt, +self.POST_HALF)]

    def _open(self, segments, **kw):
        kw.setdefault("need_clear_m", 10.0)
        return gate_opening(ANGLE_MIN, ANGLE_INC, scan_of(segments), 0.0,
                            math.radians(35.0), **kw)

    def test_at_board_height_there_is_NO_way_through(self):
        r = self._open(self._at_board_height())
        self.assertFalse(r["open"])
        self.assertIn("board", r["reason"])

    # ---- the shipped arena: corridor walls begin AT the posts ---- #
    def _posts_into_walls(self, distance=3.4, half=1.8, wall_len=10.0):
        """Below the board where each post runs straight on into its wall.

        Live run, shipped arena: from 2.3 m down to the floor the lidar
        reported "one continuous surface 1.3 m across at 3.8 m; this is the
        board" -- that was the post plus the first metre of corridor wall,
        seen as one group. The board was never there.
        """
        segs = [post(distance, 0.0, -half), post(distance, 0.0, +half)]
        for side in (-half, +half):
            segs.append(((distance, side), (distance + wall_len, side)))
        return segs

    def test_posts_that_run_into_walls_are_an_OPEN_gate(self):
        r = self._open(self._posts_into_walls(), need_clear_m=0.0)
        self.assertTrue(r["open"], r["reason"])
        self.assertAlmostEqual(r["gap_m"], 3.6, delta=0.4)
        self.assertAlmostEqual(r["gate_m"], 3.4, delta=0.3,
                               msg="gate distance must be the posts, not "
                                   "the wall centroids")

    def test_the_board_in_front_of_those_walls_is_still_the_board(self):
        segs = self._posts_into_walls() + [wall(3.4, 0.0, half_length_m=1.9)]
        r = self._open(segs, need_clear_m=0.0)
        self.assertFalse(r["open"])
        self.assertIn("board", r["reason"])
        self.assertEqual(r["clusters"], 1)

    def test_a_block_joined_to_the_wall_behind_the_gate_is_not_the_board(self):
        """Seed 1002 return lap: o4 touches the outer wall, which starts at
        the post, so post + wall + block are ONE group reaching the flight
        line 1.2 m behind the posts. That is not the board."""
        segs = self._posts_into_walls(distance=5.2, half=1.8)
        # Block 1.2 m past the posts, from the -1.8 wall to 0.33 m off the line.
        segs.append(((6.4, -1.8), (6.4, -0.33)))
        segs.append(((6.75, -1.8), (6.75, -0.33)))
        segs.append(((6.4, -0.33), (6.75, -0.33)))
        r = self._open(segs, need_clear_m=0.0)
        self.assertTrue(r["open"], r["reason"])
        self.assertAlmostEqual(r["gate_m"], 5.2, delta=0.3)
        self.assertLess(r["clear_m"], 7.0, "the block must still limit clearance")

    def test_an_obstacle_on_the_line_between_the_walls_still_blocks(self):
        segs = self._posts_into_walls() + [
            ((5.0, -0.5), (5.0, 0.6))]              # 1.1 m block on the line
        r = self._open(segs, need_clear_m=10.0)
        self.assertFalse(r["open"])
        self.assertLess(r["clear_m"], 5.5)

    def test_below_the_board_the_gate_is_OPEN(self):
        r = self._open(self._below_the_board())
        self.assertTrue(r["open"], r["reason"])
        self.assertAlmostEqual(r["gap_m"], 2 * self.POST_HALF, delta=0.4)

    def test_the_transition_is_what_marks_the_bottom_EDGE(self):
        """The pair of readings the descent is looking for, and the whole
        reason the edge does not have to be written down anywhere."""
        above = self._open(self._at_board_height())
        below = self._open(self._below_the_board())
        self.assertFalse(above["open"])
        self.assertTrue(below["open"])
        self.assertLess(above["clusters"], below["clusters"])

    def test_something_STANDING_IN_the_gate_is_not_an_opening(self):
        """A gap with an obstruction in it reads as a gap. Flying at it on
        that basis is how a clear-looking hole becomes a collision."""
        segs = self._below_the_board() + [
            wall(2.5, 0.0, half_length_m=0.45)]      # obstacle in the mouth
        r = self._open(segs, need_clear_m=10.0)
        self.assertFalse(r["open"])
        self.assertIn("standing", r["reason"])

    def test_a_DEEPER_corridor_obstacle_does_not_hide_the_gate_posts(self):
        """Run 20, return lap: the first slalom obstacle stood 7.1 m ahead,
        behind posts about 5 m ahead. That makes the requested 10 m leg
        unsafe, but it does not erase the measured board-to-post transition.
        The caller needs both distances to hand control to the corridor
        navigator just beyond the board."""
        segs = self._below_the_board() + [
            wall(7.1, 0.0, half_length_m=0.45)]

        r = self._open(segs, need_clear_m=10.0)

        self.assertFalse(r["open"])
        self.assertAlmostEqual(r["gate_m"], 5.0, delta=0.35)
        self.assertAlmostEqual(r["gap_m"], 2 * self.POST_HALF, delta=0.4)
        self.assertAlmostEqual(r["clear_m"], 7.1, delta=0.2)
        self.assertIn("standing", r["reason"])

    def test_DEEPER_side_walls_are_not_mistaken_for_wider_gate_posts(self):
        """Run 20 reported the post gap growing from 6.5 m to 8.2 m as the
        aircraft descended. The real posts stay 3.8 m apart. Those larger
        numbers came from bracketing the deeper corridor walls instead of the
        nearest pair of post returns."""
        corridor_walls = [
            ((5.2, -3.8), (11.0, -3.8)),
            ((5.2, +3.8), (11.0, +3.8)),
        ]

        r = self._open(self._below_the_board() + corridor_walls)

        self.assertTrue(r["open"], r["reason"])
        self.assertAlmostEqual(r["gate_m"], 5.0, delta=0.35)
        self.assertAlmostEqual(r["gap_m"], 2 * self.POST_HALF, delta=0.4)

    def test_a_gap_too_NARROW_to_fly_is_refused(self):
        segs = [post(5.0, 0.0, -0.35), post(5.0, 0.0, +0.35)]
        r = self._open(segs)
        self.assertFalse(r["open"])
        self.assertIn("narrow", r["reason"])

    def test_an_EMPTY_sector_is_not_an_opening(self):
        """The reading from pointing at open sky is identical to the reading
        from a wide clear gate, so it must not be treated as one."""
        r = self._open([])
        self.assertFalse(r["open"])
        self.assertIn("no returns", r["reason"])

    def test_it_reports_how_far_it_can_see_THROUGH_the_gap(self):
        r = self._open(self._below_the_board())
        self.assertGreaterEqual(r["clear_m"], 10.0)

    def test_an_OBLIQUE_gate_still_reads_as_open(self):
        r = gate_opening(ANGLE_MIN, ANGLE_INC,
                         scan_of(self._below_the_board(tilt=math.radians(20.0))),
                         math.radians(20.0), math.radians(35.0),
                         need_clear_m=10.0)
        self.assertTrue(r["open"], r["reason"])


if __name__ == "__main__":
    unittest.main()
