#!/usr/bin/env python3
"""The vectorised navigator computes exactly what the per-ray loops did.

WHY THIS EXISTS

    The corridor navigator runs at 20 Hz on the Pi 5, and nearly all of its
    time went into two Python loops: conditioning the scan ray by ray, and
    testing every candidate heading against every return (91 headings x ~300
    returns, twice a tick when the full strip is shut). Both were rewritten
    in numpy. The rewrite is only acceptable if it changes nothing the
    aircraft does, so this keeps the loops, verbatim, as a reference, and
    requires the node's output to be EQUAL to them, bit for bit, not
    approximately: the chosen heading, its width and depth, the conditioned
    ranges, the sector minima, and the heading remembered for next tick's
    commit term.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_navigator_equivalence.py -v
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))

import rclpy                                        # noqa: E402
from rclpy.parameter import Parameter               # noqa: E402
from sensor_msgs.msg import LaserScan              # noqa: E402

from avoidance.velocity_controller import VelocityController  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_slalom_traverse import BLOCKS, WALLS, rect, scan_at  # noqa: E402


# ---- the reference: the loops as they were before vectorising ---- #

def ref_conditioned(g, scan):
    lo = float(g('range_min_valid'))
    hi = float(g('range_max_valid'))
    mask = [math.radians(a) for a in g('mask_sectors_deg')]
    bearings, ranges = [], []
    for i, r in enumerate(scan.ranges):
        ang = scan.angle_min + i * scan.angle_increment
        wrapped = (ang + math.pi) % (2 * math.pi) - math.pi
        if len(mask) == 2:
            a_deg = math.degrees(wrapped) % 360.0
            if mask[0] <= math.radians(a_deg) <= mask[1]:
                continue
        if r != r or r in (float('inf'), float('-inf')):
            r = hi
        r = max(lo, min(hi, float(r)))
        bearings.append(wrapped)
        ranges.append(r)
    w = int(g('median_window'))
    if w >= 3 and len(ranges) >= w:
        half = w // 2
        smoothed = []
        for i in range(len(ranges)):
            lo_i = max(0, i - half)
            hi_i = min(len(ranges), i + half + 1)
            window = sorted(ranges[lo_i:hi_i])
            smoothed.append(window[len(window) // 2])
        ranges = smoothed
    return bearings, ranges


def ref_escape(g, bearings, ranges):
    fov = math.radians(float(g('search_fov_deg'))) / 2.0
    r_air = float(g('airframe_radius'))
    stop = float(g('stop_dist'))
    near = [(r * math.cos(b), r * math.sin(b))
            for b, r in zip(bearings, ranges) if r < 1.5]
    if not near:
        return None
    step = math.radians(2.0)
    best = None
    for i in range(-int(fov / step), int(fov / step) + 1):
        th = i * step
        c, s = math.cos(th), math.sin(th)
        m = min((abs(-x * s + y * c) for x, y in near
                 if x * c + y * s > 0.0), default=1.5)
        score = m - 0.05 * abs(th)
        if best is None or score > best[0]:
            best = (score, th, m)
    if best is None or best[2] < r_air - 0.05:
        return None
    return best[1], step, stop + 0.3


def ref_find_gap_strip(g, state, bearings, ranges, half_w):
    fov = math.radians(float(g('search_fov_deg'))) / 2.0
    comfort = float(g('passage_comfort_width'))
    look = float(g('lookahead_m'))
    stop = float(g('stop_dist'))
    turn_k = float(g('turn_penalty_m_per_rad'))
    far = float(g('range_max_valid')) - 1e-3
    r_air = float(g('airframe_radius'))
    obstacles = [(r * math.cos(b), r * math.sin(b),
                  r_air + (half_w - r_air) * min(1.0, r))
                 for b, r in zip(bearings, ranges)
                 if r < far and r <= look + comfort]
    step = math.radians(2.0)
    n = int(fov / step)
    best = None
    narrow_of = {}
    wide_of = {}
    for i in range(-n, n + 1):
        th = i * step
        c, s = math.cos(th), math.sin(th)
        narrow = wide = look
        for x, y, hw in obstacles:
            along = x * c + y * s
            if along <= 0.0:
                continue
            lat = abs(-x * s + y * c)
            if lat <= hw and along < narrow:
                narrow = along
            if lat <= comfort and 0.3 < along < wide:
                wide = along
        narrow_of[i] = narrow
        wide_of[i] = wide
    ahead = narrow_of.get(0, look)
    k = turn_k * max(0.1, min(1.0, ahead / look))
    last = state.get("last")
    commit = float(g('gap_commit_m_per_rad'))
    for i in range(-n, n + 1):
        narrow = narrow_of[i]
        if narrow <= stop:
            continue
        th = i * step
        score = 0.6 * narrow + 0.4 * wide_of[i] - k * abs(th)
        if last is not None:
            score -= commit * abs(th - last)
        if best is None or score > best[0]:
            best = (score, i, narrow)
    if best is not None:
        state["last"] = best[1] * step
    if best is None:
        return None
    _, i_best, depth = best
    lo = hi = i_best
    while lo - 1 in narrow_of and narrow_of[lo - 1] > stop:
        lo -= 1
    while hi + 1 in narrow_of and narrow_of[hi + 1] > stop:
        hi += 1
    return i_best * step, (hi - lo + 1) * step, depth


def ref_find_gap(g, state, bearings, ranges):
    gap = ref_find_gap_strip(g, state, bearings, ranges,
                             float(g('passage_half_width')))
    if gap is None:
        gap = ref_find_gap_strip(g, state, bearings, ranges,
                                 float(g('airframe_radius')) + 0.1)
    if gap is None:
        gap = ref_escape(g, bearings, ranges)
    return gap


def ref_front_min(bearings, ranges, half_fov=math.radians(20)):
    vals = [r for b, r in zip(bearings, ranges) if abs(b) <= half_fov]
    return min(vals) if vals else float('inf')


def ref_side_min(bearings, ranges, centre_deg, half_fov=math.radians(35)):
    c = math.radians(centre_deg)
    vals = []
    for b, r in zip(bearings, ranges):
        d = (b - c + math.pi) % (2 * math.pi) - math.pi
        if abs(d) <= half_fov:
            vals.append(r)
    return min(vals) if vals else float('inf')


# ---- scans ---- #

def raw_scan(ranges, angle_min=-math.pi, n=None):
    s = LaserScan()
    n = len(ranges) if n is None else n
    s.angle_min = angle_min
    s.angle_increment = 2 * math.pi / max(1, n)
    s.range_min, s.range_max = 0.05, 12.0
    s.ranges = ranges
    return s


def random_scan(rng):
    """Everything a lidar can hand over: speckle, dropouts, NaN, inf, below
    minimum, beyond maximum, and structured walls and blocks."""
    kind = rng.random()
    n = rng.choice([500, 500, 360, 720, 499, 5, 3, 2, 1, 0])
    if kind < 0.45 and n >= 50:
        # A slalom at a random pose: real structure, exact geometry.
        extra = [rect(rng.uniform(3, 14), rng.uniform(-5.5, -2.5),
                      rng.uniform(0.2, 1.5), rng.uniform(0.2, 1.5))
                 for _ in range(rng.randint(0, 3))]
        s = scan_at(rng.uniform(3.0, 14.0), rng.uniform(-5.4, -2.6),
                    rng.uniform(-1.2, 1.2), WALLS + BLOCKS + extra)
        return s
    ranges = []
    base = rng.uniform(0.3, 11.0)
    for _ in range(n):
        u = rng.random()
        if u < 0.04:
            ranges.append(float("nan"))
        elif u < 0.10:
            ranges.append(float("inf"))
        elif u < 0.11:
            ranges.append(float("-inf"))
        elif u < 0.14:
            ranges.append(rng.uniform(0.0, 0.2))
        elif u < 0.17:
            ranges.append(rng.uniform(11.5, 20.0))
        elif u < 0.30:
            ranges.append(rng.uniform(0.2, 12.0))
        else:
            base = min(12.0, max(0.1, base + rng.gauss(0.0, 0.15)))
            ranges.append(base)
    return raw_scan(ranges, angle_min=rng.choice(
        [-math.pi, -math.pi, 0.0, -math.pi / 2, 0.37, -3.1415927410125732]))


PARAM_SETS = [
    {},
    {"median_window": 5},
    {"median_window": 4},
    {"median_window": 1},
    {"mask_sectors_deg": [0.0]},
    {"mask_sectors_deg": [100.0, 260.0], "search_fov_deg": 120.0},
    {"search_fov_deg": 90.0, "gap_commit_m_per_rad": 0.0},
    {"passage_half_width": 0.9, "stop_dist": 1.2, "lookahead_m": 3.0},
    {"range_max_valid": 8.0, "range_min_valid": 0.3},
]


def as_param(k, v):
    if isinstance(v, list):
        return Parameter(k, Parameter.Type.DOUBLE_ARRAY, [float(x) for x in v])
    if isinstance(v, int):
        return Parameter(k, Parameter.Type.INTEGER, v)
    return Parameter(k, Parameter.Type.DOUBLE, float(v))


class NavigatorEquivalenceTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = VelocityController()

    def tearDown(self):
        self.node.destroy_node()

    def assert_same(self, got, want, what):
        self.assertEqual(type(got) is tuple, type(want) is tuple, what)
        if want is None:
            self.assertIsNone(got, what)
            return
        self.assertEqual(tuple(got), tuple(want), what)
        for a, b in zip(got, want):
            self.assertEqual(math.copysign(1.0, a), math.copysign(1.0, b), what)

    def check(self, scan, state, what):
        g = self.node._g
        rb, rr = ref_conditioned(g, scan)
        nb, nr = self.node.conditioned(scan)
        self.assertEqual(list(nb), rb, what + " bearings")
        self.assertEqual(list(nr), rr, what + " ranges")
        if not rr:
            self.assertEqual(len(nr), 0, what)
            return
        # Feed each implementation its own conditioned output.
        want = ref_find_gap(g, state, rb, rr)
        got = self.node.find_gap(nb, nr)
        self.assert_same(got, want, what + " gap")
        self.assertEqual(getattr(self.node, "_last_gap_bearing", None),
                         state.get("last"), what + " committed heading")
        self.assertEqual(self.node._escape(nb, nr), ref_escape(g, rb, rr),
                         what + " escape")
        for hf in (math.radians(20), math.radians(15)):
            self.assertEqual(self.node._front_min(nb, nr, hf),
                             ref_front_min(rb, rr, hf), what + " front")
        for c in (90.0, -90.0, 0.0, 180.0):
            self.assertEqual(self.node._side_min(nb, nr, c),
                             ref_side_min(rb, rr, c), what + " side")

    def test_random_and_slalom_scans_agree_exactly(self):
        rng = random.Random(20260924)
        for pi, params in enumerate(PARAM_SETS):
            if params:
                self.node.set_parameters(
                    [as_param(k, v) for k, v in params.items()])
            state = {}
            self.node._last_gap_bearing = None
            for k in range(160):
                # A run of ticks shares the commit state, as in flight.
                if k % 40 == 0:
                    state = {}
                    self.node._last_gap_bearing = None
                self.check(random_scan(rng), state, f"set {pi} scan {k}")

    def test_the_trapped_and_the_open_agree(self):
        """The escape branch and the empty-strip branch, deliberately."""
        state = {}
        cases = [
            [0.5] * 500,                       # boxed in: strip shut, escape
            [0.35] * 500,                      # too close to escape
            [12.0] * 500,                      # nothing seen at all
            [float("inf")] * 500,
            [float("nan")] * 500,
            [1.2 if i % 7 else 0.2 for i in range(500)],
            [0.9 if 200 < i < 300 else 12.0 for i in range(500)],
        ]
        for i, r in enumerate(cases):
            self.check(raw_scan(r), state, f"case {i}")


if __name__ == "__main__":
    unittest.main()
