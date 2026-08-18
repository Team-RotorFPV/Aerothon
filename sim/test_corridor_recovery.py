#!/usr/bin/env python3
"""Phase 5 — the corridor controller must not be able to stall forever.

THE FAILURE THIS EXISTS TO PREVENT
    The first full live run reached RETURN_CORRIDOR and then sat at x=4.86 for
    over 250 seconds with:

        front_m = 0.77    cmd_vx = 0.0    centering_err = -0.0

    An obstacle inside the stop distance, symmetric walls so no lateral nudge,
    and no recovery of any kind. The mission never terminated; it simply hung
    with the aircraft hovering against an obstacle until the observer gave up.

    The replacement steers toward the largest navigable gap — which handles
    centring, obstacle avoidance and pass-side choice with one mechanism — and
    escalates CRUISE -> BLOCKED -> BACKOFF -> STUCK rather than stopping dead.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_corridor_recovery.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from sensor_msgs.msg import LaserScan

from avoidance.velocity_controller import VelocityController
from test_frame_conventions import make_scan


def walled_in(distance=0.6, n=360, rmax=12.0):
    """Every direction blocked at `distance` — no gap anywhere."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.05
    scan.range_max = rmax
    scan.ranges = [distance] * n
    return scan


def obstacle_on_one_side(blocked_side="left", n=360, rmax=12.0):
    """Corridor with one side blocked close and the other open."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.05
    scan.range_max = rmax
    ranges = []
    for i in range(n):
        ang = scan.angle_min + i * scan.angle_increment
        deg = math.degrees(ang)
        if blocked_side == "left" and 5 <= deg <= 60:
            ranges.append(0.5)
        elif blocked_side == "right" and -60 <= deg <= -5:
            ranges.append(0.5)
        else:
            ranges.append(rmax)
    scan.ranges = ranges
    return scan


class CorridorRecoveryTests(unittest.TestCase):

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
        self.sent = []
        self.node.pub_sp.publish = self.sent.append
        self.node.enabled = True

    def tearDown(self):
        self.node.destroy_node()

    def tick(self, scan, n=1):
        for _ in range(n):
            self.node._on_scan(scan)
            self.node._tick()
        return self.sent[-1]

    # ---- the stall ---- #

    def test_fully_blocked_does_not_stall_forever(self):
        """The live failure: stopped, centred, and stuck for 250 s."""
        scan = walled_in(0.6)
        states = []
        for _ in range(200):
            self.node._on_scan(scan)
            self.node._tick()
            states.append(self.node.state)
        self.assertIn("BACKOFF", states,
                      "controller never attempted recovery; it stalled")
        self.assertNotEqual(states[-1], "BLOCKED",
                            "still merely BLOCKED after 200 ticks")

    def test_backoff_actually_reverses(self):
        scan = walled_in(0.6)
        for _ in range(200):
            self.node._on_scan(scan)
            self.node._tick()
            if self.node.state == "BACKOFF":
                break
        sp = self.tick(scan)
        self.assertLess(sp.velocity.x, 0.0,
                        "BACKOFF must command reverse motion")

    def test_escalates_to_STUCK_and_says_so(self):
        """Recovery is bounded: eventually it reports defeat rather than
        pretending to make progress, so the mission can fail closed."""
        scan = walled_in(0.6)
        for _ in range(2000):
            self.node._on_scan(scan)
            self.node._tick()
            if self.node.state == "STUCK":
                break
        self.assertEqual(self.node.state, "STUCK")
        self.assertLessEqual(self.node._backoffs_done,
                             int(self.node.get_parameter('max_backoffs').value))

    # ---- pass-side selection, which the old controller could not do ---- #

    def test_obstacle_on_the_left_is_passed_on_the_right(self):
        sp = self.tick(obstacle_on_one_side("left"))
        self.assertLess(sp.velocity.y, 0.0,
                        "obstacle to the left must be passed on the right (-y)")

    def test_obstacle_on_the_right_is_passed_on_the_left(self):
        sp = self.tick(obstacle_on_one_side("right"))
        self.assertGreater(sp.velocity.y, 0.0,
                           "obstacle to the right must be passed on the left (+y)")

    # ---- lidar health (goal.md Q21) ---- #

    def test_stale_scan_stops_rather_than_coasting(self):
        self.tick(make_scan(1.75, 1.75, 10.0))
        self.node._scan_t = self.node._now() - 5.0        # go quiet
        self.node._tick()
        sp = self.sent[-1]
        self.assertAlmostEqual(sp.velocity.x, 0.0, places=6)
        self.assertAlmostEqual(sp.velocity.y, 0.0, places=6)
        self.assertIn("stale", self.node.last_detail.get("fault", ""))

    def test_missing_scan_stops(self):
        self.node.scan = None
        self.node._tick()
        sp = self.sent[-1]
        self.assertAlmostEqual(sp.velocity.x, 0.0, places=6)

    def test_ranges_are_clamped_to_valid_band(self):
        scan = make_scan(1.75, 1.75, 10.0)
        scan.ranges = [0.0] * 180 + [999.0] * 180
        _, ranges = self.node.conditioned(scan)
        lo = float(self.node.get_parameter('range_min_valid').value)
        hi = float(self.node.get_parameter('range_max_valid').value)
        self.assertGreaterEqual(min(ranges), lo)
        self.assertLessEqual(max(ranges), hi)

    def test_isolated_outlier_is_filtered(self):
        """A single spuriously short return must not brake the aircraft."""
        scan = make_scan(1.75, 1.75, 10.0)
        r = list(scan.ranges)
        mid = len(r) // 2
        r[mid] = 0.2                       # one bad sample dead ahead
        scan.ranges = r
        _, ranges = self.node.conditioned(scan)
        self.assertGreater(min(ranges), 0.25,
                           "median filter should have removed the outlier")

    def test_nan_and_inf_are_handled(self):
        scan = make_scan(1.75, 1.75, 10.0)
        r = list(scan.ranges)
        r[10] = float('nan')
        r[11] = float('inf')
        scan.ranges = r
        _, ranges = self.node.conditioned(scan)
        self.assertTrue(all(x == x for x in ranges), "NaN leaked through")
        self.assertTrue(all(abs(x) != float('inf') for x in ranges))

    # ---- re-enable resets escalation ---- #

    def test_reenabling_clears_stuck_state(self):
        scan = walled_in(0.6)
        for _ in range(2000):
            self.node._on_scan(scan)
            self.node._tick()
            if self.node.state == "STUCK":
                break
        self.assertEqual(self.node.state, "STUCK")
        from std_msgs.msg import Bool
        self.node._on_enable(Bool(data=False))
        self.node._on_enable(Bool(data=True))
        self.assertEqual(self.node.state, "CRUISE")
        self.assertEqual(self.node._backoffs_done, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)


class ProgressWatchdogTests(unittest.TestCase):
    """A gap you never travel through is still a stall.

    The second live run had a gap the whole time — 31 deg to the left, 0.81 m
    deep — and covered no ground for four minutes, because forward speed was
    gated on straight-ahead clearance while the steering pointed elsewhere.
    Gap-presence is not progress; progress is measured.
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
        self.node = VelocityController()
        self.sent = []
        self.node.pub_sp.publish = self.sent.append
        self.node.enabled = True

    def tearDown(self):
        self.node.destroy_node()

    def _at(self, x, y):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        self.node._on_pose(m)

    def test_no_ground_covered_is_detected_as_stalled(self):
        self.node.set_parameters([
            rclpy.parameter.Parameter('progress_window_s',
                                      rclpy.parameter.Parameter.Type.DOUBLE, 0.0)])
        self._at(4.86, 0.0)
        self.node.stalled()                     # seed the reference
        self._at(4.86, 0.02)                    # essentially unmoved
        self.assertTrue(self.node.stalled(), "stall not detected")

    def test_real_progress_is_not_flagged(self):
        self.node.set_parameters([
            rclpy.parameter.Parameter('progress_window_s',
                                      rclpy.parameter.Parameter.Type.DOUBLE, 0.0)])
        self._at(0.0, 0.0)
        self.node.stalled()
        self._at(5.0, 0.0)
        self.assertFalse(self.node.stalled(), "normal flight flagged as stalled")

    def test_off_axis_gap_produces_forward_motion(self):
        """The exact deadlock: gap off to one side, dead ahead blocked."""
        sp = self.tick_off_axis()
        self.assertGreater(sp.velocity.x, 0.0,
                           "must travel ALONG the gap, not slide sideways forever")

    def tick_off_axis(self):
        scan = obstacle_on_one_side("left")
        self.node._on_scan(scan)
        self.node._tick()
        return self.sent[-1]


class CorridorExitDetectionTests(unittest.TestCase):
    """Exit is SEEN, not assumed from a hardcoded x threshold.

    corridor_exit_x=15.5 and corridor_return_exit_x=4.5 (geometry audit A6,
    A10) assume the corridor's length and placement are known in advance. The
    third live run sat just short of x=4.5 unable to reach it. The lidar can
    simply observe that both walls have fallen away.
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
        self.node = VelocityController()
        self.node.pub_sp.publish = lambda m: None
        self.node.enabled = True

    def tearDown(self):
        self.node.destroy_node()

    def test_inside_a_corridor_is_not_exited(self):
        b, r = self.node.conditioned(make_scan(1.75, 1.75, 10.0))
        is_open, left, right = self.node.corridor_open(b, r)
        self.assertFalse(is_open, f"walls at 1.75 m read as open (L={left} R={right})")

    def test_open_ground_is_exited(self):
        b, r = self.node.conditioned(make_scan(11.0, 11.0, 11.0))
        is_open, left, right = self.node.corridor_open(b, r)
        self.assertTrue(is_open, f"open ground not detected (L={left} R={right})")

    def test_one_wall_still_counts_as_corridor(self):
        """Half a corridor is still a corridor; do not declare exit early."""
        b, r = self.node.conditioned(make_scan(1.5, 11.0, 10.0))
        is_open, _, _ = self.node.corridor_open(b, r)
        self.assertFalse(is_open)

    def _enter(self):
        """Fly far enough inside a corridor that entry is observed."""
        closed = make_scan(1.75, 1.75, 10.0)
        need = int(self.node.get_parameter('corridor_enter_ticks').value)
        for _ in range(need + 2):
            self.node._on_scan(closed)
            self.node._tick()

    def test_exit_requires_sustained_openness(self):
        """A momentary gap in one wall must not be read as the end."""
        self._enter()
        open_scan = make_scan(11.0, 11.0, 11.0)
        self.node._on_scan(open_scan)
        self.node._tick()
        self.assertFalse(self.node._exited, "declared exit on a single frame")
        need = int(self.node.get_parameter('corridor_open_ticks').value)
        for _ in range(need + 2):
            self.node._on_scan(open_scan)
            self.node._tick()
        self.assertTrue(self.node._exited)

    # ---- you cannot exit what you were never inside ---- #

    def test_open_ground_before_the_corridor_is_NOT_an_exit(self):
        """THE LIVE FAILURE. On the open apron before the corridor both sides
        are clear, so the detector fired one second into the stage at x = 1.2 m.
        The mission then searched the takeoff pad for the delivery zone and
        reported COMPLETED. The hardcoded GotoZone waypoint had been dragging
        the aircraft to the real zone and hiding it."""
        open_scan = make_scan(11.0, 11.0, 11.0)
        for _ in range(60):
            self.node._on_scan(open_scan)
            self.node._tick()
        self.assertFalse(self.node._entered,
                         "claimed to be inside a corridor on open ground")
        self.assertFalse(self.node._exited,
                         "declared a corridor exit without ever entering one")

    def test_entry_then_exit_is_the_only_accepted_sequence(self):
        self._enter()
        self.assertTrue(self.node._entered, "walls at 1.75 m did not read as inside")
        self.assertFalse(self.node._exited, "exited while still between walls")
        open_scan = make_scan(11.0, 11.0, 11.0)
        for _ in range(int(self.node.get_parameter('corridor_open_ticks').value) + 2):
            self.node._on_scan(open_scan)
            self.node._tick()
        self.assertTrue(self.node._exited)

    def test_entry_needs_to_be_sustained_too(self):
        """One frame of clutter beside the aircraft is not a corridor."""
        self.node._on_scan(make_scan(1.75, 1.75, 10.0))
        self.node._tick()
        self.assertFalse(self.node._entered)

    def test_re_enabling_clears_the_traversal_so_the_return_trip_works(self):
        """The return leg is its own traversal and must observe entry again;
        otherwise it inherits the outbound exit and ends instantly."""
        self._enter()
        open_scan = make_scan(11.0, 11.0, 11.0)
        for _ in range(int(self.node.get_parameter('corridor_open_ticks').value) + 2):
            self.node._on_scan(open_scan)
            self.node._tick()
        self.assertTrue(self.node._exited)

        from std_msgs.msg import Bool
        self.node._on_enable(Bool(data=False))
        self.node._on_enable(Bool(data=True))
        self.assertFalse(self.node._entered)
        self.assertFalse(self.node._exited)

    def test_open_extent_is_published_for_the_zone_bound(self):
        """ObserveZone bounds the delivery zone from these two numbers."""
        self.node._on_scan(make_scan(11.0, 11.0, 11.0))
        self.node._tick()
        d = self.node._last_detail
        self.assertIn("open_depth_m", d)
        self.assertIn("open_width_m", d)
        self.assertGreater(d["open_width_m"], 0.0)

    def test_openness_counter_resets_when_walls_return(self):
        open_scan = make_scan(11.0, 11.0, 11.0)
        closed = make_scan(1.75, 1.75, 10.0)
        for _ in range(5):
            self.node._on_scan(open_scan)
            self.node._tick()
        self.node._on_scan(closed)
        self.node._tick()
        self.assertEqual(self.node._open_ticks, 0)
        self.assertFalse(self.node._exited)


# --------------------------------------------------------------------------- #
class ObserveWhileDisabledTests(unittest.TestCase):
    """Perception that only reports while it is steering cannot be handed to.

    ApproachBanner has to know "am I inside the corridor yet?" BEFORE it gives
    control to the navigator. If the navigator went silent whenever it was
    disabled, that question could never be answered and the approach would
    have to guess a distance -- reintroducing the hardcoded waypoint it exists
    to replace.
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
        self.node = VelocityController()
        self.sent = []
        self.node.pub_sp.publish = self.sent.append
        self.detail = []
        self.node.pub_detail.publish = self.detail.append
        self.node.enabled = False

    def tearDown(self):
        self.node.destroy_node()

    def test_disabled_navigator_still_reports(self):
        self.node._on_scan(make_scan(1.75, 1.75, 10.0))
        self.node._tick()
        self.assertTrue(self.detail, "navigator went silent while disabled")

    def test_disabled_navigator_commands_NOTHING(self):
        """Reporting must not become steering."""
        self.node._on_scan(make_scan(1.75, 1.75, 10.0))
        for _ in range(10):
            self.node._tick()
        self.assertEqual(self.sent, [],
                         "a disabled navigator published setpoints")

    def test_entry_is_observed_without_control(self):
        import json
        closed = make_scan(1.75, 1.75, 10.0)
        for _ in range(int(self.node.get_parameter('corridor_enter_ticks').value) + 2):
            self.node._on_scan(closed)
            self.node._tick()
        d = json.loads(self.detail[-1].data)
        self.assertTrue(d["corridor_entered"],
                        "entry was not detected while disabled")

    def test_open_ground_is_not_reported_as_entered(self):
        import json
        for _ in range(30):
            self.node._on_scan(make_scan(11.0, 11.0, 11.0))
            self.node._tick()
        self.assertFalse(json.loads(self.detail[-1].data)["corridor_entered"])
