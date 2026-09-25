#!/usr/bin/env python3
"""Closed loop: the real corridor navigator flies the shipped return slalom.

WHY A CLOSED LOOP

    Live run 3 on the shipped arena came out of the return gate past block
    o4, drifted into the shadow of o3, crept at it for fifteen seconds and
    rolled 74 degrees on contact. Every single-scan test of the navigator
    passed: each one checked a direction, and the failure was a TRAJECTORY --
    a sequence of individually plausible headings that ended against a block.

    So this flies it. The shipped return lane (home-local frame: walls at
    y = -2.2 and -5.8 from x 3.9 to 14.1, four 0.35 x 1.45 m blocks
    alternating sides) is ray-cast into a 360-beam scan every tick, the
    VelocityController's own _tick() turns it into a body-frame velocity and
    yaw rate, and a kinematic aircraft integrates that. The airframe is a
    0.3 m circle, and it must never touch a block or a wall.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_slalom_traverse.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_avoidance", "avoidance"))

import rclpy                                        # noqa: E402
from geometry_msgs.msg import PoseStamped          # noqa: E402
from sensor_msgs.msg import LaserScan              # noqa: E402

from avoidance.velocity_controller import VelocityController  # noqa: E402

# Iris: 0.25 m arm + 0.127 m (10 in) prop radius ~= 0.38 m. Live run 4 went
# down o2's flank 0.17 m from the block's edge and pitched to 47 degrees.
AIRFRAME_R = 0.4
# The airframe does not do what it is told instantly, and the lidar does not
# refresh every control tick. A fake that responds at once flew this slalom
# cleanly while the real one hit o2.
VEL_TAU_S = 0.6
SCAN_PERIOD_S = 0.1
N_BEAMS = 500          # the RPLidar C1 figure the sim uses
RMAX = 12.0


def rect(cx, cy, w, h):
    return (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)


# Shipped arena, home-local (world + (2, -2)).
WALLS = [rect(9.0, -2.2, 10.2, 0.1), rect(9.0, -5.8, 10.2, 0.1)]
BLOCKS = [rect(5.4, -2.95, 0.35, 1.45), rect(7.8, -5.05, 0.35, 1.45),
          rect(10.2, -2.95, 0.35, 1.45), rect(12.6, -5.05, 0.35, 1.45)]


def _ray_rect(ox, oy, dx, dy, r):
    """Distance along the ray to an axis-aligned rect (slab method)."""
    x0, x1, y0, y1 = r
    tmin, tmax = 0.0, RMAX
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0.0 else None


def scan_at(x, y, yaw, rects):
    s = LaserScan()
    s.angle_min = -math.pi
    s.angle_increment = 2 * math.pi / N_BEAMS
    s.angle_max = s.angle_min + s.angle_increment * (N_BEAMS - 1)
    s.range_min, s.range_max = 0.05, RMAX
    ranges = []
    for i in range(N_BEAMS):
        a = yaw + s.angle_min + i * s.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        hits = [t for t in (_ray_rect(x, y, dx, dy, r) for r in rects) if t]
        ranges.append(min(hits) if hits else float("inf"))
    s.ranges = ranges
    return s


def gap_to(x, y, r):
    x0, x1, y0, y1 = r
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


class SlalomTraverseTests(unittest.TestCase):

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
        self.node.state = "CRUISE"

    def tearDown(self):
        self.node.destroy_node()

    def fly(self, x, y, yaw, goal_x, dt=0.05, max_t=120.0):
        rects = WALLS + BLOCKS
        closest = float("inf")
        t = 0.0
        track = []
        wvx = wvy = 0.0                 # the airframe's ACTUAL world velocity
        next_scan = 0.0
        while t < max_t:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.position.z = 3.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.node._on_pose(pose)
            if t >= next_scan:
                self.node._on_scan(scan_at(x, y, yaw, rects))
                next_scan = t + SCAN_PERIOD_S
            self.node._scan_t = self.node._now()     # never "stale" in a test
            self.node._tick()
            sp = self.sent[-1]
            vx, vy = sp.velocity.x, sp.velocity.y          # body FLU
            cvx = vx * math.cos(yaw) - vy * math.sin(yaw)
            cvy = vx * math.sin(yaw) + vy * math.cos(yaw)
            a = dt / VEL_TAU_S
            wvx += (cvx - wvx) * a
            wvy += (cvy - wvy) * a
            x += wvx * dt
            y += wvy * dt
            yaw += sp.yaw_rate * dt
            t += dt
            track.append((round(t, 2), round(x, 2), round(y, 2)))
            closest = min(closest, min(gap_to(x, y, r) for r in rects))
            if closest < AIRFRAME_R:
                self.fail(f"airframe touched something at ({x:.2f}, {y:.2f}) "
                          f"t={t:.1f}s; track tail {track[-6:]}")
            if x < goal_x:
                return t, closest, track
        self.fail(f"did not get through in {max_t:.0f} s; "
                  f"stuck near ({x:.2f}, {y:.2f}); track tail {track[-6:]}")

    # Contact is checked every step; clearance must also leave a margin for
    # the real airframe's overshoot beyond this model's.
    MARGIN_M = 0.55

    def test_the_return_slalom_is_flown_without_contact(self):
        # Where run 3's return gate crossing put it: just past the posts,
        # heading west, 0.23 m north of o4's inner edge.
        t, closest, _ = self.fly(13.5, -4.1, math.pi, goal_x=4.5)
        self.assertGreaterEqual(closest, self.MARGIN_M)

    def test_from_the_lane_centre_too(self):
        t, closest, _ = self.fly(13.8, -4.0, math.pi, goal_x=4.5)
        self.assertGreaterEqual(closest, self.MARGIN_M)

    def test_the_forward_lane_with_no_blocks_goes_straight_through(self):
        walls = [rect(9.0, 0.2 - 2.0 + 2.0, 10.2, 0.1)]  # placeholder
        del walls
        # The forward lane is the mirror of the return one without blocks;
        # flying east from its mouth must simply go down the middle.
        global BLOCKS
        saved = BLOCKS
        try:
            BLOCKS = []
            WALLS[:] = [rect(9.0, 1.8, 10.2, 0.1), rect(9.0, -1.8, 10.2, 0.1)]
            x, y, yaw = 4.5, 0.1, 0.0
            rects = WALLS
            for _ in range(600):
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.position.z = 3.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                self.node._on_pose(pose)
                self.node._on_scan(scan_at(x, y, yaw, rects))
                self.node._tick()
                sp = self.sent[-1]
                vx, vy = sp.velocity.x, sp.velocity.y
                x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * 0.05
                y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * 0.05
                yaw += sp.yaw_rate * 0.05
                self.assertLess(abs(y), 1.8 - AIRFRAME_R)
                if x > 14.0:
                    break
            self.assertGreater(x, 14.0, "did not make it down a clear lane")
        finally:
            BLOCKS = saved
            WALLS[:] = [rect(9.0, -2.2, 10.2, 0.1), rect(9.0, -5.8, 10.2, 0.1)]




class CustomSlalomTests(SlalomTraverseTests):
    """Obstacle layouts a user can build in the world editor.

    The lane is built in its own frame (u along from the banner, v across)
    and mirrored into the harness frame, where travel is -x. The first case
    is the split-corridor custom arena that failed live: 3.2 m lane, gaps of
    1.7-1.85 m, alternating sides 2.5 m apart. The navigator only looked
    +/-60 deg and paid a full turn penalty in front of a block, so it crept
    into obstacle 2 and reported STUCK.
    """

    def lane(self, L, W, obstacles):
        global BLOCKS
        wy = W / 2 + 0.05
        WALLS[:] = [rect(L / 2, wy, L + 0.2, 0.1), rect(L / 2, -wy, L + 0.2, 0.1)]
        blocks = []
        for u, v, w, d, yaw in obstacles:
            a = math.radians(yaw)
            du = abs(w * math.cos(a)) / 2 + abs(d * math.sin(a)) / 2
            dv = abs(w * math.sin(a)) / 2 + abs(d * math.cos(a)) / 2
            blocks.append(rect(L - u, -v, 2 * du, 2 * dv))
        BLOCKS[:] = blocks
        self.L = L

    def setUp(self):
        super().setUp()
        self._saved = (list(WALLS), list(BLOCKS))

    def tearDown(self):
        WALLS[:], BLOCKS[:] = self._saved
        super().tearDown()

    def through(self, v0=0.0):
        t, closest, _ = self.fly(self.L - 0.7, -v0, math.pi, goal_x=0.3, max_t=120)
        self.assertGreaterEqual(closest, AIRFRAME_R + 0.1)

    def test_the_split_corridor_arena_that_stuck_live(self):
        self.lane(9.0, 3.2, [(2.0, 0.9, 0.4, 1.3, 0), (4.5, -0.8, 0.4, 1.4, 15),
                             (7.0, 0.7, 0.5, 1.2, 0)])
        self.through(v0=-0.25)

    def test_a_tighter_slalom_with_1_5_m_gaps(self):
        self.lane(10.0, 3.0, [(2.0, 0.75, 0.35, 1.5, 0), (4.5, -0.75, 0.35, 1.5, 0),
                              (7.0, 0.75, 0.35, 1.5, 0)])
        self.through()

    def test_a_block_in_the_middle_of_the_lane(self):
        # 1.6 m either side. The passage strip is 1.4 m wide (0.7 m each side
        # of the line, sized for the airframe plus its lag), so a 1.4 m gap has
        # no tolerance at all; world_spec warns below 1.6 m.
        self.lane(10.0, 4.4, [(5.0, 0.0, 0.4, 1.2, 0)])
        self.through()

    def test_turned_blocks(self):
        self.lane(12.0, 3.5, [(3.0, 0.9, 0.3, 1.4, 25), (6.5, -0.9, 0.3, 1.4, -25),
                              (10.0, 0.9, 0.3, 1.4, 25)])
        self.through()

if __name__ == "__main__":
    unittest.main()
