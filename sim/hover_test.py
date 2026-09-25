#!/usr/bin/env python3
"""Phase 2 — basic flight test with NO perception, avoidance or behaviour tree.

PHASE_PLAN.md P2 requires reproducing takeoff, position hold and landing before
any autonomy work, because Phase 0 found the aircraft sustaining >45 deg tilt
during a mission takeoff (VERIFICATION.md 0.13a). This isolates the question:
is the airframe/physics unstable, or is the mission's guidance driving it?

It talks only to MAVROS. The behaviour tree can be running; parked at WAITING
it publishes nothing.

Sequence: GUIDED -> arm -> takeoff -> hold -> optional square -> land.
Records attitude/position at full rate throughout and prints a verdict.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 sim/hover_test.py                 # takeoff, hover, land
    python3 sim/hover_test.py --square 3.0    # add a 3 m square waypoint leg
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

TAKEOFF_ALT = 5.0
ATT_WARN_DEG = 25.0
ATT_FAIL_DEG = 45.0


def rp_deg(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return math.degrees(roll), math.degrees(math.asin(sinp))


class HoverTest(Node):
    def __init__(self):
        super().__init__("hover_test")
        self.state = State()
        self.pose = PoseStamped()
        self.samples = []          # (t, phase, roll, pitch, x, y, z)
        self.phase = "init"
        self.t0 = time.time()
        self._sp = None

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.pub_sp = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.create_timer(0.1, self._stream)

    def _on_pose(self, m):
        self.pose = m
        r, p = rp_deg(m.pose.orientation)
        self.samples.append((time.time() - self.t0, self.phase, r, p,
                             m.pose.position.x, m.pose.position.y,
                             m.pose.position.z))

    def _stream(self):
        if self._sp is not None and self.state.armed:
            self._sp.header.stamp = self.get_clock().now().to_msg()
            self._sp.header.frame_id = "map"
            self.pub_sp.publish(self._sp)

    def goto(self, x, y, z):
        sp = PoseStamped()
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        sp.pose.orientation.w = 1.0
        self._sp = sp

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, pred, timeout, what):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        print(f"  TIMEOUT waiting for {what}")
        return False

    def call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        end = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def alt(self):
        return self.pose.pose.position.z


def summarize(samples):
    if not samples:
        print("no samples")
        return False

    print(f"\n{'phase':12s} {'n':>5s} {'dur(s)':>7s} {'alt min':>8s} {'alt max':>8s} "
          f"{'|roll|max':>10s} {'|pitch|max':>11s}")
    phases = []
    for t, ph, r, p, x, y, z in samples:
        if not phases or phases[-1][0] != ph:
            phases.append((ph, []))
        phases[-1][1].append((t, r, p, z))

    worst = 0.0
    for ph, rows in phases:
        dur = rows[-1][0] - rows[0][0]
        mr = max(abs(v[1]) for v in rows)
        mp = max(abs(v[2]) for v in rows)
        worst = max(worst, mr, mp)
        print(f"{ph:12s} {len(rows):5d} {dur:7.1f} "
              f"{min(v[3] for v in rows):8.2f} {max(v[3] for v in rows):8.2f} "
              f"{mr:10.1f} {mp:11.1f}")

    rate = len(samples) / max(samples[-1][0], 0.001)
    print(f"\nsample rate: {rate:.1f} Hz over {samples[-1][0]:.1f}s "
          f"({len(samples)} samples)")

    over_warn = sum(1 for _, _, r, p, *_ in samples
                    if max(abs(r), abs(p)) > ATT_WARN_DEG)
    over_fail = sum(1 for _, _, r, p, *_ in samples
                    if max(abs(r), abs(p)) > ATT_FAIL_DEG)
    print(f"samples over {ATT_WARN_DEG:.0f} deg: {over_warn}  "
          f"over {ATT_FAIL_DEG:.0f} deg: {over_fail}")
    print(f"worst attitude: {worst:.1f} deg")

    ok = over_fail == 0
    print(f"\nVERDICT: {'STABLE' if ok else 'UNSTABLE'} "
          f"(worst {worst:.1f} deg vs {ATT_FAIL_DEG:.0f} deg limit)")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=TAKEOFF_ALT)
    ap.add_argument("--hover", type=float, default=20.0, help="hover seconds")
    ap.add_argument("--square", type=float, default=0.0,
                    help="side length of a square waypoint leg (0 = skip)")
    args = ap.parse_args()

    rclpy.init()
    n = HoverTest()

    print("=" * 70)
    print(" PHASE 2 BASIC FLIGHT TEST (no perception, no avoidance, no BT)")
    print("=" * 70)

    n.phase = "connect"
    if not n.wait_for(lambda: n.state.connected, 60, "FCU connection"):
        return 2
    if not n.wait_for(lambda: n.alt() != 0.0 or len(n.samples) > 3, 90,
                      "local position"):
        return 2
    print(f"connected; mode={n.state.mode} alt={n.alt():.2f}")

    print("\n[1] GUIDED")
    n.phase = "guided"
    req = SetMode.Request(); req.custom_mode = "GUIDED"
    n.call(n.cli_mode, req)
    if not n.wait_for(lambda: n.state.mode == "GUIDED", 20, "GUIDED"):
        return 1

    print("[2] ARM")
    n.phase = "arm"
    for attempt in range(10):
        r = n.call(n.cli_arm, CommandBool.Request(value=True))
        n.spin(1.0)
        if n.state.armed:
            break
        print(f"    arm attempt {attempt + 1} "
              f"(success={getattr(r, 'success', None)})")
    if not n.state.armed:
        print("    FAILED to arm"); summarize(n.samples); return 1
    print("    armed")

    print(f"[3] TAKEOFF to {args.alt:.1f} m")
    n.phase = "takeoff"
    req = CommandTOL.Request(); req.altitude = float(args.alt)
    n.call(n.cli_takeoff, req)
    reached = n.wait_for(lambda: n.alt() > args.alt - 0.5, 90, "takeoff altitude")
    print(f"    alt={n.alt():.2f} reached={reached}")

    print(f"[4] HOVER {args.hover:.0f}s (position hold at takeoff point)")
    n.phase = "hover"
    p = n.pose.pose.position
    n.goto(p.x, p.y, args.alt)
    n.spin(args.hover)
    print(f"    alt={n.alt():.2f} pos=({n.pose.pose.position.x:.2f}, "
          f"{n.pose.pose.position.y:.2f})")

    if args.square > 0:
        print(f"[5] SQUARE {args.square:.1f} m")
        n.phase = "square"
        p = n.pose.pose.position
        x0, y0 = p.x, p.y
        for dx, dy in ((args.square, 0), (args.square, args.square),
                       (0, args.square), (0, 0)):
            n.goto(x0 + dx, y0 + dy, args.alt)
            n.wait_for(lambda: math.dist(
                (n.pose.pose.position.x, n.pose.pose.position.y),
                (x0 + dx, y0 + dy)) < 0.8, 40, f"waypoint ({dx},{dy})")
            print(f"    at ({n.pose.pose.position.x:.2f}, "
                  f"{n.pose.pose.position.y:.2f}) alt={n.alt():.2f}")

    print("[6] LAND")
    n.phase = "land"
    n._sp = None
    n.call(n.cli_land, CommandTOL.Request())
    n.wait_for(lambda: not n.state.armed, 90, "disarm after landing")
    print(f"    armed={n.state.armed} alt={n.alt():.2f}")

    ok = summarize(n.samples)
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
