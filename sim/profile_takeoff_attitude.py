#!/usr/bin/env python3
"""Record the attitude profile of a GUIDED takeoff against the live stack.

Purpose: the Phase 0 attitude rail aborted a takeoff at roll=-17.8 pitch=46.3.
Before tuning the limit, establish what the aircraft actually does — a
momentary spike and a sustained tilt need different answers, and picking a
threshold without the data is how the 45 deg guess got made in the first place.

Prints, per 0.5 s bucket: altitude, roll, pitch, and the running worst-case,
plus how long the aircraft spent beyond each candidate limit.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 sim/profile_takeoff_attitude.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State

DURATION_S = 45.0
CANDIDATE_LIMITS = (30.0, 45.0, 60.0, 75.0)


def rp_deg(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return math.degrees(roll), math.degrees(math.asin(sinp))


class Profiler(Node):
    def __init__(self):
        super().__init__("takeoff_attitude_profiler")
        self.samples = []          # (t, roll, pitch, alt)
        self.state = State()
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.pub_start = self.create_publisher(Bool, "/mission/start", 10)
        self.t0 = time.time()

    def _on_pose(self, m):
        r, p = rp_deg(m.pose.orientation)
        self.samples.append((time.time() - self.t0, r, p, m.pose.position.z))


def main():
    rclpy.init()
    n = Profiler()

    print("Waiting for FCU...")
    end = time.time() + 60
    while rclpy.ok() and not n.state.connected and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.state.connected:
        print("FCU never connected."); return 1

    print("Starting mission and recording attitude...")
    n.t0 = time.time()
    n.samples.clear()
    for _ in range(5):
        n.pub_start.publish(Bool(data=True))
        rclpy.spin_once(n, timeout_sec=0.1)

    end = time.time() + DURATION_S
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    s = n.samples
    if not s:
        print("No pose samples received."); return 1

    print(f"\n{len(s)} samples over {s[-1][0]:.1f}s "
          f"(~{len(s)/max(s[-1][0], 0.001):.0f} Hz)\n")
    print(f"{'t(s)':>6} {'alt(m)':>7} {'roll':>8} {'pitch':>8}")
    bucket = 0.5
    nxt = 0.0
    for t, r, p, a in s:
        if t >= nxt:
            print(f"{t:6.1f} {a:7.2f} {r:8.1f} {p:8.1f}")
            nxt += bucket

    worst_r = max(abs(x[1]) for x in s)
    worst_p = max(abs(x[2]) for x in s)
    print(f"\nworst |roll| = {worst_r:.1f} deg    worst |pitch| = {worst_p:.1f} deg")

    dt = s[-1][0] / len(s)
    print(f"\n{'limit(deg)':>11} {'samples over':>13} {'time over(s)':>13} "
          f"{'longest run(s)':>15}")
    for lim in CANDIDATE_LIMITS:
        over = [max(abs(r), abs(p)) > lim for _, r, p, _ in s]
        count = sum(over)
        longest = cur = 0
        for o in over:
            cur = cur + 1 if o else 0
            longest = max(longest, cur)
        print(f"{lim:11.0f} {count:13d} {count*dt:13.2f} {longest*dt:15.2f}")

    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
