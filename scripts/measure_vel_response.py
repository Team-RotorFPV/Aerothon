#!/usr/bin/env python3
"""Measure how fast the aircraft follows a velocity command: vel_response_s.

The corridor navigator's wind observer compares the sideways velocity the
airframe HAS with the one it was told to have, delayed by this lag. The
default, 0.6 s, is the simulated airframe's; sim/test_corridor_stress.py found
that at 1.0 s the 1.7 m slalom in a gusty crosswind still touches. So it is
measured, on the aircraft, before the corridor is flown.

    1. Open field, at least 10 m clear all round, light wind.
    2. The pilot takes off, holds 3 m or more, switches to GUIDED.
    3. python3 scripts/measure_vel_response.py        (bring-up running)

It commands four 0.5 m/s sideways steps (left, stop, right, stop; 4 s each,
2 m of travel at most), then stops and holds. The pilot's mode switch ends it
at any moment. It prints the median time to 63% of each step:

    ros2 param set /velocity_controller vel_response_s <that>
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

STEP_MPS, HOLD_S = 0.5, 4.0
PLAN = (STEP_MPS, 0.0, -STEP_MPS, 0.0)
FRAME_BODY_OFFSET_NED = 9
VEL_YAWRATE_MASK = 1479      # velocity + yaw rate, as velocity_controller sends


def time_constant(samples, t_step, v0, v1):
    """Seconds from `t_step` until the (t, v) samples cover 63% of v0 -> v1,
    or None if they never do."""
    target = v0 + 0.632 * (v1 - v0)
    rising = v1 > v0
    for t, v in samples:
        if t >= t_step and ((v >= target) if rising else (v <= target)):
            return t - t_step
    return None


class Probe(Node):
    def __init__(self):
        super().__init__("measure_vel_response")
        self.state, self.yaw, self.samples = None, 0.0, []
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self._on_vel, qos_profile_sensor_data)
        self.pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.alt = 0.0

    def _on_state(self, m):
        self.state = m

    def _on_pose(self, m):
        q = m.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.alt = m.pose.position.z

    def _on_vel(self, m):
        v = m.twist.linear
        left = -math.sin(self.yaw) * v.x + math.cos(self.yaw) * v.y
        self.samples.append((time.monotonic(), left))

    def guided(self):
        return self.state is not None and self.state.armed and self.state.mode == "GUIDED"

    def command(self, vy_left):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = FRAME_BODY_OFFSET_NED
        sp.type_mask = VEL_YAWRATE_MASK
        sp.velocity.y = float(vy_left)
        self.pub.publish(sp)

    def spin_for(self, seconds, vy_left):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if not self.guided():
                return False
            self.command(vy_left)
            rclpy.spin_once(self, timeout_sec=0.05)
        return True


def main():
    rclpy.init()
    node = Probe()
    try:
        end = time.monotonic() + 5.0
        while time.monotonic() < end and node.state is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.guided() or node.alt < 3.0:
            sys.exit("Needs the aircraft armed, in GUIDED, at 3 m or more.")
        taus, v_prev = [], 0.0
        node.spin_for(2.0, 0.0)
        for v in PLAN:
            t0 = time.monotonic()
            if not node.spin_for(HOLD_S, v):
                sys.exit("Left GUIDED: stopped.")
            tau = time_constant([s for s in node.samples if s[0] >= t0], t0, v_prev, v)
            print(f"  step {v_prev:+.1f} -> {v:+.1f} m/s: "
                  + (f"{tau:.2f} s" if tau is not None else "never reached 63%"))
            if tau is not None:
                taus.append(tau)
            v_prev = v
        node.spin_for(1.0, 0.0)
        if len(taus) < 3:
            sys.exit("Too few steps settled; repeat in less wind.")
        taus.sort()
        print(f"\nvel_response_s = {taus[len(taus) // 2]:.2f}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
