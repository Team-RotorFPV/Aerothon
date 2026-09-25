#!/usr/bin/env python3
"""Drive and observe a full Mission 2 run against the live stack.

Written as a single long-lived node rather than a series of `ros2 topic` calls:
short-lived CLI processes frequently fail to complete DDS discovery against
this stack and report topics as unpublished when they are publishing fine.
One node that stays up and spins sees everything.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 sim/run_mission_live.py --watch 300

Exit: 0 the mission reached a terminal outcome, 1 it never left WAITING,
      2 the stack was not usable.
"""

import argparse
import json
import math
import sys
import time

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String


def latched():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MissionRunner(Node):
    def __init__(self):
        # SIM time, so the recorded track -- and the flight time read off it --
        # is what the aircraft experienced, not what a slow host took.
        super().__init__("mission_live_runner", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time",
                                      rclpy.parameter.Parameter.Type.BOOL,
                                      True)])
        self.track = []
        self.state = State()
        self.pose = PoseStamped()
        self.mission_state = None
        self.result = None
        self.qr = {}
        self.winch = {}
        self.camera = {}
        self.payload = None           # ground-truth payload pose, WORLD frame
        self.transitions = []

        best = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), best)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(String, "/mission/state", self._on_state, 10)
        self.create_subscription(String, "/mission/result",
                                 lambda m: setattr(self, "result", m.data), latched())
        self.create_subscription(String, "/percep/qr/detail",
                                 lambda m: self._json(m, "qr"), 10)
        self.create_subscription(String, "/winch/status",
                                 lambda m: self._json(m, "winch"), 10)
        self.create_subscription(String, "/camera/pose_state",
                                 lambda m: self._json(m, "camera"), 10)
        # Where the payload really is (Gazebo, for GRADING -- see
        # sim/check_track.py). Nothing in the mission reads this topic.
        self.create_subscription(
            Pose, "/sim/payload_pose",
            lambda m: setattr(self, "payload", (m.position.x, m.position.y,
                                                m.position.z)), best)

        self.pub_start = self.create_publisher(Bool, "/mission/start", 10)

    def _on_pose(self, m):
        self.pose = m
        t = self.get_clock().now().nanoseconds * 1e-9
        # 10 Hz of sim time is plenty to resolve a 1.5 m red-zone clearance.
        if not self.track or t - self.track[-1][0] >= 0.1:
            p = m.pose.position
            q = m.pose.orientation
            # Roll and pitch too: a collision or a loss of control shows as a
            # tilt spike, and the grader must see it even when the tree goes
            # on to report COMPLETED.
            roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z),
                                           1 - 2 * (q.x * q.x + q.y * q.y)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0,
                                           2 * (q.w * q.y - q.z * q.x)))))
            self.track.append((t, p.x, p.y, p.z, bool(self.state.armed),
                               str(self.mission_state), roll, pitch,
                               str(self.state.mode), self.payload))

    def write_track(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write("t_sim,x,y,z,armed,state,roll_deg,pitch_deg,mode,"
                    "payload_wx,payload_wy,payload_wz\n")
            for t, x, y, z, armed, s, r, pt, mode, pay in self.track:
                pw = ",".join(f"{v:.3f}" for v in pay) if pay else ",,"
                f.write(f"{t:.2f},{x:.3f},{y:.3f},{z:.3f},{int(armed)},{s},"
                        f"{r:.1f},{pt:.1f},{mode},{pw}\n")

    def _json(self, msg, attr):
        try:
            setattr(self, attr, json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _on_state(self, msg):
        s = msg.data
        if s != self.mission_state:
            self.transitions.append((time.time(), s))
            self.get_logger().info(f"STATE -> {s}")
        self.mission_state = s

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, pred, timeout, what):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        print(f"  TIMEOUT waiting for {what}")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=float, default=300.0)
    ap.add_argument("--track", default="/tmp/aerothon_track.csv",
                    help="where to write the flown track (sim time, local ENU)")
    ap.add_argument("--settle", type=float, default=20.0,
                    help="discovery settling time before doing anything")
    args = ap.parse_args()

    rclpy.init()
    n = MissionRunner()

    print("=" * 72)
    print(" LIVE MISSION RUN")
    print("=" * 72)

    print(f"\n[0] settling discovery ({args.settle:.0f}s)")
    n.spin(args.settle)

    if not n.wait_for(lambda: n.state.connected, 90, "FCU connection"):
        print("stack not usable"); n.destroy_node(); rclpy.shutdown(); return 2
    print(f"  FCU connected, mode={n.state.mode}")

    if not n.wait_for(lambda: n.mission_state is not None, 60, "/mission/state"):
        print("behaviour tree not publishing"); n.destroy_node(); rclpy.shutdown(); return 2
    print(f"  mission state = {n.mission_state}")
    print(f"  camera = {n.camera.get('requested')} settled={n.camera.get('settled')}")
    print(f"  winch  = {n.winch.get('state')}")

    print("\n[1] START")
    for _ in range(40):
        n.pub_start.publish(Bool(data=True))
        n.spin(0.25)
        if n.mission_state not in (None, "WAITING"):
            break
    if n.mission_state in (None, "WAITING"):
        print("  mission never left WAITING")
        n.destroy_node(); rclpy.shutdown(); return 1
    print(f"  left WAITING -> {n.mission_state}")

    print(f"\n[2] watching for {args.watch:.0f}s\n")
    print(f"{'t(s)':>6} {'state':<18} {'x':>7} {'y':>7} {'z':>6} "
          f"{'cam':<9} {'qr':<26} {'winch':<10}")
    t0 = time.time()
    end = t0 + args.watch
    nxt = 0.0
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        el = time.time() - t0
        if el >= nxt:
            p = n.pose.pose.position
            acc = (n.qr.get("accepted") or "")[-16:]
            off = n.qr.get("offset") or [0, 0, 0]
            print(f"{el:6.0f} {str(n.mission_state):<18} "
                  f"{p.x:7.2f} {p.y:7.2f} {p.z:6.2f} "
                  f"{str(n.camera.get('requested')):<9} "
                  f"{acc + ' ' + str(off[:2]):<26} "
                  f"{str(n.winch.get('state')):<10}")
            nxt += 8.0
        if n.result is not None:
            print(f"\n  terminal outcome reached at t={el:.0f}s")
            # Keep recording until the FCU's own state shows the disarm. The
            # result latches on the tree's side first; stopping there left
            # the last track sample armed (seed 1002) and the grader rightly
            # refused to take "landed and disarmed" on trust.
            tail_end = time.time() + 20.0
            while rclpy.ok() and time.time() < tail_end and n.state.armed:
                rclpy.spin_once(n, timeout_sec=0.05)
            n.spin(2.0)
            break

    print("\n" + "=" * 72)
    print(" RESULT")
    print("=" * 72)
    print(f" latched /mission/result : {n.result}")
    print(f" final mission state     : {n.mission_state}")
    p = n.pose.pose.position
    print(f" final position          : ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
    print(f" armed                   : {n.state.armed}  mode={n.state.mode}")
    print(f" qr                      : {n.qr}")
    print(f" winch                   : {n.winch}")
    print("\n state transitions:")
    for t, s in n.transitions:
        print(f"   +{t - t0:6.1f}s  {s}")

    # Flight time in SIM seconds: first armed sample to the last one.
    armed = [row[0] for row in n.track if row[4]]   # t of armed samples
    if armed:
        print(f"\n flight time (sim)       : {armed[-1] - armed[0]:.1f} s "
              f"(rulebook window 900 s)")
    try:
        n.write_track(args.track)
        print(f" track                   : {args.track} ({len(n.track)} samples)")
    except OSError as exc:
        print(f" track not written: {exc}")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
