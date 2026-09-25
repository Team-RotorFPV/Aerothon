#!/usr/bin/env python3
"""Phase 2 — LIVE verification of camera pointing and telemetry rates.

Acceptance criteria from PHASE_PLAN.md Phase 2:
  * a commanded -90 deg is CONFIRMED at the joint, within tolerance, within a
    bounded settle time
  * a perception stage requesting NADIR BLOCKS while the joint is elsewhere —
    it must not time out to success the way ScanStartQR did
  * telemetry stream rates support closed-loop guidance

Prerequisite: scripts/launch_level6_sim.sh is running (AEROTHON_HEADLESS=1
recommended) and scripts/set_stream_rates.py has been applied.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 sim/verify_phase2_live.py
"""

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64, String
from mavros_msgs.srv import CommandLong

SETTLE_BUDGET_S = 8.0
POSE_RATE_MIN_HZ = 10.0        # closed-loop guidance needs well above ~2 Hz

RESULTS = []


def check(label, ok, detail=""):
    RESULTS.append((label, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))
    return ok


class Verifier(Node):
    def __init__(self):
        super().__init__("phase2_live_verifier")
        self.cam = None
        self.counts = {"pose": 0, "scan": 0, "image": 0}

        self.create_subscription(String, "/camera/pose_state", self._on_cam, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: self._bump("pose"), qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: self._bump("scan"), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image",
                                 lambda m: self._bump("image"), qos_profile_sensor_data)

        self.pub_pose = self.create_publisher(String, "/camera/set_pose", 10)
        self.pub_raw = self.create_publisher(Float64, "/gimbal/cmd_pitch", 10)
        self.cli_cmd = self.create_client(CommandLong, "/mavros/cmd/command")

    def apply_stream_rates(self):
        """Raise MAVLink stream rates right before measuring them."""
        if not self.cli_cmd.wait_for_service(timeout_sec=10.0):
            return False
        ok = True
        for msg_id, hz in ((30, 50.0), (32, 50.0), (33, 10.0), (74, 10.0)):
            req = CommandLong.Request()
            req.command = 511                       # SET_MESSAGE_INTERVAL
            req.param1 = float(msg_id)
            req.param2 = float(1_000_000.0 / hz)
            fut = self.cli_cmd.call_async(req)
            end = time.time() + 3.0
            while rclpy.ok() and not fut.done() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            ok = ok and fut.done() and fut.result() is not None \
                and fut.result().success
        self.spin(1.0)
        return ok

    def _bump(self, k):
        self.counts[k] += 1

    def _on_cam(self, m):
        try:
            self.cam = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def spin(self, s):
        end = time.time() + s
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, pred, timeout):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def wait_for_subscriber(self, pub, timeout=15.0):
        """ROS 2 discovery is asynchronous.

        Publishing immediately after creating a publisher drops the message on
        the floor because camera_ctrl has not matched the subscription yet.
        The same race silently swallowed /mission/start in Phase 0.
        """
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if pub.get_subscription_count() > 0:
                self.spin(0.2)          # let the match settle
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def command(self, pose, repeats=6):
        self.wait_for_subscriber(self.pub_pose)
        for _ in range(repeats):
            self.pub_pose.publish(String(data=pose))
            self.spin(0.1)

    def settled_at(self, pose):
        c = self.cam
        return bool(c and c.get("requested") == pose
                    and c.get("settled") and not c.get("stale"))


def measure_settle(v, pose, expect_deg):
    v.command(pose)
    t0 = time.time()
    ok = v.wait_for(lambda: v.settled_at(pose), SETTLE_BUDGET_S)
    dt = time.time() - t0
    actual = v.cam.get("actual_rad") if v.cam else None
    actual_deg = None if actual is None else math.degrees(actual)
    detail = (f"settled in {dt:.2f}s, joint={actual_deg:.1f} deg"
              if ok and actual_deg is not None else f"cam={v.cam}")
    check(f"{pose}: joint CONFIRMED within {SETTLE_BUDGET_S:.0f}s", ok, detail)
    if ok and actual_deg is not None:
        check(f"{pose}: measured angle matches {expect_deg:+.0f} deg",
              abs(actual_deg - expect_deg) <= 2.5,
              f"{actual_deg:+.2f} deg (want {expect_deg:+.0f})")
    return ok


def main():
    rclpy.init()
    v = Verifier()
    print("=" * 70)
    print(" PHASE 2 LIVE VERIFICATION — camera pointing + telemetry rates")
    print("=" * 70)

    print("\n[0] camera_ctrl is publishing state")
    ok = v.wait_for(lambda: v.cam is not None, 30)
    check("/camera/pose_state is published", ok, str(v.cam)[:90])
    if not ok:
        return finish(v)

    print("\n[1] Named poses are commanded and CONFIRMED at the joint")
    measure_settle(v, "NADIR", -90.0)
    measure_settle(v, "FORWARD", 0.0)
    measure_settle(v, "ALIGN", -45.0)
    measure_settle(v, "NADIR", -90.0)

    print("\n[2] settled is a MEASUREMENT, not an assumption")
    # Drive the joint away behind camera_ctrl's back, straight to the Gazebo
    # position controller. camera_ctrl still believes it commanded NADIR, but
    # the joint is elsewhere, so `settled` must go false.
    v.command("NADIR")
    v.wait_for(lambda: v.settled_at("NADIR"), SETTLE_BUDGET_S)
    was_settled = v.settled_at("NADIR")
    for _ in range(25):
        v.pub_raw.publish(Float64(data=0.0))     # force joint FORWARD
        v.spin(0.05)
    v.spin(1.0)
    unsettled = not v.settled_at("NADIR")
    err = (v.cam or {}).get("error_deg")
    check("a joint dragged off target reports settled=false",
          was_settled and unsettled,
          f"was_settled={was_settled} now_settled={not unsettled} error={err}")

    # And it recovers: camera_ctrl re-asserts the command until measured.
    ok = v.wait_for(lambda: v.settled_at("NADIR"), 15.0)
    check("camera_ctrl re-asserts until the joint is measured back on target",
          ok, f"error_deg={(v.cam or {}).get('error_deg')}")

    print("\n[3] Telemetry rates support closed-loop guidance")
    # Apply the intervals HERE, immediately before measuring. In SITL MAVProxy
    # re-requests its own lower rates within about twenty seconds, so measuring
    # minutes after an earlier application tests the decay, not the fix.
    ok = v.apply_stream_rates()
    check("MAV_CMD_SET_MESSAGE_INTERVAL accepted", ok)

    for k in v.counts:
        v.counts[k] = 0
    window = 8.0
    v.spin(window)
    rates = {k: c / window for k, c in v.counts.items()}
    check(f"/mavros/local_position/pose >= {POSE_RATE_MIN_HZ:.0f} Hz "
          f"immediately after applying",
          rates["pose"] >= POSE_RATE_MIN_HZ, f"{rates['pose']:.1f} Hz")
    print(f"       /scan          {rates['scan']:.1f} Hz")
    print(f"       /camera/image  {rates['image']:.1f} Hz")
    print("       (sensor rates are Gazebo-side and scale with the 0.55 "
          "real-time factor; see VERIFICATION.md 2.2)")

    # Document the decay rather than pretending it is not there.
    print("\n[3b] Known SITL limitation: MAVProxy re-requests lower rates")
    v.spin(30.0)
    v.counts["pose"] = 0
    v.spin(window)
    decayed = v.counts["pose"] / window
    print(f"       pose rate ~40s later: {decayed:.1f} Hz "
          f"({'decayed' if decayed < POSE_RATE_MIN_HZ else 'held'})")
    print("       No MAVProxy exists in the deployed topology (goal.md Q22/Q28),")
    print("       so this does not apply to the real aircraft. Phase 11 removes")
    print("       MAVProxy from the simulation.")

    return finish(v)


def finish(v):
    print("\n" + "=" * 70)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    for label, ok, detail in RESULTS:
        if not ok:
            print(f" FAILED: {label}  {detail}")
    print(f" PHASE 2 LIVE: {passed}/{len(RESULTS)} checks passed")
    print("=" * 70)
    v.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
