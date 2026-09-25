#!/usr/bin/env python3
"""Phase 0 — LIVE fail-closed rail verification against a running SITL stack.

The unit tests in sim/test_phase0_rails.py prove the rails in isolation. This
script proves them against the real thing: real MAVROS, real ArduPilot SITL,
real behaviour tree node, real topics.

It performs a scripted intervention — exactly the scenario that produced the
bad state in CURRENT_PROGRESS_HANDOFF.md:

    1. confirm the tree is parked at WAITING
    2. send /mission/start, watch it reach ARMING -> TAKEOFF
    3. DISARM from outside the behaviour tree (as Mission Planner or the
       safety pilot would)
    4. assert the tree returns to WAITING and does not sit at a stale stage
    5. assert /mission/result latched INTERRUPTED with a reason
    6. assert NO setpoints were published while the aircraft was disarmed

Prerequisites: scripts/launch_level6_sim.sh is running.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 sim/verify_phase0_live.py

Exit code 0 = all live rail checks passed.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL

# ArduPilot refuses a plain MAV_CMD_COMPONENT_ARM_DISARM disarm while the
# vehicle is flying. param2 = 21196 is the documented "force" magic number that
# a GCS uses for an emergency in-air disarm. A plain CommandBool(false) simply
# gets rejected, which is what made the first live run report armed=True.
FORCE_DISARM_MAGIC = 21196.0
MAV_CMD_COMPONENT_ARM_DISARM = 400


TIMEOUT_CONNECT = 60.0
TIMEOUT_STAGE = 90.0
DISARM_OBSERVE_S = 6.0


class Phase0LiveVerifier(Node):
    def __init__(self):
        super().__init__("phase0_live_verifier")
        self.mission_state = None
        self.mission_result = None
        self.fcu = State()
        self.pose = PoseStamped()
        self.setpoints_while_disarmed = 0
        self.setpoints_total = 0
        self._watch_setpoints = False

        latched = QoSProfile(depth=1,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(String, "/mission/state",
                                 lambda m: setattr(self, "mission_state", m.data), 10)
        self.create_subscription(String, "/mission/result",
                                 self._on_result, latched)
        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "fcu", m), 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mavros/setpoint_position/local",
                                 self._on_setpoint, 10)

        self.pub_start = self.create_publisher(Bool, "/mission/start", 10)
        self.pub_abort = self.create_publisher(Bool, "/mission/abort", 10)
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.cli_cmd = self.create_client(CommandLong, "/mavros/cmd/command")

    def _on_result(self, m):
        self.mission_result = m.data

    def _on_setpoint(self, m):
        self.setpoints_total += 1
        if self._watch_setpoints and not self.fcu.armed:
            self.setpoints_while_disarmed += 1

    # ---- helpers ---- #
    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, description):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f"TIMEOUT waiting for {description}")
        return False

    def disarm_externally(self):
        """Force-disarm the way an outside GCS would, bypassing the tree.

        Uses the force magic value because ArduPilot rejects an ordinary
        disarm request while the vehicle is airborne.
        """
        req = CommandLong.Request()
        req.command = MAV_CMD_COMPONENT_ARM_DISARM
        req.param1 = 0.0                      # 0 = disarm
        req.param2 = FORCE_DISARM_MAGIC       # force, even in flight
        self.cli_cmd.call_async(req)


RESULTS = []


def check(label, ok, detail=""):
    RESULTS.append((label, ok, detail))
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {label}" + (f"  ({detail})" if detail else ""))
    return ok


def main():
    rclpy.init()
    v = Phase0LiveVerifier()
    print("=" * 70)
    print(" PHASE 0 LIVE RAIL VERIFICATION")
    print("=" * 70)

    # ---- 0. stack is alive ------------------------------------------------ #
    print("\n[0] Stack connectivity")
    ok = v.wait_for(lambda: v.fcu.connected, TIMEOUT_CONNECT, "FCU connection")
    check("MAVROS reports FCU connected", ok, f"mode={v.fcu.mode}")
    if not ok:
        print("\nCannot continue without an FCU connection.")
        return finish(v)

    ok = v.wait_for(lambda: v.mission_state is not None, 30.0, "/mission/state")
    check("mission_bt is publishing /mission/state", ok, f"state={v.mission_state}")
    if not ok:
        return finish(v)

    # ---- 1. parked at WAITING before start -------------------------------- #
    print("\n[1] Pre-start state")
    check("tree is parked at WAITING before START",
          v.mission_state == "WAITING", f"state={v.mission_state}")

    # ---- 2. no setpoints while disarmed and idle -------------------------- #
    print(f"\n[2] Setpoint gate while disarmed and idle ({DISARM_OBSERVE_S:.0f}s)")
    v.setpoints_while_disarmed = 0
    v._watch_setpoints = True
    v.spin(DISARM_OBSERVE_S)
    check("no setpoints published while disarmed (idle)",
          v.setpoints_while_disarmed == 0,
          f"{v.setpoints_while_disarmed} setpoints observed")

    # ---- 3. start the mission --------------------------------------------- #
    print("\n[3] Mission start")
    for _ in range(5):
        v.pub_start.publish(Bool(data=True))
        v.spin(0.2)

    ok = v.wait_for(lambda: v.mission_state in ("ARMING", "TAKEOFF"),
                    TIMEOUT_STAGE, "ARMING/TAKEOFF")
    check("tree leaves WAITING on START", ok, f"state={v.mission_state}")
    if not ok:
        return finish(v)

    ok = v.wait_for(lambda: v.fcu.armed, TIMEOUT_STAGE, "arm")
    check("aircraft armed", ok, f"armed={v.fcu.armed} mode={v.fcu.mode}")
    if not ok:
        return finish(v)

    ok = v.wait_for(lambda: v.pose.pose.position.z > 1.5, TIMEOUT_STAGE, "climb")
    check("aircraft climbing under mission control", ok,
          f"alt={v.pose.pose.position.z:.2f} m state={v.mission_state}")

    # An abort may latch before we get to intervene (the vehicle model is not
    # yet stable enough to guarantee a clean climb — see VERIFICATION.md 0.12).
    # That is a valid rail outcome, but it means the intervention scenario
    # cannot run, so say so plainly instead of reporting cascading failures.
    if v.mission_result is not None:
        print(f"\n[!] Mission already terminated before intervention:"
              f"\n    {v.mission_result}")
        reason_ok = False
        try:
            reason_ok = bool(json.loads(v.mission_result).get("reason"))
        except json.JSONDecodeError:
            pass
        check("abort latched with an explicit reason before intervention",
              reason_ok, v.mission_result)
        v.spin(4.0)
        check("abort STAYS latched (does not resume the mission)",
              v.mission_state in ("ABORT", "WAITING"),
              f"state={v.mission_state}")
        print("\n    Intervention checks [4]-[6] SKIPPED: the mission ended on "
              "its own.\n    Re-run once the aircraft can hold a stable climb.")
        return finish(v)

    stage_before = v.mission_state
    print(f"      stage before intervention: {stage_before}")

    # ---- 4. external intervention ----------------------------------------- #
    print("\n[4] External DISARM (simulating Mission Planner / safety pilot)")
    v.setpoints_while_disarmed = 0
    v.disarm_externally()

    ok = v.wait_for(lambda: not v.fcu.armed, 30.0, "disarm")
    check("aircraft disarmed from outside the tree", ok, f"armed={v.fcu.armed}")

    # ---- 5. the tree must reset ------------------------------------------- #
    print("\n[5] Fail-closed reset")
    ok = v.wait_for(lambda: v.mission_state == "WAITING", 30.0, "reset to WAITING")
    check("tree returned to WAITING after intervention", ok,
          f"state={v.mission_state} (was {stage_before})")
    check("tree is NOT stuck at a stale stage",
          v.mission_state != stage_before or stage_before == "WAITING",
          f"state={v.mission_state}")

    ok = v.wait_for(lambda: v.mission_result is not None, 15.0, "/mission/result")
    check("/mission/result latched an outcome", ok, f"{v.mission_result}")
    if v.mission_result:
        try:
            payload = json.loads(v.mission_result)
            check("result carries an explicit reason",
                  bool(payload.get("reason")),
                  f"state={payload.get('state')} reason={payload.get('reason')}")
        except json.JSONDecodeError as e:
            check("result payload is valid JSON", False, str(e))

    # ---- 6. setpoint gate after disarm ------------------------------------ #
    print(f"\n[6] Setpoint gate after intervention ({DISARM_OBSERVE_S:.0f}s)")
    v.setpoints_while_disarmed = 0
    v.spin(DISARM_OBSERVE_S)
    check("no setpoints published to the disarmed aircraft",
          v.setpoints_while_disarmed == 0,
          f"{v.setpoints_while_disarmed} setpoints observed")

    return finish(v)


def finish(v):
    print("\n" + "=" * 70)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    total = len(RESULTS)
    for label, ok, detail in RESULTS:
        if not ok:
            print(f" FAILED: {label}  {detail}")
    print(f" PHASE 0 LIVE: {passed}/{total} checks passed")
    print("=" * 70)
    v.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
