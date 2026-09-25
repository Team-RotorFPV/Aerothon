#!/usr/bin/env python3
"""Request usable MAVLink stream rates from ArduPilot.

WHY (Phase 2, from VERIFICATION.md 0.13c)
    Measured on the live stack after Phase 0:

        /mavros/local_position/pose   1.5 - 2.8 Hz
        /scan                         6.8 Hz
        /camera/image                 4.9 Hz

    goal.md Q27 requires LiDAR >= 8 Hz and Q14 requires 10 FPS for QR, and
    ~2 Hz position is not a basis for closed-loop guidance. The low rate is not
    a ROS problem: in SITL, MAVProxy owns SERIAL0 and requests its own modest
    default stream rates, and our router simply relays whatever arrives, so
    MAVROS inherits MAVProxy's rates.

    MAV_CMD_SET_MESSAGE_INTERVAL (511) sets per-message intervals on the
    channel and is the supported way to ask for more. It applies to the real
    Pixhawk over TELEM2 exactly as it does to SITL.

Usage:
    python3 scripts/set_stream_rates.py            # apply defaults
    python3 scripts/set_stream_rates.py --verify   # apply, then measure
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong

MAV_CMD_SET_MESSAGE_INTERVAL = 511

# message id -> desired Hz. Attitude and local position drive guidance, so they
# get the highest rate; the rest are sized to goal.md's stated needs.
DESIRED = {
    30:  50.0,   # ATTITUDE            - attitude rail + control loop
    32:  50.0,   # LOCAL_POSITION_NED  - position setpoint loop
    33:  10.0,   # GLOBAL_POSITION_INT - global position
    24:   5.0,   # GPS_RAW_INT         - satellite count, HDOP (Q27 interlock)
    74:  10.0,   # VFR_HUD             - groundspeed, climb rate
    1:    4.0,   # SYS_STATUS          - battery, sensor health
    2:    4.0,   # SYSTEM_TIME
    27:  10.0,   # RAW_IMU
    65:   5.0,   # RC_CHANNELS         - RC / failsafe state
    193:  2.0,   # EKF_STATUS_REPORT   - EKF health (Q27 interlock)
}

MESSAGE_NAMES = {
    30: "ATTITUDE", 32: "LOCAL_POSITION_NED", 33: "GLOBAL_POSITION_INT",
    24: "GPS_RAW_INT", 74: "VFR_HUD", 1: "SYS_STATUS", 2: "SYSTEM_TIME",
    27: "RAW_IMU", 65: "RC_CHANNELS", 193: "EKF_STATUS_REPORT",
}


class RateSetter(Node):
    def __init__(self):
        super().__init__("stream_rate_setter")
        self.state = State()
        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.pose_count = 0
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._count, qos_profile_sensor_data)
        self.cli = self.create_client(CommandLong, "/mavros/cmd/command")

    def _count(self, _):
        self.pose_count += 1

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_connected(self, timeout=60.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected:
                return True
        return False

    def set_interval(self, msg_id, hz):
        req = CommandLong.Request()
        req.command = MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(msg_id)
        req.param2 = float(1_000_000.0 / hz)     # interval in microseconds
        fut = self.cli.call_async(req)
        end = time.time() + 3.0
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if fut.done() and fut.result() is not None:
            return bool(fut.result().success)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true",
                    help="measure /mavros/local_position/pose rate before and after")
    args = ap.parse_args()

    rclpy.init()
    n = RateSetter()

    if not n.wait_connected():
        print("ERROR: FCU never connected; is the stack running?")
        return 2
    if not n.cli.wait_for_service(timeout_sec=15.0):
        print("ERROR: /mavros/cmd/command unavailable")
        return 2

    before = None
    if args.verify:
        n.pose_count = 0
        n.spin(6.0)
        before = n.pose_count / 6.0
        print(f"before: /mavros/local_position/pose ~ {before:.2f} Hz\n")

    print(f"{'message':22s} {'id':>4s} {'target Hz':>10s} {'result':>8s}")
    ok_count = 0
    for msg_id, hz in sorted(DESIRED.items()):
        ok = n.set_interval(msg_id, hz)
        ok_count += ok
        print(f"{MESSAGE_NAMES.get(msg_id, '?'):22s} {msg_id:4d} {hz:10.1f} "
              f"{'OK' if ok else 'REJECTED':>8s}")
        n.spin(0.15)

    print(f"\n{ok_count}/{len(DESIRED)} intervals accepted")

    if args.verify:
        n.spin(2.0)
        n.pose_count = 0
        n.spin(6.0)
        after = n.pose_count / 6.0
        print(f"\nafter:  /mavros/local_position/pose ~ {after:.2f} Hz")
        if before is not None and before > 0:
            print(f"change: {after / before:.1f}x")

    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok_count == len(DESIRED) else 1


if __name__ == "__main__":
    sys.exit(main())
