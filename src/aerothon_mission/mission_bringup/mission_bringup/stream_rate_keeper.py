#!/usr/bin/env python3
"""Hold ArduPilot's MAVLink stream rates where the guidance loop needs them.

WHY (Phase 2, VERIFICATION.md 2.2)
    Measured after Phase 0: /mavros/local_position/pose arrived at 1.5-2.8 Hz.
    That is not a basis for closed-loop position control, and goal.md Q27's
    interlock cannot be evaluated from it either.

    MAV_CMD_SET_MESSAGE_INTERVAL raises it immediately — measured 26.7 Hz. But
    in SITL the rate falls back to ~2 Hz within about twenty seconds, because
    MAVProxy sits between ArduPilot and our router and periodically re-requests
    its own, much lower, default stream rates on the same channel.

    ardupilot_gz's robot.launch.py includes sitl_mavproxy.launch.py
    unconditionally, so MAVProxy cannot simply be switched off without
    reworking the upstream vehicle-spawn path.

    This node therefore re-asserts the intervals whenever the observed rate
    falls below target. It is a SIMULATION-TOPOLOGY workaround, not a flight
    feature: the deployed architecture (goal.md Q22/Q28) has the Pi's
    mav_router talking straight to the Pixhawk over TELEM2 with no MAVProxy
    anywhere, so on real hardware the first application simply sticks and this
    node never fires again.

    The clean long-term fix is to drop MAVProxy from the simulation and let
    mav_router own the SITL link, matching deployment. Tracked for Phase 11.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong

MAV_CMD_SET_MESSAGE_INTERVAL = 511

# message id -> Hz. Mirrors scripts/set_stream_rates.py; keep them in step.
DESIRED = {
    30: 50.0,    # ATTITUDE
    32: 50.0,    # LOCAL_POSITION_NED
    33: 10.0,    # GLOBAL_POSITION_INT
    24: 5.0,     # GPS_RAW_INT
    74: 10.0,    # VFR_HUD
    1: 4.0,      # SYS_STATUS
    2: 4.0,      # SYSTEM_TIME
    27: 10.0,    # RAW_IMU
    65: 5.0,     # RC_CHANNELS
}


class StreamRateKeeper(Node):
    def __init__(self):
        super().__init__("stream_rate_keeper")
        p = self.declare_parameter
        p("min_pose_rate_hz", 10.0)     # re-assert below this
        p("check_period_s", 5.0)
        p("reapply_cooldown_s", 8.0)

        self.state = State()
        self._pose_count = 0
        self._last_apply_t = None
        self._applied_once = False
        self._pending = []

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.cli = self.create_client(CommandLong, "/mavros/cmd/command")

        period = float(self.get_parameter("check_period_s").value)
        self.create_timer(period, self._check)
        self.get_logger().info("stream_rate_keeper up")

    def _on_pose(self, _):
        self._pose_count += 1

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _apply(self):
        """Issue the interval commands, RESOLVING each future.

        The first version fired nine call_async() and dropped every future on
        the floor, every few seconds, forever. rclpy keeps an entry per
        outstanding request, so that leaks without bound. Futures are now
        cancelled once they are no longer wanted, and the batch is issued at a
        modest pace rather than nine-at-once.
        """
        if not self.cli.service_is_ready():
            return False
        for msg_id, hz in DESIRED.items():
            req = CommandLong.Request()
            req.command = MAV_CMD_SET_MESSAGE_INTERVAL
            req.param1 = float(msg_id)
            req.param2 = float(1_000_000.0 / hz)
            future = self.cli.call_async(req)
            future.add_done_callback(lambda f: None)
            self._pending.append(future)

        # Drop resolved futures, and stop waiting on ones that never came back
        # so the client's request table cannot grow indefinitely.
        still = []
        for f in self._pending:
            if f.done():
                continue
            if len(self._pending) > 3 * len(DESIRED):
                self.cli.remove_pending_request(f)
                continue
            still.append(f)
        self._pending = still

        self._last_apply_t = self._now()
        return True

    def _check(self):
        period = float(self.get_parameter("check_period_s").value)
        rate = self._pose_count / period
        self._pose_count = 0

        if not self.state.connected:
            return

        target = float(self.get_parameter("min_pose_rate_hz").value)
        if self._applied_once and rate >= target:
            return

        cooldown = float(self.get_parameter("reapply_cooldown_s").value)
        if self._last_apply_t is not None and \
                (self._now() - self._last_apply_t) < cooldown:
            return

        if self._apply():
            if not self._applied_once:
                self.get_logger().info("stream rates applied")
                self._applied_once = True
            else:
                self.get_logger().warning(
                    f"pose rate {rate:.1f} Hz below {target:.0f} Hz — "
                    f"stream rates re-asserted (MAVProxy contention)")


def main():
    rclpy.init()
    node = StreamRateKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
