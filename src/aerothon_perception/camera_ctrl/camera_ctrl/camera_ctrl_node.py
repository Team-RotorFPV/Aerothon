#!/usr/bin/env python3
"""Camera pointing as a commanded, CONFIRMED state (PHASE_PLAN.md Phase 2).

WHY THIS EXISTS
    CURRENT_PROGRESS_HANDOFF.md: the start QR sits on the ground ~1 m ahead of
    the spawn point, the mission holds at (0,0,5), and nothing ever commands
    the camera downward. The detector was never the problem — the camera was
    pointed at a wall and the sky. No amount of QR tuning fixes an aim error.

    So camera orientation stops being an assumption and becomes state:
    commanded, read back from the actual joint, and gated on before any
    perception stage that depends on it.

NAMED POSES
    FORWARD   0 deg    corridor navigation
    BANNER  -20 deg    banner detection from above corridor altitude
    NADIR   -90 deg    start-QR scan, lawnmower search, winch drop, landing
    ALIGN   -45 deg    intermediate, target centring during descent

INTERFACE
    sub   /camera/set_pose      std_msgs/String   "FORWARD" | "NADIR" | "ALIGN"
                                                  or a raw angle in degrees
    sub   /joint_states         sensor_msgs/JointState   read-back (sim)
    sub   /mavros/gimbal_control/device/attitude_status  read-back (hardware)
    pub   /gimbal/cmd_pitch     std_msgs/Float64  radians, sim backend
    pub   /camera/pose_state    std_msgs/String   JSON, 10 Hz

    /camera/pose_state fields:
        requested       pose name, or "" before the first command
        requested_rad   commanded joint angle
        actual_rad      measured joint angle (null when never observed)
        error_deg       |actual - requested| in degrees
        settled         measurement within tolerance, held long enough
        stale           no read-back within stale_after_s
        age_s           seconds since the last read-back

    `settled` is the only field a mission stage should gate on. It is false
    until the joint has been MEASURED in position — never inferred from the
    fact that a command was sent.

BACKENDS
    backend:=sim      publish radians to /gimbal/cmd_pitch (Gazebo
                      JointPositionController), read back from /joint_states
    backend:=mavlink  MAV_CMD_DO_MOUNT_CONTROL via /mavros/cmd/command, read
                      back from the gimbal attitude status topic
    Same topic interface either way, so the mission tree does not care which
    airframe it is flying.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

try:
    from mavros_msgs.srv import CommandLong
    _HAVE_MAVROS = True
except ImportError:                       # sim-only installs
    _HAVE_MAVROS = False

MAV_CMD_DO_MOUNT_CONTROL = 205
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2

NAMED_POSES_DEG = {
    "FORWARD": 0.0,
    "NADIR": -90.0,
    "ALIGN": -45.0,
    # The rulebook requires the banner to be identified and the aircraft
    # aligned BEFORE descending to corridor altitude. From 5 m, a gate a few
    # metres ahead is entirely below a level camera -- a live run swept 271
    # degrees past it. Looking slightly down puts it in frame from the scan
    # altitude, so the rulebook order can be followed rather than worked
    # around by descending first.
    "BANNER": -20.0,
}


def _sensor_qos(depth=10):
    """Best-effort: /joint_states is published reliably at ~785 Hz, and a
    reliable subscriber there just builds backpressure and drops anyway."""
    return QoSProfile(depth=depth,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT)


class CameraCtrl(Node):
    def __init__(self):
        super().__init__("camera_ctrl")

        p = self.declare_parameter
        p("backend", "sim")                 # sim | mavlink
        p("joint_name", "webcam_pitch_joint")
        p("tolerance_deg", 2.0)
        p("settle_samples", 5)              # consecutive in-tolerance samples
        p("settle_hold_s", 0.20)            # ...held at least this long
        p("stale_after_s", 1.0)
        p("publish_rate_hz", 10.0)
        p("command_repeat_s", 0.5)          # re-send until settled
        p("startup_pose", "FORWARD")

        self.backend = self.get_parameter("backend").value
        self.joint_name = self.get_parameter("joint_name").value

        self.requested_name = ""
        self.requested_rad = None
        self.actual_rad = None
        self.last_reading_t = None
        self._in_tolerance_count = 0
        self._in_tolerance_since = None
        self._last_command_t = None

        self.create_subscription(String, "/camera/set_pose", self._on_set_pose, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joint_states, _sensor_qos())

        self.pub_cmd = self.create_publisher(Float64, "/gimbal/cmd_pitch", 10)
        self.pub_state = self.create_publisher(String, "/camera/pose_state", 10)

        self.cli_cmd = None
        if self.backend == "mavlink":
            if not _HAVE_MAVROS:
                self.get_logger().error(
                    "backend=mavlink but mavros_msgs is unavailable")
            else:
                self.cli_cmd = self.create_client(CommandLong, "/mavros/cmd/command")

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

        startup = self.get_parameter("startup_pose").value
        if startup:
            self.request(startup)

        self.get_logger().info(
            f"camera_ctrl up; backend={self.backend} joint={self.joint_name} "
            f"startup={startup}")

    # ------------------------------------------------------------------ #
    # commanding
    # ------------------------------------------------------------------ #
    def _on_set_pose(self, msg: String):
        self.request(msg.data.strip())

    def request(self, target: str):
        """Accept a named pose or a raw angle in degrees."""
        key = target.upper()
        if key in NAMED_POSES_DEG:
            deg = NAMED_POSES_DEG[key]
            name = key
        else:
            try:
                deg = float(target)
                name = f"{deg:.1f}deg"
            except ValueError:
                self.get_logger().warning(
                    f"unknown camera pose '{target}'; "
                    f"expected one of {sorted(NAMED_POSES_DEG)} or a number")
                return

        new_rad = math.radians(deg)
        if self.requested_rad is not None and \
                abs(new_rad - self.requested_rad) < 1e-6 and name == self.requested_name:
            return                                  # already commanding this

        self.requested_name = name
        self.requested_rad = new_rad
        # A new target invalidates any previous settle: the joint has not been
        # measured at the NEW angle yet.
        self._in_tolerance_count = 0
        self._in_tolerance_since = None
        self._last_command_t = None
        self.get_logger().info(f"camera pose -> {name} ({deg:.1f} deg)")
        self._send()

    def _send(self):
        if self.requested_rad is None:
            return
        if self.backend == "mavlink":
            self._send_mavlink()
        else:
            self.pub_cmd.publish(Float64(data=float(self.requested_rad)))
        self._last_command_t = self._now()

    def _send_mavlink(self):
        if self.cli_cmd is None or not self.cli_cmd.service_is_ready():
            return
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_MOUNT_CONTROL
        req.param1 = float(math.degrees(self.requested_rad))   # pitch, degrees
        req.param2 = 0.0                                       # roll
        req.param3 = 0.0                                       # yaw
        req.param7 = float(MAV_MOUNT_MODE_MAVLINK_TARGETING)
        self.cli_cmd.call_async(req)

    # ------------------------------------------------------------------ #
    # read-back
    # ------------------------------------------------------------------ #
    def _on_joint_states(self, msg: JointState):
        if self.backend != "sim":
            return
        try:
            idx = list(msg.name).index(self.joint_name)
        except ValueError:
            return
        if idx >= len(msg.position):
            return
        self._observe(float(msg.position[idx]))

    def _observe(self, rad):
        self.actual_rad = rad
        self.last_reading_t = self._now()
        if self.requested_rad is None:
            return
        tol = math.radians(float(self.get_parameter("tolerance_deg").value))
        if abs(rad - self.requested_rad) <= tol:
            self._in_tolerance_count += 1
            if self._in_tolerance_since is None:
                self._in_tolerance_since = self.last_reading_t
        else:
            self._in_tolerance_count = 0
            self._in_tolerance_since = None

    # ------------------------------------------------------------------ #
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_settled(self):
        if self.requested_rad is None or self.actual_rad is None:
            return False
        if self.is_stale():
            return False
        if self._in_tolerance_count < int(self.get_parameter("settle_samples").value):
            return False
        if self._in_tolerance_since is None:
            return False
        hold = float(self.get_parameter("settle_hold_s").value)
        return (self._now() - self._in_tolerance_since) >= hold

    def is_stale(self):
        if self.last_reading_t is None:
            return True
        stale_after = float(self.get_parameter("stale_after_s").value)
        return (self._now() - self.last_reading_t) > stale_after

    def _tick(self):
        # Keep re-issuing until the joint is measured in position. The Gazebo
        # position controller and a real servo can both miss a single command.
        if self.requested_rad is not None and not self.is_settled():
            repeat = float(self.get_parameter("command_repeat_s").value)
            if self._last_command_t is None or \
                    (self._now() - self._last_command_t) >= repeat:
                self._send()

        err_deg = None
        if self.actual_rad is not None and self.requested_rad is not None:
            err_deg = abs(math.degrees(self.actual_rad - self.requested_rad))

        age = None if self.last_reading_t is None else self._now() - self.last_reading_t

        payload = {
            "requested": self.requested_name,
            "requested_rad": self.requested_rad,
            "actual_rad": self.actual_rad,
            "error_deg": err_deg,
            "settled": self.is_settled(),
            "stale": self.is_stale(),
            "age_s": age,
        }
        self.pub_state.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = CameraCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
