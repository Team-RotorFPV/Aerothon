#!/usr/bin/env python3
"""Winch state machine with feedback and gated payload release.

WHY THIS EXISTS
    /winch/cmd had two publishers (the behaviour tree and the GCS) and ZERO
    subscribers. Payload delivery — the thing Mission 2 actually scores — was
    a string published into the void, and WinchDrop "succeeded" after a fixed
    20-tick delay whether or not anything had moved.

WHAT IS AND IS NOT MODELLED
    This node is the real controller: the command interface, the state machine,
    the payout integration, the ground-contact trigger and the release
    interlocks are all genuine, and the same node runs against hardware with
    `backend:=mavlink` driving MAV_CMD_DO_WINCH.

    backend:=sim      payout integrated from the commanded rate, nothing moves.
                      Proves the SEQUENCE and the INTERLOCKS only.
    backend:=gazebo   the same integration, plus the payout is sent to the
                      Iris's winch joint (/winch/gz/payout, bridged) so a real
                      100 g payload goes down on the hook, and a release sends
                      /winch/gz/detach so it physically leaves it. Whether it
                      actually did is for the CAMERA to say (WinchDrop's
                      confirmation), not for this node's own flag.
    backend:=mavlink  MAV_CMD_DO_WINCH on the aircraft.

    Ground contact is still inferred from the aircraft's altitude in every
    backend; the line is a rigid vertical rod in Gazebo, with no swing.

INTERFACE
    sub  /winch/cmd      std_msgs/String   "lower" | "release" | "stow" | "stop"
    pub  /winch/status   std_msgs/String   JSON, 5 Hz

    status fields:
        state        IDLE | LOWERING | AT_GROUND | RELEASED | STOWING | FAULT
        payout_m     metres of line paid out
        at_limit     payout has reached max_payout_m
        ground       ground contact inferred
        released     payload has left the hook
        fault        reason string, empty when healthy
        release_ok   whether a release command would currently be accepted
        blockers     list of unmet release preconditions

RELEASE INTERLOCKS (all must hold)
    * payload is at the ground, or payout is at the commanded limit
    * aircraft altitude within [min_release_alt, max_release_alt]
    * hover stable: horizontal speed below a threshold for N samples
    * no fault
    Releasing a 100 g payload from the wrong height or while drifting is how
    you miss the delivery zone, so this refuses rather than guesses.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import Empty, Float64, String

try:
    from mavros_msgs.srv import CommandLong
    _HAVE_MAVROS = True
except ImportError:
    _HAVE_MAVROS = False

MAV_CMD_DO_WINCH = 42600
WINCH_RELAXED, WINCH_RELATIVE_LENGTH_CONTROL = 0, 1

VALID_COMMANDS = ("lower", "release", "stow", "stop")


class WinchNode(Node):
    def __init__(self):
        super().__init__("winch_ctrl")
        p = self.declare_parameter
        p("backend", "sim")                  # sim | gazebo | mavlink
        p("payout_rate_mps", 0.30)           # line speed
        p("max_payout_m", 6.0)               # spool capacity
        p("ground_clearance_m", 0.25)        # payload considered down within this
        p("min_release_alt_m", 1.0)
        p("max_release_alt_m", 8.0)
        p("hover_speed_max_mps", 0.35)
        p("hover_stable_samples", 5)
        p("publish_rate_hz", 5.0)
        p("stow_rate_mps", 0.45)
        # HOW THE PAYLOAD LEAVES THE HOOK.
        #   gravity  the team's mechanism: a motor lowers a gravity-assisted
        #            hook that lets go by itself once the payload rests on the
        #            ground and the line goes slack. "release" is bookkeeping;
        #            nothing is sent to open anything.
        #   command  an actuated release: "release" opens the hook.
        p("hook", "gravity")
        # Gazebo backend, gravity hook: the payload counts as resting when its
        # centre is within this of its resting height (half its 0.08 m).
        p("payload_rest_z_m", 0.06)
        # A gravity hook needs SLACK to let go: the payload on the ground and
        # the line still paying out. "Down" is therefore line out to the
        # altitude PLUS this -- not minus ground_clearance_m, which stopped the
        # payload 0.19 m up, hanging, and flew it home (first flight on the
        # team airframe). The payload touches ~0.16 m before the hook reaches
        # the ground, so this leaves about a quarter metre of slack.
        p("gravity_slack_m", 0.10)

        self.backend = self.get_parameter("backend").value

        self.state = "IDLE"
        self.payout = 0.0
        self.released = False
        self.fault = ""
        self._alt = None
        self._speed = None
        self._stable = 0
        self._last_tick = self._now()

        self.create_subscription(String, "/winch/cmd", self._on_cmd, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped,
                                 "/mavros/local_position/velocity_local",
                                 self._on_vel, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, "/winch/status", 10)

        self.cli_cmd = None
        if self.backend == "mavlink" and _HAVE_MAVROS:
            self.cli_cmd = self.create_client(CommandLong, "/mavros/cmd/command")
        self.pub_gz_payout = self.pub_gz_detach = None
        self._detach_sends = 0
        self._payload_z = None
        self._slack_hist = []        # (payout, payload z) per tick while lowering
        self.hook_open = False       # the payload has physically left the hook
        if self.backend == "gazebo":
            self._enable_gazebo()

        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value),
                          self._tick)
        self.get_logger().info(f"winch_ctrl up; backend={self.backend}")

    def _enable_gazebo(self):
        """The Iris winch joint's payout and the payload's detach (bridged)."""
        self.backend = "gazebo"
        self.pub_gz_payout = self.create_publisher(Float64, "/winch/gz/payout", 10)
        self.pub_gz_detach = self.create_publisher(Empty, "/winch/gz/detach", 10)
        # The simulator's stand-in for the hook's own mechanics: it can only
        # know the payload is resting from where the payload is.
        self.create_subscription(Pose, "/sim/payload_pose", self._on_payload_pose,
                                 qos_profile_sensor_data)

    def _on_payload_pose(self, m):
        self._payload_z = float(m.position.z)

    def _gravity_hook(self):
        """Gazebo only: let the payload go once the line goes slack.

        SLACK, not "on the ground": the first fix tested the payload's height
        against the bare ground, and the payload landed on the target pad,
        8 cm up, where that never fired. What a gravity-assisted hook
        responds to is the line paying out while the payload no longer
        follows it: whatever it rests on, pad or grass. The height test stays
        as a second way to see it.

        And it matters in Gazebo more than in the air: the simulated line is
        a rigid joint, so a hook still latched to a resting payload is driven
        down against it and shoves the aircraft UP -- 1.3 m, watched.
        """
        if self.hook_open or self._payload_z is None or self.payout < 0.3:
            self._slack_hist = []
            return
        self._slack_hist = (self._slack_hist + [(self.payout, self._payload_z)])[-4:]
        (p0, z0), (p1, z1) = self._slack_hist[0], self._slack_hist[-1]
        slack = (len(self._slack_hist) >= 3 and p1 - p0 >= 0.10
                 and z0 - z1 < 0.25 * (p1 - p0))
        resting = self._payload_z <= float(self.get_parameter("payload_rest_z_m").value)
        if not (slack or resting):
            return
        self.hook_open = True
        self._detach_sends = 5
        self.get_logger().info(
            f"gravity hook: {'line slack' if slack else 'payload on the ground'} "
            f"(payload z {self._payload_z:.2f} m, {self.payout:.2f} m of line "
            f"out) -- the hook lets go")

    # ------------------------------------------------------------------ #
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_pose(self, m):
        self._alt = m.pose.position.z

    def _on_vel(self, m):
        v = m.twist.linear
        self._speed = math.hypot(v.x, v.y)
        if self._speed <= float(self.get_parameter("hover_speed_max_mps").value):
            self._stable += 1
        else:
            self._stable = 0

    # ------------------------------------------------------------------ #
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in VALID_COMMANDS:
            self.get_logger().warning(
                f"ignoring unknown winch command '{msg.data}' "
                f"(expected one of {VALID_COMMANDS})")
            return

        if cmd == "stop":
            self.state = "IDLE" if not self.released else "RELEASED"
            return

        if cmd == "lower":
            if self.released:
                self._set_fault("cannot lower: payload already released")
                return
            self.state = "LOWERING"
            self._send_backend("lower")

        elif cmd == "release":
            if self.released:
                # Re-commanding release on a delivered payload is a redundant
                # instruction, not a malfunction. Raising a FAULT here left the
                # GCS showing an error for the whole return leg after a
                # delivery that had actually succeeded.
                self.get_logger().info("release ignored: payload already delivered")
                return
            ok, blockers = self.release_ok()
            if not ok:
                # Refuse, loudly. A silent no-op here is how the mission
                # believed it had delivered when it had not.
                self._set_fault("release refused: " + "; ".join(blockers))
                return
            self.released = True
            self.state = "RELEASED"
            self._send_backend("release")
            self.get_logger().info(
                f"payload RELEASED at alt={self._alt:.2f} m, "
                f"payout={self.payout:.2f} m")

        elif cmd == "stow":
            self.state = "STOWING"
            self._send_backend("stow")

    def _set_fault(self, reason):
        self.fault = reason
        self.state = "FAULT"
        self.get_logger().error(f"winch fault: {reason}")

    def _send_backend(self, action):
        if self.backend == "gazebo":
            if action == "release" and self.get_parameter("hook").value == "command":
                # A bridged Empty can be lost on a busy host; re-send for a
                # few ticks (detaching an already-detached joint is a no-op).
                self._detach_sends = 5
            return
        if self.backend != "mavlink" or self.cli_cmd is None:
            return
        if not self.cli_cmd.service_is_ready():
            return
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_WINCH
        req.param1 = 1.0                                   # instance
        if action == "lower":
            req.param2 = float(WINCH_RELATIVE_LENGTH_CONTROL)
            req.param3 = float(self.get_parameter("max_payout_m").value)
            req.param4 = float(self.get_parameter("payout_rate_mps").value)
        elif action == "stow":
            req.param2 = float(WINCH_RELATIVE_LENGTH_CONTROL)
            req.param3 = -float(self.get_parameter("max_payout_m").value)
            req.param4 = float(self.get_parameter("stow_rate_mps").value)
        else:
            req.param2 = float(WINCH_RELAXED)
        self.cli_cmd.call_async(req)

    # ------------------------------------------------------------------ #
    def at_ground(self):
        """Payload is down: line paid out to within a clearance of the alt,
        or -- for a gravity hook -- past it, far enough for slack."""
        if self._alt is None:
            return False
        if self.get_parameter("hook").value == "gravity":
            return self.payout >= self._alt + float(
                self.get_parameter("gravity_slack_m").value)
        clear = float(self.get_parameter("ground_clearance_m").value)
        return self.payout >= (self._alt - clear)

    def at_limit(self):
        return self.payout >= float(self.get_parameter("max_payout_m").value)

    def release_ok(self):
        """Return (ok, blockers). Every precondition is reported, not just the
        first, so the operator sees the whole picture at once.

        A delivered payload reports `released`, not a fault: the live run left
        the GCS showing `fault: release refused: already released` for the
        whole return leg, which reads as a malfunction when the delivery had
        in fact succeeded. Phase 10 is about the panel not lying; this is the
        same principle applied at the source.
        """
        if self.released:
            return False, ["payload already delivered"]
        blockers = list(self.physical_blockers())
        if self.fault:
            blockers.append(f"fault: {self.fault}")
        return (not blockers), blockers

    def physical_blockers(self):
        """Unmet PHYSICAL preconditions, ignoring any latched fault text.

        Kept separate because the fault-clearing check needs to ask "are the
        conditions met now?" without the answer being contaminated by the very
        refusal it is trying to clear — asking release_ok() there made the
        fault permanently self-justifying.
        """
        blockers = []
        if not (self.at_ground() or self.at_limit()):
            blockers.append(
                f"payload not down (payout {self.payout:.2f} m, "
                f"alt {self._alt if self._alt is None else round(self._alt, 2)})")
        if self._alt is None:
            blockers.append("no altitude")
        else:
            lo = float(self.get_parameter("min_release_alt_m").value)
            hi = float(self.get_parameter("max_release_alt_m").value)
            if not (lo <= self._alt <= hi):
                blockers.append(
                    f"altitude {self._alt:.2f} m outside [{lo}, {hi}]")
        need = int(self.get_parameter("hover_stable_samples").value)
        if self._stable < need:
            blockers.append(f"hover not stable ({self._stable}/{need})")
        return blockers

    # ------------------------------------------------------------------ #
    def _tick(self):
        now = self._now()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now
        self.integrate(dt)
        if self.pub_gz_payout is not None:
            if self.get_parameter("hook").value == "gravity":
                self._gravity_hook()
            self.pub_gz_payout.publish(Float64(data=float(self.payout)))
            if self._detach_sends > 0:
                self.pub_gz_detach.publish(Empty())
                self._detach_sends -= 1
        self.publish_status()

    def integrate(self, dt):
        """Advance the spool by dt seconds.

        Separated from _tick so the state machine can be driven deterministically
        by tests and by replay, instead of depending on how fast the caller
        happens to loop.
        """
        # A refusal is a transient condition, not a latched failure: once the
        # payload is down and the aircraft is stable, the earlier "not down"
        # refusal must stop being reported.
        if self.fault.startswith("release refused") and not self.released:
            if not self.physical_blockers():
                self.fault = ""
                if self.state == "FAULT":
                    self.state = "AT_GROUND" if self.at_ground() else "IDLE"

        if self.state == "LOWERING":
            self.payout = min(
                float(self.get_parameter("max_payout_m").value),
                self.payout + float(self.get_parameter("payout_rate_mps").value) * dt)
            if self.at_ground() or self.at_limit():
                self.state = "AT_GROUND"
        elif self.state == "STOWING":
            self.payout = max(
                0.0,
                self.payout - float(self.get_parameter("stow_rate_mps").value) * dt)
            if self.payout <= 1e-3:
                self.payout = 0.0
                self.state = "RELEASED" if self.released else "IDLE"

    def publish_status(self):
        ok, blockers = self.release_ok()
        payload = {
            "state": self.state,
            "payout_m": round(self.payout, 3),
            "at_limit": self.at_limit(),
            "ground": self.at_ground(),
            "released": self.released,
            "hook": self.get_parameter("hook").value,
            "hook_open": self.hook_open,
            "fault": self.fault,
            "release_ok": ok,
            "blockers": blockers,
        }
        self.pub_status.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = WinchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
