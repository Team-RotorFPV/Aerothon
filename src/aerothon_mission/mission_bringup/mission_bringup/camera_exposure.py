"""Hold the flight camera at its configured exposure: auto by default.

usb_cam 0.8 sets exposure by running `v4l2-ctl -c exposure_auto=1` and
`-c exposure=<n>`. Current UVC drivers name those controls `auto_exposure`
and `exposure_time_absolute`, so v4l2-ctl answers "unknown control" and the
camera stays on auto exposure with a dynamic frame rate: the motion blur the
manual setting exists to prevent (measured on the fitted Brio 100,
16 Oct 2026). This node sets whichever names the device has, and re-checks
every `period_s` so a camera respawn or replug is corrected too.

    exposure   <= 0: auto exposure (the default; the camera sets time and
               gain). > 0: manual, in 100 us units as usb_cam's `exposure`
               (100 = 10 ms). Manual exposure also fixes gain, so a value
               tuned for daylight is far too dark indoors (16 Oct: 10 ms on
               the Brio indoors gave a near-black feed). Tune it at the venue.
               Read every cycle: `ros2 param set /camera_exposure exposure N`.
"""
import re
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# (auto-exposure control, manual value, auto value, exposure-time control), newest first.
# The auto value is UVC "aperture priority": the camera picks time and gain.
NAME_SETS = (("auto_exposure", 1, 3, "exposure_time_absolute"),
             ("exposure_auto", 1, 3, "exposure_absolute"))
CONTROL = re.compile(r"^\s*(\w+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:.*?\bvalue=(-?\d+)", re.M)


def controls(listing):
    """{name: current value} from `v4l2-ctl -l` output."""
    return {m.group(1): int(m.group(3)) for m in CONTROL.finditer(listing)}


def wanted(present, exposure):
    """{control: value} for `exposure` (<= 0 auto, > 0 manual), or None when
    the device has no exposure controls under any known name."""
    for auto, manual, automatic, time_name in NAME_SETS:
        if auto in present and time_name in present:
            if int(exposure) <= 0:
                return {auto: automatic}
            want = {auto: manual, time_name: int(exposure)}
            if "exposure_dynamic_framerate" in present:
                want["exposure_dynamic_framerate"] = 0
            return want
    return None


class CameraExposure(Node):
    def __init__(self):
        super().__init__("camera_exposure")
        self.device = self.declare_parameter("device", "/dev/video0").value
        self.declare_parameter("exposure", 0)
        period = float(self.declare_parameter("period_s", 5.0).value)
        self._reported = None
        self.create_timer(period, self.apply)
        self.apply()

    def _run(self, *args):
        return subprocess.run(["v4l2-ctl", "-d", self.device, *args],
                              capture_output=True, text=True, timeout=5)

    def apply(self):
        self.exposure = int(self.get_parameter("exposure").value)
        try:
            present = controls(self._run("-l").stdout)
        except (OSError, subprocess.SubprocessError) as exc:
            self._report(f"cannot query {self.device}: {exc}")
            return
        if not present:
            self._report(f"{self.device} not available")
            return
        want = wanted(present, self.exposure)
        if want is None:
            self._report(f"{self.device} has no known exposure controls")
            return
        change = {k: v for k, v in want.items() if present.get(k) != v}
        if not change:
            mode = "auto exposure" if self.exposure <= 0 else f"manual exposure {self.exposure}"
            self._report(f"{mode} held on {self.device}")
            return
        # Manual mode first: the time control is inactive while auto is on.
        for key in sorted(change, key=lambda k: k not in ("auto_exposure", "exposure_auto")):
            result = self._run("-c", f"{key}={change[key]}")
            if result.returncode != 0:
                self._report(f"setting {key}={change[key]} failed: {result.stderr.strip()}")
                return
        self.get_logger().info(f"camera exposure set: {change}")

    def _report(self, text):
        if text != self._reported:
            self.get_logger().info(text)
            self._reported = text


def main():
    rclpy.init()
    node = CameraExposure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
