#!/usr/bin/env python3
"""The day's conditions, applied between the simulator and the stack.

Sim-only. The Gazebo bridge publishes the rendered camera and the ray-cast
lidar on /camera/image_gz and /scan_gz; this node republishes them on the
topics the stack reads, as the real sensors would deliver them in the
world's conditions (scripts/world_spec.py, resolved by materialize_world.py
into `conditions_file`):

    camera   corruptions.CameraCorruptor on every frame; `frame_drop` of the
             frames lost; each delivered `latency_ms` after capture, stamp
             unchanged -- the USB/MJPEG pipeline delays, it does not re-time
    lidar    corruptions.LidarCorruptor on every scan
    fcu      a GPS glitch of `gps_glitch_m`, `glitch_at_s` after arming, for
             `glitch_s`: SIM_GPS_GLITCH_X/Y set through MAVROS, then cleared.
             The other SITL faults are boot parameters (materialize_world.py).

With calm conditions every message is passed straight through.
"""

import json
import math
from collections import deque

import rclpy
from cv_bridge import CvBridge
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from sim_gazebo.corruptions import CAMERA_KEYS, CameraCorruptor, LidarCorruptor


class DegradeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sim_degrade', **kwargs)
        self.declare_parameter('conditions_file', '')
        self.declare_parameter('seed', 0)
        path = self.get_parameter('conditions_file').value
        seed = int(self.get_parameter('seed').value)
        cond = {}
        if path:
            with open(path, encoding='utf-8') as f:
                cond = json.load(f)
        cam = cond.get('camera', {})
        self.camera = CameraCorruptor({k: cam.get(k, 0) for k in CAMERA_KEYS}, seed)
        self.frame_drop = float(cam.get('frame_drop', 0.0))
        self.latency_s = float(cam.get('latency_ms', 0)) / 1000.0
        self.lidar = LidarCorruptor(cond.get('lidar', {}), seed + 1)
        self.fcu = cond.get('fcu', {})
        self.rng = self.camera.rng
        self.bridge = CvBridge()
        self.pending = deque()            # (release_time_s, Image)

        self.pub_image = self.create_publisher(Image, '/camera/image', 5)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_gz', self.on_image, 5)
        self.create_subscription(LaserScan, '/scan_gz', self.on_scan,
                                 qos_profile_sensor_data)
        if self.latency_s > 0.0:
            self.create_timer(0.01, self.release)

        self.armed_at = None
        self.glitch = float(self.fcu.get('gps_glitch_m', 0.0))
        self.glitch_phase = 0              # 0 waiting, 1 applied, 2 cleared
        if self.glitch > 0.0:
            self.param_set = self.create_client(ParamSetV2, '/mavros/param/set')
            self.create_subscription(State, '/mavros/state', self.on_state, 10)
            self.create_timer(0.5, self.glitch_tick)
        self.get_logger().info(
            f"conditions: camera {self.camera.sev} drop {self.frame_drop:.2f} "
            f"latency {self.latency_s * 1000:.0f} ms; lidar noise "
            f"{self.lidar.noise_m} dropout {self.lidar.dropout} spurious "
            f"{self.lidar.spurious}; GPS glitch {self.glitch} m")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- camera ----
    def on_image(self, msg):
        if self.frame_drop and self.rng.random() < self.frame_drop:
            return
        if self.camera.active:
            frame = self.camera.apply(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
            out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            out.header = msg.header
            msg = out
        if self.latency_s > 0.0:
            self.pending.append((self._now() + self.latency_s, msg))
        else:
            self.pub_image.publish(msg)

    def release(self):
        now = self._now()
        while self.pending and self.pending[0][0] <= now:
            self.pub_image.publish(self.pending.popleft()[1])

    # ---- lidar ----
    def on_scan(self, msg):
        if self.lidar.active:
            msg.ranges = self.lidar.apply(msg.ranges, msg.range_min,
                                          msg.range_max).tolist()
        self.pub_scan.publish(msg)

    # ---- GPS glitch ----
    def on_state(self, m):
        if m.armed and self.armed_at is None:
            self.armed_at = self._now()

    def glitch_tick(self):
        if self.armed_at is None or self.glitch_phase == 2:
            return
        t = self._now() - self.armed_at
        start = float(self.fcu.get('glitch_at_s', 150.0))
        if self.glitch_phase == 0 and t >= start:
            a = self.rng.uniform(0.0, 2.0 * math.pi)
            if self._set_glitch(self.glitch * math.cos(a), self.glitch * math.sin(a)):
                self.glitch_phase = 1
                self.get_logger().warn(f"GPS glitch {self.glitch:.1f} m injected "
                                       f"{t:.0f} s after arming")
        elif self.glitch_phase == 1 and t >= start + float(self.fcu.get('glitch_s', 5.0)):
            if self._set_glitch(0.0, 0.0):
                self.glitch_phase = 2
                self.get_logger().warn("GPS glitch cleared")

    def _set_glitch(self, x, y):
        if not self.param_set.service_is_ready():
            return False
        for name, value in (('SIM_GPS_GLITCH_X', x), ('SIM_GPS_GLITCH_Y', y)):
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = name
            req.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                       double_value=float(value))
            self.param_set.call_async(req)
        return True


def main():
    rclpy.init()
    node = DegradeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
