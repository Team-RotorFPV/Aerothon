"""Laptop-only bridge: Gazebo sensors and actual SITL servo outputs."""
import argparse
import asyncio
from collections import deque
import json
import math
import socket
import threading
import time
import uuid

from aiohttp import WSMsgType, web
from .device_wire import pack

# Seconds one sensor frame may take to leave. Streams carry only the newest
# sample, so a slow LAN costs skipped frames, not stale ones; a 0.5 s limit
# closed healthy streams on a congested link to the Pi. A dead peer is
# caught by the 3 s heartbeat.
SEND_TIMEOUT_S = 5.0


class Gateway:
    def __init__(self, node, args):
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Vector3Stamped
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo, LaserScan, JointState
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float64
        self.node, self.args = node, args
        self.session = uuid.uuid4().hex
        self.bridge = CvBridge()
        self.latest = {}
        self.seq = {}
        self.samples = {}
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', args.fcu_udp))
        self.udp.setblocking(False)
        self.fcu = None
        self.heartbeat = None
        self.identity = None
        self.clients = set()
        self.parser = mavlink.MAVLink(None)
        self.encoder = mavlink.MAVLink(None, srcSystem=253, srcComponent=191)
        self.parameters = {}
        self.param_requested = 0.0
        self.parser.robust_parsing = True
        self.mount = node.create_publisher(Float64, '/gimbal/cmd_pitch', 10)
        self.winch = node.create_publisher(Float64, '/winch/gz/payout', 10)
        self.payout = 0.0
        self.last_pwm = None
        self.measured_camera = None
        self.measured_winch = None
        self.actuator_seen = None
        self.actuator_clock = None
        self.mount_feedback = node.create_publisher(Vector3Stamped, '/device/camera/orientation', 10)
        node.create_subscription(Image, '/camera/image', self.camera, qos_profile_sensor_data)
        for kind, topic, typ in [('camera_info', '/camera/camera_info', CameraInfo),
                                 ('scan', '/scan', LaserScan), ('clock', '/clock', Clock)]:
            node.create_subscription(typ, topic, lambda m, k=kind: self.record(k, m), qos_profile_sensor_data)
        node.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        # Gazebo joint names are confined to this laptop adapter.
        node.create_timer(.02, self.actuate)

    def record(self, kind, msg):
        from rclpy.serialization import serialize_message
        stamp = time.monotonic()
        self.latest[kind] = serialize_message(msg)
        self.seq[kind] = self.seq.get(kind, 0) + 1
        samples = self.samples.setdefault(kind, deque(maxlen=100))
        samples.append(stamp)

    def camera(self, msg):
        import cv2
        from sensor_msgs.msg import CompressedImage
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        ok, body = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if ok:
            self.record('camera', CompressedImage(header=msg.header, format='jpeg', data=body.tobytes()))

    def joints(self, msg):
        from sensor_msgs.msg import JointState
        wire = JointState(header=msg.header)
        for name, value in zip(msg.name, msg.position):
            if name == self.args.camera_joint:
                self.measured_camera = (float(value), time.monotonic())
                wire.name.append('camera_pitch')
                wire.position.append(float(value))
            if name == self.args.winch_joint:
                self.measured_winch = (float(value), time.monotonic())
                wire.name.append('winch_payout')
                wire.position.append(float(value))
        if wire.name:
            self.record('joints', wire)

    def metric(self, kind):
        q = self.samples.get(kind, [])
        age = time.monotonic() - q[-1] if q else None
        hz = (len(q)-1)/(q[-1]-q[0]) if len(q)>1 and q[-1]>q[0] else 0
        return {'ok': age is not None and age < 1, 'receipt_age_s': age, 'hz': round(hz, 2)}

    def manifest(self):
        now = time.monotonic()
        return {'v': 1, 'session': self.session, 'system_id': self.identity, 'component_id': 1,
                'fcu_ok': self.heartbeat is not None and now-self.heartbeat < 3,
                'topics': {k: self.metric(k) for k in ('camera','camera_info','scan','clock','joints')},
                'camera_actuator': {'ok': bool(self.parameters.get(f'SERVO{self.args.camera_channel}_FUNCTION') == 7 and self.parameters.get('MNT1_TYPE',0) > 0 and self.actuator_seen and now-self.actuator_seen < 1 and self.measured_camera and now-self.measured_camera[1] < 1),
                                    'reason': 'SITL output and measured model joint required'},
                'winch_actuator': {'ok': bool(self.parameters.get(f'SERVO{self.args.winch_channel}_FUNCTION') == 88 and self.parameters.get('WINCH_TYPE',0) > 0 and self.actuator_seen and now-self.actuator_seen < 1 and self.measured_winch and now-self.measured_winch[1] < 1),
                                   'reason': 'SITL output and measured model joint required'}}

    def actuate(self):
        # Convert configured hardware PWM channels only. Do not infer commands
        # from mission stage, command ACK or local winch payout estimates.
        now = self.node.get_clock().now().nanoseconds/1e9
        dt = 0.0 if self.actuator_clock is None else max(0.0, now-self.actuator_clock)
        self.actuator_clock = now
        if self.last_pwm is None or self.actuator_seen is None or time.monotonic()-self.actuator_seen > 1:
            return
        from std_msgs.msg import Float64
        if self.fcu and time.monotonic()-self.param_requested > 5:
            self.param_requested = time.monotonic()
            for name in ('MNT1_TYPE','WINCH_TYPE',f'SERVO{self.args.camera_channel}_FUNCTION',f'SERVO{self.args.winch_channel}_FUNCTION'):
                msg = self.encoder.param_request_read_encode(self.args.system_id,1,name.encode(),-1)
                self.udp.sendto(msg.pack(self.encoder),self.fcu)
        pwm = self.last_pwm
        cam = pwm[self.args.camera_channel-1]
        if 800 <= cam <= 2200:
            frac = max(0.0, min(1.0, (cam-self.args.camera_pwm_min)/(self.args.camera_pwm_max-self.args.camera_pwm_min)))
            pitch = math.radians(self.args.camera_min_deg + frac*(self.args.camera_max_deg-self.args.camera_min_deg))
            self.mount.publish(Float64(data=pitch))
        winch = pwm[self.args.winch_channel-1]
        if 800 <= winch <= 2200:
            rate = (winch-self.args.winch_neutral)/500*self.args.winch_rate
            self.payout = max(0.0, min(self.args.winch_length, self.payout+rate*dt))
            self.winch.publish(Float64(data=self.payout))

    async def udp_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            data, addr = await loop.sock_recvfrom(self.udp, 65536)
            # The configured local SITL channel owns the session; no fanout to
            # a second discovered FCU and no forwarding simulator truth.
            if self.fcu is not None and addr != self.fcu:
                continue
            for msg in self.parser.parse_buffer(data) or []:
                if msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                    if msg.get_srcSystem() != self.args.system_id:
                        continue
                    self.fcu, self.identity, self.heartbeat = addr, msg.get_srcSystem(), time.monotonic()
                if self.fcu == addr and msg.get_srcSystem() == self.args.system_id and msg.get_type() == 'PARAM_VALUE':
                    name = msg.param_id.decode(errors='replace') if isinstance(msg.param_id,bytes) else msg.param_id
                    self.parameters[name.rstrip('\x00')] = msg.param_value
                if self.fcu == addr and msg.get_srcSystem() == self.args.system_id and msg.get_type() == 'SERVO_OUTPUT_RAW':
                    self.last_pwm = [getattr(msg, f'servo{i}_raw', 0) for i in range(1, 17)]
                    self.actuator_seen = time.monotonic()
            if self.fcu != addr:
                continue
            for writer in list(self.clients):
                try:
                    writer.write(data)
                    await asyncio.wait_for(writer.drain(), .05)
                except (OSError, TimeoutError):
                    self.clients.discard(writer)
                    writer.close()

    async def tcp(self, reader, writer):
        # One Pi command owner. A second connection cannot steer the FCU.
        if self.clients:
            writer.close()
            return
        self.clients.add(writer)
        try:
            while data := await reader.read(65536):
                if self.fcu and self.heartbeat and time.monotonic()-self.heartbeat < 3:
                    self.udp.sendto(data, self.fcu)
        finally:
            self.clients.discard(writer)
            writer.close()
            await writer.wait_closed()

    async def stream(self, request):
        if request.query.get('session') != self.session:
            raise web.HTTPConflict(text='gateway session changed; reconnect explicitly')
        ws = web.WebSocketResponse(max_msg_size=1024, heartbeat=3)
        await ws.prepare(request)
        kinds = ['camera'] if request.path == '/camera' else ['scan','clock','camera_info','joints']
        sent = {}
        last_sent = {}
        async def receive():
            # aiohttp handles ping/pong while receive is being polled. A
            # send-only sensor loop left incoming pings unread and closed
            # otherwise healthy streams after the heartbeat timeout.
            async for message in ws:
                if message.type in {WSMsgType.TEXT, WSMsgType.BINARY}:
                    await ws.close(code=1008, message=b'device channel is observation only')
                    break
        receiver = asyncio.create_task(receive())
        try:
            while not ws.closed:
                for kind in kinds:
                    interval = {'clock': .02, 'joints': .05, 'camera_info': .1}.get(kind, 0)
                    if time.monotonic()-last_sent.get(kind,0) < interval:
                        continue
                    if self.seq.get(kind) != sent.get(kind) and kind in self.latest:
                        await asyncio.wait_for(ws.send_bytes(pack(kind, self.session, self.latest[kind])), SEND_TIMEOUT_S)
                        sent[kind] = self.seq[kind]
                        last_sent[kind] = time.monotonic()
                await asyncio.sleep(.005)
        except (TimeoutError, ConnectionError):
            await ws.close()
        finally:
            receiver.cancel()
            await asyncio.gather(receiver, return_exceptions=True)
        return ws

    async def run(self):
        app = web.Application()
        async def manifest(request):
            return web.json_response(self.manifest())
        app.router.add_get('/manifest', manifest)
        app.router.add_get('/sensors', self.stream)
        app.router.add_get('/camera', self.stream)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, self.args.bind, self.args.port).start()
        server = await asyncio.start_server(self.tcp, self.args.bind, self.args.mavlink_port)
        self.node.get_logger().info(f'Gateway ready; MAVLink TCP {self.args.mavlink_port}, sensors HTTP {self.args.port}')
        try:
            await self.udp_loop()
        finally:
            server.close()
            await server.wait_closed()
            await runner.cleanup()
            self.udp.close()


def main():
    import rclpy
    p = argparse.ArgumentParser()
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--mavlink-port', type=int, default=5762)
    p.add_argument('--fcu-udp', type=int, default=14560)
    p.add_argument('--system-id', type=int, default=1)
    p.add_argument('--camera-channel', type=int, required=True)
    p.add_argument('--winch-channel', type=int, required=True)
    p.add_argument('--camera-pwm-min', type=int, default=1000)
    p.add_argument('--camera-pwm-max', type=int, default=2000)
    p.add_argument('--camera-min-deg', type=float, default=-90)
    p.add_argument('--camera-max-deg', type=float, default=30)
    p.add_argument('--winch-neutral', type=int, default=1500)
    p.add_argument('--winch-rate', type=float, required=True)
    p.add_argument('--winch-length', type=float, default=6)
    p.add_argument('--camera-joint', default='webcam_pitch_joint')
    p.add_argument('--winch-joint', default='winch_joint')
    args = p.parse_args()
    if not 1 <= args.camera_channel <= 16 or not 1 <= args.winch_channel <= 16 or args.camera_channel == args.winch_channel:
        p.error('configure distinct camera and winch output channels between 1 and 16')
    if args.camera_pwm_min >= args.camera_pwm_max or args.winch_rate <= 0:
        p.error('invalid actuator calibration')
    rclpy.init(args=[])
    node = rclpy.create_node('laptop_gateway', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    gw = Gateway(node, args)
    def spin():
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        asyncio.run(gw.run())
    except KeyboardInterrupt:
        pass
    finally:
        gw.udp.close()
        rclpy.try_shutdown()
        thread.join(2)
        node.destroy_node()


if __name__ == '__main__':
    main()
