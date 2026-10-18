"""Pi-side remote device driver. It has no Gazebo dependency or world inputs."""
import asyncio
from collections import deque
import json
import threading
import time

from aiohttp import ClientError, ClientSession, ClientTimeout, WSMsgType
from .device_wire import TOPICS, unpack

# Seconds of steady time the simulated vehicle's clock may stand still.
CLOCK_STALL_S = 3.0
# A stream the network dropped is reopened on the SAME gateway session after
# this pause. A congested LAN to the laptop closed streams for good, freezing
# the vehicle clock until someone reconnected by hand.
STREAM_RETRY_S = 1.0
# Seconds the manifest may go unanswered before the link is a fault; identity
# reads not-ok as soon as the last answer is older than MANIFEST_FRESH_S.
MANIFEST_LOSS_S = 10.0
MANIFEST_FRESH_S = 1.0


class StreamDropped(ConnectionError):
    """The gateway or network closed a stream; not a protocol fault."""


class RemoteDevices:
    def __init__(self, node):
        from cv_bridge import CvBridge
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        self.node = node
        self.url = node.declare_parameter('gateway', '').value
        self.system_id = node.declare_parameter('system_id', 1).value
        self.component_id = node.declare_parameter('component_id', 1).value
        self.expected_hfov = node.declare_parameter('camera_hfov', .851919).value
        self.calibration = None
        self.bridge = CvBridge()
        self.types = {k: get_message(v[0]) for k,v in TOPICS.items()}
        local_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.pubs = {k: node.create_publisher(Image if k == 'camera' else self.types[k], v[1], local_qos)
                     for k,v in TOPICS.items()}
        self.status = node.create_publisher(String, '/devices/status', 10)
        self.samples = {}
        self.capture_samples = {}
        self.stamps = {}
        self.manifest = {}
        self.manifest_at = None
        self.reopened = {}
        self.session = None
        self.fault = 'not connected'
        self.camera_fault = ''
        self.clock = None
        self.clock_changed = None
        self.shutdown = threading.Event()

    def deliver(self, packet):
        from rclpy.serialization import deserialize_message
        kind, body = unpack(packet, self.session)
        msg = deserialize_message(body, self.types[kind])
        stamp = time.monotonic()
        if kind != 'camera':
            self.samples.setdefault(kind, deque(maxlen=100)).append(stamp)
        if kind == 'clock':
            value = msg.clock.sec + msg.clock.nanosec/1e9
            if self.clock is not None and value < self.clock:
                raise ValueError('vehicle clock moved backwards; explicit reconnect required')
            if value != self.clock:
                self.clock_changed = stamp
            self.clock = value
        elif kind == 'camera_info':
            import math
            self.calibration = {'width':msg.width,'height':msg.height,
                'hfov':2*math.atan(msg.width/(2*msg.k[0])) if msg.k[0]>0 else None}
            self.stamps[kind] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        elif hasattr(msg, 'header') and kind != 'camera':
            self.stamps[kind] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if kind in self.stamps:
            captures = self.capture_samples.setdefault(kind, deque(maxlen=100))
            if not captures or self.stamps[kind] > captures[-1]:
                captures.append(self.stamps[kind])
        if kind == 'camera':
            decoded = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            image = self.bridge.cv2_to_imgmsg(decoded, 'bgr8')
            image.header = msg.header  # never re-stamp an old capture as fresh
            msg = image
        self.pubs[kind].publish(msg)
        if kind == 'camera':
            self.samples.setdefault(kind, deque(maxlen=100)).append(time.monotonic())
            self.stamps[kind] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            captures = self.capture_samples.setdefault(kind, deque(maxlen=100))
            if not captures or self.stamps[kind] > captures[-1]:
                captures.append(self.stamps[kind])

    def metric(self, kind, min_hz):
        q = self.samples.get(kind, [])
        age = time.monotonic()-q[-1] if q else None
        hz = (len(q)-1)/(q[-1]-q[0]) if len(q)>1 and q[-1]>q[0] else 0
        capture_age = self.clock-self.stamps[kind] if self.clock is not None and kind in self.stamps else None
        captures = self.capture_samples.get(kind, [])
        vehicle_hz = (len(captures)-1)/(captures[-1]-captures[0]) if len(captures)>1 and captures[-1]>captures[0] else 0
        # Rates are measured in vehicle time separately from network delivery
        # freshness. Wall-clock Hz is reported without inventing sim speed.
        fault = self.fault or (self.camera_fault if kind == 'camera' else '')
        return {'ok': not fault and age is not None and age < 1 and
                      capture_age is not None and 0 <= capture_age < .3 and vehicle_hz >= min_hz,
                'hz': round(hz,2), 'receipt_age_s': age, 'capture_age_s': capture_age,
                'vehicle_hz': vehicle_hz,
                'required_vehicle_hz': min_hz, 'reason': fault or ''}

    def publish_status(self):
        from std_msgs.msg import String
        now = time.monotonic()
        healthy = not self.fault
        camera = self.metric('camera', 10)
        calibration_ok = self.calibration is not None and self.calibration.get('hfov') is not None and abs(self.calibration['hfov']-self.expected_hfov)<.02
        camera['ok'] = camera['ok'] and calibration_ok
        if not calibration_ok:
            camera['reason'] = 'camera calibration missing or HFOV differs from profile'
        lidar = self.metric('scan', 8)
        # Gazebo on a loaded laptop pauses for over a second at times (payload
        # release contacts); a stalled clock is a fault after CLOCK_STALL_S.
        clock = {'ok': healthy and self.clock_changed is not None and now-self.clock_changed < CLOCK_STALL_S,
                 'reason': self.fault or 'vehicle clock must be advancing'}
        fresh = self.manifest_at is not None and now-self.manifest_at < MANIFEST_FRESH_S
        data = {'identity': {'ok': healthy and fresh and self.manifest.get('system_id') == self.system_id and self.manifest.get('component_id') == self.component_id and self.manifest.get('fcu_ok') is True,
                             'value': self.manifest.get('system_id'), 'session': self.session,
                             'streams_reopened': dict(self.reopened)},
                'camera': camera, 'lidar': lidar, 'clock': clock,
                'camera_calibration': {'ok': calibration_ok, 'value': self.calibration}}
        for key in ('camera_actuator', 'winch_actuator'):
            data[key] = dict(self.manifest.get(key) or {'ok': False, 'reason': 'not measured'})
            data[key]['ok'] = healthy and fresh and data[key].get('ok') is True and self.metric('joints',1)['ok']
        self.status.publish(String(data=json.dumps(data)))

    async def stream(self, client, path):
        # poll() still refuses a changed session or vehicle; this only rides
        # out drops on the same one. The gap shows as stale receipts and, past
        # CLOCK_STALL_S, a clock fault -- never as fresh data.
        while not self.shutdown.is_set():
            try:
                await self.stream_once(client, path)
            except (ClientError, asyncio.TimeoutError, ConnectionError) as exc:
                self.reopened[path] = self.reopened.get(path, 0) + 1
                self.node.get_logger().warning(
                    f'{path} stream dropped ({exc}); reopening on session {self.session}')
            await asyncio.sleep(STREAM_RETRY_S)

    async def stream_once(self, client, path):
        async with client.ws_connect(self.url+path, params={'session': self.session},
                                     max_msg_size=9*1024*1024, heartbeat=2) as ws:
            inbox = asyncio.Queue(maxsize=1)
            async def decode_camera():
                while True:
                    packet = await inbox.get()
                    await asyncio.to_thread(self.deliver, packet)
            worker = asyncio.create_task(decode_camera()) if path == '/camera' else None
            try:
                async for msg in ws:
                    if worker and worker.done():
                        worker.result()
                    if msg.type == WSMsgType.BINARY:
                        if worker:
                            # Read control frames promptly and keep one newest
                            # capture while the decoder/ROS publisher is busy.
                            if inbox.full(): inbox.get_nowait()
                            inbox.put_nowait(msg.data)
                        else:
                            self.deliver(msg.data)
                raise StreamDropped(f'{path} stream closed ({ws.close_code}): {ws.exception()}')
            finally:
                if worker:
                    worker.cancel()
                    await asyncio.gather(worker, return_exceptions=True)

    async def poll(self, client):
        while not self.shutdown.is_set():
            try:
                async with client.get(self.url+'/manifest') as response:
                    response.raise_for_status()
                    manifest = await response.json()
            except (ClientError, asyncio.TimeoutError, ConnectionError):
                # One slow answer on a lossy link is not a lost vehicle.
                if self.manifest_at is None or time.monotonic()-self.manifest_at > MANIFEST_LOSS_S:
                    raise
                self.publish_status()
                await asyncio.sleep(.2)
                continue
            if manifest.get('session') != self.session or manifest.get('system_id') != self.system_id or manifest.get('component_id') != self.component_id:
                raise ValueError('vehicle identity/session changed; reconnect explicitly')
            self.manifest = manifest
            self.manifest_at = time.monotonic()
            self.publish_status()
            await asyncio.sleep(.2)

    async def run(self):
        async with ClientSession(timeout=ClientTimeout(total=3)) as client:
            try:
                async with client.get(self.url+'/manifest') as response:
                    response.raise_for_status()
                    self.manifest = await response.json()
                    self.manifest_at = time.monotonic()
                if self.manifest.get('v') != 1 or self.manifest.get('system_id') != self.system_id or self.manifest.get('component_id') != self.component_id:
                    raise ValueError('remote protocol or FCU identity mismatch')
                self.session = self.manifest['session']
                self.fault = ''
                tasks = [asyncio.create_task(self.stream(client, '/sensors')),
                         asyncio.create_task(self.stream(client, '/camera')),
                         asyncio.create_task(self.poll(client))]
                try:
                    pending = set(tasks)
                    while pending:
                        done, pending = await asyncio.wait(pending, return_when=asyncio.FIRST_EXCEPTION)
                        for task in done:
                            if task == tasks[1]:
                                try: task.result()
                                except Exception as exc: self.camera_fault = str(exc)
                            else:
                                task.result()
                finally:
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
            except Exception as exc:
                self.fault = str(exc)
            # Never follow a changed vehicle automatically, even if the URL
            # comes back. Keep the fault visible until an explicit reconnect.
            while not self.shutdown.is_set():
                self.publish_status()
                await asyncio.sleep(.2)


def main():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('remote_devices')
    driver = RemoteDevices(node)
    thread = threading.Thread(target=lambda: asyncio.run(driver.run()), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        driver.shutdown.set()
        thread.join(4)
        node.destroy_node()
        rclpy.try_shutdown()
