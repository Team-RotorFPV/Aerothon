"""Bounded binary ROS device transport. No world or grading topics are allowed."""
import json
import struct

TOPICS = {
    'camera': ('sensor_msgs/msg/CompressedImage', '/image_raw'),
    'camera_info': ('sensor_msgs/msg/CameraInfo', '/camera/camera_info'),
    'scan': ('sensor_msgs/msg/LaserScan', '/scan'),
    'clock': ('rosgraph_msgs/msg/Clock', '/clock'),
    'joints': ('sensor_msgs/msg/JointState', '/joint_states'),
}
MAX_PACKET = 8 * 1024 * 1024

def pack(kind, session, body):
    if kind not in TOPICS or len(body) > MAX_PACKET:
        raise ValueError('unsupported device packet')
    header = json.dumps({'v': 1, 'kind': kind, 'session': session}, separators=(',', ':')).encode()
    return struct.pack('!I', len(header)) + header + body

def unpack(packet, session):
    if len(packet) < 4 or len(packet) > MAX_PACKET + 1024:
        raise ValueError('invalid device packet size')
    n = struct.unpack('!I', packet[:4])[0]
    if not 0 < n <= 1024 or n > len(packet) - 4:
        raise ValueError('invalid device packet header')
    h = json.loads(packet[4:4+n])
    if h.get('v') != 1 or h.get('session') != session or h.get('kind') not in TOPICS:
        raise ValueError('device session or protocol mismatch')
    return h['kind'], packet[4+n:]
