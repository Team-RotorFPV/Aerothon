"""Validated connection/task contracts shared by HTTP and ROS adapters."""
import math
import re
from urllib.parse import urlparse

TASKS = {
    'mission': ('Full mission', True),
    'slam': ('SLAM mapping', False),
    'qr': ('QR detection', False),
    'banner': ('Banner alignment', True),
    'avoidance_manual': ('Avoidance with transmitter', True),
    'avoidance_route': ('Avoidance with waypoint route', True),
    'mechanism': ('Mechanism only', False),
    'delivery_here': ('Payload delivery here', True),
    'delivery_gps': ('Payload delivery at GPS location', True),
}
TERMINAL = {'Completed', 'Failed', 'Outcome unknown'}

def number(value, name, lo, hi):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be numeric')
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} must be numeric') from None
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{name} must be within {lo} and {hi}')
    return value

def profile(value, *, require_gateway=True):
    if not isinstance(value, dict):
        raise ValueError('profile must be an object')
    ident = str(value.get('id', ''))
    if not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', ident):
        raise ValueError('profile ID needs letters, numbers, underscores or hyphens')
    p = dict(value)
    p['name'] = str(p.get('name', ident))[:100]
    p['transport'] = p.get('transport', 'tcp')
    if p['transport'] not in {'tcp', 'udp', 'serial'}:
        raise ValueError('unsupported FCU transport')
    def integer(v, label, lo, hi):
        n = number(v, label, lo, hi)
        if n != int(n):
            raise ValueError(f'{label} must be an integer')
        return int(n)
    p['system_id'] = integer(p.get('system_id', 1), 'system ID', 1, 255)
    p['component_id'] = integer(p.get('component_id', 1), 'component ID', 1, 255)
    if p['transport'] == 'serial':
        device = str(p.get('device', ''))
        if not re.fullmatch(r'/dev/[a-zA-Z0-9_./-]+', device) or '..' in device:
            raise ValueError('serial device must be an absolute /dev path')
        p['baud'] = integer(p.get('baud', 115200), 'baud', 9600, 3000000)
    else:
        host = str(p.get('host', '')).strip()
        if not host or not re.fullmatch(r'[a-zA-Z0-9.:-]+', host):
            raise ValueError('enter an IP address or hostname')
        p['host'] = host
        p['port'] = integer(p.get('port', 5762), 'port', 1, 65535)
        p['bind_port'] = integer(p.get('bind_port', 14555), 'bind port', 1, 65535)
    gateway = str(p.get('gateway', '')).rstrip('/')
    if gateway:
        u = urlparse(gateway)
        if u.scheme not in {'http', 'https'} or not u.hostname or not re.fullmatch(r'[a-zA-Z0-9.:-]+', u.hostname) or u.username or u.password or u.query or u.fragment or u.path not in {'', '/'}:
            raise ValueError('gateway must be an HTTP origin without credentials or a path')
    p['gateway'] = gateway
    # Laptop that also receives the FC's MAVLink from the Pi router over UDP
    # (Mission Planner / QGC on --gcs-port); empty means listen-only.
    gcs_host = str(p.get('gcs_host', '') or '').strip()
    if gcs_host and not re.fullmatch(r'[a-zA-Z0-9.:-]+', gcs_host):
        raise ValueError('GCS host must be an IP address or hostname')
    p['gcs_host'] = gcs_host
    # WHAT is on the other end: the laptop's Gazebo + SITL over the intranet,
    # or the real aircraft. Older profiles say it through transport/gateway.
    vehicle = p.get('vehicle') or ('drone' if p['transport'] == 'serial' else 'simulation')
    if vehicle not in {'simulation', 'drone'}:
        raise ValueError('vehicle must be simulation or drone')
    if vehicle == 'simulation' and p['transport'] == 'serial':
        raise ValueError('a simulated vehicle connects over the network, not the Pi UART')
    if vehicle == 'drone' and gateway:
        raise ValueError('a real drone uses the Pi camera and lidar; clear the sensor gateway')
    p['vehicle'] = vehicle
    # Which winch the real drone has: the FC's DO_WINCH output, or the Pi's
    # own BTS7960 + N20 encoder. The simulated vehicle's gateway is MAVLink.
    winch = p.get('winch_backend') or 'mavlink'
    if winch not in {'mavlink', 'gpio'}:
        raise ValueError('winch backend must be mavlink or gpio')
    if winch == 'gpio' and vehicle != 'drone':
        raise ValueError('the GPIO winch is on the real drone only')
    p['winch_backend'] = winch
    if require_gateway and vehicle == 'simulation' and not gateway:
        raise ValueError('a simulated vehicle requires the laptop sensor gateway')
    # These are deployment files/devices, never arbitrary shell fragments.
    for key in ('camera_device', 'lidar_port'):
        v = str(p.get(key, '/dev/video0' if key == 'camera_device' else '/dev/ttyUSB0'))
        if not re.fullmatch(r'/dev/[a-zA-Z0-9_./-]+', v) or '..' in v:
            raise ValueError(f'{key} must be a /dev path')
        p[key] = v
    p['lidar_yaw_deg'] = number(p.get('lidar_yaw_deg', 0), 'lidar mount yaw', -180, 180)
    p['lidar_mirrored'] = bool(p.get('lidar_mirrored', False))
    p['venue_file'] = str(p.get('venue_file', ''))
    p['camera_hfov'] = number(p.get('camera_hfov', .851919), 'camera HFOV radians', .1, 2.8)
    p['marker_m'] = number(p.get('marker_m', 2.2), 'target marker size', .05, 10)
    p['rc_config'] = p.get('rc_config') or {}
    p['slam'] = p.get('slam', False) is True
    p['autoconnect'] = p.get('autoconnect', vehicle == 'drone') is True
    p['slam_odometry'] = p.get('slam_odometry', 'lidar' if vehicle == 'drone' else 'fcu')
    if p['slam_odometry'] not in {'lidar', 'fcu'}:
        raise ValueError('SLAM odometry must be lidar or fcu')
    p['hardware_reviewed'] = p.get('hardware_reviewed', False) is True
    p['delivery_calibration'] = str(p.get('delivery_calibration', ''))
    if p['delivery_calibration'] and not re.fullmatch(r'[a-zA-Z0-9_-]+\.json', p['delivery_calibration']):
        raise ValueError('select a calibration file stored in the Pi state directory')
    return p

def fcu_url(p):
    if p['transport'] == 'serial':
        return f"{p['device']}:{p['baud']}"
    host = f"[{p['host']}]" if ':' in p['host'] else p['host']
    if p['transport'] == 'tcp':
        return f"tcp://{host}:{p['port']}"
    return f"udp://0.0.0.0:{p['bind_port']}@{host}:{p['port']}"

def execution(value):
    if not isinstance(value, dict) or value.get('task') not in TASKS:
        raise ValueError('unknown task')
    p = dict(value.get('params') or {})
    task = value['task']
    if TASKS[task][1]:
        p['height'] = number(p.get('height', 5), 'height above home', 1, 10)
    if task in {'delivery_here', 'delivery_gps'}:
        if not str(p.get('target', '')).strip():
            raise ValueError('delivery needs the QR payload to verify')
    if task == 'mechanism' and p.get('action') not in {'lower', 'release', 'stow', 'stop'}:
        raise ValueError('select a mechanism action')
    if task in {'delivery_gps', 'avoidance_route'}:
        pts = p.get('waypoints') or []
        if not isinstance(pts, list) or not 1 <= len(pts) <= 100:
            raise ValueError('enter 1 to 100 GPS waypoints')
        p['waypoints'] = [{'lat': number(x.get('lat'), 'latitude', -90, 90),
                           'lon': number(x.get('lon'), 'longitude', -180, 180)} for x in pts]
        if task == 'delivery_gps' and len(pts) != 1:
            raise ValueError('delivery needs one destination')
    return {'task': task, 'params': p}

def command_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{8,100}', value):
        raise ValueError('a unique command_id is required')
    return value
