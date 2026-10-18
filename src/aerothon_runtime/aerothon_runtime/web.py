"""Always-on Pi web host. No aircraft command is issued at boot or reconnect."""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import time
from collections import deque

from aiohttp import ClientError, ClientSession, ClientTimeout, WSMsgType, web
from .contracts import TASKS, TERMINAL, command_id, execution, fcu_url, profile


def process_identity(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
        start = stat[stat.rfind(')')+2:].split()[19]
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()+':'+start
    except (OSError,IndexError):
        return None

class AdoptedProcess:
    def __init__(self, pid, identity):
        self.pid,self.identity = pid,identity
    @property
    def returncode(self):
        return None if process_identity(self.pid) == self.identity else 0
    async def wait(self):
        while self.returncode is None:
            await asyncio.sleep(.2)
        return 0


class ClientRelay:
    """A bounded writer per browser; map backpressure cannot block other peers.

    Telemetry and maps keep only their latest pending snapshot. Command receipts
    retain their order. A stalled transport closes explicitly instead of silently
    removing a browser that still thinks its socket is connected.
    """
    def __init__(self, socket, rle=False):
        self.socket = socket
        self.rle = rle
        self.latest = {}
        self.receipts = deque()
        self.changed = asyncio.Event()
        self.closed = False
        self.compressed = bool(getattr(socket, 'compress', False))
        self.next_telemetry = 0.
        self.last_snapshot = None
        self.sent = {'telemetry': 0, 'map': 0}
        self.task = asyncio.create_task(self.write())

    def offer(self, kind, data):
        if self.closed:
            raise ConnectionError('browser writer closed')
        if kind in {'telemetry', 'map'}:
            self.latest[kind] = data
        else:
            if len(self.receipts) >= 32:
                raise ConnectionError('browser receipt queue full')
            self.receipts.append(data)
        self.changed.set()

    async def write(self):
        try:
            while True:
                kind = None
                if self.receipts:
                    data = self.receipts.popleft()
                elif 'map' in self.latest and self.last_snapshot == 'telemetry':
                    kind = 'map'
                    data = self.latest.pop('map')
                elif 'telemetry' in self.latest and time.monotonic() >= self.next_telemetry:
                    kind = 'telemetry'
                    data = self.latest.pop('telemetry')
                elif 'map' in self.latest:
                    kind = 'map'
                    data = self.latest.pop('map')
                else:
                    self.changed.clear()
                    if 'telemetry' in self.latest:
                        try:
                            await asyncio.wait_for(self.changed.wait(), max(.001, self.next_telemetry-time.monotonic()))
                        except TimeoutError:
                            pass
                    else:
                        await self.changed.wait()
                    continue
                await asyncio.wait_for(self.socket.send_str(data), 5.)
                if kind:
                    self.last_snapshot = kind
                    self.sent[kind] += 1
                if kind == 'telemetry' and not self.compressed:
                    # Native browsers negotiate deflate. An uncompressed
                    # proxy/client gets bounded display traffic, full sensor data.
                    self.next_telemetry = time.monotonic() + .5
        except (TimeoutError, ConnectionError, RuntimeError):
            self.closed = True
            await self.socket.close(code=1013, message=b'connection stalled; reconnect')

    async def close(self):
        self.closed = True
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)


def map_for_legacy_client(envelope):
    """Older open GCS pages still receive the original full-array map schema."""
    grid = dict(envelope['data'])
    size = grid['w']*grid['h']
    if not 0 < size <= 1024*1024:
        raise ValueError('invalid map size')
    runs = grid.pop('rle')
    grid.pop('encoding', None)
    if len(runs) % 2:
        raise ValueError('invalid map runs')
    data = []
    for i in range(0, len(runs), 2):
        value, count = runs[i:i+2]
        if not isinstance(count, int) or count <= 0 or len(data)+count > size:
            raise ValueError('invalid map runs')
        data.extend([value]*count)
    if len(data) != size:
        raise ValueError('incomplete map')
    grid['data'] = data
    return json.dumps({**envelope, 'data': grid}, separators=(',', ':'))

class Host:
    def __init__(self, static, state_dir, launch=None, aggregator='http://127.0.0.1:8765'):
        self.static = Path(static).resolve()
        self.state_dir = Path(state_dir)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.state_dir / 'commands.sqlite')
        self.db.execute('CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, request TEXT, result TEXT)')
        self.db.execute('UPDATE commands SET result=? WHERE json_extract(result, "$.state") NOT IN ("Completed", "Failed", "Outcome unknown")',
                        (json.dumps({'state': 'Outcome unknown', 'reason': 'web service restarted; inspect aircraft before starting again'}),))
        self.db.commit()
        self.profiles_path = self.state_dir / 'profiles.json'
        self.profiles = self.read_profiles()
        self.controlled = False
        self.proc = None
        self.profile_id = None
        self.owner_path = self.state_dir / 'runtime-owner.json'
        owner = self.read_owner()
        if owner is not None:
            self.controlled = owner.get('controlled',True)
            self.profile_id = owner.get('profile_id')
            if owner.get('identity') and process_identity(owner['pid']) == owner['identity']:
                self.proc = AdoptedProcess(owner['pid'],owner['identity'])
                self.profile_id = owner['profile_id']
        self.telemetry = None
        self.received = 0.0
        self.execution = None
        self.start_reserved = None
        self.clients = set()
        self.internal_ws = None
        self.lock = asyncio.Lock()
        self.launch = launch or self.launch_runtime
        self.aggregator = aggregator
        self.session = None
        self.background = None
        self.boot_connection = None
        self.auto_suspended = False
        self.connection_error = ''
        self.selection_path = self.state_dir / 'connection-selection.json'
        self.runlog = None
        self.map_online = os.environ.get('AEROTHON_MAP_ONLINE', '1') != '0'
        self.tile_origin = os.environ.get('AEROTHON_MAP_TILE_ORIGIN',
            'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile').rstrip('/')
        self.tile_slots = asyncio.Semaphore(4)

    def fresh(self):
        return self.telemetry is not None and time.monotonic() - self.received < 3

    def read_owner(self):
        """The recorded runtime owner, or None.

        A power cut on the drone (the Pi browned out on its BEC, 16 October)
        left this file empty, and the web host crash-looped on it at every
        start, so the GCS never came back. An unreadable record is kept aside
        and treated as no adopted runtime: after a reboot none survives."""
        if not self.owner_path.exists():
            return None
        try:
            owner = json.loads(self.owner_path.read_text())
            if not isinstance(owner, dict):
                raise ValueError('owner record is not an object')
            return owner
        except (OSError, ValueError) as exc:
            aside = self.owner_path.with_name(f'runtime-owner.unreadable-{int(time.time())}.json')
            try:
                self.owner_path.replace(aside)
            except OSError:
                pass
            print(f'runtime owner record unreadable ({exc}); kept as {aside.name}, no runtime adopted', flush=True)
            return None

    def read_profiles(self):
        """Saved connection profiles, each re-validated.

        An empty or corrupt profiles.json (power cut mid-write) used to raise
        here, and the container loop restarted the web host every 2 s: no
        GCS. The unreadable file is kept aside; a profile that no longer
        validates is dropped with a message instead of failing Connect."""
        if not self.profiles_path.exists():
            return {}
        try:
            raw = json.loads(self.profiles_path.read_text())
            if not isinstance(raw, dict):
                raise ValueError('profiles file is not an object')
        except (OSError, ValueError) as exc:
            aside = self.profiles_path.with_name(f'profiles.unreadable-{int(time.time())}.json')
            try:
                self.profiles_path.replace(aside)
            except OSError:
                pass
            print(f'connection profiles unreadable ({exc}); kept as {aside.name}', flush=True)
            return {}
        profiles = {}
        for key, value in raw.items():
            try:
                p = profile(value, require_gateway=False)
                profiles[p['id']] = p
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                print(f'connection profile {key!r} skipped: {exc}', flush=True)
        return profiles

    def write_profiles(self):
        tmp = self.profiles_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            f.write(json.dumps(self.profiles, indent=2))
            f.flush()
            os.fsync(f.fileno())
        tmp.replace(self.profiles_path)

    def write_owner(self, owner):
        # Written whole or not at all: a temporary file, synced, then renamed.
        tmp = self.owner_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            f.write(json.dumps(owner))
            f.flush()
            os.fsync(f.fileno())
        tmp.replace(self.owner_path)

    def disarmed(self):
        return self.fresh() and self.telemetry.get('safety',{}).get('fcu_connected') is True and self.telemetry.get('mission', {}).get('armed') is False and self.telemetry.get('sample_age_s',{}).get('state',999)<3

    def stopped(self):
        return self.proc is None or self.proc.returncode is not None

    def editable(self):
        return self.stopped() and (self.profile_id is None or self.disarmed())

    def state(self):
        return {'profiles': list(self.profiles.values()), 'profile_id': self.profile_id,
                'runtime': 'Stopped' if self.stopped() else 'Running',
                'fresh': self.fresh(), 'can_disconnect': self.disarmed() or not self.controlled,
                'can_edit_profiles': self.editable(), 'execution': self.execution,
                'autoconnect_suspended': self.auto_suspended,
                'connection_error': self.connection_error,
                'streams': [{'compressed': peer.compressed, **peer.sent} for peer in self.clients],
                'tasks': [{'id': k, 'label': v[0], 'movement': v[1]} for k, v in TASKS.items()]}

    async def launch_runtime(self, p):
        vehicle = p.get('vehicle', 'drone' if p['transport'] == 'serial' else 'simulation')
        state = Path(os.environ.get('AEROTHON_DEVICE_STATE_DIR', Path.home()/'.aerothon'))
        state.mkdir(parents=True, exist_ok=True)
        # Refuse before anything changes: the profile router acts on
        # vehicle_connection.json, so a refused Connect must not rewrite it.
        if p.get('delivery_calibration') and not (state/p['delivery_calibration']).is_file():
            raise ValueError('selected delivery calibration is not installed on the Pi')
        current = state/'vehicle_connection.json'
        temporary = current.with_suffix('.tmp')
        temporary.write_text(json.dumps(p, indent=2))
        temporary.replace(current)
        link = 'udp://127.0.0.1:14555@127.0.0.1:14556' if p['transport']=='serial' else fcu_url(p)
        args = ['ros2', 'launch', 'mission_bringup', 'mission2.launch.py',
                'managed:=true', 'rviz:=false', f"slam:={str(p.get('slam',False)).lower()}",
                f"slam_odometry:={p.get('slam_odometry', 'lidar' if vehicle == 'drone' else 'fcu')}",
                f"use_sim:={'true' if p['gateway'] else 'false'}",
                'camera_backend:=mavlink', f"winch_backend:={p.get('winch_backend', 'mavlink')}",
                f"fcu_url:={link}",
                f"target_system_id:={p['system_id']}", f"target_component_id:={p['component_id']}",
                f"camera_hfov:={p['camera_hfov']}", f"target_marker_m:={p['marker_m']}",
                'image_topic:=/image_raw', f"camera_device:={p['camera_device']}",
                f"lidar_port:={p['lidar_port']}", f"lidar_yaw_deg:={p['lidar_yaw_deg']}",
                f"lidar_mirrored:={str(p['lidar_mirrored']).lower()}",
                f"rc_config:={json.dumps(p['rc_config'], separators=(',', ':'))}"]
        if p['gateway']:
            # ros2 launch rejects an empty 'name:=' argument; a real drone has
            # no sensor gateway and the launch file defaults it to ''.
            args.append(f"remote_gateway:={p['gateway']}")
        if p['venue_file']:
            args.append(f"params_file:={p['venue_file']}")
        env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        if p.get('delivery_calibration'):
            calibration = state/p['delivery_calibration']
            if not calibration.is_file():
                raise ValueError('selected delivery calibration is not installed on the Pi')
            env['AEROTHON_DELIVERY_CALIBRATION'] = str(calibration)
        # One handle per Connect leaked a file descriptor each time; and the
        # log grew without bound. Keep the previous run as runtime.log.1.
        if self.runlog:
            self.runlog.close()
        log = self.state_dir / 'runtime.log'
        try:
            if log.stat().st_size > 50 * 1024 * 1024:
                log.replace(log.with_name('runtime.log.1'))
        except OSError:
            pass
        self.runlog = open(log, 'ab', buffering=0)
        return await asyncio.create_subprocess_exec(*args, env=env, start_new_session=True,
                                                     stdout=self.runlog, stderr=self.runlog)

    async def start(self, app):
        self.session = ClientSession(timeout=ClientTimeout(total=10))
        self.background = asyncio.create_task(self.listen())
        if not getattr(self, 'standalone', False):
            self.boot_connection = asyncio.create_task(self.autoconnect())

    def select_connection(self, ident):
        tmp = self.selection_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            json.dump({'profile_id': ident}, f)
            f.flush()
            os.fsync(f.fileno())
        tmp.replace(self.selection_path)

    def boot_profile(self):
        # The selected profile wins, including an explicit opt-out. Migration
        # adopts the single saved hardware profile, never an arbitrary vehicle.
        try:
            ident = json.loads(self.selection_path.read_text())['profile_id']
        except (OSError, ValueError, KeyError, TypeError):
            ident = self.profile_id
        if ident in self.profiles:
            p = self.profiles[ident]
            return p if p.get('autoconnect') else None
        candidates = [p for p in self.profiles.values()
                      if p.get('vehicle') == 'drone' and p.get('autoconnect')]
        return candidates[0] if len(candidates) == 1 else None

    async def prepare_connection(self, p):
        self.telemetry = None
        self.received = 0
        self.execution = None
        # Preparing sensors and telemetry never submits a Start or takes control.
        self.proc = await self.launch(p)
        self.profile_id = p['id']
        self.connection_error = ''
        self.select_connection(p['id'])
        pid = getattr(self.proc, 'pid', None)
        identity = process_identity(pid) if pid else None
        if identity:
            self.write_owner({'pid': pid, 'identity': identity,
                              'profile_id': p['id'], 'controlled': self.controlled})

    async def autoconnect(self):
        delay = 1.0
        while True:
            try:
                async with self.lock:
                    p = self.boot_profile()
                    if not self.auto_suspended and p and self.stopped():
                        # A lost controlled runtime may recover the SAME vehicle.
                        if not self.controlled or self.profile_id == p['id']:
                            await self.prepare_connection(p)
                            delay = 1.0
            except (OSError, ValueError, KeyError) as exc:
                self.connection_error = str(exc)
                delay = min(30.0, delay * 2)
            await asyncio.sleep(delay)

    async def close(self, app):
        # Do not kill a possibly airborne runtime on HTTP shutdown. systemd
        # uses KillMode=process so its flight process survives a web restart.
        if self.background:
            self.background.cancel()
            await asyncio.gather(self.background, return_exceptions=True)
        if self.boot_connection:
            self.boot_connection.cancel()
            await asyncio.gather(self.boot_connection, return_exceptions=True)
        await asyncio.gather(*(peer.close() for peer in list(self.clients)), return_exceptions=True)
        await self.session.close()
        self.db.close()
        if self.runlog:
            self.runlog.close()

    async def listen(self):
        while True:
            try:
                async with self.session.ws_connect(self.aggregator, heartbeat=2, max_msg_size=8*1024*1024) as ws:
                    self.internal_ws = ws
                    async for m in ws:
                        if m.type != WSMsgType.TEXT:
                            continue
                        env = json.loads(m.data)
                        if env.get('kind') == 'telemetry':
                            self.telemetry = env['data']
                            self.received = time.monotonic()
                            if 'execution' in self.telemetry and not self.start_reserved:
                                self.execution = self.telemetry['execution']
                        elif env.get('kind') == 'execution':
                            if self.start_reserved and env['data'].get('command_id') != self.start_reserved:
                                continue
                            self.start_reserved = None
                            self.execution = env['data']
                            cid = self.execution.get('command_id')
                            if cid:
                                self.db.execute('UPDATE commands SET result=? WHERE id=?', (json.dumps(self.execution), cid))
                                self.db.commit()
                        elif env.get('kind') == 'receipt':
                            d = env['data']
                            if self.execution and d.get('command_id') == self.execution.get('command_id') and d.get('state') in TERMINAL:
                                self.execution = d
                                self.start_reserved = None
                            self.db.execute('UPDATE commands SET result=? WHERE id=?',(json.dumps(d),d.get('command_id')))
                            self.db.commit()
                        elif env.get('kind') == 'ack':
                            d = env['data']
                            cid = d.get('cmd_id')
                            row = self.db.execute('SELECT result FROM commands WHERE id=?', (cid,)).fetchone()
                            if row and json.loads(row[0]).get('state') == 'Submitted':
                                r = {'command_id': cid, 'state': 'Failed' if d.get('result') == 'rejected' else 'Accepted', 'reason': d.get('reason', '')}
                                self.db.execute('UPDATE commands SET result=? WHERE id=?', (json.dumps(r), cid))
                                self.db.commit()
                            # A Start the aggregator refused never reaches the
                            # execution service, so no status will release the
                            # reservation; without this every later Start and
                            # flight command was refused until reconnect.
                            if d.get('result') == 'rejected' and cid and cid == self.start_reserved:
                                self.start_reserved = None
                                self.execution = {'command_id': cid, 'state': 'Failed',
                                                  'reason': d.get('reason', '') or 'start refused by runtime'}
                        legacy_map = None
                        for client in list(self.clients):
                            try:
                                payload = m.data
                                if env.get('kind') == 'map' and env.get('data', {}).get('encoding') == 'rle' and not client.rle:
                                    if legacy_map is None:
                                        legacy_map = map_for_legacy_client(env)
                                    payload = legacy_map
                                client.offer(env.get('kind'), payload)
                            except (ConnectionError, RuntimeError):
                                self.clients.discard(client)
                                await client.close()
            except (OSError, asyncio.TimeoutError, ValueError):
                pass
            except Exception as exc:  # noqa: BLE001
                # aiohttp ClientErrors (aggregator restarting mid-handshake), a
                # malformed envelope or a sqlite error used to end this task
                # silently: no telemetry again, every command refused as stale.
                print(f'aggregator link error ({type(exc).__name__}: {exc}); reconnecting', flush=True)
            finally:
                self.internal_ws = None
                if self.start_reserved:
                    cid = self.start_reserved
                    self.execution = {'command_id':cid,'state':'Outcome unknown','reason':'runtime connection lost; no retry'}
                    self.db.execute('UPDATE commands SET result=? WHERE id=?',(json.dumps(self.execution),cid))
                    self.db.commit()
                    self.start_reserved = None
            await asyncio.sleep(1)

    async def command(self, data):
        cid = command_id(data.get('command_id'))
        canonical = json.dumps(data, sort_keys=True, allow_nan=False)
        async with self.lock:
            row = self.db.execute('SELECT request,result FROM commands WHERE id=?', (cid,)).fetchone()
            if row:
                if row[0] != canonical:
                    raise ValueError('command ID already used with different parameters')
                return json.loads(row[1])
            cmd = data.get('cmd')
            if cmd not in {'start', 'stop', 'land', 'abort', 'set_mode', 'set_delivery_zone', 'set_geofence', 'set_target', 'arm', 'disarm', 'takeoff', 'set_banner_colours', 'set_arena_dimensions', 'save_dimension_preset', 'delete_dimension_preset', 'lidar_cal_capture', 'lidar_cal_apply', 'lidar_cal_reset', 'gimbal_pitch'}:
                raise ValueError('unknown command')
            recovery = cmd in {'stop','land','abort'} or (cmd=='set_mode' and data.get('args',{}).get('mode')=='RTL')
            if not self.internal_ws or (not self.fresh() and not recovery):
                raise ValueError('runtime telemetry is unavailable or stale')
            movement = cmd in {'arm','takeoff','set_mode'} or (cmd=='start' and TASKS[execution(data.get('args') or {})['task']][1])
            if movement and not recovery and (self.telemetry.get('safety',{}).get('fcu_connected') is not True or self.telemetry.get('sample_age_s',{}).get('state',999) >= 3):
                raise ValueError('fresh FCU state is required')
            if cmd == 'disarm' and (self.telemetry.get('safety',{}).get('on_ground') is not True or self.telemetry.get('sample_age_s',{}).get('extended_state',999) >= 3):
                raise ValueError('fresh FCU on-ground evidence is required for manual disarm')
            if cmd == 'start':
                if self.telemetry.get('runtime_health',{}).get('fresh') is not True:
                    raise ValueError('production execution service is unavailable')
                if self.start_reserved:
                    raise ValueError('another start is awaiting runtime confirmation')
                task_request = execution(data.get('args') or {})
                if TASKS[task_request['task']][1] and self.telemetry.get('safety',{}).get('ready') is not True:
                    raise ValueError('flight readiness interlock not ready')
                if self.execution and self.execution.get('state') == 'Outcome unknown' and not self.disarmed():
                    raise ValueError('previous outcome unknown; confirm disarm first')
                if self.execution and self.execution.get('state') not in TERMINAL:
                    raise ValueError('another execution is active')
            if cmd in {'arm', 'takeoff', 'set_mode'} and self.execution and self.execution.get('state') not in TERMINAL:
                raise ValueError('active execution owns flight commands; stop or use the transmitter')
            if cmd in {'arm','takeoff','set_mode'} or (cmd=='start' and TASKS[execution(data.get('args') or {})['task']][1]):
                self.controlled = True
                owner = self.read_owner()
                if owner is not None:
                    owner['controlled'] = True
                    self.write_owner(owner)
            result = {'command_id': cid, 'state': 'Submitted', 'reason': ''}
            self.db.execute('INSERT INTO commands VALUES (?,?,?)', (cid, canonical, json.dumps(result)))
            self.db.commit()  # durable before any side effect
            if cmd == 'start':
                self.execution = result
                self.start_reserved = cid
            target = 'runtime_start' if cmd == 'start' else 'runtime_stop' if cmd == 'stop' else cmd
            try:
                await self.internal_ws.send_json({'kind': 'command', 'data': {
                    'cmd_id': cid, 'cmd': target, 'args': data.get('args') or {}, 'confirm':data.get('confirm') is True}})
            except Exception:
                result.update(state='Outcome unknown', reason='connection lost while submitting; command will not be retried')
                self.db.execute('UPDATE commands SET result=? WHERE id=?', (json.dumps(result), cid))
                self.db.commit()
            return result

    async def check(self):
        t = self.telemetry or {}
        devices = t.get('devices') or {}
        items = [{'key': 'fcu', 'ok': self.fresh() and t.get('safety', {}).get('fcu_connected') is True,
                  'reason': 'live FCU telemetry' if self.fresh() else 'telemetry stale or missing'}]
        items.append({'key':'runtime','ok':self.fresh() and t.get('runtime_health',{}).get('fresh') is True,'reason':'production execution service heartbeat'})
        items.extend({'key': k, **v} for k, v in devices.items() if isinstance(v, dict))
        required = {'identity', 'camera', 'lidar', 'clock', 'camera_actuator', 'winch_actuator'}
        seen = {i['key'] for i in items}
        items.extend({'key': k, 'ok': False, 'reason': 'not measured'} for k in sorted(required - seen))
        return {'items': items, 'flight_ready': self.fresh() and t.get('safety', {}).get('ready') is True}


@web.middleware
async def errors(request, handler):
    origin = request.headers.get('Origin')
    if origin and origin != f'{request.scheme}://{request.host}':
        raise web.HTTPForbidden(text='cross-origin control is disabled')
    try:
        return await handler(request)
    except (ValueError, KeyError, TypeError) as exc:
        return web.json_response({'error': str(exc)}, status=400)
    except web.HTTPException:
        raise
    except Exception as exc:  # noqa: BLE001
        # The UI parses JSON first; a plain-text 500 showed "Unexpected token".
        return web.json_response({'error': f'{type(exc).__name__}: {exc}'}, status=500)


def application(host):
    app = web.Application(middlewares=[errors], client_max_size=256 * 1024)
    app.on_startup.append(host.start)
    app.on_cleanup.append(host.close)

    async def state(request):
        return web.json_response(host.state())

    async def save(request):
        async with host.lock:
            if not host.editable():
                raise ValueError('stop the runtime with confirmed disarm before editing connections')
            p = profile(await request.json(), require_gateway=not getattr(host, 'standalone', False))
            if getattr(host, 'standalone', False) and p['transport'] == 'serial':
                raise ValueError('standalone GCS receives TCP/UDP from a router; it does not open the FCU serial port')
            host.profiles[p['id']] = p
            host.write_profiles()
            if p.get('autoconnect') and not getattr(host, 'standalone', False):
                host.select_connection(p['id'])
                host.auto_suspended = False
            return web.json_response(p)

    async def connect(request):
        async with host.lock:
            ident = (await request.json())['profile_id']
            if not host.editable() and not (host.stopped() and ident == host.profile_id):
                raise ValueError('runtime must be stopped with confirmed disarm; reconnect the same vehicle to recover a lost link')
            p = host.profiles[ident]
            if getattr(host, 'standalone', False):
                await host.connect_profile(p)
                return web.json_response(host.state())
            await host.prepare_connection(p)
            host.auto_suspended = False
            return web.json_response(host.state())

    async def disconnect(request):
        async with host.lock:
            if getattr(host, 'standalone', False):
                await host.disconnect_profile()
                return web.json_response(host.state())
            if host.controlled and not host.disarmed():
                raise ValueError('confirmed live disarm required; use Stop/return and land first')
            host.auto_suspended = True
            if not host.stopped():
                try:
                    os.killpg(host.proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass  # it exited between the check and the signal
                try:
                    await asyncio.wait_for(host.proc.wait(), 15)
                except asyncio.TimeoutError:
                    raise ValueError('runtime is still shutting down; no forced termination')
            host.owner_path.unlink(missing_ok=True)
            host.profile_id = None
            host.controlled = False
            host.telemetry = None
            return web.json_response(host.state())

    async def command(request):
        return web.json_response(await host.command(await request.json()), status=202)

    async def receipt(request):
        row = host.db.execute('SELECT result FROM commands WHERE id=?', (request.match_info['id'],)).fetchone()
        if not row:
            raise web.HTTPNotFound()
        return web.json_response(json.loads(row[0]))

    async def check(request):
        return web.json_response(await host.check())

    async def websocket(request):
        ws = web.WebSocketResponse(heartbeat=5, compress=True)
        await ws.prepare(request)
        peer = ClientRelay(ws, rle=request.query.get('map_encoding') == 'rle')
        host.clients.add(peer)
        try:
            async for _ in ws:
                pass  # all mutations use the validated, journalled HTTP API
        finally:
            host.clients.discard(peer)
            await peer.close()
        return ws

    async def video(request):
        async with host.session.get('http://127.0.0.1:8080/' + request.match_info['path'],
                                    params=request.query, timeout=ClientTimeout(total=None, sock_connect=3)) as upstream:
            response = web.StreamResponse(status=upstream.status,
                                          headers={'Content-Type': upstream.headers.get('Content-Type', 'application/octet-stream'), 'Cache-Control': 'no-store'})
            await response.prepare(request)
            async for chunk in upstream.content.iter_chunked(32768):
                await response.write(chunk)
            return response

    async def logs(request):
        p = host.state_dir / 'runtime.log'
        with p.open('rb') if p.exists() else open(os.devnull, 'rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 64000))
            body = f.read()
        return web.Response(body=body, content_type='text/plain')

    async def satellite(request):
        try:
            z, x, y = (int(request.match_info[k]) for k in ('z', 'x', 'y'))
        except ValueError:
            raise web.HTTPBadRequest(text='invalid tile coordinates') from None
        if not 0 <= z <= 19 or not 0 <= x < 2**z or not 0 <= y < 2**z:
            raise web.HTTPBadRequest(text='tile coordinates out of range')
        relative = Path(str(z)) / str(x) / f'{y}.jpg'
        cached = host.state_dir / 'map-tiles' / relative
        for path in (host.static / 'satellite' / relative, cached):
            if path.is_file():
                return web.FileResponse(path, headers={'Cache-Control': 'public, max-age=86400'})
        if not host.map_online:
            raise web.HTTPNotFound(text='map tile is not cached; online maps disabled')
        async with host.tile_slots:
            if cached.is_file():
                return web.FileResponse(cached)
            try:
                async with host.session.get(f'{host.tile_origin}/{z}/{y}/{x}',
                        headers={'User-Agent': 'AeroTHON-GCS/1.0'},
                        timeout=ClientTimeout(total=8)) as upstream:
                    upstream.raise_for_status()
                    chunks = bytearray()
                    async for chunk in upstream.content.iter_chunked(65536):
                        chunks.extend(chunk)
                        if len(chunks) > 2*1024*1024:
                            raise ValueError('map tile exceeds size limit')
                    body = bytes(chunks)
                    if (upstream.content_type != 'image/jpeg'
                            or not body.startswith(b'\xff\xd8\xff') or not body.endswith(b'\xff\xd9')):
                        raise ValueError('map provider did not return JPEG imagery')
                cached.parent.mkdir(parents=True, exist_ok=True)
                tmp = cached.with_suffix(f'.{os.getpid()}.{id(request)}.tmp')
                tmp.write_bytes(body)
                tmp.replace(cached)
                return web.Response(body=body, content_type='image/jpeg',
                                    headers={'Cache-Control': 'public, max-age=86400'})
            except (ClientError, TimeoutError, ValueError):
                raise web.HTTPBadGateway(text='map imagery unavailable; check internet or cached coverage') from None

    async def static(request):
        path = (host.static / request.match_info.get('path', 'index.html')).resolve()
        if not path.is_relative_to(host.static):
            raise web.HTTPForbidden()
        if not path.is_file():
            path = host.static / 'index.html'
        if not path.is_file():
            return web.Response(status=503, text='Build the GCS browser assets before serving the Pi webpage.')
        return web.FileResponse(path)

    app.router.add_get('/api/state', state)
    app.router.add_post('/api/profiles', save)
    app.router.add_post('/api/connect', connect)
    app.router.add_post('/api/disconnect', disconnect)
    app.router.add_get('/api/check', check)
    app.router.add_post('/api/commands', command)
    app.router.add_get('/api/commands/{id}', receipt)
    app.router.add_get('/api/logs', logs)
    app.router.add_get('/ws', websocket)
    app.router.add_get('/video/{path:.*}', video)
    app.router.add_get('/satellite/{z}/{x}/{y}.jpg', satellite)
    app.router.add_get('/{path:.*}', static)
    return app


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--static', required=True)
    p.add_argument('--state-dir', default=os.path.expanduser('~/.local/state/aerothon'))
    p.add_argument('--aggregator', default='http://127.0.0.1:8765')
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8899)
    args = p.parse_args()
    web.run_app(application(Host(args.static, args.state_dir, aggregator=args.aggregator)), host=args.bind, port=args.port)

if __name__ == '__main__':
    main()
