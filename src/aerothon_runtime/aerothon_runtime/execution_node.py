"""One production commander, one selected tree, one movement owner."""
import json
import math
import os
import time

import py_trees
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Vector3, PoseStamped, TransformStamped
from mavros_msgs.msg import RCIn, State
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import TransformBroadcaster

from mission_bt.mav_commander import Mav
from mission_bt import mission_tree as production
from mission_bt.leg_router import LegRouter
from mission_bt.banner_orbit import fence_ok
from mission_bt.search_planner import point_in_exclusion
from .contracts import TASKS, TERMINAL, execution
from .pilot import validate as validate_rc, intent

S = py_trees.common.Status

# A laptop-simulation vehicle's sensor link may pause this long (steady
# seconds) in flight before recovery; a real drone's sensors are local.
REMOTE_LINK_STALL_S = 3.0


class Route(py_trees.behaviour.Behaviour):
    def __init__(self, owner, points, height, avoidance=False):
        super().__init__('WaypointRoute')
        self.owner, self.points, self.height = owner, points, height
        self.i = 0
        self.router = LegRouter(clearance_m=owner.params['redzone_clearance'])
        self.avoidance = avoidance

    def update(self):
        mav = self.owner.mav
        home = mav.home_global()
        if not home:
            return S.FAILURE
        if self.i >= len(self.points):
            mav.enable_avoidance(False)
            mav.hold_position()
            return S.SUCCESS
        point = self.points[self.i]
        # About home, then into the EKF-origin frame that mav.pos() uses.
        hx, hy = mav.home_local_xy()
        x = math.radians(point['lon']-home[1])*6378137*math.cos(math.radians(home[0])) + hx
        y = math.radians(point['lat']-home[0])*6378137 + hy
        if not mav.geofence_local or not fence_ok(mav, on_red=lambda a,b: point_in_exclusion(a,b,self.owner.params["redzone_clearance"],mav.exclusions))(x,y):
            mav.abort_reason = 'waypoint is outside the configured fence or exclusion margin'
            return S.FAILURE
        if mav.reached(x,y,self.height):
            self.i += 1
            return S.RUNNING
        if self.avoidance:
            target = self.router.target(mav,x,y,self.height)
            if target is None:
                return S.FAILURE
            dx,dy = target[0]-mav.pos()[0],target[1]-mav.pos()[1]
            yaw = mav.yaw()
            body = (math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy)
            length = max(math.hypot(*body), .01)
            self.owner.external_intent(Vector3(x=body[0]/length*self.owner.params['search_speed_mps'], y=body[1]/length*self.owner.params['search_speed_mps'], z=0.0), self.height)
        else:
            if self.router.fly(mav,x,y,self.height,mav.yaw()) == 'BLOCKED':
                return S.FAILURE
        return S.RUNNING


class Pilot(py_trees.behaviour.Behaviour):
    def __init__(self, owner, height):
        super().__init__('TransmitterAvoidance')
        self.owner, self.height = owner, height
        self.previous = time.monotonic()

    def update(self):
        owner = self.owner
        if owner.rc_seen is None or time.monotonic()-owner.rc_seen > .5:
            owner.mav.abort_reason = 'transmitter data stale'
            return S.FAILURE
        try:
            p = intent(owner.rc.channels, owner.rc_config)
        except ValueError as exc:
            owner.mav.abort_reason = str(exc)
            return S.FAILURE
        if p['takeover']:
            owner.takeover('transmitter takeover switch')
            return S.RUNNING
        now = time.monotonic()
        self.height = max(1., min(owner.params['search_alt'], self.height+p['climb']*min(.2, now-self.previous)))
        self.previous = now
        owner.external_intent(Vector3(x=p['forward'],y=p['left'],z=p['yaw']), self.height)
        return S.RUNNING


class Mechanism(py_trees.behaviour.Behaviour):
    def __init__(self, mav, action):
        super().__init__('Mechanism')
        self.mav, self.action, self.sent = mav, action, False

    STATUS_TIMEOUT_S = 3.0

    def update(self):
        if not self.sent:
            self.mav.winch(self.action)
            self.sent = True
            self._fresh_t = self._start_t = time.monotonic()
            return S.RUNNING
        # Abort applies to the bench winch too (one pressed after this start).
        if (getattr(self.mav, 'abort_requested', False)
                and getattr(self.mav, '_abort_t', 0.0) >= getattr(self, '_start_t', float('inf'))):
            self.mav.winch('stop')
            self.mav.abort_reason = 'abort requested'
            return S.FAILURE
        if not self.mav.winch_status_fresh():
            # A dead winch node left this RUNNING for ever with the motor on.
            if time.monotonic() - getattr(self, '_fresh_t', time.monotonic()) > self.STATUS_TIMEOUT_S:
                self.mav.winch('stop')
                self.mav.abort_reason = f'winch status lost for {self.STATUS_TIMEOUT_S:.0f} s; stop sent'
                return S.FAILURE
            return S.RUNNING
        self._fresh_t = time.monotonic()
        state = self.mav.winch_status
        if state.get('fault'):
            self.mav.abort_reason = state['fault']
            return S.FAILURE
        expected = {'lower': {'AT_LIMIT','AT_GROUND'}, 'stow': {'IDLE','RELEASED'},
                    'release': {'RELEASED'}, 'stop': {'IDLE','RELEASED'}}
        if state.get('state') not in expected[self.action]:
            return S.RUNNING
        if self.mav.node.remote and self.action in {'lower','stow'}:
            measured = state.get('measured_payout_m')
            age = state.get('measurement_age_s')
            target = state.get('payout_estimate_m')
            if measured is None or age is None or age > .3 or abs(measured-target) > .1:
                return S.RUNNING
        if self.action == 'release':
            self.feedback_message = 'release requested; physical delivery remains unconfirmed'
        return S.SUCCESS


class ExecutionNode(Node):
    def __init__(self):
        # Keep the production node name so venue.yaml applies unchanged.
        super().__init__('mission_bt')
        self.params = production.declare_mission_params(self)
        self.rc_config = json.loads(self.declare_parameter('rc_config', '{}').value)
        self.remote = bool(self.declare_parameter('remote_gateway', '').value)
        self.slam_enabled = self.declare_parameter('slam_enabled', False).value is True
        self.system_id = self.declare_parameter('target_system_id', 1).value
        self.mav = Mav(self)
        self.last_state = {'value': 'IDLE'}
        self.holder = {'tree': None}
        self.holder['tree'] = self.build_normal(self.params)
        production.attach_arena_dimensions(self, self.mav, self.params, self.holder,
                                           self.build_normal, self.last_state)
        self.guard = production.CheckAbortTriggered(self.mav,self)
        self.tree = None
        self.run = None
        self.seen_ids = set()
        self.ready = False
        self.devices = {}
        self.device_seen = None
        self.fcu_seen = None
        self.fcu_seen_vehicle = None
        self.sensor_seen = {}
        self.rc = None
        self.rc_seen = None
        self.recovery = None
        self.health_pub = self.create_publisher(String, "/runtime/health", 10)
        self.pending_stop_id = None
        self.receipt_pub = self.create_publisher(String, "/runtime/receipt", 10)
        self.status_pub = self.create_publisher(String, '/runtime/status', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.intent_pub = self.create_publisher(Vector3, '/avoidance/intent', 10)
        self.intent_enable = self.create_publisher(Bool, '/avoidance/intent_enabled', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, '/runtime/request', self.request, 10)
        self.create_subscription(Bool, '/mission_ready', lambda m: setattr(self,'ready',m.data), 10)
        self.create_subscription(String, '/devices/status', self.device_status, 10)
        self.create_subscription(State, '/mavros/state', self.on_fcu_state, qos_profile_sensor_data)
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, qos_profile_sensor_data)
        # Only the arrival time matters here. Raw delivery skips deserializing
        # each 1280x720 frame, which cost ~0.45 of a Pi 5 core at 30 Hz (16 Oct).
        for typ,topic,key,raw in [(Image,'/image_raw','camera',True),(LaserScan,'/scan','lidar',False)]:
            self.create_subscription(typ,topic,lambda m,k=key: self.sensor_seen.update({k:time.monotonic()}),
                                     qos_profile_sensor_data,raw=raw)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_tf, qos_profile_sensor_data)
        self.create_timer(.2, self.tick)
        self.create_timer(.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def build_normal(self, params):
        self.params = dict(params)
        self.holder['wall_top'] = params.get('corridor_wall_m')
        self.mav.banner_placement_learned = 'ground' if params.get('banner_placement') == 'ground' else None
        tree = py_trees.trees.BehaviourTree(production.build_root(self.mav, self, params))
        tree.setup(timeout=15)
        return tree

    def normal_task(self):
        return self.run and self.run['task'] == 'mission'

    def pose_tf(self, m):
        t = TransformStamped()
        t.header = m.header
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = m.pose.position.x
        t.transform.translation.y = m.pose.position.y
        t.transform.translation.z = m.pose.position.z
        t.transform.rotation = m.pose.orientation
        self.tf.sendTransform(t)

    def on_rc(self, msg):
        self.rc, self.rc_seen = msg,time.monotonic()
        # Only a live run can be taken over: a switch left on used to rewrite
        # a Completed run as Failed at RC rate.
        if self.run and self.run['state'] not in TERMINAL and self.rc_config:
            try:
                if intent(msg.channels,validate_rc(self.rc_config))['takeover']:
                    self.takeover('transmitter takeover switch')
            except ValueError:
                pass

    def device_status(self, msg):
        self.devices = json.loads(msg.data)
        self.device_seen = time.monotonic()

    def publish(self):
        if self.run:
            self.status_pub.publish(String(data=json.dumps(self.run)))
            # /mission/state is published only while a tree ticks; without a
            # terminal value the GCS keeps the last stage (controls locked)
            # and arena dimensions are refused until reconnect.
            final = {'Completed': 'COMPLETED', 'Failed': 'FAILED'}.get(self.run['state'])
            if final and self.last_state['value'] not in {'', 'IDLE', 'WAITING', 'COMPLETED', 'FAILED'}:
                self.last_state['value'] = final
                self.state_pub.publish(String(data=final))
            if self.pending_stop_id and self.run['state'] in TERMINAL:
                self.receipt(self.pending_stop_id,self.run['state'],self.run.get('reason',''))
                self.pending_stop_id = None

    def receipt(self, cid, state, reason=''):
        self.receipt_pub.publish(String(data=json.dumps({'command_id':cid,'state':state,'reason':reason})))

    def external_intent(self, msg, height):
        self.mav._sp = None
        self.intent_enable.publish(Bool(data=True))
        self.intent_pub.publish(msg)
        self.mav.enable_avoidance(True, hold_alt=height)

    def release_control(self):
        self.mav._sp = None
        self.mav.enable_avoidance(False)
        self.intent_enable.publish(Bool(data=False))
        self.mav.mission_started = False
        # The operator's target is for this run only. Kept, the next run's
        # ScanStartQR succeeded at once on the old target and skipped the QR.
        self.mav.target_override = ""
        if self.tree:
            self.tree.root.stop(S.INVALID)
        self.tree = None

    def takeover(self, reason):
        self.release_control()
        if self.run and self.run['task'] == 'mechanism':
            self.mav.winch('stop')
        self.recovery = None
        self.run.update(state='Failed', reason=reason, control_owner='Transmitter')
        self.publish()  # never issue RTL/GUIDED against a pilot takeover

    def request(self, msg):
        try:
            data = json.loads(msg.data)
            cid = data['command_id']
            if cid in self.seen_ids:
                self.publish()
                return
            self.seen_ids.add(cid)
            if data['cmd'] == 'stop':
                self.pending_stop_id = cid
                self.receipt(cid,'Accepted','stop requested')
                if self.run and self.run['state'] not in TERMINAL:
                    if TASKS[self.run['task']][1]:
                        self.begin_recovery('operator stopped test', False)
                    else:
                        self.mav.winch('stop') if self.run['task'] == 'mechanism' else None
                        self.release_control()
                        self.run.update(state='Completed',reason='stopped by operator')
                        self.publish()
                else:
                    self.receipt(cid,'Completed','no active execution')
                    self.pending_stop_id = None
                return
            if self.run and self.run['state'] not in TERMINAL:
                raise ValueError('execution already active')
            req = execution(data['args'])
            task,p = req['task'],req['params']
            now = time.monotonic()
            if task in {'qr','slam'}:
                if task == 'slam' and not self.slam_enabled:
                    raise ValueError('enable SLAM in the stopped connection profile first')
                sensor = 'camera' if task == 'qr' else 'lidar'
                if now-self.sensor_seen.get(sensor,0) > 1:
                    raise ValueError(f'{sensor} unavailable or stale')
            if TASKS[task][1] and p['height'] > self.params['search_alt']:
                raise ValueError('requested height exceeds configured flight ceiling')
            if TASKS[task][1] and not self.ready:
                raise ValueError('flight readiness interlock not ready')
            if TASKS[task][1] and not self.mav.connected():
                raise ValueError('live FCU connection required')
            if task == 'mechanism' and self.mav.state.armed:
                raise ValueError('mechanism bench execution requires disarm')
            # The avoidance tasks never move the camera, so they do not wait on
            # its (still unqualified) actuator feedback.
            required = {'mission': ['identity','camera','lidar','clock','camera_actuator','winch_actuator'],
                        'mechanism': ['winch_actuator'], 'qr': ['camera','clock'], 'slam': ['lidar','clock'],
                        'avoidance_manual': ['identity','camera','lidar','clock'],
                        'avoidance_route': ['identity','camera','lidar','clock']}.get(task,
                        ['identity','camera','lidar','clock','camera_actuator'] + (['winch_actuator'] if task.startswith('delivery_') else []))
            if self.device_seen is None or now-self.device_seen > 1:
                raise ValueError('required devices or measured actuator feedback are unavailable')
            missing = [k for k in required if self.devices.get(k,{}).get('ok') is not True]
            if missing:
                raise ValueError('required devices or measured actuator feedback are unavailable: ' + ', '.join(
                    f"{k} ({self.devices.get(k,{}).get('reason') or 'not ok'})" for k in missing))
            if task == 'avoidance_manual':
                validate_rc(self.rc_config)
                if self.rc_seen is None or now-self.rc_seen > .5:
                    raise ValueError('live transmitter input is required')
            if task == 'mission' and self.mav.state.armed:
                raise ValueError('full mission starts from a disarmed aircraft')
            self.run = dict(command_id=cid, task=task, params=p, state='Accepted', reason='', control_owner='Runtime')
            self.recovery = None
            if self.mav.reset_pending():
                self.mav.consume_reset()
            self.mav._on_start(Bool(data=TASKS[task][1]))
            if task in {'qr','slam'}:
                self.tree = None
            elif self.normal_task():
                self.holder['tree'] = self.build_normal(self.params)
                self.tree = self.holder['tree']
            else:
                self.tree = py_trees.trees.BehaviourTree(self.component(task,p))
            if self.tree:
                self.tree.setup(timeout=15)
            self.publish()
        except (ValueError,KeyError,TypeError) as exc:
            self.receipt(locals().get('cid'),'Failed',str(exc))

    def component(self, task, p):
        mav = self.mav
        height = p.get('height',self.params['takeoff_alt'])
        seq = py_trees.composites.Sequence(name='ComponentTest',memory=True)
        leaves = []
        if TASKS[task][1]:
            leaves.extend([production.RequireDeliveryZone(mav,self.params['zone_boundary_clearance']),
                           production.UploadArenaFence(mav),production.SetModeArm(mav)])
            if not mav.state.armed or mav.alt() < 1:
                leaves.append(production.Takeoff(mav,height))
            else:
                leaves.append(production.ClimbInPlace('TestHeight',mav,height))
        if task == 'mechanism':
            leaves.append(Mechanism(mav,p['action']))
        elif task.startswith('avoidance_'):
            leaves.append(Pilot(self,height) if task == 'avoidance_manual' else Route(self,p['waypoints'],height,True))
        elif task == 'banner':
            leaves.extend([production.SetCameraPose('CameraBannerTest',mav,'BANNER'),
                           production.AlignToBanner(mav, clock=production._node_clock(self),
                               hfov_rad=self.params['camera_hfov'], image_width_px=self.params['image_width_px'],
                               banner_w_m=self.params['banner_w_m'],banner_h_m=self.params['banner_h_m'],
                               redzone_clearance_m=self.params['redzone_clearance'],alt_floor_m=self.params['corridor_alt'])])
        else:
            mav.set_target(p['target'])
            if task == 'delivery_gps':
                leaves.append(Route(self,p['waypoints'],height))
            leaves.extend([production.SetCameraPose('CameraNadirForTestDrop',mav,'NADIR'),
                production.CenterOnQR('CenterOnTarget',mav,tol=.08,require_match=True,settle_ticks=5,timeout_ticks=400,
                    hfov_rad=self.params['camera_hfov'],image_w_px=self.params['image_width_px'],image_h_px=self.params['image_height_px']),
                production.WinchDrop(mav,height,self.params['search_alt'],winch_reach_m=self.params['winch_reach_m'],
                    hfov_rad=self.params['camera_hfov'],image_w_px=self.params['image_width_px'],image_h_px=self.params['image_height_px'],
                    marker_m=self.params['target_marker_m'],payload_size_m=self.params['payload_size_m'])])
        seq.add_children(leaves)
        return seq

    def begin_recovery(self, reason, failed):
        self.release_control()
        self.mav.abort_reason = reason
        self.recovery = {'reason':reason,'failed':failed,'sent':False}
        self.run.update(state='Running',reason=reason,control_owner='Flight controller',stage='Return and land')

    def tick(self):
        if not self.run or self.run['state'] in TERMINAL:
            self.publish()
            return
        if self.mav.reset_pending() and not self.normal_task():
            reason = self.mav.consume_reset()
            if TASKS[self.run['task']][1]:
                self.takeover(reason)
                return
        if self.recovery:
            # A recovery request lost in the very link gap that caused the
            # recovery left the aircraft hovering in GUIDED with nobody
            # commanding it. Repeat it, at most once a second, while the FCU
            # still reports GUIDED and armed; any other mode is the FCU's own
            # recovery or the pilot's takeover, and is never fought.
            mode = str(getattr(self.mav.state, 'mode', '') or '')
            still_guided = bool(getattr(self.mav.state, 'armed', False)) and mode == 'GUIDED'
            if not self.recovery['sent'] or (
                    still_guided and time.monotonic() - self.recovery.get('sent_t', 0.0) > 1.0):
                production.recover(self.mav)
                self.recovery['sent'] = True
                self.recovery['sent_t'] = time.monotonic()
            if self.mav.connected() and not self.mav.state.armed:
                self.run.update(state='Failed' if self.recovery['failed'] else 'Completed', reason=self.recovery['reason'],disarmed=True)
            self.publish()
            return
        if TASKS[self.run['task']][1] and self.guard.update() == S.SUCCESS:
            self.begin_recovery(self.mav.abort_reason,True)
            return
        self.run['state'] = 'Running'
        if self.tree:
            if self.normal_task():
                self.tree = self.holder['tree']
                production.note_low(self.mav, float(self.holder.get('wall_top') or production.DEFAULT_WALL_TOP_M))
            self.tree.tick()
            if self.tree is None:
                self.publish()
                return
            tip = self.tree.root.tip()
            self.run['stage'] = tip.name if tip else ''
            self.mav.active_stage = self.run['stage']
            self.state_pub.publish(String(data=production.STATE_NAMES.get(self.run['stage'],self.run['stage'])))
            self.last_state['value'] = production.STATE_NAMES.get(self.run['stage'],self.run['stage'])
            if self.normal_task():
                production.latch_mission_failure(self.tree.root,self.mav)
                production.latch_mission_success(self.tree.root,self.mav)
                if self.mav.mission_failed:
                    self.begin_recovery(self.mav.abort_reason, True)
                elif self.mav.mission_complete and not self.mav.state.armed:
                    self.release_control()
                    self.run.update(state='Completed',disarmed=True)
                    self.last_state['value'] = 'COMPLETED'
                if self.tree and production.apply_pending_reset(self.tree.root, self.mav) is not None:
                    if self.run['state'] not in TERMINAL:
                        self.takeover('pilot or FCU intervention')
            elif self.tree.root.status in {S.SUCCESS,S.FAILURE}:
                failed = self.tree.root.status == S.FAILURE
                if TASKS[self.run['task']][1]:
                    self.begin_recovery(self.mav.abort_reason if failed else 'test finished',failed)
                else:
                    unconfirmed_release = (not failed and self.run['task'] == 'mechanism'
                        and self.run['params']['action'] == 'release'
                        and self.mav.winch_status.get('released') is not True)
                    self.release_control()
                    self.run.update(
                        state='Failed' if failed else 'Outcome unknown' if unconfirmed_release else 'Completed',
                        reason='Release requested; physical release has no confirming feedback'
                            if unconfirmed_release else self.mav.abort_reason)
        self.publish()

    def on_fcu_state(self, _msg):
        self.fcu_seen = time.monotonic()
        self.fcu_seen_vehicle = self.get_clock().now().nanoseconds / 1e9

    def fcu_age(self):
        """Seconds since FCU telemetry, in the vehicle's own time.

        A simulated vehicle's heartbeat is paced by SITL's clock: at a
        quarter real time its 1 Hz heartbeat is 4 s apart on the wall and a
        wall-clock limit aborts a healthy flight. Its clock's own liveness
        is the device-link check, on steady time. A real FCU is wall time.
        """
        if self.fcu_seen is None:
            return None
        if self.remote and getattr(self, 'fcu_seen_vehicle', None) is not None:
            return self.get_clock().now().nanoseconds / 1e9 - self.fcu_seen_vehicle
        return time.monotonic() - self.fcu_seen

    def watchdog(self):
        self.health_pub.publish(String(data=json.dumps({'controller_pid':os.getpid(),'state':'ready'})))
        if not self.remote:
            devices = {}
            for k in ('camera','lidar'):
                age = time.monotonic()-self.sensor_seen.get(k,0)
                devices[k] = {'ok':age<1,'receipt_age_s':age if k in self.sensor_seen else None}
            devices['identity'] = {'ok':self.mav.connected(),'value':self.system_id,'basis':'configured MAVROS target; live FCU link'}
            devices['clock'] = {'ok':True,'reason':'system ROS clock'}
            devices['camera_actuator'] = {'ok':False,'reason':'hardware camera motion feedback has not been qualified'}
            winch = self.mav.winch_status if self.mav.winch_status_fresh() else {}
            measured = 'encoder' in str(winch.get('payout_basis', ''))
            devices['winch_actuator'] = {'ok': measured and not winch.get('fault'),
                'reason': winch.get('fault') or ('' if measured else
                    'no measured winch feedback (select the Pi GPIO encoder winch and its measured values)')}
            self.devices = devices
            self.device_seen = time.monotonic()
            self.create_local_status(devices)
        if not self.run or self.run['state'] in TERMINAL or self.recovery:
            return
        if not TASKS[self.run['task']][1]:
            sensor = {'qr':'camera','slam':'lidar'}.get(self.run['task'])
            lost = sensor and time.monotonic()-self.sensor_seen.get(sensor,0)>1
            if lost or (self.remote and (self.device_seen is None or time.monotonic()-self.device_seen>1 or not self.devices.get('clock',{}).get('ok'))):
                if self.run['task']=='mechanism':
                    self.mav.winch('stop')
                self.release_control()
                self.run.update(state='Failed',reason='required device link lost')
                self.publish()
            return
        if TASKS[self.run['task']][1]:
            fault = None
            age = self.fcu_age()
            if age is None or age > 3:
                fault = 'FCU telemetry lost'
            if self.remote and (self.device_seen is None or time.monotonic()-self.device_seen > REMOTE_LINK_STALL_S or not self.devices.get('clock',{}).get('ok')):
                fault = 'device link or vehicle clock lost'
            if fault:
                self.begin_recovery(fault,True)
                # Run safety recovery on steady time even when ROS time stops.
                production.recover(self.mav)
                self.recovery['sent'] = True
                self.publish()

    def create_local_status(self, devices):
        if not hasattr(self,'local_status_pub'):
            self.local_status_pub = self.create_publisher(String,'/devices/status',10)
        self.local_status_pub.publish(String(data=json.dumps(devices)))


def main():
    from .ownership import acquire
    owner = acquire()
    rclpy.init()
    node = ExecutionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        owner.close()
