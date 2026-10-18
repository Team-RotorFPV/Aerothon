"""ROS interfaces and network transport checks. These are not rendered flights."""
import asyncio
import json
import socket
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import rclpy
from sensor_msgs.msg import JointState, LaserScan, Image
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from aerothon_runtime.execution_node import ExecutionNode
from aerothon_runtime.gateway import Gateway
from aerothon_runtime.remote_devices import RemoteDevices
from aerothon_runtime.device_wire import pack
from aerothon_runtime.contracts import TASKS

def test_telemetry_reports_health_when_vehicle_clock_is_missing(context):
    from gcs_aggregator.aggregator import Aggregator
    from rclpy.parameter import Parameter
    node=Aggregator()
    node.set_parameters([Parameter('use_sim_time',value=True)])
    sent=[]
    node._ws_clients={object()};node._broadcast=sent.append
    try:
        until=time.monotonic()+.6
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        assert any(json.loads(m)['kind']=='telemetry' for m in sent)
    finally:node.destroy_node()

def test_lidar_rate_uses_capture_clock_and_silence_fails_closed(context):
    from gcs_aggregator.readiness_node import Readiness
    node=Readiness()
    try:
        for i in range(30):
            scan=LaserScan();scan.header.stamp.sec=100+i//10;scan.header.stamp.nanosec=(i%10)*100000000
            node._on_lidar(scan)
        assert node.lidar.hz(103)==10
        before=len(node.lidar.stamps);node._on_lidar(scan)
        assert len(node.lidar.stamps)==before
        assert node.lidar.hz(107)==0
    finally:node.destroy_node()


def test_sensor_graphs_receive_typed_ros_vibration_and_magnetic_messages(context):
    from gcs_aggregator.aggregator import Aggregator
    from mavros_msgs.msg import Vibration
    from sensor_msgs.msg import MagneticField
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    aggregator=Aggregator()
    source=rclpy.create_node('sensor_graph_fixture')
    executor=SingleThreadedExecutor()
    executor.add_node(aggregator);executor.add_node(source)
    pub_vib=source.create_publisher(Vibration,'/mavros/vibration/raw/vibration',qos_profile_sensor_data)
    pub_mag=source.create_publisher(MagneticField,'/mavros/imu/mag',qos_profile_sensor_data)
    vib=Vibration();vib.vibration.x=2.;vib.vibration.y=3.;vib.vibration.z=4.;vib.clipping=[0.,1.,2.]
    mag=MagneticField();mag.header.frame_id='base_link';mag.header.stamp.sec=12
    mag.magnetic_field.x=30e-6;mag.magnetic_field.y=40e-6
    try:
        until=time.monotonic()+3
        while time.monotonic()<until and not {'magnetic','vibration'}.issubset(aggregator._seen):
            pub_vib.publish(vib);pub_mag.publish(mag)
            executor.spin_once(timeout_sec=.05)
        d=aggregator.state['sensor_diagnostics']
        assert d['vibration_mps2']==[2.,3.,4.]
        assert d['clipping']==[0.,1.,2.]
        assert d['magnetic_strength_ut']==pytest.approx(50.)
        assert d['magnetic_capture_stamp']==12.
    finally:
        executor.shutdown();source.destroy_node();aggregator.destroy_node()


@pytest.fixture
def context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()

def args():
    return SimpleNamespace(fcu_udp=0,system_id=1,camera_joint='webcam_pitch_joint',winch_joint='winch_joint',
        camera_channel=9,winch_channel=10,camera_pwm_min=1000,camera_pwm_max=2000,
        camera_min_deg=-90,camera_max_deg=0,winch_neutral=1500,winch_rate=1,winch_length=6)

def request(node,cid,task,params=None):
    node.request(String(data=json.dumps({'command_id':cid,'cmd':'start','args':{'task':task,'params':params or {}}})))

def test_every_task_constructs_production_components_without_arming(context):
    node=ExecutionNode()
    node.ready=True;node.rc_seen=time.monotonic();node.rc=SimpleNamespace(channels=[1500]*5)
    node.mav.connected=MagicMock(return_value=True)
    node.slam_enabled=True
    node.device_seen=time.monotonic()
    node.devices={k:{'ok':True} for k in ['identity','camera','lidar','clock','camera_actuator','winch_actuator']}
    node.mav.arm=MagicMock();node.mav.takeoff=MagicMock()
    node.sensor_seen={'camera':time.monotonic(),'lidar':time.monotonic()}
    node.rc_config={k:{'channel':i+1,'min':1000,'max':2000,'trim':1500} for i,k in enumerate(['forward','left','yaw','climb','takeover'])}
    node.rc_config['takeover']['active_above']=1700
    node.rc_config.update(max_speed=1,max_yaw_rate=.5,max_climb_rate=.5)
    try:
        for task in TASKS:
            node.sensor_seen={'camera':time.monotonic(),'lidar':time.monotonic()}
            node.rc_seen=time.monotonic()
            node.run=None
            request(node,'construct_'+task,task,dict(height=5,target='TARGET',action='stow',waypoints=[{'lat':10.,'lon':20.}]))
            assert node.run and node.run['state']=='Accepted', task
            if task=='mission':
                assert node.tree.root.name=='Root'
            node.release_control()
        node.mav.arm.assert_not_called();node.mav.takeoff.assert_not_called()
    finally:node.destroy_node()

def test_takeover_releases_both_commanders_without_competing_mode(context):
    node=ExecutionNode()
    try:
        node.run=dict(state='Running',task='banner',command_id='test_takeover',reason='')
        node.mav._sp=object();node.mav.set_mode=MagicMock();node.mav.land=MagicMock()
        node.takeover('transmitter')
        assert node.mav._sp is None and node.tree is None and node.run['control_owner']=='Transmitter'
        node.mav.set_mode.assert_not_called();node.mav.land.assert_not_called()
    finally:node.destroy_node()

def test_remote_camera_winch_capabilities_require_measured_joints_and_parameters(context):
    node=rclpy.create_node('gateway_capability_test');gateway=Gateway(node,args())
    try:
        gateway.actuator_seen=time.monotonic()
        assert not gateway.manifest()['camera_actuator']['ok']
        gateway.parameters={'SERVO9_FUNCTION':7,'SERVO10_FUNCTION':88,'MNT1_TYPE':1,'WINCH_TYPE':1}
        joints=JointState(name=['webcam_pitch_joint','winch_joint'],position=[-.5,1.])
        gateway.joints(joints)
        assert gateway.manifest()['camera_actuator']['ok']
        assert gateway.manifest()['winch_actuator']['ok']
        gateway.measured_camera=(0,time.monotonic()-2)
        assert not gateway.manifest()['camera_actuator']['ok']
    finally:gateway.udp.close();node.destroy_node()

def test_serialized_device_data_preserves_capture_time_and_rejects_clock_reset(context):
    node=rclpy.create_node('remote_driver_test');driver=RemoteDevices(node)
    source=rclpy.create_node('gateway_test');gateway=Gateway(source,args())
    driver.session=gateway.session
    try:
        clock=Clock();clock.clock.sec=100
        gateway.record('clock',clock);driver.deliver(pack('clock',driver.session,gateway.latest['clock']))
        scan=LaserScan();scan.header.stamp.sec=99;scan.header.stamp.nanosec=900000000
        scan.header.frame_id='base_scan';scan.ranges=[.71,2.,float('inf')]
        gateway.record('scan',scan)
        driver.pubs['scan']=MagicMock()
        driver.deliver(pack('scan',driver.session,gateway.latest['scan']))
        delivered=driver.pubs['scan'].publish.call_args.args[0]
        assert delivered.header.stamp==scan.header.stamp and list(delivered.ranges)==list(scan.ranges)
        image=Image();image.header=scan.header;image.height=16;image.width=16;image.encoding='bgr8';image.step=48;image.data=bytes([255,255,255]*256)
        gateway.camera(image);driver.pubs['camera']=MagicMock()
        driver.deliver(pack('camera',driver.session,gateway.latest['camera']))
        decoded=driver.pubs['camera'].publish.call_args.args[0]
        assert decoded.header.stamp==image.header.stamp and decoded.width==16 and decoded.height==16
        clock.clock.sec=1;gateway.record('clock',clock)
        with pytest.raises(ValueError):driver.deliver(pack('clock',driver.session,gateway.latest['clock']))
    finally:gateway.udp.close();source.destroy_node();node.destroy_node()

def test_remote_sensor_publishers_match_production_reliable_subscribers(context):
    node=rclpy.create_node('remote_qos_test');driver=RemoteDevices(node)
    consumer=rclpy.create_node('production_sensor_consumer')
    received=[]
    consumer.create_subscription(Image,'/image_raw',received.append,10)
    message=Image(height=1,width=1,encoding='mono8',step=1,data=b'a')
    try:
        until=time.monotonic()+2
        while not received and time.monotonic()<until:
            driver.pubs['camera'].publish(message)
            rclpy.spin_once(node,timeout_sec=.02)
            rclpy.spin_once(consumer,timeout_sec=.02)
        assert received and received[0].data==message.data
    finally:consumer.destroy_node();node.destroy_node()


@pytest.mark.asyncio
async def test_tcp_bridge_pins_fcu_identity_and_has_one_command_owner(context):
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    node=rclpy.create_node('mavlink_gateway_test');gateway=Gateway(node,args())
    sim=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sim.bind(('127.0.0.1',0));sim.setblocking(False)
    target=gateway.udp.getsockname();loop=asyncio.get_running_loop()
    task=asyncio.create_task(gateway.udp_loop())
    server=await asyncio.start_server(gateway.tcp,'127.0.0.1',0)
    port=server.sockets[0].getsockname()[1]
    encoder=mavlink.MAVLink(None,srcSystem=2,srcComponent=1)
    try:
        heartbeat=encoder.heartbeat_encode(2,3,0,0,3)
        await loop.sock_sendto(sim,heartbeat.pack(encoder),target)
        await asyncio.sleep(.02)
        assert gateway.fcu is None
        encoder.srcSystem=1
        await loop.sock_sendto(sim,heartbeat.pack(encoder),target)
        for _ in range(20):
            if gateway.fcu:break
            await asyncio.sleep(.01)
        assert gateway.identity==1
        reader,writer=await asyncio.open_connection('127.0.0.1',port)
        other_r,other_w=await asyncio.open_connection('127.0.0.1',port)
        assert await asyncio.wait_for(other_r.read(1),1)==b''
        command=encoder.command_long_encode(1,1,176,0,1,4,0,0,0,0,0).pack(encoder)
        writer.write(command);await writer.drain()
        data,_=await asyncio.wait_for(loop.sock_recvfrom(sim,65536),1)
        assert data==command
        writer.close();other_w.close();await writer.wait_closed();await other_w.wait_closed()
    finally:
        task.cancel();await asyncio.gather(task,return_exceptions=True)
        server.close();await server.wait_closed();sim.close();gateway.udp.close();node.destroy_node()

@pytest.mark.asyncio
async def test_idle_sensor_websocket_consumes_ping_pong_without_false_link_loss(context):
    from aiohttp import ClientSession, WSMsgType, web
    from aiohttp.test_utils import TestServer
    node=rclpy.create_node('heartbeat_gateway_test')
    gateway=Gateway(node,args())
    app=web.Application();app.router.add_get('/sensors',gateway.stream)
    server=TestServer(app);await server.start_server()
    try:
        async with ClientSession() as client:
            async with client.ws_connect(server.make_url('/sensors'),params={'session':gateway.session},heartbeat=.2) as ws:
                receiving=asyncio.create_task(ws.receive())
                await asyncio.sleep(1.2)
                assert not ws.closed
                clock=Clock();clock.clock.sec=12;gateway.record('clock',clock)
                message=await asyncio.wait_for(receiving,2)
                assert message.type==WSMsgType.BINARY
    finally:
        await server.close();gateway.udp.close();node.destroy_node()

@pytest.mark.asyncio
async def test_camera_overload_keeps_newest_capture_and_does_not_block_network_reader(context):
    from aiohttp import ClientSession, web
    from aiohttp.test_utils import TestServer
    node=rclpy.create_node('camera_inbox_test');driver=RemoteDevices(node)
    driver.session='fixture-session';published=[]
    def slow_delivery(packet):
        time.sleep(.08);published.append(int(packet))
    driver.deliver=slow_delivery
    async def feed(request):
        ws=web.WebSocketResponse(heartbeat=.2);await ws.prepare(request)
        async def drain():
            async for _ in ws:pass
        receiving=asyncio.create_task(drain())
        try:
            for i in range(20):
                await ws.send_bytes(str(i).encode());await asyncio.sleep(.01)
            await asyncio.sleep(.5)
            assert not ws.closed
            await ws.close()
        finally:
            receiving.cancel();await asyncio.gather(receiving,return_exceptions=True)
        return ws
    app=web.Application();app.router.add_get('/camera',feed)
    server=TestServer(app);await server.start_server();driver.url=str(server.make_url('')).rstrip('/')
    try:
        async with ClientSession() as client:
            # One connection: stream() itself reopens a dropped stream on the
            # same session (sim/test_remote_device_link.py).
            from aerothon_runtime.remote_devices import StreamDropped
            with pytest.raises(StreamDropped,match='stream closed'):
                await driver.stream_once(client,'/camera')
        assert published[-1]==19 and len(published)<10
    finally:await server.close();node.destroy_node()


def test_release_receipt_cannot_complete_physical_mechanism_test(context):
    import py_trees
    node=ExecutionNode()
    try:
        node.run=dict(state='Running',task='mechanism',command_id='test_release',params={'action':'release'})
        node.mav.winch_status={'state':'RELEASED','release_requested':True,'released':None}
        node.tree=py_trees.trees.BehaviourTree(py_trees.behaviours.Success('Release receipt'))
        node.tick()
        assert node.run['state']=='Outcome unknown'
        assert 'no confirming feedback' in node.run['reason']
    finally:node.destroy_node()
