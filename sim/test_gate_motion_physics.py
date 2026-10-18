"""Actual gate leaf, streamed lidar guard, and normal actuator physics."""
import json,math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import py_trees
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import LaserScan
from test_headless_land_position_hold import world
from headless_world import Arena,DT,WindDrift,W,LidarCorruptor
from mission_bt.mav_commander import Mav
from mission_bt.gate_braking import BrakedGateAdvance
ROOT=Path(__file__).resolve().parents[1]


def test_recorded_gate_entry_finishes_without_contact_or_clearance_loss():
    f=json.loads((ROOT/'sim/fixtures/gate_entry_mild_s3_20261008.json').read_text())
    replay(f)


def test_recorded_braking_retains_clearance_through_estimator_wander():
    f=json.loads((ROOT/'sim/fixtures/gate_brake_mild_s3_20261008.json').read_text())
    replay(f, brake_hold=(15.701924400676067,-15.18325590426256,f['flight_alt']))


def replay(f, brake_hold=None):
    w=world();w.t=f['epoch'];epoch=w.t;w.p=np.array(f['position']);w.yaw=f['yaw']
    wind=f['conditions']['wind'];w.arena=Arena(W.load(ROOT/'sim/worlds/my_world.json'),'b')
    w.wind=WindDrift(mean=wind['speed'],gust=wind['gust'],period=wind['gust_period_s'],dir_rad=math.radians(wind['dir_deg']),turb=.5*wind['veer_deg']/20*wind['speed']/8,seed=3,dt=DT)
    for i in range(round(epoch/DT)):w.wind(i*DT)
    w.ground_velocity=np.array(f['ground_velocity']);w.v=w.ground_velocity.copy();w.v[:2]-=np.array(w.wind(epoch));w.speed_cap=5.
    errors=np.array(f['errors']);mav=Mav.__new__(Mav)
    mav.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(w.t*1e9),to_msg=w.stamp)))
    mav._sp=None;mav._scan=None;mav._scan_t=None;mav.keepout_m=1.;mav.keepout_events=0;mav._keepout_on=False
    mav.pose=PoseStamped();mav.delivery_inputs={};mav.speed_cmd=5.;mav.exclusions=[];mav._avoiding=False
    mav.enable_avoidance=lambda *args,**kwargs:None;mav.log=lambda *args,**kwargs:None
    mav._track_altitude_error=lambda:None;mav.state=NS(armed=True);mav.setpoints_suppressed=0
    def publish(message):
        p=message.pose.position;q=message.pose.orientation
        w.sp_pos=(p.x,p.y,p.z,math.atan2(2*q.w*q.z,1-2*q.z*q.z));w.sp_vel=None
    mav.pub_sp=NS(publish=publish)
    def speed_request(speed):
        mav.speed_cmd=w.speed_cap=speed
        return NS(done=lambda:True,result=lambda:NS(success=True))
    mav.set_speed=speed_request
    lidar=LidarCorruptor(f['conditions']['lidar'],seed=4)
    leaf=BrakedGateAdvance(mav,tol=.25,gate_speed_mps=.35,restore_speed_mps=5.)
    leaf.initialise();leaf._target=tuple(f['target']);leaf._flight_alt=f['flight_alt'];leaf._flight_distance=f['flight_distance']
    if brake_hold is not None:
        leaf._phase='BRAKE';leaf._brake_hold=brake_hold;leaf._brake_yaw=w.yaw;w.speed_cap=.35
    minimum=float('inf');status=py_trees.common.Status.RUNNING;next_pose=next_scan=next_stream=next_tick=epoch
    for _ in range(6201):
        error=np.array([np.interp(w.t,errors[:,0],errors[:,k+1]) for k in range(3)])
        w.gps_err=error[:2];w.baro_err=error[2];est=w.est()
        if w.t>=next_pose:
            mav.pose.pose.position.x,mav.pose.pose.position.y,mav.pose.pose.position.z=map(float,est)
            mav.pose.pose.orientation.z=math.sin(w.yaw/2);mav.pose.pose.orientation.w=math.cos(w.yaw/2)
            mav.delivery_inputs['pose_stamp']=w.t
            msg=TwistStamped();msg.header.stamp=w.stamp();msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z=map(float,w.ground_velocity)
            mav._delivery_velocity(msg);next_pose=w.t+.03-1e-9
        if w.t>=next_scan:
            ranges=w.arena.raycast(*w.p[:2],w.p[2]+.08,w.yaw)
            scan=LaserScan();scan.header.stamp=w.stamp();scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/len(ranges);scan.range_min=.05;scan.range_max=12.;scan.ranges=list(map(float,lidar.apply(ranges,.05,12.)))
            mav._on_scan(scan);next_scan=w.t+.1-1e-9
        if w.t>=next_tick:
            status=leaf.update();next_tick=w.t+.2-1e-9
            if status is not py_trees.common.Status.RUNNING:break
        if w.t>=next_stream:mav._stream();next_stream=w.t+.1-1e-9
        w.t+=DT;w._step_airframe()
        minimum=min(minimum,min(W.distance_to_edge(w.p[:2],poly) for poly,z0,z1,kind in w.arena.boxes if kind in ['wall','obstacle'] and z0<=w.p[2]<=z1))
    print(dict(status=str(status),phase=leaf._phase,seconds=w.t-epoch,minimum_clearance_m=minimum,contacts=w.contacts))
    assert status is py_trees.common.Status.SUCCESS
    assert not w.contacts and minimum>=.70
    assert w.args.vel_response_s==.6 and w.args.vel_command_delay_s==0.
