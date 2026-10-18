#!/usr/bin/env python3
"""Focused rendered return traversal from an entry recorded by the FCU.

The entry fixture is an operator-specified starting condition. The production
avoidance node decides all corridor movement using lidar, pose and onboard TF.
No world polygons, contacts or simulator positions are read by this driver.
This does not qualify the full mission, perception search or delivery.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_mission/mission_bt'))


def argument_parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--entry',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--wall-timeout',type=float,default=1800)
    default=os.environ.get('AEROTHON_CORRIDOR_HANDOVER','stationary')
    if default not in ('stationary','moving'):p.error('invalid AEROTHON_CORRIDOR_HANDOVER')
    p.add_argument('--handover-mode',choices=('stationary','moving'),
                   default=default)
    return p


class MovingGateHandover:
    """Command the real gate leaf, then enable avoidance on its arrival tick."""
    max_age_s=.7

    def __init__(self,mav,entry):
        self.mav,self.entry=mav,entry
        dx=float(entry['x'])-float(entry['approach']['x'])
        dy=float(entry['y'])-float(entry['approach']['y'])
        if not all(math.isfinite(v) for v in (dx,dy)) or math.hypot(dx,dy)<=.25:
            raise ValueError('moving fixture requires a finite gate approach')
        self.setup_yaw=math.atan2(dy,dx)
        self.velocity=None;self.velocity_received=None;self.leaf=None

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def on_velocity(self,message,received_stamp):
        if self.velocity is not None and self.stamp(message)<=self.stamp(self.velocity):return
        self.velocity,self.velocity_received=message,received_stamp

    def observation(self,now):
        if self.velocity is None:return None
        pose=self.mav.pose;velocity=self.velocity
        stamps=(self.stamp(pose),self.stamp(velocity),self.velocity_received)
        p,q=pose.pose.position,pose.pose.orientation
        v,w=velocity.twist.linear,velocity.twist.angular
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z)
        if (not all(math.isfinite(value) for value in (*stamps,*values)) or
                any(stamp<=0 or not 0<=now-stamp<=self.max_age_s for stamp in stamps) or
                q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-12):return None
        return dict(pose=dict(capture_stamp=stamps[0],frame_id=pose.header.frame_id,
            position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]),
            velocity=dict(capture_stamp=stamps[1],received_stamp=stamps[2],
                frame_id=velocity.header.frame_id,linear=[v.x,v.y,v.z],angular=[w.x,w.y,w.z]),
            horizontal_speed_mps=math.hypot(v.x,v.y),max_age_s=self.max_age_s)

    def update(self,now):
        from mission_bt.mission_tree import GateAdvance
        import py_trees
        measured=self.observation(now)
        if not self.mav.state.connected or measured is None:
            return dict(state='FAILED',reason='moving handover lacks fresh finite FCU pose/velocity')
        if self.leaf is None:
            x,y,_=self.mav.pos()
            distance=math.hypot(float(self.entry['x'])-x,float(self.entry['y'])-y)
            self.leaf=GateAdvance(self.mav,advance_m=distance,alt=float(self.entry['alt']),
                tol=.25,exclusions=lambda:self.mav.exclusions)
            self.leaf.initialise()
        outcome=self.leaf.update()
        if outcome is py_trees.common.Status.FAILURE:
            return dict(state='FAILED',reason=self.leaf.feedback_message or self.mav.abort_reason)
        if outcome is py_trees.common.Status.SUCCESS:
            self.mav.enable_avoidance(True,hold_alt=float(self.entry['alt']))
            return dict(state='HANDED_OVER',transition_stamp=now,observation=measured,
                gate_target_xy=list(self.leaf._target),arrival_tolerance_m=self.leaf.tol,
                fixture_capture_stamp=self.entry.get('captured_stamp'),
                fixture_source=self.entry.get('source'),fixture_source_sha256=self.entry.get('source_sha256'))
        return dict(state='CROSSING')


class FcuDisarmEvidence:
    """A missing heartbeat cannot establish a completed landing."""
    max_age_s=1.5  # MAVROS state is delivered at 1 Hz.

    def __init__(self):
        self.state=None;self.received=None;self.seen_armed=False

    def on_state(self,message,now):
        self.state,self.received=message,now
        if self.fresh(now) and message.armed:self.seen_armed=True

    def fresh(self,now):
        if self.state is None or not self.state.connected:return False
        stamps=(MovingGateHandover.stamp(self.state),self.received)
        return all(math.isfinite(t) and t>0 and 0<=now-t<=self.max_age_s for t in stamps)

    def disarm(self,now,land_started):
        if (land_started is None or not self.seen_armed or not self.fresh(now)
                or self.state.armed or MovingGateHandover.stamp(self.state)<land_started):return None
        return dict(capture_stamp=MovingGateHandover.stamp(self.state),
            received_stamp=self.received,decision_stamp=now,connected=True,armed=False,
            max_age_s=self.max_age_s)


class LandingContract:
    """Declare the land-in-place this trial commands, before it touches down.

    The trial lands where the return lane ends instead of flying home, so the
    independent contact grader cannot read a home landing as the only
    legitimate skid contact. The anchor is the onboard FCU local position at
    the moment LAND is commanded: no world geometry, simulator truth or
    post-landing position enters it. It is written once; a stale or
    non-finite pose records the failure instead of an anchor, which leaves
    the landing contacts unexplained rather than quietly exempt.
    """
    max_age_s=.7
    radius_m=1.5          # the commanded descent's own drift, not a position check

    def __init__(self,mav,path):
        self.mav,self.path=mav,path

    def record(self,now):
        if self.path.exists():return
        pose=self.mav.pose;capture=MovingGateHandover.stamp(pose)
        p=pose.pose.position
        fresh=(all(math.isfinite(v) for v in (capture,now,p.x,p.y,p.z)) and capture>0
               and 0<=now-capture<=self.max_age_s)
        contract=dict(commanded_stamp=now,frame='home_local_enu',
            anchor=[p.x,p.y] if fresh else None,
            pose_capture_stamp=capture if fresh else None,
            pose_frame_id=pose.header.frame_id,max_age_s=self.max_age_s,
            radius_m=self.radius_m,
            source='onboard FCU local position at the commanded land-in-place',
            trial='focused return corridor; the operator lands where the lane ends')
        if not fresh:contract['failure']='no fresh finite FCU pose at the LAND command'
        with self.path.open('x') as stream:json.dump(contract,stream,indent=2)


def main():
    p=argument_parser()
    args=p.parse_args()
    entry=json.loads(args.entry.read_text())
    x,y,z,yaw=(float(entry[k]) for k in ('x','y','alt','yaw'))
    ax,ay=(float(entry['approach'][k]) for k in ('x','y'))
    if not all(math.isfinite(v) for v in (x,y,z,yaw)) or not 1<z<3:
        p.error('invalid recorded entry')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from mavros_msgs.msg import State
    from mission_bt.mav_commander import Mav
    rclpy.init()
    node=Node('rendered_corridor_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
    mav=Mav(node)
    landing=FcuDisarmEvidence()
    node.create_subscription(State,'/mavros/state',lambda msg:landing.on_state(
        msg,node.get_clock().now().nanoseconds*1e-9),10)
    contract=LandingContract(mav,args.out/'landing_contract.json')
    moving=MovingGateHandover(mav,entry) if args.handover_mode=='moving' else None
    setup_yaw=moving.setup_yaw if moving is not None else yaw
    if moving is not None:
        from geometry_msgs.msg import TwistStamped
        from rclpy.qos import qos_profile_sensor_data
        node.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',
            lambda msg:moving.on_velocity(msg,node.get_clock().now().nanoseconds*1e-9),qos_profile_sensor_data)
    stage_pub=node.create_publisher(String,'/mission/state',10)
    phase='WAIT';started=time.monotonic();phase_started=None;result=None;land_started=None
    last_tick=-1
    trace=(args.out/'trial_driver.jsonl').open('x')
    try:
        while time.monotonic()-started<args.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.05)
            now=node.get_clock().now().nanoseconds*1e-9
            if now-last_tick<.2:continue
            last_tick=now
            if phase_started is None:phase_started=now
            def advance(next_phase):
                nonlocal phase,phase_started,land_started
                phase,phase_started=next_phase,now
                if next_phase=='LAND' and land_started is None:
                    land_started=now;contract.record(now)
            if phase=='WAIT':
                if not mav.state.connected or mav.alt() is None or now-phase_started<15:continue
                mav.mission_started=True
                if mav.state.mode!='GUIDED':mav.set_mode('GUIDED');continue
                if not mav.state.armed:mav.arm();continue
                mav.takeoff(10);advance('TAKEOFF')
            elif phase=='TAKEOFF':
                if mav.alt()>9.6:advance('POSITION_ENTRY')
            elif phase=='POSITION_ENTRY':
                mav.goto(ax,ay,10,setup_yaw)
                if mav.reached(ax,ay,10,.3):advance('DESCEND_ENTRY')
            elif phase=='DESCEND_ENTRY':
                mav.goto(ax,ay,z,setup_yaw);mav.set_camera_pose('FORWARD')
                aligned=abs(math.atan2(math.sin(mav.yaw()-setup_yaw),math.cos(mav.yaw()-setup_yaw)))<.01
                if mav.reached(ax,ay,z,.12) and (moving is None or aligned):advance('CROSS_GATE')
            elif phase=='CROSS_GATE':
                if moving is not None:
                    handover=moving.update(now)
                    if handover['state']=='HANDED_OVER':
                        handover['handover_mode']=args.handover_mode
                        (args.out/'entry_handover.json').write_text(json.dumps(handover,indent=2)+'\n')
                        advance('RETURN_CORRIDOR')
                    elif handover['state']=='FAILED':
                        result=dict(state='TRIAL_FAILED',reason=handover['reason'],t=now)
                        mav.enable_avoidance(False);advance('LAND')
                else:
                    mav.goto(x,y,z,yaw)
                    if mav.reached(x,y,z,.12) and abs(math.atan2(math.sin(mav.yaw()-yaw),math.cos(mav.yaw()-yaw)))<.08:
                        advance('SETTLE_ENTRY')
            elif phase=='SETTLE_ENTRY':
                mav.goto(x,y,z,yaw)
                if now-phase_started>3:advance('RETURN_CORRIDOR')
            elif phase=='RETURN_CORRIDOR':
                mav.enable_avoidance(True,hold_alt=z)
                detail=mav.avoid_detail or {}
                if detail.get('corridor_exited') and detail.get('axis_progress_m',0)>3:
                    result=dict(state='TRIAL_COMPLETE',reason='observed far corridor exit',t=now)
                    mav.enable_avoidance(False);advance('LAND')
                elif mav.avoidance_stuck() or now-phase_started>150:
                    result=dict(state='TRIAL_FAILED',reason='return recovery exhausted or traversal timed out',t=now)
                    mav.enable_avoidance(False);advance('LAND')
            elif phase=='LAND':
                mav.land()
                if landing.disarm(now,land_started) is not None:break
            stage_pub.publish(String(data=phase))
            trace.write(json.dumps(dict(stamp=now,phase=phase,pose=mav.pos(),avoidance=mav.avoid_detail,
                handover_mode=args.handover_mode,entry_observation=moving.observation(now) if moving is not None else None))+'\n');trace.flush()
            if mav.abort_requested and phase not in ('WAIT','LAND'):
                result=dict(state='TRIAL_FAILED',reason=mav.abort_reason,t=now)
                mav.enable_avoidance(False);advance('LAND')
            if phase!='LAND' and now-phase_started>180:
                result=dict(state='TRIAL_FAILED',reason=phase+' timed out',t=now)
                mav.enable_avoidance(False);advance('LAND')
            if result is not None:
                mav.pub_result.publish(String(data=json.dumps(result)))
    finally:
        now=node.get_clock().now().nanoseconds*1e-9
        if landing.disarm(now,land_started) is None:
            if land_started is None:land_started=now
            contract.record(now)
            mav.enable_avoidance(False);mav.land()
            until=time.monotonic()+180
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.1)
                now=node.get_clock().now().nanoseconds*1e-9
                if landing.disarm(now,land_started) is not None:break
        result=result or dict(state='TRIAL_FAILED',reason='driver wall timeout')
        result['disarm_evidence']=landing.disarm(now,land_started)
        result['final_armed']=bool(landing.state.armed) if landing.fresh(now) else None
        if result['disarm_evidence'] is None:result['landing_failure']='fresh connected FCU disarm not observed'
        result['handover_mode']=args.handover_mode
        mav.pub_result.publish(String(data=json.dumps(result)))
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
        trace.close()
        (args.out/'trial_result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        node.destroy_node();rclpy.try_shutdown()
    return 0 if result['state']=='TRIAL_COMPLETE' and result['disarm_evidence'] is not None else 1


if __name__=='__main__':sys.exit(main())
