"""Braking must validate the current streamed target, including temporary holds."""
from types import SimpleNamespace
from test_search_braking_20261007 import make_stage as original_make_stage
from mission_bt.mav_commander import Mav

def make_stage():
 # Completed stationary holds at x=9..9.1 must be outside the unchanged
 # 1.50 m margin. The original x=10 edge put those stops inside it.
 # Requested travel to x=30 still crosses the observed red obstacle.
 stage,mav,now=original_make_stage()
 mav.exclusions=[(11.,20.,-5.,5.)]
 return stage,mav,now

def wire_real_command(mav):
 commander=object.__new__(Mav);commander._sp=None
 mav.position_target_xy=lambda:Mav.position_target_xy(commander)
 return commander

def test_replaced_search_goal_does_not_interrupt_current_stationary_hold():
 stage,mav,_=make_stage();commander=wire_real_command(mav)
 Mav.goto(commander,*mav.pos(),mav.yaw())
 assert not stage._search_braking_gate.tick(stage)
 assert stage._search_braking_gate.hold is None

def test_current_unsafe_command_is_checked_even_when_cached_goal_is_safe():
 stage,mav,_=make_stage();commander=wire_real_command(mav)
 stage._streamed_search_goal=(0.,0.)
 Mav.goto(commander,30.,0.,10.)
 assert stage._search_braking_gate.tick(stage)
 assert mav.gotos[-1][:3]==mav.pos()

def test_completed_margin_hold_hands_control_back_to_certified_router():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto
 mav._pos=(9.,0.,10.)
 Mav.goto(commander,30.,0.,10.)
 gate=stage._search_braking_gate
 assert gate.tick(stage)
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp
  mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  assert gate.tick(stage)
 assert gate.hold is None
 assert not gate.tick(stage), 'completed braking hold must allow a fresh certified escape'
 # The handoff does not exempt a replacement travel command.
 Mav.goto(commander,30.,0.,10.)
 assert gate.tick(stage)
 assert gate.hold is not None

def test_real_leaf_issues_safe_escape_after_completed_margin_braking():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.)
 Mav.goto(commander,30.,0.,10.)
 stage.update()
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  stage.update()
 stage.update()
 goal=mav.position_target_xy()
 assert goal != (9.,0.), 'leaf must command escape, not repeat the stationary hold'
 assert stage.router._segment_safe(mav,goal)
 assert not stage._search_braking_gate.tick(stage)

def test_completed_hold_does_not_release_after_motion_evidence_becomes_stale():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.)
 Mav.goto(commander,30.,0.,10.)
 gate=stage._search_braking_gate;assert gate.tick(stage)
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  assert gate.tick(stage)
 now[0]=2.
 assert gate.tick(stage)
 assert mav.position_target_xy()==(9.,0.)
 assert gate.completed_hold==(9.,0.)

def test_settled_braking_allows_turn_in_place_until_escape_heading_is_reached():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.);stage.crab=False
 stage._turn_hold=(9.05,0.);stage._turn_best=3.141592653589793;stage._turn_stall=0
 Mav.goto(commander,30.,0.,10.)
 stage.update()
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  stage.update()
 for _ in range(8):
  stage.update()
  assert stage._search_braking_gate.hold is None, 'a stationary turn must not restart braking'
  assert mav.position_target_xy()==mav.pos()[:2], 'old turn hold must not pull toward newly observed paint'
 stage._yaw=lambda target:0.
 stage.update()
 assert mav.position_target_xy()!=mav.pos()[:2]
 assert stage.router._segment_safe(mav,mav.position_target_xy())

def test_turn_hold_uses_measured_stop_not_pre_braking_target():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.);stage.crab=False
 Mav.goto(commander,30.,0.,10.);stage.update()
 mav._pos=(9.1,0.,10.)
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  stage.update()
 stopped=mav.pos()[:2]
 for _ in range(8):
  mav._pos=(mav._pos[0]+.001,0.,10.)
  stage.update()
  assert stage._search_braking_gate.hold is None, 'normal pose updates cannot retrigger braking during the turn'
  assert mav.position_target_xy()==stopped

def test_speed_acknowledgement_wait_keeps_the_measured_braking_anchor():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.)
 Mav.goto(commander,30.,0.,10.);stage.update();mav._pos=(9.1,0.,10.)
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  stage.update()
 stopped=mav.pos()[:2]
 stage.search_speed_mps=2.5
 mav.detector_timing={name:SimpleNamespace(qualified=lambda now:dict(hz=9.,latency_s=.14,capture_stamp=now,samples=40)) for name in ('qr','red')}
 mav.set_speed=lambda speed:SimpleNamespace(done=lambda:False)
 for _ in range(8):
  mav._pos=(mav._pos[0]+.001,0.,10.)
  stage.update()
  assert stage._search_braking_gate.hold is None
  assert mav.position_target_xy()==stopped, 'waiting for speed ACK must not chase each pose observation'

def test_motion_fluctuation_keeps_completed_hold_without_chasing_new_pose():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.);stage.crab=False
 Mav.goto(commander,30.,0.,10.);stage.update()
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  stage.update()
 stopped=mav.pos()[:2];mav._pos=(9.1,0.,10.)
 mav.delivery_inputs['speed']=.2
 stage.update()
 assert mav.position_target_xy()==stopped, 'motion during a stationary hold must not chase a fresh noisy pose'
 assert stage._search_braking_gate.hold is None
 assert stage._search_braking_gate.completed_hold==stopped

def wire_real_velocity_callback(mav,commander):
 from collections import deque
 commander.delivery_inputs=mav.delivery_inputs
 commander.delivery_velocity_samples=deque(maxlen=40)
 mav.delivery_velocity_samples=commander.delivery_velocity_samples
 def sample(stamp,speed):
  from geometry_msgs.msg import TwistStamped
  msg=TwistStamped();msg.header.stamp.sec=int(stamp)
  msg.header.stamp.nanosec=round((stamp-int(stamp))*1.e9)
  msg.twist.linear.x=speed
  Mav._delivery_velocity(commander,msg)
 return sample


def test_three_fresh_fcu_samples_can_settle_between_mission_ticks():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 sample=wire_real_velocity_callback(mav,commander)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;Mav.goto(commander,30.,0.,10.)
 stage.update()
 for stamp in (1.03,1.06,1.09):sample(stamp,.05)
 now[0]=1.10;mav.delivery_inputs['pose_stamp']=1.09
 stage.update()
 assert stage._replans==1, 'three distinct delivered FCU samples must not be reduced to one leaf snapshot'
 assert stage._search_braking_gate.hold is None


def test_fcu_history_preserves_intervening_motion_and_pre_brake_epoch():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 sample=wire_real_velocity_callback(mav,commander)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;Mav.goto(commander,30.,0.,10.)
 for stamp in (.91,.94,.97):sample(stamp,.02)
 stage.update()
 for stamp,speed in ((1.03,.02),(1.06,.2),(1.09,.02),(1.12,.02)):sample(stamp,speed)
 now[0]=1.13;mav.delivery_inputs['pose_stamp']=1.12
 stage.update()
 assert stage._replans==0 and stage._search_braking_gate.hold is not None
 sample(1.15,.02);now[0]=1.16;mav.delivery_inputs['pose_stamp']=1.15
 stage.update()
 assert stage._replans==1

def test_motion_between_leaf_ticks_resets_the_fcu_settled_streak():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 sample=wire_real_velocity_callback(mav,commander)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;Mav.goto(commander,30.,0.,10.);stage.update()
 for moving,settled in ((1.03,1.06),(1.09,1.12),(1.15,1.18)):
  sample(moving,.2);sample(settled,.02)
  now[0]=settled+.01;mav.delivery_inputs['pose_stamp']=settled
  stage.update()
  assert stage._replans==0, 'intervening motion cannot disappear between leaf snapshots'
 for stamp in (1.21,1.24):sample(stamp,.02)
 now[0]=1.25;mav.delivery_inputs['pose_stamp']=1.24
 stage.update()
 assert stage._replans==1


def inside_margin_hold():
 stage,mav,now=make_stage();commander=wire_real_command(mav)
 original=mav.goto
 def goto(*args):
  original(*args);Mav.goto(commander,*args)
 mav.goto=goto;mav._pos=(9.,0.,10.)
 mav.exclusions=[(10.,20.,-5.,5.)]
 Mav.goto(commander,30.,0.,10.)
 gate=stage._search_braking_gate;assert gate.tick(stage)
 return stage,mav,now,gate


def test_stopped_speed_inside_original_margin_cannot_finish_current_command_hold():
 stage,mav,now,gate=inside_margin_hold()
 for stamp in (1.1,1.2,1.3):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  assert gate.tick(stage)
 assert gate.escape_active and gate.completed_hold is None
 assert gate.hold is not None and mav.position_target_xy()!=(9.,0.)
 assert stage.router._segment_safe(mav,mav.position_target_xy())
 assert 10.-mav.position_target_xy()[0]>=stage.router.clearance_m
 assert stage.router.clearance_m==1.5


def test_actual_escape_arrival_needs_distinct_captures_before_stationary_handoff():
 stage,mav,now,gate=inside_margin_hold()
 now[0]=1.1;mav.delivery_inputs.update(pose_stamp=1.1,velocity_stamp=1.1,speed=.02)
 assert gate.tick(stage)
 escape=mav.position_target_xy();mav._pos=(*escape,10.)
 now[0]=1.2;mav.delivery_inputs.update(pose_stamp=1.2,velocity_stamp=1.2,speed=.02)
 for _ in range(5):assert gate.tick(stage)
 assert gate.hold is not None and gate.completed_hold is None
 for stamp in (1.3,1.4):
  now[0]=stamp;mav.delivery_inputs.update(pose_stamp=stamp,velocity_stamp=stamp,speed=.02)
  assert gate.tick(stage)
 assert gate.hold is None and gate.completed_hold==escape
 assert not gate.tick(stage)
