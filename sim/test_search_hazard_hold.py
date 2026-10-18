"""New red observations must invalidate a latched stationary search anchor."""
import pytest
from mission_bt.mission_tree import LawnmowerSearch
from test_leg_routing import RoutedMav

def test_braking_anchor_inside_new_exclusion_must_take_certified_escape():
 mav=RoutedMav(at=(49.20995036686573,-18.33730770584774,9.878185873982721))
 now=273.1;mav.delivery_inputs=dict(pose_stamp=273.,velocity_stamp=273.,speed=.134)
 stage=LawnmowerSearch(mav,(17.,57.,-21.,10.),10.,exclusions=lambda:mav.exclusions,clock=lambda:now)
 stage.initialise();gate=stage._search_braking_gate
 anchor=(49.32662271657171,-18.223619929631525,10.,0.)
 gate.hold=anchor;gate.brake_started_stamp=270.
 mav.exclusions=[(47.,50.,-19.,-16.)]
 assert gate.tick(stage)
 assert mav.gotos[-1][:2]!=anchor[:2],'a newly unsafe stationary anchor cannot stay exempt from route checks'
 assert stage.router._segment_safe(mav,mav.gotos[-1][:2],exclusions=mav.exclusions)
 assert gate.completed_hold is None

def setup_hazard_hold():
 mav=RoutedMav(at=(1.9,-2.,10.));now=[1.]
 mav.delivery_inputs=dict(pose_stamp=1.,velocity_stamp=1.,speed=.13)
 stage=LawnmowerSearch(mav,(-10.,20.,-10.,10.),10.,exclusions=lambda:mav.exclusions,clock=lambda:now[0])
 stage.initialise();gate=stage._search_braking_gate
 gate.hold=(*mav.pos(),mav.yaw());gate.brake_started_stamp=.8
 mav.exclusions=[(2.,3.,-1.,1.)]
 return stage,mav,now,gate

@pytest.mark.parametrize('kind',['missing','stale','future','nonfinite'])
def test_unknown_motion_cannot_authorize_an_escape(kind):
 stage,mav,now,gate=setup_hazard_hold();anchor=gate.hold
 if kind=='missing':del mav.delivery_inputs['velocity_stamp']
 elif kind=='stale':now[0]=2.
 elif kind=='future':mav.delivery_inputs['velocity_stamp']=2.
 else:mav.delivery_inputs['speed']=float('nan')
 assert gate.tick(stage)
 assert mav.gotos[-1]==anchor

def test_low_speed_inside_margin_cannot_complete_escape():
 stage,mav,now,gate=setup_hazard_hold();gate.tick(stage);escape=mav.gotos[-1]
 for t in (1.1,1.2,1.3,1.4):
  now[0]=t;mav.delivery_inputs.update(pose_stamp=t,velocity_stamp=t,speed=.02);gate.tick(stage)
 assert gate.hold==escape and gate.escape_active and gate.completed_hold is None

def test_completed_stationary_anchor_is_revalidated_after_new_observation():
 stage,mav,now,gate=setup_hazard_hold();gate.completed_hold=gate.hold[:2];gate.hold=None
 mav.delivery_inputs['speed']=.02
 assert gate.tick(stage)
 assert gate.completed_hold is None and gate.escape_active
 assert stage.router._segment_safe(mav,mav.gotos[-1][:2],exclusions=mav.exclusions)

def test_certified_escape_does_not_chase_cross_track_pose_updates():
 stage,mav,now,gate=setup_hazard_hold();gate.tick(stage);escape=gate.hold
 mav._pos=(1.92,-2.1,10.)
 now[0]=1.1;mav.delivery_inputs.update(pose_stamp=1.1,velocity_stamp=1.1,speed=.13)
 gate.tick(stage)
 assert gate.hold==escape

def test_new_paint_revalidates_an_in_progress_escape_segment():
 stage,mav,now,gate=setup_hazard_hold();gate.tick(stage);escape=gate.hold
 mav._pos=(1.9,-2.1,10.)
 mav.exclusions.append((1.6,2.2,-2.7,-2.5))
 now[0]=1.1;mav.delivery_inputs.update(pose_stamp=1.1,velocity_stamp=1.1,speed=.13)
 gate.tick(stage)
 assert gate.hold!=escape
 assert gate.hold[:3]==mav.pos() and gate.escape_blocked
 for t in (1.2,1.3,1.4):
  now[0]=t;mav.delivery_inputs.update(pose_stamp=t,velocity_stamp=t,speed=.02);gate.tick(stage)
 assert gate.completed_hold is None and gate.escape_blocked

def test_escape_handoff_requires_distinct_stopped_captures_outside_margin():
 stage,mav,now,gate=setup_hazard_hold();gate.tick(stage)
 mav._pos=gate.hold[:3]
 now[0]=1.1;mav.delivery_inputs.update(pose_stamp=1.1,velocity_stamp=1.1,speed=.02)
 for _ in range(5):gate.tick(stage)
 assert gate.completed_hold is None and gate.escape_active
 for t in (1.2,1.3):
  now[0]=t;mav.delivery_inputs.update(pose_stamp=t,velocity_stamp=t,speed=.02);gate.tick(stage)
 assert gate.hold is None and not gate.escape_active and gate.completed_hold==mav.pos()[:2]
