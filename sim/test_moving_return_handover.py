"""Moving focused handover must preserve observed entry motion and real gate semantics.

These are driver seam checks, not flight or sensor qualification.
"""
import math
from types import SimpleNamespace as NS
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,TwistStamped
import pytest
from corridor_trial import MovingGateHandover, argument_parser
from mission_bt.mission_tree import GateAdvance
from test_leg_routing import RoutedMav


def header(t,frame):
    sec=int(t)
    return Header(stamp=Time(sec=sec,nanosec=round((t-sec)*1e9)),frame_id=frame)


def measured(mav,t,linear=(.31,.07,-.003)):
    x,y,z=mav.pos();yaw=mav.yaw()
    mav.pose=PoseStamped(header=header(t,'map'))
    mav.pose.pose.position.x=float(x);mav.pose.pose.position.y=float(y);mav.pose.pose.position.z=float(z)
    mav.pose.pose.orientation.z=math.sin(yaw/2);mav.pose.pose.orientation.w=math.cos(yaw/2)
    velocity=TwistStamped(header=header(t,'base_link'))
    velocity.twist.linear.x=float(linear[0]);velocity.twist.linear.y=float(linear[1]);velocity.twist.linear.z=float(linear[2])
    return velocity


def crossing():
    mav=RoutedMav(at=(0.,0.,1.65));mav.state=NS(connected=True,armed=True)
    calls=[];mav.enable_avoidance=lambda on,hold_alt=None:calls.append((on,hold_alt))
    entry=dict(x=5.4,y=0.,alt=1.65,yaw=0.,approach=dict(x=0.,y=0.),captured_stamp=868.408)
    gate=MovingGateHandover(mav,entry)
    gate.on_velocity(measured(mav,10.),10.)
    return mav,gate,calls


def test_default_remains_stationary_and_cli_selects_moving(monkeypatch):
    monkeypatch.delenv('AEROTHON_CORRIDOR_HANDOVER',raising=False)
    assert argument_parser().parse_args(['--entry','entry.json','--out','out']).handover_mode=='stationary'
    assert argument_parser().parse_args(['--entry','entry.json','--out','out','--handover-mode','moving']).handover_mode=='moving'


def test_real_gate_leaf_hands_over_on_arrival_tick_without_three_second_hold():
    mav,gate,calls=crossing()
    assert gate.update(10.)['state']=='CROSSING'
    assert isinstance(gate.leaf,GateAdvance)
    assert all(on is False for on,_ in calls)
    mav._pos=(5.16,0.,1.65);mav._alt=1.65
    velocity=measured(mav,10.15)
    gate.on_velocity(velocity,10.2)
    result=gate.update(10.2)
    assert result['state']=='HANDED_OVER' and result['transition_stamp']==10.2
    assert calls[-1]==(True,1.65)
    assert result['observation']['velocity']['capture_stamp']==10.15
    assert result['observation']['velocity']['linear']==[.31,.07,-.003]
    assert velocity.twist.linear.x==.31
    assert result['arrival_tolerance_m']==.25


def test_environment_selects_moving_and_rejects_unknown_mode(monkeypatch):
    monkeypatch.setenv('AEROTHON_CORRIDOR_HANDOVER','moving')
    assert argument_parser().parse_args(['--entry','entry.json','--out','out']).handover_mode=='moving'
    monkeypatch.setenv('AEROTHON_CORRIDOR_HANDOVER','unknown')
    with pytest.raises(SystemExit):argument_parser()


def test_gate_does_not_handover_before_production_arrival_tolerance():
    mav,gate,calls=crossing();gate.update(10.)
    mav._pos=(5.14,0.,1.65);mav._alt=1.65
    gate.on_velocity(measured(mav,10.2),10.2)
    assert gate.update(10.2)['state']=='CROSSING'
    assert all(on is False for on,_ in calls)


@pytest.mark.parametrize('kind',['stale_pose','stale_velocity','future_pose','nonfinite_velocity','disconnected'])
def test_bad_observations_cannot_command_gate_or_handover(kind):
    mav,gate,calls=crossing()
    if kind=='stale_pose':mav.pose.header=header(9.2,'map')
    if kind=='stale_velocity':gate.velocity.header=header(9.2,'base_link')
    if kind=='future_pose':mav.pose.header=header(10.01,'map')
    if kind=='nonfinite_velocity':gate.velocity.twist.linear.x=float('nan')
    if kind=='disconnected':mav.state.connected=False
    assert gate.update(10.)['state']=='FAILED'
    assert mav.gotos==[] and calls==[]


def test_duplicate_velocity_does_not_refresh_captured_or_received_time():
    mav,gate,calls=crossing()
    original=gate.velocity
    gate.on_velocity(measured(mav,10.,linear=(0.,0.,0.)),10.6)
    assert gate.velocity is original and gate.velocity_received==10.
    measured(mav,10.71)
    assert gate.update(10.71)['state']=='FAILED'
    assert mav.gotos==[] and calls==[]


def test_gate_latches_its_target_and_uses_position_commands_before_handover():
    mav,gate,calls=crossing();gate.update(10.)
    first=gate.leaf._target
    assert mav.gotos[-1][:3]==(5.4,0.,1.65)
    mav._pos=(1.,0.,1.65);mav._alt=1.65
    gate.on_velocity(measured(mav,10.2),10.2);gate.update(10.2)
    assert gate.leaf._target==first
    assert all(on is False for on,_ in calls)
