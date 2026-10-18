"""Timestamped FCU velocity must be compared with its own command-response time."""
import math
import numpy as np
from std_msgs.msg import Bool
from rclpy.parameter import Parameter
from test_corridor_clearance_contract import controller


def test_velocity_receipt_lag_does_not_invent_a_changing_disturbance(controller):
    c,_=controller
    c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    c._yaw=0.;c._pos=(0.,0.);c._pose_t=1.
    c._now=lambda:1.
    c._fcu_velocity=np.zeros(2);c._fcu_velocity_t=.95
    c._compensated(0.,0.)
    velocity=np.zeros(2)
    velocities={.95:velocity.copy(),1.:velocity.copy()}
    timeline=[]
    residual=[]
    for i in range(1,61):
        t=round(1.+i*.05,8)
        sent=np.array([.8 if i<25 else -.8 if i<45 else 0.,0.])
        c._command_history.append((t,sent.copy()));c._sent_heading[t]=0.
        timeline.append((t,sent.copy()))
        # The interval ending at t uses the packet present at its midpoint;
        # a packet arriving exactly at t has produced no motion before t.
        delivered=next((cmd for stamp,cmd in reversed(timeline) if stamp<=t-.025-.4+1e-9),np.zeros(2))
        velocity+=(delivered-velocity)*(.05/.6)
        velocities[t]=velocity.copy()
        sample_t=round(t-.05,8)
        c._now=lambda t=t:t
        c._fcu_velocity_t=sample_t;c._fcu_velocity=velocities[sample_t].copy()
        c._pose_t=t
        c._compensated(0.,0.)
        residual.append(float(np.linalg.norm(c._wind_world)))
    assert max(residual)<.015, 'FCU receipt lag was incorrectly interpreted as a changing load or wind'
    assert np.linalg.norm(c._wind_rate_world)<.02


def test_repeated_velocity_sample_does_not_advance_response_past_measurement(controller):
    c,_=controller;c._now=lambda:1.
    c._fcu_velocity=np.array([.2,0.]);c._fcu_velocity_t=.95
    c._compensated(0.,0.)
    c._command_history.append((.6,np.array([.8,0.])));c._sent_heading[.6]=0.
    expected=c._expect_world.copy()
    c._now=lambda:1.05;c._compensated(0.,0.)
    assert c._model_t==.95
    np.testing.assert_array_equal(c._expect_world,expected)


def test_handover_response_seed_uses_velocity_sample_stamp(controller):
    c,_=controller;c._now=lambda:1.
    c._fcu_velocity=np.array([.2,-.1]);c._fcu_velocity_t=.95
    c._on_enable(Bool(data=False));c._on_enable(Bool(data=True))
    assert c._model_t==.95
    np.testing.assert_allclose(c._expect_world,[.2,-.1])


def test_unknown_position_target_does_not_invent_wind_before_velocity_handover(controller):
    c,_=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    t=1.;c._now=lambda:t;c._pos=(0.,0.);c._pose_t=t;c._yaw=0.
    c._fcu_velocity=np.array([.3,0.]);c._fcu_velocity_t=t
    c._on_enable(Bool(data=False));c._on_enable(Bool(data=True))
    c._command_history.append((1.,np.zeros(2)));c._sent_heading[1.]=0.
    velocity=.3;residual=[]
    for i in range(1,13):
        t=round(1.+i*.05,8)
        if t>1.4+1e-9:velocity+=(0.-velocity)*(.05/.6)
        c._fcu_velocity_t=t;c._fcu_velocity=np.array([velocity,0.])
        c._pose_t=t;c._pos=(c._pos[0]+velocity*.05,0.)
        c._compensated(0.,0.);residual.append(float(np.linalg.norm(c._wind_world)))
    assert max(residual)<.015, 'An unobserved prior position target was treated as a zero velocity target'


def test_handover_diagnostics_do_not_report_unknown_response_as_measured_wind(controller):
    c,_=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    t=1.;c._now=lambda:t;c._fcu_velocity_t=t;c._fcu_velocity=np.array([.3,0.])
    c._on_enable(Bool(data=False));c._on_enable(Bool(data=True))
    c._publish(0.,0.,2.,0.,{})
    assert c.last_detail['disturbance_estimate_valid'] is False
    assert c.last_detail['disturbance_world_mps'] is None
    assert c.last_detail['disturbance_rate_world_mps2'] is None
    t=1.45;c._fcu_velocity_t=t;c._pose_t=t
    c._publish(0.,0.,2.,0.,{})
    assert c.last_detail['disturbance_estimate_valid'] is False
    t=1.5;c._fcu_velocity_t=t;c._pose_t=t
    c._publish(0.,0.,2.,0.,{})
    assert c.last_detail['disturbance_estimate_valid'] is True
    assert c.last_detail['required_clearance_m']==.7
