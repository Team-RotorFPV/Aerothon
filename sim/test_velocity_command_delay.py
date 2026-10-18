"""Delayed actuator response must not be mistaken for a wind disturbance."""
import math
import numpy as np
import pytest
from test_corridor_clearance_contract import controller
from rclpy.parameter import Parameter


def test_delayed_calm_response_does_not_create_large_wind_correction(controller):
    c,_ = controller
    c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    # Actual FCU response starts 0.4 s after the recorded command. This
    # minimises the rendered calm-run symptom without any simulator inputs.
    t = 1.
    c._now = lambda:t
    c._compensated(0,0)
    c._last_command_world = np.array([.8,0])
    if hasattr(c,'_command_history'):
        c._command_history.append((1.,np.array([.8,0])))
    x = 0.; velocity = 0.; correction = []
    for i in range(1,31):
        t = 1.+i*.05
        if t>1.4:
            velocity += (.8-velocity)*(1-math.exp(-.05/.6))
        x += velocity*.05
        c._pos = (x,0); c._pose_t=t
        c._last_command_world = np.array([.8,0])
        if hasattr(c,'_command_history'):
            c._command_history.append((t,np.array([.8,0])))
        command = c._compensated(.8,0)
        correction.append(command[0]-.8)
    assert max(abs(v) for v in correction)<.15
    assert abs(c._wind_world[0])<.05


def test_pending_command_is_included_in_stop_clearance(controller):
    from test_local_planner import wall_scan
    c,s = controller
    c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    c._planner.observe(*wall_scan(.8,1.75))
    # The vehicle is nearly stationary, but an already transmitted lateral
    # command has yet to take effect. Raw-zero cannot stop that delayed pulse.
    c._compensated = lambda vx,vy:(vx,vy)
    c._ground_world = np.zeros(2); c._wind_world = np.zeros(2)
    if hasattr(c,'_command_history'):
        c._command_history.append((.8,np.array([0,.8])))
    c._publish(0,0,2,0,{})
    assert c.last_detail.get('braking_clearance_available') is False
    assert 'braking' in c.last_detail.get('recovery_reason','')


def test_stale_scan_stop_preserves_commands_already_in_transit(controller):
    c,_=controller
    c._command_history.append((.8,np.array([0,.8])))
    c._publish(0,0,0,0,{'fault':'scan stale'})
    assert any(np.linalg.norm(command)>.7 for _,command in c._command_history)


def test_unknown_prior_target_cannot_shorten_handover_stopping_distance(controller):
    from test_local_planner import wall_scan
    c,sent=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    c._planner.observe(*wall_scan(.95,1.75))
    c._compensated=lambda vx,vy:(vx,vy)
    c._ground_world=np.array([0.,.3]);c._wind_world=np.zeros(2)
    # Until our first packet arrives, the prior position controller can still
    # maintain measured motion. Assuming a zero target would certify this
    # stop even though .4 s of incoming travel consumes the clearance reserve.
    c._publish(0.,0.,2.,0.,{})
    assert c.last_detail.get('hold_source')=='certified active disturbance braking'
    assert sent[-1].velocity.y<0
    velocity=.3;travel=0.;dt=.005
    for t in np.arange(0.,5.,dt):
        goal=.3 if t<.4 else sent[-1].velocity.y if t<1. else 0.
        change=(goal-velocity)*dt/.6
        if t>=1.:change*=min(1.,.5*dt/max(1e-12,abs(change)))
        velocity+=change;travel+=velocity*dt
        assert .95-travel>=.725, 'Handover braking consumed the unchanged clearance reserve'
