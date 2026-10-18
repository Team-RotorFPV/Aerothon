"""LAND preserves FCU estimated XY control through the existing airframe."""
import sys,threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'sim'),str(ROOT/'scripts')]
from headless_world import HeadlessWorld,DT,LAND_SPEED


def world():
    w=HeadlessWorld.__new__(HeadlessWorld);w.lock=threading.Lock()
    w.mode='GUIDED';w.armed=True;w.t=1.;w.yaw=0.
    w.p=np.array([3.,-2.,4.]);w.v=np.zeros(3)
    w.gps_err=np.array([.2,-.1]);w.glitch=np.zeros(2);w.baro_err=w.baro_noise=0.
    w.wind=lambda t:(.15,0.)
    w.args=SimpleNamespace(vel_response_s=.6,vel_command_delay_s=0.)
    w.sp_pos=(0.,0.,4.,None);w.sp_vel=w.takeoff_alt=None;w.pending_velocity=deque()
    w.speed_cap=3.;w.arena=SimpleNamespace(contact=lambda x,y,z:None);w.contacts=[]
    w.fence_poly=None;w.params={};w.landed_t=None;w._event=lambda message:None
    return w


def enter_land(w,service):
    if service=='land':return w._srv_land(SimpleNamespace(),SimpleNamespace())
    return w._srv_mode(SimpleNamespace(custom_mode='LAND'),SimpleNamespace())


@pytest.mark.parametrize('service',['land','mode'])
def test_landing_keeps_estimated_xy_and_existing_descent_response(service):
    w=world();estimate=w.est().copy();enter_land(w,service)
    np.testing.assert_allclose(w.land_xy,estimate[:2])
    assert not np.allclose(w.land_xy,w.p[:2])
    assert not np.allclose(w.land_xy,w.sp_pos[:2])
    w.p[0]+=.3;velocity,yaw=w._desired_velocity(w.est())
    assert velocity[0]<0 and velocity[2]==-LAND_SPEED and yaw==0
    assert w.args.vel_response_s==.6 and w.args.vel_command_delay_s==0


@pytest.mark.parametrize('service',['land','mode'])
def test_repeated_land_requests_do_not_relatch_a_drifting_target(service):
    w=world();enter_land(w,service);target=w.land_xy.copy();w.p[0]+=.3
    enter_land(w,service)
    np.testing.assert_allclose(w.land_xy,target)
    assert w._desired_velocity(w.est())[0][0]<0


def test_leaving_guided_and_starting_another_landing_latches_new_estimate():
    w=world();enter_land(w,'land');old=w.land_xy.copy()
    w._srv_mode(SimpleNamespace(custom_mode='GUIDED'),SimpleNamespace())
    assert w.land_xy is None
    w.p[0]+=2.;enter_land(w,'land')
    np.testing.assert_allclose(w.land_xy,w.est()[:2]);assert not np.allclose(w.land_xy,old)


def test_wind_and_airframe_are_retained_in_actual_landing_physics():
    w=world();initial=w.p.copy();enter_land(w,'land')
    maximum_drift=0.
    for _ in range(3000):
        w.t+=DT;w._step_airframe()
        maximum_drift=max(maximum_drift,float(np.linalg.norm(w.p[:2]-initial[:2])))
        if not w.armed:break
    assert not w.armed and w.p[2]<=.02
    drift=np.linalg.norm(w.p[:2]-initial[:2])
    assert drift<.4 and maximum_drift>.04
    assert np.linalg.norm(w.p[:2])>3.,'LAND cannot use venue truth to repair landing'
    assert w.args.vel_response_s==.6 and w.args.vel_command_delay_s==0.


def test_low_altitude_descent_still_uses_original_slow_rate():
    w=world();w.p[2]=.8;enter_land(w,'land')
    assert w._desired_velocity(w.est())[0][2]==-.3


def test_disarmed_airframe_still_commands_no_motion():
    w=world();enter_land(w,'land');w.armed=False
    np.testing.assert_allclose(w._desired_velocity(w.est())[0],[0,0,0])
