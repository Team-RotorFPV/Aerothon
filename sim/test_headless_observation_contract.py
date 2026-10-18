"""Pin physical motion observations, delayed handover and terminal evidence."""
from collections import deque
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import headless_world as H
from qualification_evidence import recorded_disarm


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ChangingWind:
    def __init__(self):
        self.calls = 0

    def __call__(self, _stamp):
        self.calls += 1
        return .12*self.calls, -.08*self.calls


def world():
    w = H.HeadlessWorld.__new__(H.HeadlessWorld)
    w.lock = threading.Lock()
    w.args = SimpleNamespace(vel_command_delay_s=.4, vel_response_s=.6,
                             start_after_s=100., max_sim_s=1200., rtf=1000.)
    w.t = 10.
    w.p = np.array([0., 0., 3.])
    w.v = np.array([.25, -.1, .03])
    w.ground_velocity = np.zeros(3)
    w.yaw = math.pi/2
    w.roll = w.pitch = 0.
    w.armed = True
    w.mode = 'GUIDED'
    w.sp_pos = w.sp_vel = w.takeoff_alt = None
    w.pending_velocity = deque()
    w.speed_cap = 3.
    w.wind = None
    w.arena = SimpleNamespace(contact=lambda *_:None, home_world=(0., 0.))
    w.contacts = []
    w.params = {}
    w.fence_poly = None
    w.fence_breaches = 0
    w.landed_t = None
    w.est = lambda:w.p.copy()
    w.events = []
    w._event = lambda event:w.events.append((w.t,event))
    w.fcu = {'battery_v':12.}
    w.armed_at = 1.
    w.payload = np.array([0., 0., .04])
    w.cam_joint = w.cam_cmd = 0.
    w.track = []
    w.result = None
    w.mission_state = 'RETURN'
    w.started = True
    w.done = False
    w._finish_seen_s = None
    for name in ('pose','vel','state','batt','home','payload_pose','joint','clock','start'):
        setattr(w,'pub_'+name,Publisher())
    return w


def vector(message):
    v = message.twist.linear
    return np.array([v.x,v.y,v.z])


def flight_rows(messages):
    rows = []
    for m in messages:
        captured = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        rows.append(dict(stamp=captured,flight=dict(received_stamp=captured,data=dict(
            connected=m.connected,armed=m.armed,header=dict(stamp=dict(
                sec=m.header.stamp.sec,nanosec=m.header.stamp.nanosec))))))
    return rows


def test_wind_observation_matches_integrated_world_motion_without_advancing_wind_twice():
    w = world()
    w.wind = ChangingWind()
    for _ in range(3):
        before = w.p.copy()
        w._step_airframe()
        w.t += H.DT
        movement = (w.p-before)/H.DT
        calls = w.wind.calls
        w._publish_fcu(w.est())
        w._publish_fcu(w.est())
        np.testing.assert_allclose(vector(w.pub_vel.messages[-1]),movement,atol=1e-12)
        assert w.wind.calls == calls
        assert w.pub_vel.messages[-1].header.frame_id == 'map'


def test_wind_cache_clears_when_the_aircraft_leaves_the_wind_model():
    w = world()
    w.wind = ChangingWind()
    w._step_airframe()
    w.wind = None
    before = w.p.copy()
    w._step_airframe()
    w.t += H.DT
    w._publish_fcu(w.est())
    np.testing.assert_allclose(vector(w.pub_vel.messages[-1]),(w.p-before)/H.DT,atol=1e-12)


def test_ground_contact_velocity_matches_the_completed_clamped_step():
    w = world()
    w.mode = 'LAND'
    w.p[2] = .00001
    w.v[:] = 0.
    before = w.p.copy()
    w._step_airframe()
    w.t += H.DT
    assert w.p[2] == 0.
    w._publish_fcu(w.est())
    np.testing.assert_allclose(vector(w.pub_vel.messages[-1]),(w.p-before)/H.DT,atol=1e-12)


def test_obstacle_reaction_is_in_the_reported_motion_without_an_extra_wind_step():
    w = world()
    w.wind = ChangingWind()
    w.arena.contact = lambda *_:'test obstacle'
    before = w.p.copy()
    w._step_airframe()
    w.t += H.DT
    assert w.contacts
    w._publish_fcu(w.est())
    np.testing.assert_allclose(vector(w.pub_vel.messages[-1]),(w.p-before)/H.DT,atol=1e-12)
    assert w.wind.calls == 1


def test_position_control_continues_until_the_velocity_packet_arrives():
    w = world()
    w.sp_pos = (2., 0., 3., 0.)
    before = w._desired_velocity(w.est())
    w._on_sp_raw(SimpleNamespace(velocity=SimpleNamespace(x=.8,y=0.,z=0.),
                                yaw_rate=0.,coordinate_frame=1))
    w.t = 10.399
    during = w._desired_velocity(w.est())
    np.testing.assert_allclose(during[0],before[0])
    assert during[1] == before[1]
    assert w.sp_pos == (2.,0.,3.,0.)
    w.t = 10.4
    after = w._desired_velocity(w.est())
    np.testing.assert_allclose(after[0],[.8,0.,0.])
    assert w.sp_pos is None


def test_takeoff_control_continues_until_the_velocity_packet_arrives():
    w = world()
    w.takeoff_alt = 5.
    w._on_sp_raw(SimpleNamespace(velocity=SimpleNamespace(x=.8,y=0.,z=0.),
                                yaw_rate=0.,coordinate_frame=1))
    w.t = 10.399
    assert w._desired_velocity(w.est())[0][2] > 0.
    assert w.takeoff_alt == 5.
    w.t = 10.4
    np.testing.assert_allclose(w._desired_velocity(w.est())[0],[.8,0.,0.])
    assert w.takeoff_alt is None


def run_phase_boundary(w,monkeypatch):
    # Exercise the actual physical step/loop/publications; camera and errors
    # are outside this observation contract and do not affect the test motion.
    w._step_errors = lambda:None
    w._step_payload = lambda:None
    w._publish_scan = lambda:None
    w.frame_drop = 1.
    w.rng = SimpleNamespace(random=lambda:0.)
    w.pending_frames = []
    monkeypatch.setattr(H.rclpy,'ok',lambda:True)
    w.run()


def test_disarm_between_periodic_publications_reaches_fcu_and_truth(monkeypatch):
    w = world()
    w.mode = 'LAND'
    w.p[:] = [0.,0.,0.]
    w.v[:] = 0.
    w.landed_t = w.t-1.1
    w.result = dict(state='FAILED',reason='test terminal evidence')
    w._publish_slow()
    w._record()
    run_phase_boundary(w,monkeypatch)
    assert w.track[-1][4] == 0
    assert w.track[-1][0] >= 10.01
    assert w.pub_state.messages[-1].armed is False
    assert recorded_disarm(flight_rows(w.pub_state.messages)) is not None
    stamps = [row[0] for row in w.track]
    assert all(b>a for a,b in zip(stamps,stamps[1:]))


def test_time_limit_records_the_actual_armed_cut_without_inventing_disarm(monkeypatch):
    w = world()
    w.args.max_sim_s = 10.01
    w._publish_slow()
    w._record()
    run_phase_boundary(w,monkeypatch)
    assert w.track[-1][0] == pytest.approx(10.01)
    assert w.track[-1][4] == 1
    assert w.pub_state.messages[-1].armed is True
    assert recorded_disarm(flight_rows(w.pub_state.messages)) is None


def test_terminal_drain_resets_if_the_aircraft_rearms():
    w = world()
    w.p[2] = 0.
    w.result = dict(state='COMPLETED')
    w.armed = False
    w._start_and_stop()
    assert w.done is False
    w.t += .5
    w.armed = True
    w._start_and_stop()
    w.t += 2.
    w.armed = False
    w._start_and_stop()
    assert w.done is False


def test_banner_queue_keeps_capture_time_and_reports_actual_delivery_time():
    w=world()
    for name in ('off','qr','match','qr_detail','banner','banner_detail','red','payload'):
        setattr(w,'pub_'+name,Publisher())
    frame=dict(qr=dict(off=H.Vector3(),decoded='',matched=False,detail={}),
               banner=dict(vec=H.Vector3(),detail=dict(capture_stamp=9.8,processed_stamp=9.8)),
               red={},payload={})
    w._deliver(frame)
    import json
    detail=json.loads(w.pub_banner_detail.messages[-1].data)
    assert detail['capture_stamp']==9.8
    assert detail['processed_stamp']==10.
