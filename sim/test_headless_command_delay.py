"""Exercise the real headless setpoint callback and flight-control selection."""
from collections import deque
from types import SimpleNamespace
import threading
import sys
from pathlib import Path

import numpy as np
import pytest
import math

sys.path.insert(0, str(Path(__file__).resolve().parent))
from headless_world import HeadlessWorld


def world(delay=.4):
    w = HeadlessWorld.__new__(HeadlessWorld)
    w.lock = threading.Lock()
    w.mode = 'GUIDED'
    w.armed = True
    w.t = 1.
    w.yaw = 0.
    w.sp_pos = w.sp_vel = w.takeoff_alt = None
    w.args = SimpleNamespace(vel_command_delay_s=delay)
    w.pending_velocity = deque()
    w.speed_cap = 3.
    return w


def send(w, vx):
    w._on_sp_raw(SimpleNamespace(velocity=SimpleNamespace(x=vx, y=0., z=0.),
                               yaw_rate=0.))


def desired(w, stamp):
    w.t = stamp
    return w._desired_velocity(np.array([0., 0., 2.]))[0][0]


def test_command_waits_for_transport():
    w = world()
    send(w, .8)
    assert desired(w, 1.2) == 0.
    assert desired(w, 1.4) == pytest.approx(.8)


def test_stop_cannot_cancel_a_command_already_in_transit():
    w = world()
    send(w, .8)
    w.t = 1.2
    send(w, 0.)
    assert desired(w, 1.45) == pytest.approx(.8)
    assert desired(w, 1.65) == 0.


def test_explicit_zero_delay_preserves_immediate_model():
    w = world(0.)
    send(w, .8)
    assert desired(w, 1.) == pytest.approx(.8)


def test_position_target_cancels_pending_velocity_mode():
    w = world()
    send(w, .8)
    w._on_sp_pos(SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0., y=0., z=2.),
        orientation=SimpleNamespace(w=1., x=0., y=0., z=0.))))
    assert desired(w, 1.5) == 0.


def test_guided_timeout_still_stops_stale_velocity():
    w = world()
    send(w, .8)
    assert desired(w, 4.5) == 0.


def test_body_velocity_converts_once_at_receipt_heading():
    w=world();send(w,.8)
    w.yaw=math.pi/2;w.t=1.4
    velocity,_=w._desired_velocity(np.array([0.,0.,2.]))
    np.testing.assert_allclose(velocity,[0.,.8,0.],atol=1e-12)
    w.yaw=math.pi;w.t=1.5
    velocity,_=w._desired_velocity(np.array([0.,0.,2.]))
    np.testing.assert_allclose(velocity,[0.,.8,0.],atol=1e-12)


def test_local_velocity_does_not_rotate_with_receipt_heading():
    w=world();w._on_sp_raw(SimpleNamespace(velocity=SimpleNamespace(x=.8,y=0.,z=0.),yaw_rate=0.,coordinate_frame=1))
    w.yaw=math.pi/2;w.t=1.4
    velocity,_=w._desired_velocity(np.array([0.,0.,2.]))
    np.testing.assert_allclose(velocity,[.8,0.,0.],atol=1e-12)
