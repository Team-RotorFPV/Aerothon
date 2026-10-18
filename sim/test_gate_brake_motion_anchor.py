"""Arrival remains physical despite estimator wander and service waits."""
from unittest.mock import patch
import pytest
from test_gate_braking import actor
from mission_bt.mission_tree import GateAdvance
from mission_bt.gate_braking import RUNNING,FAILURE,SUCCESS


def enter():
    mav,leaf=actor()
    with patch.object(GateAdvance,'update',return_value=SUCCESS):
        assert leaf.update() is RUNNING
    return mav,leaf


def test_estimator_jump_is_cancelled_without_flying_to_old_gps_anchor():
    mav,leaf=enter()
    for t,xy in [(1.2,(21.,-17.)),(1.4,(19.,-14.))]:
        mav.t=t;mav.xyz=(*xy,2.);mav.observe()
        status=leaf.update()
        assert mav.commands[-1]==mav.xyz
        assert leaf._brake_displacement==[0.,0.]
    assert status is SUCCESS and mav.speed_requests==[.5,5.]


def test_ground_displacement_is_retained_when_estimate_moves():
    mav,leaf=actor();mav.observe(speed=.4)
    with patch.object(GateAdvance,'update',return_value=SUCCESS):
        assert leaf.update() is RUNNING
    mav.t=1.2;mav.xyz=(21.,-15.,2.);mav.observe(speed=.2)
    assert leaf.update() is RUNNING
    assert leaf._brake_displacement==pytest.approx([.06,0.])
    assert mav.commands[-1]==pytest.approx((20.94,-15.,2.))
    assert mav.speed_requests==[.5]
    previous=leaf._brake_displacement.copy()
    assert leaf.update() is RUNNING
    assert leaf._brake_displacement==previous


def fresh_scalar_only(mav, t):
    """Scalar speed and scan fresh at t, with no new vector capture."""
    mav.t = t
    mav._scan.header.stamp.sec = int(t); mav._scan.header.stamp.nanosec = round((t-int(t))*1e9)
    mav.delivery_inputs.update(pose_stamp=t, velocity_stamp=t, speed=0.)


@pytest.mark.parametrize('kind',['nan','malformed','long gap'])
def test_invalid_or_unbridgeable_vector_history_fails_closed_at_once(kind):
    mav,leaf=enter();mav.t=1.2;mav.observe()
    if kind=='nan':mav.ground_velocity_samples.append((1.2,float('nan'),0.))
    if kind=='malformed':mav.ground_velocity_samples.append(None)
    if kind=='long gap':                       # beyond bridge_s (0.9 s)
        mav.t=2.25;mav.observe()
    assert leaf.update() is FAILURE
    assert mav.commands[-1]==mav.xyz and mav.speed_requests==[.5]
    assert 'continuous fresh ground velocity' in mav.abort_reason


def test_missing_vector_history_holds_then_fails_closed():
    mav,leaf=enter();mav.t=1.2;mav.observe()
    mav.ground_velocity_samples.clear()
    hold=leaf._brake_hold
    assert leaf.update() is RUNNING and mav.commands[-1]==hold
    fresh_scalar_only(mav,1.75);assert leaf.update() is RUNNING     # within pause_s
    fresh_scalar_only(mav,1.85);assert leaf.update() is FAILURE
    assert mav.speed_requests==[.5]
    assert 'continuous fresh ground velocity' in mav.abort_reason


def test_a_short_capture_gap_is_bridged_not_fatal():
    mav,leaf=enter();mav.t=1.2;mav.observe(speed=.2)
    assert leaf.update() is RUNNING
    mav.t=1.6;mav.observe(speed=.2)            # 0.4 s gap: over stale_s, under bridge_s
    assert leaf.update() is RUNNING and 'gate braking' not in mav.abort_reason
    assert leaf._brake_displacement[0]==pytest.approx(.02+.08)


def test_future_vector_cannot_supply_missing_current_motion():
    mav,leaf=enter();mav.t=1.4;mav.observe()
    mav.ground_velocity_samples.clear();mav.ground_velocity_samples.append((2.,0.,0.))
    assert leaf.update() is RUNNING                 # held, never integrated
    assert leaf._brake_displacement==[0.,0.]
    fresh_scalar_only(mav,2.05)                     # still no current capture
    assert leaf.update() is FAILURE
    assert mav.speed_requests==[.5]


def test_duplicate_captures_cannot_finish_braking_after_estimate_change():
    mav,leaf=enter();mav.xyz=(21.,-18.,2.)
    for _ in range(5):
        mav.t+=.02
        assert leaf.update() is RUNNING
        assert mav.commands[-1]==mav.xyz
    assert leaf._settle_count==1 and mav.speed_requests==[.5]


def test_pending_restore_stays_at_physical_hold_through_estimate_change():
    mav,leaf=enter()
    mav.t=1.2;mav.observe();assert leaf.update() is RUNNING
    mav.t=1.4;mav.observe();mav.response='pending'
    assert leaf.update() is RUNNING and leaf._phase=='RESTORE'
    for t in [1.6,1.8,2.]:
        mav.t=t;mav.xyz=(20.+t,-16.-t,2.);mav.observe()
        assert leaf.update() is RUNNING
        assert mav.commands[-1]==mav.xyz
    assert mav.speed_requests==[.5,5.]


def test_altitude_arrival_tolerance_is_unchanged():
    mav,leaf=enter();leaf.tol=.25;mav.xyz=(20.,-16.,2.7)
    mav.t=1.2;mav.observe();assert leaf.update() is RUNNING
    mav.t=1.4;mav.observe();assert leaf.update() is RUNNING
    assert mav.commands[-1][2]==2. and mav.speed_requests==[.5]
    assert leaf.arrival_speed_mps==.08 and leaf.stale_s==.3


@pytest.mark.parametrize('epoch',[1.,682.02,9000.])
@pytest.mark.parametrize('interval_ns,accepted',[(300000001,True),(899999999,True),(900000000,True),(900000001,False)])
def test_capture_gap_uses_exact_ros_nanosecond_boundary(epoch,interval_ns,accepted):
    mav,leaf=actor();mav.t=epoch;mav.observe()
    leaf._phase='BRAKE';leaf._brake_hold=mav.xyz;leaf._brake_yaw=0.
    leaf._brake_motion_previous=mav.ground_velocity_samples[-1]
    leaf._brake_capture_stamp=leaf._brake_motion_previous[0]
    mav.t=epoch+interval_ns/1e9;mav.observe()
    assert leaf._brake_position() is accepted
    assert leaf.stale_s==.3
