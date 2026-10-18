"""Signed, continuous FCU travel may confirm the unchanged gate distance."""
from collections import deque
import math
import pytest
from geometry_msgs.msg import TwistStamped
from test_gate_braking import actor


def crossing():
    mav,leaf=actor()
    leaf._cross_epoch=1.;leaf._cross_axis=(1.,0.);leaf._flight_distance=1.25;leaf.tol=.25
    mav.ground_velocity_samples=deque(maxlen=80)
    return mav,leaf


def test_ground_velocity_callback_preserves_signed_world_components():
    mav,leaf=crossing();msg=TwistStamped();msg.header.stamp.sec=1
    msg.twist.linear.x=-.3;msg.twist.linear.y=.4
    mav._delivery_velocity(msg)
    assert mav.ground_velocity_samples[-1]==(1.,-.3,.4)
    assert mav.delivery_inputs['speed']==pytest.approx(.5)


def test_only_forward_distance_counts_without_duplicate_capture_credit():
    mav,leaf=crossing()
    for index in range(11):
        t=1.+index*.1;mav.ground_velocity_samples.append((t,1.,0.))
        assert leaf._distance_covered_by_motion(t)==(index==10)
        before=leaf._cross_displacement.copy()
        leaf._distance_covered_by_motion(t)
        assert leaf._cross_displacement==before
    assert leaf._cross_displacement==pytest.approx([1.,0.])


@pytest.mark.parametrize('velocity',[(-1.,0.),(0.,1.)])
def test_backward_or_lateral_motion_never_confirms_forward_crossing(velocity):
    mav,leaf=crossing()
    for index in range(20):
        t=1.+index*.1;mav.ground_velocity_samples.append((t,*velocity))
        assert not leaf._distance_covered_by_motion(t)


def test_motion_before_speed_admission_is_not_crossing_distance():
    mav,leaf=crossing();leaf._cross_epoch=3.
    mav.ground_velocity_samples.extend((1.+i*.1,5.,0.) for i in range(20))
    assert not leaf._distance_covered_by_motion(3.)
    assert leaf._cross_displacement==[0.,0.]


@pytest.mark.parametrize('failure',['gap','nonfinite','stale'])
def test_invalid_motion_history_cannot_supply_arrival(failure):
    mav,leaf=crossing();mav.ground_velocity_samples.append((1.,10.,0.))
    assert not leaf._distance_covered_by_motion(1.)
    if failure=='gap':mav.ground_velocity_samples.append((1.4,10.,0.))
    elif failure=='nonfinite':mav.ground_velocity_samples.append((1.1,math.nan,0.))
    assert not leaf._distance_covered_by_motion(1.4)
