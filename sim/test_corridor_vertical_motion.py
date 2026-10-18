"""Physical altitude hold follows real FCU displacement, not barometer jitter."""
import math
import pytest
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Bool,Float32
from test_corridor_clearance_contract import controller


def observe(c,t,z,vz):
    c._now=lambda:t
    pose=PoseStamped();pose.header.stamp.sec=int(t);pose.header.stamp.nanosec=round((t-int(t))*1e9)
    pose.pose.orientation.w=1.;pose.pose.position.z=z;c._on_pose(pose)
    velocity=TwistStamped();velocity.header=pose.header;velocity.twist.linear.z=vz;c._on_velocity(velocity)


def enable(c):
    c._on_enable(Bool(data=False));observe(c,1.,2.64,0.)
    c._on_enable(Bool(data=True));c._on_hold_alt(Float32(data=2.64))


def test_stationary_baro_wander_cannot_issue_vertical_travel(controller):
    c,_=controller;enable(c)
    for t,z in [(1.1,3.1),(1.2,2.1),(1.3,3.4)]:
        observe(c,t,z,0.);c._on_hold_alt(Float32(data=2.64))
        assert c._alt_correction()==pytest.approx(0.,abs=1e-6)
        assert c._alt_motion_origin==2.64 and c._alt_motion_displacement==0.


def test_signed_vertical_motion_gets_an_opposing_correction(controller):
    c,_=controller;enable(c)
    observe(c,1.1,3.,-.4);observe(c,1.2,3.3,-.4)
    assert c._alt_motion_displacement==pytest.approx(-.06)
    assert c._alt_correction()==pytest.approx(.048,abs=1e-6)
    before=c._alt_motion_displacement;observe(c,1.2,3.5,-99.)
    assert c._alt_motion_displacement==before


def test_commanded_altitude_change_keeps_original_gain_and_cap(controller):
    c,_=controller;enable(c)
    c._on_hold_alt(Float32(data=2.94))
    assert c._alt_correction()==pytest.approx(.24,abs=1e-6)
    c._on_hold_alt(Float32(data=8.))
    assert c._alt_correction()==.6


@pytest.mark.parametrize('kind',['nan','future','gap','stale','missing'])
def test_unusable_motion_cannot_turn_baro_changes_into_control(controller,kind):
    c,_=controller;enable(c)
    if kind=='nan':observe(c,1.1,3.5,float('nan'))
    if kind=='future':
        c._now=lambda:1.1
        m=TwistStamped();m.header.stamp.sec=2;c._on_velocity(m)
    if kind=='gap':observe(c,1.+3*float(c._g('stale_scan_s'))+.1,3.5,0.)
    if kind=='stale':c._now=lambda:2.
    if kind=='missing':c._fcu_velocity_t=None
    assert c._alt_correction()==0.
    assert not c._velocity_fresh()


def test_capture_gap_stays_unknown_until_an_actual_new_handover(controller):
    c,_=controller;enable(c);t=1.+3*float(c._g('stale_scan_s'))+.1
    observe(c,t,3.5,0.)
    observe(c,t+.1,3.3,0.)
    assert c._alt_motion_fault and c._alt_correction()==0.
    c._on_enable(Bool(data=False));c._on_enable(Bool(data=True))
    assert not c._alt_motion_fault and c._alt_motion_origin==3.3
    assert c._alt_correction()==0.


def test_disable_clears_previous_traversal_displacement(controller):
    c,_=controller;enable(c);observe(c,1.1,2.6,.4)
    assert c._alt_motion_displacement>.0
    c._on_enable(Bool(data=False))
    assert c._alt_motion_origin is None and c._alt_motion_displacement==0.


def test_motion_anchor_arrests_a_real_sink_despite_noisy_baro(controller):
    c,_=controller;enable(c);height=2.64
    for i in range(1,401):
        t=1.+i*.05
        measured=-.35+c._alt_correction()
        height+=measured*.05
        observe(c,t,height+.4*math.sin(t*10),measured)
    assert height>2.64-.5
    assert c._alt_motion_displacement==pytest.approx(height-2.64,abs=.02)


def test_a_brief_capture_gap_is_bridged_and_motion_resumes(controller):
    """16 Oct campaign: one gap just over stale_scan_s latched the fault, so
    _velocity_fresh() stayed false and the corridor stopped for good."""
    c,_=controller;enable(c)
    observe(c,1.1,2.64,-.2)
    t=1.1+float(c._g('stale_scan_s'))+.2           # a gap, but under 3x
    c._now=lambda:t-.05
    assert not c._velocity_fresh()                 # still stops DURING the gap
    observe(c,t,3.4,-.2)                           # baro jumps; FCU vz steady
    assert not c._alt_motion_fault and c._velocity_fresh()
    assert c._alt_motion_displacement==pytest.approx(-.01-.2*(t-1.1),abs=1e-6)   # ramp from enable, then the bridged gap
    assert c._alt_motion_origin==2.64              # baro never adopted
