"""Transient sensor gaps pause crossing without relaxing freshness or braking."""
from unittest.mock import patch
from test_gate_braking import actor
from mission_bt.mission_tree import GateAdvance
from mission_bt.gate_braking import RUNNING,FAILURE,SUCCESS


def test_transient_scan_gap_does_not_abort_or_advance_on_old_data():
    mav,leaf=actor();leaf._phase='CROSS';leaf._flight_distance=4.1
    mav.t=1.35;mav.observe();mav._scan.header.stamp.sec=1;mav._scan.header.stamp.nanosec=0
    assert leaf.update() is RUNNING
    assert mav.commands[-1]==mav.xyz and leaf._t==0
    mav.t=1.55;mav.observe()
    leaf.router.fly=lambda vehicle,x,y,z,yaw:vehicle.goto(x,y,z,yaw)
    assert leaf.update() is RUNNING
    assert mav.commands[-1]==(16.,-15.,2.) and leaf._t==1
    assert leaf.stale_s==.3 and leaf.arrival_speed_mps==.08


def test_capture_gap_cannot_reuse_old_settling_confirmations():
    mav,leaf=actor()
    with patch.object(GateAdvance,'update',return_value=SUCCESS):
        assert leaf.update() is RUNNING
        mav.t=1.2;mav.observe();assert leaf.update() is RUNNING
        assert leaf._settle_count==2
        # A missed scan need not interrupt the independent FCU velocity
        # stream. Preserve its real captures across this sensor pause.
        mav.t=1.4;mav.observe()
        mav._scan.header.stamp.sec=1;mav._scan.header.stamp.nanosec=200000000
        mav.t=1.55;assert leaf.update() is RUNNING
        assert leaf._settle_count==0 and leaf._phase=='BRAKE'
        mav.t=1.6;mav.observe();assert leaf.update() is RUNNING
        mav.t=1.8;mav.observe();assert leaf.update() is RUNNING
        assert mav.speed_requests==[.5]
        mav.t=2.;mav.observe();assert leaf.update() is SUCCESS
        assert mav.speed_requests==[.5,5.]


def test_persistent_capture_gap_remains_fatal_without_speed_restore():
    mav,leaf=actor();leaf._phase='BRAKE';leaf._brake_hold=mav.xyz;leaf._brake_yaw=0.
    mav.t=1.35;assert leaf.update() is RUNNING
    mav.t=1.66;assert leaf.update() is RUNNING     # within pause_s (0.6 s)
    mav.t=1.96;assert leaf.update() is FAILURE
    assert mav.commands[-1]==mav.xyz and not mav.speed_requests
    assert 'capture times required' in mav.abort_reason
