"""Controller integration for strict clearance and bounded safe recovery."""
import math
import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/aerothon_avoidance/avoidance'))
from avoidance.velocity_controller import VelocityController


@pytest.fixture
def controller():
    rclpy.init()
    c=VelocityController(); c.enabled=True
    # This unit fixture transmits immediately. Delayed-actuator regressions
    # override the parameter and explicitly queue transmitted commands.
    c.set_parameters([Parameter('vel_command_delay_s',value=0.0)])
    # This fixture freezes the clock while _publish computes. Independent
    # clock and deadline tests explicitly restore the production budget.
    if c.has_parameter('motion_computation_budget_s'):
        c.set_parameters([Parameter('motion_computation_budget_s',value=0.0)])
    c._now=lambda:1.0
    pose=PoseStamped();pose.pose.orientation.w=1.0;c._on_pose(pose)
    c.scan=LaserScan(angle_min=-math.pi,angle_increment=2*math.pi/360,
                     ranges=[12.0]*360)
    c.scan.header.frame_id='base_link'
    c._on_scan(c.scan)
    sent=[]; c.pub_sp.publish=sent.append
    yield c,sent
    c.destroy_node(); rclpy.shutdown()


def test_pose_staleness_stops(controller):
    c,s=controller; c._pose_t=-1;c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0
    assert 'pose stale' in c.last_detail['fault']


def test_stale_capture_cannot_be_refreshed_by_arrival(controller):
    c,s=controller; c._now=lambda:10;c.scan.header.stamp.sec=1
    c._on_scan(c.scan); c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0
    assert 'scan stale' in c.last_detail['fault']


def test_masked_reverse_never_moves(controller):
    c,s=controller;c.state='BACKOFF';c._backoff=10;c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0
    assert 'unobserved' in c.last_detail['recovery_reason']


def test_observed_reverse_requires_clearance(controller):
    c,s=controller
    c.set_parameters([Parameter('mask_sectors_deg',Parameter.Type.DOUBLE_ARRAY,[]),Parameter('median_window',value=1)])
    c.state='BACKOFF';c._backoff=10;c._tick()
    assert s[-1].velocity.x<0
    c.scan.ranges=[.6]*360;c._on_scan(c.scan);c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0


def test_reverse_and_sideways_motion_do_not_reset_stall(controller):
    c,_=controller;c._axis=0;c._progress_ref=(0,0);c._progress_t=-20
    c._pos=(-2,2)
    assert c.stalled()


def test_mouth_is_not_far_exit(controller):
    c,_=controller;c._origin=(0,0);c._axis=0;c._entered=True
    c._entered_progress=1;c._max_progress=5;c._pos=(.5,0)
    c._open_ticks=20;c._tick()
    assert not c.last_detail['corridor_exited']


def test_stuck_is_terminal_until_reenabled(controller):
    c,s=controller;c.state='STUCK';c._tick()
    assert c.state=='STUCK'
    assert s[-1].velocity.x==s[-1].velocity.y==0


def test_recovery_exhausts_even_if_reverse_is_unobserved(controller):
    c,_=controller;c.scan.ranges=[.6]*360
    c.set_parameters([Parameter('blocked_ticks_before_backoff',value=1),Parameter('backoff_ticks',value=1)])
    for _ in range(12): c._tick()
    assert c.state=='STUCK'


def test_hold_compensation_cannot_command_into_nearby_surface(controller):
    # The rendered failure continued moving under compensation after BLOCKED.
    # A fresh raycast has sufficient clearance now, but not for this hold.
    from test_local_planner import wall_scan
    c,s=controller;c._planner.observe(*wall_scan(.8,1.75))
    c._compensated=lambda vx,vy:(0,.8)
    c._publish(0,0,2,0,{'note':'no navigable gap'})
    assert s[-1].velocity.y<=0
    assert 'braking' in c.last_detail.get('recovery_reason','')


def test_rejected_gust_forecast_keeps_certified_current_wind_hold(controller):
    # A forecast can over-correct toward the opposite wall. Publishing raw
    # zero instead removes cancellation of the disturbance actually measured.
    from test_local_planner import wall_scan
    import numpy as np
    c,s=controller;c._planner.observe(*wall_scan(.8,1.75))
    c._wind_world=np.array([0,.35]);c._ground_world=np.zeros(2)
    c._compensated=lambda vx,vy:(0,.8)
    c._publish(0,0,2,0,{'note':'no navigable gap'})
    assert s[-1].velocity.y==pytest.approx(-.35)
    assert c.last_detail['hold_source']=='certified current disturbance cancellation'


def test_incoming_drift_uses_certified_active_braking_before_raw_zero(controller):
    from test_local_planner import wall_scan
    import numpy as np
    c,s=controller;c._planner.observe(*wall_scan(.8,1.75))
    c._wind_world=np.array([0,.35]);c._ground_world=np.array([0,.25])
    c._compensated=lambda vx,vy:(0,.8)
    c._publish(0,0,2,0,{'note':'no navigable gap'})
    assert s[-1].velocity.y==pytest.approx(-.60)
    assert c.last_detail['hold_source']=='certified active disturbance braking'


def test_clearance_check_includes_velocity_remaining_after_one_response_time(controller):
    from test_local_planner import wall_scan
    import numpy as np
    c,s=controller;c._planner.observe(*wall_scan(.8,1.75))
    c._wind_world=np.array([0,.35]);c._ground_world=np.array([0,.2])
    c._compensated=lambda vx,vy:(0,.8)
    c._publish(0,0,2,0,{'note':'no navigable gap'})
    # Pure cancellation travels 0.12 m before its measured incoming motion
    # decays. The former 0.6 s horizon checked only 0.074 m of that motion.
    assert s[-1].velocity.y==pytest.approx(-.55)
    assert c.last_detail['hold_source']=='certified active disturbance braking'


def test_braking_prediction_respects_configured_deceleration(controller):
    import numpy as np
    c,s=controller
    bearings=np.linspace(-math.pi,math.pi,721)
    ranges=np.full(len(bearings),12.)
    front=np.cos(bearings)>0
    ranges[front]=np.minimum(12,1.8/np.cos(bearings[front]))
    c._planner.observe(bearings,ranges)
    c._ground_world=np.array([.8,0]);c._wind_world=np.zeros(2)
    c._compensated=lambda vx,vy:(vx,vy)
    c._publish(.8,0,1.8,0,{})
    # 0.8 m/s braking at 0.5 m/s^2 needs at least 0.64 m after the
    # commanded response interval. Exponential decay alone was faster.
    assert s[-1].velocity.x<.8
    assert 'braking' in c.last_detail.get('recovery_reason','')


def test_lateral_braking_does_not_require_unobserved_reverse_travel(controller):
    from test_local_planner import wall_scan
    import numpy as np
    c,s=controller
    bearings,ranges=wall_scan(.8,1.75)
    observed=np.abs(bearings)<math.radians(150)
    c._planner.observe(bearings[observed],ranges[observed])
    c._wind_world=np.array([0,.35]);c._ground_world=np.array([.8,.2])
    c._compensated=lambda vx,vy:(0,.8)
    c._publish(0,0,2,0,{'note':'no navigable gap'})
    assert s[-1].velocity.x==pytest.approx(0)
    assert s[-1].velocity.y==pytest.approx(-.55)
    assert c.last_detail['hold_source']=='certified active disturbance braking'
