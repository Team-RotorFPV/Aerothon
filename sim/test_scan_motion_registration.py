"""Capture registration must not turn GPS jitter into obstacle movement."""
import copy
import pytest
import numpy as np
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from test_corridor_clearance_contract import controller
from test_local_planner import wall_scan
from test_headless_land_position_hold import world,DT

def velocity(t,v=(0.,0.)):
 m=TwistStamped();m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
 m.twist.linear.x,m.twist.linear.y=v;return m

def test_stationary_gps_jitter_cannot_authorize_a_clearance_violating_packet(controller):
 c,sent=controller;c.set_parameters([Parameter('motion_computation_budget_s',value=.20)])
 c._on_velocity(velocity(1.));b,r=wall_scan(.74,1.75)
 scan=LaserScan();scan.header.stamp.sec=1;scan.header.frame_id='base_link'
 scan.angle_min=float(b[0]);scan.angle_increment=float(b[1]-b[0]);scan.range_min=.05;scan.range_max=12.;scan.ranges=list(map(float,r));c._on_scan(scan)
 c._now=lambda:1.1
 pose=PoseStamped();pose.header.stamp.sec=1;pose.header.stamp.nanosec=100000000;pose.pose.orientation.w=1.;pose.pose.position.y=-.04
 c._on_pose(pose);c._on_velocity(velocity(1.1))
 angles,ranges=c.conditioned(scan);origin,angles,ranges=c._registered_scan(angles,ranges)
 c._planner.observe(angles,ranges,sensor_origin=origin)
 c._compensated=lambda vx,vy:(0.,.08);c._publish(0.,0.,2.,0.,{})
 w=world();w.p[:2]=0.;w.v[:]=0.;w.wind=lambda t:(0.,0.);w._on_sp_raw(sent[-1]);closest=.74
 for _ in range(round(.6/DT)):
  w.t+=DT;w._step_airframe();closest=min(closest,.74-w.p[1])
 stop=copy.deepcopy(sent[-1]);stop.velocity.x=stop.velocity.y=0.;w._on_sp_raw(stop)
 for _ in range(round(3.6/DT)):
  w.t+=DT;w._step_airframe();closest=min(closest,.74-w.p[1])
 print(dict(closest=closest,origin=origin.tolist(),issued_left=sent[-1].velocity.y,normal_response_s=.6,extra_queue_s=0.))
 assert closest>=.70

def test_signed_world_velocity_is_integrated_between_capture_epochs(controller):
 c,_=controller;c._now=lambda:1.2;c._scan_t=1.;c._pose_t=1.2
 for t,v in [(1.,(1.,2.)),(1.1,(3.,4.)),(1.2,(5.,6.))]:c._on_velocity(velocity(t,v))
 np.testing.assert_allclose(c._scan_ground_displacement(),[.6,.8])

def test_sideways_and_backwards_ground_motion_keep_their_sign(controller):
 c,_=controller;c._now=lambda:1.1;c._scan_t=1.;c._pose_t=1.1
 for t in (1.,1.1):c._on_velocity(velocity(t,(-.4,.3)))
 np.testing.assert_allclose(c._scan_ground_displacement(),[-.04,.03])

@pytest.mark.parametrize('scan,pose,times',[(.9,1.1,[1.,1.1]),(1.,1.2,[1.,1.1]),(1.,1.2,[1.,1.2])])
def test_uncovered_or_broken_history_cannot_align_a_scan(controller,scan,pose,times):
 c,_=controller;c._now=lambda:1.2;c._scan_t=scan;c._pose_t=pose
 for t in times:c._on_velocity(velocity(t))
 with pytest.raises(ValueError):c._scan_ground_displacement()

def test_duplicate_velocity_captures_cannot_add_displacement(controller):
 c,_=controller;c._now=lambda:1.1;c._scan_t=1.;c._pose_t=1.1
 c._on_velocity(velocity(1.,(.1,0.)));c._on_velocity(velocity(1.1,(.1,0.)))
 c._on_velocity(velocity(1.1,(999.,999.)))
 np.testing.assert_allclose(c._scan_ground_displacement(),[.01,0.])

def test_stale_velocity_cannot_align_a_scan(controller):
 c,_=controller;c._now=lambda:1.1;c._scan_t=1.;c._pose_t=1.1
 c._on_velocity(velocity(1.));c._on_velocity(velocity(1.1));c._now=lambda:3.
 with pytest.raises(ValueError):c._scan_ground_displacement()
