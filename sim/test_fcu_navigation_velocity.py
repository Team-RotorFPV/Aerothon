"""Navigation braking must use fresh onboard EKF velocity when available."""
import time
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped,TwistStamped
from rclpy.qos import qos_profile_sensor_data
from test_corridor_clearance_contract import controller
from std_msgs.msg import Bool
import numpy as np


def velocity(c,stamp,x,y):
    pub=c.create_publisher(TwistStamped,'/mavros/local_position/velocity_local',qos_profile_sensor_data)
    m=TwistStamped();m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=int(round((stamp-int(stamp))*1e9));m.header.frame_id='base_link'
    m.twist.linear.x=x;m.twist.linear.y=y
    until=time.monotonic()+.4
    while time.monotonic()<until:
        pub.publish(m);rclpy.spin_once(c,timeout_sec=.01)
        if getattr(c,'_fcu_velocity_t',None)==pytest.approx(stamp):break


def test_fresh_ekf_velocity_takes_priority_over_pose_differences(controller):
    c,_=controller;c._compensated(0.,0.)
    c._now=lambda:1.1;p=PoseStamped();p.pose.orientation.w=1.;p.pose.position.y=.03;c._on_pose(p)
    velocity(c,1.1,0.,.1);c._compensated(0.,0.)
    assert c._ground_world[1]==pytest.approx(.1)


def test_fresh_velocity_updates_even_while_pose_header_is_held(controller):
    c,_=controller;c._compensated(0.,0.);c._now=lambda:1.05
    velocity(c,1.05,.2,0.);c._compensated(0.,0.)
    assert c._ground_world[0]==pytest.approx(.2)


def test_stale_ekf_velocity_stops_despite_fresh_pose_and_lidar(controller):
    c,s=controller;velocity(c,1.,.2,0.)
    c._now=lambda:1.8;p=PoseStamped();p.pose.orientation.w=1.;c._on_pose(p)
    c.scan.header.stamp.sec=1;c.scan.header.stamp.nanosec=800000000;c._on_scan(c.scan);c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0.
    assert 'velocity stale' in c.last_detail.get('fault','')


def test_control_handover_retains_observed_incoming_motion(controller):
    c,_=controller;velocity(c,1.,.3,-.1)
    c._on_enable(Bool(data=False));c._on_enable(Bool(data=True))
    np.testing.assert_allclose(c._ground_world,[.3,-.1])
    np.testing.assert_allclose(c._expect_world,[.3,-.1])


def test_nonfinite_velocity_stops_navigation(controller):
    c,s=controller;velocity(c,1.,float('nan'),0.);c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0.
    assert 'invalid FCU velocity' in c.last_detail.get('fault','')


def test_repeated_old_velocity_stamp_cannot_refresh_freshness(controller):
    c,s=controller;velocity(c,1.,.2,0.);c._now=lambda:1.6
    velocity(c,1.,.2,0.);assert c._fcu_velocity_t==1.
    c._now=lambda:1.8;p=PoseStamped();p.pose.orientation.w=1.;c._on_pose(p)
    c.scan.header.stamp.sec=1;c.scan.header.stamp.nanosec=800000000;c._on_scan(c.scan);c._tick()
    assert 'velocity stale' in c.last_detail.get('fault','')
