"""Delayed lidar geometry uses capture-time aircraft pose."""
import math
from test_corridor_clearance_contract import controller
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


def test_scan_remembers_capture_pose_not_delivery_pose(controller):
    c,_=controller
    now=[1.0];c._now=lambda:now[0]
    p=PoseStamped();p.pose.orientation.w=1.0;c._on_pose(p)
    now[0]=1.15;p.pose.position.x=.1;c._on_pose(p)
    scan=LaserScan();scan.header.stamp.sec=1;scan.ranges=[2.0]*500
    scan.header.frame_id='base_link'
    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/500
    c._on_scan(scan)
    assert c._scan_pose==(0.0,0.0,0.0)
    assert abs(c._scan_t-1)<1e-9


def test_velocity_estimate_uses_pose_time_not_assumed_timer_ticks(controller):
    c,_=controller
    now=[1.0];c._now=lambda:now[0]
    c._compensated(0,0)
    now[0]=1.2
    p=PoseStamped();p.pose.orientation.w=1.0;p.pose.position.y=.1
    c._on_pose(p);c._compensated(0,0)
    assert abs(c._ground_world[1]-.5)<1e-9
