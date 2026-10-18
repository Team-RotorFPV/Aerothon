"""Measured mounting changes clearance, independently of venue knowledge."""
import math
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/aerothon_avoidance/avoidance'))
from avoidance.scan_geometry import register_rays
from avoidance.local_planner import LocalPlanner


def test_forward_mount_does_not_overstate_rear_clearance():
    # A rear surface 0.65 m from the centre is 0.789 m from the forward lidar.
    origin,angles,depths=register_rays(np.array([math.pi]),np.array([.789]),
        (.139,0,.082),(0,0,0,1),(0,0),(0,0,0,1),(0,0),0)
    p=LocalPlanner();p.observe(angles,depths,sensor_origin=origin)
    assert p.distances([(0,0)])[0]==pytest.approx(.65)
    assert not p.segment_clear((0,0),(-.01,0))


def test_capture_attitude_and_mount_transform_are_applied_before_current_yaw():
    origin,a,r=register_rays(np.array([0.]),np.array([2.]),(.139,0,.082),
        (0,0,0,1),(1,2),(0,0,math.sin(math.pi/4),math.cos(math.pi/4)),(1,2.1),0)
    np.testing.assert_allclose(origin,(0,.039),atol=1e-12)
    assert a[0]==pytest.approx(math.pi/2)
    assert r[0]==pytest.approx(2)


def test_unknown_sector_stays_unknown_after_sensor_translation():
    p=LocalPlanner()
    p.observe([0],[3],sensor_origin=(.139,0))
    assert p.observed([(1,0)])[0]
    assert not p.observed([(-.1,0)])[0]


def test_missing_capture_pose_holds(controller):
    c,s=controller;c._scan_pose=None;c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0
    assert 'capture-time' in c.last_detail['fault']


def test_unknown_sensor_frame_holds(controller):
    c,s=controller;c.scan.header.frame_id='unqualified_mount';c._tick()
    assert s[-1].velocity.x==s[-1].velocity.y==0
    assert 'unqualified_mount' in c.last_detail['fault']


def test_controller_uses_available_onboard_mount_transform(controller):
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer
    from rclpy.parameter import Parameter
    c,s=controller
    c.set_parameters([Parameter('mask_sectors_deg',Parameter.Type.DOUBLE_ARRAY,[])])
    t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='base_scan'
    t.transform.translation.x=.139;t.transform.rotation.w=1.0
    c._tf_buffer=Buffer();c._tf_buffer.set_transform_static(t,'measured_test_mount')
    c.scan.header.frame_id='base_scan';c.scan.ranges=[.789]*360
    c._on_scan(c.scan);c._tick()
    assert c.last_detail['observed_clearance_m']==pytest.approx(.65,abs=.001)
    assert s[-1].velocity.x==s[-1].velocity.y==0


def test_mount_listener_does_not_subscribe_to_dynamic_vehicle_truth(controller):
    c,_=controller;c.scan.header.frame_id='base_scan';c._tick()
    topics={sub.topic_name for sub in c.subscriptions}
    assert '/tf_static' in topics
    assert '/tf' not in topics


def test_static_mount_callback_rejects_absolute_vehicle_transform(controller):
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rclpy.time import Time
    c,_=controller;c.scan.header.frame_id='base_scan';c._tick()
    absolute=TransformStamped();absolute.header.frame_id='odom'
    absolute.child_frame_id='base_link';absolute.transform.translation.x=999
    absolute.transform.rotation.w=1.
    mount=TransformStamped();mount.header.frame_id='base_link';mount.child_frame_id='base_scan'
    mount.transform.translation.x=.139;mount.transform.rotation.w=1.
    c._on_static_mounts(TFMessage(transforms=[absolute,mount]))
    assert not c._tf_buffer.can_transform('odom','base_link',Time())
    assert c._tf_buffer.lookup_transform('base_link','base_scan',Time()).transform.translation.x==pytest.approx(.139)


from test_corridor_clearance_contract import controller
