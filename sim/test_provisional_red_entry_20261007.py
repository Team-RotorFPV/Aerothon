"""Exercise actual camera callback and mission consumer with bounded evidence."""
import json
import math
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from perception_redzone.redzone_node import RedZoneNode
from mission_bt.mav_commander import Mav
from mission_bt.detector_timing import DetectorTiming


@pytest.fixture
def camera():
    rclpy.init()
    node = RedZoneNode()
    node._supplied = {'green':'#07A03D','orange':'#EB5A0A','red':'#C4161C'}
    node._red_hex = '#C4161C'
    node._cam_pitch = math.pi/2
    node._hfov = .851919
    messages = []
    node.pub_detail.publish = messages.append
    yield node, messages
    node.destroy_node()
    rclpy.shutdown()


def partial_frame():
    frame = np.full((480, 640, 3), 80, np.uint8)
    frame[100:240, :100] = 255
    frame[110:230, :6] = (0, 0, 255)
    return frame


def expose(node, frame, stamp=None):
    pose = PoseStamped()
    pose.header.stamp = stamp or node.get_clock().now().to_msg()
    pose.pose.position.z = 10.
    pose.pose.orientation.w = 1.
    node.on_pose(pose)
    message = node.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    message.header.stamp = pose.header.stamp
    node.on_image(message)
    return message


def test_clipped_boundary_reaches_mission_before_persistent_confirmation(camera):
    node, messages = camera
    expose(node, partial_frame())
    detail = json.loads(messages[-1].data)
    assert detail.get('provisional_exclusions'), 'a clipped red boundary must reach the mission before confirmation'
    assert detail['status'] == 'RED'
    assert detail['blobs'] == 0, 'the unchanged narrow-feature filter still rejects this partial strip'
    assert detail['exclusions'] == []
    assert detail['confirmed_area_m2'] == 0.
    assert detail['provisional_confirmed'] is False


def test_repeated_camera_capture_cannot_accumulate_confirmation(camera):
    node, messages = camera
    message = expose(node, partial_frame())
    for _ in range(5):
        node.on_image(message)
    assert len(messages) == 1
    assert node.grid.area_m2() == 0.


def test_unprojectable_capture_cannot_create_provisional_obstacle(camera):
    node, messages = camera
    message = expose(node, partial_frame())
    node._poses.clear()
    # Float seconds at the epoch cannot distinguish a one-nanosecond change.
    message.header.stamp.nanosec += 1_000_000
    if message.header.stamp.nanosec >= 1_000_000_000:
        message.header.stamp.sec += 1
        message.header.stamp.nanosec -= 1_000_000_000
    node.on_image(message)
    detail = json.loads(messages[-1].data)
    assert detail['status'] == 'NOT_VISIBLE'
    assert detail.get('provisional_exclusions', []) == []


def consumer(now=2.):
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now*1e9)))
    return SimpleNamespace(node=SimpleNamespace(get_clock=lambda: clock),
                           detector_timing={'red': DetectorTiming()})


def send(mav, capture=1.9, provisional=None, confirmed=None):
    Mav._on_redzone(mav, String(data=json.dumps(dict(status='RED',
        capture_stamp=capture, processed_stamp=2., provisional_exclusions=provisional or [], exclusions=confirmed or []))))


def test_fresh_provisional_evidence_affects_routing_without_becoming_confirmed():
    mav = consumer()
    send(mav, provisional=[(10, 12, -1, 1)])
    assert mav.exclusions == [(10., 12., -1., 1.)]
    assert mav.confirmed_exclusions == []
    send(mav)
    assert mav.exclusions == []


@pytest.mark.parametrize('capture', [.5, 3., float('nan')])
def test_stale_future_or_nonfinite_provisional_evidence_is_discarded(capture):
    mav = consumer()
    send(mav, capture=capture, provisional=[(10, 12, -1, 1)], confirmed=[(20, 22, -1, 1)])
    assert mav.exclusions == [(20., 22., -1., 1.)]


def test_malformed_provisional_geometry_is_discarded():
    mav = consumer()
    send(mav, provisional=[(1, 2, 3), (2, 1, 0, 1), (1, 2, 0, float('nan'))])
    assert mav.exclusions == []
