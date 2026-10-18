"""Capture freshness and OCR continuity at the camera/mission boundary."""
import json
import math
from pathlib import Path
import sys
import time
import unittest
from std_msgs.msg import String

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_mission/mission_bt'))
sys.path.insert(0,str(ROOT/'src/aerothon_perception/perception_banner'))
sys.path.insert(0,str(ROOT/'sim'))
from mission_bt.banner_captures import BannerCaptures
from perception_banner.banner_node import BannerNode
from test_banner_sweep import Clock,SweepMav,AlignToBanner


class BannerCaptureTests(unittest.TestCase):
    def test_delayed_yaw_keeps_the_capture_world_ray(self):
        captures=BannerCaptures()
        captures.pose(1.,(0,0,5),0.)
        self.assertTrue(captures.receive(dict(capture_stamp=1.,processed_stamp=1.3,
                                              bearing=0.,identified=True),1.3))
        fov=math.radians(60)
        self.assertAlmostEqual(captures.bearing_now(1.3,math.radians(15),fov),
                               math.tan(math.radians(15))/math.tan(fov/2))
        self.assertIsNone(captures.bearing_now(1.3,math.radians(40),fov))

    def test_pose_pairing_does_not_extrapolate_across_missing_samples(self):
        captures=BannerCaptures()
        captures.pose(1.,(0,0,5),math.radians(179))
        captures.pose(1.2,(2,0,5),math.radians(-179))
        pose=captures.at(1.1)
        self.assertAlmostEqual(pose['xyz'][0],1.)
        self.assertAlmostEqual(abs(pose['yaw']),math.pi)
        self.assertIsNone(captures.at(1.3))
        captures.pose(1.8,(8,0,5),0.)
        self.assertIsNone(captures.at(1.5))

    def test_replays_missing_stamps_and_stale_latched_results_cannot_qualify(self):
        captures=BannerCaptures();captures.pose(1.,(0,0,5),0.)
        detail=dict(capture_stamp=1.,processed_stamp=1.1,identified=True,bearing=0.)
        self.assertTrue(captures.receive(detail,1.1))
        self.assertFalse(captures.receive(detail,1.2))
        self.assertEqual(captures.sequence,1)
        self.assertIsNone(captures.current(1.8))
        self.assertFalse(captures.receive({'identified':True},1.2))
        self.assertIsNone(captures.current(1.2))

    def test_dwell_counts_distinct_frames_captured_after_the_turn(self):
        clock=Clock();clock.t=1.
        mav=SweepMav(banner_at=0.)
        observation=dict(sequence=1,capture_stamp=1.1)
        mav.banner_observation=lambda:observation
        stage=AlignToBanner(mav,clock=clock,min_samples=6)
        stage._anchor=mav.pos();stage._target_yaw=0.
        stage._enter(stage.DWELL)
        for _ in range(5):
            clock.advance(.1);stage._dwell()
        self.assertEqual(stage._samples,1)
        self.assertEqual(stage._hits,1)
        observation.update(sequence=2,capture_stamp=.9)
        clock.advance(.1);stage._dwell()
        self.assertEqual(stage._samples,1)
        observation.update(sequence=3,capture_stamp=1.7)
        clock.advance(.1);stage._dwell()
        self.assertEqual(stage._samples,2)


class BannerImageContinuityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():rclpy.shutdown()

    def setUp(self):
        self.node=BannerNode();self.details=[]
        self.node._colours = {'green':'#07A03D','orange':'#EB5A0A','red':'#C4161C'}
        self.node.pub.publish=lambda _:None
        self.node.pub_detail.publish=lambda msg:self.details.append(json.loads(msg.data))

    def tearDown(self):self.node.destroy_node()

    def feed(self,name):
        image=cv2.imread(str(ROOT/'tests/fixtures'/name))
        self.assertIsNotNone(image)
        return self.feed_image(image)

    def feed_image(self,image):
        message=CvBridge().cv2_to_imgmsg(image,encoding='bgr8')
        message.header.stamp=self.node.get_clock().now().to_msg()
        self.node.on_image(message)
        return message

    def test_clipped_board_is_read_from_a_cold_start(self):
        # The severity-5 exposure gain clips green and shifts its hue to 86.
        image=cv2.imread(str(ROOT/'tests/fixtures/banner_sim_ambient_3m.png'))
        clipped=np.clip(image.astype(np.float32)*2.**1.75,0,255).astype(np.uint8)
        self.feed_image(clipped)
        self.assertTrue(self.details[-1]['identified'])
        self.assertTrue(self.details[-1]['text_confirmed'])
        self.assertTrue(self.details[-1]['rescued_by_text'])

    def test_clipped_color_and_white_blocks_cannot_replace_the_word(self):
        image=np.full((480,640,3),(90,70,55),dtype=np.uint8)
        cv2.rectangle(image,(150,180),(490,300),(255,255,0),-1)
        for index in range(8):
            cv2.rectangle(image,(175+index*36,215),(190+index*36,265),(255,255,255),-1)
        self.feed_image(image)
        self.assertFalse(self.details[-1]['identified'])
        self.assertFalse(self.details[-1]['text_confirmed'])

    def test_a_cached_word_cannot_identify_a_blank_panel_at_the_same_box(self):
        self.feed('banner_sim_ambient_3m.png')
        self.assertTrue(self.details[-1]['text_confirmed'])
        x,y,w,h=self.details[-1]['board_px']
        image=np.full((720,1280,3),(90,70,55),dtype=np.uint8)
        cv2.rectangle(image,(x,y),(x+w-1,y+h-1),(60,170,60),-1)
        self.node._ocr_t0=time.monotonic()
        self.feed_image(image)
        self.assertFalse(self.details[-1]['identified'])

    def test_a_previous_word_read_cannot_identify_a_blank_current_frame(self):
        self.feed('banner_sim_ambient_3m.png')
        self.assertTrue(self.details[-1]['text_confirmed'])
        self.node._ocr_t0=time.monotonic()
        self.feed('banner_absent_horizon.png')
        self.assertFalse(self.details[-1]['identified'])
        self.assertFalse(self.details[-1]['text_confirmed'])

    def test_display_annotations_never_enter_the_word_reader(self):
        image=cv2.imread(str(ROOT/'tests/fixtures/banner_sim_ambient_3m.png'))
        self.assertIsNotNone(image)
        namespace=self.node.on_image.__func__.__globals__
        normalized=namespace['normalise'](image.copy())
        reader=namespace['reads_banner']
        calls=[]
        def observed_reader(sub,*args,**kwargs):
            x,y,w,h=self.node._ocr_box
            self.assertTrue(np.array_equal(sub,normalized[y:y+h,x:x+w]),
                            'OCR received display pixels absent from the camera')
            calls.append((x,y,w,h))
            return reader(sub,*args,**kwargs)
        namespace['reads_banner']=observed_reader
        try:
            self.feed_image(image)
        finally:
            namespace['reads_banner']=reader
        self.assertTrue(calls,'The fixture must exercise the actual word reader')

    def test_duplicate_image_capture_does_not_generate_new_evidence(self):
        image=self.feed('banner_absent_horizon.png')
        count=len(self.details)
        self.node.on_image(image)
        self.assertEqual(len(self.details),count)
        stamp=image.header.stamp.sec+image.header.stamp.nanosec*1e-9
        self.assertEqual(self.details[-1]['capture_stamp'],stamp)

    def test_camera_turning_away_invalidates_the_previous_word_read(self):
        self.feed('banner_sim_ambient_3m.png')
        self.assertTrue(self.node._ocr_cache['text_confirmed'])
        self.node.gate._on_state(String(data=json.dumps(dict(requested='NADIR'))))
        self.feed('banner_sim_ambient_3m.png')
        self.assertFalse(self.details[-1]['identified'])
        self.assertFalse(self.node._ocr_cache['text_confirmed'])


if __name__=='__main__':unittest.main()
