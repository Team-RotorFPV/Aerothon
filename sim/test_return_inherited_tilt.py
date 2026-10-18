"""Missing return identity at the inherited down tilt merits a level look first."""
import math
from test_return_fine_framing import stage

def prepare():
 leaf,mav=stage();now=[1.]
 leaf.clock=lambda:now[0];leaf._anchor=mav.pos();leaf._target_yaw=0.;leaf._enter(leaf.DWELL)
 mav.camera_state=dict(requested='BANNER',actual_rad=-math.radians(20),settled=True,stale=False)
 mav.camera_settled=lambda pose: mav.camera_state['requested']==pose and mav.camera_state['settled']
 o=dict(sequence=1,capture_stamp=1.,identified=False,bearing=0.,pose=dict(xyz=list(mav.pos()),yaw=0.))
 mav.banner_observation=lambda:o
 return leaf,mav,o,now

def test_empty_informed_return_dwell_tries_level_camera_at_same_pose():
 leaf,mav,o,now=prepare();before=mav.pos()
 for seq in (1,2,3):
  o.update(sequence=seq,capture_stamp=now[0]);leaf._dwell();now[0]+=.1
 assert mav.camera_poses[-1]=='FORWARD'
 assert leaf._anchor==before and leaf._good_vantage is None
 assert leaf._relocations==0

def test_duplicate_capture_cannot_trigger_level_retry():
 leaf,mav,o,now=prepare()
 for _ in range(3):leaf._dwell()
 assert not mav.camera_poses

def test_top_cropped_level_return_identity_recovers_height_during_search_dwell():
 leaf,mav,o,now=prepare();leaf._anchor=(20.,-16.6,5.2)
 mav.camera_state=dict(requested='FORWARD',actual_rad=0.,settled=True,stale=False)
 mav.banner_identified=lambda:True;mav.banner_clipped=True
 o.update(identified=True,board_px=[0,0,1280,360],image_wh=[1280,720])
 for seq in (1,2,3):
  o.update(sequence=seq,capture_stamp=now[0]);leaf._dwell();now[0]+=.1
 assert leaf.phase is leaf.RELOCATE
 assert leaf._anchor==(20.,-16.6,5.7)
 assert leaf._good_vantage is None
