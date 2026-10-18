"""Clipped captures may guide a camera-only retry, never a range or translation."""
from pathlib import Path
import sys
from mission_bt.mission_tree import FindReturnBanner as FramedReturnBanner
from test_return_gate_search import World

def setup():
    now=[1.]
    mav=World([],start=(23.,-6.,0.))
    mav.banner_clipped=True
    mav.banner_identified=lambda: True
    mav.camera_settled=lambda pose: bool(mav.camera_poses and mav.camera_poses[-1]==pose)
    observation=dict(sequence=1,capture_stamp=1.1,identified=True,bearing=-.8,
                     pose=dict(yaw=0.,xyz=list(mav.pos())),board_px=[0,0,200,200],image_wh=[1280,720])
    mav.banner_observation=lambda: observation
    stage=FramedReturnBanner(mav,clock=lambda:now[0],look_pose="BANNER");stage.initialise()
    stage.phase='sweep';stage._turning=False;stage._since=1.;stage.hi=0;stage.headings=[0.]
    stage.here=mav.pos()[:2]
    return stage,mav,observation,now

def test_distinct_clipped_captures_centre_without_range_or_translation():
    stage,mav,observation,now=setup()
    before=mav.pos()
    for sequence in (1,2,3):
        now[0]=1.+sequence*.1
        observation.update(sequence=sequence,capture_stamp=now[0])
        stage.update()
    assert stage.phase=='clipped_focus'
    assert mav.pos()==before
    assert mav.camera_poses[-1]=='FORWARD'
    assert stage._sighting() is None
    assert stage._focus['yaw']>0.

def test_duplicate_capture_cannot_start_focus():
    stage,mav,observation,now=setup()
    for _ in range(5):stage.update()
    assert stage.phase=='sweep' and stage._clip_samples==1

def test_predwell_capture_cannot_start_focus():
    stage,mav,observation,now=setup()
    observation['capture_stamp']=.9
    for sequence in (1,2,3):
        observation['sequence']=sequence;stage.update()
    assert stage.phase=='sweep' and stage._clip_samples==0

def test_focus_requires_camera_and_yaw_settlement():
    stage,mav,observation,now=setup()
    for sequence in (1,2,3):
        now[0]=1.+sequence*.1;observation.update(sequence=sequence,capture_stamp=now[0]);stage.update()
    mav.camera_settled=lambda pose: False
    stage.update()
    assert stage.phase=='clipped_focus'
    mav.camera_settled=lambda pose: True
    stage.update()
    assert stage.phase=='sweep' and stage._hits==0
    assert stage._sighting() is None

def test_level_camera_top_crop_raises_stationary_search_before_using_clipped_range():
 stage,mav,observation,now=setup()
 stage._look='FORWARD';mav.set_camera_pose('FORWARD')
 mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
 observation['board_px']=[300,0,450,150]
 before=mav.pos();alt=stage._cur_alt
 for sequence in (1,2,3):
  now[0]=1.+sequence*.1;observation.update(sequence=sequence,capture_stamp=now[0]);stage.update()
 assert stage.phase=='clipped_focus'
 assert stage._cur_alt==alt+.5
 assert mav.pos()[:2]==before[:2]
 assert stage._sighting() is None
