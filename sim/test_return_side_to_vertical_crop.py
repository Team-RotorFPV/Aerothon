"""Actual leaf: centring a side crop must leave vertical framing available."""
import math
import pytest
from test_return_clipped_framing import setup


def test_side_crop_does_not_consume_centred_vertical_camera_recovery():
    stage,mav,o,now=setup();stage._look='FORWARD'
    mav.set_camera_pose('FORWARD');mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    before=mav.pos();o.update(board_px=[0,399,445,320],bearing=.30)
    for seq in (1,2,3):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    assert stage.phase=='clipped_focus' and stage._look=='BANNER'
    mav.camera_state['actual_rad']=-math.radians(20);stage.update()
    # The bearing is now centred; BANNER crops the top, so return to FORWARD.
    o.update(board_px=[494,0,424,326],bearing=.01)
    for seq in (4,5,6):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    assert stage._look=='FORWARD'
    mav.camera_state['actual_rad']=0.;stage.update()
    # This complete horizontal view is new framing evidence at the same tilt.
    o.update(board_px=[432,399,445,320],bearing=.01)
    for seq in (7,8,9):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    assert stage._look=='-10.0', 'initial side crop consumed the centred vertical framing attempt'
    assert mav.pos()==before
    assert stage._sighting() is None


@pytest.mark.parametrize('invalid',['duplicate','predwell','unsettled'])
def test_centred_crop_still_requires_distinct_fresh_settled_evidence(invalid):
    stage,mav,o,now=setup();stage._look='FORWARD'
    mav.set_camera_pose('FORWARD');mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    o.update(board_px=[0,399,445,320],bearing=.30)
    for seq in (1,2,3):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    mav.camera_state['actual_rad']=-math.radians(20);stage.update()
    o.update(board_px=[494,0,424,326],bearing=.01)
    for seq in (4,5,6):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    mav.camera_state['actual_rad']=0.;stage.update()
    o.update(board_px=[432,399,445,320],bearing=.01)
    if invalid=='unsettled':mav.camera_state['settled']=False
    for seq in (7,8,9):
        now[0]=1.+seq*.1;o.update(sequence=7 if invalid=='duplicate' else seq,capture_stamp=1. if invalid=='predwell' else now[0]);stage.update()
    assert stage._look in ('FORWARD','BANNER')
    assert stage._sighting() is None
