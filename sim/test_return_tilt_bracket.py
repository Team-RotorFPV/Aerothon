"""A real search leaf must bracket opposing crops without translating."""
import sys,math
import pytest
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'sim'))
from test_return_clipped_framing import setup


def test_opposing_fresh_vertical_crops_use_an_intermediate_camera_angle():
    stage,mav,observation,now=setup();stage._look='FORWARD'
    mav.set_camera_pose('FORWARD');before=mav.pos()
    mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    observation['board_px']=[432,399,445,320];observation['bearing']=.024
    for sequence in (1,2,3):
        now[0]=1.+sequence*.1
        observation.update(sequence=sequence,capture_stamp=now[0]);stage.update()
    assert stage.phase=='clipped_focus' and stage._look=='BANNER'
    mav.camera_state['actual_rad']=-math.radians(20)
    stage.update()
    # V23's same identified board leaves the opposite edge at this tilt.
    observation['board_px']=[494,0,424,326]
    for sequence in (4,5,6):
        now[0]=1.+sequence*.1
        observation.update(sequence=sequence,capture_stamp=now[0]);stage.update()
    assert stage._look not in ('BANNER','FORWARD'), 'search oscillates between two camera tilts that both crop the board'
    assert float(stage._look)==-10.
    assert mav.pos()==before and stage._cur_alt==before[2]
    assert stage._sighting() is None, 'clipped geometry still cannot accept the gate'


@pytest.mark.parametrize('invalid',['duplicate','stale','unidentified','unsettled','changed_bearing','moved_pose','side_crop','both_vertical_edges','exhausted','invalid_angle'])
def test_unqualified_opposing_crop_never_commands_an_intermediate_angle(invalid):
    stage,mav,o,now=setup();stage._look='FORWARD';mav.set_camera_pose('FORWARD')
    mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    o.update(board_px=[432,399,445,320],bearing=.024)
    for seq in (1,2,3):
        now[0]=1.+seq*.1;o.update(sequence=seq,capture_stamp=now[0]);stage.update()
    mav.camera_state['actual_rad']=-math.radians(20);stage.update()
    o['board_px']=[494,0,424,326]
    if invalid=='unidentified':o['identified']=False
    if invalid=='unsettled':mav.camera_state['settled']=False
    if invalid=='changed_bearing':o['bearing']=.8
    if invalid=='moved_pose':o['pose']['xyz'][0]+=2.
    if invalid=='side_crop':o['board_px'][0]=0
    if invalid=='both_vertical_edges':o['board_px']=[494,0,424,720]
    if invalid=='exhausted':stage._tilt_crop_adjustments=3
    if invalid=='invalid_angle':mav.camera_state['actual_rad']=.5
    for seq in (4,5,6):
        now[0]=1.+seq*.1
        o.update(sequence=4 if invalid=='duplicate' else seq,
                 capture_stamp=1. if invalid=='stale' else now[0])
        stage.update()
    assert stage._look in ('BANNER','FORWARD')
    assert stage._sighting() is None


def test_camera_bracket_has_at_most_three_bounded_adjustments():
    stage,mav,o,now=setup();stage._since=1.
    mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    o.update(board_px=[432,399,445,320],bearing=0.,capture_stamp=1.)
    assert stage._bracket_clipped_camera(o,0.) is None
    mav.camera_state['actual_rad']=-math.radians(20);o['board_px']=[494,0,424,326]
    assert stage._bracket_clipped_camera(o,0.)=='-10.0'
    mav.camera_state['actual_rad']=-math.radians(10);o['board_px']=[432,399,445,320]
    assert stage._bracket_clipped_camera(o,0.)=='-15.0'
    mav.camera_state['actual_rad']=-math.radians(15);o['board_px']=[494,0,424,326]
    assert stage._bracket_clipped_camera(o,0.)=='-12.5'
    mav.camera_state['actual_rad']=-math.radians(12.5);o['board_px']=[432,399,445,320]
    assert stage._bracket_clipped_camera(o,0.) is None
    assert stage._tilt_crop_adjustments==3
