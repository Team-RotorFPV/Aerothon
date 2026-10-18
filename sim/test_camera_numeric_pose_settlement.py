"""Numeric commands use camera_ctrl's canonical measured-angle readback."""
import math
import pytest
from mission_bt.mav_commander import Mav


def commander():
    mav=object.__new__(Mav)
    mav.camera_state=dict(requested='-10.0deg',requested_rad=-math.radians(10),
        actual_rad=-math.radians(10),settled=True,stale=False)
    return mav


def test_numeric_pose_matches_the_existing_camera_controller_readback():
    mav=commander();assert mav.camera_settled('-10.0')
    assert not mav.camera_settled('BANNER')
    assert not mav.camera_settled('-15')


@pytest.mark.parametrize('invalid',['unsettled','stale','missing_angle','wrong_angle','nan','invalid_command'])
def test_numeric_request_cannot_infer_unmeasured_or_wrong_camera_settlement(invalid):
    mav=commander();command='-10'
    if invalid=='unsettled':mav.camera_state['settled']=False
    if invalid=='stale':mav.camera_state['stale']=True
    if invalid=='missing_angle':mav.camera_state.pop('requested_rad')
    if invalid=='wrong_angle':mav.camera_state['requested_rad']=-math.radians(20)
    if invalid=='nan':mav.camera_state['requested_rad']=math.nan
    if invalid=='invalid_command':command='no angle'
    assert not mav.camera_settled(command)
