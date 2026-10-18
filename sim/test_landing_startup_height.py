"""Normal startup reaches the camera's tracking height before home descent."""
import math
from unittest.mock import MagicMock
import pytest
import py_trees
import test_fail_closed_stages as fixtures
from test_leg_routing import RoutedMav
from mission_bt.mission_tree import build_root


def stages(marker_m,hfov,at=(2.,0.,1.85)):
    params=dict(fixtures.BuiltTreeCarriesTheNewBehaviourTests.PARAMS,
                target_marker_m=marker_m,camera_hfov=hfov,image_height_px=720)
    mav=RoutedMav(at=at)
    root=build_root(mav,MagicMock(),params)
    leaves=list(root.iterate())
    home=next(s for s in leaves if type(s).__name__=='GotoHome')
    landing=next(s for s in leaves if type(s).__name__=='PrecisionDescent')
    return mav,home,landing


@pytest.mark.parametrize('marker_m,hfov',[(3.1,.851919),(2.2,.851919),(2.2,1.0472),(.3,1.0472)])
def test_actual_home_command_leaves_room_to_acquire_the_whole_marker(marker_m,hfov):
    mav,home,landing=stages(marker_m,hfov)
    home.initialise();status=home.update()
    assert status is py_trees.common.Status.RUNNING
    assert mav.pos()[2]==1.85  # position command is not a teleport
    altitude=mav.gotos[-1][2]
    half_vertical=altitude*math.tan(hfov/2)*720/1280
    assert half_vertical>marker_m/2
    assert altitude>=landing.start_alt
    assert altitude>landing.commit_alt


def test_the_flown_landing_leaf_gets_a_visual_tick_before_commit():
    mav,home,landing=stages(3.1,.851919,at=(0.,0.,5.))
    home.initialise();home.update();mav.arrive()
    mav.qr_off=(0.,0.)
    landing.initialise()
    assert landing.update() is py_trees.common.Status.RUNNING
    assert mav.landing_precision=='UNKNOWN'
    assert landing._centred_xy is not None
    assert mav.gotos[-1][2]<mav.pos()[2]
    assert mav.gotos[-1][2]>=landing.commit_alt
