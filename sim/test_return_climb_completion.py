"""The built mission climb must satisfy its high-return speed guard."""
import time
from unittest.mock import MagicMock
import pytest
import py_trees
import test_fail_closed_stages as fixtures
from test_leg_routing import RoutedMav
from test_return_navigation_speed_lease import restore
from mission_bt.mission_tree import build_root


@pytest.mark.parametrize('measured_alt',[9.41,9.50,9.69])
def test_actual_return_climb_waits_until_speed_restore_height_is_safe(measured_alt):
    mav=RoutedMav(at=(46.,0.,measured_alt));mav.speed_cmd=.65
    mav.set_speed=lambda speed:restore(mav,speed)
    mav.camera_settled=lambda pose:True
    mav.delivery_inputs={'pose_stamp':time.monotonic()}
    tree=build_root(mav,MagicMock(),dict(fixtures.BuiltTreeCarriesTheNewBehaviourTests.PARAMS))
    climb=next(n for n in tree.iterate() if n.name=='ClimbForReturn')
    speed=next(n for n in tree.iterate() if n.name=='RestoreNavigationSpeed')
    climb.initialise()
    assert climb.update() is py_trees.common.Status.RUNNING, 'normal climb completes below the following guard minimum'
    assert mav.gotos[-1][2]==speed.transit_alt==10.
    assert mav.pos()[2]==measured_alt and mav.speed_cmd==.65
    # Supply a fresh measured response. goto never teleports this aircraft.
    mav._pos=(46.,0.,9.81);mav.delivery_inputs['pose_stamp']=time.monotonic()
    assert climb.update() is py_trees.common.Status.SUCCESS
    mav.navigation_speed_before_search=5.
    search=next(n for n in tree.iterate() if n.name=='LawnmowerSearch')
    search.navigation_speed_before_search=5.
    speed.initialise()
    assert speed.update() is py_trees.common.Status.SUCCESS
    assert mav.speed_cmd==5.
