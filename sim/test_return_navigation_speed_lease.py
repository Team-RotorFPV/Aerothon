"""The flown mission must release the search cap before its high return leg."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock
import py_trees
import time
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'sim'))
import test_fail_closed_stages as fixtures
from test_leg_routing import RoutedMav
from mission_bt.mission_tree import build_root


def test_actual_return_sequence_restores_the_pre_search_navigation_cap():
    mav=RoutedMav(at=(46.,0.,10.));mav.speed_cmd=5.
    mav.set_speed=lambda speed:restore(mav,speed)
    mav.corridor_exit_pose=(16.,0.,3.,0.)
    mav.delivery_search_zone=lambda inset:(13.+inset,53.-inset,-15.+inset,15.-inset)
    mav.camera_settled=lambda pose:True
    mav.delivery_inputs={'pose_stamp':time.monotonic()}
    tree=build_root(mav,MagicMock(),dict(fixtures.BuiltTreeCarriesTheNewBehaviourTests.PARAMS))
    mission=next(n for n in tree.iterate() if n.name=='Mission')
    search=next(n for n in tree.iterate() if n.name=='LawnmowerSearch')
    search.initialise()
    # Search legitimately admitted the low camera-limited cap. Other flight
    # physics and path routing stay unchanged when its lease ends.
    mav.speed_cmd=.65
    first=next(i for i,n in enumerate(mission.children) if n.name=='ClimbForReturn')
    for leaf in mission.children[first:]:
        leaf.initialise()
        for _ in range(5):
            status=leaf.update()
            if status is not py_trees.common.Status.RUNNING:break
        if leaf.name=='ReturnToCorridorMouth':
            assert status is py_trees.common.Status.RUNNING
            assert mav.speed_cmd==5., 'search camera speed cap leaked into the high return transit'
            assert mav.gotos[-1][2]==10.
            return
        assert status is py_trees.common.Status.SUCCESS
    raise AssertionError('normal return sequence was not exercised')


def restore(mav,speed):
    mav.speed_cmd=speed
    return SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True))
