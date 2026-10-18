"""Recorded custom camera tilt must not leak across the normal return approach."""
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'sim'))
from unittest.mock import MagicMock
import py_trees
from mission_bt.mission_tree import build_root
import test_fail_closed_stages as fixtures
from test_leg_routing import RoutedMav

def handoff():
    mav=RoutedMav(at=(20.1,-16.1,5.0))
    mav._yaw=-math.pi
    mav.outbound_banner_xy=(6.3,-3.4)
    mav.camera_state=dict(requested='-10.0deg',actual_rad=-math.pi/18,settled=True,stale=False)
    camera_commands=[]
    def command(pose):
        camera_commands.append(pose)
        mav.camera_state['requested']=pose
    mav.set_camera_pose=command
    mav.camera_settled=lambda pose: (pose=='BANNER' and mav.camera_state['settled'] is True
        and mav.camera_state['stale'] is False and abs(mav.camera_state['actual_rad']+math.pi/9)<1e-6)
    mav.camera_state_summary=lambda:str(mav.camera_state)
    tree=build_root(mav,MagicMock(),dict(fixtures.BuiltTreeCarriesTheNewBehaviourTests.PARAMS))
    mission=next(n for n in tree.children if n.name=='Mission')
    first=next(i for i,n in enumerate(mission.children) if n.name=='FindReturnBanner')
    # The return gate is one selector over the hung, ground and fallback
    # passes; its first pass is the hung view this handoff restores BANNER for.
    end=next(i for i,n in enumerate(mission.children) if i>first and n.name.endswith('BannerAlignment'))
    children=mission.children[first:end+1]
    for child in children:mission.remove_child(child)
    sequence=py_trees.composites.Sequence('ActualReturnHandoff',memory=True)
    sequence.add_children(children)
    finder=children[0];finder.initialise()
    finder.phase='lower';finder.here=mav.pos()[:2];finder._cur_alt=5.
    finder.headings=[-math.pi];finder._look='-10.0deg';finder._then='done'
    finder._since=finder.clock();finder.status=py_trees.common.Status.RUNNING
    sequence.status=py_trees.common.Status.RUNNING
    sequence.current_child=finder
    # The real lower/approach handoff has measured its inherited search tilt.
    original_settled=mav.camera_settled
    mav.camera_settled=lambda pose:True if pose=='-10.0deg' else original_settled(pose)
    entered=[]
    hung=children[-1].children[0]
    leaf=next(c for c in hung.children if hasattr(c,'decorated')).decorated
    leaf.update=lambda:(entered.append(mav.camera_state.copy()) or py_trees.common.Status.RUNNING)
    mav.hung_pass=hung
    return sequence,mav,camera_commands,entered

def test_normal_return_approach_restores_banner_camera_and_waits_for_measured_settlement():
    sequence,mav,commands,entered=handoff()
    before=mav.pos()
    sequence.tick_once()
    assert not entered,'Alignment ran at the inherited -10 degree search tilt after the approach'
    assert commands[-1]=='BANNER'
    assert mav.pos()==before,'Camera handoff cannot move the aircraft'
    mav.camera_state.update(actual_rad=-math.pi/9,settled=True,stale=False)
    sequence.tick_once()
    assert len(entered)==1

def test_stale_camera_readback_cannot_release_return_alignment():
    sequence,mav,_,entered=handoff()
    sequence.tick_once()
    mav.camera_state.update(actual_rad=-math.pi/9,settled=True,stale=True)
    sequence.tick_once()
    assert not entered
    assert sequence.status is py_trees.common.Status.RUNNING

def test_wrong_measured_angle_cannot_release_even_if_settled_flag_is_true():
    sequence,mav,_,entered=handoff()
    sequence.tick_once()
    sequence.tick_once()
    assert not entered
    assert sequence.status is py_trees.common.Status.RUNNING

def test_camera_handoff_keeps_existing_bounded_failure_and_does_not_move_aircraft():
    sequence,mav,_,entered=handoff()
    before=mav.pos()
    hung=mav.hung_pass
    for _ in range(102):
        sequence.tick_once()
        if hung.status is py_trees.common.Status.FAILURE:
            break
    # The hung view gives up on its bounded camera timeout without ever
    # aligning at the wrong tilt; the gate then goes on to its other views.
    assert hung.status is py_trees.common.Status.FAILURE
    assert not entered and mav.pos()==before
    assert 'camera did not reach BANNER' in mav.abort_reason
