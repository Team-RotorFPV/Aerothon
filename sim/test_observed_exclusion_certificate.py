"""Merged routing boxes must not invent a reason to brake a safe issued leg."""
import json
from pathlib import Path
import pytest
from mission_bt.search_planner import path_hits_exclusion, point_in_exclusion
from test_search_current_command import make_stage, wire_real_command, Mav


def test_captured_v15_safe_command_is_not_cancelled_by_merge_added_ground():
 record=json.loads((Path(__file__).parent/'fixtures/search_merge_margin_v15.json').read_text())
 event=record['event'];stage,mav,_=make_stage();commander=wire_real_command(mav)
 mav._pos=(*event['at'],10.);mav.exclusions=record['exclusions']
 mav.delivery_inputs['speed']=event['speed']
 Mav.goto(commander,*event['goal'],10.)
 assert not point_in_exclusion(*event['at'],1.5,mav.exclusions)
 assert not point_in_exclusion(*event['goal'],1.5,mav.exclusions)
 assert not path_hits_exclusion([event['at'],event['goal']],1.5,mav.exclusions)
 assert not stage._search_braking_gate.tick(stage)
 assert stage._search_braking_gate.hold is None


def test_free_corner_of_an_l_keeps_the_full_original_clearance():
 stage,mav,_=make_stage();mav.exclusions=[(0.,6.,0.,3.),(0.,3.,3.,6.)]
 mav._pos=(5.,5.,10.)
 assert stage.router._segment_safe(mav,(6.5,5.))


@pytest.mark.parametrize('end',[(2.,5.),(5.,4.499),(2.,2.)])
def test_merge_shortcut_cannot_cross_paint_or_reduce_the_margin(end):
 stage,mav,_=make_stage();mav.exclusions=[(0.,6.,0.,3.),(0.,3.,3.,6.)]
 mav._pos=(5.,5.,10.)
 assert not stage.router._segment_safe(mav,end)


def test_stationary_intent_inside_an_actual_margin_is_not_exempted():
 stage,mav,_=make_stage();mav.exclusions=[(0.,6.,0.,3.)]
 mav._pos=(5.,4.499,10.)
 assert not stage.router._segment_safe(mav,mav.pos()[:2])
