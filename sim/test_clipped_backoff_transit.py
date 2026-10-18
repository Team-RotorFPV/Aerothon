"""Clipped identity cannot cancel the framing backoff it just requested."""
import sys
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'sim'))
from test_return_turning_width_crop import stage


def test_actual_width_recovery_transit_continues_until_a_complete_view():
    leaf,mav,now,captures=stage();leaf._settle()
    assert leaf._backoffs==1 and leaf.phase is leaf.RELOCATE
    target=leaf._anchor;position=mav.pos()
    captures[-1].update(identified=True,board_px=[0,402,1280,317])
    leaf._transit()
    assert leaf.phase is leaf.RELOCATE, 'clipped identity cancelled the required backoff before movement'
    assert leaf._anchor==target and mav.pos()==position
    assert mav.gotos[-1][:3]==target
    assert leaf._good_vantage is None


def test_a_complete_near_board_still_stops_guarded_orbit_for_confirmation():
    leaf,mav,now,captures=stage();leaf._settle()
    mav.banner_clipped=False
    captures[-1].update(identified=True,board_px=[200,200,600,300])
    mav.banner_board_area=leaf.focal_px**2*leaf.banner_area_m2/5.**2
    position=mav.pos();leaf._transit()
    assert leaf.phase is leaf.SETTLE and leaf._anchor[:2]==position[:2]
    assert leaf._good_vantage is None


def test_a_distant_complete_board_does_not_cancel_guarded_transit():
    leaf,mav,now,captures=stage();leaf._settle();target=leaf._anchor
    mav.banner_clipped=False
    captures[-1].update(identified=True,board_px=[200,200,200,80])
    mav.banner_board_area=leaf.focal_px**2*leaf.banner_area_m2/20.**2
    leaf._transit()
    assert leaf.phase is leaf.RELOCATE and leaf._anchor==target
