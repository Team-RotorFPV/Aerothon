"""The candidate must hold position, retain identity checks and remain bounded."""
import math,sys
from pathlib import Path
from mission_bt.mission_tree import AlignToBanner as FineReturnFraming
from test_return_gate_search import World

def stage(alt=5.,returning=True):
    mav=World([],start=(20.,-16.,math.pi))
    mav._pos=(20.,-16.,alt);mav._alt=alt
    mav.outbound_banner_xy=(0.,0.) if returning else None
    leaf=FineReturnFraming(mav)
    leaf.initialise()
    return leaf,mav

def test_fine_sweep_stays_at_measured_position():
    leaf,mav=stage()
    before=mav.pos()
    leaf._give_up()
    assert mav.gotos[-1][:3]==before
    assert len(leaf._pending_offsets)==24
    assert mav.camera_poses[-1]=='BANNER'
    assert leaf._fine_return_attempted and leaf._relocations==0
    assert leaf.phase==leaf.RELOCATE
    assert leaf._good_vantage is None, 'camera framing is not identity acceptance'

def test_low_gate_height_keeps_level_camera():
    leaf,mav=stage(3.)
    leaf._give_up()
    assert not mav.camera_poses

def test_fine_retry_is_bounded():
    leaf,mav=stage()
    leaf._give_up()
    leaf._give_up()
    assert leaf._relocations==1

def test_outbound_search_does_not_use_return_retry():
    leaf,mav=stage(returning=False)
    leaf._give_up()
    assert not leaf._fine_return_attempted
    assert leaf._relocations==1
