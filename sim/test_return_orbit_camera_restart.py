"""A missed orbit view must not leave every later vantage at its last tilt."""
import math
import py_trees
from mission_bt.mission_tree import FindReturnBanner
from test_return_gate_search import World


def test_next_orbit_vantage_restarts_level_camera_and_both_local_looks():
    now=[1.]
    mav=World([],start=(24.,-8.,math.pi))
    mav.set_speed=lambda speed: True
    leaf=FindReturnBanner(mav,clock=lambda:now[0])
    leaf.initialise()
    leaf._green=dict(x=20.,y=-15.,range=8.,area=30000.,heading=-2.,
                     source='height',cut=False,edge=False)
    assert leaf._next_orbit_vantage()
    leaf.phase='sweep';leaf.hi=len(leaf.headings)
    # A first level sweep misses; the actual state machine tries BANNER.
    assert leaf.update() is py_trees.common.Status.RUNNING
    assert leaf._look=='BANNER' and leaf._second_look_done
    leaf.update();leaf.phase='sweep';leaf.hi=len(leaf.headings)
    assert leaf.update() is py_trees.common.Status.RUNNING
    assert leaf.phase=='transit' and leaf._guard_leg
    assert leaf._look=='FORWARD', 'later orbit vantages inherited the failed down-tilted look'
    assert mav.camera_poses[-1]=='FORWARD'
    assert leaf._second_look=='BANNER' and not leaf._second_look_done
    assert leaf._since is None and leaf._hits==0 and leaf._turning
    assert leaf._cur_alt==leaf.orbit_alt_m
    assert leaf._leg_speed==leaf.guard_speed_mps
    assert leaf.vi==-1
