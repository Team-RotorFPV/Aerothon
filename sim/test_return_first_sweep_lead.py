"""Complete the first search sweep before moving toward an observed lead."""
import math
from types import SimpleNamespace
import py_trees
from mission_bt.banner_captures import BannerCaptures
from mission_bt.mission_tree import FindReturnBanner
from test_banner_orbit import BoardMav,HFOV


def test_full_first_sweep_uses_observed_lead_before_second_complete_sweep():
    now=[10.]
    mav=BoardMav((22.2,-5.5,5.),(16.2,-15.4),0.,read_range=0.)
    mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    captures=BannerCaptures()
    mav.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(now[0]*1e9))))
    mav.banner_observation=lambda:captures.current(now[0])
    mav.banner_capture_pose=lambda:(mav.banner_observation() or {}).get('pose')
    original=mav._refresh
    def refresh():
        view=original()
        if mav.banner_green:mav.banner_green['t']=now[0]
        return view
    mav._refresh=refresh
    leaf=FindReturnBanner(mav,clock=lambda:now[0],hfov_rad=HFOV)
    leaf.initialise();visited=set()
    for _ in range(400):
        now[0]+=.2
        captures.pose(now[0],mav.pos(),mav.yaw())
        captures.receive(dict(capture_stamp=now[0],processed_stamp=now[0],identified=False,bearing=0.),now[0])
        if leaf.phase=='sweep' and leaf.hi<len(leaf.headings):visited.add(leaf.hi)
        assert leaf.update() is py_trees.common.Status.RUNNING
        if leaf.phase=='transit':break
        if leaf._second_look_done:break
    assert visited==set(range(12)), 'initial search retains all twelve headings'
    assert leaf.phase=='transit' and leaf._guard_leg, 'whole observed lead waited for second full-circle sweep'
    assert not leaf._second_look_done
    assert not any('identified' in text for text,_ in mav.logs)
