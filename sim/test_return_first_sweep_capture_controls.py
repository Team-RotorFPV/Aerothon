"""A stable captured green lead starts guarded observation, not acceptance."""
import math
from types import SimpleNamespace
import pytest
import py_trees
from mission_bt.banner_captures import BannerCaptures
from mission_bt.mission_tree import FindReturnBanner
from test_return_gate_search import World


def stage():
    now=[10.]
    mav=World([],start=(22.2,-5.5,-2.05))
    # Commands do not teleport the vehicle or clear an obstruction.
    mav.goto=lambda *args:mav.gotos.append(args)
    mav.set_speed=lambda speed: True
    mav.camera_state=dict(actual_rad=0.,settled=True,stale=False)
    mav.outbound_banner_xy=(6.2,-1.2)
    mav.corridor_exit_pose=(16.2,-.3,3.,0.)
    captures=BannerCaptures()
    mav.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(now[0]*1e9))))
    mav.banner_observation=lambda:captures.current(now[0])
    mav.banner_capture_pose=lambda:(mav.banner_observation() or {}).get('pose')
    leaf=FindReturnBanner(mav,clock=lambda:now[0],hfov_rad=.851919)
    leaf.initialise();leaf.phase='sweep';leaf._turning=False;leaf._since=9.9
    def deliver(sequence,box=None,stamp=None):
        now[0]=10.+sequence*.2
        t=now[0]-.05 if stamp is None else stamp
        captures.pose(t,mav.pos(),mav.yaw())
        captures.receive(dict(capture_stamp=t,processed_stamp=now[0],identified=False,bearing=0.),now[0])
        box=box or [520,395,177,181]
        mav.banner_green=dict(t=t,wh=[1280,720],regions=[dict(px=box,area=box[2]*box[3])])
    return leaf,mav,now,deliver


def test_stable_whole_lead_does_not_interrupt_initial_sweep():
    leaf,mav,now,deliver=stage()
    for sequence in range(1,4):
        deliver(sequence)
        assert leaf.update() is py_trees.common.Status.RUNNING
    assert leaf.phase=='sweep', 'initial sweep must complete before an orbit'
    assert leaf._first_sweep_lead is not None
    leaf.hi=len(leaf.headings)
    leaf.update()
    assert leaf.phase=='transit'
    assert leaf._guard_leg and not leaf._high
    assert leaf._cur_alt==leaf.orbit_alt_m
    assert leaf._leg_speed==leaf.guard_speed_mps
    assert leaf.hi==0 and leaf.vi==-1 and leaf._hits==0
    assert leaf.target and leaf._orbit is not None
    assert not any('identified' in text for text,_ in mav.logs)


@pytest.mark.parametrize('invalid',['duplicate','stale','top_cut','bottom_cut','side_cut','unsettled','outbound','unstable'])
def test_bad_or_inconsistent_lead_keeps_searching_without_gate_acceptance(invalid):
    leaf,mav,now,deliver=stage()
    for sequence in range(1,4):
        box=None
        if invalid=='top_cut':box=[520,0,177,181]
        if invalid=='bottom_cut':box=[520,540,177,180]
        if invalid=='side_cut':box=[0,395,177,181]
        if invalid=='unstable':box=[100 if sequence%2 else 1000,395,177,181]
        if invalid=='outbound':mav.outbound_banner_xy=(18.8,-13.5);mav.corridor_exit_pose=(18.8,-13.5,3.,0.)
        deliver(sequence,box,stamp=9.95 if invalid=='duplicate' else 8. if invalid=='stale' else None)
        if invalid=='unsettled':mav.camera_state['settled']=False
        leaf.update()
    assert leaf.phase=='sweep' and leaf._orbit is None and leaf._hits==0
    assert leaf._first_sweep_lead is None


def test_one_fresh_lead_cannot_be_counted_at_multiple_leaf_ticks():
    leaf,mav,now,deliver=stage();deliver(1)
    for _ in range(6):leaf.update()
    assert leaf.phase=='sweep' and leaf._orbit is None
    assert leaf._first_sweep_lead is None
