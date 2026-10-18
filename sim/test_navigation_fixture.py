"""The object fixture must never qualify delivery or alter production imports."""
from types import SimpleNamespace
import py_trees
from mission_bt import mission_tree
from navigation_mission import UnqualifiedHeadlessDelivery


def mav_fixture(payout=0.,fault=''):
    commands=[]
    goals=[]
    mav=SimpleNamespace(pos=lambda:(1.,2.,5.),yaw=lambda:.3,
        winch_status=dict(payout_estimate_m=payout,fault=fault),
        log=lambda *a,**kw:None,winch=commands.append,
        goto=lambda *args:goals.append(args),reached=lambda *args:True)
    return mav,commands,goals


def test_importing_navigation_fixture_keeps_production_delivery():
    assert mission_tree.WinchDrop.__name__=='WinchDrop'


def test_navigation_omission_stows_first_and_never_confirms_delivery():
    mav,commands,goals=mav_fixture(1.)
    leaf=UnqualifiedHeadlessDelivery(mav,cruise_alt=10.)
    leaf.initialise()
    assert leaf.update()==py_trees.common.Status.RUNNING
    assert commands==['stow'] and goals==[]
    mav.winch_status['payout_estimate_m']=0.
    assert leaf.update()==py_trees.common.Status.SUCCESS
    assert commands[-1]=='stop' and goals==[(1.,2.,10.,.3)]
    assert mav.delivery_confirmed is False
    assert 'not exercised' in mav.delivery_note


def test_navigation_fixture_winch_fault_is_a_failure():
    mav,commands,goals=mav_fixture(fault='drive fault')
    leaf=UnqualifiedHeadlessDelivery(mav)
    leaf.initialise()
    assert leaf.update()==py_trees.common.Status.FAILURE
    assert commands==['stop'] and goals==[]
    assert mav.delivery_confirmed is False
