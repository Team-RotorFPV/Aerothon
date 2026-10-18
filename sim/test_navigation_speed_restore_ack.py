"""No high return leg begins before fresh observations and speed acceptance."""
import math
from types import SimpleNamespace
import pytest
import py_trees
from mission_bt.mission_tree import RestoreNavigationSpeed


class Mav:
    def __init__(self):
        self.t=1.;self.xyz=(40.,0.,10.);self.camera_ok=True;self.speed_cmd=.65
        self.delivery_inputs={'pose_stamp':self.t};self.commands=[];self.requests=[]
        self.response='accepted';self.abort_reason=''
    def pos(self):return self.xyz
    def yaw(self):return 0.
    def goto(self,*args):self.commands.append(args)
    def camera_settled(self,pose):return pose=='NADIR' and self.camera_ok
    def set_speed(self,speed):
        self.requests.append(speed)
        if self.response=='unavailable':return None
        if self.response=='service_exception':raise RuntimeError('offline')
        return SimpleNamespace(done=lambda:self.response!='pending',result=self.result)
    def result(self):
        if self.response=='exception':raise RuntimeError('response lost')
        return SimpleNamespace(success=self.response=='accepted')
    def observe(self,t):self.t=t;self.delivery_inputs['pose_stamp']=t


def actor(previous=5.):
    mav=Mav();leaf=RestoreNavigationSpeed(mav,lambda:previous,clock=lambda:mav.t)
    leaf.initialise();return mav,leaf


@pytest.mark.parametrize('previous,expected',[(5.,5.),(10.,5.),(1.5,1.5)])
def test_restored_cap_never_exceeds_pre_search_cap_or_configured_navigation(previous,expected):
    mav,leaf=actor(previous)
    assert leaf.update() is py_trees.common.Status.SUCCESS
    assert mav.requests==[expected] and mav.commands[-1]==(*mav.xyz,0.)


@pytest.mark.parametrize('response',['rejected','exception','unavailable','service_exception'])
def test_service_failure_never_admits_return(response):
    mav,leaf=actor();mav.response=response
    assert leaf.update() is py_trees.common.Status.FAILURE
    assert mav.commands[-1]==(*mav.xyz,0.) and mav.abort_reason


def test_pending_speed_acknowledgement_holds_a_fixed_measured_position():
    mav,leaf=actor();mav.response='pending'
    assert leaf.update() is py_trees.common.Status.RUNNING
    mav.xyz=(40.1,0.,10.);mav.observe(2.)
    assert leaf.update() is py_trees.common.Status.RUNNING
    assert mav.commands[-1]==(40.,0.,10.,0.) and mav.requests==[5.]
    mav.response='accepted';assert leaf.update() is py_trees.common.Status.SUCCESS


def test_acknowledgement_timeout_fails_safely():
    mav,leaf=actor();mav.response='pending';leaf.update();mav.observe(4.01)
    assert leaf.update() is py_trees.common.Status.FAILURE
    assert mav.requests==[5.] and mav.commands[-1]==(*mav.xyz,0.)


@pytest.mark.parametrize('invalid',['stale_pose','camera','low_altitude','missing_previous','nan_previous'])
def test_missing_return_prerequisite_never_requests_a_speed_increase(invalid):
    mav,leaf=actor(None if invalid=='missing_previous' else math.nan if invalid=='nan_previous' else 5.)
    if invalid=='stale_pose':mav.t=1.31
    if invalid=='camera':mav.camera_ok=False
    if invalid=='low_altitude':mav.xyz=(40.,0.,9.)
    assert leaf.update() is not py_trees.common.Status.SUCCESS
    assert not mav.requests and mav.commands[-1]==(*mav.xyz,0.)
