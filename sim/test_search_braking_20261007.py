"""The real search leaf must brake before it executes a replacement route."""
from types import SimpleNamespace
import pytest
from mission_bt.mission_tree import LawnmowerSearch
from test_leg_routing import RoutedMav


def make_stage():
    mav = RoutedMav(at=(0., 0., 10.))
    now = [1.]
    mav.delivery_inputs = dict(pose_stamp=1., velocity_stamp=1., speed=.6)
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: now[0], crab=True)
    stage.initialise()
    stage.wps = [(30., 0.)]
    stage.i = 0
    stage.update()
    assert mav.gotos[-1][:2] == (30., 0.)
    mav.exclusions = [(10., 20., -5., 5.)]
    return stage, mav, now


def test_new_exclusion_stops_before_replacement_route():
    stage, mav, _ = make_stage()
    stage.update()
    assert mav.gotos[-1][:3] == mav.pos(), 'new exclusion must first cancel the streamed search goal'


def test_braking_target_does_not_chase_the_moving_aircraft():
    stage, mav, _ = make_stage()
    stage.update()
    target = mav.gotos[-1][:3]
    mav._pos = (.2, 0., 10.)
    stage.update()
    assert mav.gotos[-1][:3] == target


def test_duplicate_settled_samples_cannot_resume_search():
    stage, mav, _ = make_stage()
    stage.update()
    mav.delivery_inputs['speed'] = 0.
    for _ in range(10):
        stage.update()
        assert mav.gotos[-1][:3] == (0., 0., 10.)


@pytest.mark.parametrize('bad', ['stale', 'missing', 'future', 'nonfinite'])
def test_unusable_motion_evidence_keeps_search_stopped(bad):
    stage, mav, now = make_stage()
    stage.update()
    mav.delivery_inputs['speed'] = 0.
    if bad == 'stale':
        now[0] = 2.
    elif bad == 'missing':
        del mav.delivery_inputs['velocity_stamp']
    elif bad == 'future':
        mav.delivery_inputs['velocity_stamp'] = 2.
    else:
        mav.delivery_inputs['speed'] = float('nan')
    for _ in range(5):
        stage.update()
        assert mav.gotos[-1][:3] == (0., 0., 10.)


def test_replan_uses_measured_stop_position_after_distinct_samples():
    stage, mav, now = make_stage()
    stage.update()
    replans = []
    original = stage._replan_mid_sweep
    def record():
        replans.append(mav.pos())
        original()
    stage._replan_mid_sweep = record
    mav._pos = (.3, 0., 10.)
    for stamp in (1.1, 1.2, 1.3):
        now[0] = stamp
        mav.delivery_inputs.update(pose_stamp=stamp, velocity_stamp=stamp, speed=.02)
        stage.update()
    assert replans == [(.3, 0., 10.)]
    assert mav.gotos[-1][:3] == (.3, 0., 10.), 'settled handoff freezes the measured stop, not the earlier braking target'
    stage.update()
    assert mav.gotos[-1][:2] != (30., 0.)


def test_certified_nearest_edge_escape_is_not_cancelled_by_search_braking():
    mav = RoutedMav(at=(1., 0., 10.))
    mav.exclusions = [(2., 3., -1., 1.)]
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: 1.)
    stage.initialise()
    # Aircraft is outside paint, inside its inflated clearance margin.
    # The router has already certified the nearest-edge outward escape.
    stage._streamed_search_goal = (.25, 0.)
    assert stage.router._segment_safe(mav, stage._streamed_search_goal)
    assert not stage._search_braking_gate.tick(stage), 'braking may not cancel an already certified margin escape'
    assert stage._search_braking_gate.hold is None


def test_new_paint_invalidating_escape_still_requires_braking():
    mav = RoutedMav(at=(1., 0., 10.))
    mav.exclusions = [(2., 3., -1., 1.), (0., .3, -.2, .2)]
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: 1.)
    stage.initialise()
    stage._streamed_search_goal = (.25, 0.)
    assert not stage.router._segment_safe(mav, stage._streamed_search_goal)
    assert stage._search_braking_gate.tick(stage)
    assert mav.gotos[-1][:3] == mav.pos()


def test_small_cross_track_motion_does_not_cancel_outward_escape():
    mav = RoutedMav(at=(1., 0., 10.))
    mav.exclusions = [(2., 3., -1., 1.)]
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: 1.)
    stage.initialise()
    stage._streamed_search_goal = (.25, 0.)
    assert stage.router._segment_safe(mav, stage._streamed_search_goal)
    mav._pos = (.95, .02, 10.)
    assert not stage._search_braking_gate.tick(stage), 'outward motion must not repeatedly brake for a shifted exact endpoint'


def test_escape_continuation_may_not_reduce_distance_to_paint():
    mav = RoutedMav(at=(1.9, -1.1, 10.))
    mav.exclusions = [(2., 3., -1., 1.)]
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: 1.)
    stage.initialise()
    # The endpoint exits the nearest left edge and never crosses paint,
    # but first reduces the distance to its lower-left corner.
    assert not stage.router._segment_safe(mav, (.25, 1.))


def test_outward_escape_can_approach_a_distant_zone_without_entering_its_margin():
    mav = RoutedMav(at=(1., 0., 10.))
    mav.exclusions = [(2., 3., -1., 1.), (-5., -4., -1., 1.)]
    stage = LawnmowerSearch(mav, (-10., 40., -20., 20.), 10.,
                           exclusions=lambda: mav.exclusions, clock=lambda: 1.)
    stage.initialise()
    stage._streamed_search_goal = (.25, 0.)
    assert stage.router._segment_safe(mav, stage._streamed_search_goal)
    mav._pos = (.95, .02, 10.)
    assert not stage._search_braking_gate.tick(stage), 'a distant rectangle is still clear for the entire escape'


def test_braking_checks_current_stage_observations_not_an_old_map():
    mav = RoutedMav(at=(0., 0., 10.))
    observations = []
    stage = LawnmowerSearch(mav, (-5., 40., -20., 20.), 10.,
                           exclusions=lambda: observations, clock=lambda: 1.)
    stage.initialise()
    stage._streamed_search_goal = (30., 0.)
    observations.append((10., 20., -5., 5.))
    assert stage._search_braking_gate.tick(stage)
    assert mav.gotos[-1][:3] == mav.pos()
