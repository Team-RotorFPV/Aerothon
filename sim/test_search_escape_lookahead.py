"""Look-ahead must preserve the nearest-edge escape from newly observed red."""
from test_leg_routing import FlyingMav
from mission_bt.leg_router import LegRouter, RUNNING, ARRIVED
from mission_bt.search_planner import point_in_exclusion, path_hits_exclusion


def test_search_lookahead_cannot_skip_the_exclusion_escape():
    mav = FlyingMav(at=(34.3, -.32, 10.), exclusions=[(35., 38., -1., 2.)])
    router = LegRouter(clearance_m=1.5, tol=.8, lead_m=3.)
    assert point_in_exclusion(*mav.pos()[:2], 1.5, mav.exclusions)
    assert router.fly(mav, 21., 8.6, 10.) == RUNNING
    assert mav.gotos[-1][:2] == (33.25, -.32)
    assert not path_hits_exclusion([mav.pos()[:2], mav.gotos[-1][:2]], 0., mav.exclusions)
    mav.arrive()
    status, _ = mav.fly(router, 21., 8.6, 10.)
    assert status == ARRIVED


def test_search_escapes_overlapping_observed_exclusion_margins():
    # Minimized from field seed 2: removing either observed cell removes the
    # failure. The aircraft is outside both painted cells but inside their
    # overlapping 1.50 m margins. Their nearest exits differ.
    mav = FlyingMav(at=(34.29564422831, -.32477002698, 10.01859842895),
                    exclusions=[(35.,38.,-1.,2.), (32.,35.,1.,4.)])
    router = LegRouter(clearance_m=1.5, tol=.8, lead_m=3.)
    assert router.fly(mav, 21., 8.6, 10.) == RUNNING
    assert mav.gotos[-1][:2] == (33.25, mav.pos()[1])
    assert not path_hits_exclusion([mav.pos()[:2], mav.gotos[-1][:2]], 0., mav.exclusions)
    mav.arrive()
    assert router.fly(mav, 21., 8.6, 10.) == RUNNING
    assert not path_hits_exclusion([mav.pos()[:2], mav.gotos[-1][:2]], 0., mav.exclusions)
    mav.arrive()
    assert not point_in_exclusion(*mav.pos()[:2], 1.5, mav.exclusions)
    assert mav.fly(router,21.,8.6,10.)[0] == ARRIVED


def test_lookahead_still_runs_after_leaving_the_inflated_zone():
    mav = FlyingMav(at=(33.25, -.32, 10.), exclusions=[(35., 38., -1., 2.)])
    router = LegRouter(clearance_m=1.5, tol=.8, lead_m=3.)
    assert router.fly(mav, 40., 5., 10.) == RUNNING
    assert not path_hits_exclusion([mav.pos()[:2], mav.gotos[-1][:2]], 1.5, mav.exclusions)
