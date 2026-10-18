"""Exercise an empty live-stage sweep without repeated cross-field turns."""
import math

import py_trees

from test_return_gate_search import Clock, HFOV, World
from mission_bt.mission_tree import FindReturnBanner


def shortest_travel(headings):
    return sum(abs(math.atan2(math.sin(b-a), math.cos(b-a)))
               for a, b in zip(headings, headings[1:]))


def test_empty_full_circle_visits_each_view_without_repeated_crossing():
    mav = World([], start=(14.5, -2.0, math.pi))
    leaf = FindReturnBanner(mav, alt=5.0, clock=Clock(), hfov_rad=HFOV)
    leaf.initialise()
    for _ in range(2000):
        assert leaf.update() == py_trees.common.Status.RUNNING
        if 'BANNER' in mav.camera_poses:
            break
    else:
        raise AssertionError('first complete sweep did not finish')
    headings = [g[3] for g in mav.gotos]
    distinct = set(round(h, 5) for h in headings)
    assert len(distinct) == 12, 'retain all twelve observed headings'
    assert shortest_travel(headings) <= 2*math.pi, (
        'empty full-circle search repeats already observed yaw directions')


def test_initial_view_is_preserved_at_arbitrary_yaw():
    for yaw in (-3.05, 0.0, 2.8):
        mav = World([], start=(14.5, -2.0, yaw))
        leaf = FindReturnBanner(mav, clock=Clock())
        leaf.initialise()
        leaf.update()
        assert abs(math.atan2(math.sin(mav.gotos[-1][3]-yaw),
                              math.cos(mav.gotos[-1][3]-yaw))) < 1e-9
