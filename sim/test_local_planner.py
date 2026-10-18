"""Hard-clearance connected planning, independent of ROS and simulator truth."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_avoidance/avoidance'))
from avoidance.local_planner import LocalPlanner


def wall_scan(left=1.75,right=1.75):
    b = np.linspace(-math.pi,math.pi,721)
    r = np.full(len(b),12.0)
    for i,a in enumerate(b):
        if abs(math.sin(a)) > .01:
            r[i] = min(12,(left if math.sin(a)>0 else right)/abs(math.sin(a)))
    return b,r


def test_clearance_cannot_be_reduced():
    with pytest.raises(ValueError): LocalPlanner(.69)


def test_connected_observed_lane_passes():
    p=LocalPlanner(); p.observe(*wall_scan())
    path=p.plan()
    assert path[-1][0] > 3
    assert all(p.segment_clear(a,b) for a,b in zip(path,path[1:]))


def test_narrow_lane_has_no_plan_or_escape():
    p=LocalPlanner(); p.observe(*wall_scan(.6,.6))
    assert p.plan()==[]
    assert not p.segment_clear((0,0),(.2,0))


def test_masked_rear_does_not_authorize_reverse():
    b,r=wall_scan(); keep=np.abs(b)<math.radians(150)
    p=LocalPlanner(); p.observe(b[keep],r[keep])
    assert not p.segment_clear((0,0),(-.8,0))


def test_fresh_observed_clear_rear_can_authorize_reverse():
    p=LocalPlanner(); p.observe(*wall_scan())
    assert p.segment_clear((0,0),(-.8,0))


def test_mounted_lidar_can_leave_already_occupied_aircraft_footprint():
    b,r=wall_scan();keep=np.abs(b)<math.radians(150)
    p=LocalPlanner(occupied_radius=.4)
    p.observe(b[keep],r[keep],sensor_origin=(.139,0))
    path=p.plan()
    assert path and path[-1][0]>3
    assert all(p.segment_clear(a,c) for a,c in zip(path,path[1:]))
    assert not p.segment_clear((0,0),(-.8,0))


@pytest.mark.parametrize('axis_bearing',[.39,.622,-.622,.785])
def test_mounted_lidar_link_connects_after_aircraft_turns(axis_bearing):
    b,r=wall_scan();keep=np.abs(b)<math.radians(150)
    p=LocalPlanner(occupied_radius=.4)
    p.observe(b[keep],r[keep],sensor_origin=(.139,-.002))
    path=p.plan(axis_bearing)
    assert path
    assert path[-1][0]*math.cos(axis_bearing)+path[-1][1]*math.sin(axis_bearing)>3
    assert all(p.segment_clear(a,c) for a,c in zip(path,path[1:]))
    assert not p.segment_clear((0,0),(-.8,0))


def test_occupied_sensor_link_does_not_make_blind_sideways_travel_free():
    p=LocalPlanner(occupied_radius=.4)
    p.observe([0],[3],sensor_origin=(.139,0))
    assert p.segment_clear((0,0),(.2,0))
    assert not p.segment_clear((0,0),(0,.1))


def test_intervening_obstacle_blocks_shortcut():
    b,r=wall_scan()
    r[np.abs(b)<math.atan2(.8,2)] = 2/np.cos(b[np.abs(b)<math.atan2(.8,2)])
    p=LocalPlanner(); p.observe(b,r)
    assert not p.segment_clear((0,0),(4,0))
    path=p.plan()
    assert path
    assert all(p.segment_clear(a,c) for a,c in zip(path,path[1:]))


def test_preserved_offaxis_cannot_connect_inflated_obstacles():
    sys.path.insert(0,str(ROOT/'scripts'))
    from world_spec import lane_bottleneck, rect_corners, polygon_gap
    corridor=json.loads((ROOT/'sim/worlds/my_world_offaxis.json').read_text())['return_corridor']
    # Maximin connectivity is equivalent to connecting the complement of
    # wall/obstacle distance bands. At 0.70 m those expanded bands disconnect
    # the lane. Refinement error is much smaller than the missing 0.25 m.
    for resolution in (.05,.025):
        assert lane_bottleneck(corridor,resolution)+math.sqrt(2)*resolution < .70
    # Exact polygon distances: 0.70 m Minkowski expansions overlap along
    # a chain from the upper wall through obstacles 1,5,2 to the lower wall.
    polys=[rect_corners(o['u'],o['v'],o['w'],o['d'],math.radians(o['yaw_deg']))
           for o in corridor['obstacles']]
    upper=rect_corners(5,1.80,10,.1)
    lower=rect_corners(5,-1.80,10,.1)
    chain=[upper,polys[0],polys[4],polys[1],lower]
    assert all(polygon_gap(a,b)<1.40 for a,b in zip(chain,chain[1:]))
