"""Captured native origin recovery and continuous hard-clearance certificates."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from avoidance.local_planner import LocalPlanner
from avoidance.fast_connected_planner_candidate import FastConnectedPlanner
from test_local_planner import wall_scan
from test_corridor_clearance_contract import controller
from test_headless_land_position_hold import world,DT
from geometry_msgs.msg import PoseStamped

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('planner_type',[LocalPlanner,FastConnectedPlanner])
def test_actual_native_safe_origin_has_a_continuously_clear_route(planner_type):
    fixture=json.loads((ROOT/'sim/fixtures/corridor_native_safe_origin_split_s20261003_20261009.json').read_text())
    planner=planner_type(occupied_radius=.4)
    planner.observe(fixture['bearings'],fixture['ranges'],sensor_origin=fixture['sensor_origin'],hits=fixture['hits'])
    assert abs(planner.distances([(0,0)])[0]-fixture['recorded_clearance_m'])<.005
    path=planner.plan(0)
    assert path,'Captured safe-origin native scan must not fail before searching for a route'
    dense=np.concatenate([np.linspace(a,b,2001) for a,b in zip(path,path[1:])])
    assert planner.distances(dense).min()>=.70
    assert planner.known(dense).all()

@pytest.mark.parametrize('gap',[.7001,.710,.7249,.725])
def test_an_observed_stationary_origin_does_not_have_a_sampling_gap(gap):
    planner=LocalPlanner();planner.observe(*wall_scan(gap,1.75))
    assert planner.segment_clear((0,0),(0,0))
    assert planner.polyline_clear([(0,0),(0,0)])
    assert planner.clearance==.70

def test_a_subthreshold_origin_is_still_rejected():
    planner=LocalPlanner();planner.observe(*wall_scan(.6999,1.75))
    assert not planner.segment_clear((0,0),(0,0))
    assert not planner.polyline_clear([(0,0),(0,0)])
    assert not planner.plan()

def test_a_thin_surface_between_clear_endpoints_is_still_rejected():
    bearings=np.linspace(-math.pi,math.pi,721)
    point=np.array([.025,.6999])
    bearings=np.r_[bearings,math.atan2(point[1],point[0])]
    ranges=np.r_[np.full(721,12.),np.linalg.norm(point)]
    planner=LocalPlanner();planner.observe(bearings,ranges)
    assert (planner.distances([(0,0),(.05,0)])>=.70).all()
    assert planner.distances([(.025,0)])[0]<.70
    assert not planner.segment_clear((0,0),(.05,0))
    assert not planner.polyline_clear([(0,0),(.05,0)])

def test_refinement_cannot_open_a_masked_sector():
    bearings,ranges=wall_scan(.72,1.75)
    kept=np.abs(bearings)<math.radians(150)
    planner=LocalPlanner();planner.observe(bearings[kept],ranges[kept])
    assert not planner.segment_clear((0,0),(-.8,0))
    assert not planner.polyline_clear([(0,0),(-.8,0)])

def test_certified_mild_drift_hold_preserves_actual_airframe_clearance(controller):
    node,_=controller
    aircraft=world();aircraft.p=np.array([0.,0.,2.]);aircraft.gps_err[:]=0.
    aircraft.wind=lambda t:(0.,.025)
    aircraft.sp_pos=None;aircraft.sp_vel=(0.,0.,0.,0.,aircraft.t)
    aircraft.ground_velocity=np.array([0.,.025,0.])
    node._now=lambda:aircraft.t
    node.pub_sp.publish=aircraft._on_sp_raw
    # Exercise the real publication guard with a rejected toward-wall estimate.
    # Its observed cancellation must remain available from a legal .715 m origin.
    node._compensated=lambda vx,vy:(0.,.2)
    minimum=.715;next_command=aircraft.t
    for _ in range(round(4/DT)):
        if aircraft.t>=next_command:
            pose=PoseStamped();pose.header.stamp=aircraft.stamp();pose.pose.orientation.w=1.
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,aircraft.p)
            node._on_pose(pose)
            node.scan.header.stamp=aircraft.stamp();node._scan_t=aircraft.t
            node._ground_world=aircraft.ground_velocity[:2].copy()
            node._wind_world=np.array([0.,.025]);node._wind_rate_world=np.zeros(2)
            node._planner.observe(*wall_scan(.715-aircraft.p[1],1.75+aircraft.p[1]))
            node._publish(0.,0.,2.,0.,{})
            next_command=aircraft.t+.05-1e-9
        aircraft.t+=DT;aircraft._step_airframe()
        minimum=min(minimum,float(.715-aircraft.p[1]))
    print(dict(minimum_clearance_m=minimum,normal_response_s=aircraft.args.vel_response_s,
               added_transport_delay_s=aircraft.args.vel_command_delay_s))
    assert aircraft.args.vel_response_s==.6 and aircraft.args.vel_command_delay_s==0.
    assert minimum>=.70



@pytest.mark.parametrize('planner_type',[LocalPlanner,FastConnectedPlanner])
def test_a_captured_tight_origin_route_gives_away_no_clearance(planner_type):
    """The V40 rendered split return grazed the board at 0.694474 m.

    Its own observation was still safe at 0.7015 m, inside the 0.025 m
    planning reserve. From there every replan accepted the longest shortcut
    that merely preserved the deficit, so the route rode the 0.70 m floor
    toward the turned board's corner and the airframe's own response had
    nothing left. A shortcut may not spend what such an origin has left.
    """
    fixture=json.loads((ROOT/'sim/fixtures/corridor_native_slalom_shortcut_split_s20261003_20261010.json').read_text())
    planner=planner_type(occupied_radius=.4)
    planner.observe(fixture['bearings'],fixture['ranges'],
                    sensor_origin=fixture['sensor_origin'],hits=fixture['hits'])
    origin=float(planner.distances([(0,0)])[0])
    assert abs(origin-fixture['recorded_clearance_m'])<.005
    assert planner.clearance<=origin<planner.clearance+.025,'Fixture must isolate a tight safe origin'
    path=planner.plan(0)
    assert path,'A tight but observed-safe origin must still find a route'
    dense=np.concatenate([np.linspace(a,b,2001) for a,b in zip(path,path[1:])])
    assert planner.known(dense).all()
    assert planner.distances(dense).min()>=origin-1e-9
