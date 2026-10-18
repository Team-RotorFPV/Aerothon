"""A motion certificate must retain every segment while batching lidar queries."""
import numpy as np
from test_corridor_clearance_contract import controller

def test_motion_certificate_batches_distance_queries_without_changing_command(controller):
 c,sent=controller;c._ground_world=np.array([.8,0.]);c._compensated=lambda vx,vy:(vx,vy)
 c._planner.observe(np.linspace(-np.pi,np.pi,721),np.full(721,12.))
 count=[0];original=c._planner.distances
 def counted(xy):count[0]+=1;return original(xy)
 c._planner.distances=counted
 c._publish(.8,0.,12.,0.,{})
 assert sent[-1].velocity.x==.8
 assert count[0]<=2,f'{count[0]} separate lidar distance queries in one motion certificate'


def test_batched_certificate_matches_individual_segments_with_thin_hits_and_masked_rays():
 from avoidance.local_planner import LocalPlanner
 from test_local_planner import wall_scan
 rng=np.random.default_rng(807)
 for masked in (False,True):
  planner=LocalPlanner();bearings,ranges=wall_scan()
  ranges[np.argmin(abs(bearings-.7))]=1.2
  keep=abs(bearings)<2.5 if masked else np.ones(len(bearings),dtype=bool)
  planner.observe(bearings[keep],ranges[keep])
  for _ in range(80):
   points=np.vstack((np.zeros(2),np.cumsum(rng.uniform(-.25,.25,(12,2)),axis=0)))
   assert planner.polyline_clear(points)==all(planner.segment_clear(a,b) for a,b in zip(points,points[1:]))
  assert not planner.polyline_clear([[0.,0.],[.8,.8]])
