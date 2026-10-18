"""The first far-return sweep must try the level view before a full down-tilt turn."""
import math
import py_trees
from mission_bt.mission_tree import FindReturnBanner
from test_return_gate_search import World,Clock

class FarReturnFrame(World):
 def _look(self):
  super()._look()
  if not self.camera_poses or self.camera_poses[-1]!='FORWARD':
   self.banner_z=0.;self.banner_board_area=0.

def test_far_return_at_recorded_bearing_is_read_without_two_full_turns():
 mav=FarReturnFrame([(16.3,-15.4,0.)],start=(23.,-6.,math.pi));clock=Clock()
 leaf=FindReturnBanner(mav,clock=clock);leaf.initialise()
 for _ in range(1200):
  result=leaf.update()
  if leaf.phase=='approach' or result is py_trees.common.Status.SUCCESS:break
 assert leaf.phase=='approach' or result is py_trees.common.Status.SUCCESS
 assert len({round(g[3],4) for g in mav.gotos}) <= 4, 'readable return view was deferred until a complete yaw circle'
