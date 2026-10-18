"""Real descent must retain its visual correction across a brief capture gap."""
from types import SimpleNamespace as NS
import py_trees
from mission_bt.mission_tree import PrecisionDescent

class Mav:
 def __init__(self):
  self.at=(.6328112676908547,-.38812870976581115,8.3722311432605)
  self.home_marker_xy=(.36968553276409133,-.12494144993480921)
  self.qr_offset=NS(x=.04,y=-.04,z=1.);self.gotos=[]
 def pos(self):return self.at
 def yaw(self):return 0.
 def home_local_xy(self):return (0.,0.)
 def qr_visible(self):return self.qr_offset.z>0.
 def goto(self,*args):self.gotos.append(args)
 def log(self,*args,**kwargs):pass

def test_capture_gap_does_not_erase_recent_visual_landing_correction():
 mav=Mav();leaf=PrecisionDescent(mav,marker_m=3.1,hfov_rad=.851919)
 leaf.initialise();assert leaf.update() is py_trees.common.Status.RUNNING
 visual=mav.gotos[-1]
 mav.at=(.8467049288690549,-.02237823648439341,8.204483492775719)
 mav.qr_offset=NS(x=0.,y=0.,z=0.)
 assert leaf.update() is py_trees.common.Status.RUNNING
 assert mav.gotos[-1][2]<=mav.at[2],'one lost frame must not climb back to the start height'
 assert mav.gotos[-1][:2]==visual[:2],'retain the recent visual target during the existing grace period'

def test_prolonged_gap_still_reacquires_at_original_start_height():
 mav=Mav();leaf=PrecisionDescent(mav,marker_m=3.1,hfov_rad=.851919,lost_grace=3)
 leaf.initialise();leaf.update();mav.at=(.85,0.,8.2)
 mav.qr_offset=NS(x=0.,y=0.,z=0.)
 for _ in range(4):leaf.update()
 assert mav.gotos[-1][:2]==mav.home_marker_xy
 assert mav.gotos[-1][2]>=leaf.start_alt
 assert leaf.reacquisitions==1

def test_no_prior_visual_lock_still_uses_initial_marker_fix():
 mav=Mav();mav.qr_offset=NS(x=0.,y=0.,z=0.);mav.at=(1.,0.,8.2)
 leaf=PrecisionDescent(mav,marker_m=3.1,hfov_rad=.851919)
 leaf.initialise();leaf.update()
 assert mav.gotos[-1][:2]==mav.home_marker_xy
 assert mav.gotos[-1][2]>=leaf.start_alt

def test_reinitialising_clears_previous_visual_correction():
 mav=Mav();leaf=PrecisionDescent(mav,marker_m=3.1,hfov_rad=.851919)
 leaf.initialise();leaf.update();assert leaf._visual_target_xy is not None
 leaf.initialise();mav.qr_offset=NS(x=0.,y=0.,z=0.);mav.at=(1.,0.,8.2)
 leaf.update();assert mav.gotos[-1][:2]==mav.home_marker_xy

def test_reacquired_frame_resets_gap_without_blind_descent():
 mav=Mav();leaf=PrecisionDescent(mav,marker_m=3.1,hfov_rad=.851919,lost_grace=3)
 leaf.initialise();leaf.update();mav.at=(.85,0.,8.2)
 for _ in range(3):
  mav.qr_offset=NS(x=0.,y=0.,z=0.);leaf.update()
  assert mav.gotos[-1][2]==mav.at[2]
 mav.qr_offset=NS(x=.04,y=-.04,z=1.);leaf.update();assert leaf._lost==0
 visual=mav.gotos[-1]
 mav.qr_offset=NS(x=0.,y=0.,z=0.);leaf.update()
 assert mav.gotos[-1][:2]==visual[:2]
 assert mav.gotos[-1][2]==mav.at[2]
 assert leaf.reacquisitions==0
