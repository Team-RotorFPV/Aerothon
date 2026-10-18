"""Return framing must precede trusting a lidar plane below the visible board."""
import pytest
from test_return_fine_framing import stage

def prepared():
 leaf,mav=stage(3.1);now=[10.]
 leaf.clock=lambda:now[0];leaf._anchor=mav.pos();leaf._entry_alt=3.1
 leaf._align_target=0.;leaf._align_t0=0.;leaf._recentre_needed=lambda:False
 leaf._measure_surface=lambda:dict(ok=False,points=7,reason='no board face')
 mav.camera_state=dict(settled=True,stale=False,actual_rad=0.)
 observation=dict(sequence=1,capture_stamp=10.,identified=True,board_px=[250,0,790,200],image_wh=[1280,720])
 mav.banner_observation=lambda:observation
 leaf._no_surface=lambda fit:None
 return leaf,mav,observation,now

def test_top_cropped_identified_return_board_raises_at_same_xy_before_surface_fit():
 leaf,mav,o,now=prepared()
 for seq in (1,2,3):
  o.update(sequence=seq,capture_stamp=now[0]);leaf._square();now[0]+=.1
 assert leaf._sq_phase is leaf.MOVING
 assert leaf._anchor[:2]==mav.pos()[:2]
 assert leaf._anchor[2]==pytest.approx(3.6)
 assert leaf._corrections==0

@pytest.mark.parametrize('invalid',['duplicate','stale','unidentified','tilted','outbound','lower_half'])
def test_no_climb_without_distinct_fresh_level_return_framing(invalid):
 leaf,mav,o,now=prepared()
 if invalid=='unidentified':o['identified']=False
 if invalid=='tilted':mav.camera_state['actual_rad']=-.349
 if invalid=='outbound':mav.outbound_banner_xy=None
 if invalid=='lower_half':o['board_px']=[250,200,790,350]
 for seq in (1,2,3):
  o.update(sequence=1 if invalid=='duplicate' else seq,capture_stamp=1. if invalid=='stale' else now[0]);leaf._square();now[0]+=.1
 assert leaf._anchor[2]==3.1
