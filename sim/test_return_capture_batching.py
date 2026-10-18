"""Alignment consumes captured frames, not aliases of the latest detail."""
from types import SimpleNamespace
import math
import pytest
from mission_bt.banner_captures import BannerCaptures
from mission_bt.mav_commander import Mav
from test_return_inherited_tilt import prepare


def batch_stage():
 leaf,mav,_,now=prepare();mav._yaw=0.
 leaf.dwell_s=.3
 mav.camera_state=dict(requested='FORWARD',actual_rad=0.,settled=True,stale=False)
 captures=BannerCaptures();commander=object.__new__(Mav);commander.banner_captures=captures
 commander.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(now[0]*1.e9))))
 mav.banner_observation=lambda:Mav.banner_observation(commander)
 mav.banner_observations=lambda:Mav.banner_observations(commander)
 mav.banner_identified=lambda:bool(mav.banner_observation() and mav.banner_observation()['identified'])
 mav.banner_bearing=lambda:BannerCaptures.bearing_at(mav.banner_observation(),mav.yaw(),leaf.hfov) or 0.
 mav.banner_clipped=True
 def deliver(sequence,identified):
  stamp=1.+sequence*.02
  assert captures.pose(stamp,mav.pos(),mav.yaw())
  assert captures.receive(dict(capture_stamp=stamp,processed_stamp=stamp+.01,identified=identified,bearing=.2,
   board_px=[80,430,1199,289] if identified else None,board_area_px=346511 if identified else 0.,image_wh=[1280,720]),now[0])
 return leaf,mav,now,deliver


def test_clipped_frames_between_leaf_ticks_trigger_existing_bounded_backoff():
 leaf,mav,now,deliver=batch_stage();before=leaf._anchor
 now[0]=1.2
 for seq in range(1,7):deliver(seq,seq<=4)
 leaf._dwell()
 assert leaf._samples==6 and leaf._hits==4 and leaf._clipped==4, 'fresh captured lettering cannot disappear behind a newer negative frame'
 now[0]=1.4
 for seq in range(7,13):deliver(seq,False)
 leaf._dwell()
 assert leaf._samples==12 and leaf._hits==4 and leaf._clipped==4
 assert leaf._backoffs==1 and leaf.phase==leaf.RELOCATE
 assert leaf._anchor==(before[0]-leaf.backoff_m,before[1],before[2])
 assert leaf._good_vantage is None, 'clipped framing evidence may not accept a range or gate crossing'


def test_replayed_capture_history_does_not_inflate_the_sample_count():
 leaf,mav,now,deliver=batch_stage();now[0]=1.2;leaf.dwell_s=5.
 for seq in range(1,7):deliver(seq,False)
 leaf._dwell();first=leaf._samples
 for _ in range(4):leaf._dwell()
 assert first==6 and leaf._samples==first


def test_capture_history_before_new_dwell_epoch_is_not_counted():
 leaf,mav,now,deliver=batch_stage();now[0]=1.2
 for seq in range(1,7):deliver(seq,True)
 leaf._enter(leaf.DWELL);leaf._dwell()
 assert leaf._samples==0 and leaf._hits==0


def turning_top_crop_stage():
 leaf,mav,now,_=batch_stage();now[0]=1.2
 leaf._target_yaw=math.pi;leaf._align_t0=1.;leaf._enter(leaf.SETTLE);leaf._phase_t0=1.
 mav._yaw=math.radians(150)
 # A command does not teleport the actual aircraft to its requested heading.
 mav.goto=lambda *args:mav.gotos.append(args)
 captures=[]
 for sequence in range(1,5):
  captures.append(dict(sequence=sequence,capture_stamp=1.+sequence*.03,
   identified=sequence<=3,bearing=.2,pose=dict(yaw=mav.yaw(),xyz=list(mav.pos())),
   board_px=[151,0,1128,117] if sequence<=3 else None,image_wh=[1280,720]))
 mav.banner_observations=lambda:captures
 mav.banner_observation=lambda:captures[-1]
 return leaf,mav,now,captures


def test_top_crop_captured_during_yaw_settlement_recovers_before_sweeping_past():
 leaf,mav,now,captures=turning_top_crop_stage();before=leaf._anchor
 leaf._settle()
 assert leaf.phase is leaf.RELOCATE
 assert leaf._anchor==(before[0],before[1],before[2]+.5)
 expected=captures[2]['pose']['yaw']-math.atan(.2*math.tan(leaf.hfov/2))
 assert leaf._pending_sweep_yaw==pytest.approx(expected)
 assert leaf._good_vantage is None and leaf._sq_steps==0


@pytest.mark.parametrize('invalid',['outbound','stale','duplicate','tilted'])
def test_turning_crop_recovery_remains_fail_closed(invalid):
 leaf,mav,now,captures=turning_top_crop_stage();before=leaf._anchor
 if invalid=='outbound':mav.outbound_banner_xy=None
 if invalid=='stale':now[0]=2.
 if invalid=='duplicate':
  for capture in captures:capture['sequence']=1
 if invalid=='tilted':mav.camera_state['actual_rad']=-math.radians(20)
 leaf._settle()
 assert leaf._anchor==before and leaf._good_vantage is None


def test_old_crop_samples_cannot_combine_with_a_new_capture_to_raise():
 leaf,mav,now,captures=turning_top_crop_stage();before=leaf._anchor
 mav.banner_observations=lambda:captures[:2]
 leaf._settle()
 assert leaf._framing_samples==2 and leaf._anchor==before
 now[0]=2.3
 later=dict(captures[2],sequence=5,capture_stamp=2.2)
 mav.banner_observations=lambda:[later]
 leaf._settle()
 assert leaf._framing_samples==1 and leaf._anchor==before
