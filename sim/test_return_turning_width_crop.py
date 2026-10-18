"""Fresh clipped lettering during an actual turn can guide framing only."""
import math
import pytest
from test_return_capture_batching import turning_top_crop_stage


def stage():
    leaf,mav,now,captures=turning_top_crop_stage()
    # V17's recorded return view: board fills both horizontal edges and the
    # bottom while lettering is briefly identified before yaw sweeps past it.
    for capture in captures:
        capture.update(identified=True,board_px=[0,402,1280,317],bearing=.029)
    captures.append(dict(captures[-1],sequence=5,capture_stamp=1.15,identified=False,board_px=None))
    return leaf,mav,now,captures


def test_four_fresh_turning_captures_trigger_existing_bounded_backoff():
    leaf,mav,now,captures=stage();before=leaf._anchor
    leaf._settle()
    assert leaf._backoffs==1 and leaf.phase is leaf.RELOCATE
    theta=captures[3]['pose']['yaw']-math.atan(.029*math.tan(leaf.hfov/2))
    assert leaf._anchor[0]==pytest.approx(before[0]-leaf.backoff_m*math.cos(theta))
    assert leaf._anchor[1]==pytest.approx(before[1]-leaf.backoff_m*math.sin(theta))
    assert leaf._anchor[2]==before[2]
    assert leaf._pending_sweep_yaw==pytest.approx(theta)
    assert leaf._good_vantage is None and leaf._sq_steps==0


@pytest.mark.parametrize('invalid',['outbound','stale','duplicate','tilted','framed','exhausted','old_epoch','missing_pose','unidentified','outside_frame'])
def test_turning_backoff_never_uses_unqualified_or_out_of_scope_evidence(invalid):
    leaf,mav,now,captures=stage();before=leaf._anchor;backoffs=leaf._backoffs
    if invalid=='outbound':mav.outbound_banner_xy=None
    elif invalid=='stale':now[0]=2.
    elif invalid=='duplicate':
        for capture in captures:capture['sequence']=1
    elif invalid=='tilted':mav.camera_state['actual_rad']=-math.radians(20)
    elif invalid=='framed':
        for capture in captures:capture['board_px']=[300,300,600,300]
    elif invalid=='exhausted':leaf._backoffs=backoffs=leaf.max_backoffs
    elif invalid=='old_epoch':leaf._phase_t0=1.19
    elif invalid=='missing_pose':
        for capture in captures:capture['pose']=None
    elif invalid=='unidentified':
        for capture in captures:capture['identified']=False
    elif invalid=='outside_frame':
        for capture in captures:capture['bearing']=8.
    leaf._settle()
    assert leaf._backoffs==backoffs and leaf._anchor==before
    assert leaf._good_vantage is None


def test_stale_clip_prefix_cannot_be_combined_with_new_arrival():
    leaf,mav,now,captures=stage();before=leaf._anchor
    mav.banner_observations=lambda:captures[:3];leaf._settle()
    assert leaf._turn_clip_samples==3 and leaf._anchor==before
    now[0]=2.3
    mav.banner_observations=lambda:[dict(captures[3],sequence=6,capture_stamp=2.2)]
    leaf._settle()
    assert leaf._turn_clip_samples==1 and leaf._anchor==before and leaf._backoffs==0
