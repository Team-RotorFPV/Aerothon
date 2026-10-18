"""A cleared deployment transient must not consume a release attempt."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_mission/mission_bt'))
from mission_bt.delivery_observer import DeliveryObserver,calibration


def test_loaded_hover_clears_unconfirmed_touchdown_candidate():
    o=DeliveryObserver(calibration([.424,.4241]*20,[.407,.4071]*20),1.0472)
    o.state='LOWER';o.started=o.entered=0
    det=dict(visible=True,stamp=1.,w_px=100,h_px=100,img_w=1280,x=0,y=0)
    def telemetry(t,effort):
        return dict(stamp=t,effort=effort,alt=5.5,vz=0,speed=0,
                    tilt_deg=0,accel_residual=0)
    assert o.update(1.,det,telemetry(1.,.407),.54)=='stop'
    assert o.state=='TOUCHDOWN_WAIT'
    for i in range(1,16):
        t=1.+i*.2;det['stamp']=t
        command=o.update(t,det,telemetry(t,.424),.54)
        if o.state=='LOWER':break
    assert o.state=='LOWER'
    assert o.attempt==1
    assert not o.touchdown_inferred and not o.detachment_inferred and not o.confirmed
    assert command in ('stop','lower')


def test_repeated_loaded_sample_cannot_clear_touchdown_candidate():
    o=DeliveryObserver(calibration([.424,.4241]*20,[.407,.4071]*20),1.0472)
    o.state='TOUCHDOWN_WAIT';o.started=o.entered=0
    telemetry=dict(stamp=1.,effort=.424,alt=5.5,vz=0,speed=0,
                   tilt_deg=0,accel_residual=0)
    for now in (1.,1.1,1.2,1.3,1.4):
        assert o.update(now,None,telemetry,.54)=='stop'
    assert o.state=='TOUCHDOWN_WAIT' and o.attempt==1
    assert not o.confirmed
