"""The object fixture must retain the native camera and lidar cadences."""
import threading
from types import SimpleNamespace
import pytest
import headless_world as H
from builtin_interfaces.msg import Time


def run_one_second(monkeypatch,drop=0.,latency=0.):
 world=object.__new__(H.HeadlessWorld)
 counts=dict(camera=0,scan=0,fcu=0,slow=0,record=0,delivered=[])
 world.lock=threading.Lock();world.cam_joint=world.cam_cmd=0.;world.t=0.;world.done=False
 world.args=SimpleNamespace(rtf=1.e9);world.frame_drop=drop;world.latency=latency;world.pending_frames=[]
 world.rng=SimpleNamespace(random=lambda:.5)
 world.pub_clock=SimpleNamespace(publish=lambda msg:None);world.stamp=lambda:Time()
 for name in ('_step_errors','_step_airframe','_step_payload'):setattr(world,name,lambda:None)
 world.est=lambda:None
 def count(name):counts[name]+=1
 world._publish_fcu=lambda est:count('fcu');world._publish_scan=lambda:count('scan')
 world._publish_slow=lambda:count('slow');world._record=lambda:count('record')
 def capture():
  count('camera');return dict(capture_stamp=world.t)
 world._camera_frame=capture
 world._deliver=lambda frame:counts['delivered'].append((frame['capture_stamp'],world.t))
 def stop():world.done=world.t>=1.-1.e-8
 world._start_and_stop=stop
 monkeypatch.setattr(H.rclpy,'ok',lambda:True)
 H.HeadlessWorld.run(world)
 return counts


def test_real_fixture_loop_keeps_native_20hz_camera_and_10hz_scan(monkeypatch):
 counts=run_one_second(monkeypatch)
 assert counts['camera']==20, 'normal native camera is 20 Hz, independent of 10 Hz lidar'
 assert counts['scan']==10 and counts['record']==10
 assert counts['fcu']==33 and counts['slow']==5
 # Zero configured latency still costs the minimum transport, so the frame
 # captured at 1.00 s lands after this one-second window.
 assert len(counts['delivered'])==19
 assert len({stamp for stamp,_ in counts['delivered']})==19


def test_zero_latency_never_delivers_in_the_capture_step(monkeypatch):
 # Delivered in the capture step, a detection was stamped ahead of a
 # receiver whose /clock had not caught up, and refused as from the future.
 counts=run_one_second(monkeypatch)
 assert all(at-stamp>=H.MIN_TRANSPORT_S-1e-9 for stamp,at in counts['delivered'])


@pytest.mark.parametrize('drop,latency',[(1.,0.),(0.,.14)])
def test_frame_drops_and_transport_latency_are_retained(monkeypatch,drop,latency):
 counts=run_one_second(monkeypatch,drop,latency)
 if drop:
  assert counts['camera']==0 and counts['delivered']==[]
 else:
  assert counts['camera']==20
  assert all(received-captured>=latency-1.e-8 for captured,received in counts['delivered'])

def test_fixture_rates_match_the_materialized_native_sensor_definitions():
 import sys
 import xml.etree.ElementTree as ET
 from pathlib import Path
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
 from materialize_vehicle_model import COMPETITION_HARDWARE
 from sensor_cadence import CAMERA_CAPTURE_HZ,LIDAR_SCAN_HZ
 native=ET.fromstring(COMPETITION_HARDWARE)
 assert float(native.find('.//sensor[@type="camera"]/update_rate').text)==CAMERA_CAPTURE_HZ
 assert float(native.find('.//sensor[@type="gpu_lidar"]/update_rate').text)==LIDAR_SCAN_HZ
