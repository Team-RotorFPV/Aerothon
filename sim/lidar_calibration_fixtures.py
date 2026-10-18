"""Timestamped full scans for calibration, including panel shape and clutter."""
import math
from sensor_msgs.msg import LaserScan


def panel_scan(body=None,yaw=0.,mirrored=False,distance=1.,stamp=100.):
    n=720;s=LaserScan();s.range_min=.05;s.range_max=12.
    s.angle_min=-math.pi;s.angle_increment=2*math.pi/n
    s.angle_max=s.angle_min+(n-1)*s.angle_increment
    s.header.frame_id='base_scan'
    s.header.stamp.sec=int(stamp);s.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    s.ranges=[5.]*n
    if body is not None:
        a=math.radians(yaw-body if mirrored else body-yaw)
        for i in range(n):
            d=math.atan2(math.sin(s.angle_min+i*s.angle_increment-a),math.cos(s.angle_min+i*s.angle_increment-a))
            if abs(d)<=math.radians(7):s.ranges[i]=distance/math.cos(d)
    return s


def feed(agg,body=None,yaw=0.,mirrored=False,distance=1.):
    stamp=getattr(agg,'_cal_test_stamp',100.)
    for i in range(16):agg._on_scan_raw(panel_scan(body,yaw,mirrored,distance,stamp+i*.1))
    agg._cal_test_stamp=stamp+2.


def prepare(agg):
    assert agg._dispatch('lidar_cal_geometry',dict(x_m=.139,y_m=-.002,z_m=.082,roll_deg=1.,pitch_deg=-1.))[0]=='accepted'
    feed(agg)
    assert agg._dispatch('lidar_cal_background',{})[0]=='accepted'
