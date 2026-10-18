"""Reproduce lost lidar detail and false single-return calibration."""
import math

from sensor_msgs.msg import LaserScan
from test_lidar_calibration_gcs import aggregator  # noqa: F401


def scan720():
    s = LaserScan()
    s.range_min, s.range_max = .05, 12.
    s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 720
    s.angle_max = s.angle_min + 719 * s.angle_increment
    s.ranges = [3.] * 720
    return s


def test_live_view_preserves_c1_beams_and_narrow_obstacles(aggregator):
    s = scan720()
    s.ranges[13] = .7
    aggregator._on_scan(s)
    out = aggregator.state['scan']
    assert len(out['ranges']) == 720, 'the UI throws away 90 percent of C1 beams'
    assert out['ranges'][13] == .7, 'a narrow nearby obstacle vanished from the UI'
    assert out['angle_step'] == s.angle_increment


def test_a_single_spike_is_not_accepted_as_a_calibration_target(aggregator):
    s = scan720()
    s.ranges[13] = .4
    aggregator._on_scan_raw(s)
    result, reason = aggregator._dispatch('lidar_cal_capture', {'where': 'ahead'})
    assert result == 'rejected', 'one scan and one spike cannot measure a mount'
    assert reason


def test_scan_mount_bounds_are_respected_in_readout():
    from gcs_aggregator.lidar_calibration import nearest_bearing
    s = scan720()
    s.ranges = [20.] * 720
    assert nearest_bearing(s) is None, 'out-of-range returns were treated as measurements'


def test_map_run_lengths_do_not_lose_walls_or_overflow_large_free_spaces():
    from gcs_aggregator.aggregator import pack_grid
    original=[-1]*10+[0]*70000+[100]*150
    packed=pack_grid(original)
    assert packed['encoding']=='rle'
    runs=packed['rle']
    restored=[value for i in range(0,len(runs),2) for value in [runs[i]]*runs[i+1]]
    assert restored==original
    assert pack_grid([-1,0,100])=={'data':[-1,0,100]}
