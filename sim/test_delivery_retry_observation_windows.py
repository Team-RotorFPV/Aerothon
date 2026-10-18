"""Regression for the retry collapse seen in rendered seed 20261003.

Synthetic inputs test timer policy only, never qualify delivery sensors.
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/aerothon_mission/mission_bt'))
from mission_bt.delivery_observer import DeliveryObserver, calibration


class RetryWindows(unittest.TestCase):
    def observer(self):
        o = DeliveryObserver(calibration([.26, .2605]*20, [.24, .2405]*20), 1.0472)
        o.state = 'LOWER'
        o.started = o.entered = 0
        return o

    def test_telemetry_fault_gets_a_new_window_for_each_attempt(self):
        o = self.observer()
        o.update(1, None, None, 5)
        o.update(16.1, None, None, 5)
        self.assertEqual(o.attempt, 2)
        o.update(16.3, None, None, 5)
        self.assertEqual(o.attempt, 2, 'one expired episode consumed another attempt')
        self.assertEqual(o.state, 'LOWER')
        o.update(31.4, None, None, 5)
        self.assertEqual(o.attempt, 3)
        o.update(31.6, None, None, 5)
        self.assertEqual(o.state, 'LOWER')
        o.update(46.7, None, None, 5)
        self.assertEqual(o.state, 'RECOVER_STOW')
        self.assertFalse(o.confirmed)

    def test_camera_fault_gets_a_new_window_for_each_attempt(self):
        o = self.observer()
        def step(now):
            t = dict(stamp=now, effort=.26, alt=5, vz=0, speed=0,
                     tilt_deg=0, accel_residual=0)
            return o.update(now, dict(visible=False, stamp=now), t, 5)
        step(1)
        step(31.1)
        self.assertEqual(o.attempt, 2)
        step(31.3)
        self.assertEqual(o.attempt, 2, 'old camera outage consumed another attempt')
        step(61.4)
        self.assertEqual(o.attempt, 3)
        step(61.6)
        self.assertEqual(o.state, 'LOWER')
        step(91.7)
        self.assertEqual(o.state, 'RECOVER_STOW')
        self.assertFalse(o.confirmed)


if __name__ == '__main__':
    unittest.main()
