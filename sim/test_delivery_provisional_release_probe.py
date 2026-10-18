"""Regression specification for resolving contact support without false success.

Synthetic observations exercise observer policy only. Acceptance must still
use the frozen profile and actual FCU measurements in a rendered trial.
"""
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_mission/mission_bt'))
from mission_bt.delivery_observer import DeliveryObserver,calibration


class ProvisionalProbe(unittest.TestCase):
    def setUp(self):
        self.o=DeliveryObserver(calibration([.26,.2605]*20,[.24,.2405]*20),1.0472)
        self.o.state='LOWER';self.o.started=self.o.entered=0
        self.now=1.;self.payout=4.8

    def step(self,effort=.24,unstable=True,stale=False):
        self.now=round(self.now+.1,3)
        t=dict(stamp=self.now-2 if stale else self.now,effort=effort,alt=5,
               vz=.3 if unstable else 0,speed=0,tilt_deg=0,accel_residual=0)
        command=self.o.update(self.now,dict(visible=False,stamp=self.now),t,self.payout)
        if command=='stow':self.payout=max(0,self.payout-.045)
        return command

    def begin_probe(self):
        for _ in range(25):
            self.step()
            if self.o.state=='CANDIDATE_PROBE':break
        self.assertEqual(self.o.state,'CANDIDATE_PROBE')
        self.assertFalse(self.o.touchdown_inferred)
        self.assertFalse(self.o.confirmed)

    def test_short_probe_resolves_support_but_never_confirms_unstable_hover(self):
        self.begin_probe()
        initial=self.payout
        for _ in range(50):
            self.step()
            if self.o.state=='PROBE_HOLD':break
        self.assertEqual(self.o.state,'PROBE_HOLD')
        # Bounded, but enough to lift the empty claw clear of the ground for
        # the wait (DeliveryObserver.PROBE_RETRACT_M; rendered p2r6).
        self.assertGreaterEqual(initial-self.payout,self.o.PROBE_RETRACT_M-1e-9)
        self.assertLessEqual(initial-self.payout,self.o.PROBE_RETRACT_M+.1)
        for _ in range(40):self.assertEqual(self.step(),'stop')
        self.assertFalse(self.o.confirmed)
        for _ in range(200):
            self.step(unstable=False)
            if self.o.confirmed:break
        self.assertTrue(self.o.confirmed)
        self.assertTrue(self.o.touchdown_inferred)
        self.assertLessEqual(self.payout,.02)

    def test_stale_telemetry_cannot_drive_candidate_probe(self):
        self.begin_probe()
        initial=self.payout
        for _ in range(10):self.assertEqual(self.step(stale=True),'stop')
        self.assertEqual(self.payout,initial)
        self.assertFalse(self.o.confirmed)

    def test_midair_motor_fall_cannot_initiate_unstable_probe(self):
        self.payout=1
        for _ in range(25):self.assertEqual(self.step(),'stop')
        self.assertEqual(self.o.state,'LOWER')
        self.assertFalse(self.o.confirmed)

    def test_returned_load_after_candidate_probe_retries(self):
        self.begin_probe()
        for _ in range(30):
            self.step()
            if self.o.state=='PROBE_HOLD':break
        for _ in range(40):
            self.step(effort=.26,unstable=False)
            if self.o.attempt==2:break
        self.assertEqual(self.o.attempt,2)
        self.assertFalse(self.o.confirmed)


if __name__=='__main__':unittest.main()
