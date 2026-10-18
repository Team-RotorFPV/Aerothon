"""Onboard evidence tests; no simulator contact or payload pose is provided."""
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/aerothon_mission/mission_bt'))
sys.path.insert(0, str(ROOT/'src/aerothon_sim/sim_gazebo'))
from mission_bt.delivery_observer import DeliveryObserver, calibration
from sim_gazebo.contact_release import surface_contact


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.profile = calibration([.26, .2605]*20, [.24, .2405]*20)
        self.o = DeliveryObserver(self.profile, 1.0472)
        self.time = 0
        self.payout = 0

    def step(self, effort=.26, size=22.17, visible=True, frame=True, telemetry=True):
        self.time = round(self.time + .1, 3)
        stamp = round(self.time, 3)
        d = dict(visible=visible, stamp=stamp if frame else 0,
                 w_px=size, h_px=size/2, img_w=1280)
        t = dict(stamp=stamp, effort=effort, alt=5, vz=0, speed=0,
                 tilt_deg=0, accel_residual=0) if telemetry else None
        cmd = self.o.update(self.time, d, t, self.payout)
        if cmd == 'lower': self.payout += .03
        if cmd == 'stow': self.payout = max(0, self.payout-.045)
        return cmd

    def baseline(self):
        for _ in range(35): self.step()
        self.assertEqual(self.o.state, 'LOWER')

    def touchdown(self):
        self.baseline()
        self.payout = 4.8
        for _ in range(35):
            self.step(effort=.24)
            if self.o.state == 'PROBE': break
        self.assertEqual(self.o.state, 'PROBE')

    def test_requires_calibration(self):
        self.o.profile = None
        self.step()
        self.assertEqual(self.o.state, 'FAILED')

    def test_calibration_rejects_indistinguishable_loads(self):
        with self.assertRaises(ValueError): calibration([.25]*40, [.25]*40)

    def test_missing_baseline_cannot_be_bypassed_by_retry(self):
        for _ in range(170): self.step(telemetry=False)
        self.assertEqual(self.o.state, 'FAILED')
        self.assertEqual(self.o.attempt, 1)

    def test_success_needs_unloaded_motor_signal_through_full_retraction(self):
        self.touchdown()
        for _ in range(250):
            self.step(effort=.24)
            if self.o.state == 'DONE': break
        self.assertEqual(self.o.state, 'DONE')
        self.assertTrue(self.o.confirmed)
        self.assertGreaterEqual(self.o.status()['visual_frames'], 5)
        self.assertGreaterEqual(self.o.status()['load_samples'], 10)
        self.assertLessEqual(self.payout, .02)

    def test_the_release_test_lifts_the_empty_claw_clear_before_waiting(self):
        # Rendered p2r6: after a 0.25 m probe the claw lay on the field through
        # a 16 s wait for steady telemetry and was dragged 2 m off the pad.
        self.touchdown()
        start = self.payout
        while self.o.state == 'PROBE':
            self.step(effort=.24)
        self.assertEqual(self.o.state, 'PROBE_HOLD')
        self.assertGreaterEqual(start - self.payout, 0.6 - 1e-9)

    def test_visual_ground_without_unloading_does_not_confirm(self):
        self.baseline()
        for _ in range(50): self.step(effort=.26)
        self.assertFalse(self.o.touchdown_inferred)

    def test_possible_touchdown_pauses_without_certifying_release(self):
        self.baseline()
        self.assertEqual(self.step(effort=.24), 'stop')
        self.assertEqual(self.o.state, 'TOUCHDOWN_WAIT')
        self.assertFalse(self.o.touchdown_inferred)
        self.assertFalse(self.o.confirmed)
        self.assertEqual(self.step(effort=.26), 'stop')

    def test_motor_unloading_initiates_probe_despite_wrong_camera_range(self):
        self.baseline()
        for _ in range(50): self.step(effort=.24, size=60)
        self.assertTrue(self.o.touchdown_inferred)
        self.assertFalse(self.o.confirmed)

    def test_duplicate_camera_frame_cannot_establish_visual_support(self):
        self.baseline()
        for _ in range(50): self.step(effort=.24, frame=False)
        self.assertTrue(self.o.touchdown_inferred)
        self.assertFalse(self.o.status()['visual_ground_support_inferred'])

    def test_retained_payload_retries_and_exhausts_three_attempts(self):
        self.touchdown()
        for attempt in (2, 3):
            for _ in range(40):
                self.step(effort=.26, size=60)
                if self.o.attempt == attempt: break
            self.assertEqual(self.o.attempt, attempt)
            self.assertEqual(self.o.state, 'LOWER')
            for _ in range(35):
                self.step(effort=.24)
                if self.o.state == 'PROBE': break
            self.assertEqual(self.o.state, 'PROBE')
        for _ in range(40):
            self.step(effort=.26, size=60)
            if self.o.state == 'RECOVER_STOW': break
        self.assertEqual(self.o.state, 'RECOVER_STOW')
        self.assertFalse(self.o.confirmed)
        for _ in range(250):
            self.step(visible=False, telemetry=False)
            if self.o.state == 'RECOVERED': break
        self.assertEqual(self.o.state, 'RECOVERED')
        self.assertTrue(self.o.continued_unconfirmed)
        self.assertFalse(self.o.confirmed)
        self.assertLessEqual(self.payout, .02)

    def test_recovery_cannot_continue_until_retraction_estimate_completes(self):
        self.baseline()
        self.o.attempt = 3
        self.payout = 6
        self.o._retry(self.time, 'camera inconclusive')
        self.assertEqual(self.step(visible=False), 'stow')
        self.assertFalse(self.o.continued_unconfirmed)
        self.assertFalse(self.o.confirmed)

    def test_recovery_stalled_winch_fails_instead_of_continuing_extended(self):
        self.baseline()
        self.o.attempt = 3
        self.o._retry(self.time, 'camera inconclusive')
        self.time += 46
        self.o.update(self.time, None, None, 6)
        self.assertEqual(self.o.state, 'FAILED')
        self.assertFalse(self.o.continued_unconfirmed)

    def test_lost_camera_does_not_veto_motor_driven_retraction(self):
        self.touchdown()
        self.assertEqual(self.step(effort=.24, visible=False), 'stow')
        for _ in range(250):
            self.step(effort=.24, visible=False)
            if self.o.state == 'DONE': break
        self.assertEqual(self.o.state, 'DONE')
        self.assertTrue(self.o.confirmed)
        self.assertFalse(self.o.status()['visual_ground_support_inferred'])
        self.assertLessEqual(self.payout, .02)

    def test_motor_return_prevents_success_even_if_camera_shows_ground(self):
        self.touchdown()
        for _ in range(40):
            self.step(effort=.26)
            if self.o.attempt == 2: break
        self.assertEqual(self.o.attempt, 2)
        self.assertFalse(self.o.confirmed)

    def test_initial_occlusion_allows_only_bounded_deployment(self):
        for _ in range(35): self.step(visible=False)
        self.assertEqual(self.o.state, 'ACQUIRE')
        for _ in range(40): self.step(visible=False)
        self.assertLessEqual(self.payout, .63)
        self.assertEqual(self.step(visible=False), 'stop')

    def test_stale_telemetry_stops_motion(self):
        self.touchdown()
        self.assertEqual(self.step(effort=.24, telemetry=False), 'stop')

    def test_brief_telemetry_loss_late_in_lowering_does_not_retry(self):
        self.baseline()
        for _ in range(170): self.step(size=60)
        self.assertEqual(self.step(size=60, telemetry=False), 'stop')
        self.assertEqual(self.o.attempt, 1)
        self.step(size=60)
        self.assertIsNone(self.o.telemetry_missing_since)

    def test_wind_or_vertical_motion_is_not_load_confirmation(self):
        self.baseline()
        t = dict(stamp=self.time, effort=.24, alt=5, vz=.8, speed=0,
                 tilt_deg=0, accel_residual=0)
        self.assertEqual(self.o.update(self.time, None, t, 3), 'stop')
        self.assertFalse(self.o.confirmed)

    def test_only_surface_contacts_can_detach(self):
        payload = 'aerothon_payload::body::collision'
        self.assertTrue(surface_contact(payload, 'site_ground::link::collision'))
        self.assertTrue(surface_contact('target_qr_b::link::collision', payload))
        self.assertTrue(surface_contact('takeoff_landing_zone::pad::collision', payload))
        self.assertFalse(surface_contact(payload, 'aerothon_iris::claw::tip'))
        self.assertFalse(surface_contact(payload, 'return_obstacle::link::collision'))
        from types import SimpleNamespace
        self.assertTrue(surface_contact(SimpleNamespace(name=payload),
                                        SimpleNamespace(name='site_ground::link::collision')))


class MotorConfirmationContinuationTests(unittest.TestCase):
    """Regression for the rendered run's post-confirmation camera KeyError.

    These unit fixtures exercise reporting after an already confirmed observer;
    they provide no load qualification or simulator delivery confirmation.
    """
    def build(self, detection):
        from types import SimpleNamespace
        from mission_bt.mission_tree import WinchDrop
        commands,goals=[],[]
        mav=SimpleNamespace(pos=lambda:(1.,2.,5.5),alt=lambda:5.5,yaw=lambda:0.,
            log=lambda *a,**kw:None,goto=lambda *a:goals.append(a),
            reached=lambda *a:True,winch=commands.append,
            winch_status={'payout_estimate_m':0.},
            payload_seen=lambda:detection,qr_offset=SimpleNamespace(x=0.,y=0.,z=1.))
        leaf=WinchDrop(mav,5.,10.)
        leaf.initialise()
        leaf.phase=1
        leaf.observer.state='DONE'
        leaf.observer.confirmed=True
        leaf.observer.reason='motor effort confirms unloaded hover after full hook retraction'
        return leaf,mav,commands,goals

    def test_unavailable_or_incomplete_camera_report_does_not_interrupt_return(self):
        for detection in (None, {'visible':False,'stamp':769.},
                          {'visible':True}, {'visible':True,'x':.1},
                          {'visible':True,'x':float('nan'),'y':0.},
                          {'visible':True,'x':0.,'y':float('inf')}):
            with self.subTest(detection=detection):
                leaf,mav,commands,goals=self.build(detection)
                leaf.update()
                self.assertTrue(mav.delivery_confirmed)
                self.assertIsNone(mav.delivery_offset_m)
                self.assertIn('unmeasured',mav.delivery_note)
                self.assertEqual(leaf.phase,5)
                self.assertEqual(commands,['stop'])
                self.assertEqual(leaf.update(),__import__('py_trees').common.Status.SUCCESS)
                self.assertEqual(goals[-1],(1.,2.,10.,0.))

    def test_valid_camera_coordinates_remain_a_separate_optional_report(self):
        leaf,mav,_,_=self.build({'visible':True,'x':.1,'y':-.1})
        leaf.update()
        self.assertTrue(mav.delivery_confirmed)
        self.assertGreater(mav.delivery_offset_m,0.)
        self.assertEqual(mav.delivery_confirmation_basis,'calibrated_motor_output_after_retraction')
        self.assertEqual(leaf.phase,5)


if __name__ == '__main__': unittest.main()
