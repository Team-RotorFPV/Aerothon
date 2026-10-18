"""Delivery decisions from camera and FCU observations only.

No ROS or simulator imports. Effort is mean squared normalized motor output,
not measured thrust or current. A separately measured calibration is required.
"""
from collections import deque
import math
import statistics


def fresh_telemetry(t, now):
    keys = ('stamp', 'alt', 'vz', 'speed', 'tilt_deg', 'accel_residual', 'effort')
    return bool(t and all(isinstance(t.get(k), (int, float)) and
                         math.isfinite(t[k]) for k in keys) and
                0 <= now - t['stamp'] <= 0.5 and t['alt'] > 0 and
                0 < t['effort'] < 1)


def stable(t, now):
    return (fresh_telemetry(t,now) and abs(t['vz']) <= 0.15 and
            t['speed'] <= 0.25 and t['tilt_deg'] <= 8 and
            abs(t['accel_residual']) <= 0.6)


def calibration(loaded, unloaded):
    """Freeze a profile from two independent stable-hover effort recordings."""
    if min(len(loaded), len(unloaded)) < 30:
        raise ValueError('need at least 30 stable samples in each hover')
    if not all(math.isfinite(x) and 0 < x < 1 for x in loaded + unloaded):
        raise ValueError('invalid motor effort sample')
    lm, um = statistics.mean(loaded), statistics.mean(unloaded)
    ls, us = statistics.stdev(loaded), statistics.stdev(unloaded)
    if lm - 3 * ls <= um + 3 * us or lm - um < 0.002:
        raise ValueError('loaded/unloaded motor effort is not distinguishable')
    return dict(version=1, source='fcu_motor_output_squared', loaded_mean=lm,
                unloaded_mean=um, loaded_std=ls, unloaded_std=us)


class DeliveryObserver:
    # The release test's retraction. It also lifts the EMPTY claw off the
    # ground while unloaded hover is confirmed, which can take ~15 s of
    # waiting for steady telemetry. At 0.25 m a 0.35 m sag left the claw on
    # the field; the aircraft drifted, the claw dragged 2 m off the pad and
    # the taut line yanked it to 4.16 m (rendered, clarified arena, p2r6;
    # 5.5 cm payload). A payload still attached is lifted the more clearly.
    PROBE_RETRACT_M = 0.6

    def __init__(self, profile, hfov, payload_size=0.10, max_attempts=3):
        self.profile = profile
        self.hfov, self.payload_size = hfov, payload_size
        self.max_attempts = max_attempts
        self.state = 'BASELINE'
        self.attempt = 1
        self.reason = 'loaded hover baseline pending'
        self.entered = None
        self.started = None
        self.last_frame = self.last_motor = None
        self.frames = deque(maxlen=10)
        self.centres = deque(maxlen=10)
        self.efforts = deque(maxlen=30)
        self.baseline = deque(maxlen=30)
        self.probe_payout = None
        self.range_m = None
        self.confirmed = False
        self.touchdown_inferred = False
        self.detachment_inferred = False
        self.telemetry_missing_since = None
        self.camera_missing_since = None
        self.continued_unconfirmed = False
        self.recovery_reason = None
        self.load_reduction_inferred = False
        self.candidate_efforts = deque(maxlen=30)
        self.last_candidate_motor = None
        self.provisional_probe_used = False
        self.decision_telemetry = None
        self.telemetry_stable = False

    def _profile_ok(self):
        p = self.profile
        try:
            values = [p[k] for k in ('loaded_mean', 'unloaded_mean',
                                     'loaded_std', 'unloaded_std')]
            return (p['version'] == 1 and p['source'] == 'fcu_motor_output_squared'
                    and all(math.isfinite(x) for x in values)
                    and 0 < p['unloaded_mean'] < p['loaded_mean'] < 1
                    and min(p['loaded_std'], p['unloaded_std']) >= 0
                    and p['loaded_mean'] - 3*p['loaded_std'] >
                    p['unloaded_mean'] + 3*p['unloaded_std']
                    and p['loaded_mean'] - p['unloaded_mean'] >= 0.002)
        except (TypeError, KeyError):
            return False

    def _enter(self, state, now, reason):
        self.state, self.entered, self.reason = state, now, reason
        if state != 'DONE':
            self.frames.clear()
            self.centres.clear()
            self.efforts.clear()

    def _retry(self, now, reason):
        # Each attempt gets a fresh observation window. Carrying an expired
        # outage into LOWER consumed the remaining attempts on consecutive ticks.
        self.telemetry_missing_since = None
        self.camera_missing_since = None
        self.candidate_efforts.clear()
        if self.attempt >= self.max_attempts:
            self.recovery_reason = reason + '; three attempts exhausted'
            self._enter('RECOVER_STOW', now, self.recovery_reason + '; retracting before mission continuation')
        else:
            self.attempt += 1
            self.touchdown_inferred = self.detachment_inferred = False
            self._enter('LOWER', now, 'retry: ' + reason)
        return 'stop'

    def _unloaded(self):
        return self._sustained_reduction(self.efforts)

    def _unloading_ceiling(self):
        p=self.profile
        return (p['loaded_mean']-3*p['loaded_std']+
                p['unloaded_mean']+3*p['unloaded_std'])/2

    def _sustained_reduction(self, samples):
        if len(samples) < 10 or samples[-1][0] - samples[0][0] < 1.0:
            return False
        return all(v < self._unloading_ceiling() for _, v in samples)

    def _ground(self):
        return (len(self.frames) >= 5 and self.frames[-1][0] - self.frames[0][0] >= 0.6
                and all(abs(r - a) <= max(0.35, a * 0.12) for _, r, a in self.frames)
                and max(r-a for _, r, a in self.frames) -
                min(r-a for _, r, a in self.frames) < 0.3
                and len(self.centres) == len(self.frames)
                and max(x for x, _ in self.centres)-min(x for x, _ in self.centres) < 0.12
                and max(y for _, y in self.centres)-min(y for _, y in self.centres) < 0.12)

    def update(self, now, det, telemetry, payout):
        """Return winch command. Repeated frames/telemetry never count twice."""
        if self.started is None:
            self.started = self.entered = now
        self.telemetry_stable = stable(telemetry,now)
        self.decision_telemetry = dict(telemetry) if fresh_telemetry(telemetry,now) else None
        if self.state in ('FAILED', 'DONE', 'RECOVERED'):
            return 'stop'
        if not isinstance(payout,(int,float)) or not math.isfinite(payout) or payout<0:
            self._enter('FAILED',now,'invalid winch travel estimate')
            return 'stop'
        if self.state == 'RECOVER_STOW':
            # Exhaustion changes the mission policy, not delivery evidence.
            # Retract even when the camera or load channel is inconclusive.
            # The aircraft's flight safety monitor and winch fault handling
            # remain active outside this observer.
            if stable(telemetry, now) and self._profile_ok():
                if telemetry['stamp'] != self.last_motor:
                    self.last_motor = telemetry['stamp']
                    self.efforts.append((now, telemetry['effort']))
                self.load_reduction_inferred = self._unloaded()
            else:
                self.efforts.clear()
                self.load_reduction_inferred = False
            if not math.isfinite(payout) or payout < 0:
                self._enter('FAILED', now, 'invalid winch travel estimate during recovery')
                return 'stop'
            if payout <= 0.02:
                self.continued_unconfirmed = True
                self._enter('RECOVERED', now, self.recovery_reason +
                            '; hook retraction estimate complete; continuing with delivery unconfirmed')
                return 'stop'
            if now-self.entered > 45:
                self._enter('FAILED', now, 'hook recovery retraction timed out')
                return 'stop'
            return 'stow'
        if not self._profile_ok():
            self._enter('FAILED', now, 'load calibration missing or unqualified')
            return 'stop'
        if now - self.started > 300:
            self._enter('FAILED', now, 'delivery time budget exhausted')
            return 'stop'
        # Motor effort can fall during contact before a stable hover is
        # possible. Keep this provisional evidence separate from qualification.
        # Estimated travel gates a bounded diagnostic retraction, never proves
        # ground contact. The final motor threshold and stability gates stay fixed.
        if self.state in ('LOWER','TOUCHDOWN_WAIT'):
            near_reach = (fresh_telemetry(telemetry,now) and math.isfinite(payout)
                          and payout >= max(0,telemetry['alt']-0.5))
            if near_reach and telemetry['effort'] < self._unloading_ceiling():
                stamp=telemetry['stamp']
                if stamp != self.last_candidate_motor:
                    if self.candidate_efforts and stamp-self.candidate_efforts[-1][0] > .5:
                        self.candidate_efforts.clear()
                    self.last_candidate_motor=stamp
                    self.candidate_efforts.append((stamp,telemetry['effort']))
            else:
                self.candidate_efforts.clear()
            if not self.telemetry_stable and self._sustained_reduction(self.candidate_efforts):
                self.probe_payout=payout
                self.provisional_probe_used=True
                self._enter('CANDIDATE_PROBE',now,
                            'provisional motor reduction near estimated reach; bounded release test, touchdown unconfirmed')
                return 'stop'
        if self.state == 'CANDIDATE_PROBE' and fresh_telemetry(telemetry,now):
            # Stop an extended mechanism pressing against supported payload;
            # only stable unloaded hover after this probe can certify release.
            if self.probe_payout-payout >= self.PROBE_RETRACT_M:
                self._enter('PROBE_HOLD',now,'bounded candidate retraction complete; awaiting qualified hover')
                return 'stop'
            if now-self.entered>45:
                return self._retry(now,'candidate release test timed out')
            return 'stow'
        if not stable(telemetry, now):
            self.load_reduction_inferred = False
            if self.telemetry_missing_since is None:
                self.telemetry_missing_since = now
            self.frames.clear(); self.centres.clear(); self.efforts.clear()
            if self.state == 'BASELINE':
                self.baseline.clear()
            self.reason = 'waiting for fresh stable onboard telemetry'
            if now - self.telemetry_missing_since > 15:
                if self.state == 'BASELINE':
                    self._enter('FAILED', now, 'stable loaded hover baseline unavailable')
                    return 'stop'
                return self._retry(now, 'telemetry unavailable or flight unstable')
            return 'stop'
        self.telemetry_missing_since = None
        t = telemetry
        if t['stamp'] != self.last_motor:
            self.last_motor = t['stamp']
            self.efforts.append((now, t['effort']))
            if self.state == 'BASELINE':
                self.baseline.append((now, t['effort']))
        self.load_reduction_inferred = self._unloaded()
        if self.state == 'BASELINE':
            if len(self.baseline) == 30 and now-self.baseline[0][0] >= 2:
                mean = statistics.mean(v for _, v in self.baseline)
                p = self.profile
                if abs(mean-p['loaded_mean']) > max(3*p['loaded_std'], 0.2*(p['loaded_mean']-p['unloaded_mean'])):
                    self._enter('FAILED', now, 'loaded hover does not match frozen calibration')
                else:
                    self._enter('ACQUIRE', now, 'loaded hover verified; deploying into camera view')
            return 'stop'
        if self.state == 'ACQUIRE':
            # The stowed payload is beneath the airframe and can be occluded.
            # Deployment is bounded by estimated travel and onboard altitude.
            if det and det.get('visible') and isinstance(det.get('stamp'), (int, float)) and 0 <= now-det['stamp'] <= 1:
                self._enter('LOWER', now, 'payload acquired by onboard camera')
                return 'stop'
            if now-self.entered > 15:
                return self._retry(now, 'payload not acquired after bounded deployment')
            return 'lower' if payout < min(0.6, max(0, t['alt']-2.0)) else 'stop'
        fresh = (det and det.get('visible') and isinstance(det.get('stamp'), (int, float))
                 and 0 <= now-det['stamp'] <= 1.0)
        if fresh and det['stamp'] != self.last_frame:
            self.last_frame = det['stamp']
            size = max(det.get('w_px', 0), det.get('h_px', 0))
            xy = (det.get('x', 0.0), det.get('y', 0.0))
            if math.isfinite(size) and size > 0 and all(math.isfinite(v) and abs(v) <= 1 for v in xy):
                self.range_m = self.payload_size*det.get('img_w', 1280)/(2*math.tan(self.hfov/2)*size)
                self.frames.append((det['stamp'], self.range_m, t['alt']))
                self.centres.append(xy)
            else:
                fresh = False
        if not fresh:
            if self.camera_missing_since is None:
                self.camera_missing_since = now
            self.frames.clear()
            self.centres.clear()
            self.reason = 'payload not visible in a fresh onboard camera frame'
            # Preserve bounded visual acquisition/lowering, but let the primary
            # load signal initiate and finish a release test without a camera.
            if self.state in ('LOWER', 'TOUCHDOWN_WAIT') and not self._unloaded():
                if now-self.camera_missing_since > 30:
                    return self._retry(now, 'camera evidence unavailable')
                return 'stop'
        else:
            self.camera_missing_since = None
        if self.state in ('LOWER', 'TOUCHDOWN_WAIT'):
            if self._unloaded():
                self.touchdown_inferred = True
                self.probe_payout = payout
                self._enter('PROBE', now, 'sustained motor effort reduction infers touchdown; testing release')
                return 'stop'
            if self.state == 'TOUCHDOWN_WAIT':
                # Deployment transients can briefly reduce effort before
                # contact. A sustained stable loaded hover clears that
                # unconfirmed candidate; it is not a failed release probe.
                loaded_again=(len(self.efforts)>=10 and
                    self.efforts[-1][0]-self.efforts[0][0]>=1 and
                    all(v>=self._unloading_ceiling() for _,v in self.efforts))
                if loaded_again and not self.touchdown_inferred:
                    self.candidate_efforts.clear()
                    self._enter('LOWER',now,
                        'stable suspended load clears unconfirmed touchdown candidate; resuming lowering')
                    return 'stop'
                if now-self.entered > 15:
                    return self._retry(now, 'possible touchdown not corroborated')
                return 'stop'
            # A single plausible observation can stop payout safely, but cannot
            # certify touchdown. Hold travel while the two sustained channels
            # settle; otherwise a rigid simulated winch can press into ground.
            p = self.profile
            ceiling = (p['loaded_mean'] - 3*p['loaded_std'] +
                       p['unloaded_mean'] + 3*p['unloaded_std']) / 2
            if t['effort'] < ceiling:
                self._enter('TOUCHDOWN_WAIT', now, 'possible touchdown; holding for corroboration')
                return 'stop'
            if now-self.entered > 90:
                return self._retry(now, 'touchdown not corroborated')
            return 'lower'
        if self.state in ('PROBE', 'STOW', 'PROBE_HOLD', 'FINAL'):
            # Returned suspended load is the primary contradiction to release.
            loaded = len(self.efforts) >= 10 and statistics.mean(v for _, v in self.efforts) >= (
                self.profile['loaded_mean'] + self.profile['unloaded_mean'])/2
            if loaded:
                return self._retry(now, 'suspended load returned during retraction')
        if self.state in ('PROBE', 'STOW'):
            if self.state == 'PROBE' and self.probe_payout-payout >= self.PROBE_RETRACT_M:
                self._enter('PROBE_HOLD', now, 'short retraction complete; checking payload remains')
                return 'stop'
            if self.state == 'STOW' and payout <= 0.02:
                self._enter('FINAL', now, 'retraction estimate complete; confirming unloaded hover')
                return 'stop'
            if now-self.entered > 45:
                return self._retry(now, 'retraction estimate timed out')
            return 'stow'
        if self.state in ('PROBE_HOLD', 'FINAL'):
            if self._unloaded():
                if self.state == 'PROBE_HOLD':
                    self.touchdown_inferred = True
                    self._enter('STOW', now, 'motor effort remained unloaded after short retraction')
                else:
                    self.confirmed = self.detachment_inferred = True
                    self._enter('DONE', now, 'motor effort confirms unloaded hover after full hook retraction')
                return 'stop'
            if now-self.entered > 10:
                return self._retry(now, 'release verification inconclusive')
            return 'stop'
        return 'stop'

    def status(self):
        return dict(state=self.state, attempt=self.attempt, reason=self.reason,
                    touchdown_inferred=self.touchdown_inferred,
                    detachment_inferred=self.detachment_inferred,
                    delivery_confirmed=self.confirmed, camera_range_estimate_m=self.range_m,
                    load_reduction_inferred=self.load_reduction_inferred,
                    provisional_probe_used=self.provisional_probe_used,
                    provisional_motor_samples=len(self.candidate_efforts),
                    telemetry_stable=self.telemetry_stable,
                    decision_telemetry=self.decision_telemetry,
                    continued_unconfirmed=self.continued_unconfirmed,
                    decision_priority='calibrated_motor_output',
                    visual_ground_support_inferred=self._ground(),
                    visual_frames=len(self.frames), load_samples=len(self.efforts))
