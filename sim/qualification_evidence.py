"""Read-only qualification of recorded onboard flight and delivery evidence."""
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/aerothon_mission/mission_bt'))
from mission_bt.delivery_observer import stable


def fresh_flight_record(row,max_age_s=1.5):
    """Return FCU capture time only for a fresh connected state observation."""
    try:
        flight=row['flight'];data=flight['data'];header=data['header']['stamp']
        captured=header['sec']+header['nanosec']*1e-9
        received=flight['received_stamp'];observed=row['stamp']
        if (data['connected'] is not True or type(data['armed']) is not bool or
                not all(math.isfinite(t) and t>0 for t in (captured,received,observed)) or
                not 0<=observed-captured<=max_age_s or
                not 0<=observed-received<=max_age_s):return None
        return captured
    except (KeyError,TypeError,ValueError):return None


def recorded_disarm(rows):
    """Require an armed-to-disarmed transition and a fresh final connected state."""
    armed_capture=None;transition=None
    for row in rows:
        captured=fresh_flight_record(row)
        if captured is None:continue
        if row['flight']['data']['armed']:
            armed_capture=captured;transition=None
        elif armed_capture is not None and captured>armed_capture and transition is None:
            transition=dict(capture_stamp=captured,received_stamp=row['flight']['received_stamp'],
                recorded_stamp=row['stamp'],connected=True,armed=False)
    if (not rows or fresh_flight_record(rows[-1]) is None or
            rows[-1]['flight']['data']['armed'] is not False):return None
    return transition


def camera_through_disarm(out,rows):
    """Check the capture ledger and decoded export against actual FCU disarm."""
    try:
        out=Path(out);disarm=recorded_disarm(rows)
        if disarm is None:return False
        timeline=json.loads((out/'video_timeline.json').read_text())
        frames=[json.loads(line) for line in (out/'camera_frames.jsonl').read_text().splitlines()]
        stamps=[r['capture_stamp'] for r in frames]
        if (len(stamps)<2 or not all(math.isfinite(t) and t>0 for t in stamps) or
                any(b<=a for a,b in zip(stamps,stamps[1:]))):return False
        expected=stamps[-1]-stamps[0]+.2
        return bool(timeline['decode_pass'] is True and
            (out/'onboard_sim_time.mp4').stat().st_size>0 and
            timeline['original_frames']==len(stamps) and
            int(timeline['export']['nb_read_frames'])==len(stamps)+1 and
            abs(timeline['first_capture_stamp']-stamps[0])<1e-6 and
            abs(timeline['last_capture_stamp']-stamps[-1])<1e-6 and
            abs(timeline['final_hold_s']-.2)<1e-6 and
            abs(timeline['expected_duration_s']-expected)<1e-6 and
            abs(float(timeline['export']['duration'])-expected)<=.1 and
            stamps[-1]>=max(disarm['capture_stamp'],disarm['received_stamp']))
    except (OSError,KeyError,TypeError,ValueError,IndexError):return False


def motor_delivery_confirmed(rows,profile):
    """Require actual fresh motor outputs, not just a cached DONE label."""
    try:
        p=profile['motor_config']; indices=p['motor_channels']
        minimum,maximum=p['pwm_min'],p['pwm_max']
        ceiling=(profile['loaded_mean']-3*profile['loaded_std']+
                 profile['unloaded_mean']+3*profile['unloaded_std'])/2
        if not indices or maximum<=minimum:return False
    except (KeyError,TypeError):return False
    for row in rows:
        try:
            observation=row['observation'];stamp=observation['stamp']
            if not (observation['state']=='DONE' and observation['delivery_confirmed']
                    and row['flight']['data']['armed'] is True
                    and observation['touchdown_inferred'] and observation['detachment_inferred']
                    and row['winch']['payout_estimate_m']<=.02
                    and stable(observation['decision_telemetry'],stamp)):continue
            motor=row['motor_outputs']['data']; header=motor['header']['stamp']
            captured=header['sec']+header['nanosec']*1e-9
            if not (0<=row['stamp']-captured<=.5 and
                    0<=row['stamp']-stamp<=.5 and abs(stamp-captured)<=.5):continue
            values=[motor['channels'][i] for i in indices]
            if not all(math.isfinite(v) and minimum<=v<=maximum for v in values):continue
            effort=sum(((v-minimum)/(maximum-minimum))**2 for v in values)/len(values)
            if effort<ceiling and observation['decision_telemetry']['effort']<ceiling:
                return True
        except (KeyError,TypeError,IndexError,ValueError):continue
    return False


if __name__=='__main__':
    import json
    out=Path(sys.argv[1])
    rows=[json.loads(line) for line in (out/'onboard.jsonl').read_text().splitlines()]
    profile=json.loads((out/'frozen_calibration.json').read_text())
    passed=motor_delivery_confirmed(rows,profile)
    print(json.dumps(dict(motor_delivery_qualified=passed)))
    raise SystemExit(0 if passed else 1)
