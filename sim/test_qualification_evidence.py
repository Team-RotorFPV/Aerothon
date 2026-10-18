"""Cached observer labels cannot qualify stale or contradictory FCU data."""
from qualification_evidence import motor_delivery_confirmed,recorded_disarm

PROFILE=dict(loaded_mean=.424,unloaded_mean=.407,loaded_std=.0004,unloaded_std=.0005,
             motor_config=dict(motor_channels=[0,1,2,3],pwm_min=1000,pwm_max=2000))

def record():
    return dict(stamp=10.,flight=dict(data=dict(armed=True)),observation=dict(state='DONE',stamp=10.,delivery_confirmed=True,
        touchdown_inferred=True,detachment_inferred=True,
        decision_telemetry=dict(stamp=10.,effort=.407,alt=5.5,vz=0,speed=0,
                                tilt_deg=0,accel_residual=0)),
        winch=dict(payout_estimate_m=0.),motor_outputs=dict(data=dict(
            header=dict(stamp=dict(sec=10,nanosec=0)),channels=[1638]*4)))

def test_qualified_record_requires_real_unloaded_outputs():
    row=record();assert motor_delivery_confirmed([row],PROFILE)
    row['motor_outputs']['data']['channels']=[1651]*4
    assert not motor_delivery_confirmed([row],PROFILE)

def test_stale_or_future_motor_stamp_cannot_qualify():
    for second in (9,11):
        row=record();row['motor_outputs']['data']['header']['stamp']['sec']=second
        assert not motor_delivery_confirmed([row],PROFILE)

def test_unconfirmed_continuation_or_extended_hook_cannot_qualify_delivery():
    row=record();row['observation']['state']='RECOVERED'
    assert not motor_delivery_confirmed([row],PROFILE)
    row=record();row['winch']['payout_estimate_m']=.5
    assert not motor_delivery_confirmed([row],PROFILE)
    row=record();row['flight']['data']['armed']=False
    row['motor_outputs']['data']['channels']=[1100]*4
    assert not motor_delivery_confirmed([row],PROFILE)


def flight(t,armed,connected=True):
    return dict(stamp=t,flight=dict(received_stamp=t,data=dict(connected=connected,armed=armed,
        header=dict(stamp=dict(sec=int(t),nanosec=round((t-int(t))*1e9))))))


def test_recorded_disarm_requires_armed_transition_and_fresh_final_connected_state():
    armed=flight(10.,True);disarmed=flight(12.,False)
    assert recorded_disarm([armed,disarmed])['capture_stamp']==12.
    assert recorded_disarm([disarmed]) is None
    assert recorded_disarm([armed,flight(12.,False,False)]) is None
    stale=flight(12.,False);stale['flight']['data']['header']['stamp']['sec']=8
    assert recorded_disarm([armed,stale]) is None
    assert recorded_disarm([armed,disarmed,flight(13.,False,False)]) is None
    assert recorded_disarm([armed,disarmed,flight(13.,True)]) is None


def test_duplicate_or_future_state_cannot_extend_the_disarm_evidence():
    armed=flight(10.,True);old=flight(9.,False);old['stamp']=10.1
    assert recorded_disarm([armed,old]) is None
    future=flight(12.,False);future['stamp']=11.
    assert recorded_disarm([armed,future]) is None
