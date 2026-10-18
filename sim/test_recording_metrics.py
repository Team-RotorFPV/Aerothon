from recording_metrics import delivered_metrics


def test_recorded_rates_count_distinct_captures_and_actual_arrivals():
    rows=[dict(received_stamp=t+.2,detail=dict(capture_stamp=t,processing_wall_s=.1))
          for t in (1,1.2,1.4,1.6)]
    rows.append(rows[-1])
    metrics=delivered_metrics(rows)
    assert metrics['distinct_captures']==4
    assert abs(metrics['capture_hz']-5)<1e-9
    assert abs(metrics['delivered_hz']-5)<1e-9
    assert abs(metrics['latency_sim_p95_s']-.2)<1e-9
    assert metrics['processing_wall_p95_s']==.1


def test_missing_capture_time_cannot_create_a_detector_rate():
    metrics=delivered_metrics([dict(received_stamp=1,detail={})]*10)
    assert metrics['distinct_captures']==0
    assert metrics['capture_hz'] is None


def test_wall_delivery_rate_is_distinct_from_simulation_rate():
    rows=[dict(received_stamp=t+.2,received_wall_s=100+4*t,
               detail=dict(capture_stamp=t)) for t in (1,1.2,1.4,1.6)]
    rows.append(dict(received_stamp=2.,received_wall_s=120.,detail=dict(capture_stamp=1.6)))
    metrics=delivered_metrics(rows)
    assert abs(metrics['delivered_hz']-5)<1e-9
    assert abs(metrics['delivered_wall_hz']-1.25)<1e-9
    assert metrics['distinct_captures']==4
