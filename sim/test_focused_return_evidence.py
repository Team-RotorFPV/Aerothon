"""A focused pass requires camera evidence and a real connected FCU disarm."""
import json
import runpy
import sys
from pathlib import Path

import pytest
import campaign_record
import check_track


@pytest.mark.parametrize('defect', [None, 'missing_timeline', 'early_camera', 'decode_failure',
                                    'stale_disarm', 'disconnected', 'missing_video', 'ledger_mismatch'])
def test_focused_grader_rejects_incomplete_landing_evidence(tmp_path, monkeypatch, defect):
    def write(name, value): (tmp_path/name).write_text(json.dumps(value))
    def flight(t, armed, connected=True):
        return dict(stamp=t, flight=dict(received_stamp=t, data=dict(
            header=dict(stamp=dict(sec=int(t), nanosec=0)), connected=connected, armed=armed)))
    write('evaluation_track.json', [dict(t=10.+i*.01) for i in range(101)])
    (tmp_path/'trial_driver.jsonl').write_text(json.dumps(dict(stamp=9., phase='RETURN_CORRIDOR'))+'\n'+
        json.dumps(dict(stamp=11.5, phase='LAND'))+'\n')
    write('layout.json', {})
    write('trial_result.json', dict(state='TRIAL_COMPLETE', final_armed=False))
    write('decision_inputs_navigation.json', {'pass': True})
    write('configuration.json', dict(handover_mode='moving'))
    rows = [flight(10., True), flight(12., False)]
    if defect == 'disconnected': rows[-1]['flight']['data']['connected'] = False
    if defect == 'stale_disarm': rows[-1]['flight']['data']['header']['stamp']['sec'] = 8
    (tmp_path/'onboard.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    last = 11.9 if defect == 'early_camera' else 12.2
    captures = [dict(capture_stamp=9.), dict(capture_stamp=last)]
    (tmp_path/'camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in captures))
    if defect != 'missing_timeline':
        write('video_timeline.json', dict(decode_pass=defect != 'decode_failure', original_frames=2,
            first_capture_stamp=9., last_capture_stamp=last, final_hold_s=.2,
            expected_duration_s=last-9.+.2,
            export=dict(duration=str(last-9.+.2), nb_read_frames='3')))
    if defect == 'ledger_mismatch':
        captures[-1]['capture_stamp'] = 12.4
        (tmp_path/'camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in captures))
    if defect != 'missing_video': (tmp_path/'onboard_sim_time.mp4').write_bytes(b'fixture')
    monkeypatch.setattr(campaign_record, 'navigation_grade', lambda *args: dict(
        checks={'corridor clearance at least 0.70 m': True}, measured_corridor_clearance_m=.8))
    monkeypatch.setattr(check_track, 'traversal', lambda *args, **kwargs: (dict(found=True, n_bad=0), None))
    monkeypatch.setattr(sys, 'argv', ['grade_focused_return', str(tmp_path)])
    with pytest.raises(SystemExit) as exit_result:
        runpy.run_path(str(Path(__file__).resolve().parents[1]/'.scratch/simulation-reliability/grade_focused_return.py'))
    summary = json.loads((tmp_path/'focused_summary.json').read_text())
    assert summary['focused_pass'] is (defect is None)
    assert exit_result.value.code == (0 if defect is None else 1)
