"""The actual recording callback must preserve causal winch contact evidence."""
import ast
import copy
import io
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sim'))
from test_native_contact_evidence import recording,evaluate,deployed_hook


def detail_callback(values,stamp,trace):
    # Exercise the real callback without starting a ROS node or video encoder.
    tree=ast.parse((ROOT/'sim/record_delivery.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    detail=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='detail')
    scope=dict(values=values,now=lambda:stamp,trace=trace,
               detector_events=io.StringIO(),json=json,time=time)
    exec(compile(ast.Module(body=[detail],type_ignores=[]),'record_delivery_detail','exec'),scope)
    return scope['detail']


def capture_arrival(recording,fault=None):
    row=deployed_hook(recording)
    # Actual V16 contact/receipt/snapshot times, translated by 270.112 s.
    # The later recorded snapshot proved a callback at 3.088, before contact.
    row['stamp']=3.189
    snapshot=recording[2][1]
    snapshot['winch_capture']['received_stamp']=2.888
    values=copy.deepcopy({k:v for k,v in snapshot.items() if k!='stamp'})
    values['winch']=copy.deepcopy(snapshot['winch_capture']['data'])
    arrival=3.088
    if fault=='stale':arrival=2.887
    elif fault=='future':arrival=3.190
    elif fault=='stowed':values['winch']['payout_estimate_m']=0.
    elif fault=='fault':values['winch']['fault']='sensor failure'
    elif fault=='other_phase':values['mission']='RETURN_CORRIDOR'
    elif fault=='off_target':row['contacts'][0]['positions'][0]['x']=2.
    elif fault=='wall':row['contacts'][0]['collision2']='return_banner::board::collision'
    elif fault=='missing_positions':row['contacts'][0]['positions']=[]
    stream=io.StringIO()
    detail_callback(values,arrival,stream)('winch',SimpleNamespace(data=json.dumps(values['winch'])))
    entries=[json.loads(line) for line in stream.getvalue().splitlines()]
    recording[2].extend(entries);recording[2].sort(key=lambda r:r['stamp'])
    return entries


def test_actual_arrival_preserves_fresh_hook_contact_before_video_tick(recording):
    entries=capture_arrival(recording)
    result=evaluate(recording)
    assert result['valid'] and result['contact_free'],result
    assert result['intended_deployed_hook_target_surface_pairs']==1
    assert entries[0]['stamp']==entries[0]['winch_capture']['received_stamp']==3.088


@pytest.mark.parametrize('fault',['stale','future','stowed','fault','other_phase','off_target','wall','missing_positions'])
def test_arrival_journal_never_qualifies_unsafe_or_noncausal_contact(recording,fault):
    capture_arrival(recording,fault)
    result=evaluate(recording)
    assert result['valid'] and not result['contact_free']
