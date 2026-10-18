"""Current runtime compatibility checks for the restored browser GCS."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np
import pytest
from sim import test_gcs_aggregator as support


@pytest.fixture
def node():
    case = support.TestGCSAggregator()
    case.setUp()
    case.node._clock = NS(now=lambda: NS(nanoseconds=100000000000))
    return case.node


def test_bundled_arena_schema_matches_production():
    from scripts.export_gcs_contract import contract
    root = Path(__file__).resolve().parents[1]
    assert json.loads((root / 'src/aerothon_gcs/tauri_app/src/arenaSchema.json').read_text()) == json.loads(json.dumps(contract()))


def test_sensor_rates_preserve_capture_age_and_stop_when_source_stalls():
    from gcs_aggregator.sensor_metrics import SensorMetric
    metric = SensorMetric()
    assert metric.snapshot(100., 10.)['ok'] is False
    with patch('time.monotonic', return_value=10.): metric.observe(NS(stamp=NS(sec=100, nanosec=0)))
    with patch('time.monotonic', return_value=10.1): metric.observe(NS(stamp=NS(sec=100, nanosec=100000000)))
    current = metric.snapshot(100.2, 10.2)
    assert current['hz'] == pytest.approx(10.)
    assert current['capture_age_s'] == pytest.approx(.1)
    assert metric.snapshot(104., 14.)['hz'] is None
    assert metric.snapshot(104., 14.)['ok'] is False
    assert metric.snapshot(99., 10.2)['capture_age_s'] is None


def test_real_numpy_clipping_and_magnetic_units(node):
    node._on_vibration(NS(vibration=NS(x=2., y=float('nan'), z=3.), clipping=np.array([0, 1, 2], dtype=np.float32)))
    assert node.state['sensor_diagnostics']['clipping'] == [0., 1., 2.]
    assert node.state['sensor_diagnostics']['vibration_mps2'] == [2., None, 3.]
    node._on_magnetic(NS(magnetic_field=NS(x=30e-6, y=40e-6, z=0.), header=NS(frame_id='base_link', stamp=NS(sec=10, nanosec=1))))
    assert node.state['sensor_diagnostics']['magnetic_strength_ut'] == pytest.approx(50.)
    assert node.state['sensor_diagnostics']['magnetic_frame'] == 'base_link'
    assert node.state['sensor_diagnostics']['magnetic_units_basis'] == 'tesla'
    json.dumps(node.state, allow_nan=False)


def test_mavros_milligauss_encoding_reads_as_earth_field(node):
    # ArduPilot RAW_IMU 300/400/0 mG through MAVROS's x1000 constant: the
    # Pi recorded 5.789e11 uT from this before; it is 50 uT.
    node._on_magnetic(NS(magnetic_field=NS(x=300e3, y=400e3, z=0.), header=NS(frame_id='base_link', stamp=NS(sec=10, nanosec=1))))
    diag = node.state['sensor_diagnostics']
    assert diag['magnetic_strength_ut'] == pytest.approx(50.)
    assert diag['magnetic_ut'] == pytest.approx([30., 40., 0.])
    assert diag['magnetic_units_basis'] == 'mavros_milligauss_x1000'


def test_source_ages_use_steady_time_and_do_not_refresh_each_other(node):
    node._ws_clients = {object()}
    node._broadcast = MagicMock()
    with patch('time.monotonic', return_value=10.): node._mark('lidar')
    with patch('time.monotonic', return_value=15.):
        node._mark('state')
        node._publish_snapshot()
    assert node.state['sample_age_s']['lidar'] == 5.
    assert node.state['sample_age_s']['state'] == 0.


def test_duplicate_release_is_not_republished_after_reconnect(node):
    first, second = AsyncMock(), AsyncMock()
    raw = json.dumps({'kind':'command', 'data':{'cmd_id':'release-once', 'cmd':'winch', 'args':{'action':'release'}}})
    asyncio.run(node._on_command(first, raw))
    asyncio.run(node._on_command(second, raw))
    node.pub_winch.publish.assert_called_once()
    assert json.loads(second.send.call_args.args[0])['data']['result'] == 'accepted'
    changed = json.loads(raw); changed['data']['args']['action'] = 'lower'
    asyncio.run(node._on_command(second, json.dumps(changed)))
    assert json.loads(second.send.call_args.args[0])['data']['result'] == 'rejected'
    node.pub_winch.publish.assert_called_once()


def test_concurrent_execution_cannot_replace_current_controller(node):
    assert node._dispatch('start_test', {'part':'search'})[0] == 'accepted'
    assert node._dispatch('start_mission', {})[0] == 'rejected'
    node.pub_start.publish.assert_called_once()
    assert node._dispatch('abort', {})[0] == 'accepted'

def test_all_three_hex_codes_are_required_and_cannot_change_in_flight(node):
    supplied = {'green':'#07A03D','orange':'#EB5A0A','red':'#C4161C'}
    assert node._dispatch('set_banner_colours', {'colours':{k:v for k,v in supplied.items() if k!='red'}})[0] == 'rejected'
    assert node._dispatch('set_banner_colours', {'colours':supplied})[0] == 'accepted'
    node.state['mission']['armed'] = True
    assert node._dispatch('set_banner_colours', {'colours':supplied})[0] == 'rejected'
    node.pub_banner_colours.publish.assert_called_once()


def test_arena_fence_reaches_existing_production_topic_and_refuses_airborne_edits(node):
    node.pub_geofence = MagicMock()
    vertices = [{'lat':12.,'lon':77.}, {'lat':12.001,'lon':77.}, {'lat':12.001,'lon':77.001}, {'lat':12.,'lon':77.001}]
    assert node._dispatch('set_geofence', {'vertices':vertices})[0] == 'accepted'
    assert json.loads(node.pub_geofence.publish.call_args.args[0].data)['vertices'] == vertices
    node.state['mission']['armed'] = True
    assert node._dispatch('set_geofence', {'vertices':vertices})[0] == 'rejected'
    assert node.pub_geofence.publish.call_count == 1


def test_qr_identity_reaches_the_operator(node):
    # An appearance-only flight decodes nothing, so the decoded string alone
    # would read "Unavailable"; the target and match basis say what happened.
    node._on_qr_detail(NS(data=json.dumps({
        'capture_stamp': 99.0, 'processed_stamp': 99.1, 'accepted': '',
        'matched': True, 'reference_captured': True,
        'match_basis': ['appearance'], 'appearance_score': 0.9876})))
    percep = node.state['percep']
    assert percep['qr_reference'] is True
    assert percep['qr_match_basis'] == ['appearance']
    assert percep['qr_appearance_score'] == pytest.approx(0.988)
    json.dumps(node.state, allow_nan=False)


def test_a_refused_dimension_change_keeps_the_applied_arena_ready():
    from gcs_aggregator.readiness_node import Readiness
    fake = NS(obs={})
    applied = {'applied': True, 'reason': '', 'dimensions': {
        'qr': {'target_m': 1.0}, 'corridor': {'length_m': 10.0, 'nav_height_m': 2.4384}}}
    Readiness._on_arena_dimensions(fake, NS(data=json.dumps(applied)))
    assert fake.obs['arena_dimensions_applied'] is True
    assert fake.obs['arena_dimensions_value'] == 'QR 1.00 m, corridor 10.0 m at 2.44 m'
    refused = {'applied': False, 'reason': 'Corridor length 99 is outside 2-30'}
    Readiness._on_arena_dimensions(fake, NS(data=json.dumps(refused)))
    assert fake.obs['arena_dimensions_applied'] is True
    assert 'last change refused' in fake.obs['arena_dimensions_value']
    # Nothing ever applied: a refusal blocks.
    fresh = NS(obs={})
    Readiness._on_arena_dimensions(fresh, NS(data=json.dumps(refused)))
    assert fresh.obs['arena_dimensions_applied'] is False
    assert 'outside' in fresh.obs['arena_dimensions_reason']


def test_simulated_fcu_freshness_uses_the_vehicle_clock():
    # SITL at a quarter real time: its 1 Hz heartbeat is 4 s apart on the
    # wall. The simulated vehicle's freshness is judged in its own time; a
    # real FCU stays on the wall clock.
    import time as wall
    from aerothon_runtime.execution_node import ExecutionNode
    now = {'t': 100.0}
    clock = NS(now=lambda: NS(nanoseconds=int(now['t'] * 1e9)))
    sim = NS(remote=True, fcu_seen=wall.monotonic() - 4.0, fcu_seen_vehicle=99.0,
             get_clock=lambda: clock)
    assert ExecutionNode.fcu_age(sim) == pytest.approx(1.0)
    now['t'] = 104.5
    assert ExecutionNode.fcu_age(sim) == pytest.approx(5.5)
    real = NS(remote=False, fcu_seen=wall.monotonic() - 4.0, fcu_seen_vehicle=99.0,
              get_clock=lambda: clock)
    assert ExecutionNode.fcu_age(real) >= 4.0
    assert ExecutionNode.fcu_age(NS(remote=True, fcu_seen=None)) is None


def test_recovery_is_repeated_until_the_fcu_leaves_guided(monkeypatch):
    import aerothon_runtime.execution_node as ex
    sent = []
    monkeypatch.setattr(ex.production, 'recover', lambda mav: sent.append(mav.state.mode) or 'RTL')
    state = NS(mode='GUIDED', armed=True)
    node = NS(run={'state': 'Running', 'task': 'mission'},
              recovery={'reason': 'FCU telemetry lost', 'failed': True, 'sent': False},
              mav=NS(state=state, reset_pending=lambda: False, connected=lambda: True),
              normal_task=lambda: True, publish=lambda: None)
    clock = {'t': 1000.0}
    monkeypatch.setattr(ex.time, 'monotonic', lambda: clock['t'])
    ex.ExecutionNode.tick(node)
    assert sent == ['GUIDED']
    clock['t'] += 0.5
    ex.ExecutionNode.tick(node)
    assert len(sent) == 1, 'at most once a second'
    clock['t'] += 1.0
    ex.ExecutionNode.tick(node)
    assert len(sent) == 2, 'a lost request is repeated while still GUIDED'
    state.mode = 'LOITER'                       # the pilot took over
    clock['t'] += 5.0
    ex.ExecutionNode.tick(node)
    assert len(sent) == 2, 'never fight the pilot or the FCU recovery'
    state.mode, state.armed = 'RTL', False
    ex.ExecutionNode.tick(node)
    assert node.run['state'] == 'Failed' and node.run['disarmed'] is True
