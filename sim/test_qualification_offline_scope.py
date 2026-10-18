"""Real pytest collection proves the scope does not omit ordinary or negative checks."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def collect(tmp_path, reduced):
    evidence = tmp_path / ('reduced.json' if reduced else 'full.json')
    args = [sys.executable, '-m', 'pytest', 'sim/test_corridor_stress.py',
            '--collect-only', '-q', '-p', 'qualification_offline_scope',
            '--qualification-scope-evidence', str(evidence)]
    if reduced:
        args.append('--reduced-qualification')
    env = dict(os.environ, PYTHONPATH=str(ROOT/'sim')+os.pathsep+os.environ.get('PYTHONPATH',''))
    result = subprocess.run(args, cwd=ROOT, env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    return evidence, result.stdout

def test_reduced_scope_records_all_five_strong_gust_flights_and_retains_eligible_checks(tmp_path):
    evidence, _ = collect(tmp_path, True)
    report = json.loads(evidence.read_text())
    assert report['excluded_test_count'] == 5
    assert len({r['nodeid'] for r in report['excluded_tests']}) == 5
    assert all((r['mean_wind_mps'], r['gust_mps']) == (8., 4.) for r in report['excluded_tests'])
    retained = report['included_nodes']
    assert 'sim/test_corridor_stress.py::ShippedLaneStressTests::test_dirty_lidar' in retained
    assert 'sim/test_corridor_stress.py::ShippedLaneStressTests::test_slow_airframe_and_late_scans' in retained
    assert 'sim/test_corridor_stress.py::SplitLaneStressTests::test_without_the_observer_the_wind_wins' in retained
    assert not report['original_full_suite_qualified']

def test_default_collection_excludes_nothing_and_does_not_write_reduced_evidence(tmp_path):
    evidence, output = collect(tmp_path, False)
    assert not evidence.exists()
    assert 'ShippedLaneStressTests::test_gusty_crosswind' in output
    assert 'SplitLaneStressTests::test_gusty_crosswind_with_late_scans' in output
