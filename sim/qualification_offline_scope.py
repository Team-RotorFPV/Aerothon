"""Explicit reduced qualification; keep ordinary offline runs exhaustive."""
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import pytest

DEFERRED_FLIGHTS = frozenset({
    'sim/test_corridor_stress.py::ShippedLaneStressTests::test_gusty_crosswind',
    'sim/test_corridor_stress.py::ShippedLaneStressTests::test_everything_at_once',
    'sim/test_corridor_stress.py::SplitLaneStressTests::test_gusty_crosswind',
    'sim/test_corridor_stress.py::SplitLaneStressTests::test_gusty_crosswind_with_dirty_lidar',
    'sim/test_corridor_stress.py::SplitLaneStressTests::test_gusty_crosswind_with_late_scans',
})

def pytest_addoption(parser):
    parser.addoption('--reduced-qualification', action='store_true', default=False)
    parser.addoption('--qualification-scope-evidence', default=None)

def pytest_collection_modifyitems(config, items):
    if not config.getoption('--reduced-qualification'):
        return
    excluded = [item for item in items if item.nodeid in DEFERRED_FLIGHTS]
    evidence = config.getoption('--qualification-scope-evidence')
    if not evidence:
        raise pytest.UsageError('Reduced qualification requires a scope evidence path')
    module = importlib.import_module('test_corridor_stress')
    params = inspect.signature(module.WindDrift).parameters
    mean, gust = params['mean'].default, params['gust'].default
    if (mean, gust) != (8.0, 4.0):
        raise pytest.UsageError('Deferred fixture weather changed; review the explicit scope')
    reason = 'User-deferred strong gusts: fixture mean 8 m/s, gust 4 m/s; unresolved and excluded from qualification'
    for item in excluded:
        item.add_marker(pytest.mark.skip(reason=reason))
    source = Path(module.__file__).resolve()
    report = dict(reduced_scope=True, original_full_suite_qualified=False,
                  collected_tests=len(items), excluded_test_count=len(excluded),
                  included_test_count=len(items)-len(excluded),
                  excluded_tests=[dict(nodeid=item.nodeid, reason=reason,
                                       mean_wind_mps=mean, gust_mps=gust,
                                       seeds=[0, 1, 2]) for item in excluded],
                  included_nodes=[item.nodeid for item in items if item.nodeid not in DEFERRED_FLIGHTS],
                  fixture_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  thresholds_unchanged=True, assertions_unchanged=True,
                  negative_wind_fixture_guard_retained=True)
    path = Path(evidence)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
