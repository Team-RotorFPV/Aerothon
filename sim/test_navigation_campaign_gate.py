"""Historical, partial or differently configured passes cannot unlock the matrix."""
import hashlib
import json
import pytest
from run_navigation_campaign import (verify_targeted_runs, REQUIRED_TARGETS,
                                      REQUIRED_ACCEPTANCE_CHECKS, REQUIRED_DECISION_NODES)


@pytest.fixture
def qualified(tmp_path):
    files = {'src/controller.py': 'qualified drone code\n',
             'scripts/launch.sh': '# qualified launch\n', 'sim/fixture.py': '# qualified fixture\n'}
    files.update({'sim/worlds/'+world: json.dumps({'course': world}) for world, _ in REQUIRED_TARGETS})
    manifest = {}
    for name, body in files.items():
        path = tmp_path/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(body)
        manifest[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    paths = []
    for world, seed in sorted(REQUIRED_TARGETS):
        out = tmp_path/(world+'_'+str(seed)); out.mkdir()
        (out/'summary.json').write_text(json.dumps(dict(acceptance_pass=True,
            motor_delivery_qualified=True, final_armed=False, mission_result={'state': 'COMPLETED'},
            configuration=dict(seed=seed, conditions='calm', world='sim/worlds/'+world))))
        # Absolute generated assets differ by course; the controlled code does not.
        (out/'source_manifest.json').write_text(json.dumps(dict(manifest, **{'/recorded/'+world: 'asset hash'})))
        (out/'grade.json').write_text(json.dumps({'pass': True, 'checks': dict.fromkeys(REQUIRED_ACCEPTANCE_CHECKS, True),
            'aircraft_contact_evidence':dict(valid=True,contact_free=True,ground_canary=True)}))
        audit = dict(pass_=True, nodes={name: [['/clock', ['rosgraph_msgs/msg/Clock']]]
                                       for name in REQUIRED_DECISION_NODES}, missing_nodes=[], forbidden_subscriptions=[])
        audit['pass'] = audit.pop('pass_')
        for name in ('decision_inputs.json', 'decision_inputs_navigation.json'):
            (out/name).write_text(json.dumps(audit))
        paths.append(out)
    return tmp_path, paths, tmp_path/'src/controller.py'


def test_current_qualified_source_can_start_campaign(qualified):
    root, paths, _ = qualified
    result = verify_targeted_runs(paths, root)
    assert result['drone_source_files_verified'] == 1
    assert len(result['qualified_courses_and_seeds']) == 5
    assert result['required_rendered_checks'] == 22


def test_changed_drone_is_rejected_even_with_passing_summaries(qualified):
    root, paths, source = qualified
    source.write_text('changed drone code\n')
    with pytest.raises(ValueError, match='changed since'):
        verify_targeted_runs(paths, root)


def test_legacy_geometric_contact_pass_cannot_unlock_campaign(qualified):
    root,paths,_=qualified
    grade=json.loads((paths[0]/'grade.json').read_text())
    del grade['aircraft_contact_evidence']
    (paths[0]/'grade.json').write_text(json.dumps(grade))
    with pytest.raises(ValueError,match='aircraft-contact'):
        verify_targeted_runs(paths,root)


def test_duplicate_seed_cannot_satisfy_required_trials(qualified):
    root, paths, _ = qualified
    with pytest.raises(ValueError, match='seeds'):
        verify_targeted_runs([paths[0], paths[0], *paths[2:]], root)


def test_navigation_only_grade_cannot_qualify_delivery(qualified):
    root, paths, _ = qualified
    summary = json.loads((paths[0]/'summary.json').read_text())
    summary['motor_delivery_qualified'] = False
    (paths[0]/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='full acceptances'):
        verify_targeted_runs(paths, root)


def test_two_standard_passes_cannot_skip_required_courses(qualified):
    root, paths, _ = qualified
    standard = [p for p in paths if p.name.startswith('my_world.json')]
    with pytest.raises(ValueError, match='Five'):
        verify_targeted_runs(standard, root)


def test_failed_clearance_cannot_be_hidden_by_passing_summary(qualified):
    root, paths, _ = qualified
    grade = json.loads((paths[0]/'grade.json').read_text())
    grade['checks']['corridor clearance at least 0.70 m'] = False
    (paths[0]/'grade.json').write_text(json.dumps(grade))
    with pytest.raises(ValueError, match='Every required'):
        verify_targeted_runs(paths, root)


def test_incomplete_grade_is_rejected(qualified):
    root, paths, _ = qualified
    grade = json.loads((paths[0]/'grade.json').read_text())
    grade['checks'].pop('complete camera timeline verified through disarm')
    (paths[0]/'grade.json').write_text(json.dumps(grade))
    with pytest.raises(ValueError, match='Every required'):
        verify_targeted_runs(paths, root)


def test_missing_capture_helper_audit_is_rejected(qualified):
    root, paths, _ = qualified
    audit = json.loads((paths[0]/'decision_inputs_navigation.json').read_text())
    audit['nodes'].pop('velocity_controller_capture_driver')
    (paths[0]/'decision_inputs_navigation.json').write_text(json.dumps(audit))
    with pytest.raises(ValueError, match='include capture and clock'):
        verify_targeted_runs(paths, root)


def test_changed_launch_code_cannot_reuse_prior_passes(qualified):
    root, paths, _ = qualified
    (root/'scripts/launch.sh').write_text('# changed launch\n')
    with pytest.raises(ValueError, match='changed since'):
        verify_targeted_runs(paths, root)


def test_different_recorded_fixture_source_is_rejected(qualified):
    root, paths, _ = qualified
    manifest = json.loads((paths[0]/'source_manifest.json').read_text())
    manifest['sim/fixture.py'] = 'other source version'
    (paths[0]/'source_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='different recorded source'):
        verify_targeted_runs(paths, root)


def test_wrong_rotated_seed_does_not_cover_required_course(qualified):
    root, paths, _ = qualified
    path = next(p for p in paths if p.name.startswith('rb_rotated.json'))
    summary = json.loads((path/'summary.json').read_text())
    summary['configuration']['seed'] = 1001
    (path/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='courses and seeds'):
        verify_targeted_runs(paths, root)


def test_weather_configuration_must_match_targeted_acceptance(qualified):
    root, paths, _ = qualified
    summary = json.loads((paths[0]/'summary.json').read_text())
    summary['configuration']['conditions'] = 'field'
    (paths[0]/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='calm configuration'):
        verify_targeted_runs(paths, root)


def test_added_module_cannot_reuse_prior_passes(qualified):
    root, paths, _ = qualified
    (root/'src/new_module.py').write_text('# new decision dependency\n')
    with pytest.raises(ValueError, match='changed since'):
        verify_targeted_runs(paths, root)


def test_changed_qr_payload_asset_cannot_reuse_prior_passes(qualified):
    root, paths, _ = qualified
    # An asset added after the recorded freeze is also a source-set change.
    (root/'src/qr_payloads.txt').write_text('different shipped QR payload\n')
    with pytest.raises(ValueError, match='changed since'):
        verify_targeted_runs(paths, root)
