#!/usr/bin/env python3
"""Run the requested navigation matrix after rendered targeted acceptance passes."""
import argparse
import hashlib
import json
import signal
import sys
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
from queue import Queue

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import world_spec as W
from check_corridor_clearance import audit
from fly_headless import fly,stop_all
from campaign_record import source_manifest


REQUIRED_TARGETS = frozenset((
    ('my_world.json', 20261003), ('my_world.json', 1001),
    ('shipped.json', 20261003), ('rb_rotated.json', 20261003),
    ('split_corridors.json', 20261003),
))
REQUIRED_DECISION_NODES = frozenset((
    'mission_bt', 'winch_ctrl', 'velocity_controller',
    'velocity_controller_capture_driver', 'velocity_controller_clock_driver',
))
REQUIRED_ACCEPTANCE_CHECKS = frozenset((
    'no red-zone entry', 'inside geofence',
    'outbound lane flown THROUGH the corridor', 'return lane flown THROUGH the corridor',
    'never above the corridor', 'no tilt beyond 30 deg',
    'no failsafe / unexpected flight mode', 'ceiling over zone',
    'flight within 900 s', 'no obstacle contact', 'drop within 1 m of pad',
    'landed at take-off point', 'payload released (on the ground, off the aircraft)',
    'payload on the target pad (within 1 m)', 'independent aircraft truth recorded',
    'restricted-zone clearance at least 1.50 m', 'corridor clearance at least 0.70 m',
    'distinct detector rates recorded', 'motor delivery confirmed with fresh recorded FCU outputs',
    'initial and navigation-time decision input audits passed',
    'complete camera timeline verified through disarm',
    'runtime configuration and source hashes preserved',
))


def reduced_weather_exclusion(conditions,seed):
    concrete=W.conditions({'conditions':{'preset':conditions}},seed)
    wind=concrete['wind']
    # Use the existing moderate field preset as this reduced scope's ceiling.
    # Random cases are retained only when their concrete wind stays inside it.
    if wind['speed']>4. or wind['gust']>2.:
        return dict(reason='strong wind/gust outside reduced field envelope',
                    mean_wind_mps=wind['speed'],gust_mps=wind['gust'],conditions=concrete)
    return None


def verify_targeted_runs(paths, root=ROOT):
    """All five full rendered acceptances must qualify the current frozen code."""
    if len(paths) != len(REQUIRED_TARGETS):
        raise ValueError('Five targeted full acceptances are required before the campaign')
    summaries = [json.loads((p/'summary.json').read_text()) for p in paths]
    if not all(s.get('acceptance_pass') is True and s.get('motor_delivery_qualified') is True
               and s.get('final_armed') is False and
               s.get('mission_result', {}).get('state') == 'COMPLETED' for s in summaries):
        raise ValueError('All targeted full acceptances must pass, complete and disarm')
    pairs = [(Path(s.get('configuration', {}).get('world', '')).name,
              s.get('configuration', {}).get('seed')) for s in summaries]
    if set(pairs) != REQUIRED_TARGETS or len(set(pairs)) != len(pairs):
        raise ValueError('Targeted courses and seeds must cover both standard seeds and shipped/rotated/split')
    if not all(s.get('configuration', {}).get('conditions') == 'calm' for s in summaries):
        raise ValueError('Targeted runs must use the required calm configuration')
    for path in paths:
        grade = json.loads((path/'grade.json').read_text())
        checks = grade.get('checks', {})
        if not (grade.get('pass') is True and REQUIRED_ACCEPTANCE_CHECKS <= checks.keys()
                and all(value is True for value in checks.values())):
            raise ValueError('Every required rendered acceptance check must pass: '+str(path))
        contacts=grade.get('aircraft_contact_evidence',{})
        if not (contacts.get('valid') is True and contacts.get('contact_free') is True
                and contacts.get('ground_canary') is True):
            raise ValueError('Independent native aircraft-contact evidence must qualify: '+str(path))
        for name in ('decision_inputs.json', 'decision_inputs_navigation.json'):
            audit = json.loads((path/name).read_text())
            if not (audit.get('pass') is True and REQUIRED_DECISION_NODES <= audit.get('nodes', {}).keys()
                    and not audit.get('missing_nodes') and not audit.get('forbidden_subscriptions')):
                raise ValueError('Decision input audit must include capture and clock helpers: '+str(path/name))
    manifests = [json.loads((p/'source_manifest.json').read_text()) for p in paths]
    # World/calibration absolute entries differ by course. The actual source
    # includes the mission code, launch/perception setup and fixture/grader code.
    controlled = [{name: sha for name, sha in manifest.items()
                   if name.startswith(('src/', 'scripts/', 'sim/'))} for manifest in manifests]
    core = controlled[0]
    if not all(any(name.startswith(prefix) for name in core)
               for prefix in ('src/', 'scripts/', 'sim/')):
        raise ValueError('No complete recorded drone and qualification source')
    if any(recorded != core for recorded in controlled[1:]):
        raise ValueError('Targeted runs have different recorded source')
    source_manifest.cache_clear()
    current = source_manifest(root)
    if current != core:
        raise ValueError('Drone or qualification source changed since targeted qualification; added, removed or changed files')
    for name, expected in core.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Drone or qualification source changed since targeted qualification: '+name)
    return dict(qualified_seeds=[20261003, 1001],
                qualified_courses_and_seeds=[dict(world=world, seed=seed) for world, seed in sorted(REQUIRED_TARGETS)],
                drone_source_files_verified=sum(name.startswith('src/') for name in core),
                all_source_files_verified=len(core), required_rendered_checks=len(REQUIRED_ACCEPTANCE_CHECKS))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--targeted',nargs=5,type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('-j',type=int,default=3)
    p.add_argument('--rtf',type=float,default=4.)
    p.add_argument('--reduced-no-strong-or-delay',action='store_true',
                   help='Exclude wind stronger than field preset; inject no command delay')
    args=p.parse_args()
    if not 1 <= args.j <= 16:
        p.error('Use 1 to 16 workers; each receives a distinct ROS domain')
    try:
        gate = verify_targeted_runs(args.targeted)
    except (ValueError, OSError) as e:
        raise SystemExit(str(e)) from e
    args.out.mkdir(parents=True,exist_ok=False)
    (args.out/'targeted_gate.json').write_text(json.dumps(gate, indent=2))
    signal.signal(signal.SIGINT,stop_all)
    signal.signal(signal.SIGTERM,stop_all)
    manifest=source_manifest(ROOT)
    (args.out/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
    classified=[]
    work=[]
    excluded=[]
    selected=0
    for path in sorted((ROOT/'sim/worlds').glob('*.json')):
        try:errors,warnings=W.validate(W.load(path))
        except (ValueError,TypeError,KeyError) as e:errors,warnings=[repr(e)],[]
        item=dict(world=path.name,validation_errors=errors,warnings=warnings)
        item['geometry']=[] if errors else [audit(path,lane_key=lane) for lane in ('corridor','return_corridor')]
        blocked=any(g['classification']=='geometrically blocked' for g in item['geometry'])
        item['classification']='invalid fixture' if errors else 'infeasible at 0.70 m' if blocked else 'navigation trial required'
        classified.append(item)
        for conditions in ('calm','field','worst','random'):
            for seed in range(1,6):
                exclusion=reduced_weather_exclusion(conditions,seed) if args.reduced_no_strong_or_delay else None
                if exclusion:
                    excluded.append(dict(world=path.name,preset=conditions,seed=seed,**exclusion))
                    continue
                selected+=1
                if errors or blocked:
                    print(f'{path.stem}_{conditions}_s{seed}: CLASSIFIED {item["classification"]}',flush=True)
                else:work.append((path,conditions,seed))
    (args.out/'classification.json').write_text(json.dumps(classified,indent=2))
    (args.out/'exclusions.json').write_text(json.dumps(excluded,indent=2))
    results=[]
    domains=Queue()
    # Keep DDS ports below the host's ephemeral range even at 16 workers.
    for domain in range(40,40+args.j):
        domains.put(domain)
    def run(index,trial):
        path,conditions,seed=trial
        domain=domains.get()
        try:return fly(path,conditions,seed,args.out,args.rtf,1200.,domain,navigation_only=True,
                       injected_command_delay_s=0. if args.reduced_no_strong_or_delay else .4)
        except Exception as e:
            return dict(run=f'{path.stem}_{conditions}_s{seed}',navigation_pass=False,
                        result='RUNNER_ERROR',error=repr(e))
        finally:
            domains.put(domain)
    with ThreadPoolExecutor(max_workers=args.j) as pool:
        futures=[pool.submit(run,i,trial) for i,trial in enumerate(work)]
        for future in as_completed(futures):
            result=future.result();results.append(result)
            (args.out/'summary.json').write_text(json.dumps(results,indent=2))
            print(json.dumps(result),flush=True)
    # Compare after clearing the cache so the recorded source freeze is checked.
    source_manifest.cache_clear()
    unchanged=manifest==source_manifest(ROOT)
    report=dict(original_matrix_trials=len(classified)*20,requested_trials=selected,executed_trials=len(results),
        excluded_trials=len(excluded),classified_without_flight=selected-len(results),source_unchanged=unchanged,
        reduced_scope=args.reduced_no_strong_or_delay,original_200_case_qualified=False,
        injected_command_delay_s=0. if args.reduced_no_strong_or_delay else .4,
        deferred_failures=['strong gusts','artificially delayed control'] if args.reduced_no_strong_or_delay else [],
        navigation_passes=sum(bool(r.get('navigation_pass')) for r in results),
        failures=[r for r in results if not r.get('navigation_pass')],
        rendered_perception_qualified=False,motor_delivery_qualified=False)
    (args.out/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return 0 if unchanged and not report['failures'] else 1


if __name__=='__main__':raise SystemExit(main())
