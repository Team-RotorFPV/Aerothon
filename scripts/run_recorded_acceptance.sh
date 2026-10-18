#!/usr/bin/env bash
# One owned rendered stack, onboard recording through disarm, separate truth.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
spec="$(realpath "${1:?WORLD.json}")"
seed="${2:?seed}"
out="${3:?new output directory}"
profile="$(realpath "${4:?frozen calibration profile}")"
trial="${AEROTHON_RENDER_TRIAL:-mission}"
entry="${AEROTHON_CORRIDOR_ENTRY:-}"
handover="${AEROTHON_CORRIDOR_HANDOVER:-stationary}"
case "$handover" in stationary|moving) ;; *) echo 'Unknown return handover mode'; exit 2 ;; esac
case "$trial" in
 mission) ;;
 return-corridor) entry="$(realpath "${entry:?recorded FCU entry fixture required}")" ;;
 delivery) ;;
 *) echo 'Unknown rendered trial'; exit 2 ;;
esac
[[ ! -e "$out" ]] || { echo 'Output exists'; exit 2; }
if pgrep -f '[g]z sim|[a]rducopter|[l]aunch_level6_sim' >/dev/null; then
    echo 'Another simulator is active'; exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/aerothon_stack/install/setup.bash"
source install/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH" PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${AEROTHON_ACCEPTANCE_DOMAIN:-218}" ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="${GZ_PARTITION:-aerothon_m2}" GZ_IP="${GZ_IP:-127.0.0.1}"
mkdir -p "$out"
out="$(realpath "$out")"
python3 - "$spec" "$out" "$profile" "$seed" <<'PY'
import hashlib,json,sys,os
from pathlib import Path
sys.path.insert(0,'scripts')
import world_spec as W
spec,out,profile=map(Path,sys.argv[1:4])
errors,warnings=W.validate(W.load(spec))
if errors: raise SystemExit('; '.join(errors))
for warning in warnings: print(warning)
sys.path.insert(0,'sim')
from campaign_record import source_manifest
manifest=source_manifest(Path.cwd())
manifest.update({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (spec,profile)})
(out/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
tools=[Path('.scratch/simulation-reliability')/name for name in (
    'capture_runtime_configuration.py','audit_decision_inputs.py',
    'grade_focused_return.py','grade_focused_delivery.py')]
(out/'qualification_tools_manifest.json').write_text(json.dumps(
    {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tools},indent=2))
(out/'world_spec.json').write_bytes(spec.read_bytes())
(out/'frozen_calibration.json').write_bytes(profile.read_bytes())
(out/'configuration.json').write_text(json.dumps(dict(seed=int(sys.argv[4]),conditions=os.environ.get('AEROTHON_RENDER_CONDITIONS','calm'),
    airframe='cad',payload_fixture='loaded',world=str(spec),ros_domain=int(os.environ['ROS_DOMAIN_ID'])),indent=2))
PY
# Calm unless asked: AEROTHON_RENDER_CONDITIONS=field flies the moderate
# field envelope (wind, sensor degradation) rendered, as headless does.
render_conditions="${AEROTHON_RENDER_CONDITIONS:-calm}"
case "$render_conditions" in calm|field) ;; *) echo 'Unknown rendered conditions'; exit 2 ;; esac
export AEROTHON_RENDER_CONDITIONS="$render_conditions"
export AEROTHON_WORLD_SPEC="$spec" AEROTHON_CONDITIONS="$render_conditions" AEROTHON_CONDITIONS_SEED="$seed"
export AEROTHON_SEED="$seed"
export AEROTHON_HEADLESS=1 AEROTHON_OPEN_GCS=0 AEROTHON_NO_REAP=1
export AEROTHON_AIRFRAME=cad AEROTHON_PAYLOAD_FIXTURE=loaded AEROTHON_DELIVERY_CALIBRATION="$profile"
if [[ "$trial" == return-corridor ]]; then
 export AEROTHON_PAYLOAD_FIXTURE=unloaded
 cp "$entry" "$out/entry_fixture.json"
 python3 - "$out" "$handover" <<'PY'
import hashlib,json,sys
from pathlib import Path
out=Path(sys.argv[1]);p=out/'configuration.json';c=json.loads(p.read_text())
c.update(payload_fixture='unloaded',trial='focused return corridor; full mission not exercised',
         entry_fixture_sha256=hashlib.sha256((out/'entry_fixture.json').read_bytes()).hexdigest(),
         handover_mode=sys.argv[2],handover_max_age_s=.7 if sys.argv[2]=='moving' else None,
         handover_driver_sha256=hashlib.sha256(Path('sim/corridor_trial.py').read_bytes()).hexdigest())
p.write_text(json.dumps(c,indent=2)+'\n')
PY
fi
if [[ "$trial" == delivery ]]; then
 python3 - "$out" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1])/'configuration.json';c=json.loads(p.read_text())
c.update(trial='focused rendered hover/drop; full mission not exercised',drop_alt_m=5.5)
p.write_text(json.dumps(c,indent=2)+'\n')
PY
fi
# The world is built from the spec either way. The MISSION is told the QR
# size at launch only when no GCS preset is given; with one, it starts on the
# launch default and learns the arena from the operator, as on the day.
if [[ -z "${AEROTHON_GCS_PRESET:-}" ]]; then
 export AEROTHON_TARGET_QR_M="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["qr"]["target_m"])' "$spec")"
else
 unset AEROTHON_TARGET_QR_M
fi
unset AEROTHON_RANDOM_ARENA AEROTHON_START_TARGET AEROTHON_DELIVERY_ZONE AEROTHON_GEOFENCE AEROTHON_FENCE_ONLY
# The judges' points only, local ENU "x,y;x,y;...": the aircraft gets this one
# polygon and no delivery-zone corners, as at the competition. Grading keeps
# the world's own layout.
if [[ -n "${AEROTHON_JUDGES_GEOFENCE_LOCAL:-}" ]]; then
 export AEROTHON_GEOFENCE="$AEROTHON_JUDGES_GEOFENCE_LOCAL" AEROTHON_FENCE_ONLY=1
 printf '%s\n' "$AEROTHON_GEOFENCE" > "$out/judges_geofence_local.txt"
fi
stack_pid='' recorder_pid='' truth_pid='' contact_pid='' vehicle_contact_pid='' audit_pid='' overview_pid=''
cleanup() {
 [[ -z "$overview_pid" ]] || kill -INT "$overview_pid" 2>/dev/null || true
 [[ -z "$audit_pid" ]] || kill -TERM "$audit_pid" 2>/dev/null || true
 [[ -z "$recorder_pid" ]] || kill -INT "$recorder_pid" 2>/dev/null || true
 [[ -z "$truth_pid" ]] || kill -INT "$truth_pid" 2>/dev/null || true
 [[ -z "$contact_pid" ]] || kill -INT "$contact_pid" 2>/dev/null || true
 [[ -z "$vehicle_contact_pid" ]] || kill -INT "$vehicle_contact_pid" 2>/dev/null || true
 [[ -z "$stack_pid" ]] || kill -TERM -- "-$stack_pid" 2>/dev/null || true
 [[ -z "$stack_pid" ]] || wait "$stack_pid" 2>/dev/null || true
 [[ -z "$stack_pid" ]] || kill -KILL -- "-$stack_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
setsid bash scripts/launch_level6_sim.sh > "$out/stack.log" 2>&1 & stack_pid=$!
python3 sim/record_delivery.py --out "$out" --timeout 7200 > "$out/recorder.log" 2>&1 & recorder_pid=$!
python3 sim/delivery_truth.py --out "$out/evaluation_truth.jsonl" --vehicle-out "$out/evaluation_vehicle_truth.jsonl" > "$out/evaluation.log" 2>&1 & truth_pid=$!
python3 sim/record_contact_truth.py --out "$out/evaluation_contacts.jsonl" --until-file "$out/recorded_result.json" > "$out/contact_recorder.log" 2>&1 & contact_pid=$!
python3 sim/wait_ready.py --timeout 900 --require-scan > "$out/readiness.log" 2>&1
cp /tmp/aerothon_mission2_runtime.sdf "$out/flight_world.sdf"
cp /tmp/aerothon_vehicle_models/aerothon_iris_c1_webcam/model.sdf "$out/flight_vehicle.sdf"
python3 sim/record_vehicle_contacts.py --model "$out/flight_vehicle.sdf" --out "$out/evaluation_vehicle_contacts.jsonl" --ready "$out/vehicle_contact_ready.json" --until-file "$out/recorded_result.json" > "$out/vehicle_contact_recorder.log" 2>&1 & vehicle_contact_pid=$!
cp /tmp/aerothon_arena_layout.json "$out/layout.json"
if [[ "${AEROTHON_RECORD_OVERVIEW:-0}" == 1 ]]; then
 python3 sim/record_overview.py --mission "$out" > "$out/overview.log" 2>&1 & overview_pid=$!
fi
cp /tmp/aerothon_conditions.json "$out/flight_conditions.json"
cp /tmp/aerothon_conditions.parm "$out/flight_conditions.parm"
cp -r /tmp/aerothon_vehicle_models/aerothon_iris_c1_webcam/meshes "$out/vehicle_meshes"
cp src/aerothon_sim/sim_gazebo/models/aerothon_quad/airframe.json "$out/airframe.json"
python3 .scratch/simulation-reliability/capture_runtime_configuration.py "$out" > "$out/runtime_capture.log" 2>&1
python3 - "$out" <<'PY'
import json,re,sys,time
from pathlib import Path
out=Path(sys.argv[1])
required=['camera_capture_events.jsonl','evaluation_truth.jsonl','evaluation_vehicle_truth.jsonl','vehicle_contact_ready.json','evaluation_vehicle_contacts.jsonl']
import os
if os.environ.get('AEROTHON_RECORD_OVERVIEW') == '1':
    required.append('overview/ready.json')
until=time.monotonic()+60
while time.monotonic()<until:
    if all((out/name).exists() and (out/name).stat().st_size>0 for name in required):break
    time.sleep(.5)
else:raise SystemExit('Independent recording channels were not ready; mission was not started')
print('Camera, payload truth and independent aircraft truth recording ready')
match=re.search(r'start pad names delivery target ([ABCDE])', (out/'stack.log').read_text())
if match is None:raise SystemExit('Generated start-pad target missing from evaluation record')
(out/'evaluation_target.json').write_text(json.dumps(dict(target=match[1].lower(),source='generated start-pad visual; evaluation only'))+'\n')
PY
# THE OPERATOR'S PATH. With AEROTHON_GCS_PRESET set, the arena's dimensions
# reach the aircraft as at the competition: a preset saved at the GCS (from
# AEROTHON_GCS_PRESET_FILE the first time) and applied over the aggregator
# before START. No confirmation from the aircraft, no mission.
if [[ -n "${AEROTHON_GCS_PRESET:-}" ]]; then
 if ! python3 scripts/gcs_operator.py --preset "$AEROTHON_GCS_PRESET" \
      ${AEROTHON_GCS_PRESET_FILE:+--create-from-file "$AEROTHON_GCS_PRESET_FILE"} \
      --record "$out/gcs_dimensions.json" > "$out/gcs_operator.log" 2>&1; then
  echo "GCS dimensions were not applied; mission not started" | tee "$out/gcs_refused.txt"
  exit 3
 fi
fi
mission_rc=0
# Record both the initial graph and mounting subscriptions activated during
# actual navigation. An early snapshot alone misses lazy subscriptions.
python3 .scratch/simulation-reliability/audit_decision_inputs.py "$out/decision_inputs.json" > "$out/decision_input_audit.log" 2>&1
if [[ "$trial" != delivery ]]; then
 audit_nodes=(mission_bt winch_ctrl velocity_controller velocity_controller_capture_driver velocity_controller_clock_driver)
 [[ "$trial" != return-corridor ]] || audit_nodes+=(rendered_corridor_trial)
 python3 .scratch/simulation-reliability/audit_decision_inputs.py "$out/decision_inputs_navigation.json" --wait-navigation "${audit_nodes[@]}" > "$out/navigation_input_audit.log" 2>&1 & audit_pid=$!
else
 python3 .scratch/simulation-reliability/audit_decision_inputs.py "$out/decision_inputs_delivery.json" --wait-delivery mission_bt winch_ctrl velocity_controller velocity_controller_capture_driver velocity_controller_clock_driver delivery_demo > "$out/delivery_input_audit.log" 2>&1 & audit_pid=$!
fi
if [[ "$trial" == return-corridor ]]; then
 python3 -u sim/corridor_trial.py --entry "$out/entry_fixture.json" --out "$out" --handover-mode "$handover" > "$out/mission.log" 2>&1 || mission_rc=$?
elif [[ "$trial" == delivery ]]; then
 python3 -u sim/delivery_demo.py --mode delivery --alt 5.5 --profile "$profile" --no-video --wall-timeout 1800 --out "$out/driver" > "$out/mission.log" 2>&1 || mission_rc=$?
else
 python3 -u sim/run_mission_live.py --watch 5400 --settle 10 --track "$out/track.csv" > "$out/mission.log" 2>&1 || mission_rc=$?
fi
printf '%s\n' "$mission_rc" > "$out/runner_exit_code.txt"
if [[ -n "$audit_pid" ]]; then
 kill -TERM "$audit_pid" 2>/dev/null || true
 wait "$audit_pid" || true
 audit_pid=''
fi
for _ in $(seq 1 900); do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 1; done
kill -INT "$recorder_pid" 2>/dev/null || true
wait "$recorder_pid" || true
kill -INT "$truth_pid" 2>/dev/null || true
wait "$truth_pid" || true
kill -INT "$contact_pid" 2>/dev/null || true
wait "$contact_pid" || true
kill -INT "$vehicle_contact_pid" 2>/dev/null || true
wait "$vehicle_contact_pid" || true
vehicle_contact_pid=''
if [[ -n "$overview_pid" ]]; then
 for _ in $(seq 1 20); do kill -0 "$overview_pid" 2>/dev/null || break; sleep 1; done
 kill -INT "$overview_pid" 2>/dev/null || true
 wait "$overview_pid" || true
 overview_pid=''
fi
# All evidence has closed. Stop only this run's process group before exports.
kill -TERM -- "-$stack_pid" 2>/dev/null || true
wait "$stack_pid" || true
kill -KILL -- "-$stack_pid" 2>/dev/null || true
stack_pid=''
ffmpeg -v error -y -i "$out/onboard.avi" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$out/onboard.mp4"
ffmpeg -v error -i "$out/onboard.mp4" -f null - 2> "$out/video_decode.log"
evaluation_rc=0
python3 sim/export_recording_timeline.py "$out" > "$out/video_export.log" 2>&1
python3 sim/evaluate_recorded_mission.py "$out" || evaluation_rc=$?
exit "$evaluation_rc"
