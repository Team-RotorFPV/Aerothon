#!/usr/bin/env bash
# Own one isolated stack lifetime; refuse to interrupt another simulation.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mode="${1:?loaded, unloaded or delivery}"
case "$mode" in loaded|unloaded|delivery) ;; *) exit 2 ;; esac
out="${2:?new output directory}"
profile="${3:-}"
if [[ -e "$out" ]]; then echo "Output already exists: $out" >&2; exit 2; fi
if pgrep -f '[g]z sim|[a]rducopter|[l]aunch_level6_sim' >/dev/null; then
    echo 'A simulator is active; stop it before starting an isolated trial.' >&2; exit 2
fi
mkdir -p "$out"
out="$(realpath "$out")"
python3 - "$out" <<'PY'
import hashlib, json, sys
from pathlib import Path
paths = [Path('sim/delivery_demo.py'), Path('scripts/build_cad_vehicle.py'),
         Path('src/aerothon_mission/mission_bt/mission_bt/delivery_observer.py'),
         Path('src/aerothon_mission/mission_bt/mission_bt/mission_tree.py'),
         Path('src/aerothon_payload/winch_ctrl/winch_ctrl/winch_node.py'),
         Path('src/aerothon_sim/sim_gazebo/sim_gazebo/contact_release.py')]
Path(sys.argv[1], 'source_manifest.json').write_text(json.dumps(
    {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2)+'\n')
PY
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/aerothon_stack/install/setup.bash"
source install/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH"
export AEROTHON_HEADLESS=1 AEROTHON_OPEN_GCS=0 AEROTHON_AIRFRAME=cad
export AEROTHON_NO_REAP=1
export AEROTHON_PAYLOAD_FIXTURE=loaded
[[ "$mode" != unloaded ]] || export AEROTHON_PAYLOAD_FIXTURE=unloaded
[[ -z "$profile" ]] || export AEROTHON_DELIVERY_CALIBRATION="$(realpath "$profile")"
stack_pid='' truth_pid=''
cleanup() {
    [[ -z "$truth_pid" ]] || kill -INT "$truth_pid" 2>/dev/null || true
    [[ -z "$stack_pid" ]] || kill -TERM -- "-$stack_pid" 2>/dev/null || true
    [[ -z "$stack_pid" ]] || wait "$stack_pid" 2>/dev/null || true
    # SITL can ignore termination after its launcher exits. This group belongs
    # only to the stack created above; reap any surviving children as well.
    [[ -z "$stack_pid" ]] || kill -KILL -- "-$stack_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
setsid bash scripts/launch_level6_sim.sh > "$out/stack.log" 2>&1 &
stack_pid=$!
python3 sim/delivery_truth.py --out "$out/evaluation_truth.jsonl" > "$out/evaluation.log" 2>&1 &
truth_pid=$!
extra=()
[[ -z "$profile" ]] || extra=(--profile "$(realpath "$profile")")
trial_status=0
python3 sim/delivery_demo.py --mode "$mode" --out "$out" "${extra[@]}" \
    > "$out/demo.log" 2>&1 || trial_status=$?
if [[ -s "$out/onboard.avi" ]]; then
    ffmpeg -v error -y -i "$out/onboard.avi" -c:v libx264 -pix_fmt yuv420p \
        -movflags +faststart "$out/onboard.mp4"
fi
exit "$trial_status"
