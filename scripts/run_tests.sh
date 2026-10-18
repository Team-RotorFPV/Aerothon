#!/usr/bin/env bash
# ==============================================================================
# AeroTHON 2026 — full offline test suite
# ==============================================================================
# Runs everything that does NOT need a live simulation stack: mission/GCS unit
# tests, the Phase 0 fail-closed rail tests, Python syntax checks, shell syntax
# checks, and the frontend build.
#
# This is the "automated test" half of the per-phase evidence pack. The live
# SITL run is the other half and is captured by scripts/capture_evidence.sh.
#
#   scripts/run_tests.sh            # everything
#   scripts/run_tests.sh --quick    # skip the frontend build
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT" || exit 1

QUICK=0
REDUCED_SCOPE=0
for arg in "$@"; do
    case "$arg" in
        --quick) QUICK=1 ;;
        --reduced-scope) REDUCED_SCOPE=1 ;;
        *) echo "Unknown option: $arg" >&2; exit 2 ;;
    esac
done

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

export PYTHONPATH="$ROOT/src/aerothon_mission/mission_bt:$ROOT/src/aerothon_gcs/gcs_aggregator:$ROOT/src/aerothon_perception/camera_ctrl:$ROOT/src/aerothon_avoidance/avoidance:$ROOT/src/aerothon_payload/winch_ctrl:$ROOT/src/aerothon_perception/perception_qr:$ROOT/src/aerothon_perception/perception_banner:${PYTHONPATH:-}"
export PYTHONPATH="$ROOT/src/aerothon_runtime:$PYTHONPATH"

FAILED=0
section() { printf '\n=== %s ===\n' "$1"; }
record()  { if [[ "$1" -ne 0 ]]; then FAILED=1; printf '  -> FAILED\n'; else printf '  -> ok\n'; fi; }

section "Python unit tests"
pytest_args=()
if [[ "$REDUCED_SCOPE" -eq 1 ]]; then
    export PYTHONPATH="$ROOT/sim:$PYTHONPATH"
    pytest_args=(-rs -p qualification_offline_scope --reduced-qualification
                 --qualification-scope-evidence "${AEROTHON_OFFLINE_SCOPE_EVIDENCE:-logs/offline_reduced_scope.json}")
    echo "Reduced scope: deferred strong-gust flight tests are explicitly skipped and recorded."
fi
python3 -m pytest sim/ tests/ -q "${pytest_args[@]}"
record $?

section "Native ROS device and execution interfaces"
# The laptop gateway needs pymavlink, which the distro Python lacks. Point
# AEROTHON_NATIVE_PYTHON at a --system-site-packages venv that adds it (and
# pytest-asyncio) without replacing the ROS NumPy.
"${AEROTHON_NATIVE_PYTHON:-python3}" -m pytest verification/test_pi_ros_runtime.py -q
record $?

section "Python syntax (all mission/perception/GCS/sim sources)"
find src scripts sim -name '*.py' -not -path '*/node_modules/*' -not -path '*/__pycache__/*' -print0 \
    | xargs -0 python3 -m py_compile
record $?

section "Shell syntax"
rc=0
for f in scripts/*.sh; do bash -n "$f" || rc=1; done
record $rc

section "XML / YAML well-formedness"
python3 - <<'PY'
import sys, glob, xml.dom.minidom, yaml
bad = 0
for pat in ("src/**/*.xml", "src/**/*.sdf", "src/**/*.xacro"):
    for f in glob.glob(pat, recursive=True):
        try:
            xml.dom.minidom.parse(f)
        except Exception as e:
            print(f"XML FAIL {f}: {e}"); bad = 1
for f in glob.glob("src/**/*.yaml", recursive=True):
    try:
        yaml.safe_load(open(f))
    except Exception as e:
        print(f"YAML FAIL {f}: {e}"); bad = 1
sys.exit(bad)
PY
record $?

if [[ "$QUICK" -eq 0 ]]; then
    section "GCS frontend build"
    if [[ -d src/aerothon_gcs/tauri_app/node_modules ]]; then
        # A shared Windows/WSL checkout can contain Windows .bin launchers.
        # Execute the installed JavaScript entry points with this host's Node.
        (python3 scripts/export_gcs_contract.py && cd src/aerothon_gcs/tauri_app && node --experimental-strip-types --test tests/*.test.mjs && node node_modules/typescript/bin/tsc --noEmit && node node_modules/vite/bin/vite.js build >/dev/null)
        record $?
    else
        echo "  -> SKIPPED (node_modules absent; run npm install)"
    fi
fi

printf '\n======================================================================\n'
if [[ "$FAILED" -eq 0 ]]; then
    if [[ "$REDUCED_SCOPE" -eq 1 ]]; then
        echo " ALL IN-SCOPE OFFLINE CHECKS PASSED (deferred cases excluded; see scope ledger)"
    else
        echo " ALL OFFLINE CHECKS PASSED"
    fi
else
    echo " OFFLINE CHECKS FAILED — see above"
fi
echo "======================================================================"
exit "$FAILED"
