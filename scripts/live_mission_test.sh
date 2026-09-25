#!/usr/bin/env bash
# ==============================================================================
# Live end-to-end mission run against SITL.
# ==============================================================================
# Launch, wait for health, start the mission, and record how far it gets and
# why it stopped. Everything happens inside ONE process so that the stack and
# the observer share a lifetime — running the observer as a separate long
# command has repeatedly caused the stack to be torn down mid-flight.
#
#   scripts/live_mission_test.sh                     # defaults
#   scripts/live_mission_test.sh --watch 240 --start-target c
#
# Exit: 0 the mission reached a terminal outcome, 1 it never started.
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT" || exit 1

WATCH=240
START_TARGET="${AEROTHON_START_TARGET:-random}"
LOG=/tmp/aerothon_live_mission.log

while [[ $# -gt 0 ]]; do
    case "$1" in
        --watch)        WATCH="$2"; shift 2 ;;
        --start-target) START_TARGET="$2"; shift 2 ;;
        *) echo "unknown arg: $1"; exit 2 ;;
    esac
done

echo "=================================================================="
echo " LIVE MISSION TEST   watch=${WATCH}s   start target=${START_TARGET}"
echo "=================================================================="

# FULL teardown, not just the simulator.
#
# This used to kill gz sim and arducopter only. Everything else from the
# previous run -- MAVROS, MAVProxy, the router, web_video_server, the ros_gz
# bridges, SLAM, and every perception node -- kept running and fought the new
# stack for ports and topics. The symptoms were different every time and none
# of them named the cause:
#
#   * "STACK NEVER BECAME READY, deaths: 1" with web_video_server aborting,
#     because a copy from a run hours earlier still held port 8080;
#   * an aircraft that armed and then auto-disarmed without ever taking off;
#   * "FCU connected MISSING" with nothing at all flowing.
#
# Every pattern is bracketed so it cannot match the shell running pkill --
# an unbracketed one kills the killer, which is how a stale process survived
# three teardown attempts earlier and silently corrupted a whole regression.
for pat in "[g]z sim" "[a]rducopter" "[m]avros_node" "[m]avproxy" \
           "[m]av_router" "[w]eb_video_server" "[r]os_gz_bridge" \
           "[s]lam_toolbox" "[l]ifecycle_manager" "[r]obot_state_publisher" \
           "[l]aunch_level6_sim" "[A]EROTHON/install"; do
    pkill -f "$pat" 2>/dev/null || true
done
sleep 4
for pat in "[g]z sim" "[a]rducopter" "[m]avros_node" "[m]avproxy"; do
    pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2

# Ports the stack binds. A leftover holder makes the new node abort on start.
for port in 8080 8899; do
    holder=$(ss -ltnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | head -1)
    [[ -n "$holder" ]] && kill -9 "$holder" 2>/dev/null || true
done
sleep 3

export AEROTHON_OFFICIAL_WS="${AEROTHON_OFFICIAL_WS:-$HOME/aerothon_stack}"
# Both default to a measurement run -- no GUI, no browser -- but stay
# overridable, so the same harness can be used to WATCH a run. Hardcoding them
# meant the only way to fly a regression seed by eye was to edit this file,
# and an edited harness is not the harness the results came from.
export AEROTHON_OPEN_GCS="${AEROTHON_OPEN_GCS:-0}"
export AEROTHON_HEADLESS="${AEROTHON_HEADLESS:-1}"
export AEROTHON_START_TARGET="$START_TARGET"

setsid bash scripts/launch_level6_sim.sh > "$LOG" 2>&1 &
sleep 5

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

echo ""
echo "--- waiting for stack ---"
# A LONG-LIVED node, not repeated `ros2 topic --once`/`hz` calls. Each CLI
# invocation is a new DDS participant that must discover the whole graph
# inside its own timeout, and against this stack it loses that race often
# enough to abort healthy runs: one was reported "STACK NEVER BECAME READY"
# with deaths:0 while an rclpy node saw connected=True and pose at 16.2 Hz at
# that same moment. Staying alive also allows measuring a RATE instead of
# noting that one message arrived.
ready=0
# At a real-time factor of ~0.3 (software rendering, no GPU) the EKF origin
# alone takes over three wall minutes, so 300 s was not enough on this host.
if python3 sim/wait_ready.py --timeout "${AEROTHON_READY_TIMEOUT:-900}" --require-scan; then
    ready=1
fi
if [[ "$ready" != "1" ]]; then
    echo "STACK NEVER BECAME READY"
    grep -acE "process has died" "$LOG" 2>/dev/null | sed 's/^/deaths: /'
    grep -aE "process has died" "$LOG" 2>/dev/null | head -4
    exit 1
fi

echo ""
echo "--- host speed (RTF) and sensor rates ---"
timeout 40 python3 sim/probe_rates.py --seconds 15 || echo "(probe failed)"

echo ""
echo "--- which target did the start pad get? ---"
grep -a "start pad names" "$LOG" | tail -1

echo ""
echo "--- starting and watching the mission ---"
# ONE long-lived rclpy node publishes the start and watches to the latched
# result. The `ros2 topic pub` / `ros2 topic echo --once` calls this used to
# make lose DDS discovery against this stack: the start never arrived and the
# tree sat in WAITING for the whole watch window, reported as "?" every line.
python3 sim/run_mission_live.py --watch "$WATCH" --settle 10
RUN_RC=$?

echo ""
echo "--- mission state transitions seen by the tree ---"
grep -a "Mission state ->\|Mission result\|Mission FAILED\|Mission tree reset" "$LOG" | tail -40

echo ""
echo "--- node deaths ---"
grep -ac "process has died" "$LOG" 2>/dev/null || echo 0
exit $RUN_RC
