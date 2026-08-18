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
export AEROTHON_OPEN_GCS=0
export AEROTHON_HEADLESS=1
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
if python3 sim/wait_ready.py --timeout 300 --require-scan; then
    ready=1
fi
if [[ "$ready" != "1" ]]; then
    echo "STACK NEVER BECAME READY"
    grep -acE "process has died" "$LOG" 2>/dev/null | sed 's/^/deaths: /'
    grep -aE "process has died" "$LOG" 2>/dev/null | head -4
    exit 1
fi

echo ""
echo "--- which target did the start pad get? ---"
grep -a "start pad names" "$LOG" | tail -1

echo ""
echo "--- supporting nodes ---"
timeout 12 ros2 node list 2>/dev/null | grep -E "camera_ctrl|winch_ctrl|stream_rate_keeper|perception_qr" || echo "(none found)"

echo ""
echo "--- starting mission ---"
timeout 12 ros2 topic pub -r 2 /mission/start std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 &
sleep 13

echo ""
printf "%6s  %-18s %-30s %s\n" "t(s)" "state" "position" "camera/qr"
last_state=""
for i in $(seq 1 $((WATCH / 8))); do
    st=$(timeout 4 ros2 topic echo --once /mission/state 2>/dev/null | head -1 | sed 's/data: //' | tr -d "'" || echo "?")
    pos=$(timeout 5 ros2 topic echo --once /mavros/local_position/pose 2>/dev/null \
          | grep -A3 "position:" | tail -3 | awk -F: '{printf "%.1f ", $2}' || echo "")
    det=$(timeout 4 ros2 topic echo --once /percep/qr/detail 2>/dev/null | head -1 \
          | sed 's/data: //' | cut -c1-70 || echo "")
    printf "%6s  %-18s %-30s %s\n" "$((i*8))" "$st" "[$pos]" "$det"
    [[ "$st" == "$last_state" ]] || last_state="$st"

    # Stop once the tree has latched a terminal outcome. The mission result is
    # first-writer-wins, so there is nothing further to observe -- and each
    # iteration costs up to 17 s of `ros2 topic echo` timeouts. An arena that
    # failed at t=24 s was still being watched at t=520 s, which made a
    # five-arena regression mostly a study of idle simulators.
    #
    # Read from the tree's own log rather than /mission/result: short-lived
    # ros2 CLI calls lose the DDS discovery race against this stack.
    if grep -aq "Mission result:" "$LOG" 2>/dev/null; then
        echo ""
        echo "--- terminal outcome latched, ending watch early ---"
        grep -a "Mission result:" "$LOG" | tail -1
        break
    fi
    sleep 4
done

echo ""
echo "--- outcome ---"
timeout 6 ros2 topic echo --once /mission/result 2>/dev/null | head -2 || echo "(no result latched)"

echo ""
echo "--- winch ---"
timeout 6 ros2 topic echo --once /winch/status 2>/dev/null | head -2 || echo "(none)"

echo ""
echo "--- mission state transitions seen by the tree ---"
grep -a "Mission state ->\|Mission result\|Mission FAILED\|Mission tree reset" "$LOG" | tail -25

echo ""
echo "--- node deaths ---"
grep -ac "process has died" "$LOG" 2>/dev/null || echo 0
