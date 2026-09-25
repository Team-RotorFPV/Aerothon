#!/usr/bin/env bash
# ==============================================================================
# Post-launch health gate.
# ==============================================================================
# The launcher starts a dozen processes and returns. If MAVROS never reaches the
# flight controller, or a node aborts during start-up, everything downstream
# fails in a way that looks like a code bug: "connected: false" with no error,
# or a measurement script timing out on a topic that was never going to appear.
#
# This waits for the stack to be genuinely usable and says plainly which part is
# missing if it is not.
#
#   scripts/wait_for_stack.sh              # default 240 s budget
#   scripts/wait_for_stack.sh --timeout 90 --require-camera
#
# Exit: 0 healthy, 1 timed out (with a per-check report), 2 environment problem.
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

TIMEOUT=240
REQUIRE_CAMERA=0
REQUIRE_SCAN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout)        TIMEOUT="$2"; shift 2 ;;
        --require-camera) REQUIRE_CAMERA=1; shift ;;
        --require-scan)   REQUIRE_SCAN=1; shift ;;
        *) echo "unknown argument: $1"; exit 2 ;;
    esac
done

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

command -v ros2 >/dev/null || { echo "ros2 not available"; exit 2; }

have_topic() { timeout 6 ros2 topic list 2>/dev/null | grep -qx "$1"; }
fcu_connected() {
    timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null \
        | grep -q "connected: true"
}
topic_flowing() {
    timeout 9 ros2 topic hz "$1" 2>&1 | grep -q "average rate"
}

deadline=$((SECONDS + TIMEOUT))
ok_nodes=0; ok_fcu=0; ok_pose=0; ok_cam=0; ok_scan=0

echo "Waiting up to ${TIMEOUT}s for the stack to become usable..."
while [[ $SECONDS -lt $deadline ]]; do
    [[ $ok_nodes -eq 0 ]] && have_topic /mavros/state && { ok_nodes=1; echo "  [ok] MAVROS present"; }
    if [[ $ok_nodes -eq 1 && $ok_fcu -eq 0 ]] && fcu_connected; then
        ok_fcu=1; echo "  [ok] FCU connected"
    fi
    if [[ $ok_fcu -eq 1 && $ok_pose -eq 0 ]] && topic_flowing /mavros/local_position/pose; then
        ok_pose=1; echo "  [ok] local position flowing"
    fi
    if [[ $REQUIRE_CAMERA -eq 1 && $ok_cam -eq 0 ]] && topic_flowing /camera/image; then
        ok_cam=1; echo "  [ok] camera flowing"
    fi
    if [[ $REQUIRE_SCAN -eq 1 && $ok_scan -eq 0 ]] && topic_flowing /scan; then
        ok_scan=1; echo "  [ok] lidar flowing"
    fi

    done_all=1
    [[ $ok_pose -eq 1 ]] || done_all=0
    [[ $REQUIRE_CAMERA -eq 0 || $ok_cam -eq 1 ]] || done_all=0
    [[ $REQUIRE_SCAN -eq 0 || $ok_scan -eq 1 ]] || done_all=0
    [[ $done_all -eq 1 ]] && { echo "Stack is healthy."; exit 0; }

    sleep 4
done

echo ""
echo "STACK NOT HEALTHY after ${TIMEOUT}s:"
[[ $ok_nodes -eq 1 ]] && echo "  MAVROS present            OK" || echo "  MAVROS present            MISSING  <- launcher failed early"
[[ $ok_fcu   -eq 1 ]] && echo "  FCU connected             OK" || echo "  FCU connected             MISSING  <- check the router/MAVProxy ports"
[[ $ok_pose  -eq 1 ]] && echo "  local position flowing    OK" || echo "  local position flowing    MISSING  <- EKF may still be initialising"
[[ $REQUIRE_CAMERA -eq 1 ]] && { [[ $ok_cam -eq 1 ]] && echo "  camera flowing            OK" || echo "  camera flowing            MISSING"; }
[[ $REQUIRE_SCAN -eq 1 ]] && { [[ $ok_scan -eq 1 ]] && echo "  lidar flowing             OK" || echo "  lidar flowing             MISSING"; }
echo ""
echo "Check the launcher log, and look for 'process has died' entries."
exit 1
