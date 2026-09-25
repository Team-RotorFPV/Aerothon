#!/usr/bin/env bash
# ==============================================================================
# AeroTHON 2026 — per-phase evidence capture
# ==============================================================================
# Every phase in PHASE_PLAN.md ships an evidence pack. This script collects the
# machine-gathered parts of it:
#
#   1. rosbag2 recording of the topics relevant to the phase
#   2. camera frames (raw + annotated) written as JPEG
#   3. a snapshot of every live topic's current value (topic census)
#   4. node/topic/TF inventory
#   5. optional screenshots of the desktop (Gazebo / RViz / GCS windows)
#   6. a run manifest (git-less: file hashes, versions, timestamps)
#
# The written verdict in VERIFICATION.md is still yours to author. This script
# only produces the raw material so the verdict can be checked later.
#
# Usage:
#   scripts/capture_evidence.sh --phase P0 --duration 30 --label rails
#   scripts/capture_evidence.sh --phase P3 --duration 60 --topics "/percep/qr/decoded /mission/state"
#
# Exit codes: 0 captured, 1 bad usage, 2 no ROS graph found.
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

PHASE=""
DURATION=30
LABEL="run"
EXTRA_TOPICS=""
SCREENSHOTS=1

usage() {
    sed -n '2,20p' "${BASH_SOURCE[0]}"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --phase)      PHASE="$2"; shift 2 ;;
        --duration)   DURATION="$2"; shift 2 ;;
        --label)      LABEL="$2"; shift 2 ;;
        --topics)     EXTRA_TOPICS="$2"; shift 2 ;;
        --no-screens) SCREENSHOTS=0; shift ;;
        -h|--help)    usage ;;
        *) echo "Unknown argument: $1"; usage ;;
    esac
done

[[ -z "$PHASE" ]] && { echo "ERROR: --phase is required (e.g. P0)"; usage; }

# ---- environment -------------------------------------------------------------
set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "$WORKSPACE_ROOT/install/setup.bash" ]] && source "$WORKSPACE_ROOT/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not available; cannot capture evidence."
    exit 2
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$WORKSPACE_ROOT/evidence/$PHASE/${STAMP}-${LABEL}"
mkdir -p "$OUT"

echo "======================================================================"
echo " EVIDENCE CAPTURE  phase=$PHASE  label=$LABEL  duration=${DURATION}s"
echo " Output: $OUT"
echo "======================================================================"

# ---- 0. is there a graph at all? --------------------------------------------
# `ros2 topic list` is never empty: the CLI spins up its own node, so
# /parameter_events and /rosout always appear even with nothing running.
# Presence of real NODES is the honest test.
LIVE_TOPICS="$(timeout 10 ros2 topic list 2>/dev/null || true)"
LIVE_NODES="$(timeout 10 ros2 node list 2>/dev/null || true)"
REAL_TOPICS="$(echo "$LIVE_TOPICS" | grep -vE '^(/parameter_events|/rosout)$' || true)"

if [[ -z "$(echo "$LIVE_NODES" | tr -d '[:space:]')" && -z "$(echo "$REAL_TOPICS" | tr -d '[:space:]')" ]]; then
    echo "ERROR: no ROS 2 nodes are running — there is nothing to capture."
    echo "       Only the CLI's own /parameter_events and /rosout are visible."
    echo "       Start the stack first:  scripts/launch_level6_sim.sh"
    rmdir "$OUT" 2>/dev/null || true
    exit 2
fi
echo "$LIVE_TOPICS" > "$OUT/topic_list.txt"

# ---- 1. inventory ------------------------------------------------------------
{
    echo "=== nodes ==="
    timeout 10 ros2 node list 2>/dev/null
    echo
    echo "=== topics (with types) ==="
    timeout 15 ros2 topic list -t 2>/dev/null
    echo
    echo "=== services ==="
    timeout 15 ros2 service list 2>/dev/null
} > "$OUT/graph_inventory.txt" 2>&1
echo "[1/6] Graph inventory captured."

# ---- 2. topic census: one sample of every mission-relevant topic --------------
CENSUS_TOPICS=(
    /mission/state /mission/result /mission/target /mission/start /mission/abort
    /mission_ready
    /mavros/state /mavros/local_position/pose /mavros/battery
    /mavros/global_position/global /mavros/global_position/raw/satellites
    /percep/qr/decoded /percep/qr/matched /percep/qr/target_offset
    /percep/banner /percep/redzone
    /camera/pose_state
    /avoidance/enable /avoidance/status
    /winch/cmd /winch/status
    /scan /map /tf
)
[[ -n "$EXTRA_TOPICS" ]] && read -r -a EXTRA_ARR <<< "$EXTRA_TOPICS" && CENSUS_TOPICS+=("${EXTRA_ARR[@]}")

: > "$OUT/topic_census.txt"
for t in "${CENSUS_TOPICS[@]}"; do
    if echo "$LIVE_TOPICS" | grep -qx "$t"; then
        {
            echo "----- $t -----"
            timeout 6 ros2 topic echo --once "$t" 2>&1 | head -40
            echo
        } >> "$OUT/topic_census.txt"
    else
        echo "----- $t -----" >> "$OUT/topic_census.txt"
        echo "ABSENT (topic not advertised)" >> "$OUT/topic_census.txt"
        echo >> "$OUT/topic_census.txt"
    fi
done
echo "[2/6] Topic census captured ($(grep -c '^----- ' "$OUT/topic_census.txt") topics)."

# ---- 3. rosbag ----------------------------------------------------------------
BAG_TOPICS=()
for t in "${CENSUS_TOPICS[@]}"; do
    echo "$LIVE_TOPICS" | grep -qx "$t" && BAG_TOPICS+=("$t")
done
# The camera stream is large; include it only when it exists and the caller
# asked for a short capture.
if echo "$LIVE_TOPICS" | grep -qx "/camera/image" && [[ "$DURATION" -le 90 ]]; then
    BAG_TOPICS+=(/camera/image /camera/camera_info)
fi
if echo "$LIVE_TOPICS" | grep -qx "/percep/qr/annotated" && [[ "$DURATION" -le 90 ]]; then
    BAG_TOPICS+=(/percep/qr/annotated)
fi

if [[ "${#BAG_TOPICS[@]}" -gt 0 ]]; then
    echo "[3/6] Recording ${#BAG_TOPICS[@]} topics for ${DURATION}s..."
    timeout "$((DURATION + 15))" ros2 bag record \
        -o "$OUT/rosbag" \
        --max-cache-size 0 \
        "${BAG_TOPICS[@]}" >"$OUT/rosbag_record.log" 2>&1 &
    BAG_PID=$!
    sleep "$DURATION"
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    echo "[3/6] rosbag written."
else
    echo "[3/6] SKIPPED — no recordable topics present."
fi

# ---- 4. camera frames ---------------------------------------------------------
python3 - "$OUT" <<'PY' 2>"$OUT/frame_capture.log" || echo "[4/6] Frame capture unavailable (see frame_capture.log)."
import sys, os, time
out = sys.argv[1]
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
except Exception as e:
    print(f"imports unavailable: {e}")
    raise SystemExit(1)

TOPICS = ["/camera/image", "/percep/qr/annotated", "/percep/banner/annotated"]

rclpy.init()
node = Node("evidence_frame_grabber")
bridge = CvBridge()
saved = {}

def make_cb(topic):
    def cb(msg):
        if topic in saved:
            return
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"{topic}: convert failed {e}")
            return
        name = topic.strip("/").replace("/", "_") + ".jpg"
        cv2.imwrite(os.path.join(out, name), img)
        saved[topic] = True
        print(f"{topic}: saved {name} {img.shape}")
    return cb

# ROS 2 discovery is asynchronous: get_topic_names_and_types() immediately
# after node construction returns an empty/partial graph, which made this
# report "no image topics present" while /camera/image was publishing at 5 Hz.
# Spin briefly so discovery completes before deciding what exists.
deadline = time.time() + 5.0
available = set()
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    available = {t for t, _ in node.get_topic_names_and_types()}
    if any(t in available for t in TOPICS):
        break

wanted = [t for t in TOPICS if t in available]
if not wanted:
    print(f"no image topics present (discovered {len(available)} topics)")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(0)

print(f"subscribing to: {wanted}")
subs = [node.create_subscription(Image, t, make_cb(t), 1) for t in wanted]

end = node.get_clock().now().nanoseconds + 12e9
while rclpy.ok() and len(saved) < len(subs) and node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node(); rclpy.shutdown()
PY
ls "$OUT"/*.jpg >/dev/null 2>&1 && echo "[4/6] Camera frames saved: $(ls "$OUT"/*.jpg | wc -l)."

# ---- 5. TF tree ---------------------------------------------------------------
if echo "$LIVE_TOPICS" | grep -qx "/tf"; then
    (cd "$OUT" && timeout 20 ros2 run tf2_tools view_frames >/dev/null 2>&1) || true
    # tf2_tools writes frames_<date>_<time>.pdf, not frames.pdf. Checking the
    # latter reported "TF tree unavailable" on a run that had produced it.
    TF_PDF="$(find "$OUT" -maxdepth 1 -name 'frames*.pdf' -print -quit 2>/dev/null)"
    if [[ -n "$TF_PDF" ]]; then
        echo "[5/6] TF tree saved ($(basename "$TF_PDF"))."
    else
        echo "[5/6] TF tree unavailable (tf2_tools missing or no transforms)."
    fi
else
    echo "[5/6] SKIPPED — no /tf."
fi

# ---- 6. screenshots -----------------------------------------------------------
if [[ "$SCREENSHOTS" == "1" ]]; then
    if command -v import >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
        import -window root "$OUT/desktop.png" 2>/dev/null \
            && echo "[6/6] Desktop screenshot saved." \
            || echo "[6/6] Screenshot failed."
    elif command -v gnome-screenshot >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
        gnome-screenshot -f "$OUT/desktop.png" 2>/dev/null \
            && echo "[6/6] Desktop screenshot saved." \
            || echo "[6/6] Screenshot failed."
    else
        echo "[6/6] SKIPPED — no screenshot tool or no DISPLAY."
    fi
else
    echo "[6/6] SKIPPED — --no-screens."
fi

# ---- manifest -----------------------------------------------------------------
{
    echo "phase:      $PHASE"
    echo "label:      $LABEL"
    echo "timestamp:  $STAMP"
    echo "duration_s: $DURATION"
    echo "host:       $(hostname)"
    echo "user:       $(whoami)"
    echo "ros_distro: ${ROS_DISTRO:-unknown}"
    echo "kernel:     $(uname -r)"
    echo
    echo "=== source file hashes (mission-critical) ==="
    for f in \
        src/aerothon_mission/mission_bt/mission_bt/mission_tree.py \
        src/aerothon_mission/mission_bt/mission_bt/mav_commander.py \
        src/aerothon_avoidance/avoidance/avoidance/velocity_controller.py \
        src/aerothon_gcs/gcs_aggregator/gcs_aggregator/aggregator.py \
        src/aerothon_sim/sim_gazebo/worlds/mission2.sdf ; do
        [[ -f "$WORKSPACE_ROOT/$f" ]] && sha256sum "$WORKSPACE_ROOT/$f"
    done
} > "$OUT/manifest.txt" 2>&1

echo "======================================================================"
echo " Evidence pack complete: $OUT"
ls -la "$OUT"
echo "======================================================================"
echo " Remember to write the verdict into VERIFICATION.md."
