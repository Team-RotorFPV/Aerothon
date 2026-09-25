#!/usr/bin/env bash
# ==============================================================================
# Watch (and optionally record) a running mission from a Gazebo chase camera.
# ==============================================================================
#   scripts/watch_mission_gui.sh                 # open the chase view
#   scripts/watch_mission_gui.sh --record OUT.mp4
#
# Start it alongside scripts/live_mission_test.sh (run_custom_world.sh does).
# It waits for the stack, attaches a SEPARATE Gazebo GUI to the headless server
# -- so the flight is the same one the regression grades -- follows the
# aircraft, and with --record captures that window from PREFLIGHT until just
# after the mission result.
#
# The capture runs on the wall clock, and this host flies at ~0.2-0.3x real
# time, so the clip is sped up by the MEASURED ratio of wall to sim time over
# the recording: it plays at true flight speed. 8 fps at 1280 px is plenty
# (that is 30-40 frames per SIMULATED second) and keeps the encoder from
# starving next to the simulator -- a 15 fps 1080p capture queued raw frames
# in memory until it had to be stopped.
# ==============================================================================
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REC=""
[[ "${1:-}" == "--record" ]] && REC="${2:?--record needs an output file}"
LIVE=/tmp/aerothon_live_mission.log
export GZ_PARTITION="${GZ_PARTITION:-aerothon_m2}" GZ_IP="${GZ_IP:-127.0.0.1}"
say() { echo "[watch $(date +%T)] $*"; }
srv() { gz service -s "$1" --reqtype "$2" --reptype gz.msgs.Boolean --timeout 5000 --req "$3" >/dev/null; }
simtime() {
    timeout 10 gz topic -e -n 1 -t /world/mission2/stats 2>/dev/null | awk '
        /^sim_time/ {s=1} s && /sec:/ && !/nsec/ {sec=$2} s && /nsec:/ {ns=$2; print sec + ns/1e9; exit}'
}

say "waiting for the aircraft in Gazebo"
sleep 20                                  # the harness reaps the old stack first
until timeout 10 gz topic -l 2>/dev/null | grep -q "/model/aerothon_iris/odometry"; do sleep 3; done
sleep 5
GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}" gz sim -g --render-engine-gui ogre \
    --gui-config "$ROOT/tools/world_editor/chase_gui.config" > /tmp/aerothon_watch_gui.log 2>&1 &
GUI=$!
for _ in $(seq 1 90); do
    timeout 10 gz service -l 2>/dev/null | grep -q "^/gui/follow$" && break; sleep 2
done
sleep 5
srv /gui/follow gz.msgs.StringMsg 'data: "aerothon_iris"'
srv /gui/follow/offset gz.msgs.Vector3d 'x: -4.0, y: -4.0, z: 3.0'
say "chase view open"
[[ -z "$REC" ]] && { wait $GUI; exit 0; }

until grep -qa "Mission state -> PREFLIGHT" "$LIVE" 2>/dev/null; do
    kill -0 $GUI 2>/dev/null || { say "GUI closed before the start"; exit 1; }
    sleep 2
done
# SEGMENTS. The window capture dies if the Gazebo window is minimised or
# unmapped (x11grab: "Cannot get the image data" / "Permission denied"), and
# a clip that silently ends ten minutes early is worse than a gap. Each time it
# dies, find the window again and start a new segment; every segment is sped
# up by its OWN measured wall/sim ratio and the segments are joined at the end.
TMP=$(mktemp -d)
seg=0; done_flag=0; list="$TMP/list.txt"; : > "$list"
say "recording"
while [[ "$done_flag" == "0" ]]; do
    WIN=$(xdotool search --name "^Gazebo Sim$" | head -1)
    [[ -z "$WIN" ]] && { sleep 3; kill -0 $GUI 2>/dev/null || break; continue; }
    seg=$((seg + 1)); RAW="$TMP/seg$seg.mkv"
    SIM0=$(simtime); W0=$(date +%s.%N)
    nice -n 5 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 8 -window_id "$WIN" \
        -i :0 -vf scale=1280:-2 -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
        -f matroska "$RAW" < /dev/null &
    FF=$!
    finished_at=""
    while kill -0 $FF 2>/dev/null; do
        if grep -qa "Mission result:" "$LIVE" 2>/dev/null; then
            [[ -z "$finished_at" ]] && finished_at=$(date +%s)
            # a few simulated seconds after touchdown
            if (( $(date +%s) - finished_at >= 20 )); then done_flag=1; break; fi
        fi
        sleep 2
    done
    SIM1=$(simtime); W1=$(date +%s.%N)
    kill -INT $FF 2>/dev/null; wait $FF 2>/dev/null
    grep -qa "Mission result:" "$LIVE" 2>/dev/null && [[ -n "$finished_at" ]] && done_flag=1
    [[ "$done_flag" == "0" ]] && say "capture dropped (segment $seg); restarting it"
    SPEED=$(awk -v a="$W0" -v b="$W1" -v c="$SIM0" -v d="$SIM1" \
            'BEGIN { s = (d > c) ? (b - a) / (d - c) : 1; if (s < 1) s = 1; printf "%.3f", s }')
    if [[ -s "$RAW" ]]; then
        ffmpeg -y -loglevel error -i "$RAW" -vf "setpts=PTS/${SPEED},fps=30,scale=1280:720" \
            -c:v libx264 -preset medium -crf 21 -pix_fmt yuv420p -an "$TMP/seg$seg.mp4" \
            && echo "file '$TMP/seg$seg.mp4'" >> "$list"
        say "segment $seg: x$SPEED to real time"
    fi
    [[ "$done_flag" == "0" ]] && sleep 3
done
if [[ -s "$list" ]]; then
    ffmpeg -y -loglevel error -f concat -safe 0 -i "$list" -c copy "$REC"
    say "saved $REC ($seg segment(s))"
else
    say "nothing was captured"
fi
rm -rf "$TMP"
kill $GUI 2>/dev/null
