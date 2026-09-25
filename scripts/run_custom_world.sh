#!/usr/bin/env bash
# ==============================================================================
# Fly and grade Mission 2 in a user-built arena (tools/world_editor).
# ==============================================================================
#   scripts/run_custom_world.sh sim/worlds/my_world.json
#   scripts/run_custom_world.sh sim/worlds/my_world.json --gui       # watch it
#   scripts/run_custom_world.sh sim/worlds/my_world.json --record    # + video
#
# Everything the mission meets -- where it takes off, where the corridor is and
# which way it points, the delivery zone's size, the geofence, every red zone,
# every pad, which pad the start QR names -- comes from the file. No seed and
# no randomiser: if the mission still depends on a constant anywhere, a world
# that disagrees with it shows up here as a failed grade.
#
# Output in logs/custom/: NAME.out (harness), NAME_stack.log, NAME_track.csv,
# NAME_layout.json, NAME_grade.txt, and NAME.mp4 with --record.
# ==============================================================================
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

SPEC="$(realpath "${1:?usage: run_custom_world.sh WORLD.json [--gui] [--record]}")"
shift
GUI=0; REC=0
for a in "$@"; do
    case "$a" in
        --gui) GUI=1 ;;
        --record) GUI=1; REC=1 ;;
        *) echo "unknown arg: $a"; exit 2 ;;
    esac
done
NAME="$(basename "$SPEC" .json)"
OUT="$ROOT/logs/custom"
mkdir -p "$OUT"

# Refuse a bad world before touching the simulator.
python3 - "$SPEC" <<'PY' || exit 3
import sys; sys.path.insert(0, "scripts")
import world_spec
errs, warns = world_spec.validate(world_spec.load(sys.argv[1]))
for w in warns: print("warning:", w)
for e in errs: print("ERROR:", e)
sys.exit(1 if errs else 0)
PY

export AEROTHON_WORLD_SPEC="$SPEC"
# Nothing from a previous run's environment may leak into this arena.
unset AEROTHON_RANDOM_ARENA AEROTHON_SEED AEROTHON_DELIVERY_ZONE AEROTHON_GEOFENCE
# The live log is shared by every run: clear it, or the editor's status (and
# the recorder's "Mission result" wait) read the LAST run's outcome.
rm -f /tmp/aerothon_track.csv /tmp/aerothon_live_mission.log \
      "$OUT/${NAME}_grade.txt" "$OUT/${NAME}.mp4"

echo "flying custom world '$NAME' ($SPEC)"
bash scripts/live_mission_test.sh --watch "${AEROTHON_WATCH:-5400}" > "$OUT/${NAME}.out" 2>&1 &
RUN=$!
WATCH=""
if [[ "$GUI" == "1" ]]; then
    if [[ "$REC" == "1" ]]; then
        bash scripts/watch_mission_gui.sh --record "$OUT/${NAME}.mp4" > "$OUT/${NAME}.watch.log" 2>&1 &
    else
        bash scripts/watch_mission_gui.sh > "$OUT/${NAME}.watch.log" 2>&1 &
    fi
    WATCH=$!
fi
wait $RUN
[[ -n "$WATCH" && "$REC" == "1" ]] && wait "$WATCH"
[[ -n "$WATCH" && "$REC" != "1" ]] && kill "$WATCH" 2>/dev/null

cp /tmp/aerothon_live_mission.log "$OUT/${NAME}_stack.log" 2>/dev/null
cp /tmp/aerothon_track.csv "$OUT/${NAME}_track.csv" 2>/dev/null
cp /tmp/aerothon_arena_layout.json "$OUT/${NAME}_layout.json" 2>/dev/null
grep -a "Mission result:" "$OUT/${NAME}_stack.log" | tail -1
TARGET=$(grep -am1 "start pad names delivery target" "$OUT/${NAME}.out" "$OUT/${NAME}_stack.log" 2>/dev/null \
         | head -1 | sed 's/.*target \([A-E]\).*/\1/' | tr 'A-E' 'a-e')
if [[ -f "$OUT/${NAME}_track.csv" ]]; then
    python3 sim/check_track.py --track "$OUT/${NAME}_track.csv" \
        --layout "$OUT/${NAME}_layout.json" --target "${TARGET:-a}" > "$OUT/${NAME}_grade.txt" 2>&1
    RC=$?
    grep -a "\[" "$OUT/${NAME}_grade.txt"
else
    echo "no track recorded" | tee "$OUT/${NAME}_grade.txt"
    RC=1
fi
exit $RC
