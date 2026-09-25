#!/usr/bin/env bash
# ==============================================================================
# Phase 11 — full mission across N randomised arenas.
# ==============================================================================
# WHY THIS IS THE ACCEPTANCE TEST FOR THE WHOLE ARCHITECTURE
#
#   Every phase replaced a hardcoded coordinate with perception, and every one
#   of those replacements was checked against the SINGLE arena the coordinates
#   came from. That cannot tell "derived from what the camera sees" apart from
#   "derived from a different constant that happens to agree".
#
#   Moving the arena can. Three live failures this session were only exposed
#   when a hardcoded waypoint stopped dragging the aircraft to the right place.
#   Those were found by accident; this finds them on purpose.
#
# Each run gets its own seed, world, log and classified outcome. Nothing is
# summarised as "mostly worked": every failure is named.
#
#   scripts/arena_regression.sh              # 5 arenas
#   scripts/arena_regression.sh -n 10        # 10 arenas
#   scripts/arena_regression.sh -n 3 --watch 300
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT" || exit 1

N=5
# Wall seconds per arena. A full mission is ~5 simulated minutes, which at
# the ~0.28 real-time factor of a software-rendered WSL host is ~45 minutes
# of wall time; 420 s cut every run off mid-search.
WATCH=5400
FIRST=1001
OUT="${AEROTHON_REGRESSION_DIR:-/tmp/aerothon_arena_regression}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--runs) N="$2"; shift 2 ;;
        --watch)   WATCH="$2"; shift 2 ;;
        --first-seed) FIRST="$2"; shift 2 ;;
        --out)     OUT="$2"; shift 2 ;;
        *) echo "unknown arg: $1"; exit 2 ;;
    esac
done

# ---------------------------------------------------------------------------
# EXCLUSIVE ACCESS TO THE SIMULATOR
#
# live_mission_test.sh begins by pkill-ing gz sim and arducopter. Two of these
# running at once therefore do not merely share a simulator -- each one kills
# the other's aircraft mid-flight. That happened: an orphaned run from 19:12
# (reparented to systemd when its launcher was killed) ran concurrently with a
# fresh one from 19:18, and produced a "regression result" of five failures
# including an "external disarm" that was simply the other harness shooting the
# simulator. Contaminated results that LOOK like findings are worse than none.
# ---------------------------------------------------------------------------
LOCK=/tmp/aerothon_arena_regression.lock
exec 9>"$LOCK"
if ! flock -n 9; then
    echo "ERROR: another arena regression already holds $LOCK." >&2
    echo "Running two at once makes both sets of results meaningless." >&2
    pgrep -af 'arena_regression|live_mission_test' >&2 || true
    exit 3
fi
echo $$ >&9

# Nothing left over from a previous run, orphaned or otherwise.
if pgrep -f "[l]ive_mission_test" >/dev/null; then
    echo "ERROR: a live_mission_test is already running:" >&2
    pgrep -af "[l]ive_mission_test" >&2
    exit 3
fi

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'seed\ttarget\toutcome\tgrade\treason\tduration_s\tlog\n' > "$SUMMARY"

echo "=================================================================="
echo " PHASE 11 ARENA REGRESSION   runs=$N   watch=${WATCH}s"
echo " output: $OUT"
echo "=================================================================="

pass=0
for i in $(seq 1 "$N"); do
    SEED=$((FIRST + i - 1))
    LOG="$OUT/arena_${SEED}.log"
    echo ""
    echo "--- arena $i/$N (seed $SEED) ---"

    # A different arena AND a different delivery target every run: the target
    # is read off the start QR, so a stack that guessed would be caught here.
    export AEROTHON_SEED="$SEED"
    export AEROTHON_RANDOM_ARENA=1
    export AEROTHON_START_TARGET=random

    START=$(date +%s)
    # 9>&- CLOSES the lock fd in the child. Without it the whole simulator
    # stack -- gz sim, arducopter, mavros, every ROS node -- inherits fd 9 and
    # keeps the flock alive after this script exits, so the NEXT regression
    # refuses to start against a lock nothing is really using. Verified with
    # `fuser -v` on the lock file: 25 inheriting processes.
    bash scripts/live_mission_test.sh --watch "$WATCH" --start-target random \
        > "$LOG" 2>&1 9>&-
    DUR=$(( $(date +%s) - START ))

    TARGET=$(grep -am1 "start pad names delivery target" "$LOG" \
             | sed 's/.*target \([A-E]\).*/\1/' || echo "?")

    # Parse the tree's own latched outcome line:
    #     Mission result: COMPLETED (landed and disarmed; landing PRECISE ...)
    #
    # NOT the /mission/result topic. This used to grep for '"state":' from a
    # `ros2 topic echo --once`, which appears in exactly zero logs: short-lived
    # ros2 CLI calls lose the DDS discovery race against this stack (the same
    # reason sim/run_mission_live.py is a long-lived rclpy node). Every run
    # would have been classified NO_OUTCOME and the harness would have reported
    # 0/N passes no matter how the missions actually went.
    LINE=$(grep -a "Mission result:" "$LOG" | tail -1 || true)
    RESULT=$(sed -n 's/.*Mission result: \([A-Z_]*\).*/\1/p' <<<"$LINE")
    REASON=$(sed -n 's/.*Mission result: [A-Z_]* (\(.*\))$/\1/p' <<<"$LINE")
    [[ -z "$RESULT" ]] && RESULT="NO_OUTCOME"
    [[ -z "$REASON" ]] && REASON="(none recorded)"

    # Which arena this actually was, so a failure can be re-flown.
    ARENA=$(grep -am1 "RANDOMISED ARENA:" "$LOG" | cut -c1-160 || true)
    [[ -n "$ARENA" ]] && echo "    $ARENA" >> "$OUT/layouts.txt"

    # GRADE THE TRACK, not just the outcome line. COMPLETED says the tree ran
    # to the end; it does not say the aircraft stayed off red ground, inside
    # the geofence, under the ceiling, or dropped on the pad. check_track
    # answers those from the recorded track against this arena's layout.
    cp /tmp/aerothon_track.csv "$OUT/arena_${SEED}_track.csv" 2>/dev/null
    # The stack's own log (tree decisions, routing, red-zone map). It is
    # overwritten by the next arena, and without it a graded failure cannot
    # be diagnosed after the fact.
    cp /tmp/aerothon_live_mission.log "$OUT/arena_${SEED}_stack.log" 2>/dev/null
    cp /tmp/aerothon_arena_layout.json "$OUT/arena_${SEED}_layout.json" 2>/dev/null
    GRADE="UNGRADED"
    if [[ -f "$OUT/arena_${SEED}_track.csv" && -f "$OUT/arena_${SEED}_layout.json" ]]; then
        if python3 sim/check_track.py --track "$OUT/arena_${SEED}_track.csv" \
                --layout "$OUT/arena_${SEED}_layout.json" \
                --target "$(tr 'A-E' 'a-e' <<<"$TARGET")" \
                > "$OUT/arena_${SEED}_grade.txt" 2>&1; then
            GRADE="GRADE_PASS"
        else
            GRADE="GRADE_FAIL"
        fi
    fi

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$SEED" "$TARGET" "$RESULT" "$GRADE" "$REASON" "$DUR" "$LOG" >> "$SUMMARY"
    echo "    target=$TARGET  outcome=$RESULT  grade=$GRADE  ${DUR}s"
    echo "    reason: $REASON"
    grep -a "\[FAIL\]" "$OUT/arena_${SEED}_grade.txt" 2>/dev/null | sed 's/^/    /'
    [[ "$RESULT" == "COMPLETED" && "$GRADE" == "GRADE_PASS" ]] && pass=$((pass + 1))
done

# Tear the last arena's stack down. It is left running otherwise, which holds
# the simulator (and, before 9>&-, the lock) against whatever runs next.
pkill -f "[l]ive_mission_test" 2>/dev/null || true
pkill -f "[g]z sim" 2>/dev/null || true
pkill -f "[a]rducopter" 2>/dev/null || true
pkill -f "[m]avros_node" 2>/dev/null || true
pkill -f "[m]avproxy" 2>/dev/null || true
pkill -f "[l]aunch_level6_sim" 2>/dev/null || true
sleep 3

echo ""
echo "=================================================================="
printf ' %d/%d arenas COMPLETED\n' "$pass" "$N"
echo "=================================================================="
echo ""
echo "Every run, classified — a success rate with unexplained failures is not"
echo "a result:"
column -t -s $'\t' "$SUMMARY"

# Non-zero if anything failed, so this can gate a release.
[[ "$pass" -eq "$N" ]]
