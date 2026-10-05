# Current progress, 2026-10-03

## Inspected

- Base commit `210daa7`; existing uncommitted work preserved.
- Historical root handoff is superseded by current code and logs.
- Latest stored nine-world campaign passed 7/9; both failures reproduced or
  isolated before editing.
- Physical claw findings remain in `docs/hook_physics.md`: uneven loading
  loses the payload; even loading can carry it but stow may grab it again.
  Full-mission simulated payload release does not qualify that CAD mechanism.

## Continued

- Reject clipped banner geometry before placing the return stand-off.
- Retain an expired stall reference until measurable movement starts a new
  window, allowing existing backoff/STUCK recovery to run.
- Three regression tests cover the false range, stationary visible-gap
  recovery, and recovery after actual movement.
- Targeted final checks: 95 passed (`logs/continue_targeted_20261003.log`).
- Rotated seed 1 and shipped seed 1: 2/2 COMPLETED and independently PASS,
  zero contacts and fence breaches (`logs/continue_fix_20261003`).
- Off-axis seed 1: bounded STUCK failure after all three backoffs. Still FAIL
  as a mission; its original geometry was preserved.

## Additional closed-loop validation

- Rotated seeds 2-3 and current `my_world` seeds 2-3: 4/4 COMPLETED and
  independently PASS, zero contacts and fence breaches.
  Evidence: `logs/continue_regression_20261003/summary.json`.
- Together with rotated/shipped seed 1, 6/6 successful mission runs in this
  continuation. All used the worlds' own calm presets; these are different
  mission seeds, not a new random/worst-conditions qualification campaign.
- Both current-world runs reported delivery UNMEASURED in mission telemetry
  because the pad was absent from the release frame. Payload ground detection
  was confirmed, and the independent ground-truth grader passed placement.
- Initial full offline run: 1218 passed, 3 skipped; Python/shell syntax and
  XML/YAML checks passed (`logs/continue_offline_20261003.log`).
- Final full offline run: 1220 passed, 3 skipped; Python/shell syntax and
  XML/YAML checks passed (`logs/continue_final_offline_20261003.log`).
  The runner used `--quick`; the unchanged GCS frontend was not rebuilt.

## Remaining

- Off-axis chicane planner capability, issue 03.
- Re-run rendered Gazebo + ArduPilot SITL with the current code. The last
  saved `my_world` rendered mission completed but failed restricted-zone
  grading. The interrupted detector-rate pacing change targets that defect;
  headless passing missions do not verify its rendered-camera timing.
- Physical claw retention and release/stow, then real bench qualification.
- Pi, sensors, real-camera and flight qualification in `docs/FIELD_READINESS.md`.

## Recorded rendered rerun - 2026-10-03

- Ran the last failed world `my_world_offaxis.json`, seed 1, calm, target B,
  unchanged, in rendered Gazebo Harmonic with ArduPilot SITL.
- FAILED at 796.8 simulated seconds: return recovery backed out through the
  entrance; `ReturnCorridor` rejected the mouth as the far end. RTL then
  landed and FCU disarmed. The full airborne interval was 780.0 seconds.
- Full-flight independent grade: FAIL. One restricted-zone entry during
  SEARCH_QR, incomplete return traversal, and RTL over the corridor. No
  obstacle contacts, tilt events or geofence exits. Payload error 0.17 m;
  landing error 0.03 m. The rendered pacing defect remains open.
- Video: `logs/custom/my_world_offaxis_calm_s1.mp4`, 789.367 seconds,
  H.264 1280x720 at 30 fps; one continuous segment, measured wall/sim
  speed correction x3.597, fast-start remux, complete decode successful.
- The original live runner stops shortly after the mission result. Its
  grade therefore cannot assess the later RTL landing. A read-only observer
  held the recorder controller until disarm; the encoder and flight kept
  running. `*_full_track.csv` adds only observer samples after the original
  track's final timestamp; `*_full_grade.txt` grades that complete flight.
- Evidence: `logs/custom/my_world_offaxis_calm_s1_manifest.json`,
  `*_stack.log`, `*_layout.json`, `*_disarm.json`, `*_full_track.csv`,
  `*_full_grade.txt`, and `*_video_decode.log` (empty, decode exit 0).
