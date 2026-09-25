# Codebase assessment, 2026-09-09

## Continuation, 2026-09-11

The working tree also contains an unfinished supplied delivery-zone boundary
implementation, newer than the assessment below. The mission and GCS backend
accept boundary input and the search uses its local rectangle. The frontend
and launch scripts do not yet supply that required input. Finish that wiring
before attempting the next full mission run.

This continuation fixed a readiness mismatch in that implementation. A
regression showed four irregular geographic corners passing readiness even
though the mission rejects their geometry. Readiness now uses the same
boundary validator as the mission, waits for FCU home, and reevaluates when
either input changes. Boundary and home subscriptions request retained data
for late startup. Invalid replacement input revokes readiness. The shared
validator also rejects nonfinite home coordinates. The GCS package declares
its new dependency on mission_bt.

Validation in Ubuntu WSL with ROS Jazzy sourced and system Python:

- Boundary and readiness group: 45 passed, including the reproduced failure.
- GCS group: 43 tests blocked at import by missing `websockets`.
- Full offline suite: collection blocked by missing `py_trees`; the default
  environment also reports a NumPy 2 / cv_bridge binary incompatibility.
- Python syntax, shell syntax, XML/YAML parsing, and `git diff --check` passed.
- No live flight, frontend build or clean colcon build was performed in this
  continuation. Prior flight results below remain historical evidence.

Next work: make the WSL test environment reproducible, finish boundary input
in the GCS and simulation launcher, then record seed 1001 through delivery,
return and landing. Gate acceptance remains open. Existing edits were
preserved; no files were staged or committed.

The current stack implements the complete Mission 2 sequence. Its present
acceptance gap is a repeatable, recorded flight through delivery, return, and
landing on randomized seed 1001. The older README and progress handoff describe
earlier versions. They are useful history, but do not describe today's wiring.

## System map

| Area | Implementation | Current evidence and limits |
| --- | --- | --- |
| Simulation and transport | `scripts/launch_level6_sim.sh`, world and vehicle materializers, `mission_bringup`, MAVROS and MAVLink router | All 12 ROS packages build from a clean build directory. Seeded arena generation and the live harness are available. |
| Mission policy | `mission_tree.py`, `mav_commander.py` | The tree connects camera poses, start QR, banner alignment, both gate crossings, corridor traversal, zone search, winch, return and landing. Abort/result/reset paths have offline coverage. |
| Banner perception | `perception_banner`, `scan_geometry.py` | Camera identity and bearing select the lidar surface. Lidar measures perpendicularity and standoff. The pending deeper-wall regression now passes. |
| Gate transit | `AlignToBanner`, `DuckUnderBoard`, `GateAdvance` | Both legs descend below a measured edge. New behavior tests cover a gate at 5 m with an obstacle at 7.1 m, dynamic crossing distance, and off-center alignment. Live validation is in progress. |
| Corridor navigation | `avoidance/velocity_controller.py` | A separate velocity controller chooses gaps, controls yaw/altitude, observes entry before exit, and has bounded backoff recovery. The shortened gate crossing hands control to this node before the first obstacle. |
| QR and search | `perception_qr`, `search_planner.py`, `decode_hover.py`, search/decode stages | QR plausibility, target matching, exclusion routing and expanding search bands are implemented. Run 20 reached target A with the 60 m search budget. |
| Red ground and fences | `perception_redzone/georef.py`, `leg_router.py`, `geofence.py` | Camera observations become exclusions used by routed legs. Fence upload/readback is implemented. Whole-flight exclusion compliance still needs evaluation from the recorded track. |
| Payload | `WinchDrop`, `winch_ctrl` | Run 20 released the simulated payload. Drop position is latched when the stage begins, without a matched-target centering stage beforehand. This is an accuracy gap. |
| Return and landing | `ReturnToCorridorMouth`, return alignment/duck/corridor, `PrecisionDescent`, `Land` | Full sequence exists. Seed 1001 has not yet demonstrated it through landing. Precision descent may explicitly degrade to an ordinary landing, so `COMPLETED` alone is not proof of precision. |
| GCS | `gcs_aggregator`, readiness nodes, Tauri/React app | Telemetry, command acknowledgments, readiness checks, scan ledger and overlay are implemented. Frontend type checking and production build pass. Some intermediate tree stages still publish `IDLE`, limiting operator visibility. |

## Confirmed defects addressed in this continuation

1. Deeper corridor walls were selected as gate posts. The inherited geometry
   patch selects the nearest pair at similar depth. All 42 geometry tests pass.
2. Requiring a clear 10 m leg rejected a traversable return gate. The duck now
   measures the gate along the intended flight direction and publishes a
   crossing endpoint 1 m beyond it, with 0.75 m clearance before the next
   obstacle. The configured advance remains an upper limit. Both tree legs use
   the measured distance and altitude, with a 0.25 m arrival tolerance.
3. Exhausting yaw corrections could fall through to alignment success despite
   a visible off-center banner. The stage now makes a bounded lateral
   correction. A regression reproduced success at bearing 0.52 with tolerance
   0.10 before the fix.
4. Delivery fallback telemetry called the sighting altitude the release
   altitude. Its age counter also reset between winch phases, allowing an old
   descent sighting to appear fresh. Regressions reproduce both defects. The
   corrected report distinguishes the two altitudes and keeps sighting age
   continuous across phase changes.
5. `GateAdvance` recorded its endpoint as the corridor exit. The actual exit
   could not replace that first-write-wins value. Run 21's initial search
   bounds were x 6.7..20.9, overlapping both corridors, as the operator
   reported. Only `Corridor` now records the exit. Run 22 measured initial
   search bounds x 18.9..31.7 at the far corridor exit.
6. The return stage could accept the outbound `corridor_exited` flag while
   the navigator was still OBSERVING. A regression reproduced immediate
   return-stage success. The commander now requires a CRUISE observation.

The delivery telemetry corrections were made after run 21's mission process
started. Run 21 tests the gate changes and retains the earlier delivery
reporting code. Its saved startup patch identifies that version.

## Remaining concerns

- A decoded target is not necessarily centered. Delivery needs a measured
  target-centering and hold condition, with vehicle lag and lost-lock coverage.
- The high-altitude search currently exits on `qr_matched`. The general claim
  that it finds undecodable candidate markers at altitude is stronger than this
  call path proves; QR offsets also come from successful decodes.
- `ReturnToCorridorMouth` still derives its camera standoff from a default
  banner-center height of 3.38 m. This is an arena-height assumption, unlike the
  new gate-edge measurement.
- Gate clearance certifies a forward strip. Exclusion detours require separate
  care because a detoured route need not stay inside that measured strip.
- Real-image corpus tests remain skipped. Simulation success does not complete
  the real-camera or hardware qualification work.

## Verification

- `python3 -m pytest sim/test_scan_geometry.py -q`: 42 passed.
- Gate, alignment and routing group after the fix: 325 passed.
- `scripts/run_tests.sh` before run 21: 870 passed, 3 skipped; syntax,
  XML/YAML and frontend checks passed.
- Clean colcon build: 12 packages built.
- Delivery/reporting group after the additional corrections: 188 passed.
- Run 21: outbound alignment, duck and crossing passed; stopped during search
  after the operator reported the incorrect search location. Recorded track
  confirms the initial lanes overlapped the corridors.
- Run 22 preflight: 873 tests passed, 3 skipped, all offline checks passed,
  12 packages built in a fresh build directory.
- Run 22: outbound duck and crossing passed; corrected search anchor observed
  live. Return and landing outcome pending.

The gate issue remains claimed. No files were staged or committed.
