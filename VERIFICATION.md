# VERIFICATION LOG

Append-only record of what has actually been proven, and how.

**Rules for this file**
1. One section per phase (see `PHASE_PLAN.md`), one entry per claim.
2. Every entry states: **Claim → Method → Raw output → Verdict**.
3. A verdict is `VERIFIED`, `PARTIAL`, `BLOCKED` or `FAILED`. Nothing else.
4. `VERIFIED` requires evidence a third party could re-run. "It looked right" is not a method.
5. Entries are never edited to look better. A `FAILED` entry stays; a later `VERIFIED` entry supersedes it and says so.

**Why this file exists.** `CURRENT_PROGRESS_HANDOFF.md` records a stack with 13 passing tests and a mission that timed out to success while sitting disarmed at a 54° tilt. Passing tests were not evidence because nothing tied a test to the behaviour it claimed to cover. Every entry here names the specific defect it closes.

---

## Phase 0 — Test harness and minimal fail-closed rails

**Date:** 2026-08-15
**Scope:** evidence tooling, launcher idempotency, geometry audit, and the four minimal fail-closed rails. Perception, avoidance and mission logic are explicitly *not* in scope — those are P1–P9.

---

### 0.1 — Test non-vacuity (meta-verification)

**Claim:** The Phase 0 tests fail when the defect they describe is reintroduced, so a pass is meaningful.

**Method:** Mutation testing. Each rail's implementation was reverted to its pre-Phase-0 behaviour in place, the corresponding test selection was run, and the source was restored from backup.

**Raw output:**

```text
###### MUTATION 1: remove the disarmed-setpoint gate (restore old behaviour)
FAILED sim/test_phase0_rails.py::TestSetpointGate::test_disarmed_setpoints_are_suppressed
FAILED sim/test_phase0_rails.py::TestSetpointGate::test_gate_closes_again_on_disarm_mid_flight
================== 2 failed, 2 passed, 18 deselected in 0.65s ==================

###### MUTATION 2: reset does not invalidate the tree (old memory-sequence bug)
FAILED sim/test_phase0_rails.py::TestTreeReset::test_restart_after_reset_is_deterministic
FAILED sim/test_phase0_rails.py::TestTreeReset::test_tree_advances_then_resets_to_waiting
================== 2 failed, 2 passed, 18 deselected in 0.65s ==================

###### MUTATION 3: guard ignores attitude (old guard)
FAILED sim/test_phase0_rails.py::TestAttitudeAbort::test_guard_trips_on_excessive_attitude
================== 1 failed, 4 passed, 17 deselected in 0.64s ==================

###### RESTORED — full suite
35 passed in 0.83s
```

**Verdict:** `VERIFIED`. Each mutation failed exactly the tests scoped to it and no others. This entry is a precondition for trusting 0.2–0.5 below.

---

### 0.2 — Rail: no setpoints to a disarmed aircraft

**Claim:** `Mav` never publishes to `/mavros/setpoint_position/local` while the FCU reports disarmed.

**Defect closed:** handoff §"Mission logic defects" — *"The mission may continue publishing setpoints while disarmed."*

**Method:** `sim/test_phase0_rails.py::TestSetpointGate`, 4 tests, against a **real** `rclpy` node and the **real** `Mav` object (not `MockMav`), with `pub_sp.publish` replaced by a recorder. Covers: disarmed suppression, armed pass-through, mid-flight disarm re-closing the gate, and no-setpoint-requested.

**Raw output:**

```text
TestSetpointGate::test_armed_setpoints_flow PASSED
TestSetpointGate::test_disarmed_setpoints_are_suppressed PASSED
TestSetpointGate::test_gate_closes_again_on_disarm_mid_flight PASSED
TestSetpointGate::test_no_setpoint_when_none_requested PASSED
```

**Verdict:** `VERIFIED` at unit level. Live-stack confirmation in 0.6.

---

### 0.3 — Rail: external intervention resets the mission

**Claim:** An external DISARM, or an uncommanded mode change into LAND/RTL, raises a mission-reset request; a landing we commanded ourselves does not.

**Defect closed:** handoff §"Current runtime state" — the tree reported `GOTO_CORRIDOR` while disarmed on the ground; and §"Mission logic defects" — *"External LAND/DISARM does not reset the behaviour tree."*

**Method:** `sim/test_phase0_rails.py::TestExternalIntervention`, 6 tests on the real `Mav`. Distinguishes our own commanded mode changes (5 s ownership window) from outside intervention, and asserts `consume_reset()` clears `mission_started`, `abort_requested` and the stale setpoint, and cannot fire twice.

**Raw output:**

```text
TestExternalIntervention::test_commanded_landing_disarm_is_not_an_intervention PASSED
TestExternalIntervention::test_consume_reset_clears_mission_state PASSED
TestExternalIntervention::test_external_disarm_requests_reset PASSED
TestExternalIntervention::test_external_mode_change_requests_reset PASSED
TestExternalIntervention::test_idle_aircraft_does_not_reset PASSED
TestExternalIntervention::test_our_own_rtl_is_not_an_intervention PASSED
TestExternalIntervention::test_new_start_clears_stale_expect_disarm PASSED
TestExternalIntervention::test_new_start_clears_stale_attitude_violations PASSED
```

**Verdict:** `VERIFIED` at unit level.

---

### 0.3b — Defect found in the Phase 0 work itself: stale state leaked across runs

**Claim:** State latched during one mission must not survive into the next.

**How it surfaced:** review of the Phase 0 code, not a test failure. The `Land` leaf sets `expect_disarm` so its own touchdown is not misread as an intervention — but nothing cleared it. After one completed mission the watchdog would have been **permanently deaf to external disarms** for every subsequent run: exactly the defect Phase 0 exists to close, reintroduced by its own fix. The same leak applied to `abort_reason` and the attitude violation counter.

**Fix:** `Mav._on_start()` now clears `_expect_disarm`, `abort_reason` and `_attitude_violations` alongside the already-cleared result latch.

**Method:** two new tests, mutation-checked by removing the clearing lines.

**Raw output (mutation — clearing lines removed):**

```text
FAILED sim/test_phase0_rails.py::TestExternalIntervention::test_new_start_clears_stale_attitude_violations
FAILED sim/test_phase0_rails.py::TestExternalIntervention::test_new_start_clears_stale_expect_disarm
2 failed, 22 deselected in 1.73s
```

**Raw output (restored):**

```text
37 passed in 2.71s
```

**Verdict:** `VERIFIED` fixed.

---

### 0.4 — Rail: the tree actually returns to WAITING (and restart is deterministic)

**Claim:** After an intervention the py_trees memory `Sequence` does not resume mid-mission; the next tick is at `WaitForMissionStart`, and a fresh START begins at `SetModeArm`.

**Defect closed:** handoff §"Mission logic defects" — *"Abort/start restart semantics are incomplete because the memory sequence can retain progress."*

**Method:** `sim/test_phase0_rails.py::TestTreeReset`, 4 tests. Builds the **real** root via `build_root()`, ticks it into `Takeoff`, injects an external disarm through `Mav._on_state`, applies `apply_pending_reset()`, and asserts the tip. Also asserts the latched `INTERRUPTED` result and that a no-reset tick does not disturb a healthy mission.

**Raw output:**

```text
TestTreeReset::test_no_reset_means_no_interference PASSED
TestTreeReset::test_reset_latches_interrupted_result PASSED
TestTreeReset::test_restart_after_reset_is_deterministic PASSED
TestTreeReset::test_tree_advances_then_resets_to_waiting PASSED
```

**Verdict:** `VERIFIED` at unit level.

---

### 0.5 — Rail: excessive-attitude abort, and latched mission outcome

**Claim (a):** A sustained roll/pitch beyond the limit trips the abort guard with a stated reason; a single noisy sample does not.
**Claim (b):** `/mission/result` latches exactly one terminal outcome per run as JSON with an explicit reason.

**Defect closed:** handoff §"Live evidence of current failures" — the aircraft sat at RPY `[-53.786, 5.925, -177.882]` against a corridor wall and no part of the stack objected; and §"Mission logic defects" — *"Completion and failure states are not robustly latched/reported."*

**Method:** `sim/test_phase0_rails.py::TestAttitudeAbort` (5 tests) and `::TestMissionResult` (3 tests). The attitude tests replay the **actual recorded failure attitude** (−53.8° roll, 5.9° pitch) through the real quaternion→RPY path and assert both the recovered angles and that `CheckAbortTriggered` trips.

**Raw output:**

```text
TestAttitudeAbort::test_guard_trips_on_excessive_attitude PASSED
TestAttitudeAbort::test_level_flight_is_not_excessive PASSED
TestAttitudeAbort::test_roll_pitch_recovered_from_quaternion PASSED
TestAttitudeAbort::test_single_noisy_sample_does_not_trip PASSED
TestAttitudeAbort::test_sustained_tilt_trips_after_n_samples PASSED
TestMissionResult::test_new_start_clears_previous_result PASSED
TestMissionResult::test_result_is_latched_first_writer_wins PASSED
TestMissionResult::test_result_payload_is_json_with_reason PASSED
```

**Verdict:** `VERIFIED` at unit level.

---

### 0.6 — LIVE stack run: all four rails against real SITL

**Claim:** Every Phase 0 rail holds against the real stack — real Gazebo Harmonic, real ArduPilot SITL, real MAVROS, real behaviour tree — not just against unit-test doubles.

**Method:** `sim/verify_phase0_live.py`. Scripted reproduction of the exact scenario from `CURRENT_PROGRESS_HANDOFF.md`: start the mission from the GCS topic, let it arm and climb, then **force-disarm from outside the behaviour tree** the way Mission Planner or the safety pilot would, and assert the tree fails closed.

Force-disarm uses `MAV_CMD_COMPONENT_ARM_DISARM` with `param2 = 21196`. An ordinary `CommandBool(false)` is **rejected by ArduPilot while airborne** — the first attempt at this test reported `armed=True` for exactly that reason.

**Raw output:**

```text
======================================================================
 PHASE 0 LIVE RAIL VERIFICATION
======================================================================

[0] Stack connectivity
  [PASS] MAVROS reports FCU connected  (mode=STABILIZE)
  [PASS] mission_bt is publishing /mission/state  (state=WAITING)

[1] Pre-start state
  [PASS] tree is parked at WAITING before START  (state=WAITING)

[2] Setpoint gate while disarmed and idle (6s)
  [PASS] no setpoints published while disarmed (idle)  (0 setpoints observed)

[3] Mission start
  [PASS] tree leaves WAITING on START  (state=ARMING)
  [PASS] aircraft armed  (armed=True mode=GUIDED)
  [PASS] aircraft climbing under mission control  (alt=1.56 m state=TAKEOFF)
      stage before intervention: TAKEOFF

[4] External DISARM (simulating Mission Planner / safety pilot)
  [PASS] aircraft disarmed from outside the tree  (armed=False)

[5] Fail-closed reset
  [PASS] tree returned to WAITING after intervention  (state=WAITING (was TAKEOFF))
  [PASS] tree is NOT stuck at a stale stage  (state=WAITING)
  [PASS] /mission/result latched an outcome
         ({"state": "INTERRUPTED", "reason": "external disarm", "t": 60.8})
  [PASS] result carries an explicit reason  (state=INTERRUPTED reason=external disarm)

[6] Setpoint gate after intervention (6s)
  [PASS] no setpoints published to the disarmed aircraft  (0 setpoints observed)

======================================================================
 PHASE 0 LIVE: 13/13 checks passed
======================================================================
```

**Direct comparison with the handoff's recorded failure:**

| Handoff (before) | This run (after) |
|---|---|
| `Armed: false`, `Mission state: GOTO_CORRIDOR` | `armed=False`, `state=WAITING` |
| no failure reason anywhere | `INTERRUPTED — external disarm` |
| mission kept publishing setpoints while disarmed | 0 setpoints observed while disarmed |

**Verdict:** `VERIFIED`.

---

### 0.6b — Rail: abort latches (defect found by the live run)

**Claim:** Once an abort fires, the mission must not resume.

**How it surfaced:** the **first** live run, not a unit test. `/mission/result` latched:

```text
{"state": "ABORTED_RTL", "reason": "Excessive attitude (roll=-17.8 pitch=46.3)", "t": 80.2}
```

The attitude rail correctly fired during takeoff and commanded RTL. But the tree then reported `TAKEOFF` again. Every guard condition is **level-triggered** — attitude recovers once RTL levels the aircraft, battery voltage recovers under reduced load, the FCU reconnects — so the guard released and the memory `Sequence` resumed its previous leg **while the aircraft was already flying itself home**.

**Fix:** `Mav.abort_latched`. `CheckAbortTriggered` returns SUCCESS unconditionally once latched, and the latch clears only on an explicit new START or a mission reset. The first reason wins, so the latched cause is never overwritten by a later symptom.

**Method:** two new unit tests plus mutation check.

**Raw output (mutation — latch check removed):**

```text
FAILED sim/test_phase0_rails.py::TestAttitudeAbort::test_abort_latch_clears_only_on_new_start
FAILED sim/test_phase0_rails.py::TestAttitudeAbort::test_abort_latches_and_does_not_resume
2 failed, 2 passed, 22 deselected in 1.98s
```

**Verdict:** `VERIFIED` fixed. This defect was invisible to unit testing and only appeared under real flight dynamics — it is the strongest argument for the live-evidence half of the per-phase evidence pack.

---

### 0.7 — Regression: pre-existing suite still passes

**Claim:** Phase 0 changes do not break the existing behaviour-tree and aggregator tests.

**Method:** Full offline suite via the new `scripts/run_tests.sh`. `MockMav` was extended with the new commander API (`attitude_excessive`, `expect_disarm`, `publish_result`, `consume_reset`) — the three initial failures were the mock lagging the interface, not a behaviour regression.

**Raw output:**

```text
=== Python unit tests ===
...................................                                      [100%]
35 passed in 0.93s
  -> ok
=== Python syntax (all mission/perception/GCS/sim sources) ===   -> ok
=== Shell syntax ===                                             -> ok
=== XML / YAML well-formedness ===                               -> ok
====================== ALL OFFLINE CHECKS PASSED =====================
```

**Verdict:** `VERIFIED`.

---

### 0.9 — Defect found and fixed: the test suite was poisoning its own imports

**Claim:** Test results were order-dependent. Both pre-existing test files installed a `MagicMock` in place of `mavros_msgs` for the entire pytest session, so any test file collected after them silently received mocks instead of real ROS messages.

**How it surfaced:** `sim/test_phase0_rails.py` passed when invoked directly but **22 of its 35 tests failed** under `pytest sim/`. pytest collects alphabetically, so `test_behavior_tree.py` imported first and poisoned `sys.modules`.

**Root cause:** the guard was

```python
if 'mavros_msgs' not in sys.modules:   # WRONG
```

On a machine that *has* `mavros_msgs`, the module is simply not imported *yet* — so this condition is true and the mock gets installed regardless. The intent was "mock only if unavailable"; the test performed was "mock unless already imported".

**Fix:** replaced with a real availability check in both files:

```python
try:
    import mavros_msgs.msg
    import mavros_msgs.srv
except ImportError:
    ...install mocks...
```

**Method of verification:** ran the suite in three collection orders.

**Raw output:**

```text
-- directory collection --      35 passed in 0.85s
-- reversed order --            35 passed in 0.85s
-- behaviour tree first --      29 passed in 0.84s   (subset: 2 files only)
```

**Verdict:** `VERIFIED` fixed. **Significance:** this is the same class of defect as the handoff's central complaint — a green suite that was not testing what it appeared to test. Any earlier claim resting on `pytest sim/` output predating this fix should be treated as unproven.

---

### 0.8 — Environment: the simulation overlay was missing

**Claim:** The ArduPilot/Gazebo overlay this project depends on had been destroyed, and the launcher's dependency check correctly refused to run without it.

**Method:** `scripts/preflight_stack.sh` after the previous session's stack died.

**Raw output:**

```text
PASS  ROS 2 Jazzy command
PASS  Gazebo Harmonic command
FAIL  MAVROS package
FAIL  ros_gz bridge package
FAIL  slam_toolbox package
FAIL  web_video_server package
FAIL  ArduPilot SITL launcher (sim_vehicle.py)
FAIL  ArduPilot ROS/Gazebo bringup package
FAIL  ArduPilot Gazebo model/plugin package
```

**Root cause:** the overlay had been built into `/tmp/ardupilot_stack2`. `/tmp` was cleared, taking `sim_vehicle.py`, `ardupilot_gazebo` and `ardupilot_sitl` with it. The same event killed the process group (`176131`) that `CURRENT_PROGRESS_HANDOFF.md` reported as still running.

**Note on this output:** the four MAVROS/ros_gz/slam_toolbox/web_video_server rows are a **false alarm in the check script itself** — those packages are present in `/opt/ros/jazzy`, but `preflight_stack.sh` never sources `/opt/ros/jazzy/setup.bash`, only the overlay. Confirmed present:

```text
mavros                   /opt/ros/jazzy
ros_gz_bridge            /opt/ros/jazzy
slam_toolbox             /opt/ros/jazzy
web_video_server         /opt/ros/jazzy
```

Only the three ArduPilot rows were real failures.

**Verdict:** `VERIFIED` as a diagnosis. Two follow-ups, both done: (a) the overlay is rebuilt persistently via the new `scripts/install_ardupilot_overlay.sh` into `~/aerothon_stack`; (b) `preflight_stack.sh` now sources the ROS distro before checking distro packages, removing the four false failures.

---

### 0.10 — Overlay rebuild: three upstream build blockers and how each was resolved

**Claim:** The ArduPilot/Gazebo overlay can be rebuilt from scratch on this machine **without root**, reproducibly.

**Method:** iterative `scripts/install_ardupilot_overlay.sh` runs, each failure diagnosed and encoded back into the script so the next machine does not hit it.

| # | Blocker | Diagnosis | Resolution |
|---|---|---|---|
| 1 | `Could not checkout ref 'Copter-4.6'` | There is no `Copter-4.6` branch upstream; newest stable is `Copter-4.5`. `goal.md` says "4.5/4.6". | Pinned `Copter-4.5`. |
| 2 | `ardupilot_gazebo`: `gstreamer-1.0`, `gstreamer-app-1.0` not found | Only the GStreamer *runtime* libs are installed; `-dev` needs root. Used by exactly one target, `GstCameraPlugin` (RTP video streaming), which this project does not use — camera frames reach ROS via the gz camera sensor + `ros_gz` bridge. | `scripts/patch_ardupilot_gazebo_gst.py` makes GStreamer optional and guards that one target. Applied only when GStreamer is absent. |
| 3 | `ardupilot_sitl`: `microxrceddsgen` not found | `Tools/ros2/ardupilot_sitl/CMakeLists.txt` hardcodes `--enable-dds`. The generator's bundled Gradle 7.6 rejects JDK 21 ("class file major version 65"); bumping to Gradle 8.5 then fails in the vendored `IDL-Parser` submodule (`classifier` property removed in Gradle 8). Installing JDK 17 needs root. | `scripts/patch_ardupilot_sitl_dds.py` removes `--enable-dds`. AP_DDS is unused: the locked architecture reaches the FCU over MAVLink/MAVROS. Applied only when `microxrceddsgen` is absent. |

**Also fixed in the installer along the way:** `vcs import` runs three times with backoff (a parallel clone was refused by GitHub once), and its success is no longer trusted on exit code alone — the script now verifies `sim_vehicle.py`, `ardupilot_gazebo/CMakeLists.txt` and `modules/mavlink` actually exist, because `vcs` reports success even when an individual repo failed to check out its ref.

**Both patches are:** idempotent, backed up to `*.aerothon-orig`, applied **only** when the proper dependency is missing, and documented in-file with the exact `sudo apt` command to get the unpatched build instead.

**Standing caveat:** two upstream source files in `~/aerothon_stack` are locally modified. They are outside this repository, and re-running `vcs import` may revert them — the installer re-applies both on every run, so re-running it is the correct recovery.

**Raw output (final run):**

```text
Summary: 7 packages finished [2min 32s]
PASS  sim_vehicle.py on PATH
PASS  arducopter SITL binary built
PASS  ardupilot_gazebo package
PASS  ardupilot_gz_bringup package
PASS  ardupilot_sitl package
 Overlay installed at: /home/sarthak/aerothon_stack
```

**Verdict:** `VERIFIED`. `scripts/preflight_stack.sh` subsequently reports 9/9 PASS.

---

### 0.11 — Live stack brought up: three launch defects fixed

**Claim:** `scripts/launch_level6_sim.sh` brings up a stack in which MAVROS actually reaches the flight controller.

Three defects blocked this; none were in the handoff.

**(a) The reaper killed its own launch.** The straggler pattern included `launch_level6_sim`, which matches this script, any shell wrapping it, and any process whose command line merely mentions it. The first run killed its own parent. Fixed twice over: stragglers are now matched by `ps` **excluding our own process group**, and the pattern no longer contains `launch_level6_sim` at all (previous launcher instances are handled by the recorded PGID).

**(b) ArduCopter was crash-looping on DDS parameters.**

```text
PANIC: Failed to load defaults from .../copter.parm,.../gazebo-iris-gimbal.parm,
       .../dds_udp.parm,.../dds_use_ns.parm
Running: sh dumpstack.sh 29609 ... Failed
```

Upstream `iris.launch.py` defaults the `defaults` argument to a list including `dds_udp.parm` and `dds_use_ns.parm`. Those name AP_DDS parameters, and since 0.10(3) built the firmware without AP_DDS, ArduPilot panicked on unknown parameters. Symptom at the top of the stack was simply `connected: false` with no error. Fixed **without patching upstream** — `sim_full.launch.py` now passes an explicit `defaults` omitting the DDS files, which is correct whether or not DDS is compiled in.

**(c) Nothing was feeding the MAVLink router.**

```text
Router status: 53 pkts routed | 0 FCU, 1 MAVROS, 0 GCS endpoints active.
```

The launcher started `mav_router.py --fcu-in 14560`, but `ardupilot_gz`'s `robot.launch.py` computes `mavlink_out = 14550 + port_offset` and hands it to MAVProxy, so with instance 0 the stream lands on `127.0.0.1:14550`. The router's *GCS* port was also 14550, so MAVProxy's stream arrived on the wrong side of the router. Fixed: router FCU input is now 14550, its GCS listen moved to 14552, and its fan-out to 14553. MAVProxy's hardcoded second `--out 127.0.0.1:14551` remains the Mission Planner endpoint for SITL, and the launcher now prints that correctly.

**Raw output after all three fixes:**

```text
connected: true armed: false mode: STABILIZE
```

**Verdict:** `VERIFIED`.

---

### 0.12 — Evidence pack captured

**Claim:** `scripts/capture_evidence.sh` produces a reviewable pack from the live stack.

**Raw output:**

```text
[1/6] Graph inventory captured.
[2/6] Topic census captured (24 topics).
[3/6] Recording 25 topics for 20s...
[3/6] rosbag written.
subscribing to: ['/camera/image', '/percep/qr/annotated']
/camera/image: saved camera_image.jpg (480, 640, 3)
/percep/qr/annotated: saved percep_qr_annotated.jpg (480, 640, 3)
[4/6] Camera frames saved: 2.
[5/6] TF tree unavailable (tf2_tools missing or no transforms).
[6/6] Screenshot failed.
```

```text
Bag size:   273.5 MiB      Messages: 2663      Duration: 18.6s
Storage:    mcap           Distro:   jazzy
```

Pack: `evidence/P0/20260815-202023-rails-live/`

**Defect found and fixed in the tool itself:** the first capture reported "no image topics present" while `/camera/image` was publishing at 5 Hz. ROS 2 discovery is asynchronous, and the script called `get_topic_names_and_types()` immediately after node construction. It now spins until discovery completes.

**Second defect in the tool, found while auditing the pack:** the script reported `[5/6] TF tree unavailable` — but the pack in fact contains `frames_2026-08-15_20.22.20.pdf` and its `.gv` source. `tf2_tools view_frames` writes a **timestamped** filename; the script checked for a literal `frames.pdf`. The TF tree was captured correctly all along and the tool was lying about its own output. Now globbed with `find -name 'frames*.pdf'`.

**Pack contents (verified on disk, 274 MB):**

```text
camera_image.jpg              percep_qr_annotated.jpg
frames_2026-08-15_20.22.20.pdf / .gv     graph_inventory.txt
manifest.txt                  topic_census.txt      topic_list.txt
rosbag/ (mcap, 2663 msgs)     rosbag_record.log     frame_capture.log
```

**Known degraded, not fixed:** desktop screenshots fail in a non-graphical shell. Reported honestly rather than silently skipped; they work when run from a desktop session.

**Verdict:** `VERIFIED` for rosbag, census, inventory, camera frames, TF tree and manifest. Screenshot capture remains environment-dependent.

---

### 0.13 — Defects found live that Phase 0 does NOT fix

These were discovered while proving the rails. They are recorded here so they are not rediscovered later as surprises, and are **out of Phase 0 scope** by design.

**(a) The aircraft cannot hold a stable climb.** The attitude rail aborted a takeoff at `roll=-17.8 pitch=46.3` at 1.5 m altitude. Pose is published at only ~1.5–2.8 Hz (see (c)), so the 5-sample debounce corresponds to roughly **1.8 seconds sustained** beyond 45° — this is a real sustained tilt, not a sensor spike or a twitchy threshold. It is the same signature as the handoff's recorded `RPY [-53.786, 5.925, -177.882]`. Suspected cause: `scripts/materialize_vehicle_model.py` adds lidar and camera links to the Iris, changing mass and inertia. **Owner: Phase 2** (before any camera-pointing work, per the plan's "stabilize frames and basic flight before autonomy").

**(b) The mission runs while the aircraft is on the ground.** A later run reached `CORRIDOR_NAV` with the aircraft at `x=9.28, y=1.47, z=0.117` — 11 cm altitude, armed, in GUIDED, crawling along the ground while the tree reported corridor navigation. Altitude is never a success condition for any stage. **Owner: Phase 5** (corridor traversal), with the general fix in Phase 10 (every transition needs a sensor/flight success condition).

**(c) Telemetry stream rates are far too low.** Measured live:

```text
/mavros/local_position/pose      average rate: 1.504 – 2.776
/scan                            average rate: 6.773
/camera/image                    average rate: 4.910
```

`goal.md` Q14 specifies 10 FPS for QR and Q27 requires LiDAR ≥ 8 Hz. Position at ~2 Hz is not adequate for closed-loop guidance, and `/scan` at 6.8 Hz already fails the stated interlock. Stream rates must be requested explicitly from ArduPilot (`SR*` parameters / `MAV_CMD_SET_MESSAGE_INTERVAL`). **Owner: Phase 2**, since every later perception loop's timing budget depends on it.

**(d) Pose topic takes 30–60 s to appear** after MAVROS connects, with no indication anywhere. Belongs in the Phase 10 readiness interlock as an explicit "waiting for EKF origin" state rather than an unexplained silence.

**Verdict:** `FAILED` as flight behaviour — recorded, assigned, and deliberately not patched in Phase 0.

---

## Phase 2 — Flight stability, telemetry rates, and camera pointing

**Date:** 2026-08-16
**Scope:** the two blockers Phase 0 handed forward (0.13a instability, 0.13c stream rates), then the phase's own subject: camera orientation as commanded, confirmed state.

---

### 2.1 — CORRECTION: the aircraft is stable. Phase 0's attribution was wrong.

**Phase 0 claimed** (0.13a): *"Aircraft cannot hold a stable climb... Suspected cause: `scripts/materialize_vehicle_model.py` adds lidar and camera links to the Iris, changing mass and inertia."*

**That suspicion is refuted.** It was a hypothesis stated without a controlled test, and the controlled test disagrees.

**Method:** `sim/hover_test.py` — MAVROS only. No behaviour tree, no perception, no avoidance. GUIDED → arm → takeoff 5 m → hold 20 s → land, recording attitude throughout.

**Raw output:**

```text
phase            n  dur(s)  alt min  alt max  |roll|max  |pitch|max
connect          1     0.0    -0.04    -0.04        0.1         0.1
arm              4     1.5    -0.04    -0.04        0.1         0.1
takeoff         25     9.2    -0.04     4.63        0.2         2.2
hover           60    19.5     4.79     5.04        0.2         0.4
land            50    17.1     0.06     5.00        0.3         0.4

samples over 25 deg: 0  over 45 deg: 0
worst attitude: 2.2 deg
VERDICT: STABLE (worst 2.2 deg vs 45 deg limit)
```

Position hold was within 5 cm of the takeoff point across the 20 s hover, and the aircraft landed and disarmed normally.

**Verdict:** `VERIFIED` — the airframe, mass distribution and ArduPilot tune are fine. The 0.235 kg of added sensors shift the CG about 1 cm forward, which the attitude controller trims out without difficulty. The instability came from somewhere else — see 2.3.

---

### 2.2 — Telemetry stream rates: two independent causes

**Claim:** `/mavros/local_position/pose` at 1.5–2.8 Hz (Phase 0, 0.13c) is not one problem but two, with different fixes.

**Cause A — MAVLink rates were never requested.** In SITL, MAVProxy owns SERIAL0 and requests its own modest default stream rates; our router relays whatever arrives, so MAVROS inherits them. `MAV_CMD_SET_MESSAGE_INTERVAL` (511) fixes this.

```text
before: /mavros/local_position/pose ~ 1.50 Hz
9/9 intervals accepted
after:  /mavros/local_position/pose ~ 5.67 Hz     (GUI + RViz running)
after:  /mavros/local_position/pose ~ 29.26 Hz    (headless)
change: 3.8x / 19x
```

**But the rate does not stick.** Measured decay after a single application:

```text
t+10s: average rate: 26.690
t+20s: average rate: 2.184
t+30s: average rate: 2.199
t+40s: average rate: 2.093
t+50s: average rate: 1.875
```

MAVProxy periodically re-requests its own lower rates on the same channel. `ardupilot_gz`'s `robot.launch.py` includes `sitl_mavproxy.launch.py` unconditionally, so MAVProxy cannot be switched off without reworking the upstream vehicle-spawn path. `mission_bringup/stream_rate_keeper.py` therefore re-asserts the intervals whenever the observed rate drops below target. **This is a simulation-topology workaround, not a flight feature** — the deployed architecture (goal.md Q22/Q28) runs `mav_router` straight to the Pixhawk with no MAVProxy anywhere, so the first application simply sticks. Dropping MAVProxy from the sim is the clean fix and is now tracked for Phase 11.

**Cause B — the simulator runs at 0.55 real time.** `/scan` and `/camera/image` do not come from MAVLink at all; they come from Gazebo through `ros_gz`. Measured real-time factor:

```text
real_time_factor: 0.55006014907730161
real_time_factor: 0.58491948875697009
```

`/scan` is configured at 10 Hz and measured 5.53 Hz — exactly 10 × 0.55. **The sensors are publishing correctly in simulation time; the simulator is running slow.** This is a compute-budget issue, not a code defect, and it does not exist on the real Pi where sensors run in real time.

Reducing that load is worth doing anyway, and exposed a launch defect (2.5b). Headless measurements:

| | GUI + RViz | headless |
|---|---|---|
| `/camera/image` | 4.96 Hz | 7.79 Hz |
| `/scan` | 2.95 Hz | 5.53 Hz |

**Verdict:** `PARTIAL`. Position rate is solved for guidance purposes (29 Hz). Sensor rates remain bounded by simulator speed; `goal.md` Q27's "LiDAR ≥ 8 Hz" is an interlock on the **real aircraft**, and asserting it against a 0.55-RTF simulation would be measuring the wrong thing. Phase 11 should either raise RTF or evaluate that interlock in simulation time.

---

### 2.3 — ROOT CAUSE of the instability: an inverted body-frame sign

**Claim:** The aircraft was flown into the ground by the avoidance controller, because lateral velocity was commanded in the wrong direction.

**Method:** with stable flight established in 2.1, the mission was re-run and the difference isolated.

**Raw output — mission timeline:**

```text
Mission state -> TAKEOFF
Mission state -> START_QR
Mission state -> GOTO_CORRIDOR
Mission state -> CORRIDOR_NAV
Mission result: ABORTED_LAND (Excessive attitude (roll=23.0 pitch=45.7))
final position: x=8.489  y=1.522  z=0.028      <-- on the ground
```

The corridor spans x = 5 → 15.5; the aircraft crashed 3.5 m in.

**The defect:** `velocity_controller.py` published body-frame velocity on `/mavros/setpoint_raw/local` using MAVLink's FRD convention, which its own header documented as *"x forward, y right, z down"*. But MAVROS's `setpoint_raw` plugin takes body-frame setpoints in **ROS FLU** and runs `transform_frame_baselink_aircraft` (FLU→FRD) itself. On that topic, positive y is **LEFT**.

So the centring law `k_center * (right - left)` steered the drone *into* the wall it was avoiding. A sign error like this is invisible in a symmetric corridor and catastrophic in an asymmetric one.

**Fix:** `center_err = left - right`, with the frame convention documented at the top of the file in the terms MAVROS actually uses.

**Mutation check** (restore `right - left`):

```text
FAILED sim/test_frame_conventions.py::FrameConventionTests::test_room_on_the_left_commands_positive_y
FAILED sim/test_frame_conventions.py::FrameConventionTests::test_room_on_the_right_commands_negative_y
2 failed, 9 passed
```

**Live result after the fix** — the same mission, same world:

```text
TAKEOFF          x=0.05   y=0.01   z=3.16
START_QR         x=0.03   y=0.02   z=4.99
CORRIDOR_NAV     x=8.78   y=-0.01  z=2.93
CORRIDOR_NAV     x=16.29  y=0.32   z=2.96     <-- corridor cleared
ENTER_ZONE       x=18.22  y=-1.16  z=9.98
SEARCH_QR        x=45.77  y=-12.10 z=10.00
```

The corridor is now traversed end to end while holding 2.93–2.96 m, the aircraft climbs to the search altitude, and the lawnmower runs across the delivery zone.

`sim/test_frame_conventions.py` (11 tests) is the standing guard — this is `CURRENT_PROGRESS_HANDOFF.md` repair step 2, *"Verify ENU/NED and body-frame signs with automated assertions."*

**Verdict:** `VERIFIED`. Note the scope boundary: the *sign* is fixed and the corridor is flyable. The controller's *algorithm* — pass-side selection, recovery when boxed in, exit detection from perception — remains Phase 5.

---

### 2.4 — Camera pointing as a commanded, CONFIRMED state

**Claim:** camera orientation is measured state, and no perception stage can run while the camera is somewhere other than where the mission believes it to be.

**Defect closed:** the handoff's root cause for the start-QR failure — *"the mission holds at (0,0,5) but does not command the camera downward... the default camera remains forward-facing, so the ground QR is generally not visible."*

**Built:** `camera_ctrl` — named poses (`FORWARD` 0°, `NADIR` −90°, `ALIGN` −45°, per goal.md Q18), commanded to the Gazebo joint controller or via `MAV_CMD_DO_MOUNT_CONTROL` on hardware, **read back from `/joint_states`**, and published as JSON on `/camera/pose_state`. `settled` requires N consecutive in-tolerance samples *held* for a minimum time. A `SetCameraPose` behaviour gates each perception stage and **fails closed** on timeout.

Joint limit widened from −1.570796 to −1.65 rad: a position controller asked to hold exactly at its own hard stop settles slightly short, so a −90° NADIR command could never read back inside tolerance.

**Unit tests:** 20, mutation-checked twice.

```text
=== MUTATION 1: settled true as soon as a command is sent ===
8 failed, 12 passed
=== MUTATION 2: new request does not clear previous settle ===
1 failed, 19 passed
=== restored ===
20 passed
```

**Verdict:** `VERIFIED` at unit level; live results in 2.6.

---

### 2.5 — Two launch defects found in passing

**(a) The `camera_backend` Python conditional.** My own first edit wrote `'backend': 'sim' if use_sim else 'mavlink'`. `use_sim` is a `LaunchConfiguration` **object**, which is always truthy, so this would have silently selected the Gazebo backend on the real aircraft — the camera would never have moved in flight. Replaced with an explicit `camera_backend` launch argument.

**(b) `rviz:=false` did not disable RViz.** `sim_full.launch.py` declared and read the `rviz` argument but never applied it to `rviz_node`, so RViz always launched — ~20–30% CPU on a simulator already at 0.55 real-time factor, dragging every sensor rate down. Fixed with `condition=IfCondition(rviz)`.

**Verdict:** `VERIFIED` fixed. (a) is worth noting as a class: launch-file conditionals that look like Python but are substitutions fail silently and only on the configuration you test least.

---

### 2.6 — LIVE verification: camera pointing against real SITL

**Method:** `sim/verify_phase2_live.py` against the running Gazebo + ArduPilot SITL + MAVROS stack.

**Raw output:**

```text
======================================================================
 PHASE 2 LIVE VERIFICATION — camera pointing + telemetry rates
======================================================================

[0] camera_ctrl is publishing state
  [PASS] /camera/pose_state is published

[1] Named poses are commanded and CONFIRMED at the joint
  [PASS] NADIR: joint CONFIRMED within 8s     (settled in 0.02s, joint=-89.7 deg)
  [PASS] NADIR: measured angle matches -90 deg (-89.67 deg)
  [PASS] FORWARD: joint CONFIRMED within 8s   (settled in 1.26s, joint=-0.5 deg)
  [PASS] FORWARD: measured angle matches +0 deg (-0.52 deg)
  [PASS] ALIGN: joint CONFIRMED within 8s     (settled in 0.39s, joint=-45.2 deg)
  [PASS] ALIGN: measured angle matches -45 deg (-45.15 deg)
  [PASS] NADIR: joint CONFIRMED within 8s     (settled in 0.52s, joint=-89.5 deg)
  [PASS] NADIR: measured angle matches -90 deg (-89.45 deg)

[2] settled is a MEASUREMENT, not an assumption
  [PASS] a joint dragged off target reports settled=false  (error=67.0 deg)
  [PASS] camera_ctrl re-asserts until the joint is measured back on target
         (error_deg=0.56)

[3] Telemetry rates support closed-loop guidance
  [PASS] MAV_CMD_SET_MESSAGE_INTERVAL accepted
  [PASS] /mavros/local_position/pose >= 10 Hz immediately after applying (11.8 Hz)
       /scan          5.4 Hz
       /camera/image  7.5 Hz

[3b] Known SITL limitation: MAVProxy re-requests lower rates
       pose rate ~40s later: 2.1 Hz (decayed)

======================================================================
 PHASE 2 LIVE: 13/13 checks passed
======================================================================
```

Check [2] is the important one. The verifier drives the joint away behind `camera_ctrl`'s back — publishing straight to the Gazebo position controller while `camera_ctrl` still believes it commanded NADIR. `settled` goes false with a 67° error, then recovers as `camera_ctrl` re-asserts. That is the difference between *knowing* where the camera points and *assuming* it.

**Two defects found in the verifier itself and fixed:**
- The first NADIR command was lost: the verifier published before `camera_ctrl` had matched the subscription. Same asynchronous-discovery race that swallowed `/mission/start` in Phase 0 and that made the evidence tool report "no image topics". It now waits for `get_subscription_count() > 0`. **Third occurrence of this class this project** — worth a standing habit, not three separate fixes.
- The rate check originally measured minutes after rates were applied, so it was measuring MAVProxy's decay rather than the fix. It now applies immediately before measuring, and reports the decay separately as [3b].

**Verdict:** `VERIFIED`.

---

### 2.7 — Evidence pack

```text
[1/6] Graph inventory captured.
[2/6] Topic census captured (26 topics).
[3/6] Recording 28 topics for 20s...  rosbag written.
[4/6] Camera frames saved: 1.
[5/6] TF tree saved (frames_2026-08-16_17.21.46.pdf).
[6/6] Screenshot failed.
```

Pack: `evidence/P2/20260816-171940-camera-and-rates/`

The TF tree is captured this time — the filename-glob fix from 0.12 working as intended. Screenshots remain unavailable in a non-graphical shell.

**Verdict:** `VERIFIED`.

---

### 2.8 — Carried forward from Phase 2

**(a) `stream_rate_keeper` is off by default.** Two consecutive launches carrying it ended with `mavros_node` and `robot_state_publisher` aborting (SIGABRT) within a minute; the same stack without it ran for many minutes, and the run after disabling it recorded **zero** process deaths. The causal link is a correlation across two runs, not a proven mechanism — but a simulation-only workaround does not belong in the default flight stack on suspicion. Enable with `stream_rate_keeper:=true` if wanted. → **P11 removes MAVProxy instead.**

**(b) Search-exhausted restarts the mission silently.** With the corridor now flyable, the mission reaches `SEARCH_QR`, sweeps the zone, finds no match (the QR pads are still untextured boxes — Phase 1), and `LawnmowerSearch` returns FAILURE. The root Selector then has no viable child, and on the next tick the memory Sequence restarts from the top: observed state going `SEARCH_QR → START_QR` mid-flight. A failed mission must latch a failure, not loop. → **P6/P10**

**(c) Simulator real-time factor 0.55.** Bounds every Gazebo-side sensor rate. → **P11**

---

## Phase 1 — Sim asset realism and the QR decode envelope

**Date:** 2026-08-16

---

### 1.1 — CORRECTION to Phase 2: the camera was pointing at the sky

**Phase 2 claimed** (2.4, 2.6): camera pointing verified, `NADIR` confirmed at −89.5°, 13/13 live checks passed.

**The joint angle was right. The camera was looking up.**

**How it surfaced:** the Phase 1 decode sweep returned **zero decodes at every altitude**, including 12.3 px/module at 3 m where decoding is trivial. Following the Phase 0 rule — look at the image before theorising — the captured nadir frame was **uniform pale blue**. Sky.

**Root cause:** the `webcam_pitch_joint` axis was `<xyz>0 1 0</xyz>`. Rotating about +Y maps the camera's forward vector to `(cos t, 0, −sin t)` in FLU, so with a +Y axis a **positive** angle looks down. But the joint limits (`lower=-1.65, upper=+0.52`), `goal.md` Q18 and MAVLink `DO_MOUNT_CONTROL` all use the aerospace convention where pitch-down is **negative**. The model could look 30° *down* and 94° *up*, and every `NADIR` command aimed at the sky.

**Fix:** axis `<xyz>0 -1 0</xyz>` in both `materialize_vehicle_model.py` and `uav.urdf.xacro`.

**Why Phase 2 missed it — the lesson.** Phase 2's whole purpose was to stop the mission assuming where the camera pointed. It replaced *"we sent a command"* with *"the joint reports that angle"*. That is one step short: the meaningful property is the **view direction in the world**, and a joint angle only implies it if the axis convention is right. I verified the number, not the consequence.

**Guard added:** `test_NADIR_actually_points_at_the_ground` parses the axis and limits out of both the SDF generator and the URDF, applies the rotation, and asserts the resulting forward vector has strongly negative Z. Plus `test_sdf_and_urdf_axes_agree`, because a mismatch would make RViz show a different camera orientation from the one being flown.

**Mutation check** (restore the +Y axis):

```text
FAILED sim/test_camera_ctrl.py::CameraCtrlTests::test_NADIR_actually_points_at_the_ground
FAILED sim/test_camera_ctrl.py::CameraCtrlTests::test_sdf_and_urdf_axes_agree
2 failed, 21 deselected
```

**Verdict:** `VERIFIED` fixed. Phase 2's entry 2.4/2.6 stands as to the *servo control loop*; its claim to have made camera pointing trustworthy did not, and is superseded by this entry.

---

### 1.2 — The sim QR needs no textures

**Claim:** the existing box-geometry QR rendering is fully decodable; the planned texture work is unnecessary.

`CURRENT_PROGRESS_HANDOFF.md` recorded QR pads as *"rendered as box geometry because texture rendering was unreliable"* and `PHASE_PLAN.md` Phase 1a therefore budgeted for fixing the OGRE2/PBR material path. **That work is not needed.** Once the camera was aimed correctly, the geometry rendering decoded at 100% at 3 m and 5 m.

Captured frame at 5 m nadir shows a crisp, high-contrast, correctly-proportioned QR against the pad.

**Verdict:** `VERIFIED`. Phase 1a's texture task is closed as **not required** — the "unreliable rendering" diagnosis in the handoff was a misattribution of the camera aiming bug.

---

### 1.3 — Decode envelope measured

**Method:** `sim/measure_qr_decode.py` — fly to station above the start pad, camera confirmed NADIR, 10 frames decoded per altitude with the mission's own `cv2.QRCodeDetector`.

**Raw output** (`docs/qr_decode_envelope.csv`):

```text
 alt(m)  px/module   decoded    rate  marker px  payload ok
    3.0      12.32   10/10     100%      289.7          yes
    5.0       7.39   10/10     100%      181.9          yes
    7.0       5.28    9/10      90%      132.7          yes
   10.0       3.70    3/10      30%       95.3          yes

 off-axis at 7 m:
   0.00 m (0.00 half-FOV)   9/10   90%
   1.01 m (0.25 half-FOV)  10/10  100%
   2.02 m (0.50 half-FOV)   0/10    0%
   2.83 m (0.70 half-FOV)   0/10    0%
```

**Reliable floor: ≈5.3 px/module.** Decoding is reliable only within roughly the inner quarter of the half-FOV — a lane plan that merely gets the marker *in frame* is not sufficient, it must pass near frame centre.

Analysis, projections to real marker sizes and camera resolutions, and the Phase 6 recommendation are in **`docs/QR_DECODE_ENVELOPE.md`**.

**The headline consequence:** `search_alt = 10.0` (`docs/GEOMETRY_AUDIT.md` A2) is unachievable for any plausible competition marker. At the simulated 640×480 a 0.4 m marker is readable only to 1.3 m; even at 4K it is 5.4 m. A single-altitude lawnmower that must both cover the zone and decode the payload is over-constrained — Phase 6 should sweep high to find candidate *pads* and descend to decode.

**Also identified:** the simulated camera is 640×480 while `goal.md` Q14 specifies a 1080p stream, so every Gazebo perception result is pessimistic by 3–6×. Raising the sim resolution costs real-time factor, which is already 0.55.

**Verdict:** `VERIFIED` as a measurement. A2 now has a measured basis instead of a guess.

---

### 1.4 — Real-image corpus: harness ready, photographs outstanding

**Status:** `BLOCKED` on physical capture — the one part of the plan that cannot be done from here.

Built:
- `tests/perception/test_real_corpus.py` — decode-rate by condition, banner detection by lighting, and the px/module envelope from real photographs. **Skips** cleanly while the corpus is empty (`3 skipped`), so it is safe in the tree before the shoot and cannot masquerade as a pass.
- `docs/CORPUS_SHOT_LIST.md` — exact shot list, naming convention, and a ~30-photograph minimum set that alone unblocks Phase 6.

The corpus is what decides whether the 5.3 px/module floor measured in simulation survives a real lens, real sensor noise and real sunlight. Expect it to be worse.

Note: `qrcode` is not installed here, so markers cannot be regenerated locally — the already-generated PNGs in `src/aerothon_sim/sim_gazebo/materials/` are committed and can be printed directly.

**Verdict:** `BLOCKED` — harness verified, data pending.

---

## Interlude — clearing the "still broken" list before Phase 3

**Date:** 2026-08-16
**Scope:** the eight known-broken items, fixed as a block on the user's instruction rather than left to the phases that nominally owned them. 101 offline tests, 3 skipped.

---

### I.1 — ScanStartQR fails closed

The handoff's headline defect. The leaf returned SUCCESS after a fixed tick count **with an empty target string**, so a mission that had never read its delivery target proceeded to deliver.

Now: SUCCESS only on a payload confirmed over K consecutive frames, or an explicit operator override (goal.md Q19, logged as OPERATOR); FAILURE on timeout with a stated reason. Consecutive-frame confidence matters because the mission commits to a delivery target on the strength of one read.

Mutation (restore timeout→SUCCESS): 2 failed, 3 passed. `VERIFIED`.

### I.2 — Altitude success conditions

A live run reported `CORRIDOR_NAV` while the aircraft dragged along the ground at z=0.117 m: horizontal progress was being made and no stage checked height.

Now: `Corridor` fails if altitude leaves its band; `Takeoff` arms an airborne floor and the abort guard trips on a sustained sag below it; `Land` clears the floor because descending is the point.

Mutation (remove the band check): 2 failed, 2 passed. `VERIFIED`.

### I.3 — Mission failure latches

`LawnmowerSearch` returning FAILURE made the root Selector fail, and py_trees restarted the memory Sequence from the top on the next tick — mid-flight, observed as `SEARCH_QR → START_QR`.

Now `latch_mission_failure()` publishes `FAILED` with a reason, clears `mission_started`, disables avoidance and parks the tree. A new START is required.

Mutation (remove the latch): 2 failed, 3 passed. `VERIFIED`.

### I.4 — The start QR no longer names target A by coincidence

`qr_start.png` and `qr_target_a.png` were generated with the **same payload**, and delivery pad A is also the first pad the lawnmower reaches. "The drone matched the target" was satisfied by a fixture coincidence; the search was never actually exercised.

The start pad now renders a **chosen** delivery target's matrix, defaulting to a random one per run (`AEROTHON_START_TARGET=c` to pin). This reuses the existing matrices rather than regenerating PNGs, because `qrcode` is not installed here.

```text
a        start pad names delivery target A (AEROTHON2026:M2:TARGET_A)
c        start pad names delivery target C (AEROTHON2026:M2:TARGET_C)
random   start pad names delivery target B (AEROTHON2026:M2:TARGET_B)
```

`VERIFIED`.

### I.5 — The winch exists

`/winch/cmd` had two publishers and **zero subscribers**; `WinchDrop` "delivered" after a fixed 20-tick timer. New `winch_ctrl` package: real command interface, state machine, payout integration, ground-contact trigger, `/winch/status` feedback, and release gated on payload-down **and** altitude window **and** hover stability **and** no fault — reporting *all* unmet blockers, not just the first. `WinchDrop` now waits on reported state and fails closed on timeout.

**Scope boundary, stated plainly:** the controller and interlocks are real and the same node drives `MAV_CMD_DO_WINCH` on hardware. There is **no Gazebo tether** — payout is integrated from commanded rate and ground contact inferred from altitude. This proves the sequence and the interlocks, not the mechanics.

14 tests. `VERIFIED` for the stated scope.

### I.6 — The GCS stops asserting things it never measured

`ekf: True` and `geofence: "INSIDE"` were hardcoded and never updated; satellite count initialised to 0 and stayed there. Now wired to `/mavros/estimator_status` (per-axis flags, healthy only when the states guidance depends on are all good) and `/mavros/global_position/raw/satellites`.

Geofence is the interesting one: MAVROS publishes the loaded fence **list** but not breach state — ArduPilot's `FENCE_STATUS` is not exposed — so `"INSIDE"` was never measurable at all. It now reports `NONE`/`LOADED`, and every safety field carries a staleness entry so a field that stops updating is visibly stale rather than frozen at its last confident value. Defaults are `UNKNOWN`, not `True`.

`VERIFIED` at code level; live GCS confirmation still owed.

### I.7 + I.8 — CORRECTION: the stack aborts were misdiagnosed three times

Recorded in full because the reasoning failure matters more than the fix.

`mavros_node` and `robot_state_publisher` intermittently died with SIGABRT during start-up. I attributed this to, in order:

1. **The stream-rate keeper** — disabled it. Wrong; the aborts also happen without it, and I had cleaned shared memory in the same step, confounding the comparison.
2. **Stale FastDDS shared memory** — added cleanup. Wrong; aborts recurred on a clean `/dev/shm`.
3. **Open DDS discovery range** — confined to localhost. Wrong; aborts recurred.

The log said what it was the whole time:

```text
[mavros_node] [INFO] signal_handler(SIGINT/SIGTERM)
[mavros_node] terminate called after throwing 'rclcpp::exceptions::RCLError'
  what(): failed to initialize rcl node: the given context is not valid,
          either rcl_init() was not called or rcl_shutdown() was called
```

The process was **SIGTERMed while still initialising** and then tried to finish constructing nodes on a dead context. The abort is a symptom of the stack being torn down mid-start-up — an environment/process-lifetime interaction — not a defect in any node.

**Consequences:**
- `stream_rate_keeper` is **exonerated and re-enabled by default**. Its own real bug (leaking one pending future per command per cycle, forever) is fixed separately.
- The shared-memory cleanup and localhost discovery are **kept** — both are good hygiene and the latter matches the locked architecture — but neither is now claimed as a fix, and the launcher comment says so.
- `scripts/wait_for_stack.sh` added: a post-launch health gate that names which part is missing. It has already earned itself by correctly reporting "launcher failed early" instead of leaving a measurement script to time out mysteriously.

**Verdict:** `VERIFIED` for the code changes; the live multi-minute rate-hold test is `BLOCKED` by the same process-lifetime behaviour, which is an environment property rather than a project defect.

**The lesson, third time this session:** I twice acted on a correlation without isolating a variable, and once changed two things at once. The log had the answer before any of the three hypotheses.

---

## Phase 3 — The QR loop closes

**Date:** 2026-08-17
**Scope:** the remainder of Phase 3 after the interlude had already made `ScanStartQR` fail closed with confidence gating and an operator override. 126 offline tests, 3 skipped.

---

### 3.1 — The offset was measured all along and never used

`mav_commander` subscribed `/percep/qr/target_offset` and **nothing read it**. Worse, `qr_node` only populated it when the decoded payload *matched the target* — and during the start scan there is no target yet, that being the point of the scan. So the offset was structurally guaranteed to be zero exactly when the mission needed it most.

`qr_node` now publishes the offset for the best-visible marker, with `z` encoding what it refers to: `1.0` the matched target, `0.5` some marker, `0.0` nothing. The commander exposes `qr_visible()` and `qr_centred(tol)`.

**Test-quality note.** A mutation restoring the "matched only" rule initially **passed all 20 tests**, because they drove a `FakeMav` with its own copy of the logic. That is a test measuring itself. Added `RealCommanderOffsetSemanticsTests` against the actual `Mav`; the mutation then failed 2 tests as it should.

`VERIFIED`.

### 3.2 — Decodes are checked for plausibility

Any decode was previously accepted at face value and could set the delivery target — a reflection, a QR on a phone screen, or a marker in a different part of the arena. `qr_node` now compares apparent marker width against what any physically plausible marker could subtend at the current altitude (`fx · S / h`), rejecting reads outside that band and reporting them on `/percep/qr/detail`.

The gate is a deliberately wide **range** (0.15–3.5 m, ×1.6 slack) because the competition marker size is unconfirmed. Narrow it once the organisers answer. Where altitude or intrinsics are unknown the gate **abstains** rather than rejecting — an unknown must not become a silent veto.

Mutation (accept everything): 2 failed. `VERIFIED`.

### 3.3 — The mission centres on the marker instead of hovering over a guess

`scan_pose` was hardcoded `(0,0,5)`, directly over the takeoff point, while the start pad sits about a metre away — roughly a third of the half-FOV at 5 m, outside the reliable decode zone Phase 1 measured. I predicted before running that the mission would now stop at `START_QR` because of it.

Replaced with a three-step ladder: `FindStartQR` (hold, and step down if nothing is visible, failing closed at a floor rather than descending into the ground) → `CenterOnQR` (drive the marker to frame centre) → `ScanStartQR` (hold *where centring left us*, not a constant).

**The sign convention is asserted, not eyeballed** — image +y is *down*, so a marker low in frame is *behind* the aircraft, and the correction is rotated by the aircraft's own yaw. This is precisely the class of error that flew the drone into a corridor wall in Phase 2, so there are five directional tests.

Mutation (invert the correction): 5 failed. `VERIFIED`. This closes geometry-audit item A4.

### 3.4 — A crash that only a live run could find

The first live attempt died immediately:

```text
File ".../qr_node.py", line 104, in _on_info
ValueError: The truth value of an array with more than one element is
            ambiguous. Use a.any() or a.all()
```

`CameraInfo.k` is a numpy array, so my `if m.k and m.k[0] > 0:` raised on the first camera-info message and killed the node. Every unit test passed because they set `_fx` directly and never exercised the callback.

Fixed to a length check, and a regression test now feeds a real `CameraInfo` with numpy intrinsics. Mutation (restore the truthiness test): 2 failed.

`VERIFIED`. Worth recording as the clearest justification this session for the live half of the evidence standard: 126 green unit tests and the node could not survive one real message.

### 3.5 — Live integration flight: BLOCKED, and precisely characterised

**Claim attempted:** the full mission runs end to end with the Phase 3 loop, the camera axis fix, the winch controller, the altitude conditions and the failure latch all active.

**Not achieved.** The stack is torn down 60–90 s after launch, every time, before the mission can be started. This is an environment behaviour, not a project defect, and it is now pinned down rather than guessed at.

**Evidence.** A launch with *no* polling, *no* pipes, and nothing else running:

```text
exit code  -2   x3   parameter_bridge, odom_tf, camera_ctrl_node   <- SIGINT
exit code  -6   x2   robot_state_publisher, mavros_node            <- SIGINT during init
exit code -15   x2   mavproxy, arducopter                          <- launcher cleanup, after
exit code   1   x4   python nodes, rclpy shutdown race after SIGINT
```

**SIGINT is delivered to the `ros2 launch` process group.** The launcher's own cleanup (`-15`) fires *afterwards*, as a consequence. ArduPilot was healthy throughout — the last messages before teardown are `EKF3 IMU0/IMU1 tilt alignment complete`, i.e. it was still coming up normally.

Approaches tried and eliminated: piping the launcher's output (a real self-inflicted bug — `| head -N` kills it via SIGPIPE, fixed); running the observer as a separate long call; running launch and observer in the same call; `setsid`; `nohup` + `disown`; launching as its own background task. All produce the same SIGINT.

**One self-inflicted bug found and fixed along the way:** `pkill -f "gz sim"` matches the *shell running it*, because the pattern string is in that shell's own command line. Several of my cleanup commands were killing the very call that was about to launch the stack. Now written `pkill -f "[g]z sim"`.

**How to run it yourself**, in a normal terminal where this does not occur:

```bash
AEROTHON_OFFICIAL_WS="$HOME/aerothon_stack" ./scripts/live_mission_test.sh --watch 240 --start-target c
```

It launches, waits for health, starts the mission, prints a state/position/QR table, and reports the latched outcome and every tree transition.

**Prediction to check against, recorded before the run:** with `scan_pose` now replaced by find→centre→decode, the mission should get *past* `START_QR` — which the old hardcoded hover would not have, since the marker sat at ~0.35 of the half-FOV. If it still fails there, the ladder's dwell or the centring tolerance is the thing to adjust, and `/percep/qr/detail` carries the offset and the plausibility band to say which.

**Verdict:** `BLOCKED` — environment, characterised, with a reproduction path for the user.

### 3.6 — CORRECTION: it was never the environment. Four wrong diagnoses.

The stack teardown was a **project defect**, found by reading a line I had been
scrolling past:

```text
[ERROR] [launch]: Caught exception in launch:
package 'mission_bringup' found at '.../install/mission_bringup',
but libexec directory '.../install/mission_bringup/lib/mission_bringup'
does not exist
```

`mission_bringup` had **no `setup.cfg`**. Every other package has one; it tells
setuptools to install console scripts into `lib/<pkg>` where `ros2 launch`
looks, rather than `bin/`. When I added `stream_rate_keeper` as the package's
first executable, the launch began referencing a node that could not be found,
`ros2 launch` threw, and **on that exception it SIGINTs every process it has
already started**. That is the SIGINT I attributed to the harness.

The full sequence of wrong answers, kept because the pattern matters more than
any one of them:

| # | Blamed | Why it looked right | Why it was wrong |
|---|---|---|---|
| 1 | `stream_rate_keeper` | aborts began when it was added | correct *correlation*, wrong mechanism — and I cleared shared memory in the same step, confounding the test |
| 2 | stale DDS shared memory | 86 stale segments present | aborts recurred on a clean `/dev/shm` |
| 3 | open DDS discovery range | machine has several interfaces | aborts recurred with discovery on localhost |
| 4 | "the environment SIGINTs us" | reproduced under every detachment method | the SIGINT came from `ros2 launch` itself |

Hypothesis 1 was closest and I talked myself out of it, because the *mechanism*
I imagined for it was wrong. The lesson is not "trust correlations" but: when a
correlation is strong and the mechanism is unclear, look for the mechanism
rather than discarding the correlation.

Also fixed: `pkill -f "gz sim"` matches the shell running it, since the pattern
is in that shell's own command line — several cleanup commands were killing the
call that was about to launch the stack. Now `pkill -f "[g]z sim"`.

**Verdict:** `VERIFIED` fixed. With `setup.cfg` in place: 22 processes started,
**0 deaths, 0 launch exceptions**.

---

## THE MISSION FLIES — first complete end-to-end run

**Date:** 2026-08-17

```text
latched /mission/result : {"state": "COMPLETED", "reason": "landed and disarmed"}
final position          : (-0.21, -0.02, 0.03)      <- home, on the ground
armed                   : False
elapsed                 : 183 s
```

| +t | stage |
|---:|---|
| 4.4 s | TAKEOFF |
| 15.0 s | CAMERA_NADIR |
| 20.3 s | CAMERA_FWD |
| 22.1 s | GOTO_CORRIDOR |
| 26.1 s | CORRIDOR_NAV |
| 47.4 s | ENTER_ZONE |
| 55.6 s | CAMERA_NADIR |
| 57.3 s | SEARCH_QR |
| 100.7 s | WINCH_DROP |
| 143.1 s | CAMERA_FWD |
| 144.8 s | RETURN |
| 161.3 s | CAMERA_NADIR |
| 163.1 s | LAND -> COMPLETED |

183 s is inside `goal.md` Q1's 5-8 minute budget.

Everything built across Phases 0-5 participated: fail-closed rails, camera pose
gating at five stages, the randomised start target (`TARGET_C`, decoded in
flight rather than matched by fixture coincidence), the winch state machine
(`LOWERING -> AT_GROUND -> STOWING -> RELEASED`, ground contact confirmed), the
altitude conditions, and the new corridor navigator.

**What made the difference on the fourth attempt:** the return corridor stalled
in runs 1-3. Three fixes, in order of discovery:

1. **Recovery.** The old controller stopped dead at `front=0.77 m` with
   symmetric walls and no lateral imbalance, forever. Replaced with
   follow-the-gap plus a CRUISE -> BLOCKED -> BACKOFF -> STUCK ladder.
2. **Travel along the gap.** My own first version still gated forward speed on
   *straight-ahead* clearance while steering toward an off-axis gap, so it slid
   sideways at 0.6 m/s and never advanced. Velocity is now decomposed along the
   gap bearing, plus a measured forward-progress watchdog — a gap you never
   travel through is still a stall.
3. **Perception-based exit.** `corridor_return_exit_x = 4.5` (audit A10) was
   simply unreachable; the aircraft crept to 4.63 and stopped. The navigator
   now reports the corridor has opened out when both walls fall away, which is
   what Phase 5 was always supposed to do. **Closes audit items A6 and A10.**

---

## Phase 4 — Banner identity and alignment

**Date:** 2026-08-17

### 4.1 — Green decoys added to the world first

`perception_banner` gated only on blob area and aspect, so any green rectangle
was "the banner" and the GCS displayed ALIGNED. With nothing else green in the
simulated world, an identity check that simply returned `True` would have
passed every test I could write — a test measuring itself, which has already
bitten this project twice.

So the decoys came first: a grass-like apron, a banner-shaped tarpaulin and a
green panel, placed at two locations away from the corridor mouth
(`green_decoy_visuals` in `materialize_world.py`). They are the simulator
stand-in for the physical decoy photographs that need a camera.

### 4.2 — Identity is structural, and says what it is not

The banner is a green board carrying white `AEROTHON` lettering inside a white
frame. Two cheap structural signatures a plain green object lacks: white
content occupying a characteristic fraction of the board, and **several
separate white components** in a horizontal band. Not OCR, and not claimed to
be.

`/percep/banner.z` now distinguishes three states that used to be two:

| z | meaning |
|---|---|
| 1.0 | identified banner |
| 0.5 | something green, rejected as not the banner |
| 0.0 | nothing green in view |

That middle value matters to an operator: "no banner in view" and "a green
thing in view that is not the banner" are very different situations.

Mutation (remove the identity gate): 5 tests failed. 17 tests total.

### 4.3 — Alignment replaces the hardcoded corridor entry

`AlignToBanner` yaws until the identified banner is centred and held for
several frames, and **fails closed** if no banner is identified during the
sweep. It acts only on `z=1.0`, so a green tarpaulin leaves it searching rather
than flying at the decoy.

This closes geometry audit **A5** (`corridor_entry = (5.0, 0.0, 3.0)`). Yaw
direction is asserted in both directions — the same class of sign error that
flew the aircraft into a wall in Phase 2.

**Live:** `BANNER_ALIGN` at +24.7 s, converged in ~11 s, mission completed.

**Verdict:** `VERIFIED`.

---

## Phase 6 — Search geometry derived from the camera

**Date:** 2026-08-17

### 6.1 — The constants disagreed with each other and with reality

`search_alt = 10.0` and `spacing = 6.0` (while `goal.md` Q9 says 5.0). Neither
derived from the camera; they contradicted each other; and Phase 1 measured
that 10 m is unachievable for any realistic marker.

`mission_bt/search_planner.py` computes, from camera width, HFOV, marker size,
module count and the **measured** 5.3 px/module floor:

* `decode_alt` — highest altitude the payload can still be decoded from
* `detect_alt` — highest altitude the pad is still a usable blob (much higher,
  which is what makes sweep-then-descend worth doing)
* `lane_spacing` — from swath width and an overlap factor
* `coverage` — the fraction of the zone actually swept

The planner is calibrated against the real measurements: its `px_per_module`
reproduces both Phase 1 data points (5.28 at 640×480/2.2 m, 5.04 at
1920×1080/0.5 m) to the measured precision, asserted in tests.

**Marker size is an INPUT.** The organisers' answer is still unknown, so the
plan is computed for whatever it turns out to be rather than baked in — the
unknown stopped being a blocker.

### 6.2 — Coverage is proven, and the proof can fail

`coverage_fraction` samples the zone and checks every point falls within half a
swath of some lane. A test deliberately feeds an over-wide spacing and asserts
coverage drops below 0.95, so the proof is capable of failing rather than
being decorative.

22 tests. Closes geometry audit **A2** and **B1**.

**Verdict:** `VERIFIED` for the geometry. Zone *extent* is still the hardcoded
rectangle (A8) and zone entry is still a waypoint (A7) — those need zone-
boundary perception and are not done.

---

## Standing summary — 2026-08-17

**The mission flies end to end, repeatably.** Three consecutive complete runs
with different randomised start targets:

| run | start target | result | elapsed |
|---|---|---|---|
| 1 | C | COMPLETED | 183 s |
| 2 | D | COMPLETED | 151 s |
| 3 | E (random) | COMPLETED | 152 s |

All within `goal.md` Q1's 5–8 minute budget. 189 offline tests, 3 skipped
(the real-image corpus, which needs a camera).

---

## §6b — Zone, return leg and decode altitude stop being constants (2026-08-17)

**Claim:** `zone_entry`, `zone_bounds`, `corridor_return_entry` and the banner
area gate are no longer asserted values, and the sweep no longer decodes at
the altitude it searches from.

**Method:** replacements plus mutation testing, then live flight.

| audit | was | now |
|---|---|---|
| A7 | `zone_entry = (18,0,3)` | `ObserveZone` — lidar depth/width at the corridor mouth |
| A8 | `zone_bounds = (20,52,-12,12)` | same, rotated by the observed exit heading |
| A9 | `corridor_return_entry = (15,0,3,pi)` | `ReturnToCorridorMouth` — the recorded exit, reversed and wrapped |
| E1 | `min_area_frac = 0.01` | derived from the banner's projected size at `max_detect_range_m` |
| — | sweep and decode at one altitude | `DescendToDecode` at the measured 5.3 px/module floor |

**Mutation testing** — every new claim was broken on purpose and the suite
caught it:

```
M1 return heading not wrapped     -> test_reverse_heading_is_wrapped_not_accumulated
M2 zone ignores exit heading      -> test_zone_rotates_with_the_exit_heading
M3 descend ignores the floor      -> test_never_descends_below_the_floor
M4 corridor exit never recorded   -> test_exit_pose_is_recorded_on_success
                                     test_the_return_trip_does_not_overwrite_the_mouth
```

**Verdict:** `VERIFIED` for A7, A8, A9, E1 and the descend-to-decode leg.

---

## §6c — What removing the constants EXPOSED (2026-08-17)

This is the important entry. Four defects had been sitting behind the
hardcoded waypoints, invisible because the constants were dragging the
aircraft to the right place regardless of what perception said.

**1. The corridor could be "exited" without ever being entered.**

`corridor_exited` fired on the open apron before the corridor. Live evidence:

```
Mission state -> CORRIDOR_NAV     (1786955804)
Mission state -> IDLE             (1786955805)   <- one second later
...
data: '{"state": "COMPLETED", "reason": "landed and disarmed", "t": 126.4}'
```

The aircraft never left x = 1.2 m. It "searched the delivery zone" on the
takeoff pad and reported success. `Goto("GotoZone", 18, 0, 3)` had been
flying it to the real zone and hiding this for every previous run.

Fix: entry must be observed before exit means anything.
Test: `test_open_ground_before_the_corridor_is_NOT_an_exit`.

**2. A completed mission re-armed and took off again.**

```
Mission result: COMPLETED (landed and disarmed)   (1786955890.896)
Mission state -> ARMING                           (1786955891.324)
Mission state -> TAKEOFF                          (1786955894.813)
```

`latch_mission_failure` parked FAILED missions; SUCCESS fell through, the
Selector returned SUCCESS, and py_trees re-initialised the memory Sequence on
the next tick. Fix: `latch_mission_success`.

**3. Aligning to the banner is not arriving at it.**

The gate stands at y = +2; `AlignToBanner` only yaws. Live result: the
aircraft flew forward from y = 0, missed the gate and wedged itself in a
corner —

```
{"state": "BLOCKED", "front_m": 0.21, "side_left_m": 0.46, "side_right_m": 0.31,
 "note": "no navigable gap"}        pos (4.94, -3.34, 3.07)
```

The deleted `corridor_entry` waypoint had been doing two jobs — heading AND
position — and only the heading half was replaced. Fix: `ApproachBanner`.

**4. The gate cannot be SEEN from the takeoff altitude.**

At 5 m with the gate 3 m ahead, the banner is entirely below the forward
camera's field of view. The aircraft swept 271 degrees looking for it, locked
onto a distant green object and flew away from the corridor. Two fixes: the
descent to corridor altitude now happens BEFORE the search, and the sweep is
bounded to half a turn so it cannot align to the return gate behind it.

**5. The banner is mounted on a green gate in front of a green corridor.**

They form one connected green blob many times the board's area, so white
lettering measured over the blob fell far below the 2% floor and the real
banner was rejected as "no white lettering". Fix: find the lettering band
first and derive the board from it. Rejections also now always state a reason
— a candidate dropped on area or aspect used to report `reason: ""`.

**Verdict:** all five `FIXED` with tests; live confirmation in progress.

---

## §4b — The banner detector was checking the wrong things (2026-08-17)

Once the hardcoded `GotoZone` waypoint stopped carrying the mission past this
stage, banner identification turned out never to have worked. Three separate
faults, each found by a live frame and none of which any synthetic test could
have caught.

**1. The aspect gate was applied to the wrong object.**

Live probe, camera pointed at the real gate:

```
{"identified": false, "reason": "green region aspect 17.40 outside 1.2-8.0",
 "candidates": 3}
```

The banner is mounted on a green gate in front of a green corridor. The green
mask returns all of that as one long thin region. Aspect is a claim about the
BOARD's proportions, so it now applies to the board derived from the lettering
band, not to whatever green happened to be connected.

**2. "White" lettering was an absolute brightness threshold.**

Captured frame `tests/fixtures/banner_sim_ambient_3m.png` — the banner fills
most of the image with AEROTHON plainly legible — was reported as:

```
{"identified": false, "reason": "only 0 white component(s); lettering expected",
 "candidates": 4, "white_frac": 0.0, "components": 0}
```

Under flat ambient light the lettering renders at roughly V=150, below the
fixed `white_v_min = 170`. Every synthetic fixture in `test_banner_identity.py`
drew its lettering at (245,245,245) and passed — **the fixtures could not fail,
because they were drawn by the same assumption the detector was making.**

The lettering is now found RELATIVE to the board it sits on: markedly brighter
and markedly less saturated than the green underneath. That is invariant under
studio light, ambient sim light, overcast and direct sun. The absolute
floor/ceiling remain only as a backstop.

Replaying the real frames through the fixed detector:

```
align_0.png  z=1.0  bearing=+0.069  components=8   white_frac=0.361
align_4.png  z=1.0  bearing=+0.207  components=10  white_frac=0.381
```

That frame is now checked in as a regression fixture, because synthetic images
demonstrably could not catch this class of fault.

**3. The search sweep counted commanded yaw, not actual yaw.**

`_swept` accumulated the per-tick yaw STEP, so at 10 Hz it "swept 180 degrees"
in about five seconds while the airframe had physically turned 23 degrees
(measured: yaw 360 -> -337 across the whole stage). The bound now measures the
actual heading change from the start of the stage.

This is the same assert-versus-measure error as the camera pointing in Phase 2
and the corridor exit in Phase 5, in a third place.

**Verdict:** all three `FIXED`; 43 banner tests including the rendered-frame
regression.

---

## §6d — First fully perception-driven end-to-end flight (2026-08-17, run 10)

With every arena coordinate removed and the banner detector fixed, the mission
ran the whole chain on perception alone:

```
ARMING -> TAKEOFF -> CAMERA_NADIR -> FindStartQR/CenterOnQR -> ScanStartQR
       -> CAMERA_FWD -> DescendToCorridorAlt -> BANNER_ALIGN
       -> ApproachBanner -> CORRIDOR_NAV (25 s) -> ObserveZone -> UploadFence
       -> SEARCH_QR -> DescendToDecode -> WINCH_DROP
       -> CAMERA_FWD -> ReturnToCorridorMouth -> RETURN
       -> CAMERA_NADIR -> PrecisionDescent
```

MAVROS confirmed the fence upload independently:

```
[mavros.geofence]: GF: mission sended        (1786959100)
```

**It did not complete.** `PrecisionDescent` held at 2.57 m for 601 ticks
without regaining pad lock and reported the whole mission FAILED — after the
payload had already been delivered.

That verdict was wrong, and the fix is not to weaken the check. Precision
landing is an ENHANCEMENT to landing: the only thing at stake once the payload
is down is whether touchdown is on the pad or merely near it. The stage now
degrades and says so, and the mission outcome carries it:

```
COMPLETED  "landed and disarmed; landing PRECISE (committed at 1.42 m, 0 re-acquisition(s))"
COMPLETED  "landed and disarmed; landing DEGRADED (precision lock not achieved by 2.57 m ...)"
```

"Landed" alone never said whether it landed on the pad.

**Observability defect found at the same time.** Every derived number the
mission flies on — the observed zone bounds, the sweep and decode altitudes,
the coverage fraction, the fence verification result — was written with
py_trees' `self.logger`, which nothing in this stack configures. They were
computed, used to fly the aircraft, and recorded nowhere. Run 10 produced no
evidence of any of them. They now go through the ROS node logger, which is
what the Phase 6 and Phase 7 evidence packs actually consist of.

---

## §R — Rulebook conformance: three real violations (2026-08-17, runs 11–12)

Asked to check the mission against the prescribed phase and altitude sequence,
not just against the task list. Three of the ten requirements were being
violated, and one more was found in flight afterwards.

| Rulebook requirement | Before | After |
|---|---|---|
| Take off → 5 m, scan start QR | ok | ok |
| **Identify banner + align BEFORE descending** | **descended first** | aligns at 5 m |
| 5 m → 3 m corridor navigation | ok | ok |
| 3 m → 10 m on exit, identify pad | ok | ok |
| 10 m → 5 m, lower and release | ok | ok |
| **Ascend to 10 m for the return lap** | **crossed at 3 m** | climbs first |
| **Detect the banner again (return lap)** | **never looked** | re-detects |
| Return through corridor, land at start | ok | ok |

**1. Order inversion at the banner.** The rulebook says identify and align
*before* descending to corridor altitude. The mission descended first — because
from 5 m a gate a few metres ahead is entirely below a level camera. The fix
was to point the camera where the banner is, not to move the aircraft: a
`BANNER` pose at −20° covers the gate from scan altitude, so the required order
is followed rather than worked around.

**2. The return lap was wrong twice.** "Corridor Entry Detection Return Lap" is
a separately scored task and the aircraft never performed it — it flew home on
the recorded corridor pose without looking for the gate again, and crossed the
delivery zone at 3 m instead of ascending to 10 m.

**3. `DescendToDecode` could command a CLIMB above the ceiling.** For the 2.2 m
pad the decode envelope computes to 13.9 m, and `max(want, floor)` commanded a
climb to 13.9 m — above the 10 m identification altitude — in order to
"descend" to read a marker the aircraft could already read.

Run 12 then flew the corrected sequence end to end:

```
TAKEOFF 4.9 m -> BANNER_ALIGN (5 m) -> corridor 3.0 m -> climb -> pad matched
-> WINCH_DROP 5.0 m -> climb 10.0 m -> BANNER_ALIGN 8.8 m -> return 3.0 m -> home
```

### §R.1 — A fourth violation the order tests could not see

Run 12 swept at **12.8 m** (MAVProxy: `height 15`). Every stage was in the
right order and every leaf-level altitude assertion passed, because the
violation was not in the behaviour tree at all:

```python
sweep_alt = detect_alt if max_altitude is None else min(detect_alt, max_altitude)
sweep_alt = max(sweep_alt, decode_alt)      # never sweep below decode alt
```

The second line silently undid the ceiling. `decode_alt` is 13.9 m for the
2.2 m pad, so `max(10.0, 13.9)` → 13.9. The optimisation ("don't fly lower than
you can already decode from") is sound; letting it override a regulatory
ceiling is not. The clamp now comes last and wins.

Worse, the *test suite asserted the bug*:

```python
def test_never_sweeps_below_decode_altitude(self):
    plan = plan_search(ZONE, 1920, HFOV, 0.5, MODULES, max_altitude=0.5)
    self.assertGreaterEqual(plan["sweep_alt_m"], plan["decode_alt_m"] - 1e-9)
```

Confirmed fixed in flight (run 13): `sweep 10.0 m, decode 13.9 m`.

---

## §7b — The search could only ever find one of the five pads (2026-08-17)

Prompted by "nothing should be hardcoded, everything from perception", the
arena geometry was compared against what the aircraft actually observes.

| | x range | y range |
|---|---|---|
| Real delivery zone | 12 → 52 | −15 → +15 |
| Observed by run 12 | 16.5 → 27.9 | −8.1 → +6.9 |

Pads sit at A(21,10) B(47,10) **C(23,1)** D(33,−10) E(45,−6); red zones at
(38,5) (29,10) (40,−11).

**Only target C falls inside the observed window, and every live run to date
had been launched with `--start-target c`.** A search that could only ever find
one of five pads had looked like a working search for eleven runs. All three
red zones are outside the window too — which is why every run logged
`avoiding 0 red zone(s)` and Phase 7 had never actually been exercised.

The cause is not a bug but a category error: `ObserveZone` bounds the zone with
the lidar, and **the lidar's range is not the zone's size**. It reports 12 m of
open ground for a zone 40 m deep. The window is a *frontier*, not the zone.

`LawnmowerSearch` now advances that frontier: when a strip is swept without a
match, it plans the next strip beyond it and continues, until the target is
found or the search budget (`search_budget_m`, endurance — a property of the
aircraft, not of the arena) is spent. The geofence is uploaded around the whole
search envelope rather than the first window, so an expansion cannot breach it.

### §7b.1 — Gating the step on the lidar was wrong, and only flight showed it

The first implementation asked the lidar, at each expansion, how much open
ground lay ahead. It reads as the more principled choice. Run 13 never advanced
once and failed with:

```
swept [16.3, 27.8, -8.0, 6.8] at 10.0 m over 1 strip(s) without matching the
target; no open ground left ahead and 45 m of search budget unused
```

The horizontal lidar is on the airframe. At the corridor mouth, 3 m up between
two walls, its forward reading is a real measurement. At sweep altitude, 10 m
over open ground, it is above everything — `/avoidance/detail` reported
`open_depth_m: 0.32` with nothing at all in front of the aircraft. Probed
directly to confirm rather than inferred:

```
avoidance/detail: {"state": "OBSERVING", "front_m": 0.32, "side_left_m": 12.0,
                   "side_right_m": 12.0, "open_depth_m": 0.32, "open_width_m": 24.0}
```

The step is now the depth of the window measured **where the lidar could
see**, repeated outward. Still an observation — a different opening gives a
different step — but taken from the one place the sensor is meaningful.

### §7b.2 — Phase 7 worked the moment the search reached the red zones

Run 13, same flight, first live evidence of georeferenced red-zone avoidance:

```
red zone confirmed mid-sweep (0 -> 5):  re-planned the current strip, 2 lanes, coverage 97%
red zone confirmed mid-sweep (5 -> 26): re-planned the current strip, 2 lanes, coverage 99%
red zone confirmed mid-sweep (26 -> 32): re-planned the current strip, 2 lanes, coverage 99%
```

Exclusions were sampled **once**, before the sweep, from the corridor exit —
where none of the red zones are visible. Anything confirmed later was recorded
and then ignored. They are now re-read during the sweep and the current strip
is re-planned around them.

The georeferencer confirms red ground cell by cell, so the count climbs while
sweeping. Since each re-plan restarts the strip, that is bounded
(`max_replans_per_strip`) — otherwise a steadily growing exclusion set stalls
the sweep on lane 0 indefinitely. Verified by a test that feeds it an exclusion
set which grows on every read.

Spot-check of the georeferenced cells against the world file: the NW red zone
is at (29,10), 6×4 m, spanning x 26..32. Reported cells include
`[26.0, 29.0, ...]`, `[29.0, 32.0, ...]` — x extent correct.

---

## §10b — The GCS shows the interlock, not just its verdict (2026-08-17)

The aggregator subscribed to `/mission_ready` and `/percep/redzone` — the
*Bool* topics — so the eleven-item interlock and the red-zone tri-state built
in Phase 7/10 reached the operator as two booleans. It now consumes
`/mission_ready/detail` and `/percep/redzone/detail`.

The panel rendered `!redzone_visible` as **"CLEAR"**, so a detector that had
never published, or one that could not see the ground at all, read as safe —
the same defect as §I.6, in new code. `redzone_status` starts `UNKNOWN` and
`NOT_VISIBLE` renders as "NO GROUND VIEW", distinct from "CLEAR".

Each interlock row shows its measured value and its reason, so a blocked ARM
button says which of the eleven checks is holding and what it read. Waived
items stay visible as waived.

### §7b.3 — Run 14: a pad that was unreachable for eleven runs

Same arena, same code, `--start-target e`. Target E is at (45, −6); the
observed window ended at x = 28.0.

```
search plan over observed zone [16.5, 28.0, -8.2, 6.9] avoiding 0 red zone(s):
    sweep 10.0 m, decode 13.9 m, spacing 8.1 m, 2 lanes, coverage 100%,
    frontier budget 45 m
red zone confirmed mid-sweep (0 -> 10):  re-planned the current strip, coverage 98%
red zone confirmed mid-sweep (10 -> 29): re-planned the current strip, coverage 99%
red zone confirmed mid-sweep (29 -> 31): re-planned the current strip, coverage 97%
frontier advance 1: stepping 12.0 m (the observed opening)
    -> sweeping [28.5, 40.0, -8.6, 6.5] avoiding 42 red zone(s), 33 m budget left
red zone confirmed mid-sweep (42 -> 62): re-planned the current strip, 1 lanes, coverage 71%
frontier advance 2: stepping 12.0 m (the observed opening)
    -> sweeping [40.5, 52.0, -9.0, 6.1] avoiding 69 red zone(s), 21 m budget left
Mission state -> WINCH_DROP
```

Flown positions, which is what actually settles it:

```
 t(s)  state         position
   88  SEARCH_QR     [28.2 -4.6 10.0]     original window
   96  SEARCH_QR     [32.4 -4.8 10.0]     first advance
  104  (advancing)   [40.1 -4.8 10.0]     second advance
  112  WINCH_DROP    [46.1 -5.1  5.2]     over target E (45, -6)
```

Two further things this run confirms:

* **Target discrimination.** At t 64/72/88 the QR node reports
  `"accepted": "AEROTHON2026:M2:TARGET_C", "matched": false` — the aircraft
  read pad C, recognised it was not the assigned target, and kept sweeping.
  Previous runs never exercised this, because C *was* the target every time.
* **Sweep altitude held at 10.0 m** across all three strips, and the drop
  happened at 5.2 m. Both rulebook altitudes.

Outcome, latched by the tree:

```
Mission result: COMPLETED (landed and disarmed; landing DEGRADED
    (precision lock not achieved by 2.64 m after 601 ticks; landing normally))
```

The geofence was uploaded over the *extended* search envelope rather than the
first window — otherwise the first frontier advance would have breached it —
and confirmed by read-back:

```
[mavros.geofence]: GF: mission sended
fence verified: 4 items, 0 exclusion(s)
```

Coverage of the second strip fell to 71% once 62 exclusions were confirmed.
That is the honest number for ground the aircraft may not overfly, and it is
reported rather than smoothed — but it is worth noting that a lower coverage
figure is exactly what routing around red zones costs.

---

## §11b — The randomised arena had never run (2026-08-17)

Phase 11's premise is that moving the arena distinguishes "derived from
perception" from "derived from a different constant that happens to agree".
The generator was written, documented as working, and recorded in the phase
plan. Checked before committing to a regression run:

```
File "scripts/materialize_world.py", line 243, in main
    arena = randomise_arena(world, random.Random(args.seed or None))
NameError: name 'randomise_arena' is not defined
```

`if __name__ == "__main__": main()` sat immediately after `main()` — above the
definitions of `randomise_arena()` and `_set_pose()` — so those names were
unbound when `main()` ran. The default path never calls them, and every live
run had used the default path, so it was silent.

**A regression harness that cannot vary the arena reports five passes on five
identical arenas.** `sim/test_arena_randomisation.py` now runs the script as
the launcher does (through the environment) and checks that pads, red zones
and the gate's *heading* all actually move, that two seeds differ, that a seed
is reproducible so a failing arena can be re-flown, and — guarding the rest —
that the pose parser locates every model it claims to compare.

Verified by reintroducing the ordering: 2 failures, 7 errors.

---

## §9b — The precision descent was chasing an altitude the camera cannot reach

Runs 12 and 14 ended identically: descend to ~3 m, lose the pad, climb to
re-acquire, oscillate, time out, degrade, land normally. Run 14's tail:

```
 t(s)  position                 t(s)  position
  184  [0.6 0.1 4.6]             296  [0.7 0.2 3.7]
  192  [0.8 0.1 4.0]             304  [0.7 0.1 3.7]
  200  [0.9 0.2 3.8]             312  [0.7 0.1 3.2]   <- timeout, degraded
  208  [0.8 0.2 4.1]             320  [0.7 0.1 0.1]   <- landed
```

The controller was not misbehaving. `land_commit_alt` was a flat **1.5 m**,
and the marker does not fit in the frame anywhere near that:

```
vfov 1280x720 @ 60 deg hfov = 36.0 deg
2.2 m pad tracking floor    = 3.90 m
```

At 1280x720 the *vertical* field of view is only 36°, so the 2.2 m pad fills
the short frame dimension at 3.90 m. **The oscillation band observed in flight
was 3.4–4.1 m.** The derived floor lands inside the measured band without any
tuning, which is the strongest evidence available that the model is the right
one and the constant was simply wrong.

`min_track_altitude()` is the floor of the tracking envelope, the counterpart
to `max_decode_altitude()`: too high and the modules are too small to read, too
low and the marker is not in the picture. `PrecisionDescent` now raises its
commit altitude to it, and `land_commit_alt` is a floor rather than a target —
a smaller marker still commits at 1.5 m.

### §9b.1 — Confirmed live on a randomised arena

Arena regression seed 1004 (clean run, single harness):

```
Mission result: COMPLETED (landed and disarmed;
    landing PRECISE (committed at 3.86 m, 0 re-acquisition(s)))
```

`committed at 3.86 m` is the derived tracking floor (3.90 m for the 2.2 m pad
at 1280x720) doing precisely what it was added for. The same descent with
`land_commit_alt = 1.5` oscillated in the 3.4-4.1 m band for 601 ticks and
degraded. **Zero re-acquisitions** — it never lost the pad, because it was no
longer being asked to descend below where the pad fits in frame.

This is also the first randomised arena to complete end to end, and it did so
on a layout the stack had never seen.

**The honest limitation this exposes.** With this camera and a 2.2 m marker the
useful precision-descent window is 3.90–5.0 m, which is narrow. The aircraft
centres as low as the sensor allows and then commits; residual error is
whatever it drifts over the final 3.9 m. Run 14 touched down 0.71 m from home
having degraded; the derived floor should improve on that, but the ceiling on
this approach is set by optics, not by control.

Closing that properly means tracking the pad's **outline** rather than its QR
below the decode floor — a white quadrilateral is trackable far closer than
33 readable modules — or wiring the MAVROS `landing_target` plugin, which is
loaded but unused. Recorded rather than attempted.

### §7b.4 — What is NOT yet verified about the georeferencing

The projection maths is checked against ground truth at nadir
(`sim/test_redzone_georef.py::ProjectionTests`), and the exclusion x-extents
spot-checked against the world file agree with the NW zone at (29, 10). What
has **not** been checked is the absolute placement of confirmed exclusions in
flight, end to end, against the world's true red-zone rectangles.

This matters in both directions: an exclusion placed short makes the aircraft
avoid clear ground (costing coverage — the second strip of run 14 fell to
71%), and an exclusion placed long lets it overfly red ground while believing
it is clear. `GroundGrid.inflate` biases toward over-covering, which is the
safe direction, but "biased safe" is not the same as "measured".

The check to run is mechanical: dump the confirmed exclusion set at the end of
a flight, take its union, and compare against the red-zone poses parsed from
the runtime SDF. Recorded as an open gap rather than claimed.

### §11b.1 — The first randomised arena failed, and could not say why

Seed 1001 moved the gate to (5.19, −3.53) and **rotated it 10.4°**, put the
delivery zone at (37.8, −6.54), and the start pad named target A at
(50.68, −0.25). The mission failed 24 s in:

```
Mission result: FAILED (no AEROTHON banner identified within 180 deg of the
    corridor-entry sweep (green objects seen but rejected, or none))
```

**"green objects seen but rejected, or none"** is the problem. Those are two
different failures needing opposite fixes — the aircraft looking the wrong way
or out of range, versus an identity check whose thresholds are too tight — and
the message cannot tell them apart. Nor could the run artifacts: the detector
publishes a per-frame reason on `/percep/banner/detail`, and **nothing was
subscribed to it**. The same observability defect as §6d, in a different node.

`MavCommander` now tracks those reasons and ranks them, so the abort message
carries what the detector actually said:

```
no AEROTHON banner identified within 180 deg of the corridor-entry sweep;
detector said: aspect 17.40 outside 1.5..6.0 (x12); area 120 px below
derived minimum 900 px (x8)
```

and "nothing green ever entered the frame" is a distinct, explicit answer.

**The same seed then behaved differently.** Re-run with identical code and
identical seed, arena 1 *did* identify the banner and got as far as:

```
target=A  outcome=FAILED  165s
reason: ApproachBanner: banner lost before reaching the mouth
```

So the seed 1001 banner identification is **marginal and non-deterministic**,
not a clean failure — SITL and rendering timing decide it. That is a more
useful characterisation than either single run, and it is only visible because
the arena was held fixed while the run was repeated.

**A plausible diagnosis that measurement killed.** The obvious explanation for
a gate at 6.3 m failing where the nominal 2.8 m succeeds is that the lettering
is too small to resolve at 2.2x the range. `sim/measure_banner_identity.py`
was written to confirm it and refuted it instead: identity holds down to
15 px/letter, about **30 m** for this banner and camera, so at 6.3 m the
detector has roughly five times the resolution it needs — and beyond 30 m it
is the *area* gate that fails first, not the lettering. The measured envelope
is in `docs/BANNER_IDENTITY_ENVELOPE.md`.

The fix aimed at range would have been aimed at nothing. Recorded because this
is the third time this session that the plausible cause was not the cause.

### §11b.2 — The harness spent most of its time watching nothing

Arena 1's mission reached a terminal, first-writer-wins outcome at t ≈ 24 s.
The watcher kept polling until t = 520 s, and because each iteration costs up
to 17 s of `ros2 topic echo` timeouts, the real elapsed time was far longer
again. A five-arena regression was mostly a study of idle simulators.

`live_mission_test.sh` now ends its watch as soon as the tree latches a
`Mission result:`. It reads that from the tree's log rather than
`/mission/result`, for the same reason the outcome parser does: short-lived
`ros2` CLI calls lose the DDS discovery race against this stack. The log is
truncated per run (`> "$LOG"` in the launcher), so a previous run's result
cannot trigger a false early exit.

---

## §5b — "hold altitude" was a comment, not a control loop (2026-08-17)

Arena regression seed 1002, during corridor navigation:

```
 t32  CORRIDOR_NAV  [9.8 -2.3 2.8]
 t40  ABORT         [9.7 -1.8 0.9]
```

Forward progress stopped and the aircraft **sank 1.9 m in eight seconds**. The
mission's altitude-band guard caught it and aborted, which is what it is for —
but the guard exists to catch what should not happen.

In `velocity_controller._publish()`:

```python
sp.velocity.z = 0.0            # hold altitude
```

Zero vertical velocity is a command for zero vertical **rate**. It is not a
held altitude: any thrust bias or disturbance integrates and nothing corrects
it. And `_on_pose` stored only `(x, y)` — **z was discarded**, so the
controller did not know its own altitude and could not have held it whatever
the comment said.

This is the same defect as the first live failure ever recorded in this
project (`CORRIDOR_NAV` reported while dragging along the ground at
z = 0.117 m). The guard was added then. The cause was not.

Now closed-loop: the pose handler keeps z, `_alt_correction()` returns a
capped proportional correction, and the mission **states** the altitude to
hold via `/avoidance/hold_alt` when it hands over control — the corridor
stage knows it wants the rulebook's 3 m, whereas latching whatever the
aircraft happened to be at may already be wrong.

`sim/test_altitude_hold.py` simulates the seed 1002 sink against a constant
disturbance and requires recovery inside the band, with the old behaviour as
an explicit negative control that fails the same test.

**A gap in my own tests, found by mutation.** Reverting `_publish` to
`velocity.z = 0.0` while leaving `_alt_correction()` intact broke *nothing*:
every test called the method directly, so they checked a calculation rather
than the command that reaches the flight controller.
`ThePublishedSetpointCarriesItTests` now asserts the published setpoint, and
the same mutation fails two tests.

---

## §11c — RETRACTED: the first randomised-arena numbers were contaminated

A five-arena regression produced this, and it was written up as a result:

| seed | target | outcome | reason |
|---|---|---|---|
| 1001 | A | FAILED | `ApproachBanner: banner lost before reaching the mouth` |
| 1002 | E | FAILED | `Corridor: altitude 1.15 m outside 3.0 +/- 1.5 m band` |
| 1003 | D | INTERRUPTED | `external disarm` |
| 1004 | E | FAILED | `Corridor: altitude 1.34 m outside 3.0 +/- 1.5 m band` |

**Two regressions were running at the same time.** An earlier run's
`arena_regression.sh` (started 19:12) had been reparented to systemd when its
launcher was killed, survived, and kept going while a second one started at
19:18:

```
  PID   PPID  STARTED               COMMAND
32018   2336  Mon Aug 17 19:12:24   bash scripts/arena_regression.sh -n 5 --watch 520
39414  39280  Mon Aug 17 19:18:46   bash scripts/arena_regression.sh -n 5 --watch 520
```

`live_mission_test.sh` opens each run with `pkill -f "[g]z sim"` and
`pkill -f "[a]rducopter"`. So the two harnesses were not merely sharing a
simulator — **each was shooting the other's aircraft mid-flight.** "external
disarm" is exactly what the surviving mission sees when its flight controller
is killed underneath it, and an aircraft whose FC dies mid-corridor also
stops holding altitude.

So the table above cannot distinguish stack defects from harness collisions,
and every row of it is withdrawn. It is kept here rather than deleted because
a plausible-looking result table is precisely the thing that would otherwise
get quoted later.

**What survives the retraction, and why:**

* **The altitude-hold defect is real regardless of the flight data.**
  `sp.velocity.z = 0.0  # hold altitude` with `z` discarded in `_on_pose` is
  indefensible on inspection: the controller could not know its altitude, so
  it could not hold it. See §5b. The fix is proven by
  `sim/test_altitude_hold.py`, which simulates a sink against a constant
  disturbance and carries the old behaviour as a failing negative control.
  What is *not* established is that this caused seeds 1002 and 1004.
* **The seed 1003 arena was genuinely invalid** — that was established from
  the world file's own geometry, not from the flight: the takeoff point sat
  at corridor-local (−3.77, 1.72), inside the channel. Independent of any
  collision.
* **Nothing is established about `ApproachBanner`.** Seed 1001 failed there
  twice, but under contamination.

### Why the orphan survived three attempts to kill it

Worth recording, because it silently invalidated results and the failure mode
is invisible:

```
pkill -9 -f arena_regression
```

was issued three times and reported success. It never killed PID 32018. The
commands here run inside a wrapper shell whose own command line **contains the
pattern being searched for**, so `pkill -f` matched the wrapper, and the shell
running `pkill` was killed before it reached the real target. The tell was an
`exit 144` (128 + SIGKILL) that looked like an ordinary "nothing matched" and
was waved through.

The `pgrep` check afterwards then reported a clean system — because it ran in a
*new* shell, after the killer had already died, and by then the orphan's child
`live_mission_test` had finished its run and not yet started the next one.
Two independent reasons to believe a false thing.

Kill by PID, or use the bracket trick (`pkill -f "[a]rena_regression"`) so the
pattern cannot match the process issuing it.

**Harness fix.** `arena_regression.sh` now takes an exclusive `flock` and
refuses to start if any `live_mission_test` is already running. It caught four
surviving processes on its first invocation after this was written, which is
how the orphan was finally found. Contaminated results that look like findings
are worse than no results.

### The lock then leaked, and refused a legitimate run

The next regression was blocked by its own lock with nothing running. Checked
rather than assumed:

```
$ fuser -v /tmp/aerothon_arena_regression.lock
  sarthak  85857  gz sim
  sarthak  86010  arducopter
  sarthak  86088  mavros_node
  ... 25 processes
```

`exec 9>"$LOCK"` opens a file descriptor, and file descriptors are **inherited
by children**. The whole simulator stack inherited fd 9, so the flock survived
the script that took it for as long as any ROS node stayed alive — and
`arena_regression.sh` never tore the last arena's stack down.

Both fixed: `9>&-` closes the descriptor when invoking `live_mission_test.sh`,
and the harness now kills the stack after the final run.

**And the self-kill trap caught me a second time.** Cleaning up by hand:

```
pkill -f "[g]z sim"  ...  pkill -f "http.server 8899"
```

Every pattern was bracketed except the last, which matched the wrapper shell
issuing it — `exit 144` again, cleanup abandoned half-done. The bracket has to
be on *every* pattern, not most of them.

## §11c.1 — (superseded, see the retraction above)

Five arenas, each with a different gate position **and heading**, delivery
zone, five pads, three red zones, and a delivery target read off the start QR.

| seed | target | outcome | reason |
|---|---|---|---|
| 1001 | A | FAILED | `ApproachBanner: banner lost before reaching the mouth` |
| 1002 | E | FAILED | `Corridor: altitude 1.15 m outside 3.0 +/- 1.5 m band` |
| 1003 | D | INTERRUPTED | `external disarm` (never left the ground) |

**Nothing completed.** On the nominal arena the mission completes end to end;
move the arena and it does not. That is the whole point of Phase 11, and it is
the honest headline: **the stack is not yet arena-robust.**

What the three failures are:

* **1002 is a real defect, now fixed** — the altitude-hold defect in §5b.
* **1001 is a real gap** — with the gate offset and rotated 10.4°,
  `ApproachBanner` loses the banner before completing the gate transit.
  Not yet fixed. Note also that the *same seed* identified the banner on one
  run and failed to on another, so that stage is marginal rather than
  deterministically broken.
* **1003 is an arena-validity problem, not a stack failure** — the aircraft
  never got off the ground (0.7 m, then disarm). The randomiser places
  `corridor_walls` *centred* on the gate, and the walls are 10.2 m long, so
  they extend 5.1 m **behind** it. For seed 1003 the takeoff point sits at
  corridor-local (−3.77, 1.72): **inside the corridor channel**. The
  competition starts the aircraft in front of the gate, outside the corridor.

That last distinction matters. An arena the aircraft cannot fly is not
evidence about the aircraft, and "fix" the harness to avoid a failure and the
regression stops meaning anything. Constraining the randomiser to keep the
takeoff point clear of the corridor structure makes the test *valid*, not
easier — and it is recorded here so the change is visible as a deliberate one.

**Contamination note.** `summary.tsv` also holds a `1001 ... 948s` row written
by the first, abandoned regression run after this one recreated the file. It
is from older code and is not part of this table.

---

## §5c — ApproachBanner commanded a receding carrot (2026-08-17)

The clean regression (single harness, `flock` held) gave a consistent
signature across independent arenas:

| seed | outcome | reason |
|---|---|---|
| 1001 | FAILED | flew to the gate and sank 3.2 m → 0.3 m, corridor guard fired at 1.37 m |
| 1002 | ABORTED_RTL | `Excessive attitude (roll=-3.5 pitch=50.9)` |
| 1003 | ABORTED_RTL | `Excessive attitude (roll=-2.5 pitch=48.5)` |

Two arenas aborting at ~50° of pitch is not a coincidence, and the cause is
one line:

```python
self.mav.goto(x + self.step_m * math.cos(target_yaw),
              y + self.step_m * math.sin(target_yaw),
              self.alt, target_yaw)
```

`x` and `y` are the **current** position, re-read every tick. The commanded
target therefore moves away exactly as fast as the aircraft chases it: the
position controller sees a permanent 1.5 m error and accelerates continuously,
with no arrival to decelerate into. That is not a waypoint, it is a carrot.

**The sink and the attitude abort are the same event.** A multirotor held at
50° of pitch has lost most of its vertical thrust component
(cos 50° ≈ 0.64), so it descends whatever its altitude controller wants.
Seed 1001 shows the descent; 1002 and 1003 show the attitude that caused it.
Two symptoms previously filed as separate problems.

Why the shipped arena never showed it: the gate is 2.8 m from the takeoff
point, so the transit completes in a couple of steps before any speed builds.
Every randomised arena puts it further out.

The target is now **latched** and re-issued until reached, so the aircraft
decelerates into each waypoint.

### §5c.1 — The first test for this passed with the bug still in

`test_the_target_does_NOT_recede_every_tick` originally asserted that the
distance from the aircraft to its commanded target was under a full step.
With the carrot that distance sits at `step - advance` = 1.2 m, comfortably
under 1.5 m, so **the test passed against the defect it was written for**.
Only the "same target re-issued" test caught the mutation.

Instantaneous distance is the wrong discriminator. What a carrot never permits
is *arrival*, so the assertion is now on the minimum residual over the run:
the aircraft must at some point get within `arrive_tol` of what it was told to
fly to. Both tests now fail the mutation.

Found because every fix this session is reverted and re-run. A test that
passes both with and without the fix is not evidence of anything.

---

## §11d — The real Phase 11 baseline: 2/5 (2026-08-17)

Single harness, exclusive `flock`, code as of the receding-carrot discovery
but **before** its fix. This is the honest number.

| seed | target | outcome | reason |
|---|---|---|---|
| 1001 | A | FAILED | sank 3.2 m → 0.3 m; corridor guard fired at 1.37 m |
| 1002 | E | ABORTED_RTL | `Excessive attitude (roll=-3.5 pitch=50.9)` |
| 1003 | D | ABORTED_RTL | `Excessive attitude (roll=-2.5 pitch=48.5)` |
| 1004 | D | **COMPLETED** | landing PRECISE (committed at 3.86 m, 0 re-acquisitions) |
| 1005 | D | **COMPLETED** | landing PRECISE (committed at 3.86 m, 0 re-acquisitions) |

**Three of the three failures are one defect** — the receding carrot in
`ApproachBanner` (§5c). Whether it bites depends on how far the randomised
gate lands from the takeoff point: far enough to build speed, and the aircraft
pitches past 50°; close enough and the transit ends first. The shipped arena
is 2.8 m, which is why eleven end-to-end runs never saw it.

**Both completions landed PRECISE at 3.86 m with zero re-acquisitions.** That
is the derived tracking floor (§9b) reproducing across two different
randomised layouts, not a one-off.

What this baseline establishes, stated carefully:

* The perception-driven chain — read the start QR, find and align to a moved
  and rotated banner, traverse a moved corridor, observe the zone, upload a
  fence around it, sweep, find a pad the aircraft was told about only by the
  QR, deliver, return and land precisely — **does work on arenas it has never
  seen**. Twice out of five.
* It is **not yet arena-robust**, and the gap is a flight-control defect
  rather than a perception one, which was not the expected answer.
* Nothing here was visible from the shipped arena. That is the entire
  argument for Phase 11, now demonstrated rather than asserted.

A verification run with the waypoint fix is queued; its results belong in a
separate entry and must not be merged into this table.

### §5b.1 — Altitude hold confirmed in flight, by measurement

Claimed only from unit tests until now. Probed `/avoidance/detail` during a
live corridor traversal (seed 1002, the arena that aborted at pitch 50.9 deg
in the baseline):

```
state    alt_m  hold_alt_m  cmd_vz  cmd_vx
CRUISE    3.0      3.0       -0.0    0.67
CRUISE    3.0      3.0       -0.0    0.66
CRUISE    3.0      3.0       -0.0    0.64
... 25 consecutive samples, all alt = 3.0
```

The navigator knows the altitude it is holding, measures its own, and the
correction sits at zero because there is no error to correct. Before the fix
it could not have reported `alt_m` at all — `_on_pose` discarded z.

Note what this does NOT show: a large correction arresting a sink. The
aircraft never left 3.0 m, so the loop was never exercised hard here. The
recovery behaviour remains proven only in simulation
(`sim/test_altitude_hold.py`).

### §5c.2 — The waypoint fix helped, and did not fully fix seed 1001

Same seed, same 4.8 m of travel from the takeoff point to the gate:

| | start | at the gate | lost |
|---|---|---|---|
| receding carrot | 3.2 m | **0.3 m** | 2.9 m |
| latched waypoint | 3.0 m | **2.6 m** | 0.4 m |

A sevenfold reduction, and the violent pitch-over is gone. But seed 1001 still
failed later at 1.24 m (baseline 1.37 m), so **something continues to bleed
altitude during the approach** — position setpoints at a constant z should
not permit that, and the corridor hold demonstrably works.

Not diagnosed. Recorded as open rather than guessed at: three times this
session the plausible cause was not the cause, and the honest state of this
one is "improved sevenfold, root cause of the remainder unknown".

### §11d.1 — The baseline may have been measuring the HARNESS

Chasing the residual sink with an altitude-divergence trace produced a result
that undermines the numbers rather than explaining them.

`live_mission_test.sh` began each run by killing exactly two things:

```bash
pkill -f "[g]z sim"
pkill -f "[a]rducopter"
```

Everything else from the previous run survived — MAVROS, MAVProxy, the router,
`web_video_server`, both `ros_gz` bridges, SLAM, and **every mission and
perception node, including a second `mission_bt` and a second
`velocity_controller` still publishing setpoints.** `arena_regression.sh`
calls that script once per arena, so each arena inherited the previous one's
live stack.

The symptoms were different every time and none of them named the cause:

* `STACK NEVER BECAME READY, deaths: 1` — `web_video_server` aborting on
  SIGABRT because a copy from hours earlier still held port 8080;
* an aircraft that armed and then auto-disarmed without ever leaving the
  ground;
* `FCU connected MISSING` with nothing flowing at all.

With a full teardown added, seed 1001 — which fails in **both** recorded
regressions — flew: corridor traversed, `zone observed`, `fence verified`,
**sweep at 10.0 m** (the rulebook ceiling fix, confirmed in flight), payload
`RELEASED at alt=5.00 m`, and on to the return lap. **The corridor altitude
sink did not occur.** It failed later and for an unrelated reason: the return
banner was not re-detected.

Two competing setpoint publishers fighting over one aircraft is a completely
sufficient explanation for an altitude that sags out of its band, and it is
now the leading one. What it is not yet is proven — one flight is one flight,
and the same seed behaved differently across runs even before this.

**Both the 2/5 baseline and the 2/5 verification are therefore suspect**, for
the same reason the first regression was retracted: a harness that contaminates
its own runs cannot produce evidence about the aircraft. Re-running with the
teardown in place is the only way to find out, and until that lands no claim
should rest on either table.

### The residual signature, as recorded before the harness fix

With the waypoint fix in (verified present in the source the nodes load —
`build/mission_bt/mission_bt` and `build/avoidance/avoidance` are symlinks to
`src/`, so the running code is the edited code):

| seed | baseline | with the waypoint fix |
|---|---|---|
| 1001 | sank to 0.3 m | `altitude 1.24 m outside band` |
| 1002 | `Excessive attitude (pitch 50.9)` | `altitude 1.44 m outside band` |
| 1003 | `Excessive attitude (pitch 48.5)` | `altitude 1.23 m outside band` |

Every violent pitch-over abort became a gradual altitude failure, and all
three now cross the band floor (3.0 − 1.5 = 1.5 m) at 1.2–1.4 m. That is not
a settling altitude — it is simply where the guard catches a descent that is
still going. **The failure mode changed; the pass rate did not.**

Two facts that must be reconciled by whatever explains this, and are not yet:

* the corridor altitude hold demonstrably works — 25 consecutive live samples
  at exactly 3.0 m with zero commanded correction (§5b.1), on seed 1002, the
  arena that then failed at 1.44 m;
* `ApproachBanner` commands position setpoints at a constant z = 3.0 m, and a
  position setpoint should not permit a 1.5 m descent.

So the sink is either in a phase neither of those covers, or one of them is
not in force when it happens. The cheap next check is to log the commanded
setpoint z and the measured altitude continuously through
`AlignToBanner → ApproachBanner → Corridor`, and find the tick where they
diverge — rather than reasoning about which stage "should" be in control.

A second candidate, independent of the bleed: `Corridor` fails immediately if
it inherits an out-of-band altitude, so the handover may need to settle into
the band before transferring. That would convert a hard failure into a pause
without explaining the descent, and must not be mistaken for a fix.


---

## §5d — The corridor navigator could not turn (2026-08-18)

**This is the root cause of Phase 11's 2/5, and it is one line.**

```python
sp.yaw_rate = float(self._g('max_yaw_rate'))
```

That publishes a *parameter* as the command. The parameter was `0.0`,
commented "heading is owned by the mission", so the aircraft could never yaw.
The navigator measured a gap bearing every tick and threw it away, leaving
only sideways translation to cross a rotated corridor.

The shipped arena's corridor is axis-aligned, so entering it on the banner
heading is already correct and nothing shows. Phase 11 rotates it, and the
outcome tracks the rotation exactly:

| seed | corridor yaw | outcome |
|---|---|---|
| 1001 | 10.4 deg | FAILED |
| 1002 | -11.2 deg | FAILED |
| 1003 | 8.9 deg | FAILED |
| 1004 | 4.5 deg | **completed** |
| 1005 | 1.5 deg | **completed** |

Every arena past ~9 degrees fails; both under 4.5 degrees pass. Five for five.

### The recorded flight that settled it

Seed 1002, sampled at 2 Hz off the wire:

```
 t   state         x     y     z    roll  pitch   cmd_vx  nav       front
46   CORRIDOR_NAV  7.7  -2.0  3.00   0.6   -2.8    0.80   CRUISE     3.1
50   CORRIDOR_NAV  8.9  -2.4  3.00   0.0    1.8    0.15   CRUISE     1.1
52   CORRIDOR_NAV  9.6  -2.7  3.00  -0.5    1.9    0.80   CRUISE     4.1
52   CORRIDOR_NAV  9.7  -2.7  3.00   1.1    1.5    0.00   BLOCKED    0.3
56   CORRIDOR_NAV  9.7  -2.8  3.01   1.5    7.1   -0.35   BACKOFF    0.3
61   CORRIDOR_NAV  9.7  -2.9  2.96  28.5   19.3    0.00   BLOCKED    0.1
61   CORRIDOR_NAV  9.8  -3.0  2.66   3.1   47.8   -0.35   BACKOFF    0.1
```

The aircraft cruises in cleanly — 3.00 m held exactly, 0.8 m/s, attitude
inside 5 degrees — while drifting steadily from y = -1.8 to y = -2.9. It
wedges with an obstacle 0.3 m ahead, exhausts the backoff ladder, and tips.

**The commanded velocities when it tips are 0.00, 0.00, 0.03.** The
pitch-over is a COLLISION, not a commanded manoeuvre. Everything downstream
of that — `Excessive attitude`, `Sank below airborne floor`, `altitude
outside band` — is the aircraft already in contact with a wall.

### What this retracts

**The receding-carrot fix (§5c) does not explain these aborts.** That defect
was real and the fix is correct on its own terms, but the aborts happen in
`CORRIDOR_NAV` under the velocity controller, not in `ApproachBanner`. The
two symptoms I filed together — the sink and the ~50 degree pitch — are one
event, but the event is a collision and the cause is upstream of both.

The measurement said so plainly and only because it recorded commands
alongside attitude. Reasoning from "which stage should be in control" had
produced two confident wrong answers before this.

`_yaw_rate_for()` now turns toward the gap, capped by `max_yaw_rate` (raised
from 0.0 to 0.5 rad/s). The mission still owns the heading everywhere else;
inside the corridor the thing worth pointing at is the gap.

---

## §11e — RETRACTED again: three of five failures were a pillar in the lane

The corridor-rotation correlation in §5d was real, and the conclusion drawn
from it was wrong.

| seed | corridor yaw | outcome |
|---|---|---|
| 1001 | 10.4 deg | FAILED |
| 1002 | -11.2 deg | FAILED |
| 1003 | 8.9 deg | FAILED |
| 1004 | 4.5 deg | completed |
| 1005 | 1.5 deg | completed |

Five for five is a strong signal, and it pointed at the aircraft's ability to
follow a rotated corridor. It was actually pointing at **how far the rotation
dragged the forward lane into obstacles that had not moved.**

`randomise_arena` moved three models — the banner, the wall block, the forward
surround. The corridor is six: the return lane's marker, its banner, and
`return_static_obstacles` stayed exactly where they were. So a rotated forward
corridor was driven straight through them.

For seed 1002, pillar `o4c` (0.35 x 1.45 x **3.4 m** — tall enough to span the
3 m corridor altitude) ended up at (10.60, -3.05), **0.97 m** from where the
aircraft jammed at (9.7, -2.7).

### The recording that settled it

```
 t   state         x     y     z    yaw    roll  pitch  cvx   cyaw  nav      gapdeg front
51   CORRIDOR_NAV  9.6  -2.7  3.00  -11.2   0.2   -0.3  0.80  0.00  CRUISE     0.0   3.0
52   CORRIDOR_NAV  9.7  -2.7  3.02   -2.6  -0.0    5.9  0.00  0.00  BLOCKED    0.0   0.4
55   CORRIDOR_NAV  9.7  -2.8  3.00   -1.0   1.0   17.8 -0.35  0.00  BACKOFF    0.0   0.2
62   CORRIDOR_NAV  9.7  -2.8  2.99   -0.5   3.5   16.6 -0.35  0.00  BACKOFF    0.0   0.2
66   CORRIDOR_NAV  9.8  -3.0  2.77   29.6  21.4   45.6  0.00  0.00  BLOCKED    0.0   0.1
```

Three facts kill the rotation story outright:

* **`yaw` is -11.2 deg against a -11.2 deg corridor.** The aircraft was
  perfectly aligned. It was never flying the wrong heading.
* **`gapdeg` is 0.0 throughout.** The navigator saw the gap dead ahead, so
  there was never an off-centre bearing for a yaw fix to act on.
* **Ten seconds of BACKOFF at -0.35 m/s moved it zero metres** while pitch
  climbed monotonically to 45.6 deg. That is a drone pinned against a pillar,
  not one mishandling a corridor.

### What this retracts

**The yaw-alignment fix (§5d) does not explain these failures.** Publishing
`max_yaw_rate` as the command was a real defect and the fix is correct on its
own terms — but it addressed an actuator with no signal driving it, and the
regression came back 2/5 with it in place, unchanged.

That is now **three** wrong causes for the same symptom: harness contamination
(real problem, not the cause), the receding carrot (real defect, wrong stage),
and yaw authority (real defect, no signal). Each was believed on the strength
of a plausible mechanism; each was killed by a measurement that recorded
commands alongside the response.

**The aircraft was not at fault in seeds 1001-1003.** The harness was.

All six corridor components now move as one rigid body, and
`sim/test_arena_randomisation.py` checks twelve seeds for obstacles inside the
forward channel. Reverting to the three-model move fails two tests.

---

## §11f — The first Phase 11 measurement of the AIRCRAFT: 3/5 (2026-08-18)

Every previous figure in this document measured the harness. This is the first
run where the arenas are genuinely flyable: the corridor moves as one rigid
body, the take-off point sits outside the channel, and no static obstacle lies
in the forward lane.

| seed | target | outcome | reason |
|---|---|---|---|
| 1001 | A | FAILED | return banner not re-identified: `only 1 white component(s); board aspect 0.73 outside 1.2-8.0` |
| 1002 | E | FAILED | swept `[16.9, 74.7, -24.1, 1.2]` at 10.0 m over **4 strips** without matching; budget exhausted |
| 1003 | D | **COMPLETED** | landing PRECISE (committed at 3.86 m, 0 re-acquisitions) |
| 1004 | D | **COMPLETED** | landing PRECISE (committed at 3.86 m, 0 re-acquisitions) |
| 1005 | D | **COMPLETED** | landing PRECISE (committed at 3.87 m, 0 re-acquisitions) |

**Every corridor collision is gone.** Seed 1003, which aborted at 46-51 degrees
of pitch in three consecutive regressions, now completes. Seeds 1001 and 1002
traverse the corridor cleanly and fail much later, at stages that are actually
about perception.

### What the two remaining failures are

Both are real, both name themselves, and neither is a harness artifact.

**1001 — the return banner.** The detector's own reason: seen from the
delivery-zone side the board presents at aspect 0.73, outside the 1.2-8.0 gate,
and only one white component is found where lettering is expected. The
outbound identification of the same banner succeeds. This is the separately
scored "Corridor Entry Detection Return Lap" task, and it is failing on
viewing geometry, not on detection in principle.

**1002 — the frontier search does not cover pad E.** It expanded through four
strips out to x = 74.7 and exhausted its budget. The mechanism worked exactly
as designed — expanded on evidence, logged each strip, failed closed with a
precise reason — it simply did not cover the ground the pad was on.

### What 3/5 establishes

Three arenas the stack had never seen, flown end to end with **no arena
constant anywhere in the loop**: start QR read, a moved and rotated banner
identified and aligned to, a moved corridor traversed, the delivery zone
observed, a geofence uploaded around it, the zone swept at the rulebook's
10 m, a pad found that the aircraft knew about only from the QR, payload
released at 5 m, return lap flown, precision landing committed at the derived
tracking floor with zero re-acquisitions.

The landing floor reproduces at 3.86-3.87 m across all three, and across every
completion recorded in this document — six now, on five different layouts.

**Confidence in the number itself.** Three regressions were retracted before
this one, all of them measuring harness defects: concurrent runs shooting each
other's aircraft, stale nodes fighting for ports, and a pillar in the flight
lane. Each retraction came from a measurement rather than an argument, and the
failures that remain here are ones the aircraft owns.

---

## §12 — Four defects seen on screen during a watched flight (2026-08-18)

Observed directly by the operator watching the new `/percep/overlay` feed,
which is the first time any of this was visible. The overlay earned its place
in the first flight it was used on.

### 12.1 The banner is boxed and REJECTED, not missed

The screenshot shows the gate region boxed in amber, labelled
`GREEN, NOT BANNER`, with the status bar reading `banner —`. So the detector
finds the region and fails it at a gate — it is not blind to it.

Two conditions in that frame, both visible:

* the board is **in shadow** — a wall cuts across it;
* it is viewed at a steep **oblique** angle, heavily foreshortened.

`lettering_mask()` requires the letters to be *markedly brighter and less
saturated than the board*. In shadow the white lettering darkens toward the
board's own brightness and stops registering. That is exactly seed 1001's
`only 1 white component(s)` (x895), and it is why the text rescue cannot fire
either: it needs letter blobs to read, and there is one.

The relative threshold was itself a fix for an absolute one that failed in
flat sim light. It is still relative to the *board*, which is the right idea,
but the ratio was tuned on a sunlit board.

### 12.2 The banner sweep oscillates

Watched behaviour: the aircraft yaws right, detects the banner, and
immediately yaws left again — losing it. `AlignToBanner` yaws continuously,
which both motion-blurs the frame and gives the detector very few frames at
any one heading, so a marginal detection appears and vanishes.

The operator's proposal is the correct shape of fix and is what should be
built: **stop-and-stare**. Yaw a discrete step, hold for a dwell (~5 s), let
the detector settle, and only then step again — continuing to a full 360
rather than a bounded arc.

### 12.3 RETRACTED — red zones ARE seen; the plan does not act on them

The operator saw `RED NOT_VISIBLE` and I concluded the detector never sees the
ground. **Both observations were taken at the wrong moment and the conclusion
is wrong.**

The screenshot was captured during banner alignment, when the camera is at
-20 degrees — `NOT_VISIBLE` is *correct* there. I then probed live and read
`altitude -0.0 m too low to project`, and nearly filed "the node has no
altitude" as the cause. Checking first: `mission=ABORT, alt=-0.038`, 174 pose
messages flowing. **The aircraft was on the ground.** Correct behaviour,
wrong moment again.

Sampled during the actual nadir sweep, gated on mission state:

```
SEARCH_QR  alt=10.0  camdeg=-91.1  settled=True | RED RED  excl=38   red_frac=0.16
SEARCH_QR  alt=10.0  camdeg=-91.0  settled=True | RED RED  excl=77   red_frac=0.47
SEARCH_QR  alt=10.0  camdeg=-90.9  settled=True | RED CLEAR excl=172 red_frac=0.00
SEARCH_QR  alt=10.0  camdeg=-90.8  settled=True | RED RED  excl=198
```

The camera settles at -91 degrees, projection succeeds, and the detector
georeferences **198 exclusion cells and still climbing** — plus a correct
`CLEAR` on a frame with no red in view, so the tri-state works.

**So the defect is downstream.** 198 exclusions exist and the aircraft still
overflies red ground. Narrowed candidates: the exclusions never reach
`mav.exclusions`; `_exclusions_changed()` does not trigger a mid-sweep
re-plan; or the clipped plan is computed and the unclipped waypoints are
flown.

The aircraft therefore flies over red ground because it has no idea the ground
is red, not because the avoidance logic is wrong. The avoidance logic has
never once been exercised in flight. Restricted Zone Avoidance is 10 marks
with **-5 per violation**.

Worth checking first: the redzone node gates on `/camera/pose_state` being
`settled` and not `stale`, and takes the camera pitch from it. If the camera
is at BANNER (-20 deg) or FORWARD during the sweep, or the pose feedback is
not settling, `ground_point()` correctly refuses to project — and returns
NOT_VISIBLE for a completely sound reason.

### 12.4 RETRACTED — seed 1003's abort was the operator

Flew correctly to delivery — corridor traversed, 10 m sweep, `TARGET_D`
matched, descended to 5.0 m — then `ABORTED_RTL (Abort requested)` with no
`payload RELEASED` and `delivery_offset_m: null`.

I called this a regression from my own changes. It was not. `/mission/abort`
is published by **exactly one thing — the GCS aggregator** (`pub_abort`);
neither the mission nor the harness can raise it. The operator was driving the
GCS live at the time and confirmed pressing ABORT.

Re-flown unattended, seed 1003:

```
Mission result: COMPLETED (landed and disarmed; delivery UNMEASURED;
    landing PRECISE (committed at 3.87 m, 0 re-acquisition(s)))
```

**Still passing.** No regression from the text rescue, lateral search, return
descent, return standoff or the delivery measurement.

One real thing does survive: `delivery UNMEASURED`. The accuracy figure is
plumbed end to end and reports honestly rather than claiming zero, but it did
not capture on this run — the target was not in frame at release. Worth
chasing, and 15 marks depend on it.

**The lesson, five times over.** Every causal story this session that was
believed on plausibility rather than measurement was wrong: harness
contamination, the receding carrot, yaw authority, corridor rotation, and now
these two. What killed each was a recording taken in the state the failure
actually occurs in. Any diagnosis here should be treated as provisional until
it has been recorded that way.

### What the rulebook actually requires, checked again

> **QR Code Detection:** Take off and scan the start QR code from a 5-meter
> altitude and decode the delivery location information.

Scan and decode at 5 m. **Alignment is not required** — `CenterOnQR` exceeds
the requirement, which is harmless and helps decode reliability. There is no
requirement to display, persist, or hover over the decoded data. Showing it in
the GCS and holding station briefly are both sensible for demonstrating the
capability to a judge, and the 15-minute window has roughly 8 minutes spare,
but neither is mandated.

All eight listed Mission Tasks are implemented, including the separately
scored Corridor Entry Detection Return Lap.

---

## §13 — The six gaps from §12, closed (2026-08-18)

Spec: `.scratch/mission2-final-gaps/spec.md`. Offline suite **724 passed, 3
skipped** (the skips are the deferred photo corpus). Every fix below was
mutation-checked: the fix was reverted, the suite was confirmed to go red, and
the fix restored. Where a mutation SURVIVED, that is recorded too — one did.

### 13.1 Restricted zones: the plan now constrains every leg, not just the sweep

§12.3 retracted "red zones are never detected" and left a narrow question:
198 exclusions were confirmed and the aircraft still overflew red ground.

**The answer was scope.** `plan_lawnmower_excluding` clipped sweep *lanes*, and
nothing else in the mission asked about exclusions at all. Every straight leg
— corridor exit to the observed zone, the approach, the reposition before
descent-to-decode, the return to the corridor mouth, go-home — called
`mav.goto(x, y, z)` and flew the straight line. So did the transit *between*
two clipped lane segments, which on a zone with red ground down the middle is
the diagonal straight across it.

Closed with one primitive and one caller:

| new | what it answers |
|---|---|
| `route_leg` | waypoints from A to B around the exclusion union, or an explicit refusal |
| `merge_exclusions` | 198 confirmed cells → the few boxes worth routing around |
| `leg_hits_exclusion` | would flying A→B put the **airframe** inside one |
| `point_in_exclusion` | would **holding station** here sit inside one |
| `LegRouter` | flies one routed leg across many ticks; one object, shared by the stages and the tests |

`Goto`, `GotoHome`, `ReturnToCorridorMouth`, `DescendToDecode` and
`LawnmowerSearch` route. `ApproachBanner` cannot — it is a visual servo and a
detour would take the banner out of frame — so it *checks* and aborts instead.
A blocked leg is an abort with a reason in every case; the straight line is
never the fallback.

Two asymmetries are deliberate:

- **Starting inside a zone is allowed to leave.** Refusing to move would hold
  the aircraft over the violation it is trying to end. `point_in_exclusion`
  exists precisely because *stopping* there is a different question, and
  `DescendToDecode` refuses to descend onto a candidate standing on red ground.
- **Exhausting the mid-strip re-plan budget is not permission.** The bound
  stays (a growing exclusion set would otherwise restart the strip forever),
  but remaining waypoints are still tested and intersecting ones are *skipped*.
  Running out of budget costs coverage, never compliance.

Mutations caught: legs ignore exclusions (7 tests), clearance ignored (6),
no detour budget (1).

### 13.2 The sweep stops and stares

The operator's description was exact — "it yaws to the right, it detects the
banner, but as soon as it detects the banner it yaws left". `AlignToBanner`
yawed further every tick while unidentified and switched to a bearing-follow
the instant it was identified, so a detector confirming one frame in four made
the stage reverse, which took the banner back out of frame.

Replaced with `STEP → SETTLE → DWELL → DECIDE → CENTRE`. Twelve 30° steps, a
five-second dwell at each, verdicts accumulated across the whole dwell, and the
heading **measured** as settled rather than assumed — counting commanded yaw as
achieved yaw is what once reported 180° swept for 23° turned.

**The half-turn limit is gone, and the reason it existed was wrong.** A live
run had swept 271° and locked onto something to the south, which was read as
"it searched too far". It was not: a continuous yaw acts on the first frame
that says yes, and over a long sweep something greenish always will. The
protection is the confidence floor on a dwell — most of the five seconds must
agree — not a limit on how far the aircraft may look. With that floor in
place, covering the full turn is free, and it removes the opposite failure: a
banner behind the start heading used to be unfindable, which is where seed
1001's return leg kept stranding.

A failed sweep now names all twelve headings and what each saw.

Mutations caught: one frame ends the dwell (2 tests), settle assumed rather
than measured (1), half-turn limit (3).

### 13.3 Lettering: a second path, not a looser threshold

`lettering_mask` asks whether a pixel is brighter than the **board median**.
Under a shade gradient that median is set by the sunlit half and the shaded
letters fall under it. Identification often survives on the lit half alone —
but the *reading* does not, and the reading is what rescues the oblique return
view whose aspect the gate refuses. The two failures compound, which is why
"there is very clearly a banner there" and the stack disagreed.

Added `lettering_mask_stroke`, which asks a strictly local question — is this
pixel brighter than its immediate neighbourhood — that a shadow moves
uniformly and so cannot break. Both run on **every** frame and either may
confirm; the one that actually reads the lettering wins.

Measured on the real rendered frame under a linear shade ramp, letters read out
of AEROTHON's eight:

| far edge at | 100% | 70% | 50% | 40% | 30% | 25% | 20% | 15% |
|---|---|---|---|---|---|---|---|---|
| brightness only | 8 | 8 | 5 | 3 | 3 | 0 | 0 | 0 |
| both paths | 8 | 8 | 7 | 6 | **5** | 4 | 4 | 0 |

The rescue needs five. Brightness alone holds to 50% shade; both paths hold to
30%. In full light the brightness path still wins, so the rescue does not
displace the path that works.

The fixture is a photometric transform of a frame the simulator actually
rendered, not a hand-drawn one. Four attempts at drawing letters pixel by pixel
produced fixtures that fragmented into one component per bitmap row and proved
nothing about the detector.

A blank green board and a *noisy* green board are both still refused — that is
the test that matters when widening a reader.

Mutations caught: stroke path never runs (3 tests), stroke path always wins (1).

### 13.4 The scan ledger

Everything decoded, identified or refused, in the order it first happened,
de-duplicated **in the aggregator** rather than in the browser — computing the
list twice in two languages lets the two disagree about what the run contained.
Matches are tagged and never withdrawn by a later frame; refusals keep their
reason; the repeat count is itself evidence, separating a solid read from a
single-frame blip. The bound drops the oldest *unmatched* row, because the
matched one is the single row of the run the score depends on.

Rendered in a right-hand rail, and held to the aggregator's shape by the
existing frontend contract test.

### 13.5 A five-second hover on every distinct marker

Per **distinct payload**, not per decode: a marker held in frame decodes every
frame, and hovering per decode would park the aircraft over the first pad until
the 15-minute window ran out. Wired into `ScanStartQR` and `LawnmowerSearch`.

**A mutation survived here, and it is the more useful result.** Removing the
hold command from every tick *after* the first passed all eighteen tests: the
opening tick was asserted, the continuation was not. One setpoint at the start
of a five-second hover decays into whatever the position controller drifts to,
and the caller's own setpoint from the previous tick is still the last thing
the vehicle was told. This is the same defect as `velocity.z = 0.0 # hold
altitude` wearing different clothes. Test added; mutation now caught.

### 13.6 `delivery UNMEASURED` on a completed run

15 marks. Seed 1003 flew the whole mission, landed precisely, and could not say
how far from the pad it dropped. Two causes, two guards:

- **Geometry.** Below the altitude at which the whole pad fits in frame, the
  offset is not unlikely but impossible. `WinchDrop` now derives that tracking
  floor from the pad size and the camera, and raises the drop altitude to it
  rather than honouring a configured value that makes the scored quantity
  unmeasurable.
- **A single frame.** The offset was read on exactly the tick the winch
  reported release — and the payload swinging under the aircraft is in the
  nadir camera's view at precisely that moment. The offset is now sampled all
  the way down, and a lost lock at release falls back to the most recent
  sighting, converted at **the altitude it was taken at**, reported with its
  age, and refused if stale.

`UNMEASURED` remains reachable and honest. Nothing here invents a zero.

Mutations caught: no fallback (4 tests), no floor guard (2), no sampling (1).

### 13.7 The recorder is now a repo tool

`sim/record_stage.py`, promoted from a scratch script in `/tmp`. `--stage` is
**required**: there is no way to run it without saying what state the answer is
about. That is not tidiness. The ungated version of this same probe caught the
aircraft on the pad, read "altitude too low to project", and was one edit away
from being filed as a fourth wrong diagnosis (§12.3). On exit it reports
whether the aircraft's own recorded track ever entered a confirmed exclusion,
which is the artefact the red-zone claim rests on.

### What is not closed

*(see also 13.8 — a review pass afterwards found three more defects)*

- **No live flight has been run against any of this.** Everything above is
  offline evidence. §12.3 and §12.4 are both retractions of conclusions that
  looked at least this solid, and the arena regression is the acceptance test,
  not the unit suite.
- Seed 1001's return-banner identification is *addressed* — full-turn sweep,
  shadow-robust reading, return standoff — but not demonstrated.
- 1b (photo corpus) remains deferred; 3 tests skipped.
- The world builder is sequenced after this work, by the user's instruction.

### 13.8 The review found three defects the tests did not

A review pass over the §13 diff, reading the new state machines with fresh
eyes rather than running them. All three were confirmed by probe before being
fixed, and all three are now mutation-checked.

**A. A refused leg crashed on its second tick.** `LegRouter` caches its route
under "same destination, same zones". A blocked leg set that key, stored an
empty waypoint list, and returned BLOCKED. The next tick hit the cache, took
the success path, and indexed an empty list:

```
tick 1: BLOCKED
tick 2: IndexError: list index out of range
```

A stage that reports FAILURE can still be re-entered by the tree, so this was
reachable. Every test in `test_leg_routing.py` ticked a blocked leg exactly
once — the helper returns as soon as it sees BLOCKED, which is precisely how
the gap survived. A crash inside the behaviour tree is worse than the violation
the refusal was preventing.

**B. The centring timeout measured the wrong interval.** `_centre` timed out
on "how long since this phase began" while reporting "how long since the banner
was last seen". After ten seconds of entirely normal centring, one dropped
frame aborted the mission with:

> the banner confirmed at 0 deg was not seen again within 8 s of centring

about a banner that had been in frame a tenth of a second earlier. The failure
is bad; the false explanation is worse, because it sends the next hour of
debugging somewhere else. Now timed from the last sighting.

**C. The five-second hover opened a window to un-make its own decode.** This
one is a regression §13.5 introduced. Before the hover, a confident decode
returned SUCCESS in the same tick, so the decode could not be withdrawn. Adding
a five-second hold created a window in which one blurred frame resets the
streak, drops the stage back to "still waiting", and lets the timeout fail the
mission with `start QR not decoded` — about a marker it was at that moment
hovering directly over. The payload is now latched when confirmed. The
fail-closed timeout still fires for a decode that was never confirmed, which is
the defect that whole stage exists for.

**What this says about the §13 evidence.** Every fix in §13 was
mutation-checked and every mutation was caught, and three real defects were
still sitting in the same code. Mutation testing proves a test suite notices
the behaviour it was written about; it says nothing about behaviour nobody
wrote a test for. All three of these live in the transitions — a second tick, a
late frame, a window that did not previously exist — which is exactly where
per-behaviour tests are thinnest.

One usability fix went in alongside: the scan ledger is cleared when a mission
starts, so two runs' markers cannot end up in one list with nothing to say
which was which.

Offline suite after the review: **735 passed, 3 skipped**.
