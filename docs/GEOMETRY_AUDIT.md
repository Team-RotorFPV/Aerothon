# Geometry Audit — every assumption the mission currently hardcodes

**Produced:** Phase 0 (see `PHASE_PLAN.md`)
**Purpose:** The user's locked architecture decision is **fully perception-driven navigation with no fixed arena geometry**. This document enumerates every place the code currently assumes it already knows where things are, and names the phase that replaces each one with a sensor observation.

Nothing in this list may still be a constant when Phase 11 (randomised-arena regression) runs. That regression is the enforcement mechanism: a generated arena will simply not match any of these numbers.

**Legend for "Replaced by"** — the perception source that must supply the value instead.

---

## A. Mission tree waypoint constants

File: `src/aerothon_mission/mission_bt/mission_bt/mission_tree.py` (the `defaults` dict in `main()`)

These are documented in the source as "declare as params". They are **not** declared as ROS parameters — they are a plain Python dict, so they cannot even be overridden at launch today.

| # | Constant | Value | What it assumes | Replaced by | Phase |
|---|---|---|---|---|---|
| A1 | `takeoff_alt` | 5.0 | Rulebook start altitude | Legitimate (rulebook datum) — keep as a **declared param**, not a dict entry | P0 note / P10 |
| A2 | `search_alt` | 10.0 | QR readable from 10 m | ✅ **CLOSED (P6).** `search_planner.plan_search()` derives sweep and decode altitudes from camera geometry + the measured px/module floor. **MEASURED, and the original assumption was false.** `docs/QR_DECODE_ENVELOPE.md`: reliable decoding needs ≈5.3 px/module, giving 7.0 m for the 2.2 m *simulated* pad and only **1.3 m** for a realistic 0.4 m marker at 640×480 (5.4 m at 4K). A single-altitude sweep cannot both cover the zone and decode. P6 must sweep high for candidate pads and descend to decode | ✅ P1 measured → P6 implements |
| A3 | `drop_alt` | 5.0 | Winch payout length | Winch payout capability + measured ground-detect margin | P8 |
| A4 | `scan_pose` | (0,0,5) | Start QR is directly under the spawn point | ✅ **CLOSED (P3).** FindStartQR descend-and-retry, CenterOnQR visual centring, then ScanStartQR holds where centring left it | ✅ done |
| A5 | `corridor_entry` | (5,0,3) | Corridor mouth is 5 m east of home | ✅ **CLOSED (P4).** `AlignToBanner` yaws to the identified banner; that heading is the corridor. Fails closed if no banner is identified | ✅ done |
| A6 | `corridor_exit_x` | 15.5 | Corridor is 15.5 m long, aligned to +X | ✅ **CLOSED (P5).** Navigator reports `corridor_exited` when both side clearances exceed `corridor_open_m` for N ticks. Verified in the first complete end-to-end flight | ✅ done |
| A7 | ~~`zone_entry`~~ | **GONE** | — | `ObserveZone`: lidar depth/width at the corridor mouth | ✅ P6 |
| A7b | observed window treated as the whole zone | — | **The lidar's range is not the zone's size.** 12 m observed vs a 40 m zone, so only pad C of five was ever reachable and no red zone was | ✅ **CLOSED (P7).** `LawnmowerSearch` advances the frontier strip by strip on an endurance budget; fence covers the envelope |
| A8 | ~~`zone`~~ | **GONE** | — | Same observation, rotated by the measured exit heading; implausible extents fail closed | ✅ P6 |
| A9 | ~~`corridor_return_entry`~~ | **GONE** | — | `ReturnToCorridorMouth`: the recorded exit pose, reversed and wrapped | ✅ P9 |
| A10 | `corridor_return_exit_x` | 4.5 | Return corridor ends at x=4.5 | ✅ **CLOSED (P5).** Same detector. This constant was not merely inelegant — it was unreachable, and stalled three live runs | ✅ done |
| A11 | `home` | (0,0,5) | Home is the local-ENU origin | Legitimate — GPS home position is the one valid global reference. Should read **`/mavros/home_position`**, not assume (0,0) | P9 |

**A1 is the only one that survives** as a genuine constant. A11 now reads
`/mavros/home_position/home` from the flight controller. Every value in the
mission is a declared ROS parameter — `declare_mission_params()` — and the
ones perception replaced were deleted outright rather than left as dead
parameters.

---

## B. Search pattern constants

File: `mission_tree.py`, `LawnmowerSearch`

| # | Constant | Value | What it assumes | Replaced by | Phase |
|---|---|---|---|---|---|
| B1 | `spacing` | 6.0 vs Q9's 5.0 | A lane width that happens to give camera coverage | ✅ **CLOSED (P6).** Derived from swath width and overlap; coverage proven by sampling, and the proof is capable of failing | ✅ done |
| B2 | lawnmower rectangle | from A8 | Zone is axis-aligned and rectangular | Observed zone polygon | P6 |
| B3 | waypoint tolerance | 0.8 | — | Keep, but declare as a param | P6 |
| B4 | red-zone handling | **none** | Search may fly straight through a penalty zone | ✅ **CLOSED (P7).** Exclusions subtracted from the lane plan, re-read DURING the sweep (they are invisible from the corridor exit where they used to be sampled), bounded re-plans. First live confirmation run 13 | ✅ done |

Note the spec conflict in B1: the code default (6.0 m) does not match `goal.md` Q9 (5.0 m). Neither is derived from the camera geometry. Both are wrong until computed.

---

## C. Winch sequence constants

File: `mission_tree.py`, `WinchDrop`

| # | Constant | Value | What it assumes | Replaced by | Phase |
|---|---|---|---|---|---|
| C1 | `self._t > 20` | ✅ **CLOSED** | WinchDrop waits on `/winch/status` reporting AT_GROUND; no fixed timer | ✅ done |
| C2 | phase-2 `stow` | ✅ **CLOSED** | stow completion observed via `/winch/status` | ✅ done |
| C3 | drop tolerance | 0.5 / 0.6 | — | Declared param, tied to measured delivery accuracy | P8 |

C1 is the worst of these: a fixed tick delay standing in for the actual physical event the rulebook scores.

---

## D. Avoidance controller constants

File: `src/aerothon_avoidance/avoidance/avoidance/velocity_controller.py`

These *are* properly declared as ROS parameters — credit where due. But several encode arena assumptions rather than vehicle limits.

| # | Param | Value | Assessment | Phase |
|---|---|---|---|---|
| D1 | `cruise_speed` | 0.8 m/s | Vehicle limit — legitimate param | — |
| D2 | `front_fov_deg` | 40.0 | Sensor geometry — legitimate | — |
| D3 | `side_fov_deg` | 30.0 | Sensor geometry — legitimate | — |
| D4 | `brake_dist` | 2.5 m | Vehicle stopping distance — legitimate | — |
| D5 | `stop_dist` | 0.8 m | Matches `goal.md` Q8 safety bubble — legitimate, but currently a *stop* threshold, not a *repulsive* constraint | P5 |
| D6 | `k_center` | ✅ **CLOSED (P5)** | replaced by follow-the-gap; no assumption of two symmetric walls | ✅ done |
| D7 | `max_lateral` | 0.6 m/s | Vehicle limit — legitimate | — |
| D8 | Q21 filter chain | ✅ **CLOSED (P5)** | angular masking, range clamp and median outlier filter implemented and tested | ✅ done |

D6 is the structural problem: the controller's whole model is "I am between two walls." That is an arena assumption wearing a parameter's clothing.

---

## E. Perception constants

Files: `src/aerothon_perception/*`

| # | Param | Value | Assessment | Phase |
|---|---|---|---|---|
| E1 | `min_area_frac` | **derived** | Now computed from the banner's projected pixel area at `max_detect_range_m` for this camera; an explicit fraction still overrides | ✅ closed |
| E1b | banner IDENTITY range | **undeclared** | `max_detect_range_m` = 25 m gates AREA only. The lettering check decides identity and its range was never derived or stated | ✅ **MEASURED.** 15 px/letter floor (~30 m here); the area gate binds first everywhere in the usable range. `docs/BANNER_IDENTITY_ENVELOPE.md` |
| E2 | `s_lo` / `v_lo` | 90 / 60 | HSV thresholds tuned by eye against sim rendering; unvalidated against real sunlight | P1 (corpus) |
| E3 | `target` | `''` | Injected from the start QR — correct design | — |
| E4 | `process_every` | 1 | CPU decimation — legitimate | — |
| E5 | banner identity | ✅ **CLOSED (P4)** | white-lettering structure check; green decoys added to the world so rejection is testable | ✅ done |
| E6 | QR confidence | ✅ **CLOSED (P3)** | K-consecutive-frame confirmation + altitude-aware size plausibility | ✅ done |

---

## F. Simulation world fixtures

File: `src/aerothon_sim/sim_gazebo/worlds/mission2.sdf` — **50 `<pose>` elements**, all fixed.

These are *legitimately* fixed for a test fixture. The defect is not that the world has coordinates; it is that **the mission code shares those coordinates**. The audit item is therefore:

| # | Item | Requirement | Phase |
|---|---|---|---|
| F1 | Spawn pose, corridor walls, obstacles, banner, zone, red zones, target pads | Must become **generated** from a randomised seed so no mission constant can silently match them | P11 |
| F2 | QR payload strings (`AEROTHON2026:M2:TARGET_A..E`, `scripts/generate_competition_assets.py`) | Start payload must be randomly chosen per run from the target set, so a hardcoded target string fails | P11 |
| F3 | QR/banner rendering | Currently box geometry, textures unreliable — perception is untested through the sim camera | P1 |

F2 deserves emphasis: `qr_start.png` and `qr_target_a.png` currently carry **the same payload**, so the "matching" logic is satisfied by a fixture coincidence rather than by a decode.

---

## G. GCS / map constants

| # | Item | Value | Assessment | Phase |
|---|---|---|---|---|
| G1 | Offline tile cache | lat −35.36324, lon 149.1652, z14–19, radius 1 tile | ArduPilot SITL Canberra default. Venue recache required, larger radius | Deferred (needs venue coords) |
| G2 | `safety.ekf` | hardcoded `true` | Must come from MAVROS diagnostics | P10 |
| G3 | `safety.geofence` | hardcoded `INSIDE` | Must come from MAVROS | P10 |
| G4 | GPS satellite count | initialised 0, never populated | Must come from GPS status | P10 |

---

## Summary of enforcement

| Phase | Audit items closed |
|---|---|
| P1 | A2 (measurement), E2, F3 |
| P2 | A4 (camera pointing precondition) |
| P3 | A4, E6 |
| P4 | A5, E1, E5 |
| P5 | A6, A7, D5, D6, D8 |
| P6 | A2, A8, B1, B2, B3 |
| P7 | B4 |
| P8 | A3, C1, C2, C3 |
| P9 | A9, A10, A11 |
| P10 | A1, G2, G3, G4 |
| P11 | F1, F2 — and re-verifies every row above |
| Deferred | G1 (blocked on venue coordinates) |

**Total: 38 audited items. 2 are legitimate constants (A1, A11) and still need to become declared parameters. 1 is blocked externally (G1). The remaining 35 must be replaced by perception.**
