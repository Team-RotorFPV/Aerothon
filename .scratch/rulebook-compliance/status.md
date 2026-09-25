# Mission 2 rulebook compliance — work log and evidence

Status: done (sim); hardware untested
Started 2026-09-23. Rulebook: `AEROTHON-2026-Track-1-Rulebook (1).pdf`, §4.2.4, §5.4, §5.6.

## What the rulebook says about the geofence

- §4.2.4 Delivery Zone Navigation: "Delivery zone geo-fence coordinates will be
  provided to teams during Phase 2."
- §4.2.4 Geo-fencing: "Coordinates for the geo-fence boundary will be provided.
  Teams must program these into the ground station software to ensure the UAS
  stays within the designated area."
- §5.4 item 6.3: "Geo-Fence Check: Verify geo-fence limits and configurations"
  (2 marks, technical inspection).

## Why the aircraft spiralled out after the corridor

`ObserveZone` bounded the delivery zone from one 12 m lidar glance at the
corridor mouth; `LawnmowerSearch._advance_frontier()` then grew it forward,
left and right until a 60 m budget ran out. The fence was drawn round that
same guess and uploaded with `enforce=False`. Nothing held the aircraft in.

## Changes

| Area | Change |
|---|---|
| Delivery zone | Search area is the organiser's boundary (`/mission/delivery_zone`), inset 1 m. `RequireDeliveryZone` before arming; `EnterDeliveryZone` replaces `ObserveZone`; no frontier expansion. |
| Lawnmower | Always starts at the bottom-left (min x, min y) corner of the boundary. One cross-grid pass on the other axis, also from bottom-left, if the first misses. Re-plans resume at the current lane instead of waypoint 0. Crabbed 90° to travel so the camera's long axis looks ahead; speed capped at 2.5 m/s (`DO_CHANGE_SPEED`). |
| Arena geofence | New `/mission/geofence` input (polygon, 3+ vertices). `UploadArenaFence` before arming: checks home and the delivery zone are inside, pushes, reads back vertex by vertex, sets `FENCE_TYPE 5`, `FENCE_ACTION 1`, `FENCE_ALT_MAX 20`, `FENCE_MARGIN 2`, then `FENCE_ENABLE 1`. Fails closed. GCS readiness has an "Arena geofence" item. |
| MAVROS params | `/mavros/param/set` is `ParamSetV2` in MAVROS2; the old `ParamSet` client never connected, so `FENCE_ENABLE` had never been set by anything. |
| Fail-safes (§5.4 item 6) | `src/aerothon_sim/sim_gazebo/config/aerothon_failsafe.parm`: battery low RTL / critical LAND, GCS-loss RTL, RC-loss RTL, fence params. Loaded as SITL defaults; load the same file onto the Pixhawk from Mission Planner. |
| Delivery | `CenterOnTarget` (matched pad only) before the drop; `WinchDrop` servos onto the matched pad during the descent and holds heading. The matched pad's ground position is fixed when first seen, so centring can fly back to it. |
| Banner gate choice | `AlignToBanner` estimates range from board pixel area and prefers the NEAR banner; a banner beyond 80% of lidar range is the far gate. |
| Timing | Dwell / hover stages use the node's ROS clock (sim time) instead of `time.monotonic()`. |
| Gate duck | `gate_opening()` treats a surface as the board only if it crosses the flight line (walls running from the posts are not the board); crossing margins 0.6 / 0.4 m fit the 1.2 m to the first return-lane block. |
| Return lap | Return stand-off sized for the 5 m identification altitude, not the 10 m transit. Square-up aims inside the 2.5–6.0 m band, not at its edge. |
| Corridor navigator | `find_gap()` tests each heading physically: a strip of airframe radius (0.4 m) + margin (0.7 m half-width) must stay clear; near returns shrink the margin so it can peel away; escape creep as last resort. Replaces widest-angular-arc, which steered at obstacles 2 m away. |
| Red-zone map | Each frame projected with the pose nearest its capture stamp and the full attitude quaternion; blobs filled as polygons (interior, not a ring of boundary samples); one hit per cell per frame. |
| Routing | A leg starting inside an inflated exclusion steps out by the nearest edge, then routes with every box in play (it used to fly straight at the destination through the paint). |
| Arena generation | Delivery zone placed past the corridor exit and covering its mouth; red zones kept 6 m from the mouth; pads 3.5 m clear of red zones and 7 m apart. Layout file gives the organiser inputs in **home-local** coordinates (the vehicle spawns at world (-2, 2); publishing world coordinates had shifted the field 2 m). |
| Harness | `run_mission_live.py` starts and watches (the `ros2 topic pub` start never arrived); records a sim-time track with roll, pitch and mode. `probe_rates.py`, `probe_perception_rates.py`, `probe_redzone_map.py`. |
| Grader | `sim/check_track.py`: red-zone entries, geofence exits, corridor traversal **through** the correct lane below the wall tops (not over or beside), tilt > 30°, unexpected / failsafe modes, ceiling, 900 s window, true drop error, landing at home. `arena_regression.sh` counts a seed only if COMPLETED **and** the grade passes. |

## Host

WSL2 on this laptop renders sensors in software (llvmpipe; Windows reports only
the Intel UHD adapter). Real-time factor ~0.28, so one mission (~6.5 simulated
minutes) takes 45–60 wall minutes.

## Evidence (live runs)

See `logs/`. Each run has `_stack.log`, `_track.csv`, `_layout.json`, `_grade.txt`.

| Run | Arena | Outcome | Grade notes |
|---|---|---|---|
| run1–run4 | shipped | failed at duck / search / return slalom | each failure fixed above |
| run5 | shipped | COMPLETED, 405 s sim | through both lanes, in fence, drop 0.34 m true, landed 0.68 m; 1 red-zone entry on a detour leg (routing fix after) ; no attitude data |
| A/1001 | random | FAILED square-up at band edge | fixed |
| A/1002 | random | FAILED CenterOnTarget lost pad | fixed |
| A/1003 | random | COMPLETED, 378 s sim | every check passes except tilt (track predates attitude logging) |
| B/1001–1005 | random | 5/5 COMPLETED | 4 graded pass; 1002 ungraded (recorder stopped before disarm, fixed); 1004 passed a red-zone check at 0.31 m on the old 0.3 m radius (radius raised to 0.4 m) |
| D/1001 | random | COMPLETED | all checks pass |
| D/1002 | random | FAILED return duck | a wall behind the gate read as the board; board now only the surface crossing the flight line near the group front (fixed, test added) |
| D/1003 | random | COMPLETED | all checks pass |
| D/1004 | random | COMPLETED, GRADE_FAIL | red-zone entry on the return transit: camera still at the banner pose, so the red-zone map was not updated; camera now nadir for the transit (fixed, test added) |
| D/1005 | random | COMPLETED | all checks pass; drop 0.35 m true, landed 0.71 m from home, max tilt 17.7° |
| E/1001 | random | INTERRUPTED "external disarm" | not a disarm: SITL starved on the host, MAVROS declared the link lost and had it back 0.27 s later while the aircraft hovered armed in GUIDED. Two fixes: a disarm counts only when reported over a live link (and `Land` needs a live link to report COMPLETED); a link drop gets a 2 s grace before the abort latches. Also: pass 1 missed pad A because after the 5th mid-sweep re-plan the plan froze, and two lanes whose far ends had turned red were skipped whole. Re-plans now resume ahead of the aircraft and are spaced, not capped, so lanes are clipped, not dropped. Tests added for each (980 pass). |
| E/1002 | random | COMPLETED | all checks pass (the arena whose return duck failed in D) |
| E/1003 | random | COMPLETED | all checks pass; 10 mid-sweep re-plans (past the old cap), closest red 2.26 m, drop 0.31 m, landed 0.69 m, tilt 18.2°. One segment start turned red inside the re-plan spacing and was skipped: a blocked waypoint now re-plans at once (test added). |
| E/1004 | random | FAILED at the return stand-off | the main red zone's corner was 2 m from the nominal stand-off (inside the router's inflated box); the router refused the destination. The stand-off now moves to the nearest clear point (3–5 m out, ±2 m across) and `AlignToBanner` never steps onto red ground (tests added; each fails on the old code). |
| E/1005 | random | COMPLETED | all checks pass; delivery 0.04 m (tree's measure) |
| F/1001 | random | COMPLETED | all checks pass; pad A found on pass 1 (12 re-plans, 0 skipped); a live 2.7 s heartbeat loss was ridden out; 369 s sim, closest red 2.67 m, drop 0.32 m, landed 0.68 m |
| F/1002 | random | COMPLETED | all checks pass |
| F/1003 | random | COMPLETED | all checks pass |
| F/1004 | random | FAILED square-up | the stand-off moved off red as designed, then the square-up's 1.5 m step round the return board ran at the red zone's corner; the guard held position and the same step was asked for 14 times. A blocked step now takes the nearest clear version of the move (turned up to 80° or shortened, path checked), holding only if none is clear (test added). Note: the layout's `red_zones` / `pads` are WORLD coordinates (only the zone and fence rects are home-local); main red here is home-local x 24.8–34.8, y 3.4–10.4. |
| F/1005 | random | COMPLETED | all checks pass |
| F2/1004 | random | COMPLETED | all checks pass on the final code: stand-off moved off red to (22.4, 3.3); one square-up step turned round the red corner; closest red 1.97 m, 278 s sim, drop 0.29 m, landed 0.71 m, tilt 18.1° (`logs/F2_1004_*`) |

## Result (before the return-lane stand-off)

Seeds 1001–1005 all COMPLETED **and** passed every ground-truth check:
1001, 1002, 1003, 1005 from batch F (`logs/regression_F`), 1004 re-flown.
Superseded by batch G below, which is the result for the final code.

## Return stand-off faces the return lane

The stand-off was laid out from the recorded exit, which is on the
OUTBOUND lane's centreline (arena 1004's exit, in the corridor frame, is
(14.8, 2.1)); the return banner hangs over the return lane at (12, -2). So
the return square-up always began ~35° off the board and had to travel
round it, which is where arena 1004's red corner lay. `ReturnToCorridorMouth`
now lays the stand-off out in front of the return lane: the exit moved
`return_lane_offset_m` (default -4.0, port-positive facing out; a property
of the corridor, set it from the real one) across the corridor axis. The
axis is the heading squared on the outbound gate (`Mav.gate_heading`,
lidar-measured, latched by the first `GateAdvance`), else the exit yaw. The
exit pose and gate heading are now cleared on each mission start. Offline:
992 passed. Batch G re-flies 1001–1005 with it.

| Run | Arena | Outcome | Grade notes |
|---|---|---|---|
| G/1001 | random | COMPLETED | all checks pass; return square-up 1 step (range only; batch F: 5 steps from -23° off); 337 s sim, closest red 2.65 m, drop 0.32 m, landed 0.67 m |
| G/1002 | random | COMPLETED | all checks pass; return 2 steps (+7° then range); 197 s, drop 0.25 m, landed 0.76 m |
| G/1003 | random | COMPLETED | all checks pass; return 2 steps; 328 s, red 2.31 m, drop 0.30 m, landed 0.68 m |
| G/1004 | random | COMPLETED | all checks pass; return 1 step, square at -0.1° from (21.4, -0.7) in front of the return lane; no stand-off move and no red-blocked step (the arena that failed E and F); red 2.62 m, drop 0.30 m |
| G/1005 | random | COMPLETED | all checks pass; return 3 steps; closest red 1.65 m on a clipped search lane at 10 m (the 1.5 m clearance working); drop 0.30 m, landed 0.72 m |

Batch G: 5/5 COMPLETED and GRADE_PASS in one run, on the final code
(`logs/regression_G`). Max tilt 18.2°, no link drops, no red-ground
fallbacks needed.

## Custom worlds, a physical payload, separable corridors

| Area | Change |
|---|---|
| World editor | `tools/world_editor/index.html` (three.js) + `serve.py` (stdlib, `http://127.0.0.1:8777/`, run in WSL): place take-off, outbound and return corridors (pose, length, width, wall height; return linked beside the outbound one or free), return-lane obstacles, zone, fence (auto or polygon), red zones, pads, decoys, target, QR sizes; live checks from `scripts/world_spec.py`; save to `sim/worlds/`; *Save & run simulation* (optional chase view / video). |
| World spec | `scripts/world_spec.py`: schema, geometry, validation (mission-capability warnings too: banner range from take-off, passable gap past each obstacle >= 1.6 m warned, < 1.0 m refused). `materialize_world.py --world-spec` builds the world and a layout with both lanes, obstacle footprints and a polygon fence. |
| Runner | `scripts/run_custom_world.sh WORLD.json [--gui] [--record]`; `scripts/watch_mission_gui.sh` (chase camera, window capture sped up by the measured wall/sim ratio, segments restarted if the window is lost). |
| Payload | Real 100 g payload in Gazebo on a winch: prismatic line hung from a damped universal joint (a rigid rod put 1.3 kg m^2 on the attitude loop and the aircraft rolled over), `DetachableJoint` release. `winch_ctrl backend:=gazebo` (now the sim default). `perception_redzone payload_node` finds it; `WinchDrop` confirms the drop with the camera after the hook winds up (size-at-altitude check), measures payload against pad, and an unconfirmed drop ends DELIVERY_UNCONFIRMED. Grader: payload released + payload on the pad from ground truth. |
| Return gate | `FindReturnBanner`: full turn at the stand-off, then vantage points along the zone edge looking outward; ignores the outbound banner (recorded on the way out). |
| Navigator | Gap search +/-90 deg, turn penalty scaled by how clear straight ahead is, heading clamped to +/-60 deg off the lane axis, hysteresis on the chosen side. Closed-loop custom slalom tests added. |
| Grader | Per-lane traversal in each corridor's own frame; obstacle contact; old layouts graded as before. |

| Run | World | Outcome | Notes |
|---|---|---|---|
| example_custom | custom (turned take-off and corridor, 4 turned red zones, polygon fence) | COMPLETED | all checks pass; flown from the editor's Run button |
| shipped + payload #1 | shipped | ABORTED_RTL | rigid winch rod: roll -60 deg at 3.6 m payout (fixed: universal joint) |
| shipped + payload #2 | shipped | COMPLETED | released at 5.04 m, camera: 0.27 m from pad; ground truth 0.27 m; 14/14 checks (`logs/custom/shipped*`) |
| split_corridors #1 | return corridor moved to the zone's south edge, 3 custom obstacles | FAILED STUCK in the return lane | return banner found by camera; navigator crept into obstacle 2 (fixed above; reproduced and verified offline) |
| split_corridors #2 | same | ABORTED_RTL (pitch 48.6 deg) | the S-turn past obstacles 1 and 2 now worked; then flew into obstacle 3: 3.2 m tall, UNDER the lidar's 3.23 m scan plane at the 3 m corridor altitude, so invisible yet hit. `world_spec` now refuses obstacles with tops between 2.65 m and 3.33 m (test added); the world's obstacle 3 raised to 3.5 m |
| split_corridors #3 | same, obstacle 3 at 3.5 m | COMPLETED | 14/14: return corridor found by camera, custom obstacles flown (closest 0.43 m -- near the navigator's limit), payload 0.26 m from pad (ground truth), landed 0.71 m (`logs/custom/split_corridors*`) |
| H/1001-1005 | random | 5/5 COMPLETED, GRADE_PASS | regression after the navigator changes, now with the physical payload on every seed: 13/13 checks each (payload released and on the pad from ground truth, 0.23-0.34 m), return banner found on the first look every time, max tilt 18.3 deg, closest red 1.62-14.4 m (`logs/regression_H`). The randomised layouts carry no obstacle footprints, so "no obstacle contact" is graded on custom worlds only. |

## Open

- Real-camera and hardware qualification remain untested (sim only).
- Banner size used for range estimation (3.7 x 1.15 m) is the simulated board;
  set `banner_w_m` / `banner_h_m` to the real banner.
