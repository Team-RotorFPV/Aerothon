# AeroTHON Mission 2 — Current Progress and Honest Handoff

**Date:** 2026-08-15  
**Workspace:** `/mnt/newvolume/MY DOCUMENTS/VIT/Team Rotor Fpv/AEROTHON`  
**Intended reader:** Claude or the next engineer continuing this work

## Executive summary

This repository currently provides a connected ArduPilot SITL demonstration,
not a competition-ready autonomous Mission 2 system.

Gazebo, ArduPilot SITL, MAVROS, ROS 2, the custom GCS, camera transport, lidar,
SLAM, RViz and MAVLink routing can all run together. The GCS can start the
behaviour tree and the simulated Iris can arm and take off. However, the
perception outputs are not correctly integrated into flight guidance, the
obstacle controller cannot reliably navigate the return corridor, several GCS
safety fields are placeholders, and the mission can advance after perception
failure. It must not be deployed for autonomous real-aircraft flight in its
current state.

The most recent live run ended with the aircraft disarmed but the behaviour
tree still reporting `GOTO_CORRIDOR`. The live camera was pointed at a wall and
sky because the aircraft was severely tilted near a corridor wall. RViz was
largely reflecting this real bad pose rather than inventing it.

## Current runtime state

At the time this report was written, the stack was still running under process
group `176131` from:

```bash
./scripts/launch_level6_sim.sh
```

Observed live state:

```text
MAVROS connected: true
Armed: false
Mode: GUIDED
Mission state: GOTO_CORRIDOR
```

This is itself a bug: disarming or landing outside the behaviour tree does not
reset/cancel the active mission.

To stop the current stack safely:

```bash
kill -TERM -- -176131
```

Do not assume that process group remains valid on a future run. Resolve it
again with:

```bash
ps -eo pid,pgid,stat,cmd | rg 'launch_level6_sim|gz sim|arducopter|mav_router.py'
```

## What is genuinely implemented and verified

### Simulation and transport

- Gazebo Harmonic loads the Mission 2 fixture.
- ArduPilot Copter SITL connects through the official Gazebo JSON interface.
- MAVROS receives a real ArduPilot heartbeat and telemetry.
- The custom WebSocket aggregator runs on port `8765`.
- `web_video_server` runs on port `8080`.
- The local GCS is served on `http://127.0.0.1:8899/`.
- The custom UDP router fans MAVLink out to MAVROS and Mission Planner.
- A real ArduPilot heartbeat was received on the local Mission Planner port
  `14551`.
- The router UDP fan-out and command return path were tested independently.
- Direct Pixhawk serial options were added to `scripts/mav_router.py`:
  `--fcu-serial` and `--fcu-baud`. These have not been tested against physical
  hardware.

### Vehicle and sensors

- `scripts/materialize_vehicle_model.py` starts from the maintained upstream
  `iris_with_gimbal/model.sdf` and retains the Iris flight dynamics and
  ArduPilot control plugins.
- The old three-axis gimbal include and old camera are removed from the
  materialized model.
- A visible RPLidar C1-style body is rigidly mounted above `base_link`.
- A front webcam is attached through one physical pitch joint named
  `webcam_pitch_joint`.
- Camera commands from the GCS physically move that Gazebo joint. A `-45°`
  command was verified at approximately `-0.785 rad` in Gazebo joint state.
- `/camera/image`, `/camera/camera_info` and `/scan` publish live data.
- Corresponding links exist in the RViz URDF.

### GCS

- The frontend has Flight Map, Video and SLAM tabs.
- The WebSocket endpoint is editable.
- The authentication token field and backend token check were removed as
  requested.
- Connection status displays distinguish GCS WebSocket and FCU connection.
- Satellite imagery is rendered from locally cached Esri tiles.
- Camera pitch buttons send live commands.
- `START M2` sends a command and receives an acknowledgement.

### Mission start plumbing

An important behaviour-tree execution bug was fixed. Previously
`BehaviourTree.tick_tock()` blocked before `rclpy.spin()`, so mission-start and
MAVROS callbacks were not processed. The tree now ticks from a ROS timer.

One controlled test through the real GCS button produced:

```text
WAITING -> ARMING -> TAKEOFF
mode: GUIDED
armed: true
observed altitude: 4.06 m
```

The vehicle was then commanded to land and disarmed. This proves command and
takeoff plumbing only. It does **not** prove the autonomous mission.

### Tests that passed

```text
13 Python mission/GCS unit tests passed.
Frontend TypeScript/Vite production build passed.
Python syntax checks passed.
Shell syntax checks passed.
Router UDP fan-out and return-path test passed.
```

The existing unit tests do not cover the full perception/control loop and
should not be treated as mission qualification.

## Live evidence of current failures

The live perception topics were sampled while the problem was reproduced:

```text
/percep/qr/decoded:       ""
/percep/qr/matched:       false
/percep/banner:           x=0, y=0, z=0
/percep/redzone:          false
```

The captured live camera frames are:

- `/tmp/aerothon_camera_live.jpg`
- `/tmp/aerothon_qr_annotated.jpg`

Both showed a grey wall/sky view and no target. The detector cannot detect
objects that are not in the camera field of view.

The live `map -> base_link` transform during the failed run was approximately:

```text
translation: [2.807, -0.371, -0.157]
RPY degrees: [-53.786, 5.925, -177.882]
```

Gazebo `/odometry` and MAVROS local pose agreed up to quaternion sign, so the
large tilt seen in RViz represented the actual simulated aircraft pose.

## Perception: what is real and what is missing

### QR detection

Real implementation:

- `perception_qr/qr_node.py` uses `cv2.QRCodeDetector.detectAndDecodeMulti()`.
- All generated source QR PNG files decode correctly offline with OpenCV.
- Payloads are deterministic simulation fixtures such as
  `AEROTHON2026:M2:TARGET_A`.

Broken integration:

- The start QR is on the ground approximately 1 m ahead of the spawn point.
- The mission holds at `(0, 0, 5)` but does not command the camera downward.
- The default camera remains forward-facing, so the ground QR is generally not
  visible during `ScanStartQR`.
- `ScanStartQR` returns `SUCCESS` after 80 ticks even with an empty decoded
  value. At a 5 Hz tree tick this is approximately 16 seconds.
- An empty start target therefore does not fail or abort the mission.
- `qr_offset` is subscribed by the mission commander but is never used to
  centre the aircraft over a matched target.
- QR performance has not been validated at the rulebook stand-off distances,
  with realistic camera noise, motion blur or lighting.

Relevant files:

- `src/aerothon_perception/perception_qr/perception_qr/qr_node.py`
- `src/aerothon_mission/mission_bt/mission_bt/mission_tree.py`
- `src/aerothon_mission/mission_bt/mission_bt/mav_commander.py`

### Banner detection

Real implementation:

- `perception_banner/banner_node.py` performs configurable HSV green
  segmentation, morphology, area gating and aspect-ratio gating.

Limitations and false integration claims:

- It detects a green rectangular blob; it does not recognize the word
  `AEROTHON` or validate the banner identity.
- The mission commander stores `/percep/banner`, but no behaviour-tree node
  uses it for yaw or lateral alignment.
- The mission jumps directly from start-QR scanning to a hardcoded corridor
  waypoint.
- `velocity_controller.py` says heading was set by banner alignment, but no
  banner-alignment controller exists.
- The GCS label `ALIGNED` means only that a qualifying green blob was detected,
  not that the vehicle is geometrically aligned.

### Red-zone detection

Real implementation:

- `perception_redzone/redzone_node.py` measures the fraction of red HSV pixels
  in the camera frame.

Missing functionality:

- The mission tree does not subscribe to `/percep/redzone`.
- No planner uses the red-zone geometry to generate a safe search path.
- No actual exclusion polygon is sent to ArduPilot as a geofence.
- The detector only updates the GCS warning.
- A `false` value can mean either “clear” or simply “red zone outside the
  camera view”; the UI currently presents both as `CLEAR`.

## Navigation and avoidance failures

`src/aerothon_avoidance/avoidance/avoidance/velocity_controller.py` is a basic
reactive controller, not a complete obstacle-avoidance planner.

Current behaviour:

- It computes minimum front, left and right lidar ranges.
- It slows/stops when the front sector is obstructed.
- It applies lateral velocity from left/right wall imbalance.

Missing behaviour:

- It does not plan a path around a frontal obstacle.
- It does not select a safe pass side based on obstacle extent.
- It has no recovery behaviour when stopped.
- It assumes vehicle heading is already correct, but banner alignment is not
  implemented.
- It holds `yaw_rate = 0` and relies on BODY_OFFSET_NED velocity.
- Corridor completion is determined only by crossing a hardcoded local X
  threshold.

This controller is insufficient for the rulebook return corridor with
alternating obstacles.

## Mission logic defects and hardcoding

File: `src/aerothon_mission/mission_bt/mission_bt/mission_tree.py`

The following values are hardcoded in a Python dictionary and are not actually
declared as ROS parameters despite the nearby comment:

```python
takeoff_alt = 5.0
search_alt = 10.0
drop_alt = 5.0
scan_pose = (0.0, 0.0, 5.0)
corridor_entry = (5.0, 0.0, 3.0)
corridor_exit_x = 15.5
zone_entry = (18.0, 0.0, 3.0)
zone = (20.0, 52.0, -12.0, 12.0)
corridor_return_entry = (15.0, 0.0, 3.0, pi)
corridor_return_exit_x = 4.5
home = (0.0, 0.0, 5.0)
```

Additional defects:

- QR scan times out to success.
- No banner alignment behaviour exists.
- No red-zone behaviour exists.
- No visual-servo target-centering behaviour exists.
- Search uses a fixed lawnmower rectangle and ignores red zones.
- External LAND/DISARM does not reset the behaviour tree.
- Abort/start restart semantics are incomplete because the memory sequence can
  retain progress.
- The mission may continue publishing setpoints while disarmed.
- Completion and failure states are not robustly latched/reported.
- The winch sequence uses fixed tick delays rather than actuator feedback.

## Payload/winch status

`/winch/cmd` currently has two publishers and **zero subscribers**.

The mission and GCS can publish `lower`, `release` and `stow` strings, but no
simulation plugin, ROS hardware driver, servo controller or feedback topic
consumes them. Payload release is presently a stub.

## GCS fields that are placeholders or misleading

File: `src/aerothon_gcs/gcs_aggregator/gcs_aggregator/aggregator.py`

- `safety.ekf` initializes to hardcoded `true` and is not updated from MAVROS
  diagnostics.
- `safety.geofence` initializes to hardcoded `INSIDE` and is not updated.
- GPS satellite count initializes to zero and is never populated.
- Checklist `takeoff` becomes true when state enters `TAKEOFF`, not when the
  target altitude is reached.
- Checklist `corridor`, `return` and `land` similarly represent state entry,
  not verified completion.
- Banner `ALIGNED` means green blob detected only.
- Red-zone `CLEAR` is displayed when the detector publishes false, even if the
  camera cannot see the ground.

`/mission_ready` is also too weak. It currently requires only:

- MAVROS connected
- valid GPS status
- recent `/scan`
- recent camera image

It does not require valid EKF health, geofence state, TF consistency, SLAM
health, detector health, appropriate camera angle, actuator health or RC
failsafe state.

## RViz assessment

RViz is receiving real data, but its presentation and frame policy need work.

Working:

- Robot description is published.
- `odom -> base_link` follows Gazebo odometry.
- `map -> odom` is supplied by `slam_toolbox` after initialization.
- `/scan` and `/map` are displayed.
- The RViz URDF contains the top lidar and front servo webcam.

Problems:

- The screenshot with the tilted drone reflected an actual badly tilted
  vehicle pose.
- TF display enables every frame, arrow and name, producing unreadable label
  clutter.
- The RViz config still contains stale link entries such as `camera_link` and
  `laser_frame` alongside the current model.
- `Fixed Frame` is `map`; before SLAM connects the trees, RViz reports
  disconnected transforms.
- The robot model is a separate URDF representation of the Gazebo SDF. It must
  be kept synchronized manually.
- No reliable trajectory `/plan` publisher was verified.
- A 2D lidar at flight altitude is only useful when arena geometry intersects
  its scan plane. It is not a general 3D mapping solution.

Relevant files:

- `src/aerothon_mission/mission_bringup/config/aerothon_slam.rviz`
- `src/aerothon_mission/uav_description/urdf/uav.urdf.xacro`
- `src/aerothon_sim/sim_gazebo/sim_gazebo/odom_tf.py`

## Simulation world: fixed test-fixture assumptions

File: `src/aerothon_sim/sim_gazebo/worlds/mission2.sdf`

The world is a handcrafted interpretation of the rulebook, not a measured
competition venue model. The following are fixed:

- spawn position
- start QR location and payload
- corridor dimensions and wall locations
- obstacle count, sizes and positions
- banner locations and geometry
- delivery-zone dimensions
- target QR locations and payload strings
- red-zone sizes and locations

QR and sign geometry is generated at launch by:

- `scripts/generate_competition_assets.py`
- `scripts/materialize_world.py`

The generated QR source PNGs are genuine and decode offline. In Gazebo they are
rendered as box geometry because texture rendering was unreliable on this
machine. Camera-distance legibility still needs validation.

## Satellite map limitations

The GCS uses real raster tiles, but the offline cache defaults to a small
radius around the ArduPilot SITL Canberra location:

```text
latitude  -35.36324
longitude 149.1652
zoom      14..19
radius    1 tile
```

File: `scripts/cache_map_tiles.py`

For the actual venue, recache using the official coordinates and a sufficient
radius. The current tile set is not a general worldwide offline map.

## MAVLink and deployment architecture

The intended architecture is:

```text
Pixhawk <-> MAVLink router on Pi
                 |-> local MAVROS UDP endpoint
                 |-> Mission Planner on laptop over Wi-Fi UDP

ROS/MAVROS on Pi -> WebSocket 8765 -> custom GCS on laptop
camera on Pi     -> MJPEG 8080    -> custom GCS on laptop
```

The custom GCS and Mission Planner are separate clients and separate data
paths. The WebSocket is not MAVLink.

Local SITL Mission Planner connection:

```text
Connection type: UDP
Listen port: 14551
```

Real Pi router example, not yet hardware-validated:

```bash
python3 scripts/mav_router.py \
  --fcu-serial /dev/ttyAMA0 --fcu-baud 921600 \
  --mavros-port 14555 \
  --gcs-port 14550 \
  --gcs-host <LAPTOP_IP> --gcs-out-port 14550
```

The Pi must own the Pixhawk serial port; MAVROS should connect to the router's
local UDP endpoint rather than opening the same serial device independently.

## Important files changed during the recent work

- `scripts/launch_level6_sim.sh`
- `scripts/materialize_vehicle_model.py`
- `scripts/materialize_world.py`
- `scripts/generate_competition_assets.py`
- `scripts/cache_map_tiles.py`
- `scripts/mav_router.py`
- `src/aerothon_sim/sim_gazebo/worlds/mission2.sdf`
- `src/aerothon_sim/sim_gazebo/config/gz_bridge.yaml`
- `src/aerothon_sim/sim_gazebo/launch/sim_full.launch.py`
- `src/aerothon_sim/sim_gazebo/sim_gazebo/odom_tf.py`
- `src/aerothon_mission/uav_description/urdf/uav.urdf.xacro`
- `src/aerothon_mission/mission_bt/mission_bt/mission_tree.py`
- `src/aerothon_mission/mission_bt/mission_bt/mav_commander.py`
- `src/aerothon_gcs/gcs_aggregator/gcs_aggregator/aggregator.py`
- `src/aerothon_gcs/tauri_app/src/App.tsx`
- `src/aerothon_gcs/tauri_app/src-tauri/src/main.rs`

`docs/STACK_AND_DEPLOYMENT.md` contains some recently updated claims that are
still too optimistic. Treat this handoff as the authoritative current-status
document until that file is corrected.

## Recommended repair order

### 1. Make the mission fail closed

- Never proceed when the start QR is empty.
- Add explicit timeout failure/abort states.
- Cancel/reset the tree on LAND, DISARM or ABORT.
- Prevent setpoint publication while disarmed outside ARMING.
- Make mission restart deterministic.
- Publish explicit success/failure reason topics.

### 2. Stabilize frames and basic flight before autonomy

- Reproduce takeoff, position hold, landing and fixed waypoint flight without
  perception or avoidance.
- Verify ENU/NED and body-frame signs with automated assertions.
- Verify the vehicle can traverse the clear corridor using position control.
- Add collision and excessive-attitude aborts.
- Do not debug perception while the camera is pointed at a wall.

### 3. Implement camera-state control

- Define named camera poses: forward banner, downward QR/search and payload
  alignment.
- Command and confirm joint position before each perception stage.
- Include camera orientation in mission readiness for that stage.

### 4. Make QR perception a gated closed loop

- Validate rendered QR decoding at 5 m and 10 m.
- Add debounce/confidence and consecutive-frame requirements.
- Fail the mission if the start payload is not decoded.
- Use target offset for visual centring.
- Add altitude-aware expected marker size checks.
- Test with noise, blur, exposure and off-axis views.

### 5. Implement real banner alignment

- Add a behaviour that waits for banner detection near the corridor entrance.
- Use horizontal error to command yaw/lateral correction.
- Require stable alignment over multiple frames.
- Consider text/logo validation so any green rectangle is not accepted.

### 6. Replace reactive obstacle stopping with navigation

- Build a corridor-specific local planner or tested state machine that selects
  pass sides and makes forward progress.
- Add lidar validity and stale-scan failsafes.
- Test every obstacle arrangement in both directions.
- Keep collision geometry and lidar scan plane representative of the real
  course.

### 7. Integrate red zones and payload hardware

- Represent red zones as exclusion polygons in the planner.
- Load verified fence data into ArduPilot and read it back.
- Make camera red detection supplementary, not the primary boundary source.
- Implement a real/simulated winch subscriber, limit handling and completion
  feedback.
- Release only after position, altitude and payload-actuator checks pass.

### 8. Correct GCS truthfulness

- Replace hardcoded EKF/geofence values with real MAVROS diagnostics.
- Populate satellite count from appropriate GPS status messages.
- Distinguish `NOT VISIBLE` from `CLEAR`.
- Mark checklist items only after verified completion.
- Display failure reasons and stale-data ages.

### 9. Clean RViz and add automated integration tests

- Disable TF names/arrows by default and whitelist useful frames.
- Remove stale link entries.
- Add a clear follow-camera view and separate SLAM/debug configuration.
- Record rosbags for every perception stage.
- Add launch tests that assert mission transitions, QR decoding, alignment,
  obstacle progress, red-zone exclusion, landing and abort behaviour.

## Definition of done before real-drone autonomous testing

Do not call the stack competition-ready until all of the following are true:

- Every mission transition has an explicit sensor/flight success condition.
- No required perception stage can time out to success.
- Clear and obstructed corridor runs pass repeatedly without collision.
- QR payload and target centring pass at required altitudes.
- Banner alignment actively changes and stabilizes vehicle pose.
- Search paths provably exclude red zones.
- Winch actuation has feedback and abort handling.
- EKF, GPS, fence, battery, RC and companion-link failsafes are real.
- RViz and GCS show only measured state, with stale/unknown clearly indicated.
- SITL tests pass repeatedly, followed by prop-off hardware tests, tethered
  tests and controlled flight-envelope expansion.

## Bottom line

Keep the current simulation and UI as development infrastructure. Do not build
new presentation polish on top of the current mission assumptions. The next
work should focus on fail-closed state management and one small verified closed
loop at a time: stable waypoint flight, camera pointing, start-QR decode,
banner alignment, corridor traversal, target centring, payload actuation and
safe return.
