# Current status and deployment stack

## Current verified status

The repository builds and the Python mission / GCS unit tests pass. The local
launcher now runs an end-to-end Mission 2 SITL loop: ArduPilot Copter, the
official Iris dynamics, Gazebo Harmonic, MAVROS, the mission behaviour tree,
camera and lidar perception, SLAM/RViz, the WebSocket GCS, MJPEG video, and an
independent MAVLink stream for Mission Planner.

| Area | Status | Evidence / gap |
|---|---|---|
| Mission sequence | SITL flight-proven | The GCS `START M2` button was verified to drive `WAITING → ARMING → TAKEOFF`, arm ArduPilot in GUIDED, and climb above 4 m before a commanded safe landing. |
| Safety interlock | working baseline | The GCS rejects arm, takeoff, and start until `/mission_ready`; abort, RTL and land remain available. Real-airframe fence, RC, battery, EKF and companion-loss parameters still require prop-off and flight-test sign-off. |
| Gazebo world | running | The rebuilt rulebook fixture contains the clear outbound corridor, obstructed return corridor, generated QR targets, separate red zones, and AEROTHON banner. |
| Vehicle and sensors | running | The upstream ArduPilot Iris dynamics are retained. The old gimbal is removed; a visible top RPLidar C1 and front webcam on one pitch servo publish `/scan` and `/camera/image`. |
| RViz / SLAM | running | RViz receives the robot model, sensor TF, `/scan`, annotated camera data and a live `slam_toolbox` occupancy grid. |
| GCS | flight-proven in SITL | Three tabs, connection state, no auth token, satellite tiles, camera pitch commands and START M2 were exercised against the live backend. |

The rulebook does require the autonomous chain represented above: start QR at
approximately 5 m, green-banner alignment, a 3.5 m autonomous corridor at
approximately 3 m, 10 m target QR detection, red-zone avoidance, payload
release from approximately 5 m, return through the corridor, and landing. It
also says the actual geofence coordinates arrive in Phase 2. Treat the world
dimensions and QR placements here as test fixtures until those coordinates,
marker size, and obstacle geometry are received.

## Approved architecture

```mermaid
flowchart LR
  FC["Pixhawk 6X\nArduPilot Copter"] <-->|"MAVLink serial"| PI["Raspberry Pi 5\nUbuntu 24.04 + ROS 2 Jazzy"]
  CAM["Brio / simulated RGB camera"] --> PI
  LIDAR["RPLidar C1 / simulated lidar"] --> PI
  PI -->|"MAVROS setpoints"| FC
  PI -->|"WebSocket 8765 + MJPEG 8080"| GCS["Laptop tactical GCS"]
  RC["ELRS safety pilot"] --> FC
  SITL["ArduPilot SITL + ardupilot_gz\nGazebo Harmonic"] -->|"same MAVROS topics"| PI
```

Use one code path above MAVROS. `use_sim:=true` changes clocks and camera / lidar
sources; it must not change mission policy. The RC link, FC geofence, battery
failsafe, RTL, and companion-loss failsafe are independent of ROS and take
priority over the GCS.

| Layer | Simulation | Competition aircraft |
|---|---|---|
| Vehicle dynamics | ArduPilot SITL + official `ardupilot_gz` Harmonic vehicle | Pixhawk 6X, current Copter stable |
| Interface | MAVROS ROS 2 Jazzy | MAVROS ROS 2 Jazzy over `/dev/ttyAMA0:921600` or USB serial |
| Camera | SDF camera publishing `/image_raw` | Logitech Brio via `v4l2_camera` / `usb_cam` publishing the same topic |
| Range sensing | SDF lidar publishing `/scan` | RPLidar C1 ROS 2 driver publishing the same topic |
| Perception | OpenCV QR / banner / red-zone nodes | Identical nodes, calibrated with logged Brio data |
| Avoidance | Body-frame velocity controller, then costmap validation | Same, with ArduPilot proximity avoidance as a separately tested fallback |
| Mission control | `mission_bt` → MAVROS GUIDED setpoints | Identical |
| Operator link | localhost WebSocket / MJPEG | Pi Wi-Fi AP or Ethernet: `ws://<pi-ip>:8765`, `http://<pi-ip>:8080` |
| Safety | SITL fence and fault injection | FC fence, RC failsafe, RTL, battery/EKF failsafe, hardware kill policy |

## Connecting the GCS

1. Put the laptop and Pi on the same isolated flight network. The Pi should use
   a stable address (for example `192.168.4.1`). Keep ROS DDS local to the Pi.
2. Run MAVROS, the WebSocket aggregator (8765), MJPEG server (8080), and
   MAVLink router on the Pi. Run the custom GCS frontend only on the laptop.
3. Start the laptop GCS, enter
   `ws://192.168.4.1:8765` in the GCS connection field and press **CONNECT**.
   The Video Feed uses `http://192.168.4.1:8080/stream?topic=/percep/qr/annotated&type=mjpeg`
   automatically. In local SITL use `ws://127.0.0.1:8765`.
4. Configure the Pi router with `AEROTHON_MISSION_PLANNER_IP=<laptop-ip>`.
   Mission Planner listens for the independent MAVLink stream on UDP 14550.
5. Wait for every interlock item to be green. Only then are ARM, TAKEOFF 5M,
   and START M2 enabled. START M2 is deliberately separate from takeoff.

For the real aircraft, let the Pi router own the Pixhawk telemetry serial port
and fan the stream out to both MAVROS and the laptop. Replace both example IP
and serial device with the values on the aircraft:

```bash
python3 scripts/mav_router.py \
  --fcu-serial /dev/ttyAMA0 --fcu-baud 921600 \
  --mavros-port 14555 \
  --gcs-port 14550 \
  --gcs-host 192.168.4.2 --gcs-out-port 14550

# MAVROS consumes the router's independent local endpoint.
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14555@127.0.0.1:14556
```

Mission Planner on `192.168.4.2` selects **UDP**, listens on port **14550**,
and does not use the custom GCS WebSocket. The laptop custom GCS separately
connects to `ws://192.168.4.1:8765`; its video tab uses Pi port 8080.

## Test ladder

Run this first in a Bash shell (the preflight uses the active ROS environment):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/preflight_stack.sh
```

The official overlay is installed on this development machine at
`/tmp/ardupilot_stack2`. The launcher and preflight auto-source that path; set
`AEROTHON_OFFICIAL_WS` to the overlay path used on another machine.

```bash
# Existing headless checks — no simulator claim
python3 sim/test_behavior_tree.py
python3 sim/test_gcs_aggregator.py
python3 sim/check_mission.py

# Validate the course SDF loads (world only; it does not prove flight)
gz sim -s src/aerothon_sim/sim_gazebo/worlds/mission2.sdf
```

The installed closed-loop baseline materializes the maintained upstream
`iris_with_gimbal` airframe, removes its old gimbal camera, and adds the
competition sensor layout. Verified ROS topics include `/camera/image`,
`/camera/camera_info`, `/scan`, `/imu`, `/odometry`, and `/map`. The optional
GStreamer RTMP plugin and micro-ROS agent are not required for this
MAVLink/MAVROS path; the camera is bridged natively by Gazebo/ROS.
Prove each command before moving to the next:

1. SITL connects to MAVROS and maintains GUIDED loiter.
2. GCS receives state and accepts commands without an auth token on the isolated flight network.
3. Camera shows a decodable QR at the required stand-off; record a rosbag.
4. Lidar and TF render in RViz; avoidance stops before a wall.
5. Each nominal leg and each abort / RC / battery / companion-loss fault is
   replayed in SITL.
6. Repeat from a tethered, prop-off airframe, then controlled outdoor tests.

## Competition gates before flight

- Replace all fixture QR images with high-resolution printable QR codes using
  the organizer's payload identifiers and measured marker dimensions.
- Load and read-back verify the official inclusion and red-zone exclusion
  polygons in ArduPilot before every attempt.
- Implement and bench-test winch encoder/current ground detection. A fixed
  tick counter is not a valid release condition.
- Log `.bin`, `.tlog`, rosbag, GCS event stream, and annotated video to a
  time-synchronised NVMe.
- Conduct range, RC override, battery, EKF, geofence, and companion-loss tests
  at a safe test site. A desktop GCS is never the sole flight-safety control.

## References

The official ArduPilot Gazebo plugin supports Harmonic and supplies the SITL ↔
Gazebo connection; use it instead of a hand-made, unvalidated vehicle plugin:
[ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo).
ArduPilot's ROS 2 guide documents the supported `ardupilot_gz_bringup`
workflow and its `iris_runway.launch.py` smoke test:
[ROS 2 with Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html).
