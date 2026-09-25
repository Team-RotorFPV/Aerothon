# Run & Check — AeroTHON 2026 Mission 2

> **Status note (2026-08-15):** Levels 1–2 below are development checks.
> The checked-in world does not include an ArduPilot vehicle, camera, or lidar,
> so Levels 3–6 are a target integration plan, not evidence of a working
> closed-loop simulator. Use [STACK_AND_DEPLOYMENT.md](STACK_AND_DEPLOYMENT.md)
> for the current, safe run sequence and GCS connection instructions.

Complete verification ladder from zero-dependency headless unit tests up to the full multi-process simulation loop with Gazebo Harmonic, RViz2 SLAM, MAVLink Routing, Mission Planner, and the custom Tactical GCS.

---

## Level 1 — Mission Logic & Control Law (No ROS / Headless)
Runs the corridor-centering control law + mission state machine and asserts rulebook success criteria.

```bash
python3 sim/check_mission.py      # Assertions check (PASS/FAIL)
python3 sim/test_behavior_tree.py # Behavior tree unit tests (7/7 PASS)
python3 sim/test_gcs_aggregator.py# GCS aggregator command dispatch tests (6/6 PASS)
```
*Expected*: `RESULT: PASS ✅` (Lands, reaches target within 0.09 m, zero red-zone incursions).

---

## Level 2 — GCS Dynamic Interface & Video Stream (development replay only)
Opens the real-time, dark monochrome tactical GCS interface.

```bash
python3 -m http.server 8899 -d sim
# In browser, visit: http://localhost:8899/gcs_preview.html
```
Features in the replay preview:
- Primary Flight Display (HUD) with artificial horizon, pitch ladder, heading tape, altitude/airspeed tapes
- Dynamic Mission Planner GPS map (unknown arena mode, breadcrumbs, live heading vector)
- Continuous live camera stream (MJPEG from `web_video_server` on port 8080)
- 2D SLAM occupancy grid & Nav2 costmap inflation safety layer

---

## Level 3 — ArduPilot SITL Flight (MAVROS Guided Mode; prerequisite not bundled)
Tests the flight controller interface against ArduPilot SITL in GUIDED mode.

```bash
# Terminal 1: Start SITL
sim_vehicle.py -v ArduCopter --console --map

# Terminal 2: Start MAVROS + Mission Tree
source /opt/ros/jazzy/setup.bash
ros2 launch mission_bringup mission2.launch.py use_sim:=true fcu_url:=udp://127.0.0.1:14550@
```

---

## Level 4 — Gazebo Harmonic + ROS Bridge Sensor Loop (not yet complete)
Spawns the Mission 2 arena (corridor walls, obstacle bumps, delivery zone, QR codes) and bridges `/scan`, `/image_raw`, and `/clock`.

```bash
# Terminal 1: SITL with Gazebo frame
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console

# Terminal 2: Gazebo World + Parameter Bridge
ros2 launch sim_gazebo mission2_world.launch.py
```

---

## Level 5 — Online 2D SLAM (`slam_toolbox`) & RViz 2 Visualizations (requires Level 4)
Brings up `slam_toolbox` asynchronous 2D mapping and RViz2 configured with custom displays.

```bash
ros2 launch mission_bringup mission2.launch.py use_sim:=true rviz:=true slam:=true
```
*RViz Visualizations*:
- `/robot_description`: Full 3D quad frame model (9″ quad + RPLidar C1 + Brio camera)
- `/map`: Live discovered 2D occupancy grid
- `/scan`: Live laser returns colored by range and intensity
- `/local_costmap/costmap`: Dynamic obstacle inflation safety zone
- `/percep/qr/annotated`: Real-time annotated camera stream

---

## Level 6 — Complete Integrated Master Simulation (target; do not use as acceptance evidence)
Brings up the entire ecosystem in one coordinated workflow:
1. **MAVLink Router**: Splits FCU telemetry to MAVROS (UDP 14555) and Mission Planner / QGC (UDP 14550).
2. **Gazebo Harmonic Window**: Full Mission 2 3D world with physical sensors.
3. **RViz 2 Window**: Full SLAM, costmap, TF, and camera stream displays.
4. **Mission Planner / QGC Access**: Connect directly to `udp:127.0.0.1:14550`.
5. **Tactical GCS**: Live at `http://localhost:8899/gcs_preview.html`.

### Run All-in-One:
```bash
./scripts/launch_level6_sim.sh
```

### Or Run in Separate Terminals:
```bash
# Terminal 1: MAVLink Router (MAVROS + Mission Planner multiplexer)
python3 scripts/mav_router.py --fcu-in 14560 --mavros-port 14555 --gcs-port 14550

# Terminal 2: ArduPilot SITL
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14560

# Terminal 3: Master Gazebo + ROS 2 + SLAM + RViz + GCS Launch
source /opt/ros/jazzy/setup.bash
ros2 launch sim_gazebo sim_full.launch.py fcu_url:=udp://127.0.0.1:14555@ rviz:=true slam:=true

# Terminal 4 (Optional): Open Mission Planner
# In Mission Planner: Select UDP -> Port 14550 -> Connect
```
