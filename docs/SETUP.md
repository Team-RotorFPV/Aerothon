# Version-locked setup — AeroTHON 2026 Mission 2

The whole point of this file: **a known-good version matrix** so the team doesn't lose a week to dependency hell. Everything below is chosen to be mutually compatible with **ROS 2 Jazzy on Ubuntu 24.04**.

## 1. Version matrix (pin these)

| Component | Version | Notes |
|---|---|---|
| Ubuntu | **24.04 LTS (Noble)** | Native home for Jazzy; boots on Pi 5 |
| ROS 2 | **Jazzy Jalisco** | LTS to 2029 |
| Gazebo | **Harmonic (gz-sim 8)** | Official Jazzy pairing via `ros_gz` |
| ArduPilot Copter | **4.5.x or 4.6.x stable** | SITL + real; supports Lua, GUIDED, polygon fence |
| MAVROS | **ros-jazzy-mavros(-extras)** | Exposes MAVLink as ROS 2 (ENU) |
| ardupilot_gz | **main (pin tested commit)** | Official ROS 2 SITL↔Gazebo Harmonic bringup |
| slam_toolbox | **ros-jazzy-slam-toolbox** | 2D SLAM |
| Nav2 | **ros-jazzy-navigation2** | for `costmap_2d` (we don't use the full planner) |
| rplidar driver | **rplidar_ros (ros2 branch)** | supports C1 |
| web_video_server | **ros-jazzy-web-video-server** | MJPEG for GCS |
| py_trees | **py_trees + py_trees_ros (jazzy)** | behavior tree |
| Rust | **stable (rustup)** | Tauri backend |
| Tauri | **2.x** | desktop shell |
| Node | **20 LTS** | React build |
| React | **18** | UI |
| Charts / Map | **ECharts 5**, **MapLibre GL JS 4**, **PMTiles** | offline map |
| Type sync | **ts-rs** | Rust→TS codegen |

> ⚠️ **Jetson note (not chosen):** JetPack 6 is Ubuntu 22.04, so it does *not* natively run Jazzy. Pi 5 was chosen partly to avoid this. If you ever add a Jetson, plan for containers.

## 2. Native install (Pi 5 / dev box on 24.04)

```bash
# ROS 2 Jazzy
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Core packages
sudo apt install -y \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-costmap-2d \
  ros-jazzy-web-video-server ros-jazzy-ros-gz \
  python3-vcstool python3-colcon-common-extensions

# Tauri GCS dependencies (for native Linux build)
sudo apt install -y \
  libwebkit2gtk-4.1-dev libappindicator3-dev librsvg2-dev patchelf \
  build-essential pkg-config libssl-dev

# MAVROS GeographicLib datasets (required, one-time)
ros2 run mavros install_geographiclib_datasets.sh   # or the packaged script
```

ArduPilot SITL + Gazebo Harmonic + `ardupilot_gz` follow the upstream ROS 2 guide (pin a tested commit before flight). Prefer the **Docker path below** for reproducibility.

## 3. Docker (recommended — reproducible across the team)

```bash
cd docker
docker compose build
xhost +local:docker            # allow GUI (X11)
docker compose up sim          # SITL + Gazebo + MAVROS
# in another shell:
docker compose up ros          # perception / avoidance / mission nodes
```

See [`../docker/`](../docker) for the `Dockerfile` and `docker-compose.yml`. GPU passthrough (`--gpus`, nvidia-container-toolkit) is wired for NVIDIA dev boxes; the Pi 5 uses CPU rendering.

## 4. Workspace bootstrap (vcstool)

```bash
mkdir -p ~/aerothon_ws/src && cd ~/aerothon_ws
vcs import src < /path/to/aerothon.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 5. Quick smoke tests

```bash
# 1. SITL + Gazebo world
ros2 launch sim_gazebo mission2_world.launch.py

# 2. MAVROS connected?
ros2 topic echo /mavros/state          # expect connected: true

# 3. Full sim mission (headless CI variant)
ros2 launch mission_bringup mission2.launch.py use_sim:=true

# 4. GCS aggregator up?
ros2 run gcs_aggregator aggregator      # then connect the Tauri app / a WS client
```
