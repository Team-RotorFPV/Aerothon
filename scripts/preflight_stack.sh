#!/usr/bin/env bash
# Read-only environment check. It does not start motors, SITL, or Gazebo.
set -u

# Source the ROS distro FIRST. Without this the checks below report mavros,
# ros_gz_bridge, slam_toolbox and web_video_server as missing even when they
# are installed in /opt/ros/jazzy, because `ros2 pkg prefix` only sees what is
# on AMENT_PREFIX_PATH. That produced four false failures on every run.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

# The upstream ArduPilot/Gazebo overlay. Default to a PERSISTENT location:
# building it into /tmp meant a reboot silently destroyed the simulation stack.
# scripts/install_ardupilot_overlay.sh creates this.
OFFICIAL_WS="${AEROTHON_OFFICIAL_WS:-$HOME/aerothon_stack}"
if [[ -f "$OFFICIAL_WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$OFFICIAL_WS/install/setup.bash"
  set -u
fi
if [[ -f "$OFFICIAL_WS/src/ardupilot/Tools/autotest/sim_vehicle.py" ]]; then
  export PATH="$OFFICIAL_WS/src/ardupilot/Tools/autotest:$OFFICIAL_WS/src/ardupilot/build/sitl/bin:$HOME/.local/bin:$PATH"
fi

check() {
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then
    printf 'PASS  %s\n' "$label"
  else
    printf 'FAIL  %s\n' "$label"
    FAILED=1
  fi
}

FAILED=0
check 'ROS 2 Jazzy command' command -v ros2
check 'Gazebo Harmonic command' command -v gz
check 'MAVROS package' ros2 pkg prefix mavros
check 'ros_gz bridge package' ros2 pkg prefix ros_gz_bridge
check 'slam_toolbox package' ros2 pkg prefix slam_toolbox
check 'web_video_server package' ros2 pkg prefix web_video_server
check 'ArduPilot SITL launcher (sim_vehicle.py)' command -v sim_vehicle.py
check 'ArduPilot ROS/Gazebo bringup package' ros2 pkg prefix ardupilot_gz_bringup
check 'ArduPilot Gazebo model/plugin package' ros2 pkg prefix ardupilot_gazebo

if [[ "$FAILED" -ne 0 ]]; then
  printf '\nStack is not ready for a closed-loop SITL flight. See docs/STACK_AND_DEPLOYMENT.md.\n'
  exit 1
fi

printf '\nStack prerequisites are present. Run the sensor/topic acceptance checks before arming SITL.\n'
