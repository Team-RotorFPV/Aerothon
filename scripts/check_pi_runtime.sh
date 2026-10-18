#!/usr/bin/env bash
# Read-only aircraft prerequisite check. Does not launch the stack or actuators.
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "$root/install/setup.bash" ]] && source "$root/install/setup.bash"
set -u
failed=0
check() {
    local label="$1"; shift
    if "$@" >/dev/null 2>&1; then printf 'PASS %s\n' "$label";
    else printf 'FAIL %s\n' "$label"; failed=1; fi
}
for package in aerothon_runtime aerothon_slam_grid mission_bringup mission_bt uav_description avoidance winch_ctrl camera_ctrl perception_qr perception_banner perception_redzone perception_overlay gcs_aggregator mavros usb_cam ldlidar_stl_ros2 sllidar_ros2 web_video_server robot_state_publisher xacro slam_toolbox nav2_lifecycle_manager; do
    check "ROS package: $package" ros2 pkg prefix "$package"
done
for module in cv_bridge cv2 numpy scipy py_trees websockets aiohttp pyzbar.pyzbar serial; do
    check "Python import: $module" python3 -c "import $module"
done
check 'OCR confirming path: tesseract' command -v tesseract
check 'MAVROS geoid data' test -d /usr/share/GeographicLib/geoids
web_root="$root/gcs"
[[ -f "$web_root/index.html" ]] || web_root="$root/src/aerothon_gcs/tauri_app/dist"
check 'Built revised GCS' test -f "$web_root/index.html"
printf '\nThis checks software availability only. The GPIO winch/encoder backend refuses to run until its encoder counts and spool radius are measured.\n'
printf 'Camera/lidar wiring, actuator calibration, loaded hover and failsafes require hardware evidence.\n'
exit "$failed"
