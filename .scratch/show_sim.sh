#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=aerothon_m2
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="/tmp/aerothon_vehicle_models:/home/sarthak/ardupilot_gazebo/models:/home/sarthak/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/sarthak/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
/usr/bin/python3 scripts/materialize_vehicle_model.py --source /home/sarthak/ardupilot_gazebo/models/iris_with_gimbal/model.sdf --output-root /tmp/aerothon_vehicle_models
/usr/bin/python3 scripts/materialize_world.py --source src/aerothon_sim/sim_gazebo/worlds/mission2.sdf --assets src/aerothon_sim/sim_gazebo/materials --output /tmp/aerothon_mission2_runtime.sdf
nohup /usr/bin/python3 -m http.server 8899 --directory src/aerothon_gcs/tauri_app/dist >/tmp/aerothon_gcs_web.log 2>&1 </dev/null &
nohup bash .scratch/open_gazebo.sh >/tmp/aerothon_gazebo_visible.log 2>&1 </dev/null &
echo "Gazebo PID: $!"
wait
