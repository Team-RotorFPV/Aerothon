#!/usr/bin/env bash
set -euo pipefail
deploy=/home/pi5/aerothon-deploy
workspace=/home/pi5/aerothon_ws
source "$deploy/runtime.env"
mode=${1:?console, mission or check required}
if [[ $mode == web ]] && [[ $(docker inspect -f '{{.State.Running}}' aerothon-runtime 2>/dev/null || true) == true ]]; then
    exec docker attach --sig-proxy=false aerothon-runtime
fi
args=(run --rm --init --network host --user 1000:1000 --cap-drop ALL
      --security-opt no-new-privileges --group-add 20 --group-add 44
      --env ROS_LOCALHOST_ONLY=1 --env ROS_DOMAIN_ID=42
      --env ROS_LOG_DIR=/home/aerothon/state/roslogs
      --mount "type=bind,src=$workspace,dst=/workspace"
      --mount "type=bind,src=$deploy,dst=/deployment,readonly"
      --mount "type=bind,src=/home/pi5/.aerothon,dst=/home/aerothon/.aerothon"
      --mount "type=bind,src=/home/pi5/.local/state/aerothon,dst=/home/aerothon/state")
for device in "$AEROTHON_CAMERA_DEVICE" "$AEROTHON_LIDAR_DEVICE"; do
    [[ ! -c $device ]] || args+=(--device "$device")
done
# --device only grants nodes present when the container starts (at boot).
# A camera or lidar plugged in later, or a profile naming another port, was
# then invisible to the runtime. Allow USB-serial (188) and V4L2 video (81)
# by class and expose /dev/serial plus the video nodes through a /dev view.
if [[ $mode == web ]]; then
    # The host /dev view would also share the host's /dev/shm, where Fast DDS
    # then failed to lock its shared-memory ports ("open_and_lock_file
    # failed"); keep the container's own private /dev/shm as before.
    args+=(--device-cgroup-rule 'c 188:* rmw' --device-cgroup-rule 'c 81:* rmw'
           --mount type=bind,src=/dev,dst=/dev,bind-propagation=rslave
           --mount type=tmpfs,dst=/dev/shm,tmpfs-size=268435456)
fi
# The GPIO winch (BTS7960 + N20 encoder) needs the GPIO character devices;
# granted only when runtime.env selects it, with the devices' own group.
if [[ ${AEROTHON_WINCH_BACKEND:-mavlink} == gpio ]]; then
    for chip in /dev/gpiochip*; do
        [[ -c $chip ]] || continue
        args+=(--device "$chip" --group-add "$(stat -c %g "$chip")")
    done
fi
if [[ -n ${AEROTHON_DELIVERY_CALIBRATION:-} ]]; then
    [[ $AEROTHON_DELIVERY_CALIBRATION = /home/pi5/.aerothon/* ]] || {
        echo 'Put the measured load profile under /home/pi5/.aerothon.' >&2; exit 1;
    }
    args+=(--env "AEROTHON_DELIVERY_CALIBRATION=/home/aerothon/.aerothon/${AEROTHON_DELIVERY_CALIBRATION#/home/pi5/.aerothon/}")
fi
case $mode in
    web) docker rm aerothon-runtime 2>/dev/null || true
        args+=(--name aerothon-runtime --sig-proxy=false --env AEROTHON_DEVICE_STATE_DIR=/home/aerothon/.aerothon --env AEROTHON_STATE_DIR=/home/aerothon/state)
        command='source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; while true; do ros2 run aerothon_runtime web_host --static /workspace/gcs --state-dir /home/aerothon/state; sleep 2; done' ;;
    console) args+=(--name aerothon-console --stop-signal SIGINT --stop-timeout 30)
        command='source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; exec ros2 launch /deployment/console.launch.py' ;;
    mission) echo 'Select a connection and reviewed Start in the Pi GCS; the managed runtime owns flight.' >&2; exit 1 ;;
    check) command='bash /workspace/scripts/check_pi_runtime.sh' ;;
    *) echo 'Unknown mode' >&2; exit 2 ;;
esac
exec /usr/bin/docker "${args[@]}" "$AEROTHON_IMAGE" /bin/bash -c "$command"
