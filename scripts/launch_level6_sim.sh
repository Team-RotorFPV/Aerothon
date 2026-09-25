#!/usr/bin/env bash
# ==============================================================================
# AeroTHON 2026 Mission 2 — Full Level 6 Simulation Master Launcher
# ==============================================================================
# Launches:
#   1) Bi-directional MAVLink Router (routes to MAVROS + Mission Planner / QGC)
#   2) ArduPilot Copter SITL (Gazebo Iris model)
#   3) Gazebo Harmonic GUI (Mission 2 Arena, corridor, obstacles, QR targets)
#   4) Full ROS 2 Stack (MAVROS, QR detector, Banner detector, Avoidance, BT)
#   5) slam_toolbox 2D SLAM Mapping node
#   6) RViz 2 Visualizer with SLAM, PointCloud, TF, and Camera stream
#   7) Web Video Server (MJPEG camera feed on port 8080)
#   8) GCS WebSocket Aggregator (port 8765) + Web GCS (port 8899)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Keep every Gazebo Transport participant on the same local discovery domain.
# This machine has multiple network interfaces; leaving discovery implicit can
# make gz-server healthy while the GUI and ROS spawner see zero services.
export GZ_PARTITION="${GZ_PARTITION:-aerothon_m2}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

# Keep ROS 2 DDS discovery on the loopback interface.
#
# This matches the locked architecture: DDS stays onboard the aircraft, and the
# GCS talks to it over its own WebSocket, never over DDS. On a machine with
# several network interfaces it also keeps discovery traffic off the wire.
#
# It is NOT a fix for the intermittent start-up aborts of mavros_node and
# robot_state_publisher. That was tried and did not help. The aborts are
#
#   signal_handler(SIGINT/SIGTERM)
#   what(): failed to initialize rcl node: the given context is not valid
#
# i.e. the process is SIGTERMed while still initialising and then tries to
# finish constructing nodes on a dead context — a symptom of the stack being
# torn down mid-start-up, not a DDS problem. Recorded here because that cause
# was misattributed three times (to the stream-rate keeper, to stale shared
# memory, and to discovery range) before being read off the log properly.
#
# Set AEROTHON_OPEN_DDS=1 if you ever need multi-machine ROS.
# OFF BY DEFAULT. This was added on the (wrong) hypothesis that open discovery
# caused the start-up aborts. It did not — the real cause was a missing
# setup.cfg in mission_bringup, so console scripts landed in bin/ instead of
# lib/, ros2 launch threw "libexec directory does not exist", and aborted the
# whole launch by SIGINTing every process it had started.
#
# Worse, enabling it ISOLATES the stack from any shell that does not set the
# same variables, so `ros2 topic list` from a normal terminal sees nothing.
# That is a bad default for a simulator people poke at by hand.
if [[ "${AEROTHON_LOCALHOST_DDS:-0}" == "1" ]]; then
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    echo "[DDS] discovery confined to localhost (set AEROTHON_LOCALHOST_DDS=0 to open)"
fi

echo "======================================================================"
echo "    AEROTHON 2026 MISSION 2 (SKYSCAN) — LEVEL 6 SIMULATION"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 0. Reap any stack left running by a previous invocation.
#
# A crashed or Ctrl-Z'd run leaves gz-server, arducopter and the router holding
# their UDP ports and the Gazebo partition, so the next launch half-connects and
# reports confusing failures. A previous session left PGID 176131 alive for
# hours exactly this way. Set AEROTHON_NO_REAP=1 to skip.
# ------------------------------------------------------------------------------
STACK_PGID_FILE="${AEROTHON_PGID_FILE:-/tmp/aerothon_stack.pgid}"
# Deliberately does NOT match 'launch_level6_sim': that pattern also matches
# this script, any shell wrapping it, and any editor/CI process whose command
# line merely mentions it. An earlier version killed its own parent shell.
# Previous *launcher* instances are handled by the recorded PGID above; this
# pattern is only for orphaned CHILD processes whose group is already gone.
REAP_PATTERN='gz sim|arducopter|mav_router\.py|sim_full\.launch\.py'

# Our own process group. Everything we launch inherits it, so excluding it is
# what stops the reaper from killing the very run it is preparing — matching on
# the command pattern alone also matches THIS script.
SELF_PGID="$(ps -o pgid= -p $$ | tr -d ' ')"

# Stragglers = processes matching the pattern that are NOT in our process group.
find_stragglers() {
    ps -eo pid=,pgid=,args= 2>/dev/null \
        | awk -v self="$SELF_PGID" -v pat="$REAP_PATTERN" \
              '$2 != self && $0 ~ pat { print $1 }'
}

reap_previous_stack() {
    local reaped=0

    if [[ -f "$STACK_PGID_FILE" ]]; then
        local old_pgid
        old_pgid="$(cat "$STACK_PGID_FILE" 2>/dev/null || true)"
        if [[ -n "$old_pgid" && "$old_pgid" =~ ^[0-9]+$ && "$old_pgid" != "$SELF_PGID" ]]; then
            if kill -0 -- "-$old_pgid" 2>/dev/null; then
                echo "[REAP] Terminating previous stack process group $old_pgid..."
                kill -TERM -- "-$old_pgid" 2>/dev/null || true
                reaped=1
            fi
        fi
    fi

    # Belt and braces: catch orphans whose process group is already gone.
    local stragglers
    stragglers="$(find_stragglers)"
    if [[ -n "$stragglers" ]]; then
        echo "[REAP] Terminating orphaned simulation processes: $(echo "$stragglers" | tr '\n' ' ')"
        # shellcheck disable=SC2086
        kill -TERM $stragglers 2>/dev/null || true
        reaped=1
    fi

    if [[ "$reaped" == "1" ]]; then
        for _attempt in $(seq 1 30); do
            [[ -z "$(find_stragglers)" ]] && break
            sleep 0.2
        done
        stragglers="$(find_stragglers)"
        if [[ -n "$stragglers" ]]; then
            echo "[REAP] Forcing remaining processes down."
            # shellcheck disable=SC2086
            kill -KILL $stragglers 2>/dev/null || true
            sleep 1
        fi
        echo "[OK] Previous stack reaped."
    fi
}

# Stale FastDDS shared-memory segments.
#
# Every killed stack leaves segments behind in /dev/shm. They accumulated to 86
# over one session, and the runs that failed with mavros_node and
# robot_state_publisher aborting (SIGABRT) were the ones with the largest
# backlog. Participants can fail to attach to a stale segment left by a process
# that no longer exists, and the failure surfaces as an abort in an unrelated
# node, which is a genuinely confusing way to lose an afternoon.
#
# Only safe once nothing is running, so it goes after the reap.
clean_stale_dds_shm() {
    local n
    n="$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' -o -maxdepth 1 -name 'sem.fastrtps_*' 2>/dev/null | wc -l)"
    if [[ "$n" -gt 0 ]]; then
        if pgrep -f "$REAP_PATTERN" >/dev/null 2>&1; then
            echo "[SHM] $n stale segments present but processes still running; skipping."
            return
        fi
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
        echo "[SHM] Cleared $n stale FastDDS segment(s)."
    fi
}

if [[ "${AEROTHON_NO_REAP:-0}" != "1" ]]; then
    reap_previous_stack
    clean_stale_dds_shm
fi

# Record our real process group. $$ is only the PGID when this script happens
# to be a group leader, which is true from an interactive shell but not when
# it is invoked from another script or a CI runner.
STACK_PGID="$SELF_PGID"
echo "$STACK_PGID" > "$STACK_PGID_FILE"
echo "[OK] This stack's process group: $STACK_PGID (recorded in $STACK_PGID_FILE)"
echo "     Stop it with:  kill -TERM -- -$STACK_PGID"

# 1. Source ROS 2 Environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy (/opt/ros/jazzy/setup.bash)"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    echo "[OK] Sourced ROS 2 Iron (/opt/ros/iron/setup.bash)"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] Sourced ROS 2 Humble (/opt/ros/humble/setup.bash)"
else
    echo "[WARN] No global ROS 2 setup.bash found in /opt/ros/. Ensure ROS 2 is in your environment."
fi

# Source local workspace if built
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "[OK] Sourced local workspace ($WORKSPACE_ROOT/install/setup.bash)"
fi

# The maintained ArduPilot/Gazebo vehicle layer is built in a separate overlay
# because it is upstream code, not part of this mission repository.  Allow the
# path to be overridden for deployment machines.
OFFICIAL_WS="${AEROTHON_OFFICIAL_WS:-$HOME/aerothon_stack}"
if [ -f "$OFFICIAL_WS/install/setup.bash" ]; then
    source "$OFFICIAL_WS/install/setup.bash"
    echo "[OK] Sourced official ArduPilot/Gazebo overlay ($OFFICIAL_WS)"
fi
if [ -f "$OFFICIAL_WS/src/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
    export PATH="$OFFICIAL_WS/src/ardupilot/Tools/autotest:$OFFICIAL_WS/src/ardupilot/build/sitl/bin:$HOME/.local/bin:$PATH"
fi

# package:// resources inside the official Iris model need the parent of the
# package share directory when the model is preloaded directly by gz-server.
ARDUPILOT_GAZEBO_PREFIX="$(ros2 pkg prefix ardupilot_gazebo 2>/dev/null || true)"
if [ -n "$ARDUPILOT_GAZEBO_PREFIX" ]; then
    export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_PREFIX/share:${GZ_SIM_RESOURCE_PATH:-}"
    export SDF_PATH="$ARDUPILOT_GAZEBO_PREFIX/share:$GZ_SIM_RESOURCE_PATH:${SDF_PATH:-}"
fi

# Do not turn a missing vehicle/sensor stack into a misleading "Level 6" run.
# The official ardupilot_gz bringup and its sensor-equipped vehicle are an
# explicit prerequisite; the local course world alone is only visual geometry.
if ! command -v sim_vehicle.py >/dev/null 2>&1 || ! ros2 pkg prefix ardupilot_gz_bringup >/dev/null 2>&1; then
    echo "[BLOCKED] Closed-loop SITL is not installed (sim_vehicle.py and ardupilot_gz_bringup are required)."
    echo "[BLOCKED] See docs/STACK_AND_DEPLOYMENT.md before attempting a flight simulation."
    exit 2
fi

# 2. Cleanup on Exit
PIDS=()
CLEANING_UP=0
cleanup() {
    if [[ "$CLEANING_UP" == "1" ]]; then
        return
    fi
    CLEANING_UP=1
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "[*] Shutting down simulation and background services..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    for _attempt in $(seq 1 20); do
        any_alive=0
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                any_alive=1
            fi
        done
        [[ "$any_alive" == "0" ]] && break
        sleep 0.1
    done
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    rm -f "$STACK_PGID_FILE"
    echo "[OK] All services stopped."
}
trap cleanup SIGINT SIGTERM EXIT

# 3. Start MAVLink Router (Splits telemetry to MAVROS & Mission Planner)
if [[ -n "${AEROTHON_MISSION_PLANNER_IP:-}" ]]; then
    MP_HOST="$AEROTHON_MISSION_PLANNER_IP"
    MP_PORT="${AEROTHON_MISSION_PLANNER_PORT:-14550}"
else
    # Same-laptop SITL: 14550 is occupied by the router, so Mission Planner
    # listens on 14551. On the Pi set AEROTHON_MISSION_PLANNER_IP to the
    # laptop's Wi-Fi address; its standard destination port is then 14550.
    # Same-laptop SITL: MAVProxy already delivers a direct stream to 14551,
    # which is what Mission Planner should listen on. Send the router's own
    # GCS fan-out to a different port so the two paths do not duplicate.
    MP_HOST="127.0.0.1"
    MP_PORT="${AEROTHON_MISSION_PLANNER_PORT:-14553}"
fi
# Router FCU input must match where the SITL side actually sends MAVLink.
# ardupilot_gz's robot.launch.py computes mavlink_out = 14550 + port_offset and
# hands it to MAVProxy as its first --out, so with instance 0 the stream lands
# on 127.0.0.1:14550 — not the 14560 this script used to assume. Nothing fed
# 14560, the router reported "0 FCU endpoints active", and MAVROS sat at
# connected:false with no error anywhere.
#
# MAVProxy additionally hardcodes a second --out to 127.0.0.1:14551, which is
# what Mission Planner connects to for SITL. The router therefore keeps its own
# GCS ports clear of both.
# 14560 is SITL SERIAL1, given to the router by sim_full.launch.py.
# 14550 is MAVProxy's --out and is deliberately NOT used: streams
# requested through MAVProxy are reset to its 4 Hz default.
FCU_IN_PORT="${AEROTHON_FCU_IN_PORT:-14560}"
ROUTER_GCS_PORT="${AEROTHON_ROUTER_GCS_PORT:-14552}"

echo "[1/5] Starting MAVLink Router (FCU in ${FCU_IN_PORT}; MAVROS 14555; GCS -> ${MP_HOST}:${MP_PORT})..."
python3 "$SCRIPT_DIR/mav_router.py" --fcu-in "$FCU_IN_PORT" --mavros-port 14555 \
    --gcs-port "$ROUTER_GCS_PORT" --gcs-host "$MP_HOST" --gcs-out-port "$MP_PORT" &
PIDS+=($!)
sleep 1

# 4. Start GCS Web Server
echo "[2/5] Starting built three-tab GCS on http://localhost:8899..."
python3 -m http.server 8899 -d "$WORKSPACE_ROOT/src/aerothon_gcs/tauri_app/dist" >/dev/null 2>&1 &
PIDS+=($!)

echo "======================================================================"
echo " [MISSION PLANNER CONNECTION]"
echo "   -> Open Mission Planner / QGroundControl"
if [[ -n "${AEROTHON_MISSION_PLANNER_IP:-}" ]]; then
    echo "   -> Mission Planner: UDP listen on ${MP_PORT} (router target ${MP_HOST}:${MP_PORT})"
else
    echo "   -> Mission Planner (SITL): UDP listen on 14551  <- direct from MAVProxy"
    echo "   -> Router GCS fan-out also available on ${MP_HOST}:${MP_PORT}"
fi
echo "   -> Web GCS URL: http://localhost:8899/"
echo "   -> Live Video : http://localhost:8080/stream?topic=/percep/qr/annotated"
echo "======================================================================"

# Open the GCS automatically alongside Gazebo and RViz.
#
# Under WSL, Brave has no working Vulkan surface path, so it needs to be forced
# onto XWayland with GPU rendering off or it never paints. Those two flags were
# applied unconditionally, and on a NATIVE Wayland session they cause the very
# failure they were added to prevent: --ozone-platform=x11 pushes a Wayland-
# native browser through XWayland and --disable-gpu takes away the compositing
# path it then needs, giving a window that opens, loads the page and renders
# nothing. The GCS was serving correctly the whole time -- the same URL in any
# other browser showed live telemetry -- so the blank window read as a broken
# frontend rather than a browser flag.
#
# Gate them the way the Gazebo GUI renderer above is gated: WSL only.
# Set AEROTHON_OPEN_GCS=0 for headless launches.
if [[ "${AEROTHON_OPEN_GCS:-1}" == "1" ]]; then
    GCS_BROWSER_FLAGS=()
    if [[ -n "${WSL_DISTRO_NAME:-}" ]]; then
        GCS_BROWSER_FLAGS=(--ozone-platform=x11 --disable-gpu)
    fi
    if command -v brave-browser >/dev/null 2>&1; then
        brave-browser "${GCS_BROWSER_FLAGS[@]}" \
            --new-window http://127.0.0.1:8899/ >/dev/null 2>&1 &
    elif command -v xdg-open >/dev/null 2>&1; then
        xdg-open http://127.0.0.1:8899/ >/dev/null 2>&1 &
    fi
fi

# 5. Start Gazebo outside ROS launch. Gazebo transport can stall on this GPU
# stack when gz-server is a launch-owned child process.
WORLD_RUNTIME="/tmp/aerothon_mission2_runtime.sdf"
VEHICLE_MODELS_DIR="/tmp/aerothon_vehicle_models"
# ------------------------------------------------------------------------------
# Simulation fidelity knobs.
#
#   AEROTHON_CAMERA_W / _H   camera render size. High = representative
#                            perception (goal.md Q14 wants 1080p), low = usable
#                            frame rate. Measured on this machine:
#                              640x480   -> RTF 0.55, camera 7.5 Hz
#                              1280x720  -> default compromise
#                              1920x1080 -> RTF 0.32, camera 1.45 Hz
#                            Use high resolution for MEASUREMENT runs and low
#                            for CLOSED-LOOP runs.
#
#   AEROTHON_START_QR_M      marker edge lengths. The competition size is still
#   AEROTHON_TARGET_QR_M     unconfirmed and is the dominant term in the search
#                            altitude (docs/QR_DECODE_ENVELOPE.md). The defaults
#                            (2.2 / 3.0 m) are much larger than any plausible
#                            real marker and make the simulation easy; sweep
#                            them to test the strategy across the real range.
#
# Example measurement run with a realistic marker:
#   AEROTHON_CAMERA_W=1920 AEROTHON_CAMERA_H=1080 \
#   AEROTHON_START_QR_M=0.5 AEROTHON_HEADLESS=1 ./scripts/launch_level6_sim.sh
# ------------------------------------------------------------------------------
echo "[SIM] camera ${AEROTHON_CAMERA_W:-1280}x${AEROTHON_CAMERA_H:-720}" \
     "| start QR ${AEROTHON_START_QR_M:-2.2} m" \
     "| target QR ${AEROTHON_TARGET_QR_M:-3.0} m"

python3 "$SCRIPT_DIR/materialize_vehicle_model.py" \
    --source "$ARDUPILOT_GAZEBO_PREFIX/share/ardupilot_gazebo/models/iris_with_gimbal/model.sdf" \
    --output-root "$VEHICLE_MODELS_DIR"
export GZ_SIM_RESOURCE_PATH="$VEHICLE_MODELS_DIR:${GZ_SIM_RESOURCE_PATH:-}"
python3 "$SCRIPT_DIR/materialize_world.py" \
    --source "$WORKSPACE_ROOT/src/aerothon_sim/sim_gazebo/worlds/mission2.sdf" \
    --assets "$WORKSPACE_ROOT/src/aerothon_sim/sim_gazebo/materials" \
    --output "$WORLD_RUNTIME" \
    --layout-out /tmp/aerothon_arena_layout.json

# The organiser inputs for THIS arena. A randomised arena moves the delivery
# field, and publishing the shipped arena's boundary for it sent the search
# to the wrong place. An explicit AEROTHON_DELIVERY_ZONE / AEROTHON_GEOFENCE
# still wins, so a boundary can be supplied by hand.
if [[ -z "${AEROTHON_DELIVERY_ZONE:-}" || -z "${AEROTHON_GEOFENCE:-}" ]]; then
    eval "$(python3 - <<'PY'
import json
d = json.load(open("/tmp/aerothon_arena_layout.json"))
z = d["delivery_zone_rect"]; f = d["geofence_rect"]
print("LAYOUT_ZONE=%s" % ",".join("%.3f" % v for v in z))
if d.get("geofence_poly"):     # a user-built arena's polygon (x,y;x,y;...)
    print("LAYOUT_FENCE='%s'" % ";".join("%.3f,%.3f" % (x, y)
                                        for x, y in d["geofence_poly"]))
else:
    print("LAYOUT_FENCE=%s" % ",".join("%.3f" % v for v in f))
PY
)"
    export AEROTHON_DELIVERY_ZONE="${AEROTHON_DELIVERY_ZONE:-$LAYOUT_ZONE}"
    export AEROTHON_GEOFENCE="${AEROTHON_GEOFENCE:-$LAYOUT_FENCE}"
fi
echo "[SIM] delivery zone cx,cy,w,h = $AEROTHON_DELIVERY_ZONE | geofence x0,x1,y0,y1 = $AEROTHON_GEOFENCE"

echo "[3/5] Starting Gazebo server and waiting for the Mission 2 world..."
# Render the sensors on the GPU rather than on the CPU.
#
# glxinfo in this WSL reports "llvmpipe (LLVM 20.1.2)", so every gpu_lidar ray
# and every camera frame was being rasterised in software. The 720-sample
# lidar then delivered 2.3 Hz against its configured 10 -- under the 8 Hz Q27
# requires -- and the camera detectors ran at 0.1 Hz, so the interlock refused
# to arm for a reason that is nowhere in the code.
#
# GALLIUM_DRIVER=d3d12 selects the real adapter; verified on this machine as
# "D3D12 (Intel(R) UHD Graphics)". The GUI branch below already did this and
# the server was deliberately left on default Mesa, which is what put sensor
# rendering on llvmpipe while the window got the GPU.
#
# MEASURED, and it is a trade rather than a win, which is why it is OFF by
# default. With the GUI up on this machine:
#
#   llvmpipe (software)      lidar 2.3 Hz   camera detectors 0.1 Hz
#   D3D12 (Intel UHD iGPU)   lidar 1.0 Hz   camera detectors 0.4 Hz
#
# The camera got 4x faster and the lidar 2.3x slower. The 720-sample gpu_lidar
# is many small render passes, where D3D12 translation overhead on a weak iGPU
# costs more than it saves; the single large camera frame benefits. Neither
# setting reaches the 8 Hz Q27 needs while the GUI is up -- only
# AEROTHON_HEADLESS=1 does.
#
# Set AEROTHON_SERVER_GPU=1 to opt in (worth it if the camera is what matters).
SERVER_ENV=()
if [[ "${AEROTHON_SERVER_GPU:-0}" == "1" && -n "${WSL_DISTRO_NAME:-}" && -e /dev/dxg ]]; then
    SERVER_ENV=("GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}"
                "LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}")
    echo "[OK] gz-server sensor rendering on the GPU (GALLIUM_DRIVER=d3d12)."
fi
env "${SERVER_ENV[@]}" gz sim -s -r --headless-rendering -v 3 "$WORLD_RUNTIME" &
PIDS+=($!)

WORLD_READY=false
for _attempt in $(seq 1 10); do
    if timeout 3 gz service -l 2>/dev/null | grep -q '^/gazebo/worlds$'; then
        WORLD_READY=true
        break
    fi
    sleep 0.5
done
if [ "$WORLD_READY" != true ]; then
    echo "[BLOCKED] Gazebo did not advertise /gazebo/worlds within 15 seconds."
    exit 3
fi
# Headless mode for automated testing.
#
# The Gazebo GUI and RViz cost roughly 200% and 30% CPU respectively on top of
# a gz-server already running with --headless-rendering, so rendering is paid
# for twice. Under that load the simulator cannot keep up with its own sensor
# update rates: /scan was measured at 2.9 Hz against a configured 10 Hz and
# /camera/image at 5.0 Hz against 20 Hz, which breaks goal.md Q27 (LiDAR >=
# 8 Hz) and Q14 (10 FPS QR) for reasons that have nothing to do with the code.
# Set AEROTHON_HEADLESS=1 for measurement runs; leave it unset for flying by eye.
if [[ "${AEROTHON_HEADLESS:-0}" == "1" ]]; then
    echo "[OK] Gazebo Mission 2 world is ready. HEADLESS mode: no GUI, no RViz."
    RVIZ_ARG="false"
else
    echo "[OK] Gazebo Mission 2 world is ready. Opening GUI..."
    GUI_ENV=()
    GUI_ENGINE="${AEROTHON_GUI_RENDER_ENGINE:-ogre2}"
    if [[ -n "${WSL_DISTRO_NAME:-}" && -e /dev/dxg ]]; then
        # WSL's default Mesa selection may fall back to llvmpipe. OGRE2 on
        # D3D12 also left Qt waiting on its render thread in the watched run.
        # Use the verified GUI renderer; server sensor rendering stays OGRE2.
        GUI_ENV=("GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}")
        GUI_ENGINE="${AEROTHON_GUI_RENDER_ENGINE:-ogre}"
    fi
    env "${GUI_ENV[@]}" gz sim -g --render-engine-gui "$GUI_ENGINE" &
    PIDS+=($!)
    RVIZ_ARG="true"
fi

# 5b. Play the organiser and supply the delivery-zone boundary.
#
# The mission and gcs_readiness both require a four-corner boundary before
# `/mission_ready` can go true, and nothing in the simulator supplied one, so
# every simulated run stopped at not-ready with "delivery-zone boundary is
# missing" and could not arm. This publishes the shipped arena's field, latched,
# once the FCU home position exists.
#
# For a randomised arena, pass that seed's zone centre:
#   AEROTHON_DELIVERY_ZONE="cx,cy,40,30" ./scripts/launch_level6_sim.sh
# Set AEROTHON_SUPPLY_DELIVERY_ZONE=0 to drive the boundary from the GCS instead.
if [[ "${AEROTHON_SUPPLY_DELIVERY_ZONE:-1}" == "1" ]] && command -v ros2 >/dev/null 2>&1; then
    echo "[4/5] Supplying the delivery-zone boundary (${AEROTHON_DELIVERY_ZONE:-32,0,40,30})..."
    python3 "$SCRIPT_DIR/publish_delivery_zone.py" &
    PIDS+=($!)
fi

# 6. Launch Full Simulation Stack (SITL + ROS 2 + SLAM + RViz)
echo "[5/5] Launching Gazebo Harmonic + ROS 2 Stack + slam_toolbox + RViz 2..."
if command -v ros2 >/dev/null 2>&1; then
    ros2 launch sim_gazebo sim_full.launch.py \
        fcu_url:=udp://127.0.0.1:14555@127.0.0.1:14556 rviz:="$RVIZ_ARG" slam:=true \
        stream_rate_keeper:="${AEROTHON_STREAM_KEEPER:-true}" \
        start_gz_server:=false gui:=false
else
    echo "[INFO] Running in headless mode (ros2 command not in current shell)."
    echo "[INFO] Telemetry aggregator and router are running."
    wait
fi
