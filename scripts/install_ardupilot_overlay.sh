#!/usr/bin/env bash
# ==============================================================================
# AeroTHON 2026 — ArduPilot SITL + Gazebo overlay installer (persistent)
# ==============================================================================
# WHY THIS EXISTS
#   The upstream ArduPilot/Gazebo layer was previously built into /tmp
#   (/tmp/ardupilot_stack2). A reboot wiped it, taking sim_vehicle.py,
#   ardupilot_gazebo and ardupilot_sitl with it, and the whole simulation stack
#   stopped being runnable with no obvious cause. This script rebuilds that
#   layer into a PERSISTENT location and is safe to re-run.
#
# WHAT IT INSTALLS  (upstream code, deliberately outside the mission repo)
#   ardupilot            - flight firmware + SITL + sim_vehicle.py
#   ardupilot_gazebo     - the Gazebo Harmonic plugin (JSON model interface)
#   ardupilot_gz         - ROS 2 bringup packages (ardupilot_gz_bringup, ...)
#   SITL_Models          - shared vehicle/sensor models
#
# NO SUDO REQUIRED. Gazebo Harmonic development headers are already present,
# vendored by ROS 2 Jazzy under /opt/ros/jazzy/opt/gz_*_vendor, so the plugin
# builds against those rather than against apt libgz-sim8-dev.
#
# Usage:
#   scripts/install_ardupilot_overlay.sh                 # default location
#   AEROTHON_OFFICIAL_WS=~/somewhere scripts/install_ardupilot_overlay.sh
#   ARDUPILOT_VERSION=master scripts/install_ardupilot_overlay.sh
#
# Afterwards, export the location so the launcher finds it:
#   export AEROTHON_OFFICIAL_WS="$HOME/aerothon_stack"
# ==============================================================================
set -uo pipefail

# Resolve our own directory BEFORE any cd, so helper scripts stay findable.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WS="${AEROTHON_OFFICIAL_WS:-$HOME/aerothon_stack}"
# goal.md locks ArduPilot Copter "4.5/4.6". Copter-4.5 is the newest stable
# BRANCH that actually exists upstream — there is no Copter-4.6 branch — so we
# pin that rather than following the manifest's floating master, keeping the
# simulation on the firmware line the aircraft is intended to fly.
# Override with ARDUPILOT_VERSION=master if ardupilot_gz main stops building
# against 4.5.
ARDUPILOT_VERSION="${ARDUPILOT_VERSION:-Copter-4.5}"
JOBS="${JOBS:-$(( $(nproc) > 8 ? 8 : $(nproc) ))}"

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
fail() { printf '\n[ERROR] %s\n' "$*" >&2; exit 1; }

log "Overlay workspace: $WS"
log "ArduPilot version: $ARDUPILOT_VERSION   parallel jobs: $JOBS"

set +u
source /opt/ros/jazzy/setup.bash || fail "ROS 2 Jazzy not found at /opt/ros/jazzy"
set -u

command -v vcs    >/dev/null || fail "vcstool (vcs) is required"
command -v colcon >/dev/null || fail "colcon is required"
command -v git    >/dev/null || fail "git is required"

mkdir -p "$WS/src" || fail "cannot create $WS/src"
cd "$WS" || fail "cannot enter $WS"

# ------------------------------------------------------------------------------
# 1. Source manifest
#
# Trimmed from the upstream ArduPilot/ardupilot_gz ros2_gz.repos: ros_gz,
# sdformat_urdf and micro_ros_agent are omitted because ROS 2 Jazzy already
# provides ros_gz and sdformat_urdf, and the DDS/micro-ROS path is not used by
# this project (MAVROS is the FCU interface, per the locked architecture).
# ------------------------------------------------------------------------------
cat > "$WS/aerothon_overlay.repos" <<EOF
repositories:
  ardupilot:
    type: git
    url: https://github.com/ArduPilot/ardupilot.git
    version: ${ARDUPILOT_VERSION}
  ardupilot_gazebo:
    type: git
    url: https://github.com/ArduPilot/ardupilot_gazebo.git
    version: ros2
  ardupilot_gz:
    type: git
    url: https://github.com/ArduPilot/ardupilot_gz.git
    version: main
  ardupilot_sitl_models:
    type: git
    url: https://github.com/ArduPilot/SITL_Models.git
    version: main
EOF

log "Importing sources (this pulls ArduPilot submodules and takes a while)..."
# Four repositories clone in parallel and GitHub intermittently refuses one of
# the connections ("Failed to connect ... after 12 ms"). vcs has no retry, so
# wrap it: re-running is cheap because already-cloned repos are just updated.
IMPORT_OK=0
for attempt in 1 2 3; do
    if vcs import --recursive --input "$WS/aerothon_overlay.repos" "$WS/src"; then
        IMPORT_OK=1
        break
    fi
    log "vcs import attempt $attempt failed; retrying in 10s..."
    sleep 10
done
[[ "$IMPORT_OK" == "1" ]] || fail "vcs import failed after 3 attempts"

# vcs reports success even when an individual repo failed to check out its ref,
# so verify the things we actually need rather than trusting the exit code.
[[ -f "$WS/src/ardupilot/Tools/autotest/sim_vehicle.py" ]] \
    || fail "ardupilot source incomplete (sim_vehicle.py missing)"
[[ -f "$WS/src/ardupilot_gazebo/CMakeLists.txt" ]] \
    || fail "ardupilot_gazebo source incomplete (CMakeLists.txt missing)"
[[ -d "$WS/src/ardupilot/modules/mavlink" ]] \
    || fail "ardupilot submodules incomplete (modules/mavlink missing)"
log "Source tree verified."

# ------------------------------------------------------------------------------
# 1b. GStreamer: required by upstream, but only for a plugin we do not use.
#
# ardupilot_gazebo hard-requires gstreamer-1.0 dev files, which need root to
# install. They are used solely by GstCameraPlugin (RTP video streaming); the
# ArduPilotPlugin that carries the SITL <-> Gazebo interface does not need
# them. If the dev files are absent, relax that one requirement rather than
# blocking the entire simulation stack.
# ------------------------------------------------------------------------------
if pkg-config --exists gstreamer-1.0 gstreamer-app-1.0 2>/dev/null; then
    log "GStreamer development files found — building ardupilot_gazebo unmodified."
else
    log "GStreamer development files ABSENT."
    echo "      GstCameraPlugin (RTP video streaming) will be skipped. It is not"
    echo "      used by this project. To build upstream unmodified instead:"
    echo "          sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
    echo "      then delete $WS/src/ardupilot_gazebo and re-run this script."
    python3 "$SCRIPT_DIR/patch_ardupilot_gazebo_gst.py" \
        "$WS/src/ardupilot_gazebo/CMakeLists.txt" \
        || fail "could not make GStreamer optional in ardupilot_gazebo"
fi

# ------------------------------------------------------------------------------
# 1c. microxrceddsgen
#
# Tools/ros2/ardupilot_sitl/CMakeLists.txt calls waf with a hardcoded
# --enable-dds, so ArduPilot will not configure without the micro-XRCE-DDS
# code generator, even though this project talks to the FCU over MAVROS and
# never uses AP_DDS. Rather than patch --enable-dds out of upstream, build the
# generator ArduPilot expects: it needs only Java (already present) and the
# bundled Gradle wrapper, no root.
# ------------------------------------------------------------------------------
DDS_GEN_DIR="$WS/tools/Micro-XRCE-DDS-Gen"
export PATH="$DDS_GEN_DIR/scripts:$PATH"

if command -v microxrceddsgen >/dev/null 2>&1; then
    log "microxrceddsgen found — building ardupilot_sitl unmodified (AP_DDS enabled)."
else
    log "microxrceddsgen NOT available — disabling AP_DDS for this build."
    echo "      AP_DDS is unused by this project (the FCU link is MAVLink/MAVROS)."
    echo "      Building the generator upstream requires a JDK the bundled Gradle"
    echo "      wrapper supports; this machine has only JDK 21, which Gradle 7.6"
    echo "      rejects. To build it properly instead:"
    echo "          sudo apt install openjdk-17-jdk"
    echo "          git clone --recurse-submodules \\"
    echo "              https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git $DDS_GEN_DIR"
    echo "          cd $DDS_GEN_DIR && JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 \\"
    echo "              ./gradlew assemble"
    echo "      then delete $WS/src/ardupilot and re-run this script."
    python3 "$SCRIPT_DIR/patch_ardupilot_sitl_dds.py" \
        "$WS/src/ardupilot/Tools/ros2/ardupilot_sitl/CMakeLists.txt" \
        || fail "could not disable AP_DDS in ardupilot_sitl"
fi

# ------------------------------------------------------------------------------
# 2. Build
#
# BUILD_TESTING=OFF keeps upstream gtest suites out of the build; we test the
# mission code, not ArduPilot's own regression suite.
# ------------------------------------------------------------------------------
log "Building overlay with colcon..."
colcon build \
    --packages-up-to ardupilot_gz_bringup ardupilot_gazebo \
    --parallel-workers "$JOBS" \
    --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF
BUILD_RC=$?

if [[ "$BUILD_RC" -ne 0 ]]; then
    log "colcon build FAILED (rc=$BUILD_RC)."
    echo "If the failure mentions ardupilot_gz vs ArduPilot ${ARDUPILOT_VERSION}," >&2
    echo "retry with the upstream-tested combination:" >&2
    echo "    ARDUPILOT_VERSION=master $0" >&2
    exit "$BUILD_RC"
fi

# ------------------------------------------------------------------------------
# 3. Verify
# ------------------------------------------------------------------------------
set +u
source "$WS/install/setup.bash"
set -u
export PATH="$WS/src/ardupilot/Tools/autotest:$WS/src/ardupilot/build/sitl/bin:$PATH"

log "Verifying overlay..."
FAILED=0
check() {
    local label="$1"; shift
    if "$@" >/dev/null 2>&1; then printf 'PASS  %s\n' "$label"
    else printf 'FAIL  %s\n' "$label"; FAILED=1; fi
}
check 'sim_vehicle.py on PATH'          command -v sim_vehicle.py
check 'arducopter SITL binary built'    test -x "$WS/src/ardupilot/build/sitl/bin/arducopter"
check 'ardupilot_gazebo package'        ros2 pkg prefix ardupilot_gazebo
check 'ardupilot_gz_bringup package'    ros2 pkg prefix ardupilot_gz_bringup
check 'ardupilot_sitl package'          ros2 pkg prefix ardupilot_sitl

if [[ "$FAILED" -ne 0 ]]; then
    fail "Overlay built but verification failed. See output above."
fi

cat <<EOF

======================================================================
 Overlay installed at: $WS
======================================================================
Add these to your shell profile so the launcher finds everything persistently:

    export AEROTHON_OFFICIAL_WS="$WS"
    export PATH="$DDS_GEN_DIR/scripts:\$PATH"

Then re-run the environment check:

    scripts/preflight_stack.sh
======================================================================
EOF
