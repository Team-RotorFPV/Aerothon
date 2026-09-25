# AeroTHON 2026 — Track 1 — Team Rotor FPV

Autonomous QR-guided delivery multirotor + custom control GCS for **SAEINDIA AeroTHON 2026, Track 1 (Rotorcraft Systems Challenge)**.

- 🗺️ **[PHASE_PLAN.md](PHASE_PLAN.md)** — **the active plan.** 12 dependency-ordered phases from fail-closed rails to a randomised-arena regression. Start here.
- ✅ **[VERIFICATION.md](VERIFICATION.md)** — append-only log of what has actually been *proven*, and how. Claim → method → raw output → verdict.
- 🔍 **[docs/GEOMETRY_AUDIT.md](docs/GEOMETRY_AUDIT.md)** — every hardcoded arena assumption, and the phase that replaces it with perception.
- 📋 **[CURRENT_PROGRESS_HANDOFF.md](CURRENT_PROGRESS_HANDOFF.md)** — honest defect inventory that the phase plan works through.
- 📘 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — full system design (75 locked decisions, diagrams, node graph, GCS protocol).
- ⚙️ **[docs/SETUP.md](docs/SETUP.md)** — version-locked environment + install.
- 🚦 **[docs/STACK_AND_DEPLOYMENT.md](docs/STACK_AND_DEPLOYMENT.md)** — current qualification status, GCS connection, test ladder, and real-aircraft deployment gates.
- 📦 **[aerothon.repos](aerothon.repos)** — vcstool workspace manifest.
- 🐳 **[docker/](docker/)** — reproducible dev/sim containers.

## The two missions
- **Mission 1 — Eyes in the Sky:** fully **manual** obstacle course, observation, precision delivery.
- **Mission 2 — SkyScan:** fully **autonomous** QR scan → corridor → matched-QR delivery → return. *(This repo's primary focus.)*

**Hard constraint:** MTOW **< 2 kg**. One airframe flies both missions.

## Target package tree (`src/`)

```
src/
├── aerothon_mission/
│   ├── mission_bringup/        # launch files (top-level + per-subsystem), sim/real args
│   ├── mission_bt/             # py_trees behavior tree (guard + mission sequence)
│   └── uav_description/        # URDF/SDF, TF, frames
├── aerothon_perception/
│   ├── perception_qr/          # OpenCV QRCodeDetector (start QR + target match)
│   ├── perception_banner/      # HSV green-banner detect + align
│   └── perception_redzone/     # HSV red-zone detect (geofence is the hard backstop)
├── aerothon_avoidance/
│   └── avoidance/              # costmap_2d config + custom velocity_controller
├── aerothon_gcs/
│   ├── gcs_aggregator/         # ROS2 -> WebSocket JSON (telemetry/event/ack) + command in
│   ├── gcs_command/            # command node -> MAVROS services (arm/takeoff/mode/winch)
│   ├── gcs_readiness/          # diagnostics + prearm -> /mission_ready interlock
│   └── tauri_app/              # Rust backend + React UI (ECharts, MapLibre GL)
└── aerothon_sim/
    └── sim_gazebo/             # Gazebo Harmonic worlds, models, sim launch
```

## Quickstart

**1. Install the upstream ArduPilot/Gazebo overlay** (once per machine — no sudo needed):

```bash
scripts/install_ardupilot_overlay.sh
```

This builds ArduPilot SITL, `ardupilot_gazebo` and `ardupilot_gz` into
`~/aerothon_stack`. It is deliberately **persistent**: an earlier setup built
this into `/tmp`, and a reboot destroyed the entire simulation stack. Add
`export AEROTHON_OFFICIAL_WS="$HOME/aerothon_stack"` to your shell profile.

**2. Build this workspace and check the environment:**

```bash
colcon build --symlink-install && source install/setup.bash
scripts/preflight_stack.sh
```

**3. Run the offline checks** (unit tests, syntax, XML/YAML, frontend build):

```bash
scripts/run_tests.sh
```

**4. Run the live stack** (idempotent — it reaps any previous run first):

```bash
scripts/launch_level6_sim.sh
```

**5. Capture an evidence pack** while the stack runs:

```bash
scripts/capture_evidence.sh --phase P0 --duration 30 --label my-run
```

Writes rosbag, topic census, camera frames, TF tree, graph inventory and a
manifest to `evidence/<phase>/<timestamp>-<label>/`. Then write the verdict
into [VERIFICATION.md](VERIFICATION.md).

> **Status caveat.** Per [CURRENT_PROGRESS_HANDOFF.md](CURRENT_PROGRESS_HANDOFF.md),
> this stack is a connected SITL demonstration, **not** a competition-ready
> autonomous system. Perception is not yet integrated into flight guidance and
> the mission can still advance after perception failure. Do not fly it
> autonomously on a real aircraft. [PHASE_PLAN.md](PHASE_PLAN.md) is the route
> from here to a qualified system.

## Build discipline
- **Fallback-first:** get a scoring baseline flying with ArduPilot-native proximity avoidance before the full Nav2/SLAM stack.
- **Mass budget** is the gating artifact — keep MTOW ≤ ~1.8 kg.
- **Safety ladder:** ELRS pilot > custom GCS > Mission Planner; FC failsafes + Lua heartbeat underneath everything.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) §10 for outstanding pre-build items.
