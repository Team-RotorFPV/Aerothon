# SITL bring-up: six defects, and a hardware ceiling

Session of 2026-09-11, continuing from `gazebo-gcs-debug-2026-09-11.md`.

The stack was run end to end on the Windows laptop via WSL. It now reaches
`Mission state -> WAITING` with a healthy FCU, and the Q27 readiness interlock
goes from 1/12 items passing to 9/12. The remaining three cannot pass on this
machine, for a reason that is not in the code — see **The ceiling**.

Full suite: **905 passed, 0 failed** (`python3 -m pytest sim/`, run under
`/usr/bin/python3` with the workspace sourced).

---

## Defects fixed

### 1. Vehicle materialiser mis-scoped `base_link` on merge-includes

`scripts/materialize_vehicle_model.py`

Last session taught the materialiser two shapes of `iris_with_gimbal`: links
inline (`base_link`) and a plain `<include>` (`iris_with_standoffs::base_link`).
The overlay built by `install_ardupilot_overlay.sh` ships a **third**:

```xml
<include merge="true">
  <uri>package://ardupilot_gazebo/models/iris_with_standoffs</uri>
  <name>iris</name>
</include>
```

`merge="true"` splices the included links into the parent scope, so the `<name>`
is *not* a frame prefix and the correct reference is a bare `base_link` — which
is exactly what the source model's own `gimbal_joint` uses. The materialiser
read the include name and emitted `iris::base_link`.

Worse than the original bug: instead of silently dropping a joint, gz-sim
refused the **entire world** with `Error Code 21`, never advertised
`/gazebo/worlds`, and the launcher timed out with a message that mentions
neither the vehicle nor the joint.

Regression: `sim/test_vehicle_model_materialisation.py::test_a_merge_include_keeps_the_plain_base_link`,
plus the merge shape added to the three existing subtest loops. Verified
red/green.

### 2. `EKF_STATUS_REPORT` was requested by nothing

`scripts/set_stream_rates.py`, `mission_bringup/stream_rate_keeper.py`

The interlock reads `/mavros/estimator_status`, which MAVROS fills only from
MAVLink message 193. That ID appeared in neither stream table — both stopped at
`RC_CHANNELS` (65). Since `is_ready()` requires **every** item, the interlock
could never go true. **This was true on the aircraft, not only in simulation.**

Regression: `sim/test_stream_rate_coverage.py` (3 tests). It pins every
interlock-critical message to both tables, asserts the two tables agree (they
had already drifted), and checks the setpoint rail is above the keeper's 10 Hz
warning threshold. Parses with `ast` rather than importing, so it runs on a
development machine with no ROS.

### 3. `/mavros/global_position/gp_hdop` does not exist

`gcs_aggregator/readiness_node.py`

The string `hdop` appears nowhere in `/opt/ros/jazzy/lib/libmavros_plugins.so`.
MAVROS 2.14 does not publish that topic, so "GPS HDOP" read "no data received"
for the life of every run. Satellites arrive from the same MAVLink message,
which made the GPS look half-alive and hid the cause. Same failure on hardware.

Now subscribes to `/mavros/gpsstatus/gps1/raw` (`mavros_msgs/GPSRAW`), whose
`eph` field is documented as the HDOP, with `UINT16_MAX` treated as unknown
rather than coerced to a number. Confirmed live: the item now reports a real
value (1.21).

### 4. SITL defaults path broke on spaces in the workspace path

`sim_gazebo/launch/sim_full.launch.py`

`ardupilot_sitl` builds the `arducopter` command with `shell=True` and does not
quote its arguments. This workspace lives under `/mnt/d/MY DOCUMENTS/...`, so a
params file passed by install path split the comma-separated `--defaults` list
mid-path. SITL died at startup with `PANIC: Failed to load defaults`, and with
no flight controller *every* MAVLink-derived interlock item reported "no data
received" — a failure that names the parameter file nowhere.

Our file is now staged into `tempfile.gettempdir()` before being handed to SITL,
matching how the launcher already stages the world and vehicle models.

**This hazard applies to anything this repo hands to upstream launch code.**

### 5. The router sat behind MAVProxy on SERIAL0

`sim_full.launch.py`, `scripts/launch_level6_sim.sh`

`robot.launch.py` starts MAVProxy unconditionally with `--master tcp:5760` and
`--out 14550` — the port the router was listening on. MAVProxy holds the master
link and re-requests its streams at its 4 Hz default, so every
`SET_MESSAGE_INTERVAL` the stack issued was clobbered on MAVProxy's next sweep.

SITL now hands the router its own channel, `serial1: udpclient:127.0.0.1:14560`,
with the launcher listening there. MAVProxy keeps SERIAL0 and is harmless.
Verified: `SR1_POSITION 50`, `SR1_EXTRA3 3`, `SERIAL1_PROTOCOL 2` all read back
correctly from the FCU, and the keeper now logs "stream rates applied" with no
ack timeouts.

### 6. Simulator presented values the interlock could never accept

`sim_gazebo/config/aerothon_sitl.parm` (new)

Stock SITL offers a 3S pack at 12.6 V and a 10-satellite fix against Q27's
`> 15.0 V` and `>= 12`. **The thresholds were not touched** — the simulator now
presents what the 4S competition aircraft presents (`SIM_BATT_VOLTAGE 16.8`,
`SIM_GPS_NUMSATS 18`), so what is tested is what is flown. Also sets `SR1_*`
stream rates for the mission channel.

---

## The ceiling

Three items cannot pass on this laptop. All three have one cause.

Measured with `pymavlink` against the FCU's own clock, comparing sim time to
wall time:

| configuration | real-time factor | lidar |
|---|---|---|
| GUI + llvmpipe (software) | ~0.23 | 2.3 Hz |
| GUI + D3D12 (Intel UHD) | 0.104 | 1.0 Hz |
| headless + llvmpipe | **0.292** | **3.0 Hz** |

Lidar rate ≈ 10 sim-Hz × RTF, every time.

Stream and sensor rates are specified in **sim** time; the interlock measures
them in **wall** time. At RTF 0.29 everything arrives at roughly a third of its
configured rate, so:

- **Lidar** 10 sim-Hz → 3.0 wall-Hz, against the 8 Hz Q27 requires.
- **EKF health** 3 sim-Hz → one message every ~3.4 s wall, which exceeds
  `max_stale_s` of 3.0. It is stale on arrival and reports "no data received".
  The message-193 fix is correct; the message cannot arrive often enough.
- **Pose rate** 50 sim-Hz reads as ~0.

Reaching 8 Hz needs **RTF ≥ 0.8**. Headless nearly tripled RTF and still lands
at 0.29. `glxinfo` reports `llvmpipe (LLVM 20.1.2)` — there is no GPU
acceleration; a 720-sample `gpu_lidar` and a 1280×720 camera are rasterised on
the CPU.

GPU passthrough exists (`/dev/dxg`, `d3d12_dri.so`) and
`GALLIUM_DRIVER=d3d12` does select `D3D12 (Intel(R) UHD Graphics)`, but it made
things worse overall: camera detectors 0.1 → 0.4 Hz, lidar 2.3 → 1.0 Hz. The
720-sample lidar is many small render passes, where translation overhead on a
weak iGPU costs more than it saves. Left in as `AEROTHON_SERVER_GPU=1`,
**off by default**, with the measurement recorded in the script.

### For the machine you run it on next

A discrete GPU should clear this. If it does not, the levers in order of
directness are: lidar sample count (720 is the dominant cost), camera
resolution, then running the interlock's rate checks against the simulation
clock via `use_sim_time` — on the aircraft sim time equals wall time, so that
check would be identical in the field while measuring the sensor rather than
the host in SITL. That last one was offered and **not** taken this session; it
removes a signal that is currently telling the truth about the hardware.

---

## Environment, as found

Preflight went 7/12 → 12/12. Installed: `python3-websockets`,
`ros-jazzy-py-trees`, `ros-jazzy-web-video-server`, `python3-vcstool`,
`python3-rosdep`, `tesseract-ocr`. The apt index was stale enough to 404.

The ArduPilot overlay did not exist; `install_ardupilot_overlay.sh` built it to
`~/aerothon_stack` (4.3 GB, Copter-4.5, 5/5 verified). Add to your profile so it
survives a reboot:

```bash
export AEROTHON_OFFICIAL_WS="$HOME/aerothon_stack"
export PATH="$HOME/aerothon_stack/tools/Micro-XRCE-DDS-Gen/scripts:$PATH"
```

`tesseract-ocr` was missing, so `word_reader.py` silently fell back to its
template correlator and 4 banner-lettering tests failed. Installing it took the
suite to zero failures.

---

## Known, not fixed

Deliberately left alone — none block a run.

1. **`~/venv-ardupilot` carries OpenCV 5.0.0**, where `CV_8UC3 == 64`.
   `cv_bridge` is built against OpenCV 4 and looks up `16`, so every image
   conversion raises `KeyError: 16` under that interpreter. It accounted for 41
   of 45 test failures when pytest ran under it. The live stack is unaffected —
   node shebangs are `#!/usr/bin/python3` and sourcing `install/setup.bash` puts
   the system interpreter first. **Run `pytest` with `/usr/bin/python3`.**

2. **`preflight_stack.sh:59` gives a false PASS** on that exact failure: it
   tests `import numpy, cv_bridge`, which survives. Only a conversion fails.
   Worth strengthening to a real `cv2_to_imgmsg` round-trip, and adding a
   `tesseract` check.

3. **`stream_rate_keeper`'s warning blames "MAVProxy contention"**, which is now
   a misleading diagnosis — after fix 5 the cause is real-time factor. The text
   sent me down a wrong path for a while.

4. **`aggregator.py:530` logs a traceback on every browser disconnect**
   (`ConnectionClosedError`, code 1005). Cosmetic: `try/finally` cleans up
   correctly and the node survives. Catching `ConnectionClosed` would quiet it.

5. **ROS introspection from an external shell sees nothing** — `ros2 topic list`
   returns rc=0 with zero topics while the stack is plainly running. Not
   `ROS_LOCALHOST_ONLY` (the launcher leaves it off by default, and setting it
   isolated *me*). Unexplained. Workaround: read the aggregator's WebSocket on
   `ws://127.0.0.1:8765`, or attach `pymavlink` to the router's GCS port 14552 —
   both used heavily this session and both work well.
