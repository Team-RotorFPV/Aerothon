# Agent instructions for AeroTHON Mission 2

## Current context

The user authorised replacement of the old Pi code/GCS with this runtime on
14 October 2026. Destination: `pi5@192.168.0.101`, Debian 13 arm64. Deployment
uses an Ubuntu 24.04/Jazzy container and destination scripts in
`scripts/pi_runtime/`. Flight algorithms remain unchanged. Documentation,
cleanup, packaging and the hard field video preceded this hardware step.

Read `MASTER_DOCUMENTATION.md` for architecture, operation, qualification,
hardware limitations and simulation commands. These are the two maintained
project Markdown documents. Historical reports/issues are archived locally
under `.scratch/pi_consolidation_20261013/`; runs remain in `logs/`.
Repo-local `.agents/skills` is developer tooling, excluded from the Pi bundle.

V44 passed three calm rendered clarified-arena worlds, 16 headless navigation
cases and reduced offline checks with 1920 passed/eight skipped. The next
laid-only field run was aborted while in outbound banner alignment to permit
consolidation, then landed/disarmed with GRADE_FAIL. It is operator-interrupted
evidence, not a completed mission or a graded 900 s violation. Strong gusts,
coupled delay, original campaign closure and physical qualification remain
outstanding. Fresh matching-source evidence is required for new claims.

This cleanup's final combined-banner field run completed and disarmed but
failed 21/22 acceptance checks: native clearance 0.663645 m against 0.70 m
at return obstacle 0, native time 842.099 s. It had zero unexpected contacts,
0.11 m payload error and 886.1 s flight time. Evidence:
`logs/pi_cleanup_hard_field_both_s20261004_retry1_20261013`. Its source freeze
is `logs/pi_cleanup_source_with_wiring_final_20261013`; documentation was then
updated to record the result. Flight source remained unchanged. Do not call
this run accepted or weaken its threshold. The next functional issues are
field clearance diagnosis and the Pi GPIO/encoder winch backend, separate
from the completed documentation/cleanup/video scope.

On 15 October the organiser clarifications were applied (master guide
section 1, "15 October 2026"). Final source `188aacd0...` passed the full
reduced offline suite and two calm rendered acceptances 22/22: pattern-only
QR, combined banners, 10 m corridor; payload decoding, 5 m corridor. The
planner's 0.10 m tracking reserve addresses calm-condition clearance only;
field and strong-gust conditions are not re-qualified on it. The user asked
for near-ideal competition conditions, not extreme-condition work. The Pi was
unreachable, so that bundle is not deployed.

## Work sequence

1. Read the relevant master-guide sections and actual source/configuration.
   Check Git status and active ROS/SITL/Gazebo processes. Preserve unrelated
   user edits and existing evidence directories.
2. Establish the active runtime path through
   `src/aerothon_mission/mission_bringup/launch/mission2.launch.py`.
   `use_sim:=false` selects real sensors/MAVLink actuators and turns SLAM/RViz
   off by default. Inspect package entry points and conditional imports.
3. For cleanup, prove disuse through imports, launch/resource/subprocess
   references and tests. Archive original documentation with verified hashes.
   Keep flight decisions unchanged unless the user authorises a functional fix.
4. Compare decision-source/configuration changes, run affected regressions
   and required offline checks. Any changed decision module needs renewed
   flight evidence. Documentation/package-only results need explicit scope.
5. Build the revised GCS and verify the runtime-only ZIP, asset URLs, hashes
   and exactly two Markdown documents. Build ROS binaries on the Pi architecture.
6. Run acceptance into a fresh directory with no competing simulator. Record
   source/configuration, input audits, native truth/contacts and camera timeline
   through connected disarm. Deliver the video and actual pass/failure.

## Runtime boundaries

- For Pi deployment/operation, read the Debian route in master-guide section 5.
  Source is `/home/pi5/aerothon_ws`, launcher/configuration is
  `/home/pi5/aerothon-deploy`, measured state is `/home/pi5/.aerothon`.
  The native router starts after local filesystems, before networking/Docker,
  using the measured runtime.env UART until a saved profile selects another link.
  The saved drone profile connects the production runtime automatically at boot,
  without a props checkbox, arming, mission Start or automatic camera movement;
  reviewed Start controls mission or component execution. The old console is
  disabled and direct mission-service launch refuses a second controller.
  An explicit Disconnect suspends automatic runtime connection for that web-host
  session. Reviewed Start controls execution. Connection requires no props review.
  Preserve the archive and destination receipts in `.scratch/pi_deploy_20261014/`.
- The measured current link is TELEM3 at 115200 on `/dev/ttyAMA2`, BCM4 TX/header7
  and BCM5 RX/header29. UART0 BCM14/15 was superseded on 17 October.
  `/dev/serial0` is the debug/login UART. Keep one FC serial owner and update
  measured link configuration explicitly; proposal defaults are not evidence.
- `scripts/package_pi_runtime.py` lists the 13 local runtime ROS packages,
  operating scripts and built browser GCS. External ROS/sensor dependencies
  still need supported destination installation.
- `fast_connected_planner_candidate.py`, buffered scans, capture inbox and
  independent clock drivers are production startup paths. Preserve them.
- Gazebo truth, contacts, attachment state, layout and overview camera are
  evaluation-only. Onboard decisions use camera/lidar/FCU observations.
- Organiser dimensions/colours use the GCS/operator path and aircraft applied
  confirmation. Delivery-zone boundary, arena fence and camera-derived red
  exclusions have distinct meanings. Camera footprint is not aircraft entry.
- One owner per serial interface: router owns the FC UART in the documented
  topology; MAVROS consumes UDP. DDS remains on the Pi. The revised GCS is
  command-capable; receive-only conversion would change behavior.
- Preserve immutable source ZIPs, measured calibration, interrupted runs and
  failures. Passes from different source versions do not combine into acceptance.
- LiDAR-only mapping uses `lidar_odom -> lidar_mapping_base -> lidar_mapping_scan`
  and `/scan_slam`; flight navigation keeps FCU odometry. A held registration
  does not publish a fresh mapping scan. Full mounting calibration requires
  measured XYZ/roll/pitch, repeated background-isolated panels and independent
  direction/range checks. Legacy yaw-only records leave XYZ unmeasured. Read the
  master guide's Sensors and mechanism section before calibrating; keep receipts
  and source inventories under `.scratch/lidar_boot_20261017/`.
- Stop a flying simulator through GCS recovery and recorded disarm before
  shutdown. The current harness TERM trap can clean up and then continue;
  TERM alone is not a reliable pre-arm cancellation. Preserve interrupted
  attempts as such, and target only the identified simulator process group.

## Hardware context

The user identified CUAV X7+ and confirmed ArduPilot `SERVO9` for camera tilt.
The existing assignment is `SERVO9_FUNCTION 7`, mount pitch. Confirm the
physical connector, supply, direction, PWM endpoints and measured travel.
`readback=timed` assumes settlement; it does not prove motion or FC acceptance.

Selected winch: Robu N20 12 V 60 RPM with encoder, linked in the master guide.
User-supplied pinout: red/white motor, black encoder VCC 3.3–5 V, blue encoder
ground, yellow/green feedback. Output logic and resolution remain unmeasured.
User-supplied stall current is 1 A. Recommended topology is Pi through the H-bridge,
with encoder inputs on compatible GPIO; camera servo remains on the FC.

The winch adapter's `mavlink` backend sends MAVLink commands and integrates
travel. Its `gpio` backend (`winch_ctrl/gpio_winch.py`) drives the user-selected
BTS7960 43 A through `lgpio` and measures travel from the N20 encoder; it is
software-tested only and refuses to run until encoder counts per output turn
and spool radius are measured. `hardware_wiring.yaml` assigns RPWM/LPWM
BCM18/19, enables BCM22/23 and yellow/green encoder inputs BCM17/27, with
physical header numbers, and is still a record, not loaded parameters.
Encoder supply is planned at 3.3 V. Verify module input logic and Pi 5 PWM
support. SERVO10/PWM is legacy proposed wiring. Never power
the motor from GPIO or expose Pi inputs to incompatible levels. Record BCM
and physical header numbers together once wiring is established.

Encoder support is functional work, separate from cleanup. Estimate, command
request, accepted command, shaft travel, payload support, detachment and delivery
confirmation remain distinct. Motor PWM effort is neither current nor thrust;
physical loaded/unloaded hover calibration is required.

## Verification and completion

- `bash scripts/run_tests.sh --reduced-scope`: complete in-scope regression,
  Python/shell syntax, XML/YAML and frontend build. Retain explicit skip ledger.
- `bash scripts/check_pi_runtime.sh`: read-only destination software check;
  it does not launch motors or qualify hardware.
- `bash scripts/serve_gcs.sh`: serves the built browser GCS; verify remote
  Pi endpoint and offline satellite assets.
- `scripts/run_recorded_acceptance.sh WORLD SEED NEW_OUT PROFILE`: refuses
  existing output/competing simulator. Preserve its process/evidence boundaries.
- Keep 0.70 m corridor and 1.50 m restricted-zone clearance, 900 s mission
  limit, native contact channels/canary, freshness and through-disarm recording.
- Strong-gust unrestricted failures remain unresolved. Report exclusions;
  never weaken thresholds/tests to manufacture a pass.
- Done means two maintained guides, archived originals, reviewed cleanup,
  verified runtime bundle, reported checks and a fresh requested video/run
  with actual outcome. Unsupported hardware connections stay explicitly pending.

## Documentation and tracking

Maintain knowledge in the master guide and editing workflow here. Historical
issues remain recoverable from the archive. New `.scratch/<feature>/` JSON
receipts record problem/scope/status/evidence/blockers/completion without adding
a third maintained Markdown guide. Canonical triage roles remain `needs-triage`,
`needs-info`, `ready-for-agent`, `ready-for-human` and `wontfix`. Terminology and
decisions belong in the relevant master-guide section. Update qualification
separately from plans and unknown hardware measurements.
