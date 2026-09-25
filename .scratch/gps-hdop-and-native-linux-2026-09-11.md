# The simulated GPS could not pass its own interlock

Session of 2026-09-11, third of the day, continuing from
`sitl-interlock-and-rtf-2026-09-11.md`. Run on the **native Linux** install,
not WSL — which turned out to matter before anything else could start.

Interlock went **9/12 → 10/12**, verified live. Full suite **913 passed**
(905 + 8 new).

---

## 0. The workspace had been built on a different machine

`build/` and `install/` were produced under WSL, where this repo lives at
`/mnt/d/MY DOCUMENTS/...`. Booted natively the same disk mounts at
`/mnt/newvolume`, so every `--symlink-install` link pointed at a path that does
not exist: 44 dangling symlinks, 88 files carrying the stale prefix. The
`mission-bt.egg-link` pointed into the dead tree too, so the Python packages
were not importable either.

Nothing reports this as a path problem. A launch just fails somewhere
downstream of the first missing file.

`rm -rf build install log` then a clean `colcon build --symlink-install`: 12
packages, 5.5 s. **Anyone switching between the WSL and native boots has to do
this every time.** Worth a guard in `preflight_stack.sh` that compares a known
install symlink's target against `WORKSPACE_ROOT` — not written this session.

## 1. SITL's simulated u-blox reports an HDOP the aircraft's spec rejects

Q27 requires `HDOP < 1.2` (goal.md:74). The simulated receiver reports exactly
1.21, so "GPS HDOP" was red in every simulated run and the interlock could not
reach 12/12. The gap is 0.01.

Chain, every link read in source:

| Stage | Location | Value |
|---|---|---|
| SITL stub | `SIM_GPS_UBLOX.cpp:234` — `dop.hDOP = 121;` | 121 |
| Driver parses NAV_DOP | `state[0].hdop` | 121 |
| MAVLink out | `AP_GPS.cpp:1485` — `get_hdop(0)` → `GPS_RAW_INT.eph` | 121 |
| MAVROS | `/mavros/gpsstatus/gps1/raw` `.eph` | 121 |
| Readiness | `readiness_node.py:165` — `m.eph / 100.0` | 1.21 |
| Gate | `readiness.py:117` — `hdop < 1.2` | always false |

Two things make it a defect in the simulator rather than a tight threshold.

**There is no parameter.** `grep -rn hdop libraries/SITL/SITL.{cpp,h}` returns
nothing, so unlike `SIM_BATT_VOLTAGE` and `SIM_GPS_NUMSATS` it cannot be set
from `aerothon_sitl.parm`. It is a compile-time literal.

**It is not a model.** Four lines above, `sol.satellites` reads
`_sitl->gps_numsats[instance]` — satellite count is simulated. `dop.hDOP` is
assigned unconditionally, among five siblings all set to 65535, the u-blox
"unknown" sentinel. It is a placeholder in a block of placeholders, and it does
not move when satellites do. Stock SITL presents an 18-satellite fix beside a
dilution figure no receiver reports with 18 satellites in view.

Same defect as the 3S battery and the 10-satellite fix that
`aerothon_sitl.parm` already handles, so it gets the same treatment: **the
threshold is not touched.** The simulator changes, so that what is tested is
what is flown.

`scripts/patch_ardupilot_sitl_gps_hdop.py` rewrites the literal to 80. Wired
into `install_ardupilot_overlay.sh` §1d, unconditionally — this is not a
missing dependency like the other two patches, it is wrong on every machine.

**Why 80 and not 119.** A value that squeaks under the gate passes while
proving nothing, and flips back to red on any later tightening. 0.80 is what
`test_readiness_interlock.py`'s own `healthy()` fixture has always used to mean
a good fix. The patched value is still a constant; it does not respond to
`SIM_GPS_NUMSATS`. It is consistent with the satellite count shipped beside it,
which the stock value is not. Exposing `SIM_GPS*_HDOP` upstream is the correct
fix and belongs in ArduPilot.

**Rejected alternative:** `GPS_TYPE` is 1 (AUTO), so switching `SIM_GPS_TYPE`
to SBP (stub 100) or MSP (stub 100) would auto-detect and needs no rebuild. It
trades one arbitrary constant for another *and* moves the simulated receiver
away from u-blox, which is what the Here3+ (goal.md:55 Q20) is. Nova is worse —
its stub is exactly 1.20, which the strict `<` still rejects.

Regression: `sim/test_sitl_gps_hdop.py`, 8 tests. The load-bearing ones do not
assert that a string was edited — they push the emitted `eph` through
`eph / 100.0` and `evaluate()`, the same two steps the live stack runs, and
assert the item goes green patched and red unpatched. `InstalledOverlay` checks
the real file when the overlay is present and names the remedy in its failure
message; verified red before the patch, green after. A separate test pins that
the patch fails loudly if upstream moves the literal, because a patch that
quietly no-ops would put the interlock back at 1.21 with nothing saying why.

Rebuild is one translation unit: `colcon build --packages-select ardupilot_sitl`,
4.75 s.

## Live result

```
[PASS] GPS satellites  18
[PASS] GPS HDOP        0.8        <- was 1.21, red
[FAIL] EKF health      no data received
[FAIL] Lidar rate      3.7 Hz, need 8.0
```

## The ceiling, re-measured on native Linux

The previous session attributed the RTF ceiling to WSL running everything on
llvmpipe with no GPU. **That was not the whole story.** This machine is native
Linux with working acceleration — `glxinfo` reports `Mesa Intel(R) Graphics
(RPL-P)`, `Accelerated: yes`, 16 cores — and RTF measured from
`/world/mission2/stats` is still only **0.31–0.40** with the GUI up.

Lidar tracks it exactly, as before: 10 sim-Hz × 0.37 = 3.7 Hz. EKF health is
the same cause — 3 sim-Hz is one message every ~3.4 s wall, past `max_stale_s`
of 3.0.

So real GPU acceleration bought roughly 0.29 → 0.37, not the ≥ 0.8 the 8 Hz
lidar gate needs. Under load, `gz sim -s` and the GUI take ~145% and ~133% CPU
with load average 15.6 on 16 cores. **Headless was not measured this session**
and is the obvious next data point. The levers named last session stand, in the
same order: lidar sample count (720 dominates), camera resolution, then running
the rate checks on the simulation clock.

## Not done

- No flight. The stack reaches `WAITING` with a healthy FCU and will not arm.
- Headless RTF not measured on this machine.
- No preflight guard for the WSL/native install-tree mismatch in §0.
- Seed 1001 through delivery, return and landing remains the open acceptance
  gate, untouched.
- Nothing staged or committed.
