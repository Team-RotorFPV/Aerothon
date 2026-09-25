#!/usr/bin/env python3
"""Make SITL's simulated u-blox report an HDOP consistent with its own fix.

WHY
    The Q27 arming interlock requires HDOP < 1.2 (goal.md Q27). SITL's
    simulated u-blox reports exactly 1.21, so "GPS HDOP" failed in every
    simulated run and the interlock could never reach 12/12 — blocking arming
    for a reason that is nowhere in this repository.

    libraries/SITL/SIM_GPS_UBLOX.cpp, in the NAV_DOP block:

        dop.gDOP = 65535;
        dop.pDOP = 65535;
        dop.tDOP = 65535;
        dop.vDOP = 200;
        dop.hDOP = 121;      <-- this
        dop.nDOP = 65535;
        dop.eDOP = 65535;

    That 121 is a compile-time literal. There is no SIM_* parameter for it:
    `grep -rn hdop libraries/SITL/SITL.{cpp,h}` returns nothing, unlike
    SIM_GPS_NUMSATS and SIM_BATT_VOLTAGE which are both exposed.

    It is also not a model. Four lines above, `sol.satellites` reads
    `_sitl->gps_numsats[instance]`, so satellite count is simulated —
    `dop.hDOP` is assigned unconditionally, sitting among five siblings set to
    65535, the u-blox "unknown" sentinel. It is a placeholder in a block of
    placeholders, and it does not move when satellites do. Stock SITL therefore
    presents an 18-satellite fix alongside a dilution figure no receiver would
    report with 18 satellites in view.

    This is the same defect as the 3S battery and 10-satellite fix already
    handled in sim_gazebo/config/aerothon_sitl.parm, and it gets the same
    treatment for the same reason: the interlock threshold is the aircraft's
    and is NOT touched. The simulator is what changes, so that what is tested
    is what is flown. The aircraft carries a Here3+ (goal.md Q20), a u-blox
    based receiver, which is why the u-blox backend stays selected rather than
    switching SIM_GPS_TYPE to a backend with a friendlier stub.

THE VALUE
    80, i.e. HDOP 0.80 — the value sim/test_readiness_interlock.py's own
    `healthy()` fixture has always used to mean "a good fix".

    Deliberately not 119. A value that merely squeaks under the limit passes
    the gate while telling you nothing, and would flip back to failing on any
    later tightening of the threshold.

    This remains a constant. The patch does not add a DOP model and does not
    make HDOP respond to SIM_GPS_NUMSATS; it makes the constant consistent
    with the satellite count shipped beside it, which the stock value is not.

PREFERRED ALTERNATIVE
    Expose it upstream as a SIM_GPS*_HDOP parameter alongside SIM_GPS*_NUMSATS
    and drop this patch. That is the correct fix and belongs in ArduPilot, not
    here; this patch exists because the overlay is built from upstream and
    needs to work today.

Idempotent; only ever rewrites one integer literal.

Usage:  patch_ardupilot_sitl_gps_hdop.py <path/to/libraries/SITL/SIM_GPS_UBLOX.cpp>
Exit:   0 patched or already patched, 1 unexpected file layout.
"""

import sys
from pathlib import Path

MARKER = "// AEROTHON: HDOP (see scripts/patch_ardupilot_sitl_gps_hdop.py)"

# The upstream literal, and what it becomes. Matched with surrounding
# whitespace so this cannot collide with the `uint16_t hDOP;` declaration.
UPSTREAM = "    dop.hDOP = 121;"
PATCHED = f"    dop.hDOP = 80;   {MARKER}"


def patch(path: Path) -> int:
    text = path.read_text()

    if MARKER in text:
        print(f"[skip] already patched: {path}")
        return 0

    occurrences = text.count(UPSTREAM)
    if occurrences != 1:
        print(f"[ERROR] expected exactly one {UPSTREAM.strip()!r} in {path}, "
              f"found {occurrences}; upstream layout changed",
              file=sys.stderr)
        return 1

    backup = path.with_suffix(path.suffix + ".aerothon-orig")
    if not backup.exists():
        backup.write_text(text)
    path.write_text(text.replace(UPSTREAM, PATCHED))

    print(f"[patched] {path}  (simulated u-blox HDOP 1.21 -> 0.80)")
    print(f"[backup ] {backup}")
    return 0


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"[ERROR] not a file: {path}", file=sys.stderr)
        return 1
    return patch(path)


if __name__ == "__main__":
    sys.exit(main())
