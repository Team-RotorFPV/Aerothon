#!/usr/bin/env python3
"""The simulated u-blox must not report a dilution the aircraft's spec rejects.

WHAT WAS WRONG
    Q27 requires HDOP < 1.2. SITL's simulated u-blox hardcodes 121, which
    reaches the interlock as 1.21, so "GPS HDOP" failed in every simulated run
    and the interlock sat at 11/12 at best. Nothing in this repository names
    the cause: the item just reads "HDOP 1.21, need below 1.2" next to a
    perfectly healthy 18-satellite fix.

    The failure is 0.01 wide and there is no SIM_* parameter for it, so it
    cannot be fixed from a .parm file the way SIM_BATT_VOLTAGE and
    SIM_GPS_NUMSATS were. scripts/patch_ardupilot_sitl_gps_hdop.py rewrites the
    literal in the overlay's copy of upstream.

WHAT THIS PINS
    Not that the patch edits a string -- that would pass while the interlock
    still refused to arm. It runs the patched value through the same two steps
    the live stack does (eph/100 in readiness_node, then evaluate()'s gate) and
    asserts the item goes green, and that the UNPATCHED value makes it red.
    That is the assertion that would have caught this before a run.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_sitl_gps_hdop.py -v
"""

import importlib.util
import os
import re
import shutil
import sys
import tempfile
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.insert(0, os.path.join(
    _ROOT, "src", "aerothon_gcs", "gcs_aggregator"))

from gcs_aggregator.readiness import DEFAULT_LIMITS, evaluate

PATCH_SCRIPT = os.path.join(_ROOT, "scripts", "patch_ardupilot_sitl_gps_hdop.py")

# The NAV_DOP block as upstream ships it (ArduPilot Copter-4.5,
# libraries/SITL/SIM_GPS_UBLOX.cpp). Every sibling is 65535 -- the u-blox
# "unknown" sentinel -- which is what makes 121 a placeholder rather than a
# simulated value.
UPSTREAM_BLOCK = """\
    dop.time = gps_tow.ms;
    dop.gDOP = 65535;
    dop.pDOP = 65535;
    dop.tDOP = 65535;
    dop.vDOP = 200;
    dop.hDOP = 121;
    dop.nDOP = 65535;
    dop.eDOP = 65535;
"""

# readiness_node.py:165 -- `self.obs["hdop"] = m.eph / 100.0`. Pinned below so
# a change there fails this test rather than silently invalidating it.
EPH_SCALE = 100.0


def load_patcher():
    spec = importlib.util.spec_from_file_location("patch_hdop", PATCH_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def hdop_item(eph):
    """Run a raw GPS_RAW_INT eph through the live path and return the item."""
    obs = {"hdop": eph / EPH_SCALE}
    return next(i for i in evaluate(obs) if i["key"] == "gps_hdop")


def emitted_eph(source):
    """The hDOP the simulated receiver would put on the wire."""
    match = re.search(r"dop\.hDOP\s*=\s*(\d+);", source)
    assert match, "no dop.hDOP assignment in source"
    return int(match.group(1))


class SimulatedHdopReachesTheInterlock(unittest.TestCase):
    """The whole chain, from the C++ literal to the green/red item."""

    def setUp(self):
        self.patcher = load_patcher()
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir)
        self.path = os.path.join(self.dir, "SIM_GPS_UBLOX.cpp")
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write(UPSTREAM_BLOCK)

    def read(self):
        with open(self.path, encoding="utf-8") as handle:
            return handle.read()

    def test_the_unpatched_simulator_blocks_arming(self):
        """The defect, reproduced: stock SITL can never pass this item."""
        item = hdop_item(emitted_eph(UPSTREAM_BLOCK))
        self.assertFalse(item["ok"])
        self.assertEqual(item["value"], 1.21)
        self.assertIn("need below 1.2", item["reason"])

    def test_the_patched_simulator_passes_the_item(self):
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        item = hdop_item(emitted_eph(self.read()))
        self.assertTrue(item["ok"], item["reason"])
        self.assertEqual(item["value"], 0.8)

    def test_the_patched_value_is_not_merely_under_the_limit(self):
        """A value that squeaks under the gate passes while proving nothing.

        It must also survive the threshold being tightened, and must agree with
        what this suite's own `healthy()` fixture calls a good fix.
        """
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        hdop = emitted_eph(self.read()) / EPH_SCALE
        self.assertLessEqual(hdop, DEFAULT_LIMITS["max_hdop"] - 0.2)
        self.assertEqual(hdop, 0.8)

    def test_the_satellite_count_stays_simulated(self):
        """The patch touches dilution only; it must not disturb the fix."""
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        patched = self.read()
        for sibling in ("gDOP", "pDOP", "tDOP", "nDOP", "eDOP"):
            self.assertIn(f"dop.{sibling} = 65535;", patched)
        self.assertIn("dop.vDOP = 200;", patched)

    def test_patching_twice_changes_nothing(self):
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        once = self.read()
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        self.assertEqual(self.read(), once)

    def test_it_refuses_a_file_whose_layout_moved(self):
        """Upstream changing the literal must fail loudly, not silently pass.

        A patch that quietly no-ops when upstream drifts is worse than no
        patch: the interlock goes back to 1.21 with nothing reporting why.
        """
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("    dop.hDOP = 99;\n")
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 1)

    def test_the_declaration_is_not_mistaken_for_the_assignment(self):
        """`uint16_t hDOP;` sits 137 lines above and must not be rewritten."""
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("        uint16_t hDOP;\n" + UPSTREAM_BLOCK)
        self.assertEqual(self.patcher.patch(self.patcher.Path(self.path)), 0)
        self.assertIn("uint16_t hDOP;", self.read())


class InstalledOverlay(unittest.TestCase):
    """If the overlay is on this machine, check the real file, not a fixture."""

    def setUp(self):
        ws = os.environ.get("AEROTHON_OFFICIAL_WS",
                            os.path.expanduser("~/aerothon_stack"))
        self.path = os.path.join(
            ws, "src", "ardupilot", "libraries", "SITL", "SIM_GPS_UBLOX.cpp")
        if not os.path.isfile(self.path):
            self.skipTest(f"ArduPilot overlay not installed at {self.path}")

    def test_the_installed_simulator_would_pass_the_interlock(self):
        with open(self.path, encoding="utf-8") as handle:
            source = handle.read()
        item = hdop_item(emitted_eph(source))
        self.assertTrue(
            item["ok"],
            f"{self.path} emits HDOP {item['value']}: {item['reason']}. "
            f"Run scripts/patch_ardupilot_sitl_gps_hdop.py on it and rebuild "
            f"SITL.")


if __name__ == "__main__":
    unittest.main()
