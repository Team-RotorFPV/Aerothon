"""The close-up bench must let contact, rather than a command, carry the payload."""

import csv
import inspect
import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "sim"))
import build_cad_vehicle as vehicle  # noqa: E402
import winch_bench as bench  # noqa: E402


class PhysicalHookTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        airframe = json.loads((ROOT / "src/aerothon_sim/sim_gazebo/models/"
                               "aerothon_quad/airframe.json").read_text())
        cls.claw = airframe["claw"]
        cls.model = ET.Element("model", name="test_vehicle")
        vehicle.add_claw(cls.model, cls.claw, "model://test/meshes", detachable=False)

    def test_jaws_have_collision_shapes_that_can_support_the_payload(self):
        for side in ("a", "b"):
            link = self.model.find(f"link[@name='claw_jaw_{side}']")
            collision = link.find("collision")
            self.assertIsNotNone(collision, side)
            bounds = self.claw["jaw_tip_bounds"][side]
            centre = [float(x) for x in collision.findtext("pose").split()[:3]]
            size = [float(x) for x in collision.findtext("geometry/box/size").split()]
            for i in range(3):
                self.assertAlmostEqual(centre[i] + self.claw["centre_pin"][i],
                                       (bounds["min"][i] + bounds["max"][i]) / 2,
                                       places=5)
                self.assertAlmostEqual(size[i],
                                       bounds["max"][i] - bounds["min"][i] + 0.0002)

    def test_the_ring_is_sized_to_the_hook_pockets(self):
        """Each pocket holds one side of the ring: the wire fits the pocket,
        passes the slot (so the open claw can let it go) and its centreline
        runs through both pocket centres, one in each jaw plate."""
        ring, pockets = bench.ring_geometry(self.claw), self.claw["pockets"]
        self.assertLess(ring["wire"], ring["slot"] - 0.0003)
        self.assertLess(ring["wire"], ring["pocket"] - 0.0005)
        for side, sgn in (("a", 1), ("b", -1)):
            self.assertAlmostEqual(ring["x"] + sgn * ring["R"], pockets[side]["centre"][0], places=6)
            y0, y1 = pockets[side]["plate_y"]
            self.assertTrue(y0 - 0.0002 <= ring["y"] <= y1 + 0.0002, side)

    def test_the_claw_starts_clear_of_the_ring_and_can_be_loaded_onto_it(self):
        """The mistake this guards against: the last tab began with the jaws
        and links inside its material, and contact began by ejecting it."""
        self.assertGreaterEqual(bench.ring_clearance(self.claw, 0.0), 0.0)
        # Shut, it cannot come down over the ring: the tips land on the wire.
        self.assertLess(bench.ring_clearance(self.claw, 0.0, -0.002), 0.0)
        # Open, there is a band of heights where it comes down over the ring
        # and closes round it with air to spare -- wide enough to hit.
        window = bench.loading_window(self.claw)
        self.assertIsNotNone(window)
        self.assertGreater(window[1] - window[0], 0.0005)

    def test_bench_collides_with_the_visible_jaw_mesh(self):
        model = ET.fromstring(ET.tostring(self.model))
        bench.use_cad_jaw_collision(model)
        for side in ("a", "b"):
            link = model.find(f"link[@name='claw_jaw_{side}']")
            self.assertEqual(link.findtext("collision/geometry/mesh/uri"),
                             link.findtext("visual/geometry/mesh/uri"))
            # ...and where it is drawn: the tip box's offset must not carry over.
            self.assertEqual(link.findtext("collision/pose") or "0 0 0 0 0 0",
                             link.findtext("visual/pose") or "0 0 0 0 0 0")

    def test_the_payload_ring_and_the_loading_stand_are_collidable(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            geometry = bench.rig_geometry(5.0, (0.10, 0.05, 0.05), self.claw, loading=True)
            bench.write_world(out, 5.0, (0.10, 0.05, 0.05), geometry)
            world = ET.parse(out / "world.sdf")
        link = world.find(".//model[@name='aerothon_payload']/link[@name='body']")
        names = [c.get("name", "") for c in link.findall("collision")]
        self.assertEqual(len([n for n in names if n.startswith("lifting_ring_")]), 24)
        self.assertEqual(len([n for n in names if n.startswith("lifting_leg_")]), 2)
        stand = world.find(".//model[@name='loading_stand']")
        self.assertIsNotNone(stand)
        self.assertEqual(stand.findtext(".//plugin/topic"), "/bench/stage_target")

    def test_payload_has_no_scripted_attach_or_detach(self):
        plugins = self.model.findall("plugin")
        self.assertFalse(any("DetachableJoint" in (p.get("name") or "") for p in plugins))
        source = inspect.getsource(bench.run_drop)
        self.assertNotIn("pub_attach", source)
        self.assertNotIn("pub_detach", source)


class VerdictTests(unittest.TestCase):
    """assess_drop grades measured motion against the measured line length."""

    HEIGHT = 0.05

    def verdict(self, rows):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            with (out / "timeline.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["sim_t", "phase", "winch_state", "payout_m", "line_m",
                            "ctrl_believes_open", "claw_open_deg", "payload_z", "payload_x",
                            "payload_y", "payload_tilt_deg"])
                for t, phase, state, line, z in rows:
                    w.writerow([t, phase, state, "", line, "", 0, z, 0, 0, 0])
            return bench.assess_drop(out, self.HEIGHT)["result"]

    def flight(self, fall_at=None, lift_after=None):
        """Hang at 4.87 m, lower to the ground and past it, stow."""
        rows = [(t * 0.1, "hang check", "IDLE", 0.015, 4.87) for t in range(30)]
        line, z = 0.015, 4.87
        for k in range(170):
            line += 0.03
            z = max(self.HEIGHT / 2, 4.87 - (line - 0.015))
            if fall_at is not None and k >= fall_at:
                z = self.HEIGHT / 2
            rows.append((3 + k * 0.1, "lowering", "LOWERING", line, z))
        for k in range(50):
            line -= 0.1
            # Touchdown was at 4.86 m of line: a re-grab lifts it from there.
            zz = self.HEIGHT / 2 + (max(0.0, 4.86 - line) if lift_after else 0.0)
            rows.append((20 + k * 0.1, "stowing", "STOWING", line, zz))
        return rows

    def test_a_payload_that_follows_the_line_down_and_stays_is_released(self):
        self.assertEqual(self.verdict(self.flight()), "RELEASED_ON_GROUND")

    def test_a_payload_that_leaves_the_line_early_dropped(self):
        self.assertEqual(self.verdict(self.flight(fall_at=40)), "DROPPED_EARLY")

    def test_a_payload_that_comes_back_up_was_regrabbed(self):
        self.assertEqual(self.verdict(self.flight(lift_after=True)), "REGRABBED")

    def test_a_payload_held_up_by_something_else_is_not_following(self):
        rows = self.flight()
        rows = [(t, ph, st, line, 4.87 if ph == "lowering" else z)
                for t, ph, st, line, z in rows]
        self.assertEqual(self.verdict(rows), "NOT_FOLLOWING_LINE")

    def test_a_payload_that_drops_when_the_stand_leaves_was_not_held(self):
        rows = [(t * 0.1, "hang check", "IDLE", 0.015, 4.87 - (0.3 if t > 5 else 0))
                for t in range(30)]
        self.assertEqual(self.verdict(rows), "NOT_HELD_AFTER_LOADING")


if __name__ == "__main__":
    unittest.main()
