#!/usr/bin/env python3
"""Phase 11 — the randomised arena actually randomises.

WHY THIS FILE EXISTS

    `scripts/materialize_world.py --randomise-arena` is the foundation of the
    Phase 11 regression: it is what makes "derived from what the camera sees"
    distinguishable from "derived from a different constant that happens to
    agree". It was written, documented, and recorded as working.

    It raised NameError on every invocation:

        File "scripts/materialize_world.py", line 243, in main
            arena = randomise_arena(world, random.Random(args.seed or None))
        NameError: name 'randomise_arena' is not defined

    `if __name__ == "__main__": main()` had been placed immediately after
    main(), ABOVE the definitions of randomise_arena() and _set_pose(), so
    those names were not bound by the time main() ran. Nothing caught it
    because the default (non-randomised) path never touches them, and every
    live run to date used the default path.

    A regression harness that cannot vary the arena is worse than none: it
    reports five passes on five identical arenas.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_arena_randomisation.py -v
"""

import json
import math
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts" / "materialize_world.py"
SOURCE = ROOT / "src" / "aerothon_sim" / "sim_gazebo" / "worlds" / "mission2.sdf"
ASSETS = ROOT / "src" / "aerothon_sim" / "sim_gazebo" / "materials"

TRACKED = ("restricted_red_zone_main", "restricted_red_zone_northwest",
           "restricted_red_zone_south", "delivery_qr_target_a",
           "delivery_qr_target_b", "delivery_qr_target_c",
           "delivery_qr_target_d", "delivery_qr_target_e",
           "forward_aerothon_banner")


def materialise(seed=None, out=None):
    """Run the script the way launch_level6_sim.sh does — through the env."""
    env = dict(os.environ)
    if seed is not None:
        env["AEROTHON_SEED"] = str(seed)
        env["AEROTHON_RANDOM_ARENA"] = "1"
    else:
        env.pop("AEROTHON_RANDOM_ARENA", None)
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--source", str(SOURCE),
         "--assets", str(ASSETS), "--output", str(out)],
        capture_output=True, text=True, env=env, cwd=str(ROOT))
    return proc


def poses(sdf_text):
    """{model: (x, y, yaw)} — the <pose> is the LAST tag in a model block."""
    found = {}
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', sdf_text,
                         re.S):
        name, body = m.group(1), m.group(2)
        if name not in TRACKED:
            continue
        p = re.findall(r'</link>\s*<pose>([^<]*)</pose>', body)
        if p:
            v = p[-1].split()
            found[name] = (float(v[0]), float(v[1]), float(v[5]))
    return found


class RandomisationRunsAtAllTests(unittest.TestCase):
    """The NameError. It exited non-zero and wrote no world at all."""

    def test_the_randomised_path_does_not_crash(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "w.sdf"
            proc = materialise(seed=1001, out=out)
            self.assertEqual(proc.returncode, 0,
                             f"--randomise-arena failed:\n{proc.stderr}")
            self.assertTrue(out.exists(), "no world file was written")

    def test_the_default_path_still_works(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "w.sdf"
            proc = materialise(seed=None, out=out)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertTrue(out.exists())

    def test_the_layout_is_reported_so_a_failure_can_be_reproduced(self):
        """A regression run that cannot say which arena failed is a
        percentage, not a result."""
        with tempfile.TemporaryDirectory() as d:
            proc = materialise(seed=1001, out=Path(d) / "w.sdf")
            self.assertIn("RANDOMISED ARENA", proc.stdout)
            blob = proc.stdout.split("RANDOMISED ARENA:", 1)[1]
            layout = json.loads(blob.split("\n", 1)[0])
            for key in ("gate", "pads", "red_zones", "zone"):
                self.assertIn(key, layout)


class ThingsActuallyMoveTests(unittest.TestCase):
    """Running without crashing is not the same as randomising."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cls.a = Path(d) / "a.sdf"
        cls.b = Path(d) / "b.sdf"
        cls.plain = Path(d) / "plain.sdf"
        materialise(seed=1001, out=cls.a)
        materialise(seed=1002, out=cls.b)
        materialise(seed=None, out=cls.plain)
        cls.pa, cls.pb = poses(cls.a.read_text()), poses(cls.b.read_text())
        cls.pp = poses(cls.plain.read_text())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_tracked_model_is_found(self):
        """Guards the parser: if the <pose> extraction silently found nothing,
        every 'it moved' assertion below would be vacuous."""
        for name in TRACKED:
            self.assertIn(name, self.pp, f"{name} not located in the world")

    def test_two_seeds_give_different_arenas(self):
        self.assertNotEqual(self.pa, self.pb)

    def test_the_pads_move(self):
        moved = [n for n in self.pa if "target" in n
                 and self.pa[n][:2] != self.pb[n][:2]]
        self.assertEqual(len(moved), 5, f"only {len(moved)} of 5 pads moved")

    def test_the_red_zones_move(self):
        moved = [n for n in self.pa if "red_zone" in n
                 and self.pa[n][:2] != self.pb[n][:2]]
        self.assertEqual(len(moved), 3, f"only {len(moved)} of 3 red zones moved")

    def test_the_gate_changes_HEADING_not_just_position(self):
        """A gate that only translates leaves the corridor axis fixed, and a
        stack that assumed the axis would still pass."""
        ya = self.pa["forward_aerothon_banner"][2]
        yb = self.pb["forward_aerothon_banner"][2]
        self.assertNotAlmostEqual(ya, yb, places=6)
        self.assertTrue(any(abs(y) > 1e-6 for y in (ya, yb)),
                        "the gate never rotates away from the nominal axis")

    def test_randomising_changes_things_versus_the_default_world(self):
        self.assertNotEqual(self.pa, self.pp)

    def test_the_same_seed_is_reproducible(self):
        """A failing arena has to be re-flyable, or the failure cannot be
        investigated."""
        with tempfile.TemporaryDirectory() as d:
            again = Path(d) / "again.sdf"
            materialise(seed=1001, out=again)
            self.assertEqual(poses(again.read_text()), self.pa)


class ArenaValidityTests(unittest.TestCase):
    """A randomised arena still has to be one the aircraft can fly.

    The first version set the banner, the wall block and the green surround
    all to the gate position. They are not co-located: in the shipped arena
    the banner is at (2, 2) and the 10.2 m wall block is centred at (7, 0),
    five metres further in. Collapsing them onto one point left the CENTRED
    wall block extending 5.1 m behind the banner, and for seed 1003 the
    takeoff point ended up at corridor-local (-3.77, 1.72) — inside the
    corridor channel. The aircraft reached 0.7 m and disarmed.

    An arena the aircraft cannot fly is not evidence about the aircraft.
    """

    WALL_HALF_LEN = 5.1        # 10.2 m block, centred
    INNER_Y, OUTER_Y = 0.20, 3.80

    def corridor_pose(self, sdf_text):
        m = re.search(r'<model name="corridor_walls">(.*?)</model>',
                      sdf_text, re.S)
        p = re.findall(r'</link>\s*<pose>([^<]*)</pose>', m.group(1))[-1].split()
        return float(p[0]), float(p[1]), float(p[5])

    def takeoff_is_inside_the_channel(self, sdf_text):
        """Is the origin (where the aircraft spawns) between the walls?"""
        cx, cy, yaw = self.corridor_pose(sdf_text)
        dx, dy = -cx, -cy
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return (abs(lx) <= self.WALL_HALF_LEN
                and self.INNER_Y <= abs(ly) <= self.OUTER_Y)

    def test_the_shipped_arena_starts_OUTSIDE_the_corridor(self):
        """The reference the randomised ones have to preserve."""
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "w.sdf"
            materialise(seed=None, out=out)
            self.assertFalse(self.takeoff_is_inside_the_channel(out.read_text()))

    def test_no_seed_spawns_the_aircraft_inside_the_corridor(self):
        with tempfile.TemporaryDirectory() as d:
            for seed in range(1001, 1013):
                out = Path(d) / f"w{seed}.sdf"
                materialise(seed=seed, out=out)
                self.assertFalse(
                    self.takeoff_is_inside_the_channel(out.read_text()),
                    f"seed {seed} spawns the aircraft inside the corridor")

    def test_the_corridor_keeps_its_SHAPE_when_moved(self):
        """The banner-to-wall-block offset is the corridor. Randomising the
        gate must translate it, not collapse it."""
        with tempfile.TemporaryDirectory() as d:
            plain, moved = Path(d) / "p.sdf", Path(d) / "m.sdf"
            materialise(seed=None, out=plain)
            materialise(seed=1003, out=moved)

            def offset(text):
                cx, cy, _ = self.corridor_pose(text)
                b = poses(text)["forward_aerothon_banner"]
                return math.hypot(cx - b[0], cy - b[1])

            self.assertAlmostEqual(offset(moved.read_text()),
                                   offset(plain.read_text()), places=2,
                                   msg="the corridor changed shape when moved")

    def obstacles_in_forward_channel(self, sdf_text):
        """Static obstacles that end up inside the forward corridor lane.

        The corridor is one structure: forward lane, return lane, both
        banners, and the return lane's static obstacles. The first rigid-move
        took only the forward three, so a rotated forward corridor was driven
        through obstacles that had not moved.

        Seed 1002 put pillar `o4c` (0.35 x 1.45 x 3.4 m, tall enough to span
        the 3 m corridor altitude) 0.97 m from where the aircraft jammed. It
        was correctly aligned — heading -11.4 deg against a -11.2 deg
        corridor, gap bearing 0.0 the whole way in — and simply flew into
        something the harness had placed in its path. Three of five Phase 11
        failures were this.
        """
        wx, wy, wyaw = self.corridor_pose(sdf_text)
        m = re.search(r'<model name="return_static_obstacles">(.*?)</model>',
                      sdf_text, re.S)
        if m is None:
            return []
        p = re.findall(r'</link>\s*<pose>([^<]*)</pose>', m.group(1))[-1].split()
        ox, oy, oyaw = float(p[0]), float(p[1]), float(p[5])
        hits = []
        for c in re.finditer(r'<collision name="(o\dc)"><pose>([^<]*)</pose>',
                             m.group(1)):
            v = [float(q) for q in c.group(2).split()]
            gx = ox + v[0] * math.cos(oyaw) - v[1] * math.sin(oyaw)
            gy = oy + v[0] * math.sin(oyaw) + v[1] * math.cos(oyaw)
            dx, dy = gx - wx, gy - wy
            lx = dx * math.cos(wyaw) + dy * math.sin(wyaw)
            ly = -dx * math.sin(wyaw) + dy * math.cos(wyaw)
            if abs(lx) <= self.WALL_HALF_LEN and self.INNER_Y <= ly <= self.OUTER_Y:
                hits.append((c.group(1), round(lx, 2), round(ly, 2)))
        return hits

    def test_no_seed_puts_an_obstacle_in_the_FORWARD_channel(self):
        with tempfile.TemporaryDirectory() as d:
            for seed in range(1001, 1013):
                out = Path(d) / f"w{seed}.sdf"
                materialise(seed=seed, out=out)
                hits = self.obstacles_in_forward_channel(out.read_text())
                self.assertEqual(hits, [],
                                 f"seed {seed} flies the forward corridor "
                                 f"through static obstacles: {hits}")

    def test_the_shipped_arena_has_a_clear_forward_channel(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "w.sdf"
            materialise(seed=None, out=out)
            self.assertEqual(self.obstacles_in_forward_channel(out.read_text()), [])

    def test_the_return_corridor_moves_WITH_the_forward_one(self):
        """They are one structure; moving half of it invents an arena that
        could not exist."""
        with tempfile.TemporaryDirectory() as d:
            plain, moved = Path(d) / "p.sdf", Path(d) / "m.sdf"
            materialise(seed=None, out=plain)
            materialise(seed=1002, out=moved)

            def gap(text):
                fx, fy, _ = self.corridor_pose(text)
                m = re.search(r'<model name="return_static_obstacles">(.*?)</model>',
                              text, re.S)
                p = re.findall(r'</link>\s*<pose>([^<]*)</pose>',
                               m.group(1))[-1].split()
                return round(math.hypot(fx - float(p[0]), fy - float(p[1])), 2)

            self.assertAlmostEqual(gap(moved.read_text()),
                                   gap(plain.read_text()), places=1,
                                   msg="the return obstacles did not move with "
                                       "the corridor")

    def test_the_gate_still_actually_MOVES(self):
        """Guards the fix against the lazy way of passing the tests above."""
        with tempfile.TemporaryDirectory() as d:
            plain, moved = Path(d) / "p.sdf", Path(d) / "m.sdf"
            materialise(seed=None, out=plain)
            materialise(seed=1003, out=moved)
            a = poses(plain.read_text())["forward_aerothon_banner"]
            b = poses(moved.read_text())["forward_aerothon_banner"]
            self.assertGreater(math.hypot(a[0] - b[0], a[1] - b[1]), 0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
