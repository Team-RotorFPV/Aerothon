#!/usr/bin/env python3
"""The scissor claw's kinematics (sim_gazebo/claw.py) against its CAD pins.

    python3 -m pytest sim/test_claw.py -v
"""

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src/aerothon_sim/sim_gazebo"))
from sim_gazebo.claw import Claw  # noqa: E402

GEO = json.loads((ROOT / "src/aerothon_sim/sim_gazebo/models/aerothon_quad/"
                  "airframe.json").read_text())["claw"]


class ClawTests(unittest.TestCase):

    def setUp(self):
        self.claw = Claw(GEO)

    def test_as_drawn_it_is_shut_and_nothing_has_moved(self):
        for k, v in self.claw.pose(0.0).items():
            self.assertAlmostEqual(v, 0.0, places=9, msg=k)

    def test_the_links_keep_their_length_as_it_opens(self):
        c = self.claw
        for deg in (5, 15, 30):
            phi = math.radians(deg)
            p = c.pose(phi)
            # Link a, turned by its joint about the top pin (now `drop` lower),
            # still reaches jaw a's pin, turned by its joint about the centre pin.
            T = (c.T0[0], c.T0[1] - p["drop"])
            ang = -p["link_a"]
            v = (c.A0[0] - c.T0[0], c.A0[1] - c.T0[1])
            end = (T[0] + v[0] * math.cos(ang) - v[1] * math.sin(ang),
                   T[1] + v[0] * math.sin(ang) + v[1] * math.cos(ang))
            a = -p["jaw_a"]
            w = (c.A0[0] - c.P0[0], c.A0[1] - c.P0[1])
            pin = (c.P0[0] + w[0] * math.cos(a) - w[1] * math.sin(a),
                   c.P0[1] + w[0] * math.sin(a) + w[1] * math.cos(a))
            self.assertLess(math.dist(end, pin), 1e-6, deg)

    def test_opening_takes_the_top_pin_down_and_is_symmetric(self):
        last = 0.0
        for deg in range(1, 36):
            p = self.claw.pose(math.radians(deg))
            self.assertGreater(p["drop"], last)
            last = p["drop"]
            self.assertAlmostEqual(p["jaw_a"], -p["jaw_b"])
            self.assertAlmostEqual(p["link_a"], -p["link_b"], places=2)

    def test_a_couple_of_mm_of_slack_lets_the_eyelet_go(self):
        """What makes it a gravity release: the tips clear the eyelet within
        about 2 mm of slack, far less than the winch pays out past touchdown."""
        slack = self.claw.pose(self.claw.release)["drop"]
        self.assertLess(slack, 0.003)
        self.assertAlmostEqual(self.claw.phi_for_drop(slack), self.claw.release, places=4)


if __name__ == "__main__":
    unittest.main()
