#!/usr/bin/env python3
"""The headless world's camera cannot see through walls.

Without occlusion it read the return gate's banner 12 m away THROUGH the
near board and the corridor walls, and the mission chased it; a real camera
never could. These pin the line-of-sight test the detectors now go through.

    python3 -m pytest sim/test_headless_world.py -v
"""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))

import world_spec as W                                      # noqa: E402
from headless_world import Arena                            # noqa: E402


class LineOfSightTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.a = Arena(W.load(os.path.join(HERE, "worlds", "shipped.json")), "b")
        cls.out, cls.ret = cls.a.banners

    def board_centre(self, b):
        return (*b["xy"], (b["z0"] + b["z1"]) / 2)

    def test_the_near_board_in_open_air_is_seen(self):
        x, y = self.out["xy"]
        self.assertTrue(self.a.clear_los((x - 4.0, y, 3.0), self.board_centre(self.out)))

    def test_the_far_board_is_hidden_from_gate_height(self):
        """Home at gate height: the near board and the walls are in the way."""
        self.assertFalse(self.a.clear_los((1.3, -0.9, 3.0), self.board_centre(self.ret)))

    def test_from_high_enough_it_is_seen_over_the_walls(self):
        self.assertTrue(self.a.clear_los((1.3, -0.9, 8.0), self.board_centre(self.ret)))

    def test_a_board_does_not_hide_itself(self):
        x, y = self.ret["xy"]
        f = self.ret["facing"]
        eye = (x + 5.0 * math.cos(f), y + 5.0 * math.sin(f), 3.4)
        self.assertTrue(self.a.clear_los(eye, self.board_centre(self.ret)))


if __name__ == "__main__":
    unittest.main()
