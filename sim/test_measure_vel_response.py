#!/usr/bin/env python3
"""scripts/measure_vel_response.py reads a first-order lag back correctly.

    python3 -m pytest sim/test_measure_vel_response.py -v
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "scripts"))

from measure_vel_response import time_constant               # noqa: E402


def response(tau, v0, v1, noise=0.0, rate_hz=30.0, seconds=4.0, seed=0):
    rng = random.Random(seed)
    return [(k / rate_hz, v1 + (v0 - v1) * math.exp(-(k / rate_hz) / tau)
             + rng.gauss(0.0, noise)) for k in range(int(seconds * rate_hz))]


class TimeConstantTests(unittest.TestCase):

    def test_reads_back_the_lag_in_both_directions(self):
        for tau in (0.4, 0.6, 1.0):
            for v0, v1 in ((0.0, 0.5), (0.5, 0.0), (0.0, -0.5)):
                with self.subTest(tau=tau, v0=v0, v1=v1):
                    self.assertAlmostEqual(
                        time_constant(response(tau, v0, v1), 0.0, v0, v1), tau,
                        delta=0.04)

    def test_gps_velocity_noise_reads_early_not_wildly(self):
        got = time_constant(response(0.8, 0.0, 0.5, noise=0.03), 0.0, 0.0, 0.5)
        self.assertLess(abs(got - 0.8), 0.25)

    def test_a_step_never_followed_says_so(self):
        self.assertIsNone(time_constant(response(0.6, 0.0, 0.0), 0.0, 0.0, 0.5))


if __name__ == "__main__":
    unittest.main()
