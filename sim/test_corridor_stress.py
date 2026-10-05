#!/usr/bin/env python3
"""The corridor navigator, flown closed-loop in the conditions a field brings.

test_slalom_traverse flies the real VelocityController through ray-cast
lanes with a lagging airframe and a clean lidar in still air. The corridor on
the day has wind, a lidar in sunlight and dust, and a flight controller that
answers a velocity command late. This flies the same loop with each of those,
then all of them together.

WIND is modelled as what it does to a multirotor under ArduPilot's velocity
controller (WindDrift): drag, quadratic in the air-relative wind, on a 1.94 kg
airframe with 0.05 m^2 of drag area; the velocity loop's integrator absorbs
the steady part over ~1.5 s and its P term opposes the rest (tau 0.5 s). What
is left is the velocity error the navigator has to live with. The worst
credible case is 8 m/s mean with 4 m/s gusts and 1.5 m/s turbulence across
the lane -- above that, flying stops for everyone.

FINDINGS THIS ENCODES
    * Lidar noise (3 cm), 10% dropout, 1% false short returns, 150 ms scan
      latency and a 1.0 s velocity response are each harmless on their own.
    * Gusty crosswind was not: in the 3.2 m split corridor (1.7 m gaps) the
      airframe was pushed 0.3-0.5 m sideways onto a block in 4 of 6 flights.
      The navigator steers by choosing a heading and did not know it was
      being pushed. The lateral disturbance observer in velocity_controller
      fixed every one of them.
    * The envelope's edge: that same 1.7 m slalom in that wind, flown by an
      airframe that answers velocity commands in 1.0 s rather than 0.6 s,
      still touches. `vel_response_s` must be MEASURED on the aircraft
      (docs/FIELD_READINESS.md); the 3.5 m rulebook corridor with the shipped
      2 m gaps is flown in every combination.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_corridor_stress.py -v
"""

import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src", "aerothon_sim",
                                "sim_gazebo"))

from rclpy.parameter import Parameter                       # noqa: E402
from sim_gazebo.corruptions import LidarCorruptor           # noqa: E402
# The module, not its classes: a TestCase imported by name is collected (and
# its rclpy set-up run) a second time from this file.
import test_slalom_traverse as slalom                       # noqa: E402

RHO = 1.2
SEEDS = (0, 1, 2)
SPLIT = (9.0, 3.2, [(2.0, 0.9, 0.4, 1.3, 0), (4.5, -0.8, 0.4, 1.4, 15),
                    (7.0, 0.7, 0.5, 1.2, 0)])


class WindDrift:
    """World-frame velocity error a gusty wind leaves under the FCU's loop."""

    def __init__(self, mean=8.0, dir_rad=math.pi / 2, gust=4.0, period=6.0,
                 turb=1.5, cda=0.05, mass=1.94, tau_v=0.5, tau_i=1.5, seed=0,
                 dt=0.05):
        self.mean, self.gust, self.period, self.turb = mean, gust, period, turb
        self.k = 0.5 * RHO * cda / mass
        self.tau_v, self.tau_i, self.dt = tau_v, tau_i, dt
        self.rng = np.random.default_rng(seed)
        self.phase = self.rng.uniform(0, 2 * math.pi)
        self.u = np.array([math.cos(dir_rad), math.sin(dir_rad)])
        self.ou = np.zeros(2)
        # The aircraft enters the corridor already trimmed into the mean wind.
        self.trim = self._accel(self.u * mean)
        self.err = np.zeros(2)

    def _accel(self, w):
        return self.k * np.linalg.norm(w) * w

    def __call__(self, t):
        dt = self.dt
        self.ou += -self.ou * dt + self.turb * math.sqrt(2 * dt) * \
            self.rng.normal(size=2)
        speed = self.mean + self.gust * math.sin(2 * math.pi * t / self.period
                                                 + self.phase)
        a = self._accel(self.u * speed + self.ou)
        self.trim += (a - self.trim) * dt / self.tau_i
        self.err += ((a - self.trim) - self.err / self.tau_v) * dt
        return float(self.err[0]), float(self.err[1])


def each_seed(case):
    """SEEDS, each on a fresh controller (and, for a custom lane, fresh
    geometry): the framework set up the first, tear down and re-set the rest."""
    for i, seed in enumerate(SEEDS):
        if i:
            case.tearDown()
            case.setUp()
        yield seed


def dirty_lidar(seed):
    lc = LidarCorruptor({"noise_m": 0.03, "dropout": 0.10, "spurious": 0.01},
                        seed=seed)
    return lambda ranges: lc.apply(ranges, 0.05, 12.0)


class ShippedLaneStressTests(slalom.SlalomTraverseTests):
    """The shipped return slalom (2 m gaps in a 3.5 m lane): every condition."""

    def _through(self, **kw):
        _, closest, _ = self.fly(13.8, -4.0, math.pi, goal_x=4.5, max_t=150, **kw)
        self.assertGreaterEqual(closest, slalom.AIRFRAME_R)

    def test_gusty_crosswind(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(drift=WindDrift(seed=seed))

    def test_dirty_lidar(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(corrupt=dirty_lidar(seed))

    def test_slow_airframe_and_late_scans(self):
        self._through(vel_tau_s=1.0, scan_delay_s=0.15)

    def test_everything_at_once(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(drift=WindDrift(seed=seed),
                              corrupt=dirty_lidar(seed),
                              vel_tau_s=1.0, scan_delay_s=0.15)

    # The inherited still-air tests already run in test_slalom_traverse.
    test_the_return_slalom_is_flown_without_contact = None
    test_from_the_lane_centre_too = None
    test_the_forward_lane_with_no_blocks_goes_straight_through = None


class SplitLaneStressTests(slalom.CustomSlalomTests):
    """A 3.2 m lane with 1.7 m gaps: narrower than the rulebook's."""

    def _through(self, **kw):
        self.lane(*SPLIT)
        _, closest, _ = self.fly(self.L - 0.7, 0.0, math.pi, goal_x=0.3,
                                 max_t=150, **kw)
        self.assertGreaterEqual(closest, slalom.AIRFRAME_R)

    def test_gusty_crosswind(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(drift=WindDrift(seed=seed))

    def test_gusty_crosswind_with_dirty_lidar(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(drift=WindDrift(seed=seed),
                              corrupt=dirty_lidar(seed))

    def test_gusty_crosswind_with_late_scans(self):
        for seed in each_seed(self):
            with self.subTest(seed=seed):
                self._through(drift=WindDrift(seed=seed), scan_delay_s=0.15)

    def test_without_the_observer_the_wind_wins(self):
        """Guards the fixture: the wind here is strong enough to matter. With
        the disturbance observer off, some of these flights end on a block."""
        touched = 0
        for seed in each_seed(self):
            self.node.set_parameters([Parameter("drift_comp_gain", value=0.0)])
            try:
                self._through(drift=WindDrift(seed=seed))
            except AssertionError:
                touched += 1
        self.assertGreater(touched, 0)

    # The inherited layouts run in still air in test_slalom_traverse.
    for _name in [n for n in dir(slalom.CustomSlalomTests) if n.startswith("test_")]:
        locals()[_name] = None
    del _name


if __name__ == "__main__":
    unittest.main()
