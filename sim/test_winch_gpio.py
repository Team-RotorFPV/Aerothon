#!/usr/bin/env python3
"""The Pi GPIO winch backend against a simulated BTS7960 and N20 encoder.

No GPIO is touched: a fake driver records what the H-bridge pins would see
and turns a commanded spool into quadrature edges. These pin that travel is
MEASURED from the encoder, that both PWM sides are never driven together,
that a stall or reversed wiring stops the motor and faults, that the hook at
its stop is the stow reference, and that unmeasured settings refuse to run.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_winch_gpio.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_payload", "winch_ctrl"))

from winch_ctrl.gpio_winch import EncoderWinch          # noqa: E402

GREY = (0b00, 0b01, 0b11, 0b10)       # quadrature sequence, one direction


class FakeBridge:
    """H-bridge + spool + encoder. `wired_sign` -1 models swapped phases."""

    def __init__(self, wired_sign=1):
        self.phase = 0
        self.wired_sign = wired_sign
        self.handler = None
        self.calls = []
        self.direction = 0
        self.jammed = False

    def on_edge(self, handler):
        self.handler = handler

    def read_ab(self):
        return GREY[self.phase % 4]

    def drive(self, direction, duty):
        self.calls.append((direction, duty))
        self.direction = direction

    def turn(self, counts):
        """Advance the spool by the motor's own direction, `counts` edges."""
        if self.jammed or not self.direction:
            return
        step = self.direction * self.wired_sign
        for _ in range(counts):
            self.phase += step
            self.handler()


class BatchedBridge(FakeBridge):
    """lgpio's real delivery: edges are queued and handed over in a batch,
    each report carrying its pin and level, after the pins have moved on."""
    enc = (17, 27)                      # A, B (BCM)

    def turn(self, counts):
        if self.jammed or not self.direction:
            return
        reports = []
        for _ in range(counts):
            before = GREY[self.phase % 4]
            self.phase += self.direction * self.wired_sign
            after = GREY[self.phase % 4]
            changed = before ^ after
            pin = self.enc[0] if changed & 0b10 else self.enc[1]
            level = (after >> 1) & 1 if pin == self.enc[0] else after & 1
            reports.append((pin, level))
        for pin, level in reports:      # the pins already read the final state
            self.handler(pin, level)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def winch(bridge, clock, **kw):
    params = dict(counts_per_output_rev=1192.0, spool_radius_m=0.01,
                  drive_duty_pct=60.0, stall_timeout_s=0.6, stow_window_m=0.15,
                  clock=clock)
    params.update(kw)
    return EncoderWinch(bridge, **params)


class EncoderWinchTests(unittest.TestCase):

    def setUp(self):
        self.bridge, self.clock = FakeBridge(), Clock()
        self.w = winch(self.bridge, self.clock)

    def test_travel_is_measured_from_the_encoder(self):
        self.assertTrue(self.w.command("lower"))
        self.bridge.turn(1192)                         # one output revolution
        self.assertAlmostEqual(self.w.payout_m(), 2 * math.pi * 0.01, places=6)
        self.assertEqual(self.w.invalid_transitions, 0)

    def test_batched_edge_reports_lose_no_counts(self):
        """16 Oct audit: decoding by re-reading the pins saw a batch's final
        state for every edge, so payout was under-measured and the line
        overran max_payout_m."""
        bridge = BatchedBridge()
        w = winch(bridge, self.clock)
        self.assertTrue(w.command("lower"))
        for _ in range(10):
            bridge.turn(119)                           # batches, not single edges
        bridge.turn(2)
        self.assertEqual(w.counts * w.lower_sign, 1192)
        self.assertAlmostEqual(w.payout_m(), 2 * math.pi * 0.01, places=6)
        self.assertEqual(w.invalid_transitions, 0)

    def test_stop_zeroes_the_drive(self):
        self.w.command("lower")
        self.w.command("stop")
        self.assertEqual(self.bridge.calls[-1], (0, 0))
        self.assertEqual(self.w.direction, 0)

    def test_a_stalled_spool_stops_and_faults(self):
        self.w.command("lower")
        self.bridge.turn(400)
        self.bridge.jammed = True
        self.clock.t += 0.7
        self.assertEqual(self.w.update(), "fault")
        self.assertIn("no encoder motion", self.w.fault)
        self.assertEqual(self.bridge.calls[-1], (0, 0))
        self.assertFalse(self.w.command("lower"), "a fault allows only stop")
        self.assertTrue(self.w.command("stop"))

    def test_the_hook_at_its_stop_is_the_stow_reference(self):
        self.w.command("lower")
        self.bridge.turn(500)
        self.w.command("stow")
        self.bridge.turn(480)                          # stops 20 counts short
        self.bridge.jammed = True
        self.clock.t += 0.7
        self.assertEqual(self.w.update(), "stowed")
        self.assertEqual(self.w.counts, 0)
        self.assertEqual(self.w.fault, "")

    def test_reversed_wiring_is_a_fault_not_a_correction(self):
        bridge = FakeBridge(wired_sign=-1)
        w = winch(bridge, self.clock)
        w.command("lower")
        bridge.turn(20)
        self.assertEqual(w.update(), "fault")
        self.assertIn("opposite", w.fault)
        self.assertEqual(bridge.calls[-1], (0, 0))

    def test_lower_sign_maps_the_measured_direction(self):
        bridge = FakeBridge(wired_sign=-1)
        w = winch(bridge, self.clock, lower_sign=-1)
        w.command("lower")
        bridge.turn(1192)
        self.assertIsNone(w.update())
        self.assertGreater(w.payout_m(), 0.06)

    def test_unmeasured_settings_refuse_to_start(self):
        for kw in (dict(counts_per_output_rev=0), dict(spool_radius_m=0.0),
                   dict(lower_sign=0), dict(drive_duty_pct=0)):
            with self.assertRaises(ValueError):
                winch(FakeBridge(), self.clock, **kw)


class WinchNodeGpioTests(unittest.TestCase):
    """The node's state machine on measured travel."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

    def node(self, **params):
        from rclpy.parameter import Parameter
        from winch_ctrl.winch_node import WinchNode
        overrides = [Parameter("backend", value="sim"),
                     Parameter("max_payout_m", value=0.5)]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        node = WinchNode(parameter_overrides=overrides)
        node.backend = "gpio"
        self.addCleanup(node.destroy_node)
        return node

    def test_lowering_stops_at_the_measured_limit(self):
        node = self.node(encoder_counts_per_output_rev=1192.0, spool_radius_m=0.01)
        bridge, clock = FakeBridge(), Clock()
        node._enable_gpio(driver=bridge, clock=clock)
        from std_msgs.msg import String
        node._on_cmd(String(data="lower"))
        self.assertEqual(node.state, "LOWERING")
        bridge.turn(int(0.55 / node.gpio.m_per_count))
        node.integrate(0.2)
        self.assertEqual(node.state, "AT_LIMIT")
        self.assertEqual(bridge.calls[-1], (0, 0))
        self.assertGreaterEqual(node.measured_payout, 0.5)

    def test_unmeasured_gpio_winch_faults_closed(self):
        node = self.node()
        node._enable_gpio(driver=FakeBridge(), clock=Clock())
        self.assertIsNone(node.gpio)
        self.assertEqual(node.state, "FAULT")
        self.assertIn("not measured", node.fault)


if __name__ == "__main__":
    unittest.main()
