#!/usr/bin/env python3
"""Fail-closed behaviour of individual mission stages.

Three defects from the "still broken" list, each of which let the mission carry
on believing something it had not established:

  1. ScanStartQR returned SUCCESS with an empty target after a timeout, so a
     mission that never read its delivery target proceeded to deliver.
  2. No stage checked altitude, so a live run reported CORRIDOR_NAV while the
     aircraft dragged along the ground at z = 0.117 m.
  3. A failed mission Sequence made the root Selector return FAILURE, and
     py_trees restarted the memory Sequence from the top on the next tick —
     mid-flight. Observed as SEARCH_QR -> START_QR.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_fail_closed_stages.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

import py_trees

from mission_bt.scan_geometry import no_surface
from mission_bt.mission_tree import (
    Corridor,
    ScanStartQR,
    Takeoff,
    latch_mission_failure,
    latch_mission_success,
)


class FakeMav:
    """Minimal stand-in exposing only what these stages touch."""

    def __init__(self):
        self.qr_decoded = ""
        self.qr_streak = 0
        self.target_override = ""
        self.target_set = None
        self.abort_reason = ""
        self.mission_failed = False
        self.mission_started = True
        self.airborne_floor = None
        self.avoidance = None
        self.avoid_hold_alt = None
        self.results = []
        self.exited = False
        self.stuck = False
        self.avoid_detail = {}
        self._alt = 5.0
        self._pos = (0.0, 0.0, 5.0)
        self._confident_at = 3
        self._yaw = 0.0
        self.corridor_exit_pose = None
        self.observed_zone = None
        self.gotos = []
        self.qr_offset_xy = None
        self.qr_off = None
        self.qr_matched = False
        self.mission_complete = False
        self.landing_precision = "UNKNOWN"
        self.logs = []
        self.banner_z = 0.0
        self.banner_x = 0.0
        self.banner_y = 0.0
        self.banner_reject = ""
        # THE LIDAR SEAM. The tree asks for the flat face in a sector and gets
        # back the dict `fit_surface` returns, or a refusal. Nothing here
        # builds a LaserScan: the geometry is tested against synthetic scans
        # in test_scan_geometry.py, and the stages are tested against the
        # answers those scans would produce. Neither test grades a copy of the
        # other's arithmetic.
        #
        # The default is a REFUSAL, deliberately. A fake that reports a
        # perfect surface until told otherwise would let a stage that never
        # asks the lidar anything pass every test in the suite.
        self.surface = None
        self.surface_calls = []
        self.square_on = []
        self.camera_poses = []

    # ---- stage interface ---- #
    def goto(self, *a, **k):
        self.gotos.append(a)

    def alt(self):
        return self._alt

    def pos(self):
        return self._pos

    def set_target(self, s):
        self.target_set = s

    def qr_confident(self, frames):
        return bool(self.qr_decoded) and self.qr_streak >= frames

    def enable_avoidance(self, on, hold_alt=None):
        self.avoidance = on
        self.avoid_hold_alt = hold_alt

    def corridor_exited(self):
        return self.exited

    def corridor_entered(self):
        return bool((self.avoid_detail or {}).get("corridor_entered", False))

    def banner_identified(self):
        return self.banner_z >= 1.0

    def banner_bearing(self):
        return self.banner_x if self.banner_identified() else 0.0

    def banner_rejection_summary(self, top=3):
        return self.banner_reject or "nothing green ever entered the frame"

    def surface_ahead(self, bearing_rad, half_width_rad,
                      expected_range_m=None, **kw):
        self.surface_calls.append((bearing_rad, half_width_rad,
                                   expected_range_m))
        if callable(self.surface):
            return self.surface(self, bearing_rad, half_width_rad)
        if self.surface is None:
            return no_surface("no lidar scan has arrived on /scan")
        return self.surface

    def publish_square_on(self, payload):
        self.square_on.append(payload)

    def set_camera_pose(self, pose):
        self.camera_poses.append(pose)

    def avoidance_stuck(self):
        return self.stuck

    def set_airborne_floor(self, f):
        self.airborne_floor = f

    def clear_airborne_floor(self):
        self.airborne_floor = None

    def takeoff(self, alt):
        pass

    def log(self, msg, warn=False):
        self.logs.append((msg, warn))

    def publish_result(self, state, reason=""):
        self.results.append((state, reason))

    # ---- observed geometry (audit A7, A8, A9) ---- #
    def yaw(self):
        return self._yaw

    def record_corridor_exit(self):
        if self.corridor_exit_pose is None:
            x, y, z = self.pos()
            self.corridor_exit_pose = (x, y, z, self.yaw())
        return self.corridor_exit_pose

    def open_extent(self):
        d = self.avoid_detail or {}
        return (float(d.get("open_depth_m", 0.0) or 0.0),
                float(d.get("open_width_m", 0.0) or 0.0))

    def home_local_xy(self):
        return (0.0, 0.0)

    def reached(self, x, y, z, tol=0.6):
        px, py, pz = self.pos()
        return (abs(px - x) <= tol and abs(py - y) <= tol and abs(pz - z) <= tol)

    def qr_visible(self):
        return self.qr_off is not None or self.qr_offset_xy is not None

    @property
    def qr_offset(self):
        from geometry_msgs.msg import Vector3
        ox, oy = self.qr_off or (0.0, 0.0)
        # z mirrors the real detector: it says WHAT the offset refers to.
        # Hardcoding 0.0 made the fake permanently report "no target", which
        # is not what a FakeMav with an offset set should mean.
        z = 1.0 if self.qr_off is not None else 0.0
        return Vector3(x=float(ox), y=float(oy), z=float(z))

    def qr_hold_xy(self, default=(0.0, 0.0)):
        return self.qr_offset_xy or default


def run(leaf, n):
    """Tick until terminal or n ticks; py_trees resets a finished leaf."""
    for _ in range(n):
        leaf.tick_once()
        if leaf.status != py_trees.common.Status.RUNNING:
            break
    return leaf.status


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class _LaggedGateMav(FakeMav):
    """Gate-height fake whose pose approaches setpoints over several ticks."""

    def __init__(self, opening_below_m=2.6, blocked_below=False,
                 never_opens=False):
        super().__init__()
        self._alt = 3.0
        self._pos = (0.0, 0.0, 3.0)
        self.opening_below_m = float(opening_below_m)
        self.blocked_below = bool(blocked_below)
        self.never_opens = bool(never_opens)
        self.opening_calls = []

    def opening_ahead(self, bearing_rad, half_width_rad, need_clear_m=10.0):
        self.opening_calls.append((self._alt, need_clear_m))
        if self.never_opens or self._alt > self.opening_below_m + 0.01:
            return {"open": False, "clusters": 1, "gap_m": 0.0,
                    "gate_m": None, "clear_m": 0.0, "reason": "one continuous board face"}
        if self.blocked_below:
            return {"open": False, "clusters": 3, "gap_m": 3.8,
                    "gate_m": 5.0, "clear_m": 2.5,
                    "reason": "something standing 2.5 m into the opening"}
        return {"open": True, "clusters": 2, "gap_m": 3.8,
                "gate_m": 5.0, "clear_m": 12.0, "reason": ""}

    def move_toward_last_setpoint(self, gain=0.5):
        if not self.gotos:
            return
        tx, ty, tz, *_ = self.gotos[-1]
        x, y, z = self._pos
        self._pos = (x + gain * (tx - x),
                     y + gain * (ty - y),
                     z + gain * (tz - z))
        self._alt = self._pos[2]


class DuckUnderBoardTests(unittest.TestCase):

    def test_return_crosses_the_board_then_hands_off_before_the_obstacle(self):
        from mission_bt.mission_tree import GateAdvance
        from test_scan_geometry import scan_of, wall, post, ANGLE_MIN, ANGLE_INC
        from mission_bt.scan_geometry import gate_opening

        class ReturnGate(_LaggedGateMav):
            def opening_ahead(self, bearing_rad, half_width_rad, need_clear_m=10.0):
                segments = [post(5.0, 0.0, -1.9), post(5.0, 0.0, 1.9),
                            wall(7.1, 0.0, half_length_m=0.45)]
                if self._alt > self.opening_below_m:
                    segments.append(wall(5.0, 0.0, half_length_m=1.9))
                return gate_opening(ANGLE_MIN, ANGLE_INC, scan_of(segments),
                                    bearing_rad, half_width_rad,
                                    need_clear_m=need_clear_m)

        mav = ReturnGate()
        duck = self._fly(mav)
        self.assertIs(duck.status, py_trees.common.Status.SUCCESS, mav.abort_reason)
        advance = GateAdvance(mav, advance_m=lambda: duck.advance_m,
                              alt=lambda: duck.alt, tol=0.25)
        advance.tick_once()
        target = mav.gotos[-1]
        self.assertGreater(target[0] - advance.tol, 5.1)
        self.assertLess(target[0], 7.1 - 0.75)
        for _ in range(50):
            mav.move_toward_last_setpoint()
            advance.tick_once()
            if advance.status != py_trees.common.Status.RUNNING:
                break
        self.assertIs(advance.status, py_trees.common.Status.SUCCESS)
        self.assertGreater(mav.pos()[0], 5.1)
        self.assertLess(mav.pos()[0], 7.1 - 0.75)

    def _fly(self, mav, floor_m=1.2, ticks=120):
        from mission_bt.mission_tree import DuckUnderBoard
        clock = _FakeClock()
        leaf = DuckUnderBoard(mav, step_m=0.4, floor_m=floor_m,
                              margin_m=0.6, settle_s=0.0,
                              arrive_tol_m=0.02, clock=clock)
        for _ in range(ticks):
            leaf.tick_once()
            if leaf.status != py_trees.common.Status.RUNNING:
                return leaf
            mav.move_toward_last_setpoint()
            clock.advance(0.1)
        self.fail("DuckUnderBoard did not terminate")

    def test_it_finds_the_edge_and_descends_with_vehicle_lag(self):
        mav = _LaggedGateMav(opening_below_m=2.6)
        mav.airborne_floor = 2.0

        leaf = self._fly(mav)

        self.assertIs(leaf.status, py_trees.common.Status.SUCCESS)
        self.assertLess(leaf.alt, 2.1, "did not leave clearance below the board")
        self.assertGreaterEqual(leaf.alt, 1.2, "descended through the safety floor")
        self.assertTrue(any(2.0 < call[0] < 3.0 for call in mav.opening_calls))
        self.assertTrue(all(abs(g[0]) < 1e-9 and abs(g[1]) < 1e-9
                            for g in mav.gotos),
                        "the edge search translated toward the board")
        self.assertLess(mav.airborne_floor, 1.2,
                        "the takeoff floor would abort the commanded descent")
        self.assertIn("lower edge", " ".join(msg for msg, _ in mav.logs))

    def test_an_obstruction_below_the_board_is_refused(self):
        mav = _LaggedGateMav(opening_below_m=2.6, blocked_below=True)

        leaf = self._fly(mav, floor_m=1.7)

        self.assertIs(leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("Refusing to advance", mav.abort_reason)
        self.assertIn("standing", mav.abort_reason)
        self.assertTrue(all(abs(g[0]) < 1e-9 and abs(g[1]) < 1e-9
                            for g in mav.gotos))

    def test_no_transition_before_the_floor_is_refused_with_measurements(self):
        mav = _LaggedGateMav(never_opens=True)

        leaf = self._fly(mav, floor_m=1.7)

        self.assertIs(leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("reached the floor", mav.abort_reason)
        self.assertIn("Altitudes tried", mav.abort_reason)
        self.assertIn("3.0 m", mav.abort_reason)
        self.assertGreaterEqual(len(mav.opening_calls), 3)

    def test_the_advance_after_the_duck_never_uses_board_height(self):
        from mission_bt.mission_tree import GateAdvance
        mav = _LaggedGateMav(opening_below_m=2.6)
        duck = self._fly(mav)
        advance = GateAdvance(mav, advance_m=10.0, alt=lambda: duck.alt)
        advance.initialise()

        status = advance.update()

        self.assertIs(status, py_trees.common.Status.RUNNING)
        self.assertGreater(mav.gotos[-1][0], 0.0, "no through-gate leg was sent")
        self.assertLess(mav.gotos[-1][2], 2.805,
                        "the through-gate waypoint is inside the board span")


# --------------------------------------------------------------------------- #
class ScanStartQRTests(unittest.TestCase):

    def setUp(self):
        self.mav = FakeMav()
        # The stage now holds over the marker for a few seconds after reading
        # it, so the decode is watchable. These tests care about the decode
        # gate, not the pause, so the pause is driven from an injected clock
        # rather than waited out.
        self.clock = _FakeClock()
        self.leaf = ScanStartQR(self.mav, (0.0, 0.0, 5.0),
                                timeout_ticks=10, confirm_frames=3,
                                hover_s=0.0, clock=self.clock)

    def test_timeout_FAILS_it_does_not_succeed(self):
        """The headline defect: no target must mean no mission."""
        status = run(self.leaf, 15)
        self.assertEqual(status, py_trees.common.Status.FAILURE)
        self.assertIsNone(self.mav.target_set)
        self.assertIn("start QR not decoded", self.mav.abort_reason)

    def test_empty_decode_never_sets_a_target(self):
        self.mav.qr_decoded = ""
        self.mav.qr_streak = 99
        self.assertEqual(run(self.leaf, 15), py_trees.common.Status.FAILURE)
        self.assertIsNone(self.mav.target_set)

    def test_single_frame_decode_is_not_enough(self):
        self.mav.qr_decoded = "AEROTHON2026:M2:TARGET_C"
        self.mav.qr_streak = 1
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.RUNNING)
        self.assertIsNone(self.mav.target_set)

    def test_confirmed_decode_succeeds_and_latches_target(self):
        self.mav.qr_decoded = "AEROTHON2026:M2:TARGET_C"
        self.mav.qr_streak = 3
        self.assertEqual(run(self.leaf, 3), py_trees.common.Status.SUCCESS)
        self.assertEqual(self.mav.target_set, "AEROTHON2026:M2:TARGET_C")

    def test_operator_override_is_honoured_and_visible(self):
        """goal.md Q19 — allowed, but explicit, never a silent empty string."""
        self.mav.target_override = "AEROTHON2026:M2:TARGET_E"
        self.assertEqual(run(self.leaf, 3), py_trees.common.Status.SUCCESS)
        self.assertEqual(self.mav.target_set, "AEROTHON2026:M2:TARGET_E")
        self.assertIn("OPERATOR", self.leaf.feedback_message)


# --------------------------------------------------------------------------- #
class CorridorAltitudeTests(unittest.TestCase):

    def test_return_does_not_accept_the_outbound_exit_while_navigator_is_idle(self):
        from mission_bt.mav_commander import Mav
        mav = FakeMav()
        mav._alt = 3.0
        mav._pos = (20.0, 0.0, 3.0)
        mav.avoid_detail = {"state": "OBSERVING", "corridor_exited": True}
        mav.corridor_exited = lambda: Mav.corridor_exited(mav)
        stage = Corridor("ReturnCorridor", mav, forward=False, alt=3.0)
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING)
        mav.avoid_detail = {"state": "CRUISE", "corridor_exited": False}
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING)
        mav.avoid_detail = {"state": "CRUISE", "corridor_exited": True}
        self.assertIs(stage.update(), py_trees.common.Status.SUCCESS)

    def setUp(self):
        self.mav = FakeMav()
        # exit_x is gone: the corridor ends when perception says both walls
        # fell away, not at an asserted x (audit A6, A10).
        self.leaf = Corridor("Corridor", self.mav,
                             forward=True, alt=3.0, alt_band=1.5)

    def test_dragging_along_the_ground_FAILS(self):
        """The exact live failure: horizontal progress at z = 0.117 m."""
        self.mav._alt = 0.117
        self.mav._pos = (9.28, 1.52, 0.117)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("outside", self.mav.abort_reason)
        self.assertFalse(self.mav.avoidance,
                         "avoidance must be disabled when the stage fails")

    def test_in_band_keeps_running(self):
        self.mav._alt = 3.1
        self.mav._pos = (9.0, 0.0, 3.1)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.RUNNING)
        self.assertTrue(self.mav.avoidance)

    def test_corridor_opening_out_succeeds(self):
        """Exit is DETECTED by the navigator, not read off an x threshold."""
        self.mav._alt = 3.0
        self.mav._pos = (16.0, 0.0, 3.0)
        self.mav.exited = True
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_corridor_open_while_on_the_ground_still_FAILS(self):
        """Altitude is checked before the exit condition is even considered."""
        self.mav._alt = 0.1
        self.mav._pos = (16.0, 0.0, 0.1)
        self.mav.exited = True
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)

    def test_navigator_STUCK_fails_the_stage(self):
        """Three live runs hovered against an obstacle indefinitely."""
        self.mav._alt = 3.0
        self.mav._pos = (4.86, 0.0, 3.0)
        self.mav.stuck = True
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("STUCK", self.mav.abort_reason)
        self.assertFalse(self.mav.avoidance)


# --------------------------------------------------------------------------- #
class AirborneFloorTests(unittest.TestCase):

    def test_takeoff_arms_the_floor(self):
        mav = FakeMav()
        mav._alt = 0.0
        leaf = Takeoff(mav, 5.0)
        leaf.tick_once()
        self.assertIsNone(mav.airborne_floor)
        mav._alt = 4.8
        leaf.tick_once()
        self.assertEqual(leaf.status, py_trees.common.Status.SUCCESS)
        self.assertIsNotNone(mav.airborne_floor)
        self.assertGreater(mav.airborne_floor, 0.0)
        self.assertLess(mav.airborne_floor, 5.0)


# --------------------------------------------------------------------------- #
class _StubRoot:
    def __init__(self, status):
        self.status = status
        self.stopped_with = None

    def stop(self, status):
        self.stopped_with = status


class MissionFailureLatchTests(unittest.TestCase):

    def test_failure_latches_and_parks(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.FAILURE)
        mav.abort_reason = "search exhausted, no matching QR"

        reason = latch_mission_failure(root, mav)
        self.assertEqual(reason, "search exhausted, no matching QR")
        self.assertTrue(mav.mission_failed)
        self.assertFalse(mav.mission_started,
                         "a failed mission must not remain started")
        self.assertEqual(root.stopped_with, py_trees.common.Status.INVALID)
        self.assertEqual(mav.results[0][0], "FAILED")
        self.assertFalse(mav.avoidance)

    def test_latch_fires_once(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.FAILURE)
        latch_mission_failure(root, mav)
        self.assertIsNone(latch_mission_failure(root, mav),
                          "failure latched twice")
        self.assertEqual(len(mav.results), 1)

    def test_running_mission_is_untouched(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.RUNNING)
        self.assertIsNone(latch_mission_failure(root, mav))
        self.assertTrue(mav.mission_started)
        self.assertEqual(mav.results, [])

    def test_idle_tree_failure_is_not_a_mission_failure(self):
        """Before START the root legitimately fails; that is not an outcome."""
        mav = FakeMav()
        mav.mission_started = False
        root = _StubRoot(py_trees.common.Status.FAILURE)
        self.assertIsNone(latch_mission_failure(root, mav))
        self.assertEqual(mav.results, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)


# --------------------------------------------------------------------------- #
class CorridorExitIsRecordedTests(unittest.TestCase):
    """The corridor mouth is the anchor for everything downstream.

    zone_entry, zone_bounds and corridor_return_entry were asserted arena
    coordinates (audit A7, A8, A9). They are replaced by a point the aircraft
    measures for itself — but only if it is actually recorded when the corridor
    opens out, which is what this asserts.
    """

    def setUp(self):
        self.mav = FakeMav()
        self.leaf = Corridor("Corridor", self.mav, forward=True, alt=3.0)

    def test_exit_pose_is_recorded_on_success(self):
        self.mav._alt = 3.0
        self.mav._pos = (14.2, -0.7, 3.0)
        self.mav._yaw = 0.1
        self.mav.exited = True
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)
        self.assertIsNotNone(self.mav.corridor_exit_pose,
                             "corridor exit was not recorded; the return leg "
                             "and the zone bound have nothing to anchor on")
        x, y, _, yaw = self.mav.corridor_exit_pose
        self.assertAlmostEqual(x, 14.2)
        self.assertAlmostEqual(y, -0.7)
        self.assertAlmostEqual(yaw, 0.1)

    def test_nothing_is_recorded_while_still_inside(self):
        self.mav._alt = 3.0
        self.mav._pos = (5.0, 0.0, 3.0)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.RUNNING)
        self.assertIsNone(self.mav.corridor_exit_pose)

    def test_the_return_trip_does_not_overwrite_the_mouth(self):
        """The outbound exit IS the mouth. Recording the return exit over it
        would send the aircraft back into the zone it just left."""
        self.mav._alt = 3.0
        self.mav._pos = (14.2, -0.7, 3.0)
        self.mav.exited = True
        self.leaf.tick_once()
        first = self.mav.corridor_exit_pose

        self.mav._pos = (1.0, 0.0, 3.0)
        self.mav.record_corridor_exit()
        self.assertEqual(self.mav.corridor_exit_pose, first)


# --------------------------------------------------------------------------- #
class ReturnToCorridorMouthTests(unittest.TestCase):
    """corridor_return_entry = (15.0, 0.0, 3.0, pi) is replaced by memory."""

    def setUp(self):
        from mission_bt.mission_tree import ReturnToCorridorMouth
        self.mav = FakeMav()
        self.leaf = ReturnToCorridorMouth(self.mav, alt=3.0)

    def test_flies_to_the_recorded_mouth_not_a_constant(self):
        """Still derived from MEMORY of the exit, not from a constant.

        The target is now offset by a derived standoff (see
        ReturnStandoffTests): stopping exactly on the mouth put the aircraft
        on top of the return banner, which is why seed 1001 swept 180 degrees
        without seeing it. What this guards is unchanged -- the position comes
        from where the aircraft actually exited (then moved across to the
        return lane; see ReturnLaneStandoffTests).
        """
        self.mav.corridor_exit_pose = (22.5, -4.0, 3.0, 0.0)
        self.mav._pos = (30.0, 0.0, 3.0)
        self.leaf.tick_once()
        x, y, z, yaw = self.mav.gotos[-1]
        back = self.leaf.standoff()
        self.assertAlmostEqual(x, 22.5 + back, places=3)
        self.assertAlmostEqual(y, -4.0 + self.leaf.lane_offset_m, places=3)
        self.assertNotAlmostEqual(x, 15.0, msg="flew to the old constant")

    def _near_the_mouth(self):
        """Within the turn-in distance, where the final heading applies.
        Further out the transit is crabbed (see crab_yaw)."""
        rx, ry, tyaw = self.leaf.return_mouth(self.mav.corridor_exit_pose)
        back = self.leaf.standoff()
        self.mav._pos = (rx - back * math.cos(tyaw) + 1.0,
                         ry - back * math.sin(tyaw), 10.0)

    def test_heading_is_the_exit_heading_reversed(self):
        self.mav.corridor_exit_pose = (22.5, -4.0, 3.0, 0.0)
        self._near_the_mouth()
        self.leaf.tick_once()
        _, _, _, yaw = self.mav.gotos[-1]
        self.assertAlmostEqual(abs(yaw), math.pi, places=6)

    def test_the_transit_is_crabbed_until_the_turn_in(self):
        """Far from the mouth the nose is perpendicular to travel, so the
        camera's long axis looks where the aircraft is going."""
        from mission_bt.mission_tree import crab_yaw
        self.mav.corridor_exit_pose = (22.5, -4.0, 3.0, 0.0)
        self.mav._pos = (45.0, -4.0, 10.0)
        yaw = crab_yaw(self.mav, (27.0, -4.0), math.pi)
        self.assertAlmostEqual(abs(math.cos(yaw)), 0.0, places=6)

    def test_reverse_heading_is_wrapped_not_accumulated(self):
        """Exiting at +3.0 rad must come back at -0.14, not 6.14."""
        self.mav.corridor_exit_pose = (10.0, 0.0, 3.0, 3.0)
        self._near_the_mouth()
        self.leaf.tick_once()
        _, _, _, yaw = self.mav.gotos[-1]
        self.assertLessEqual(abs(yaw), math.pi + 1e-9)
        self.assertAlmostEqual(yaw, 3.0 - math.pi, places=6)

    def test_arriving_succeeds(self):
        """Arrival is at the STANDOFF point, not on the mouth itself."""
        self.mav.corridor_exit_pose = (22.5, -4.0, 3.0, 0.0)
        back = self.leaf.standoff()
        self.mav._pos = (22.5 + back, -4.0 + self.leaf.lane_offset_m, 3.0)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_sitting_ON_the_mouth_is_NOT_arrival(self):
        """The old behaviour, now explicitly wrong: on the mouth the return
        banner is beneath the aircraft and out of frame."""
        self.mav.corridor_exit_pose = (22.5, -4.0, 3.0, 0.0)
        self.mav._pos = (22.5, -4.0, 3.0)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.RUNNING)

    def test_no_recorded_exit_falls_back_to_home(self):
        self.mav.corridor_exit_pose = None
        self.leaf.tick_once()
        x, y, _, _ = self.mav.gotos[-1]
        self.assertEqual((x, y), (0.0, 0.0))


# --------------------------------------------------------------------------- #
class DescendToDecodeTests(unittest.TestCase):
    """Sweep high to FIND, descend to READ.

    search_planner computed decode_alt_m and the mission ignored it: it swept
    and decoded at the same altitude, which only works because the simulated
    pad is 2.2 m across. For a realistic marker the sweep altitude is well
    above the decode envelope measured in Phase 1.
    """

    def setUp(self):
        from mission_bt.mission_tree import DescendToDecode
        self.DescendToDecode = DescendToDecode
        self.mav = FakeMav()
        self.mav._pos = (20.0, 3.0, 12.0)
        self.leaf = DescendToDecode(self.mav, decode_alt=3.2, floor_alt=2.0,
                                    timeout_ticks=20)

    def test_commands_the_derived_decode_altitude(self):
        self.leaf.tick_once()
        z = self.mav.gotos[-1][2]
        self.assertAlmostEqual(z, 3.2)

    def test_a_callable_decode_altitude_is_resolved_at_run_time(self):
        """The plan does not exist when the tree is built."""
        leaf = self.DescendToDecode(self.mav, decode_alt=lambda: 4.4,
                                    floor_alt=2.0)
        leaf.tick_once()
        z = self.mav.gotos[-1][2]
        self.assertAlmostEqual(z, 4.4)

    def test_never_descends_below_the_floor(self):
        leaf = self.DescendToDecode(self.mav, decode_alt=0.3, floor_alt=2.0)
        leaf.tick_once()
        z = self.mav.gotos[-1][2]
        self.assertGreaterEqual(z, 2.0,
                                "a bad decode altitude flew it into the ground")

    def test_a_decode_ends_the_descent_immediately(self):
        self.mav.qr_matched = True
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_no_decode_by_the_floor_FAILS_closed(self):
        self.mav.qr_matched = False
        for _ in range(25):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(self.leaf.status, py_trees.common.Status.FAILURE)
        self.assertIn("decode", self.mav.abort_reason.lower())

    def test_holds_station_over_the_candidate_while_descending(self):
        """Drifting off the pad on the way down loses the thing being read."""
        self.mav.qr_offset_xy = (19.4, 3.6)
        self.leaf.tick_once()
        x, y = self.mav.gotos[-1][:2]
        self.assertAlmostEqual(x, 19.4)
        self.assertAlmostEqual(y, 3.6)


# --------------------------------------------------------------------------- #
class ClimbInPlaceTests(unittest.TestCase):
    """The corridor altitude must be commanded, not inherited.

    It used to be reached as a side effect of flying to the hardcoded
    corridor_entry waypoint. Deleting that waypoint (audit A5) deleted the
    descent with it and the next live run failed the corridor altitude band at
    4.92 m -- a hardcode removal that quietly removed a REQUIREMENT.
    """

    def setUp(self):
        from mission_bt.mission_tree import ClimbInPlace
        self.ClimbInPlace = ClimbInPlace
        self.mav = FakeMav()

    def test_descends_to_the_target_without_translating(self):
        self.mav._pos = (7.3, -1.2, 5.0)
        leaf = self.ClimbInPlace("Descend", self.mav, 3.0)
        leaf.tick_once()
        x, y, z, _ = self.mav.gotos[-1]
        self.assertAlmostEqual(x, 7.3)
        self.assertAlmostEqual(y, -1.2)
        self.assertAlmostEqual(z, 3.0)
        self.assertEqual(leaf.status, py_trees.common.Status.RUNNING)

    def test_holds_the_current_yaw(self):
        """goto()'s yaw defaults to 0.0; using it would spin the aircraft off
        the banner heading the previous stage just acquired."""
        self.mav._pos = (7.3, -1.2, 5.0)
        self.mav._yaw = 1.31
        leaf = self.ClimbInPlace("Descend", self.mav, 3.0)
        leaf.tick_once()
        _, _, _, yaw = self.mav.gotos[-1]
        self.assertAlmostEqual(yaw, 1.31,
                               msg="alt change threw away the banner heading")

    def test_succeeds_once_within_tolerance(self):
        self.mav._pos = (7.3, -1.2, 3.1)
        leaf = self.ClimbInPlace("Descend", self.mav, 3.0)
        leaf.tick_once()
        self.assertEqual(leaf.status, py_trees.common.Status.SUCCESS)

    def test_a_derived_altitude_is_resolved_at_run_time(self):
        self.mav._pos = (0.0, 0.0, 5.0)
        alt = [9.0]
        leaf = self.ClimbInPlace("Climb", self.mav, lambda: alt[0])
        leaf.tick_once()
        self.assertAlmostEqual(self.mav.gotos[-1][2], 9.0)
        alt[0] = 4.0
        leaf.tick_once()
        self.assertAlmostEqual(self.mav.gotos[-1][2], 4.0)


# --------------------------------------------------------------------------- #
class MissionSuccessLatchTests(unittest.TestCase):
    """A COMPLETED mission is terminal too.

    THE LIVE FAILURE: "Mission result: COMPLETED (landed and disarmed)" was
    followed 0.4 s later by "Mission state -> ARMING" and a second takeoff.
    latch_mission_failure() parked FAILED missions and SUCCESS fell straight
    through: the Selector returns SUCCESS, py_trees re-initialises the memory
    Sequence on the next tick, and the aircraft flies the mission again by
    itself. An aircraft that re-arms after delivering is a safety problem.
    """

    def test_success_latches_and_parks(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.SUCCESS)
        reason = latch_mission_success(root, mav)
        self.assertIsNotNone(reason)
        self.assertTrue(mav.mission_complete)
        self.assertFalse(mav.mission_started,
                         "a completed mission must not remain started; it "
                         "re-armed and took off again")
        self.assertEqual(root.stopped_with, py_trees.common.Status.INVALID)
        self.assertEqual(mav.results[0][0], "COMPLETED")
        self.assertFalse(mav.avoidance)

    def test_latch_fires_once(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.SUCCESS)
        latch_mission_success(root, mav)
        self.assertIsNone(latch_mission_success(root, mav))
        self.assertEqual(len(mav.results), 1)

    def test_running_mission_is_untouched(self):
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.RUNNING)
        self.assertIsNone(latch_mission_success(root, mav))
        self.assertTrue(mav.mission_started)

    def test_idle_tree_success_is_not_an_outcome(self):
        mav = FakeMav()
        mav.mission_started = False
        root = _StubRoot(py_trees.common.Status.SUCCESS)
        self.assertIsNone(latch_mission_success(root, mav))
        self.assertEqual(mav.results, [])

    def test_a_new_start_clears_the_latch(self):
        """Terminal means "until told otherwise", not "forever"."""
        mav = FakeMav()
        root = _StubRoot(py_trees.common.Status.SUCCESS)
        latch_mission_success(root, mav)
        mav.mission_complete = False          # what a START does
        mav.mission_started = True
        self.assertIsNotNone(latch_mission_success(root, mav))


# --------------------------------------------------------------------------- #
class PrecisionDescentTests(unittest.TestCase):
    """Phase 9 — landing on the pad, and knowing whether it did.

    Landing was `mav.land()`: hand ArduPilot LAND mode and hope. LAND descends
    wherever the aircraft happens to be, GPS puts that within a few metres of
    home, and the pad is not a few metres wide. Accuracy was never measured
    because nothing in the stack knew where the pad was during the descent.
    """

    def setUp(self):
        from mission_bt.mission_tree import PrecisionDescent
        self.PrecisionDescent = PrecisionDescent
        self.mav = FakeMav()
        self.mav._pos = (0.2, -0.1, 5.0)
        self.mav.qr_off = (0.0, 0.0)
        self.leaf = PrecisionDescent(self.mav, start_alt=5.0, commit_alt=1.5,
                                     step_m=0.4, tol=0.10, lost_grace=3,
                                     timeout_ticks=100)

    def test_a_centred_pad_descends(self):
        self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertLess(z, 5.0)

    def test_an_OFF_CENTRE_pad_does_NOT_descend(self):
        """Coming down off-centre moves the error closer to the ground, where
        there is less room to correct it."""
        self.mav.qr_off = (0.5, 0.3)
        self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertAlmostEqual(z, 5.0, places=6)

    def test_it_corrects_toward_the_pad(self):
        """Same image->local mapping CenterOnQR asserts: image +y is DOWN, so
        a pad below frame centre is BEHIND and needs a backward correction."""
        self.mav.qr_off = (0.0, 0.5)
        self.leaf.tick_once()
        x, _, _, _ = self.mav.gotos[-1]
        self.assertLess(x, 0.2, "pad below frame centre must move us back")

    def test_pad_right_of_centre_moves_right(self):
        self.mav.qr_off = (0.5, 0.0)
        self.leaf.tick_once()
        _, y, _, _ = self.mav.gotos[-1]
        self.assertLess(y, -0.1, "pad right of frame centre must move us right")

    def test_losing_lock_HOLDS_rather_than_descending_blind(self):
        """Descending blind turns an ordinary landing into one that believes
        it was precise."""
        self.mav.qr_off = None
        self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertAlmostEqual(z, 5.0, places=6)

    def test_a_sustained_loss_climbs_to_RE_ACQUIRE(self):
        self.mav.qr_off = None
        for _ in range(6):
            self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertGreater(z, 5.0 - 1e-9)
        self.assertGreaterEqual(self.leaf.reacquisitions, 1)

    def test_re_acquisition_is_COUNTED_not_hidden(self):
        """A landing that needed four re-acquisitions is a different result
        from one that needed none, and the report should say so."""
        self.mav.qr_off = None
        for _ in range(6):
            self.leaf.tick_once()
        self.mav.qr_off = (0.0, 0.0)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.reacquisitions, 1)

    def test_regaining_lock_resumes_the_descent(self):
        self.mav.qr_off = None
        for _ in range(6):
            self.leaf.tick_once()
        self.mav.qr_off = (0.0, 0.0)
        before = self.mav._pos[2]
        self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertLess(z, before)

    def test_below_the_commit_altitude_it_finishes(self):
        """The pad leaves the field of view close in; climbing back to look
        for it would be worse than finishing."""
        self.mav._pos = (0.2, -0.1, 1.2)
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_a_lost_pad_below_commit_still_finishes(self):
        self.mav._pos = (0.2, -0.1, 1.0)
        self.mav.qr_off = None
        self.leaf.tick_once()
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)

    def test_never_getting_a_lock_DEGRADES_rather_than_failing_the_mission(self):
        """A live run hung at 2.57 m for 601 ticks and reported the whole
        mission FAILED -- after the payload had already been delivered. The
        only thing at stake at this point is whether touchdown is ON the pad
        or merely near it; "landed, precision not achieved" is the truthful
        answer, not "mission failed"."""
        self.mav.qr_off = (0.9, 0.9)           # never centres
        for _ in range(120):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertEqual(self.leaf.status, py_trees.common.Status.SUCCESS)
        self.assertIn("DEGRADED", self.mav.landing_precision)

    def test_a_degraded_landing_is_REPORTED_not_hidden(self):
        self.mav.qr_off = None
        for _ in range(120):
            self.leaf.tick_once()
            if self.leaf.status != py_trees.common.Status.RUNNING:
                break
        self.assertNotEqual(self.mav.landing_precision, "UNKNOWN")
        self.assertIn("DEGRADED", self.mav.landing_precision)

    def test_a_successful_lock_is_reported_as_PRECISE(self):
        self.mav._pos = (0.2, -0.1, 1.2)
        self.leaf.tick_once()
        self.assertIn("PRECISE", self.mav.landing_precision)

    def test_it_never_commands_below_the_commit_altitude(self):
        self.mav._pos = (0.0, 0.0, 1.6)
        self.leaf.tick_once()
        _, _, z, _ = self.mav.gotos[-1]
        self.assertGreaterEqual(z, 1.5 - 1e-9)


# --------------------------------------------------------------------------- #
class RulebookAltitudeTests(unittest.TestCase):
    """The rulebook prescribes an ORDER and a set of altitudes, not just a
    set of tasks (AeroTHON 2026 Track 1, Mission 2 - SkyScan).

        take-off -> 5 m       scan the start QR
        5 m                   identify the banner and align, BEFORE descending
        5 m -> 3 m            corridor navigation altitude (10 ft)
        3 m -> 10 m           ascend on leaving the corridor, identify the pad
        10 m -> 5 m           descend, lower and release the payload
        5 m -> 10 m           ascend for the return lap
        10 m                  detect the banner again, align, return through
                              the corridor, land at the take-off point
    """

    def _stage_names(self):
        from mission_bt.mission_tree import build_root
        from unittest.mock import MagicMock
        params = {
            'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
            'image_width_px': 1280, 'camera_hfov': 1.0472,
            'target_marker_m': 2.2, 'qr_modules': 33,
            'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
            'zone_margin': 1.0, 'corridor_alt': 3.0,
            'waypoint_tol': 0.8, 'drop_tol': 0.5, 'scan_floor_alt': 2.0,
            'land_commit_alt': 1.5, 'banner_sweep_limit': math.pi,
        }
        root = build_root(FakeMav(), MagicMock(), params)
        mission = [c for c in root.children if c.name == "Mission"][0]
        return [c.name for c in mission.children]

    def test_the_banner_is_identified_BEFORE_descending_to_the_corridor(self):
        """RULEBOOK: "identify the ... Green Banner ... and align itself
        before descending to the corridor navigation altitude". Descending
        first also works, but it inverts the required order."""
        names = self._stage_names()
        align = names.index("AlignToBanner")
        descend = names.index("DescendToCorridorAlt")
        self.assertLess(align, descend,
                        "descended to corridor altitude before identifying "
                        "the banner")

    def test_the_return_lap_re_detects_the_banner(self):
        """RULEBOOK, "Corridor Entry Detection Return Lap" -- a separately
        scored task. The return leg used to fly back on the recorded corridor
        pose without ever looking for the gate again."""
        names = self._stage_names()
        self.assertGreaterEqual(names.count("AlignToBanner"), 2,
                                "the banner is never re-detected for the "
                                "return lap")

    def test_both_gate_legs_duck_under_the_board_before_advancing(self):
        """Square-on needs the lidar to intersect the board, but transit at
        that same height hits it. Both legs must measure the lower edge and
        descend before committing the through-gate waypoint."""
        names = self._stage_names()
        aligns = [i for i, name in enumerate(names) if name == "AlignToBanner"]
        ducks = [i for i, name in enumerate(names) if name == "DuckUnderBoard"]
        advances = [i for i, name in enumerate(names) if name == "GateAdvance"]

        self.assertEqual(len(aligns), 2)
        self.assertEqual(len(ducks), 2, "one or both gate legs skip the duck")
        self.assertEqual(len(advances), 2)
        for align, duck, advance in zip(aligns, ducks, advances):
            self.assertLess(align, duck)
            self.assertLess(duck, advance)

    def test_the_return_lap_climbs_before_transiting(self):
        """RULEBOOK: "ascend to 10-meter altitude, navigate back through the
        corridor". Crossing the delivery zone at corridor altitude was both a
        deviation and needlessly low over the pads."""
        names = self._stage_names()
        self.assertIn("ClimbForReturn", names)
        self.assertLess(names.index("ClimbForReturn"),
                        names.index("ReturnToCorridorMouth"))

    def test_the_stages_appear_in_rulebook_order(self):
        names = self._stage_names()
        def at(n):
            return names.index(n)
        # Organiser inputs are pre-flight: no boundary / no enforced fence,
        # no arming (rulebook Mission 2 Operation, "Geo-fencing").
        self.assertLess(at("RequireDeliveryZone"), at("SetModeArm"))
        self.assertLess(at("UploadArenaFence"), at("SetModeArm"))
        self.assertLess(at("Takeoff"), at("ScanStartQR"))
        self.assertLess(at("ScanStartQR"), at("AlignToBanner"))
        self.assertLess(at("Corridor"), at("EnterDeliveryZone"))
        self.assertLess(at("EnterDeliveryZone"), at("LawnmowerSearch"))
        self.assertLess(at("LawnmowerSearch"), at("CenterOnTarget"))
        self.assertLess(at("CenterOnTarget"), at("WinchDrop"))
        self.assertNotIn("ObserveZone", names,
                         "the search area must be the supplied boundary")
        # Every leg over the delivery field is flown with the camera at
        # nadir, where the red-zone detector can see (seed 1004 crossed the
        # field on the return transit with the camera at the banner pose).
        self.assertLess(at("CameraNadirForSearch"), at("EnterDeliveryZone"))
        self.assertLess(at("CameraNadirForReturnTransit"),
                        at("ReturnToCorridorMouth"))
        self.assertLess(at("ReturnToCorridorMouth"), at("CameraBannerReturn"))
        self.assertLess(at("LawnmowerSearch"), at("WinchDrop"))
        self.assertLess(at("WinchDrop"), at("ReturnCorridor"))
        self.assertLess(at("ReturnCorridor"), at("Land"))

    def test_the_SWEEP_never_goes_above_the_10_m_ceiling(self):
        """Live run 12 flew the sweep at 13.9 m.

        Every stage was in the right order and every leaf-level altitude test
        passed, because the violation was not in the tree at all: the search
        planner raised the ceiling to the decode altitude of the 2.2 m pad.
        The order tests above cannot see that, so this asks the object that
        actually decides -- with the mission's own declared parameters.
        """
        from mission_bt.mission_tree import build_root
        from unittest.mock import MagicMock
        params = {
            'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
            'image_width_px': 1280, 'camera_hfov': 1.0472,
            'target_marker_m': 2.2, 'qr_modules': 33,
            'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
            'zone_margin': 1.0, 'corridor_alt': 3.0,
            'waypoint_tol': 0.8, 'drop_tol': 0.5, 'scan_floor_alt': 2.0,
            'land_commit_alt': 1.5, 'banner_sweep_limit': math.pi,
        }
        root = build_root(FakeMav(), MagicMock(), params)
        mission = [c for c in root.children if c.name == "Mission"][0]
        search = [c for c in mission.children
                  if c.name == "LawnmowerSearch"][0]
        # A zone of the size the corridor exit actually observes.
        search._replan((16.5, 27.9, -8.1, 6.9))
        self.assertLessEqual(search.alt, params['search_alt'] + 1e-9,
                             f"sweep planned at {search.alt:.1f} m, above the "
                             f"{params['search_alt']} m rulebook ceiling")
        self.assertLessEqual(search.plan["waypoints"][0][2],
                             params['search_alt'] + 1e-9)


class DecodeAltitudeCeilingTests(unittest.TestCase):
    """Descend to decode -- never climb to it."""

    def setUp(self):
        from mission_bt.mission_tree import DescendToDecode
        self.DescendToDecode = DescendToDecode
        self.mav = FakeMav()

    def test_a_decode_altitude_ABOVE_the_aircraft_does_not_command_a_climb(self):
        """For the 2.2 m simulated pad the decode envelope computes to 13.9 m.
        `max(want, floor)` commanded a climb to 13.9 m -- above the rulebook's
        10 m identification altitude -- in order to "descend" to read a marker
        the aircraft could already read."""
        self.mav._pos = (20.0, 3.0, 10.0)
        leaf = self.DescendToDecode(self.mav, decode_alt=13.9, floor_alt=2.0)
        leaf.tick_once()
        z = self.mav.gotos[-1][2]
        self.assertLessEqual(z, 10.0 + 1e-9,
                             "commanded a climb above the sweep altitude")

    def test_a_decode_altitude_below_still_descends(self):
        self.mav._pos = (20.0, 3.0, 10.0)
        leaf = self.DescendToDecode(self.mav, decode_alt=4.0, floor_alt=2.0)
        leaf.tick_once()
        z = self.mav.gotos[-1][2]
        self.assertAlmostEqual(z, 4.0)


class FrontierSearchTests(unittest.TestCase):
    """The search must be able to reach a target the lidar never saw.

    THE BUG THIS IS FOR

        ObserveZone bounds the delivery zone with the lidar, whose range is
        12 m. The real zone is 40 m deep. So the "observed zone" was a window
        covering x 16.5..27.9 of a real 12..52, and LawnmowerSearch swept it
        once and reported "swept all lanes without matching the target".

        Target C sits at (23, 1) -- inside. Targets B (47,10), D (33,-10) and
        E (45,-6) do not, and neither do any of the three red zones. Every
        live run had been launched with --start-target c, so a search that
        could only ever find one of the five pads looked like a working
        search.
    """

    def setUp(self):
        from mission_bt.mission_tree import LawnmowerSearch
        self.LawnmowerSearch = LawnmowerSearch
        self.mav = FakeMav()
        self.mav.exclusions = []
        self.mav._pos = (17.0, 0.0, 10.0)
        self.mav.corridor_exit_pose = (17.0, 0.0, 10.0, 0.0)
        # 12 m of open ground ahead: the lidar's range, not the zone's size.
        self.mav.avoid_detail = {"open_depth_m": 12.0, "open_width_m": 16.6}

    def _search(self, budget, **kw):
        return self.LawnmowerSearch(
            self.mav, (16.5, 27.9, -8.1, 6.9), 10.0,
            exclusions=lambda: self.mav.exclusions,
            search_budget_m=budget, **kw)

    @staticmethod
    def _dist_to_leg(p, a, b):
        """Closest approach of point `p` to the segment a->b."""
        (px, py), (ax, ay), (bx, by) = p, a, b
        dx, dy = bx - ax, by - ay
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    def _fly(self, leaf, target_xy=None, swath_m=5.0, max_ticks=4000):
        """Tick, teleporting along each commanded leg.

        If `target_xy` is given the QR matches when the aircraft's FLIGHT PATH
        passes within half a camera swath of it -- not when a waypoint lands
        on it. Lanes only have waypoints at their ends, so checking waypoints
        would miss every pad in the middle of a lane, which is most of them.
        So "did the search find the pad" is decided by where the aircraft
        actually flew.
        """
        for _ in range(max_ticks):
            prev = self.mav._pos[:2]
            leaf.tick_once()
            if leaf.status != py_trees.common.Status.RUNNING:
                return leaf.status
            if self.mav.gotos:
                x, y, z = self.mav.gotos[-1][:3]
                self.mav._pos = (x, y, z)
                self.mav._alt = z
                if (target_xy is not None
                        and self._dist_to_leg(target_xy, prev, (x, y)) <= swath_m):
                    self.mav.qr_matched = True
        return leaf.status

    # ---- the regression ---- #
    def test_a_target_INSIDE_the_window_was_always_found(self):
        """Target C -- which is why the bug survived every live run."""
        leaf = self._search(budget=0.0)
        st = self._fly(leaf, target_xy=(23.0, 1.0))
        self.assertEqual(st, py_trees.common.Status.SUCCESS)
        self.assertEqual(leaf.expansions, 0)

    def test_a_target_BEYOND_the_window_is_now_found(self):
        """Target E at (45, -6), 17 m past the far edge of the window."""
        leaf = self._search(budget=45.0)
        st = self._fly(leaf, target_xy=(45.0, -6.0))
        self.assertEqual(st, py_trees.common.Status.SUCCESS,
                         self.mav.abort_reason)
        self.assertGreater(leaf.expansions, 0,
                           "reached the target without advancing the frontier")

    def test_run19_seed1001_reaches_target_A_inside_the_supplied_boundary(self):
        """Live run 19 swept four strips and missed target A at (50.68,
        -0.25). The mission's search area is now the SUPPLIED delivery-zone
        boundary, so the far pad is inside the plan from the start -- and
        the sweep begins at the boundary's bottom-left corner."""
        self.mav._pos = (20.6, -3.6, 10.0)
        self.mav.corridor_exit_pose = (20.6, -3.6, 3.0, 0.182)
        self.mav.observed_zone = (9.8, 22.8, -12.3, 3.7)   # ignored now
        boundary = (12.0, 52.0, -15.0, 15.0)
        self.mav.delivery_search_zone = lambda c: (boundary[0] + c,
                                                   boundary[1] - c,
                                                   boundary[2] + c,
                                                   boundary[3] - c)
        from mission_bt.mission_tree import build_root
        from unittest.mock import MagicMock
        params = {
            'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
            'image_width_px': 1280, 'camera_hfov': 1.0472,
            'target_marker_m': 2.2, 'qr_modules': 33,
            'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
            'zone_margin': 1.0, 'corridor_alt': 3.0,
            'waypoint_tol': 0.8, 'drop_tol': 0.5,
            'scan_floor_alt': 2.0, 'land_commit_alt': 1.5,
        }
        root = build_root(self.mav, MagicMock(), params)
        mission = next(c for c in root.children if c.name == "Mission")
        leaf = next(c for c in mission.children if c.name == "LawnmowerSearch")

        st = self._fly(leaf, target_xy=(50.68, -0.25))

        self.assertEqual(st, py_trees.common.Status.SUCCESS,
                         self.mav.abort_reason)
        self.assertEqual(leaf.expansions, 0, "searched outside the boundary")
        first = leaf.wps[0]
        self.assertLess(first[0], boundary[0] + 2.0,
                        f"first waypoint {first} is not at the west edge")
        self.assertLess(first[1], boundary[2] + 6.0,
                        f"first waypoint {first} is not at the south edge")
        for g in self.mav.gotos:
            x, y = g[0], g[1]
            self.assertTrue(boundary[0] <= x <= boundary[1]
                            and boundary[2] <= y <= boundary[3],
                            f"commanded ({x:.1f}, {y:.1f}) outside the boundary")

    def test_that_target_is_UNREACHABLE_without_a_budget(self):
        """The same flight with expansion disabled must fail -- otherwise the
        test above proves nothing about the mechanism."""
        leaf = self._search(budget=0.0)
        st = self._fly(leaf, target_xy=(45.0, -6.0))
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertIn("without matching the target", self.mav.abort_reason)

    def test_the_aircraft_actually_flies_over_the_far_target(self):
        """Not just 'the plan extends far enough' -- a commanded waypoint has
        to come within a swath of the pad."""
        leaf = self._search(budget=45.0)
        self._fly(leaf)
        reached = max(g[0] for g in self.mav.gotos)
        self.assertGreater(reached, 45.0,
                           f"furthest waypoint was only x={reached:.1f}")

    # ---- termination ---- #
    def test_expansion_does_NOT_depend_on_the_lidar_at_sweep_altitude(self):
        """The regression from live run 13.

        Gating the step on `open_extent()` looked principled and killed the
        search: the horizontal lidar sits on the airframe, and at 10 m over
        open ground it is above everything. /avoidance/detail reported
        open_depth_m = 0.32 with nothing at all in front of the aircraft, so
        every advance was refused as "boundary observed" and the mission
        failed with 45 m of budget unused.
        """
        leaf = self._search(budget=45.0)
        leaf.initialise()
        self.mav.avoid_detail = {"open_depth_m": 0.32, "open_width_m": 24.0}
        self.assertTrue(leaf._advance_frontier(),
                        "a meaningless lidar reading blocked the frontier")

    def test_the_step_is_the_depth_the_lidar_measured_where_it_COULD_see(self):
        """Still an observation -- a different opening gives a different step,
        so this is not the arena's size in disguise."""
        leaf = self._search(budget=45.0)
        leaf.initialise()
        self.assertAlmostEqual(leaf._window_depth, 27.9 - 16.5, places=6)
        leaf._advance_frontier()
        self.assertAlmostEqual(leaf._frontier[0], 27.9, places=6)

    def test_the_budget_bounds_the_search(self):
        leaf = self._search(budget=20.0)
        st = self._fly(leaf)
        self.assertEqual(st, py_trees.common.Status.FAILURE)
        self.assertLessEqual(leaf.expansions, 2)

    def test_the_failure_reason_reports_what_was_actually_swept(self):
        """"swept all lanes" said nothing about WHERE. The reason has to name
        the ground covered, or the next person debugs the same thing again."""
        leaf = self._search(budget=20.0)
        self._fly(leaf)
        self.assertIn("strip(s)", self.mav.abort_reason)
        self.assertRegex(self.mav.abort_reason, r"swept \[.*\]")

    def test_every_advance_is_logged(self):
        leaf = self._search(budget=45.0)
        self._fly(leaf)
        advances = [m for m, _ in self.mav.logs if "frontier advance" in m]
        self.assertEqual(len(advances), leaf.expansions)
        self.assertTrue(all("budget left" in m for m in advances))

    # ---- Phase 7: red zones only become visible DURING the sweep ---- #
    def test_a_red_zone_confirmed_mid_sweep_is_avoided(self):
        """Exclusions were sampled once, before the sweep, from the corridor
        exit -- where none of the red zones are visible. Anything the
        georeferencer confirmed later was recorded and then ignored."""
        leaf = self._search(budget=45.0)
        leaf.initialise()
        for _ in range(6):
            leaf.tick_once()
        self.mav.exclusions = [(20.0, 26.0, -6.0, 0.0)]
        for _ in range(3):
            leaf.tick_once()
        self.assertEqual(len(leaf.exclusions), 1,
                         "the confirmed red zone was never picked up")
        replans = [m for m, _ in self.mav.logs if "mid-sweep" in m]
        self.assertTrue(replans, "no re-plan logged for the new red zone")

    def test_lanes_do_not_cross_a_red_zone_confirmed_mid_sweep(self):
        from mission_bt.search_planner import plan_intersects_exclusions
        red = [(20.0, 26.0, -6.0, 0.0)]
        leaf = self._search(budget=0.0)
        leaf.initialise()
        for _ in range(6):
            leaf.tick_once()
        self.mav.exclusions = red
        for _ in range(3):
            leaf.tick_once()
        remaining = [(x, y, leaf.alt) for x, y in leaf.wps[leaf.i:]]
        self.assertFalse(
            plan_intersects_exclusions(remaining, 1.5, red),
            "a planned lane still crosses the confirmed red zone")

    def test_a_growing_exclusion_set_cannot_stall_the_sweep(self):
        """Live run 13 saw exclusions climb 0 -> 5 -> 26 -> 32 while sweeping,
        because the georeferencer confirms red ground cell by cell. A
        re-plan that restarted the strip, or stopped to re-plan every tick,
        would keep the aircraft re-flying lane 0 forever."""
        leaf = self._search(budget=0.0)
        leaf.initialise()
        n = [0]

        def growing():
            n[0] += 1
            return [(20.0 + 0.01 * i, 26.0, -6.0, 0.0) for i in range(n[0])]

        leaf._exclusions_src = growing
        st = self._fly(leaf, max_ticks=600)
        self.assertNotEqual(st, py_trees.common.Status.RUNNING,
                            "the sweep never terminated")

    def test_replan_resumes_ahead_not_at_the_lane_start(self):
        """Half-way along a lane, a re-plan carries on to the lane's end."""
        leaf = self._search(budget=0.0)
        leaf.initialise()
        (x0, y0), (x1, y1) = leaf.wps[0], leaf.wps[1]
        lane = leaf._lane_coord(leaf.wps[1])
        mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        self.assertEqual(leaf._resume_index(lane, mid), 1)
        # At the lane start it begins at the start.
        self.assertEqual(leaf._resume_index(lane, (x0, y0)), 0)
        # Past the lane's end it goes on to the next lane.
        self.assertEqual(leaf._resume_index(lane, (x1, y1)), 2)

    def test_red_lane_end_clips_the_lane_instead_of_dropping_it(self):
        """Arena 1001, batch E: after five re-plans the plan froze; two lanes
        whose far ends had since been confirmed red were skipped whole, and
        the target sat at their clear, near ends."""
        self._clip_not_drop(replan_min_ticks=0)

    def test_waypoint_turned_red_between_replans_is_replanned_not_skipped(self):
        """Arena 1003, batch E: a segment start confirmed red inside the
        re-plan spacing was skipped. With spacing that never lets the
        routine re-plan fire, the blocked waypoint alone must re-plan."""
        self._clip_not_drop(replan_min_ticks=10 ** 6, min_replans=1)

    def _clip_not_drop(self, replan_min_ticks, min_replans=6):
        # An arena-sized field: enough lanes to outlast the old cap of five.
        leaf = self.LawnmowerSearch(
            self.mav, (16.5, 54.5, -20.0, 36.0), 10.0,
            exclusions=lambda: self.mav.exclusions, search_budget_m=0.0)
        leaf.initialise()
        leaf.replan_min_ticks = replan_min_ticks
        n_lanes = leaf.plan["n_lanes"]
        zx0, zx1, zy0, zy1 = leaf._frontier
        # Red ground confirming cell by cell, well past five re-plans, that
        # ends up swallowing the far (east) end of every lane.
        along_lane = set()          # lanes a leg actually ran ALONG
        for tick in range(4000):
            # The one source both the planner and the router read, as live.
            self.mav.exclusions = [
                (zx1 - 3.0 - 0.1 * i, zx1 + 1.0, zy0 - 1.0, zy1 + 1.0)
                for i in range(min(tick + 1, 40))]
            prev = self.mav._pos[:2]
            leaf.tick_once()
            if leaf.status != py_trees.common.Status.RUNNING:
                break
            if self.mav.gotos:
                x, y, z = self.mav.gotos[-1][:3]
                self.mav._pos = (x, y, z)
                self.mav._alt = z
                if abs(y - prev[1]) < 0.3 and abs(x - prev[0]) >= 2.0:
                    along_lane.add(round(y, 1))
        replans = [m for m, _ in self.mav.logs if "mid-sweep" in m]
        self.assertGreaterEqual(len(replans), min_replans,
                                "never got past the old cap")
        self.assertEqual(leaf.skipped, 0,
                         "a lane was dropped instead of clipped")
        self.assertGreaterEqual(len(along_lane), n_lanes,
                                "a lane's clear part went unflown")


class PrecisionDescentFloorTests(unittest.TestCase):
    """The commit altitude must be reachable by the camera.

    Runs 12 and 14 both ended the same way: descend to ~3 m, lose the pad,
    climb to re-acquire, oscillate between 3.4 m and 4.1 m until the stage
    timed out and degraded. Nothing was wrong with the controller -- it was
    told to hold lock down to 1.5 m, and a 2.2 m marker does not fit in the
    frame below about 2.5 m.
    """

    def setUp(self):
        from mission_bt.mission_tree import PrecisionDescent
        self.PrecisionDescent = PrecisionDescent
        self.mav = FakeMav()

    def test_the_commit_altitude_is_RAISED_to_the_tracking_floor(self):
        leaf = self.PrecisionDescent(self.mav, start_alt=5.0, commit_alt=1.5,
                                     marker_m=2.2, hfov_rad=1.0472,
                                     image_w_px=1280, image_h_px=720)
        self.assertGreater(leaf.commit_alt, 1.5)
        self.assertAlmostEqual(leaf.commit_alt, leaf.track_floor, places=9)

    def test_a_small_marker_keeps_the_configured_floor(self):
        """The parameter is a floor, not a target: a marker that can be
        tracked lower must not force the aircraft to commit early."""
        leaf = self.PrecisionDescent(self.mav, start_alt=5.0, commit_alt=1.5,
                                     marker_m=0.3, hfov_rad=1.0472,
                                     image_w_px=1280, image_h_px=720)
        self.assertAlmostEqual(leaf.commit_alt, 1.5)

    def test_it_commits_instead_of_oscillating_at_the_floor(self):
        """The live signature: at 2.6 m with the pad in view it used to keep
        descending toward an unreachable 1.5 m."""
        leaf = self.PrecisionDescent(self.mav, start_alt=5.0, commit_alt=1.5,
                                     marker_m=2.2, hfov_rad=1.0472,
                                     image_w_px=1280, image_h_px=720)
        self.mav._pos = (0.0, 0.0, leaf.commit_alt - 0.05)
        self.mav.qr_off = (0.0, 0.0)
        leaf.tick_once()
        self.assertEqual(leaf.status, py_trees.common.Status.SUCCESS)
        self.assertIn("PRECISE", self.mav.landing_precision)

    def test_the_derived_floor_is_reported(self):
        leaf = self.PrecisionDescent(self.mav, marker_m=2.2, hfov_rad=1.0472,
                                     image_w_px=1280, image_h_px=720)
        leaf.initialise()
        msgs = [m for m, _ in self.mav.logs if "precision descent" in m]
        self.assertTrue(msgs)
        self.assertIn("leaves frame below", msgs[0])

    def test_no_marker_size_falls_back_to_the_parameter(self):
        leaf = self.PrecisionDescent(self.mav, commit_alt=1.5, marker_m=None)
        self.assertAlmostEqual(leaf.commit_alt, 1.5)


class AltitudeDivergenceTraceTests(unittest.TestCase):
    """The trace that has to name the stage bleeding altitude.

    Three randomised arenas fail below the corridor band and two facts refuse
    to reconcile: the corridor's altitude hold measurably works, and the
    approach commands position setpoints at a constant z. Rather than reason
    about which stage "should" be in control, MavCommander records where
    commanded and measured altitude actually diverge.
    """

    def setUp(self):
        from mission_bt.mav_commander import Mav
        self.Mav = Mav

    def make(self):
        c = self.Mav.__new__(self.Mav)
        c.active_stage = ""
        c.alt_error_worst = 0.0
        c.alt_error_worst_stage = ""
        c.alt_error_warn = 0.8
        c._alt_warned_stage = None
        c._alt_target = None
        c._alt_arrived = False
        c.logs = []
        c.log = lambda m, warn=False: c.logs.append(m)
        return c

    def arm(self, c, cmd_z, meas_z, stage):
        from geometry_msgs.msg import PoseStamped
        sp = PoseStamped()
        sp.pose.position.z = cmd_z
        c._sp = sp
        c.active_stage = stage
        c.alt = lambda: meas_z
        c.pos = lambda: (1.0, 2.0, meas_z)
        return c

    def test_a_matching_altitude_says_nothing(self):
        c = self.arm(self.make(), 3.0, 3.0, "Corridor")
        c._track_altitude_error()
        self.assertEqual(c.logs, [])

    def _settle(self, c, z, stage):
        """Fly the aircraft to its commanded altitude first — a shortfall only
        counts once the target has actually been reached."""
        self.arm(c, z, z, stage)
        c._track_altitude_error()

    def test_a_divergence_is_reported_WITH_the_stage(self):
        c = self.make()
        self._settle(c, 3.0, "ApproachBanner")
        self.arm(c, 3.0, 1.4, "ApproachBanner")
        c._track_altitude_error()
        self.assertEqual(len(c.logs), 1)
        self.assertIn("ApproachBanner", c.logs[0])
        self.assertIn("1.60", c.logs[0])

    def test_the_worst_divergence_and_its_stage_are_retained(self):
        c = self.make()
        self._settle(c, 3.0, "A")
        for cmd, meas, stage in ((3.0, 2.5, "A"), (3.0, 1.2, "B"), (3.0, 2.9, "C")):
            self.arm(c, cmd, meas, stage)
            c._track_altitude_error()
        self.assertAlmostEqual(c.alt_error_worst, 1.8, places=6)
        self.assertEqual(c.alt_error_worst_stage, "B")

    def test_it_does_not_spam_the_same_stage(self):
        c = self.make()
        self._settle(c, 3.0, "ApproachBanner")
        for _ in range(20):
            self.arm(c, 3.0, 1.0, "ApproachBanner")
            c._track_altitude_error()
        self.assertEqual(len(c.logs), 1)

    def test_a_NEW_stage_diverging_is_reported_again(self):
        c = self.make()
        self._settle(c, 3.0, "ApproachBanner")
        self.arm(c, 3.0, 1.0, "ApproachBanner")
        c._track_altitude_error()
        self.arm(c, 3.0, 1.0, "Corridor")
        c._track_altitude_error()
        self.assertEqual(len(c.logs), 2)
        self.assertIn("Corridor", c.logs[1])

    def test_flying_HIGH_is_not_reported_as_a_sink(self):
        """Only a low aircraft is the failure being hunted."""
        c = self.arm(self.make(), 3.0, 6.0, "ClimbToSweep")
        c._track_altitude_error()
        self.assertEqual(c.logs, [])
        self.assertEqual(c.alt_error_worst, 0.0)

    def test_a_COMMANDED_climb_is_not_a_divergence(self):
        """The false alarms the first version produced in flight:

            ALTITUDE DIVERGENCE in ClimbToSweep: commanded 10.00 m,
                measured 2.99 m (low by 7.01 m)
            ALTITUDE DIVERGENCE in WinchDrop: commanded 10.00 m,
                measured 5.00 m (low by 5.00 m)

        Both are the aircraft on its way to a newly commanded altitude, which
        is the mission working correctly.
        """
        c = self.make()
        self._settle(c, 3.0, "Corridor")
        self.arm(c, 10.0, 3.0, "ClimbToSweep")     # new target, still climbing
        c._track_altitude_error()
        self.assertEqual(c.logs, [])

    def test_falling_away_AFTER_arriving_is_still_caught(self):
        """The guard must not suppress the real thing."""
        c = self.make()
        self._settle(c, 10.0, "ClimbToSweep")
        self.arm(c, 10.0, 8.5, "LawnmowerSearch")
        c._track_altitude_error()
        self.assertEqual(len(c.logs), 1)
        self.assertIn("LawnmowerSearch", c.logs[0])

    def test_an_unknown_altitude_is_not_a_divergence(self):
        c = self.arm(self.make(), 3.0, 0.0, "Corridor")
        c.alt = lambda: None
        c._track_altitude_error()
        self.assertEqual(c.logs, [])


class FrontierGrowsSidewaysTests(unittest.TestCase):
    """The SEARCH must expand laterally, not just the helper that can.

    sim/test_search_planner.py proves grow_zone() can push a zone in any
    direction. That is not the same as LawnmowerSearch using more than one:
    reducing EXPANSION_DIRECTIONS to forward-only passed the entire suite,
    which is the third time this session a calculation was tested and its
    wiring was not.

    Seed 1002 swept four strips out to x = 74.7 and never found pad E, which
    sat off to one side of the corridor.
    """

    def build(self, budget=60.0, window=(10.0, 30.0, -5.0, 5.0)):
        from mission_bt.mission_tree import LawnmowerSearch
        mav = FakeMav()
        mav.open_extent = lambda: (12.0, 16.0)
        leaf = LawnmowerSearch(mav, window, alt=10.0,
                               search_budget_m=budget, min_step_m=4.0)
        leaf.initialise()
        return leaf

    def test_the_first_advance_goes_forward(self):
        leaf = self.build()
        before = leaf.swept_zone
        self.assertTrue(leaf._advance_frontier())
        self.assertGreater(leaf.swept_zone[1], before[1],
                           "first expansion did not extend the far edge")

    def test_the_swept_zone_grows_in_BOTH_axes(self):
        leaf = self.build()
        start = leaf.swept_zone
        for _ in range(3):
            self.assertTrue(leaf._advance_frontier(), "ran out of budget early")
        grew_x = leaf.swept_zone[1] - start[1]
        grew_y = (leaf.swept_zone[3] - start[3]) + (start[2] - leaf.swept_zone[2])
        self.assertGreater(grew_x, 0.0, "never expanded forward")
        self.assertGreater(grew_y, 0.0,
                           "never expanded sideways — a pad beside the "
                           "corridor stays unreachable")

    def test_it_reaches_BOTH_sides(self):
        leaf = self.build(budget=120.0)
        start = leaf.swept_zone
        for _ in range(6):
            if not leaf._advance_frontier():
                break
        self.assertGreater(leaf.swept_zone[3], start[3], "never expanded left")
        self.assertLess(leaf.swept_zone[2], start[2], "never expanded right")

    def test_the_budget_still_bounds_it(self):
        leaf = self.build(budget=20.0)
        advances = 0
        while leaf._advance_frontier():
            advances += 1
            self.assertLess(advances, 50, "expansion never terminated")
        self.assertLess(leaf._budget_left, 4.0)

    def test_each_advance_sweeps_NEW_ground(self):
        """A band that re-flew covered ground would burn budget for nothing."""
        leaf = self.build()
        seen = []
        for _ in range(3):
            self.assertTrue(leaf._advance_frontier())
            seen.append(tuple(leaf._frontier))
        self.assertEqual(len(set(seen)), len(seen), f"repeated bands: {seen}")

    def test_a_pad_beside_the_corridor_becomes_reachable(self):
        """Seed 1002's failure, as geometry: forward-only can never cover it."""
        leaf = self.build(budget=120.0)
        pad = (25.0, 13.0)
        for _ in range(8):
            if not leaf._advance_frontier():
                break
        z = leaf.swept_zone
        self.assertTrue(z[0] <= pad[0] <= z[1] and z[2] <= pad[1] <= z[3],
                        f"pad {pad} still outside the swept zone {z}")


class DeliveryAccuracyTests(unittest.TestCase):
    """Payload Delivery Accuracy is 15 rulebook marks and nothing measured it.

    The mission reported

        landing PRECISE (committed at 3.86 m, 0 re-acquisition(s))

    which is a TRACKING-QUALITY claim wearing an accuracy-sounding name: how
    high the aircraft was when visual lock committed, and how many times the
    pad was re-acquired. Nothing about how far from the pad anything landed.

    Derived from PERCEPTION -- the QR detector's normalised offset converted
    to metres at the known altitude and field of view. Reading the pad's true
    pose out of the simulator would measure the simulator and would be the
    hardcoding the architecture removed.
    """

    def build(self, ox=0.0, oy=0.0, z=1.0, alt=5.0):
        from mission_bt.mission_tree import WinchDrop
        mav = FakeMav()
        mav._pos = (20.0, 3.0, alt)
        mav._alt = alt          # FakeMav.alt() reads _alt, not _pos[2]
        mav.qr_off = (ox, oy) if z > 0.0 else None
        leaf = WinchDrop(mav, 5.0, 10.0, hfov_rad=1.0472,
                         image_w_px=1280, image_h_px=720)
        return leaf, mav

    def test_a_centred_release_measures_about_zero(self):
        leaf, mav = self.build(0.0, 0.0)
        leaf._record_delivery_offset()
        self.assertIsNotNone(mav.delivery_offset_m)
        self.assertLess(mav.delivery_offset_m, 0.05)

    def test_an_offset_release_measures_METRES_not_pixels(self):
        """At 5 m with a 60 degree HFOV the half-width is 2.89 m, so a
        half-frame offset is about 1.44 m on the ground."""
        leaf, mav = self.build(0.5, 0.0, alt=5.0)
        leaf._record_delivery_offset()
        self.assertAlmostEqual(mav.delivery_offset_m, 1.44, places=1)

    def test_the_measurement_scales_with_ALTITUDE(self):
        """The same image offset is more ground error from higher up."""
        leaf_lo, mav_lo = self.build(0.4, 0.0, alt=3.0)
        leaf_lo._record_delivery_offset()
        leaf_hi, mav_hi = self.build(0.4, 0.0, alt=9.0)
        leaf_hi._record_delivery_offset()
        self.assertGreater(mav_hi.delivery_offset_m,
                           mav_lo.delivery_offset_m * 2.5)

    def test_vertical_offset_uses_the_VERTICAL_field_of_view(self):
        """A landscape sensor sees less vertically; using HFOV for both axes
        would overstate the error."""
        leaf_x, mav_x = self.build(0.5, 0.0)
        leaf_x._record_delivery_offset()
        leaf_y, mav_y = self.build(0.0, 0.5)
        leaf_y._record_delivery_offset()
        self.assertLess(mav_y.delivery_offset_m, mav_x.delivery_offset_m)

    def test_no_target_in_frame_reports_UNKNOWN_not_zero(self):
        """Zero would read as a perfect drop. Unknown is the honest answer."""
        leaf, mav = self.build(0.0, 0.0, z=0.0)
        leaf._record_delivery_offset()
        self.assertIsNone(mav.delivery_offset_m)
        self.assertIn("no target", mav.delivery_note)

    def test_an_unknown_altitude_reports_UNKNOWN(self):
        leaf, mav = self.build(0.3, 0.3)
        mav.alt = lambda: None
        leaf._record_delivery_offset()
        self.assertIsNone(mav.delivery_offset_m)

    def test_the_measurement_is_taken_AT_RELEASE(self):
        """Not at arrival over the pad, and not after the climb-out: the
        number has to describe the moment the payload actually left."""
        import inspect
        from mission_bt.mission_tree import WinchDrop
        src = inspect.getsource(WinchDrop.update)
        before = src.index('_record_delivery_offset')
        released = src.index('w.get("released")')
        self.assertLess(released, before,
                        "the offset is recorded somewhere other than at the "
                        "release gate")

    def test_the_delivery_number_reaches_the_MISSION_RESULT(self):
        """A measurement nobody reads is not a measurement.

        The completion line carried the landing figure (5 marks) and said
        nothing about the drop (15 marks).
        """
        import inspect
        from mission_bt.mission_tree import Land
        src = inspect.getsource(Land)
        self.assertIn("delivery_offset_m", src)
        self.assertIn("delivery", src)

    def test_an_unmeasured_delivery_says_so_rather_than_claiming_zero(self):
        import inspect
        from mission_bt.mission_tree import Land
        self.assertIn("UNMEASURED", inspect.getsource(Land))


class ReturnIdentificationAltitudeTests(unittest.TestCase):
    """The return lap must identify the banner from where it is resolvable.

    Seed 1001 failed here on every recorded run. Aligning from 10 m the
    detector reported, across one sweep:

        only 1 white component(s); lettering expected   (x895)
        only 0 white component(s)                       (x92)
        board aspect 0.73 outside 1.2-8.0               (x70)

    The lettering is not resolvable at that range and depression angle, so
    even the text rescue cannot help: it needs letter blobs to read and there
    is one. The outbound leg identifies the same banner from 5 m without
    trouble, so the return leg now descends to the same altitude first.
    """

    def _stage_names(self):
        from mission_bt.mission_tree import build_root
        from unittest.mock import MagicMock
        params = {
            'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
            'image_width_px': 1280, 'image_height_px': 720,
            'camera_hfov': 1.0472, 'target_marker_m': 2.2, 'qr_modules': 33,
            'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
            'zone_margin': 1.0, 'corridor_alt': 3.0, 'waypoint_tol': 0.8,
            'drop_tol': 0.5, 'scan_floor_alt': 2.0, 'land_commit_alt': 1.5,
            'banner_sweep_limit': math.pi,
        }
        root = build_root(FakeMav(), MagicMock(), params)
        mission = [c for c in root.children if c.name == "Mission"][0]
        return [c.name for c in mission.children]

    def test_the_return_lap_descends_BEFORE_re_identifying(self):
        names = self._stage_names()
        descend = names.index("DescendToReturnIdent")
        aligns = [i for i, n in enumerate(names) if n == "AlignToBanner"]
        self.assertGreater(len(aligns), 1, "the return lap never re-identifies")
        self.assertLess(descend, aligns[-1],
                        "the return alignment still happens before descending")

    def test_it_still_crosses_the_zone_at_the_rulebook_altitude(self):
        """The descent must come AFTER the 10 m transit, not instead of it."""
        names = self._stage_names()
        self.assertLess(names.index("ClimbForReturn"),
                        names.index("ReturnToCorridorMouth"))
        self.assertLess(names.index("ReturnToCorridorMouth"),
                        names.index("DescendToReturnIdent"))

    def test_the_corridor_descent_still_follows_identification(self):
        names = self._stage_names()
        self.assertLess(names.index("DescendToReturnIdent"),
                        names.index("DescendToReturnCorridor"))


class ReturnStandoffTests(unittest.TestCase):
    """Stop SHORT of the corridor mouth so the banner is in frame.

    The recorded exit pose is where the corridor opened out, which is where
    the return banner hangs. Flying exactly there puts the aircraft on top of
    it: from 5 m with the camera at -20 degrees the board is almost straight
    down and out of frame.

    Seed 1001 failed here on every run, sweeping a full 180 degrees while the
    detector reported "only 1 white component" — looking past a banner that
    was below it. The outbound leg identifies the same banner without trouble
    because it starts about 6 m away.
    """

    def build(self, alt=5.0, exit_pose=(15.7, -5.6, 3.0, 0.18), **kw):
        from mission_bt.mission_tree import ReturnToCorridorMouth
        mav = FakeMav()
        mav.corridor_exit_pose = exit_pose
        return ReturnToCorridorMouth(mav, alt=alt, **kw), mav

    def test_the_standoff_is_DERIVED_from_the_camera_geometry(self):
        """(altitude - banner centre) / tan(depression)."""
        leaf, _ = self.build(alt=5.0)
        expected = (5.0 - 3.38) / math.tan(math.radians(20.0))
        self.assertAlmostEqual(leaf.standoff(), expected, places=2)

    def test_a_higher_approach_stands_further_off(self):
        low, _ = self.build(alt=4.0)
        high, _ = self.build(alt=8.0)
        self.assertGreater(high.standoff(), low.standoff())

    def test_it_stops_SHORT_of_the_mouth_not_on_it(self):
        leaf, mav = self.build()
        leaf.tick_once()
        tx, ty = mav.gotos[-1][0], mav.gotos[-1][1]
        mx, my = 15.7, -5.6
        self.assertGreater(math.hypot(tx - mx, ty - my), 2.0,
                           "the aircraft still flies onto the banner")

    def test_it_stops_on_the_DELIVERY_ZONE_side(self):
        """Short of the mouth means back toward the zone it came from, not
        past the mouth into the corridor."""
        leaf, mav = self.build(exit_pose=(15.7, 0.0, 3.0, 0.0))
        leaf.tick_once()
        tx = mav.gotos[-1][0]
        self.assertGreater(tx, 15.7,
                           "stood off on the wrong side — inside the corridor")

    def test_the_standoff_follows_a_ROTATED_corridor(self):
        leaf, mav = self.build(exit_pose=(10.0, 0.0, 3.0, math.pi / 2))
        leaf.tick_once()
        tx, ty = mav.gotos[-1][0], mav.gotos[-1][1]
        # Exit heading +90 deg; reversed is -90; standing off moves +y. The
        # return lane is 4 m to starboard of a +90 deg exit: +x.
        self.assertGreater(ty, 0.0)
        self.assertAlmostEqual(tx, 14.0, places=1)

    def test_no_recorded_exit_still_heads_home(self):
        leaf, mav = self.build(exit_pose=None)
        leaf.tick_once()
        self.assertIn("heading home", leaf.feedback_message)

    def test_a_standoff_on_red_ground_moves_to_the_nearest_clear_point(self):
        """Arena 1004, batch E: the main red zone's corner (home-local x 24.8,
        y 3.4) sat 2 m from the nominal stand-off; with the map's edge and
        the router's inflation that put the point on red, and the mission
        failed with the corridor in sight."""
        from mission_bt.search_planner import routing_obstacles
        # lane_offset_m=0: this is about clear_standoff itself; with the
        # return-lane offset the nominal point is no longer on the red at all
        # (ReturnLaneStandoffTests).
        leaf, mav = self.build(exit_pose=(18.4, 3.35, 3.0, 0.079),
                               lane_offset_m=0.0)
        mav._pos = (40.0, 0.0, 10.0)
        # As confirmed by the map, whose cells run ~0.5 m past the paint.
        mav.exclusions = [(22.3, 33.3, 4.9, 12.9)]
        blocks = routing_obstacles(mav.exclusions, leaf.router.clearance_m)
        mx, my = 18.4, 3.35
        tyaw = 0.079 + math.pi
        back = leaf.standoff()
        nominal = (mx - back * math.cos(tyaw), my - back * math.sin(tyaw))
        inside = lambda p: any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1
                               for x0, x1, y0, y1 in blocks)
        self.assertTrue(inside(nominal), "the fixture no longer reproduces it")

        leaf.tick_once()
        self.assertNotEqual(leaf.status, py_trees.common.Status.FAILURE,
                            leaf.feedback_message)
        tx, ty = leaf.clear_standoff(mx, my, tyaw, back)
        self.assertFalse(inside((tx, ty)))
        # Still a stand-off AlignToBanner can use, on the zone side.
        self.assertGreaterEqual(math.hypot(tx - mx, ty - my), 3.0)
        self.assertLessEqual(math.hypot(tx - mx, ty - my), 5.5)
        self.assertGreater(tx, mx)

    def test_a_clear_standoff_is_not_moved(self):
        leaf, mav = self.build(exit_pose=(18.4, 3.35, 3.0, 0.0),
                               lane_offset_m=0.0)
        mav.exclusions = [(40.0, 45.0, -10.0, -5.0)]
        back = leaf.standoff()
        tx, ty = leaf.clear_standoff(18.4, 3.35, math.pi, back)
        self.assertAlmostEqual(tx, 18.4 + back, places=6)
        self.assertAlmostEqual(ty, 3.35, places=6)


# --------------------------------------------------------------------------- #
class ReturnLaneStandoffTests(unittest.TestCase):
    """The return stand-off faces the RETURN lane, not the one flown out of.

    The recorded exit is on the outbound lane's centreline; the return banner
    hangs over the other lane, 4 m to starboard (corridor model: lanes at y
    +2 / -2, banners at (2, +2) and (12, -2)). Standing off the outbound exit
    left the aircraft 35 deg off the return board, and in arena 1004 (batch
    F) the square-up's travel round the board ran into the main red zone.
    """

    # Arena 1004's outbound exit, as logged (home-local; corridor yaw 0.079).
    EXIT = (18.4, 2.95, 3.0, 0.079)

    def build(self, gate_heading=None, **kw):
        from mission_bt.mission_tree import ReturnToCorridorMouth
        mav = FakeMav()
        mav.corridor_exit_pose = self.EXIT
        mav.gate_heading = gate_heading
        mav._pos = (40.0, 0.0, 10.0)
        return ReturnToCorridorMouth(mav, alt=10.0, ident_alt=5.0, **kw), mav

    def _target(self, leaf, mav):
        leaf.tick_once()
        return mav.gotos[-1][0], mav.gotos[-1][1]

    @staticmethod
    def _corridor_frame(p, origin, axis):
        """(along, across) of p from origin, across port-positive."""
        dx, dy = p[0] - origin[0], p[1] - origin[1]
        c, s = math.cos(axis), math.sin(axis)
        return dx * c + dy * s, -dx * s + dy * c

    def test_the_standoff_is_across_in_front_of_the_return_lane(self):
        leaf, mav = self.build()
        t = self._target(leaf, mav)
        along, across = self._corridor_frame(t, self.EXIT[:2], self.EXIT[3])
        self.assertAlmostEqual(across, -4.0, places=3,
                               msg="not in line with the return lane")
        self.assertAlmostEqual(along, leaf.standoff(), places=3)

    def test_it_faces_back_down_the_corridor_axis(self):
        leaf, mav = self.build()
        _, _, tyaw = leaf.return_mouth(self.EXIT)
        self.assertAlmostEqual(math.cos(tyaw - (self.EXIT[3] + math.pi)), 1.0,
                               places=9)

    def test_the_gate_heading_beats_the_exit_yaw(self):
        """The navigator weaves down the lane, so the exit yaw is a noisy
        axis; the outbound square-on is lidar-measured to 5 deg."""
        wobbly = (18.4, 2.95, 3.0, 0.079 + math.radians(12.0))
        leaf, mav = self.build(gate_heading=0.079)
        mav.corridor_exit_pose = wobbly
        t = self._target(leaf, mav)
        _, across = self._corridor_frame(t, wobbly[:2], 0.079)
        self.assertAlmostEqual(across, -4.0, places=3)

    def test_arena_1004_the_new_standoff_is_clear_of_the_red_zone(self):
        """Batch F's failure: the square-up's arc from the outbound-lane
        stand-off ran at the main red zone's corner (home-local x 24.8,
        y 3.4). The return-lane stand-off is not near it."""
        from mission_bt.search_planner import routing_obstacles
        leaf, mav = self.build(gate_heading=0.079)
        mav.exclusions = [(24.5, 35.0, 3.1, 10.7)]      # the map's box on it
        t = self._target(leaf, mav)
        blocks = routing_obstacles(mav.exclusions, leaf.router.clearance_m)
        self.assertFalse(any(x0 <= t[0] <= x1 and y0 <= t[1] <= y1
                             for x0, x1, y0, y1 in blocks))
        self.assertFalse(any("on red ground" in m for m, _ in mav.logs),
                         "the stand-off still had to be moved off red")

    def test_the_offset_is_a_parameter(self):
        leaf, mav = self.build(lane_offset_m=3.0)
        t = self._target(leaf, mav)
        _, across = self._corridor_frame(t, self.EXIT[:2], self.EXIT[3])
        self.assertAlmostEqual(across, 3.0, places=3)


# --------------------------------------------------------------------------- #
class DeliveryMeasurementSurvivesTests(unittest.TestCase):
    """A completed run reported `delivery UNMEASURED`. 15 marks ride on it.

    THE OBSERVED FAILURE: arena seed 1003 flew the whole mission and landed
    precisely, and still could not say how far from the pad it dropped. The
    plumbing was right and reported honestly rather than inventing a zero --
    but at the instant of release the detector had no valid offset.

    TWO CAUSES, TWO GUARDS

        1. Geometry. Below the altitude at which the whole pad fits in frame,
           a measurement is not merely unlikely, it is impossible. The drop
           altitude has to be at or above that floor or the number can never
           exist.

        2. A single frame. The offset was read on exactly the tick the winch
           reported the release. One dropped frame there discards a perfectly
           good measurement taken a moment earlier -- and the payload swinging
           under the aircraft is in the nadir camera's view at precisely that
           moment.
    """

    def build(self, ox=0.0, oy=0.0, z=1.0, alt=5.0, **kw):
        from mission_bt.mission_tree import WinchDrop
        mav = FakeMav()
        mav._pos = (20.0, 3.0, alt)
        mav._alt = alt
        mav.qr_off = (ox, oy) if z > 0.0 else None
        kw.setdefault("hfov_rad", 1.0472)
        kw.setdefault("image_w_px", 1280)
        kw.setdefault("image_h_px", 720)
        leaf = WinchDrop(mav, kw.pop("drop_alt", 5.0), 10.0, **kw)
        return leaf, mav

    # ---- guard 1: the geometry has to permit a measurement ---- #
    def test_a_drop_below_the_tracking_floor_is_raised_to_it(self):
        """Below it the pad cannot fit in frame, so the offset cannot exist.
        A drop altitude that makes the scored quantity unmeasurable is not a
        trade-off anyone chose."""
        leaf, mav = self.build(drop_alt=2.0, marker_m=2.2)
        leaf.initialise()
        self.assertGreater(leaf.drop_alt, 2.0)
        self.assertGreaterEqual(leaf.drop_alt, leaf.tracking_floor())

    def test_raising_the_drop_altitude_is_LOGGED(self):
        leaf, mav = self.build(drop_alt=2.0, marker_m=2.2)
        leaf.initialise()
        self.assertTrue([m for m, _ in mav.logs if "tracking floor" in m],
                        mav.logs)

    def test_a_drop_altitude_already_above_the_floor_is_left_alone(self):
        leaf, mav = self.build(drop_alt=5.0, marker_m=2.2)
        leaf.initialise()
        self.assertAlmostEqual(leaf.drop_alt, 5.0)

    def test_the_floor_is_derived_from_the_marker_and_the_camera(self):
        """Not a constant. A bigger pad needs more altitude to fit in frame."""
        small, _ = self.build(marker_m=1.0)
        large, _ = self.build(marker_m=3.0)
        self.assertGreater(large.tracking_floor(), small.tracking_floor())

    # ---- guard 2: one dropped frame must not discard the measurement ---- #
    def test_an_offset_seen_moments_earlier_is_used_when_the_frame_is_lost(self):
        leaf, mav = self.build(0.5, 0.0, alt=5.0)
        leaf.observe_offset()                       # a good frame
        mav.qr_off = None                           # lock lost at release
        leaf._record_delivery_offset()
        self.assertIsNotNone(mav.delivery_offset_m)
        self.assertAlmostEqual(mav.delivery_offset_m, 1.44, places=1)

    def test_a_fallback_measurement_SAYS_it_is_a_fallback(self):
        """An operator must be able to tell a measurement at release from one
        recovered from a second earlier."""
        leaf, mav = self.build(0.5, 0.0)
        leaf.observe_offset()
        mav.qr_off = None
        leaf._record_delivery_offset()
        self.assertIn("last seen", mav.delivery_note)

    def test_the_fallback_reports_its_AGE(self):
        leaf, mav = self.build(0.5, 0.0)
        leaf.observe_offset()
        for _ in range(7):
            leaf.observe_offset() if False else None
            leaf._t += 1
        mav.qr_off = None
        leaf._record_delivery_offset()
        self.assertRegex(mav.delivery_note, r"\d+ tick")

    def test_a_STALE_offset_is_refused(self):
        """A measurement from thirty seconds ago is about somewhere else."""
        leaf, mav = self.build(0.5, 0.0)
        leaf.observe_offset()
        leaf._t += 10000
        mav.qr_off = None
        leaf._record_delivery_offset()
        self.assertIsNone(mav.delivery_offset_m)

    def test_phase_changes_do_not_make_an_old_descent_sighting_fresh_again(self):
        leaf, mav = self.build(0.5, 0.0, alt=9.15, max_offset_age_ticks=40)
        mav.winch = lambda command: None
        leaf.initialise()
        mav.winch_status = {}
        for _ in range(60):
            leaf.update()
        mav.qr_off = None
        # Arrive at the (servoed) drop point. With the pad out of frame the
        # descent waits out the servo window before lowering.
        mav._pos = (leaf.drop_x, leaf.drop_y, 5.0)
        mav._alt = 5.0
        for _ in range(leaf.servo_timeout + 2):  # descent -> lower
            leaf.update()
        self.assertEqual(leaf.phase, 1)
        for _ in range(45):
            leaf.update()
        mav.winch_status = {"ground": True}
        leaf.update()  # lower -> release
        mav.winch_status = {"released": True}
        leaf.update()
        self.assertIsNone(mav.delivery_offset_m,
                          "old descent sighting was reported as release accuracy")

    def test_fallback_altitude_is_not_reported_as_release_altitude(self):
        leaf, mav = self.build(0.5, 0.0, alt=9.15)
        leaf.observe_offset()
        mav.qr_off = None
        mav._alt = 5.0
        leaf._record_delivery_offset()
        self.assertIn("release altitude 5.00 m", mav.delivery_note)
        self.assertIn("sighting altitude 9.15 m", mav.delivery_note)

    def test_a_live_offset_at_release_still_wins_over_the_fallback(self):
        """The fallback is a rescue, not a replacement: it must not shadow a
        good reading with an older one."""
        leaf, mav = self.build(0.5, 0.0)
        leaf.observe_offset()
        mav.qr_off = (0.0, 0.0)                     # centred, right now
        leaf._record_delivery_offset()
        self.assertLess(mav.delivery_offset_m, 0.05)
        self.assertNotIn("last seen", mav.delivery_note)

    def test_the_fallback_uses_the_altitude_it_was_MEASURED_at(self):
        """Metres come from the altitude at the time of the reading. Using the
        release altitude for an older offset would scale it by the wrong
        number."""
        leaf, mav = self.build(0.5, 0.0, alt=5.0)
        leaf.observe_offset()
        mav._alt = 1.0                              # descended since
        mav.qr_off = None
        leaf._record_delivery_offset()
        self.assertAlmostEqual(mav.delivery_offset_m, 1.44, places=1)

    def test_never_seeing_the_target_at_all_still_reports_UNKNOWN(self):
        """The honest answer stays available. Nothing here invents a zero."""
        leaf, mav = self.build(0.0, 0.0, z=0.0)
        leaf._record_delivery_offset()
        self.assertIsNone(mav.delivery_offset_m)
        self.assertIn("no target", mav.delivery_note)

    def test_the_descent_actually_records_offsets_as_it_goes(self):
        """The wiring. A recorder nothing calls is the defect three times
        over in this codebase."""
        leaf, mav = self.build(0.4, 0.0, alt=8.0)   # still descending
        mav.winch_status = {}
        leaf.initialise()
        for _ in range(3):
            leaf.update()
        self.assertEqual(leaf.phase, 0, "the fixture left the descent phase")
        self.assertIsNotNone(leaf._last_offset,
                             "WinchDrop never sampled the offset while flying")


# --------------------------------------------------------------------------- #
class BuiltTreeCarriesTheNewBehaviourTests(unittest.TestCase):
    """The tree the mission actually flies, not the stages in isolation.

    Every fix in this round is a per-stage parameter, and a parameter that
    build_root does not pass is a fix that exists only in its unit test. That
    is the exact shape of three defects already recorded here: a helper that
    was correct and a caller that never used it.
    """

    PARAMS = {
        'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
        'image_width_px': 1280, 'camera_hfov': 1.0472,
        'target_marker_m': 2.2, 'qr_modules': 33,
        'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
        'zone_margin': 1.0, 'corridor_alt': 3.0,
        'waypoint_tol': 0.8, 'drop_tol': 0.5, 'scan_floor_alt': 2.0,
        'land_commit_alt': 1.5, 'redzone_clearance': 1.5,
    }

    def _stages(self):
        from mission_bt.mission_tree import build_root
        from unittest.mock import MagicMock
        root = build_root(FakeMav(), MagicMock(), dict(self.PARAMS))
        out = []

        def walk(n):
            out.append(n)
            for c in getattr(n, "children", ()):
                walk(c)
        walk(root)
        return out

    def _of_type(self, name):
        return [s for s in self._stages() if type(s).__name__ == name]

    def test_the_flown_sweep_covers_a_full_turn(self):
        aligns = self._of_type("AlignToBanner")
        self.assertGreaterEqual(len(aligns), 2)
        for a in aligns:
            self.assertAlmostEqual(a.n_steps * a.step_rad, 2 * math.pi, places=6,
                                   msg="not a full turn")
            # Neighbouring stares must overlap, or a bearing falls between.
            self.assertLess(a.step_rad, a.hfov, "stares leave gaps")

    def test_the_flown_sweep_dwells_for_five_seconds(self):
        for a in self._of_type("AlignToBanner"):
            self.assertAlmostEqual(a.dwell_s, 5.0)

    def test_the_flown_sweep_requires_a_confident_dwell(self):
        """The confidence floor is what makes a full turn safe to sweep."""
        for a in self._of_type("AlignToBanner"):
            self.assertGreaterEqual(a.min_hit_ratio, 0.5)

    def test_the_flown_start_scan_hovers_over_the_marker(self):
        for s in self._of_type("ScanStartQR"):
            self.assertAlmostEqual(s.hover.hover_s, 5.0)

    def test_the_flown_sweep_hovers_on_each_marker_it_reads(self):
        for s in self._of_type("LawnmowerSearch"):
            self.assertAlmostEqual(s.hover.hover_s, 5.0)

    def test_every_transit_stage_routes_around_red_ground(self):
        """The defect this round exists for: only the sweep clipped anything."""
        for name in ("GotoHome", "ReturnToCorridorMouth", "DescendToDecode",
                     "LawnmowerSearch"):
            stages = self._of_type(name)
            self.assertTrue(stages, f"{name} is not in the tree at all")
            for s in stages:
                self.assertTrue(hasattr(s, "router"),
                                f"{name} still flies straight lines")
                self.assertAlmostEqual(s.router.clearance_m, 1.5)

    def test_the_drop_knows_the_pad_size_it_must_keep_in_frame(self):
        """Without it the tracking floor is computed for the wrong pad and
        the delivery offset can be unmeasurable by construction."""
        for s in self._of_type("WinchDrop"):
            self.assertAlmostEqual(s.marker_m, 2.2)
            self.assertGreaterEqual(s.drop_alt, 0.0)

    def test_the_flown_alignment_uses_the_real_camera_fov(self):
        """The correction converts a bearing (a fraction of the half-FOV) into
        an angle. Built with the default lens instead of the configured one,
        it would under- or over-correct on every run."""
        for a in self._of_type("AlignToBanner"):
            self.assertAlmostEqual(a.hfov, 1.0472, places=4)

    def test_the_flown_alignment_is_bounded(self):
        """It hunted for a hundred seconds in flight before this existed."""
        for a in self._of_type("AlignToBanner"):
            self.assertGreater(a.max_corrections, 0)
            self.assertLessEqual(a.max_corrections, 20)

    def test_each_gate_advance_uses_its_preceding_ducks_measured_altitude(self):
        """A DuckUnderBoard that succeeds without supplying GateAdvance is
        theatre. The built mission must carry each measured altitude into the
        matching through-gate leg."""
        stages = self._stages()
        ducks = [s for s in stages if type(s).__name__ == "DuckUnderBoard"]
        advances = [s for s in stages if type(s).__name__ == "GateAdvance"]
        self.assertEqual(len(ducks), 2)
        self.assertEqual(len(advances), 2)

        for measured, duck, advance in zip((2.1, 1.9), ducks, advances):
            duck._target_alt = measured
            self.assertTrue(callable(advance.alt))
            self.assertAlmostEqual(advance.alt(), measured)
