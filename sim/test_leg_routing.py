#!/usr/bin/env python3
"""Every leg the aircraft flies is checked against confirmed red ground.

WHAT THIS REPLACES

    `mav.goto(x, y, z, yaw)` — a straight line to a destination, issued by
    every transit stage in the tree, that never once asked whether the line
    crossed a restricted zone. Only the search sweep clipped anything.

    The measured failure: 198 confirmed exclusion cells during a nadir sweep,
    and the aircraft still flew over red ground. Restricted-zone avoidance is
    10 marks with -5 per violation.

WHAT THESE TESTS ARE CAREFUL ABOUT

    They assert on the setpoints that were actually COMMANDED, not on what
    route_leg() returns. Three separate defects in this stack shipped with
    passing tests because the tests exercised a correct helper that nothing
    called correctly — an altitude hold whose correction was computed and then
    published as zero, a banner rescue that was never reached, a lateral
    search that was never invoked. The helper is tested in
    test_search_planner.py. This file tests the wiring.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_leg_routing.py -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mission_bt.leg_router import ARRIVED, BLOCKED, RUNNING, LegRouter
from mission_bt.search_planner import path_hits_exclusion

BOX = [(10.0, 20.0, -5.0, 5.0)]


class FlyingMav:
    """A vehicle that teleports to whatever it was last told to fly to.

    Deliberately crude: the point is to observe the SEQUENCE of commands, not
    to model flight. Anything that reads back a position the router never
    commanded would be modelling the simulator instead of the router.
    """

    def __init__(self, at=(0.0, 0.0, 5.0), exclusions=()):
        self._pos = tuple(float(v) for v in at)
        self.exclusions = list(exclusions)
        self.gotos = []
        self.logs = []

    def goto(self, x, y, z, yaw=0.0):
        self.gotos.append((float(x), float(y), float(z), float(yaw)))

    def pos(self):
        return self._pos

    def reached(self, x, y, z, tol=0.6):
        px, py, pz = self._pos
        return math.dist((px, py, pz), (x, y, z)) < tol

    def log(self, msg, warn=False):
        self.logs.append((msg, warn))

    # ---- test driving ---- #
    def arrive(self):
        """Teleport to the last commanded setpoint."""
        if self.gotos:
            self._pos = self.gotos[-1][:3]

    def fly(self, router, x, y, z, yaw=0.0, ticks=12):
        """Tick the router to completion, arriving at each setpoint."""
        seen = [self._pos[:2]]
        for _ in range(ticks):
            status = router.fly(self, x, y, z, yaw)
            if status is BLOCKED:
                return status, seen
            self.arrive()
            if seen[-1] != self._pos[:2]:
                seen.append(self._pos[:2])
            if status is ARRIVED:
                return status, seen
        return RUNNING, seen


class ClearLegTests(unittest.TestCase):

    def test_arrival_tolerance_cannot_cut_across_an_exclusion_corner(self):
        mav = FlyingMav(at=(17.75, -8.0, 10.0),
                        exclusions=[(8.0, 16.0, -8.0, 8.0)])
        router = LegRouter(clearance_m=1.5, tol=0.8)
        router.fly(mav, 6.25, -8.0, 10.0)
        self.assertFalse(path_hits_exclusion(
            [mav.pos()[:2], mav.gotos[-1][:2]], 1.5, mav.exclusions))

    def test_a_clear_leg_is_commanded_straight_to_the_destination(self):
        mav = FlyingMav()
        r = LegRouter()
        status = r.fly(mav, 30.0, 0.0, 5.0)
        self.assertEqual(mav.gotos[0][:3], (30.0, 0.0, 5.0))
        self.assertIs(status, RUNNING)

    def test_it_reports_arrival_once_the_destination_is_reached(self):
        mav = FlyingMav()
        r = LegRouter()
        status, _ = mav.fly(r, 30.0, 0.0, 5.0)
        self.assertIs(status, ARRIVED)

    def test_a_vehicle_with_no_exclusions_attribute_still_flies(self):
        """Defensive: an unwired mav must not silently stop flying legs."""
        class Bare(FlyingMav):
            pass
        mav = Bare()
        del mav.exclusions
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertEqual(mav.gotos[0][:3], (30.0, 0.0, 5.0))

    def test_arrival_is_three_dimensional(self):
        """Reaching the x,y of a waypoint 4 m below is not reaching it."""
        mav = FlyingMav(at=(30.0, 0.0, 1.0))
        r = LegRouter()
        self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), RUNNING)


class DetouredLegTests(unittest.TestCase):

    def test_the_first_setpoint_is_NOT_the_destination(self):
        """The wiring assertion. A router that computes a detour and then
        commands the straight line is the exact bug this file exists for."""
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertNotEqual(mav.gotos[0][:2], (30.0, 0.0))

    def test_it_eventually_commands_the_destination(self):
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        status, _ = mav.fly(r, 30.0, 0.0, 5.0)
        self.assertIs(status, ARRIVED)
        self.assertEqual(mav.gotos[-1][:2], (30.0, 0.0))

    def test_the_flown_track_never_crosses_the_exclusion(self):
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        status, track = mav.fly(r, 30.0, 0.0, 5.0)
        self.assertIs(status, ARRIVED)
        self.assertFalse(path_hits_exclusion(track, r.clearance_m, BOX),
                         f"flown track crossed red ground: {track}")

    def test_every_intermediate_setpoint_holds_the_leg_altitude(self):
        """A detour that descends is a detour into the ground."""
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        mav.fly(r, 30.0, 0.0, 7.5)
        self.assertTrue(all(g[2] == 7.5 for g in mav.gotos), mav.gotos)

    def test_every_intermediate_setpoint_holds_the_commanded_yaw(self):
        """The camera has to keep pointing where the stage wanted it."""
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        mav.fly(r, 30.0, 0.0, 5.0, yaw=1.25)
        self.assertTrue(all(g[3] == 1.25 for g in mav.gotos), mav.gotos)

    def test_the_detour_is_logged_once_not_every_tick(self):
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        mav.fly(r, 30.0, 0.0, 5.0)
        detour_logs = [m for m, _ in mav.logs if "red zone" in m]
        self.assertEqual(len(detour_logs), 1, mav.logs)


class ReplanTests(unittest.TestCase):

    def test_a_zone_confirmed_mid_leg_diverts_the_leg_being_flown(self):
        """Exclusions arrive cell by cell while the aircraft is already
        moving. A route computed once, before the zone was confirmed, is a
        route through it."""
        mav = FlyingMav(exclusions=[])
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertEqual(mav.gotos[-1][:2], (30.0, 0.0))   # straight, so far
        mav.exclusions = list(BOX)
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertNotEqual(mav.gotos[-1][:2], (30.0, 0.0),
                            "kept flying the straight line after the zone was "
                            "confirmed")

    def test_an_unchanged_exclusion_set_does_not_rebuild_the_route(self):
        """Re-planning every tick would make the setpoint jitter between two
        equal-length ways round."""
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        first = mav.gotos[-1]
        for _ in range(5):
            r.fly(mav, 30.0, 0.0, 5.0)
        self.assertTrue(all(g == first for g in mav.gotos), mav.gotos)

    def test_a_new_destination_rebuilds_the_route(self):
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        n = len(mav.gotos)
        r.fly(mav, 0.0, 30.0, 5.0)
        self.assertNotEqual(mav.gotos[n][:2], mav.gotos[n - 1][:2])


class FailClosedTests(unittest.TestCase):

    WALL = [(10.0, 12.0, -400.0, 400.0)]

    def test_a_leg_with_no_route_reports_BLOCKED(self):
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)

    def test_a_blocked_leg_commands_nothing_at_all(self):
        """Fail closed means STOP, not 'fly it anyway and hope the fence
        catches it'."""
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertEqual(mav.gotos, [])

    def test_a_blocked_leg_carries_a_reason_naming_the_destination(self):
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertIn("30.0", r.blocked_reason)
        self.assertTrue(r.blocked_reason)

    def test_a_destination_inside_a_zone_is_refused(self):
        mav = FlyingMav(exclusions=BOX)
        r = LegRouter()
        self.assertIs(r.fly(mav, 15.0, 0.0, 5.0), BLOCKED)
        self.assertIn("destination", r.blocked_reason.lower())

    def test_leaving_a_zone_the_aircraft_is_already_inside_is_allowed(self):
        mav = FlyingMav(at=(15.0, 0.0, 5.0), exclusions=BOX)
        r = LegRouter()
        status, _ = mav.fly(r, 30.0, 0.0, 5.0)
        self.assertIs(status, ARRIVED)

    def test_a_blocked_leg_can_be_ticked_again_without_crashing(self):
        """Found in review, not by a test: every test in this file ticked a
        blocked leg exactly once.

        A stage that reports FAILURE can still be re-entered by the tree, and
        the second tick took the cached "same destination, same zones" path
        straight into an empty waypoint list. A crash inside the behaviour
        tree is worse than the violation it was preventing.
        """
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)
        for _ in range(5):
            self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)
        self.assertEqual(mav.gotos, [])

    def test_a_blocked_leg_keeps_its_reason_across_ticks(self):
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        first = r.blocked_reason
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertEqual(r.blocked_reason, first)

    def test_a_blocked_leg_unblocks_when_the_zones_change(self):
        """The exclusion set only ever grows in flight, but a re-plan that
        could never clear a block would strand the aircraft on a false
        positive that later got corrected."""
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)
        mav.exclusions = []
        self.assertIsNot(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)
        self.assertEqual(r.blocked_reason, "")

    def test_a_blocked_leg_unblocks_when_the_destination_changes(self):
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        self.assertIs(r.fly(mav, 30.0, 0.0, 5.0), BLOCKED)
        self.assertIsNot(r.fly(mav, 0.0, 20.0, 5.0), BLOCKED)

    def test_reset_clears_a_previous_block(self):
        mav = FlyingMav(exclusions=self.WALL)
        r = LegRouter()
        r.fly(mav, 30.0, 0.0, 5.0)
        self.assertTrue(r.blocked_reason)
        r.reset()
        self.assertEqual(r.blocked_reason, "")


if __name__ == "__main__":
    unittest.main()


# --------------------------------------------------------------------------- #
# The wiring: which stages actually route
# --------------------------------------------------------------------------- #

import py_trees                                              # noqa: E402
from test_fail_closed_stages import FakeMav                  # noqa: E402


class RoutedMav(FakeMav):
    """FakeMav that carries exclusions and moves when told to."""

    def __init__(self, exclusions=(), at=(0.0, 0.0, 10.0)):
        super().__init__()
        self.exclusions = list(exclusions)
        self._pos = tuple(float(v) for v in at)
        self._alt = self._pos[2]

    def goto(self, *a, **k):
        self.gotos.append(a)

    def arrive(self):
        if self.gotos:
            g = self.gotos[-1]
            self._pos = (float(g[0]), float(g[1]), float(g[2]))
            self._alt = self._pos[2]

    def home_local_xy(self):
        return (0.0, 0.0)


class StageWiringTests(unittest.TestCase):
    """A stage that computes a detour and then flies the straight line is the
    defect. These check the setpoint, not the plan."""

    # A zone sitting between the aircraft and everywhere it wants to go.
    WALL = [(8.0, 16.0, -8.0, 8.0)]

    def _first_goto(self, stage, mav):
        stage.setup() if hasattr(stage, "setup") else None
        stage.initialise()
        stage.update()
        self.assertTrue(mav.gotos, f"{stage.name} commanded nothing")
        return mav.gotos[0]

    def test_GotoHome_routes_around_red_ground(self):
        from mission_bt.mission_tree import GotoHome
        mav = RoutedMav(exclusions=self.WALL, at=(30.0, 0.0, 10.0))
        stage = GotoHome(mav, alt=10.0)
        g = self._first_goto(stage, mav)
        self.assertNotEqual((round(g[0], 1), round(g[1], 1)), (0.0, 0.0),
                            "flew the straight line home through a red zone")

    def test_GotoHome_flies_straight_when_the_ground_is_clear(self):
        from mission_bt.mission_tree import GotoHome
        mav = RoutedMav(exclusions=[], at=(30.0, 0.0, 10.0))
        stage = GotoHome(mav, alt=10.0)
        g = self._first_goto(stage, mav)
        self.assertEqual((round(g[0], 1), round(g[1], 1)), (0.0, 0.0))

    def test_ReturnToCorridorMouth_routes_around_red_ground(self):
        from mission_bt.mission_tree import ReturnToCorridorMouth
        mav = RoutedMav(exclusions=self.WALL, at=(30.0, 0.0, 10.0))
        mav.corridor_exit_pose = (0.0, 0.0, 3.0, 0.0)
        stage = ReturnToCorridorMouth(mav, alt=10.0, standoff_m=0.0)
        g = self._first_goto(stage, mav)
        self.assertNotEqual((round(g[0], 1), round(g[1], 1)), (0.0, 0.0))

    def test_DescendToDecode_refuses_to_descend_onto_red_ground(self):
        """Descending onto a candidate is a commitment to sit above it for the
        whole descent. There is nowhere to route to; stop instead."""
        from mission_bt.mission_tree import DescendToDecode
        mav = RoutedMav(exclusions=self.WALL, at=(12.0, 0.0, 10.0))
        mav.qr_off = (0.0, 0.0)
        stage = DescendToDecode(mav, target_alt=4.0)
        stage.initialise()
        self.assertIs(stage.update(), py_trees.common.Status.FAILURE)
        self.assertEqual(mav.gotos, [],
                         "descended onto a candidate standing on red ground")

    def test_LawnmowerSearch_routes_between_lanes(self):
        """Lane SEGMENTS were clipped; the transit from the end of one to the
        start of the next never was."""
        from mission_bt.mission_tree import LawnmowerSearch
        mav = RoutedMav(exclusions=self.WALL, at=(0.0, 0.0, 10.0))
        mav.observed_zone = (-5.0, 40.0, -20.0, 20.0)
        mav.corridor_exit_pose = (0.0, 0.0, 10.0, 0.0)
        stage = LawnmowerSearch(mav, zone=lambda: mav.observed_zone,
                                exclusions=lambda: mav.exclusions,
                                image_width_px=1280, hfov_rad=math.radians(60),
                                marker_m=2.2, alt=10.0, search_budget_m=400.0)
        stage.initialise()
        track = [mav.pos()[:2]]
        for _ in range(120):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            mav.arrive()
            if track[-1] != mav.pos()[:2]:
                track.append(mav.pos()[:2])
        self.assertGreater(len(track), 2, "the sweep never moved")
        self.assertFalse(path_hits_exclusion(track, 1.5, self.WALL),
                         f"the swept track crossed red ground: {track[:12]}")

    def test_clipped_search_waypoints_are_accepted_by_the_leg_router(self):
        from mission_bt.mission_tree import LawnmowerSearch
        for exclusions in (
                [(15.0, 25.0, -3.0, 3.0)],
                [(15.0, 18.0, -3.0, 3.0), (18.0, 25.0, 1.0, 3.0)]):
            with self.subTest(exclusions=exclusions):
                mav = RoutedMav(exclusions=exclusions, at=(0.0, -8.0, 10.0))
                stage = LawnmowerSearch(
                    mav, (0.0, 40.0, -10.0, 10.0), 10.0,
                    exclusions=lambda: mav.exclusions, search_budget_m=0.0,
                    image_width_px=1280, hfov_rad=math.radians(60), marker_m=2.2)
                stage.initialise()
                for _ in range(100):
                    result = stage.update()
                    if result is not py_trees.common.Status.RUNNING:
                        break
                    mav.arrive()
                self.assertEqual(stage.skipped, 0,
                                 "the router rejected the planner's clipped endpoints")

    def test_a_blocked_leg_aborts_with_a_reason_rather_than_flying_it(self):
        from mission_bt.mission_tree import GotoHome
        walled = [(8.0, 10.0, -400.0, 400.0)]
        mav = RoutedMav(exclusions=walled, at=(30.0, 0.0, 10.0))
        stage = GotoHome(mav, alt=10.0)
        stage.initialise()
        status = stage.update()
        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertIn("red zone", mav.abort_reason)
        self.assertEqual(mav.gotos, [], "commanded a leg it had refused")

class GateAdvanceTests(unittest.TestCase):
    """Passing THROUGH the gate is a distance, not a detector state.

    THE OPERATOR'S REPORT: "the drone thinks it has been into the drop
    location because it detected the banner". ApproachBanner ended when the
    banner left the top of the frame, which happens when you get close to the
    board -- so a detection was being treated as arrival and the mission ran
    the delivery-zone stages while still at the mouth.
    """

    def _stage(self, mav, **kw):
        from mission_bt.mission_tree import GateAdvance
        kw.setdefault("advance_m", 10.0)
        kw.setdefault("alt", 3.0)
        stage = GateAdvance(mav, **kw)
        stage.initialise()
        return stage

    def test_it_does_not_arrive_just_because_the_banner_is_visible(self):
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        mav.banner_z = 1.0
        stage = self._stage(mav)
        self.assertIs(stage.update(), py_trees.common.Status.RUNNING)

    def test_it_arrives_after_the_configured_DISTANCE(self):
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav, advance_m=10.0)
        stage.update()
        mav._pos = (10.0, 0.0, 3.0)
        self.assertIs(stage.update(), py_trees.common.Status.SUCCESS)

    def test_the_target_is_LATCHED_not_recomputed_each_tick(self):
        """A target recomputed from the current position recedes as fast as
        the aircraft chases it -- the defect that pitched arena 1002 to 50
        degrees."""
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav)
        stage.update()
        first = stage._target
        for step in (2.0, 4.0, 6.0):
            mav._pos = (step, 0.0, 3.0)
            stage.update()
        self.assertEqual(stage._target, first)

    def test_it_advances_along_the_ALIGNED_heading(self):
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        mav._yaw = math.pi / 2.0            # aligned north
        stage = self._stage(mav, advance_m=10.0)
        stage.update()
        self.assertAlmostEqual(stage._target[0], 0.0, places=3)
        self.assertAlmostEqual(stage._target[1], 10.0, places=3)

    def test_it_uses_the_altitude_measured_immediately_before_the_leg(self):
        """The board edge is unknown until the preceding lidar descent has
        finished. GateAdvance must resolve that measured altitude when the
        leg begins instead of retaining the 3 m board-height configuration."""
        mav = RoutedMav(at=(0.0, 0.0, 2.1))
        stage = self._stage(mav, alt=lambda: 2.1)
        stage.update()
        self.assertAlmostEqual(mav.gotos[-1][2], 2.1)

    def test_it_refuses_when_no_safe_transit_altitude_was_measured(self):
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav, alt=lambda: None)

        status = stage.update()

        self.assertIs(status, py_trees.common.Status.FAILURE)
        self.assertFalse(mav.gotos, "advanced without a measured safe altitude")
        self.assertIn("safe transit altitude", mav.abort_reason)

    def test_it_latches_the_measured_distance_once(self):
        mav = RoutedMav(at=(0.0, 0.0, 2.0))
        measured = [6.0]
        stage = self._stage(mav, advance_m=lambda: measured[0])
        stage.update()
        measured[0] = 10.0
        mav._pos = (2.0, 0.0, 2.0)
        stage.update()
        self.assertAlmostEqual(mav.gotos[-1][0], 6.0)

    def test_unmeasured_or_invalid_crossing_distance_cannot_command_motion(self):
        for value in (None, float("nan"), float("inf"), 0.0, -1.0):
            with self.subTest(value=value):
                mav = RoutedMav(at=(0.0, 0.0, 2.0))
                stage = self._stage(mav, advance_m=lambda: value)
                self.assertIs(stage.update(), py_trees.common.Status.FAILURE)
                self.assertFalse(mav.gotos)
                self.assertIn("safe transit distance", mav.abort_reason)

    def test_only_ONE_controller_drives_the_leg(self):
        """MEASURED, run 18. This stage used to hand control to the
        follow-the-gap navigator AND keep streaming position setpoints at its
        latched target, because enabling avoidance clears the streamed
        setpoint and the router's next tick sets it again. Both then publish,
        ten times a second, and they do not want the same thing.

        The aircraft squared up, latched a target 10 m ahead at (11.6, -4.8),
        and finished at (16.0, -68.0) -- sixty-three metres south, at a steady
        3.0 m, into open field. Follow-the-gap steers at the widest opening
        and has no notion of a destination, so with a wall one side and an
        empty arena the other it flies at the arena.

        The avoidance this leg needs is the exclusion routing the LegRouter
        already does. The corridor stage after it is where the gap navigator
        belongs.
        """
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav)
        stage.update()
        self.assertFalse(mav.avoidance,
                         "handed the leg to a navigator with no destination "
                         "while still streaming setpoints at one")
        self.assertTrue(mav.gotos, "nothing flew the leg at all")

    def test_the_leg_is_still_ROUTED_around_confirmed_red_ground(self):
        """Dropping the gap navigator must not drop the exclusion routing;
        they are different things and only one of them knows the target."""
        blocked = [(3.0, 7.0, -2.0, 2.0)]
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav, exclusions=lambda: blocked, clearance_m=1.0)
        stage.update()
        for _ in range(40):
            if stage.update() is not py_trees.common.Status.RUNNING:
                break
            mav._pos = (mav.gotos[-1][0], mav.gotos[-1][1], 3.0)
        flown = [(g[0], g[1]) for g in mav.gotos]
        inside = [(x, y) for x, y in flown
                  if 3.0 <= x <= 7.0 and -2.0 <= y <= 2.0]
        self.assertEqual(inside, [],
                         f"flew the airframe through red ground: {inside}")

    def test_avoidance_is_released_when_the_leg_ends(self):
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav)
        stage.update()
        mav._pos = (10.0, 0.0, 3.0)
        stage.update()
        stage.terminate(py_trees.common.Status.SUCCESS)
        self.assertFalse(mav.avoidance)

    def test_only_the_observed_corridor_exit_anchors_search_and_return(self):
        from mission_bt.mission_tree import Corridor
        mav = RoutedMav(at=(0.0, 0.0, 3.0))
        stage = self._stage(mav)
        stage.update()
        mav._pos = (10.0, 0.0, 3.0)
        stage.update()
        corridor = Corridor("Corridor", mav, alt=3.0)
        mav._pos = (20.0, 0.0, 3.0)
        mav.exited = True
        self.assertIs(corridor.update(), py_trees.common.Status.SUCCESS)
        self.assertEqual(mav.corridor_exit_pose[:2], (20.0, 0.0))


class StartInsideMarginTests(unittest.TestCase):
    """Live run 5, shipped arena: the only red-zone entry of a completed run.

    The aircraft was inside the inflated margin of the main zone's south
    strip (confirmed mid-leg) and heading for the next lane at (47.8, 2.0).
    route_leg dropped the box it was inside and returned the straight line,
    which ran diagonally across the paint. Leaving has to be by the nearest
    edge, and the rest of the leg routed with the box still in play.
    """

    # Main red zone, home-local, as the camera had mapped it (1 m cells).
    RED = (35.0, 45.0, -0.5, 6.5)

    def test_the_route_never_crosses_the_zone_it_starts_beside(self):
        from mission_bt.search_planner import route_leg, _segment_hits_rect
        start = (34.9, -0.6)          # inside the 1.5 m margin, outside paint
        end = (47.8, 2.0)
        r = route_leg(start, end, 1.5, [self.RED])
        self.assertTrue(r["ok"], r["reason"])
        pts = [start] + list(r["waypoints"])
        for a, b in zip(pts, pts[1:]):
            self.assertFalse(
                _segment_hits_rect(a[0], a[1], b[0], b[1], self.RED),
                f"leg {a} -> {b} crosses the red zone")
        # It steps OUT first: the first waypoint is no further inside.
        first = r["waypoints"][0]
        self.assertLessEqual(first[1], start[1] + 1e-9)
