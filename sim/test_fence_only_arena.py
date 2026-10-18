#!/usr/bin/env python3
"""A fence-only arena: the judges give six geofence points and nothing else.

The polygon surrounds take-off, both corridors and the delivery field. The
search area is its part beyond the outbound corridor's exit, known only in
flight. These pin the clipping, the lane trimming, the pre-flight check that
now accepts the fence alone, the readiness item, and that supplied
delivery-zone corners still take precedence.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_fence_only_arena.py -v
"""

import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_mission", "mission_bt"))

import py_trees                                                  # noqa: E402

from mission_bt.delivery_zone import (bounding_zone, chord_inside,  # noqa: E402
                                      clip_beyond, point_inside_with_margin,
                                      polygon_area)

# The judges' six points, local ENU: a start area joined by the corridor
# neck to a wider field -- the clarified rendered world's layout.
L_FENCE = [(-9.9, -10.0), (20.0, -10.0), (20.0, -26.4),
           (71.6, -26.4), (71.6, 15.6), (-9.9, 15.6)]
EXIT = (17.0, -3.5)


class ClipTests(unittest.TestCase):

    def test_the_field_is_the_fence_beyond_the_exit(self):
        field = clip_beyond(L_FENCE, EXIT, 0.0, 1.0)
        x0, x1, y0, y1 = bounding_zone(field)
        self.assertAlmostEqual(x0, 18.0)
        self.assertAlmostEqual(x1, 71.6)
        self.assertAlmostEqual(y0, -26.4)
        self.assertAlmostEqual(y1, 15.6)
        self.assertGreater(abs(polygon_area(field)), 1000.0)
        # Nothing behind the exit survives.
        self.assertTrue(all(p[0] >= 18.0 - 1e-9 for p in field))

    def test_a_lane_across_the_neck_is_trimmed_to_the_fence(self):
        field = clip_beyond(L_FENCE, EXIT, 0.0, 1.0)
        # Across the narrow part (x < 20) the fence only spans y -10..15.6.
        a, b = chord_inside(field, (19.0, -30.0), (19.0, 20.0), 1.0)
        self.assertGreaterEqual(min(a[1], b[1]), -9.0 - 0.3)
        self.assertLessEqual(max(a[1], b[1]), 14.6 + 0.3)
        for p in (a, b):
            self.assertTrue(point_inside_with_margin(p[0], p[1], field, 1.0))

    def test_a_lane_wholly_outside_is_dropped(self):
        field = clip_beyond(L_FENCE, EXIT, 0.0, 1.0)
        self.assertIsNone(chord_inside(field, (80.0, -5.0), (90.0, -5.0), 1.0))


def commander(fence, zone_vertices=None, exit_pose=None):
    from mission_bt.mav_commander import Mav
    mav = Mav.__new__(Mav)
    mav.geofence_local = fence
    mav.delivery_zone_global = zone_vertices
    mav.delivery_zone_local = None
    mav.delivery_zone_reason = "delivery-zone boundary is missing"
    mav.corridor_exit_pose = exit_pose
    mav.gate_heading = None
    mav._logged = set()
    mav.node = NS(get_logger=lambda: NS(info=lambda *a: None))
    return mav


class CommanderTests(unittest.TestCase):

    def test_before_the_corridor_there_is_no_search_area_yet(self):
        mav = commander(L_FENCE)
        self.assertTrue(mav.fence_only)
        self.assertIsNone(mav.delivery_search_zone(1.0))
        self.assertIn("corridor exit", mav.delivery_zone_reason)

    def test_after_the_exit_the_search_area_is_the_inset_field(self):
        mav = commander(L_FENCE, exit_pose=(EXIT[0], EXIT[1], 2.4, 0.0))
        zone = mav.delivery_search_zone(1.0)
        self.assertEqual(tuple(round(v, 2) for v in zone), (19.0, 70.6, -25.4, 14.6))
        self.assertIsNotNone(mav.search_polygon())

    def test_the_gate_heading_beats_a_yawed_exit(self):
        mav = commander(L_FENCE, exit_pose=(EXIT[0], EXIT[1], 2.4, 0.6))
        mav.gate_heading = 0.0
        self.assertAlmostEqual(mav.delivery_search_zone(0.0)[0], 18.0)

    def test_supplied_corners_still_win(self):
        mav = commander(L_FENCE, zone_vertices=[(0, 0)] * 4,
                        exit_pose=(EXIT[0], EXIT[1], 2.4, 0.0))
        mav.delivery_zone_local = (25.6, 65.6, -20.4, 9.6)
        self.assertFalse(mav.fence_only)
        self.assertIsNone(mav.search_polygon())
        self.assertEqual(mav.delivery_search_zone(1.0), (26.6, 64.6, -19.4, 8.6))


class PreflightTests(unittest.TestCase):

    def test_the_fence_alone_lets_the_mission_arm(self):
        from mission_bt.mission_tree import RequireDeliveryZone
        mav = commander(L_FENCE)
        leaf = RequireDeliveryZone(mav, 1.0)
        leaf.tick_once()
        self.assertEqual(leaf.status, py_trees.common.Status.SUCCESS)
        self.assertIn("geofence only", leaf.feedback_message)

    def test_no_fence_and_no_zone_still_blocks(self):
        from mission_bt.mission_tree import RequireDeliveryZone
        mav = commander(None)
        mav.abort_reason = ""
        leaf = RequireDeliveryZone(mav, 1.0)
        leaf.tick_once()
        self.assertEqual(leaf.status, py_trees.common.Status.FAILURE)


class LaneTrimTests(unittest.TestCase):

    def test_every_flown_lane_stays_inside_the_fenced_field(self):
        from mission_bt.mission_tree import LawnmowerSearch
        from test_fail_closed_stages import FakeMav
        fake = FakeMav()
        fake.exclusions = []
        fake._pos = (EXIT[0], EXIT[1], 10.0)
        fake.corridor_exit_pose = (EXIT[0], EXIT[1], 10.0, 0.0)
        fake.avoid_detail = {"open_depth_m": 12.0, "open_width_m": 16.6}
        # A notched field: the bounding box overhangs the fence at the top
        # left, which an untrimmed lane would fly into.
        notched = [(18.0, -26.4), (71.6, -26.4), (71.6, 15.6), (30.0, 15.6),
                   (30.0, 2.0), (18.0, 2.0)]
        fake.search_polygon = lambda: notched
        zone = bounding_zone(notched)
        leaf = LawnmowerSearch(fake, (zone[0] + 1, zone[1] - 1, zone[2] + 1, zone[3] - 1),
                               10.0, exclusions=lambda: [], search_budget_m=0.0)
        leaf.initialise()
        self.assertTrue(leaf.wps)
        for a, b in zip(leaf.wps[0::2], leaf.wps[1::2]):
            for t in (0.0, 0.25, 0.5, 0.75, 1.0):
                x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
                self.assertTrue(point_inside_with_margin(x, y, notched, 0.9),
                                f"lane point ({x:.1f}, {y:.1f}) leaves the field")


# The rulebook's reading: "Delivery zone geo-fence coordinates will be
# provided". Six points round the delivery field, take-off well outside.
ZONE6 = [(25.6, -20.4), (65.6, -20.4), (65.6, 4.0), (55.0, 9.6),
         (35.0, 9.6), (25.6, 2.0)]
HOME = (-35.3632135, 149.1651905)


def to_global(points):
    from mission_bt.geofence import local_to_global
    return [local_to_global(x, y, *HOME) for x, y in points]


class JudgesRoleTests(unittest.TestCase):

    def test_points_round_take_off_are_the_arena(self):
        from mission_bt.delivery_zone import ROLE_ARENA, judges_polygon_role
        self.assertEqual(judges_polygon_role(L_FENCE), ROLE_ARENA)

    def test_points_away_from_take_off_are_the_delivery_zone(self):
        from mission_bt.delivery_zone import ROLE_DELIVERY_ZONE, judges_polygon_role
        self.assertEqual(judges_polygon_role(ZONE6), ROLE_DELIVERY_ZONE)

    def test_the_safety_fence_holds_take_off_zone_and_corridors(self):
        from mission_bt.delivery_zone import safety_fence
        fence = safety_fence(ZONE6)
        self.assertLessEqual(len(fence), 64)
        self.assertTrue(point_inside_with_margin(0.0, 0.0, fence, 2.0))
        for x, y in ZONE6:
            self.assertTrue(point_inside_with_margin(x, y, fence, 9.0))
        # Both corridors of the clarified world, walls included.
        for p in [(6.0, -5.3), (16.0, -5.3), (16.6, -8.3), (8.0, -8.5), (17.0, 0.0)]:
            self.assertTrue(point_inside_with_margin(p[0], p[1], fence, 2.0), p)

    def test_the_commander_searches_the_zone_and_enforces_the_derived_fence(self):
        mav = commander(None)
        mav.home = NS(geo=NS(latitude=HOME[0], longitude=HOME[1]),
                      position=NS(x=0.0, y=0.0))
        mav.geofence_global = to_global(ZONE6)
        mav._resolve_geofence()
        self.assertTrue(mav.fence_only)
        self.assertIsNotNone(mav.judges_zone_local)
        # Known before take-off: no corridor exit needed.
        zone = mav.delivery_search_zone(1.0)
        self.assertEqual(tuple(round(v, 1) for v in zone), (26.6, 64.6, -19.4, 8.6))
        self.assertTrue(point_inside_with_margin(0.0, 0.0, mav.geofence_local, 2.0))
        self.assertIn("delivery zone", mav.geofence_reason)

    def test_an_arena_fence_is_used_unchanged(self):
        mav = commander(None)
        mav.home = NS(geo=NS(latitude=HOME[0], longitude=HOME[1]),
                      position=NS(x=0.0, y=0.0))
        mav.geofence_global = to_global(L_FENCE)
        mav._resolve_geofence()
        self.assertIsNone(mav.judges_zone_local)
        self.assertEqual(len(mav.geofence_local), 6)
        self.assertIsNone(mav.delivery_search_zone(1.0))   # needs the exit first

    def test_lanes_keep_clear_of_an_arena_fence_the_fc_enforces(self):
        mav = commander(None)
        mav.home = NS(geo=NS(latitude=HOME[0], longitude=HOME[1]),
                      position=NS(x=0.0, y=0.0))
        mav.geofence_global = to_global(L_FENCE)
        mav._resolve_geofence()
        self.assertEqual(mav.search_margin_m, 3.0)   # FENCE_MARGIN 2 m + 1 m
        mav.geofence_global = to_global(ZONE6)
        mav._resolve_geofence()
        self.assertEqual(mav.search_margin_m, 1.0)

    def test_the_fence_upload_accepts_the_derived_fence(self):
        from mission_bt.mission_tree import UploadArenaFence
        mav = commander(None)
        mav.home = NS(geo=NS(latitude=HOME[0], longitude=HOME[1]),
                      position=NS(x=0.0, y=0.0))
        mav.geofence_global = to_global(ZONE6)
        mav._resolve_geofence()
        ok, why = UploadArenaFence(mav)._geometry_ok(mav.geofence_local)
        self.assertTrue(ok, why)


class DeadlineTests(unittest.TestCase):
    """A sweep still searching when only the return lap's time is left
    stops, so the delivery is skipped and the return is flown in time."""

    def sweep(self, late):
        from mission_bt.mission_tree import LawnmowerSearch
        from test_fail_closed_stages import FakeMav
        fake = FakeMav()
        fake.exclusions = []
        fake.abort_reason = ""
        fake._pos = (30.0, 0.0, 10.0)
        fake.avoid_detail = {"open_depth_m": 12.0, "open_width_m": 16.6}
        leaf = LawnmowerSearch(fake, (26.6, 64.6, -19.4, 8.6), 10.0,
                               exclusions=lambda: [], search_budget_m=0.0,
                               overdue=lambda: late)
        leaf.initialise()
        return fake, leaf

    def test_past_the_deadline_the_sweep_fails_with_its_reason(self):
        fake, leaf = self.sweep(12.0)
        self.assertEqual(leaf.update(), py_trees.common.Status.FAILURE)
        self.assertIn("search deadline", fake.abort_reason)

    def test_before_the_deadline_the_sweep_flies_on(self):
        fake, leaf = self.sweep(-30.0)
        self.assertNotEqual(leaf.update(), py_trees.common.Status.FAILURE)

    def test_a_match_at_the_deadline_is_still_delivered(self):
        fake, leaf = self.sweep(12.0)
        fake.qr_matched = True
        fake.target_xy = (40.0, 0.0)
        self.assertEqual(leaf.update(), py_trees.common.Status.SUCCESS)

    def test_overdue_counts_from_arming_against_limit_less_reserve(self):
        from mission_bt.mission_tree import search_overdue
        p = {"mission_limit_s": 900.0, "return_reserve_s": 300.0}
        self.assertIsNone(search_overdue(NS(armed_at=None), lambda: 50.0, p))
        self.assertAlmostEqual(search_overdue(NS(armed_at=100.0), lambda: 650.0, p), -50.0)
        self.assertAlmostEqual(search_overdue(NS(armed_at=100.0), lambda: 720.0, p), 20.0)
        # The team's 300 s extension moves the deadline to 900 s after arming.
        p["search_deadline_extension_s"] = 300.0
        self.assertAlmostEqual(search_overdue(NS(armed_at=100.0), lambda: 720.0, p), -280.0)
        self.assertAlmostEqual(search_overdue(NS(armed_at=100.0), lambda: 1010.0, p), 10.0)

    def test_the_skip_flies_the_return_and_records_why(self):
        from mission_bt.mission_tree import SkipDeliveryAndReturn
        fake, leaf = self.sweep(12.0)
        leaf.update()
        fake.enable_avoidance = lambda on: None
        fake.log = lambda *a, **k: None
        skip = SkipDeliveryAndReturn(fake)
        self.assertEqual(skip.update(), py_trees.common.Status.SUCCESS)
        self.assertIn("search deadline", fake.delivery_skipped)


class ArenaOverlapTests(unittest.TestCase):
    """In the arena reading the field beyond the corridor is bigger than the
    delivery zone; lanes there are 20% overlapped instead of 30%."""

    def leaf(self, arena):
        from mission_bt.mission_tree import LawnmowerSearch
        from test_fail_closed_stages import FakeMav
        fake = FakeMav()
        fake.exclusions = []
        fake.arena_search = arena
        fake._pos = (20.0, -3.5, 10.0)
        fake.avoid_detail = {"open_depth_m": 12.0, "open_width_m": 16.6}
        leaf = LawnmowerSearch(fake, (20.6, 70.6, -25.4, 14.4), 10.0,
                               hfov_rad=0.851919, marker_m=1.0,
                               exclusions=lambda: [], search_budget_m=0.0,
                               arena_overlap=0.20)
        leaf.initialise()
        return leaf

    def test_the_arena_field_flies_fewer_wider_lanes(self):
        arena, zone = self.leaf(True), self.leaf(False)
        self.assertLess(arena.plan["n_lanes"], zone.plan["n_lanes"])
        self.assertEqual(arena.plan["n_lanes"], 6)
        self.assertEqual(zone.plan["n_lanes"], 7)

    def test_a_pad_stays_wholly_in_frame_between_arena_lanes(self):
        plan = self.leaf(True).plan
        half_gap = plan["lane_spacing_m"] / 2.0
        self.assertGreaterEqual(plan["swath_m"] / 2.0 - (half_gap + 0.5), 0.35)

    def test_the_arena_sweep_starts_at_the_end_nearer_the_aircraft(self):
        leaf = self.leaf(True)                 # aircraft at y = -3.5, mid-field
        top = max(w[1] for w in leaf.wps)
        self.assertTrue(leaf._descending)
        self.assertAlmostEqual(leaf.wps[0][1], top)
        self.assertLess(math.dist(leaf.wps[0], (20.0, -3.5)),
                        math.dist(leaf.wps[-1], (20.0, -3.5)))
        # Still one continuous boustrophedon: lane by lane, top to bottom.
        lanes = [leaf.wps[k][1] for k in range(0, len(leaf.wps), 2)]
        self.assertEqual(lanes, sorted(lanes, reverse=True))

    def test_a_descending_sweep_resumes_on_the_next_lane_down(self):
        leaf = self.leaf(True)
        lanes = sorted({round(w[1], 3) for w in leaf.wps}, reverse=True)
        k = leaf._resume_index(lanes[1], pos=(leaf.wps[1][0], lanes[0]))  # end of lane 1
        self.assertAlmostEqual(leaf.wps[k][1], lanes[1])

    def test_a_zone_sweep_keeps_its_validated_order(self):
        leaf = self.leaf(False)
        self.assertFalse(leaf._descending)
        self.assertEqual(leaf.wps[0][1], min(w[1] for w in leaf.wps))

    def test_the_commander_flags_only_the_arena_reading(self):
        mav = commander(None)
        mav.home = NS(geo=NS(latitude=HOME[0], longitude=HOME[1]),
                      position=NS(x=0.0, y=0.0))
        mav.geofence_global = to_global(L_FENCE)
        mav._resolve_geofence()
        self.assertTrue(mav.arena_search)
        mav.geofence_global = to_global(ZONE6)
        mav._resolve_geofence()
        self.assertFalse(mav.arena_search)


class ReadinessTests(unittest.TestCase):

    def test_delivery_zone_points_pass_readiness(self):
        from gcs_aggregator.readiness_node import Readiness
        fake = NS(obs={}, fence_vertices=to_global(ZONE6), delivery_vertices=None,
                  fence_parse_reason="", home=NS(geo=NS(latitude=HOME[0], longitude=HOME[1])),
                  delivery_parse_reason="delivery-zone boundary is missing")
        fake._resolve_delivery_zone = lambda: Readiness._resolve_delivery_zone(fake)
        Readiness._resolve_geofence(fake)
        self.assertTrue(fake.obs["geofence_valid"])
        self.assertIn("derived", fake.obs["geofence_value"])
        self.assertTrue(fake.obs["delivery_zone_valid"])
        self.assertIn("delivery zone", fake.obs["delivery_zone_value"])

    def test_a_valid_fence_satisfies_the_search_area_item(self):
        from gcs_aggregator.readiness_node import Readiness
        fake = NS(obs={"geofence_valid": True}, delivery_vertices=None,
                  delivery_parse_reason="delivery-zone boundary is missing", home=None)
        Readiness._resolve_delivery_zone(fake)
        self.assertTrue(fake.obs["delivery_zone_valid"])
        self.assertIn("fenced field", fake.obs["delivery_zone_value"])
        fake = NS(obs={"geofence_valid": False}, delivery_vertices=None,
                  delivery_parse_reason="delivery-zone boundary is missing", home=None)
        Readiness._resolve_delivery_zone(fake)
        self.assertFalse(fake.obs["delivery_zone_valid"])



class FailedRecoveryTests(unittest.TestCase):
    """A failed mission still armed in GUIDED is sent home again and again."""

    def mav(self, armed=True, mode="GUIDED", connected=True):
        calls = []
        st = NS(connected=connected, armed=armed, mode=mode)
        m = NS(mission_failed=True, state=st, _fcu_state=NS(connected=True, armed=armed, mode=mode),
               alt=lambda: 1.2, land=lambda: calls.append("LAND"),
               set_mode=lambda md: calls.append(md), log=lambda *a, **k: None)
        return m, calls

    def test_a_hovering_failed_mission_is_landed_and_retried(self):
        from mission_bt.mission_tree import ensure_failed_recovery
        m, calls = self.mav()
        self.assertEqual(ensure_failed_recovery(m, 10.0), "LAND")
        self.assertIsNone(ensure_failed_recovery(m, 11.0))      # spaced
        self.assertEqual(ensure_failed_recovery(m, 12.5), "LAND")
        self.assertEqual(calls, ["LAND", "LAND"])

    def test_a_stale_state_uses_the_last_fcu_report(self):
        from mission_bt.mission_tree import ensure_failed_recovery
        m, calls = self.mav()
        m.state = NS(connected=False, armed=False, mode="")    # MAVROS defaults on a gap
        self.assertEqual(ensure_failed_recovery(m, 10.0), "LAND")

    def test_nothing_is_sent_once_the_fc_has_left_guided_or_disarmed(self):
        from mission_bt.mission_tree import ensure_failed_recovery
        for kw in ({"mode": "LAND"}, {"mode": "RTL"}, {"armed": False}):
            m, calls = self.mav(**kw)
            self.assertIsNone(ensure_failed_recovery(m, 10.0))
            self.assertEqual(calls, [])

    def test_a_healthy_mission_is_left_alone(self):
        from mission_bt.mission_tree import ensure_failed_recovery
        m, calls = self.mav()
        m.mission_failed = False
        self.assertIsNone(ensure_failed_recovery(m, 10.0))


if __name__ == "__main__":
    unittest.main()
