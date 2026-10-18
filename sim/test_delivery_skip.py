#!/usr/bin/env python3
"""A delivery search that fails before the winch moves still flies home.

The return lap, landing and time are 40 rulebook marks that do not depend on
the payload. A sweep with no match, a pad on confirmed red ground or a pad
the aircraft cannot centre on used to fail the mission into an FC recovery
that never flies the corridor. These pin that such a failure is recorded and
the mission continues, that the drop is then not attempted, and that a winch
failure is NOT swallowed (a payload may be hanging).

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_delivery_skip.py -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "aerothon_mission", "mission_bt"))

import py_trees                                              # noqa: E402

from mission_bt.mission_tree import (                       # noqa: E402
    DeliverySkipped, Land, SkipDeliveryAndReturn, build_root)
import test_behavior_tree                                     # noqa: E402
from test_fail_closed_stages import FakeMav                  # noqa: E402

S = py_trees.common.Status


class Fixed(py_trees.behaviour.Behaviour):
    def __init__(self, name, status, mav=None, reason=""):
        super().__init__(name)
        self.status_out, self.mav, self.reason, self.ticks = status, mav, reason, 0

    def update(self):
        self.ticks += 1
        if self.mav is not None and self.reason:
            self.mav.abort_reason = self.reason
        return self.status_out


def logging_mav():
    mav = FakeMav()
    mav.log = lambda msg, warn=False: mav.logs.append(msg)
    mav.delivery_skipped = None
    mav.delivery_confirmed = None
    return mav


class SkipTests(unittest.TestCase):

    def test_a_failed_search_is_recorded_and_the_mission_continues(self):
        mav = logging_mav()
        mav.avoidance = True
        search = py_trees.composites.Selector(name="DeliverySearch", memory=True)
        search.add_children([
            Fixed("LawnmowerSearch", S.FAILURE, mav, "sweep found no matching pad"),
            SkipDeliveryAndReturn(mav)])
        search.tick_once()
        self.assertEqual(search.status, S.SUCCESS)
        self.assertEqual(mav.delivery_skipped, "sweep found no matching pad")
        self.assertFalse(mav.delivery_confirmed)
        self.assertEqual(mav.abort_reason, "", "a skip is not an abort")
        self.assertFalse(mav.avoidance)

    def test_the_drop_is_not_attempted_after_a_skip(self):
        mav = logging_mav()
        mav.delivery_skipped = "pad on red ground"
        winch = Fixed("WinchDrop", S.SUCCESS)
        drop = py_trees.composites.Selector(name="DropUnlessSkipped", memory=True)
        drop.add_children([DeliverySkipped(mav), winch])
        drop.tick_once()
        self.assertEqual(drop.status, S.SUCCESS)
        self.assertEqual(winch.ticks, 0)

    def test_a_winch_failure_still_fails_the_mission(self):
        mav = logging_mav()
        winch = Fixed("WinchDrop", S.FAILURE)
        drop = py_trees.composites.Selector(name="DropUnlessSkipped", memory=True)
        drop.add_children([DeliverySkipped(mav), winch])
        drop.tick_once()
        self.assertEqual(drop.status, S.FAILURE)
        self.assertEqual(winch.ticks, 1)

    def test_the_landing_reports_the_skip(self):
        mav = logging_mav()
        mav.delivery_skipped = "pad on red ground"
        mav.delivery_confirmed = False
        mav.state.armed = False
        mav.connected = lambda: True
        results = []
        mav.publish_result = lambda state, reason="": results.append((state, reason))
        mav.land = lambda: None
        mav.expect_disarm = lambda value=True: None
        mav.clear_airborne_floor = lambda: None
        land = Land(mav)
        land.initialise()
        for _ in range(5):
            if land.update() == S.SUCCESS:
                break
        self.assertEqual(results[0][0], "DELIVERY_SKIPPED")
        self.assertIn("pad on red ground", results[0][1])


class TreeShapeTests(unittest.TestCase):

    def setUp(self):
        # The full tree, built as test_behavior_tree builds it (not imported
        # as a class, which would collect its tests twice).
        test_behavior_tree.TestMissionBT.setUp(self)

    def test_only_the_find_stages_can_be_skipped(self):
        root = build_root(self.mav, self.node, self.defaults)
        mission = [c for c in root.children if c.name == "Mission"][0]
        names = [c.name for c in mission.children]
        search = mission.children[names.index("DeliverySearch")]
        self.assertEqual([c.name for c in search.children],
                         ["DeliveryFind", "SkipDeliveryAndReturn"])
        find = [c.name for c in search.children[0].children]
        self.assertIn("DescendToDecode", find)
        self.assertIn("CenterOnTarget", find)
        self.assertNotIn("WinchDrop", find)
        drop = mission.children[names.index("DropUnlessSkipped")]
        self.assertEqual([c.name for c in drop.children], ["DeliverySkipped", "WinchDrop"])
        self.assertLess(names.index("DeliverySearch"), names.index("DropUnlessSkipped"))


if __name__ == "__main__":
    unittest.main()
