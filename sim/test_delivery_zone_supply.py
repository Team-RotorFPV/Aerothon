#!/usr/bin/env python3
"""The simulated organiser must hand the mission a boundary it accepts.

`gcs_readiness` and the mission both refuse to arm without a four-corner
delivery-zone boundary, and nothing in the simulator supplied one, so every
simulated run stopped at not-ready. `scripts/publish_delivery_zone.py` plays the
organiser.

The claim under test is not "it publishes JSON" but "the mission's own
validator turns that JSON back into the arena rectangle". The validator rejects
corners that miss an axis-aligned ENU rectangle by more than 0.75 m, so a
boundary built with a slightly different earth model would be refused.
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "scripts"))
sys.path.insert(0, os.path.join(_ROOT, "src", "aerothon_mission", "mission_bt"))

from mission_bt.delivery_zone import boundary_to_local_zone, parse_boundary
import publish_delivery_zone as supply

# ArduPilot SITL's default home (CMAC), which the Gazebo iris spawns at.
HOME = (-35.363262, 149.165237)


class DeliveryZoneSupply(unittest.TestCase):
    def _round_trip(self, cx, cy, width, height, home=HOME):
        payload = supply.boundary_payload(cx, cy, width, height, *home)
        vertices, why = parse_boundary(payload)
        self.assertIsNotNone(vertices, f"mission rejected the payload: {why}")
        zone, why = boundary_to_local_zone(vertices, *home)
        self.assertIsNotNone(zone, f"mission rejected the geometry: {why}")
        return zone

    def test_shipped_arena_zone_round_trips_to_the_field(self):
        """The default is the shipped arena's delivery_zone_40x30, world
        (32, 0), expressed about the FCU home -- the vehicle spawns at world
        (-2, 2), so the field is at home-local (34, -2)."""
        zone = self._round_trip(*supply.DEFAULT_ZONE)
        x0, x1, y0, y1 = zone
        for got, want, label in ((x0, 14.0, "x0"), (x1, 54.0, "x1"),
                                 (y0, -17.0, "y0"), (y1, 13.0, "y1")):
            self.assertAlmostEqual(
                got, want, places=3,
                msg=f"{label} came back {got:.3f}, not the arena's {want}")

    def test_a_randomised_zone_centre_survives_the_round_trip(self):
        zone = self._round_trip(38.5, -6.25, 40.0, 30.0)
        self.assertAlmostEqual(zone[0], 18.5, places=3)
        self.assertAlmostEqual(zone[1], 58.5, places=3)
        self.assertAlmostEqual(zone[2], -21.25, places=3)
        self.assertAlmostEqual(zone[3], 8.75, places=3)

    def test_corners_are_distinct_and_four(self):
        payload = supply.boundary_payload(*supply.DEFAULT_ZONE, *HOME)
        vertices, why = parse_boundary(payload)
        self.assertEqual(len(vertices), 4, why)
        self.assertEqual(len(set(vertices)), 4, "duplicate corner emitted")

    def test_uninitialised_home_is_refused(self):
        """(0, 0) is MAVROS's default, not a position.

        A boundary georeferenced about null island is a perfectly valid
        rectangle, so the mission would accept it and search the Gulf of
        Guinea. The refusal has to happen here.
        """
        self.assertFalse(supply.home_is_usable(0.0, 0.0))
        self.assertFalse(supply.home_is_usable(float("nan"), 149.0))
        self.assertFalse(supply.home_is_usable(-35.363262, float("inf")))
        self.assertTrue(supply.home_is_usable(*HOME))

    def test_zone_override_is_parsed(self):
        self.assertEqual(
            supply.zone_from_env({"AEROTHON_DELIVERY_ZONE": "38.5,-6.25,40,30"}),
            (38.5, -6.25, 40.0, 30.0))
        self.assertEqual(supply.zone_from_env({}), supply.DEFAULT_ZONE)
        self.assertEqual(supply.zone_from_env({"AEROTHON_DELIVERY_ZONE": "  "}),
                         supply.DEFAULT_ZONE)

    def test_malformed_zone_override_fails_loudly(self):
        """A typo must stop the run, not silently fly the default arena."""
        for bad in ("32,0,40", "32,0,40,30,7", "32,0,zero,30",
                    "32,0,0,30", "32,0,40,-30", "32,0,nan,30"):
            with self.subTest(bad):
                with self.assertRaises(ValueError):
                    supply.zone_from_env({"AEROTHON_DELIVERY_ZONE": bad})


if __name__ == "__main__":
    unittest.main()
