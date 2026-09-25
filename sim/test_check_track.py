#!/usr/bin/env python3
"""The track grader catches what the rulebook scores.

A grader that passes everything is worse than none: it turns a red-zone
overflight into a green tick. Each check here is shown to FAIL on a track
built to violate it.

    python3 -m pytest sim/test_check_track.py -v
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_track import grade  # noqa: E402

LAYOUT = {
    "home_world": [-2.0, 2.0],
    "delivery_zone_rect": [34.0, -2.0, 40.0, 30.0],     # home-local
    "geofence_rect": [-7.5, 60.0, -23.0, 19.0],          # home-local
    "red_zones": {"restricted_red_zone_main": [38.0, 5.0]},  # world, 10 x 7
    "pads": {"b": [47.0, 10.0]},                          # world
}


def track(points, state="SEARCH_QR"):
    out = []
    for i, (x, y, z, *rest) in enumerate(points):
        out.append({"t": float(i), "x": x, "y": y, "z": z, "armed": True,
                    "state": rest[0] if rest else state,
                    "roll": 2.0, "pitch": -3.0, "mode": "GUIDED"})
    return out


# Shipped corridor, home-local (home is world (-2, 2)): forward lane
# y -1.75..1.75, return lane y -5.75..-2.25, walls x 3.9..14.1.
OUTBOUND = [(1.5, 0, 3), (4.5, 0, 1.9), (8, 0.3, 3), (12, -0.2, 3),
            (13.8, 0, 3), (16, 0, 3)]
RETURN = [(17, -4, 3), (13.8, -4, 1.9), (10, -4.5, 3), (7, -3.3, 3),
          (4.2, -4, 3), (2, -4, 3)]


class GradeTests(unittest.TestCase):

    def _clean(self):
        # Take off, fly THROUGH the forward lane, sweep the south of the
        # zone, drop over pad B (home-local (49, 8)), back THROUGH the return
        # lane, come home and land (disarmed last sample).
        pts = ([(0, 0, 5)] + OUTBOUND +
               [(20, -12, 10), (50, -12, 10), (49, 8, 10),
                (49, 8, 5, "WINCH_DROP"), (49, 8, 5, "WINCH_DROP"),
                (20, -12, 10)] + RETURN + [(0.3, 0.2, 0.2, "LAND")])
        t = track(pts)
        t[-1]["armed"] = False
        return t

    def test_flying_OVER_the_corridor_fails(self):
        t = self._clean()
        for p in t:
            if 3.9 <= p["x"] <= 14.1 and abs(p["y"]) < 2.0:
                p["z"] = 5.0
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["outbound lane flown THROUGH the corridor"])
        self.assertFalse(res["checks"]["never above the corridor"])

    def test_flying_BESIDE_the_corridor_fails(self):
        t = self._clean()
        for p in t:
            if p["x"] >= 1.5 and p["x"] <= 16 and abs(p["y"]) <= 0.5:
                p["y"] = 6.0             # outside the walls, north
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["outbound lane flown THROUGH the corridor"])

    def test_a_collision_tilt_fails_even_if_the_rest_is_clean(self):
        t = self._clean()
        t[len(t) // 2]["pitch"] = 47.5
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["no tilt beyond 30 deg"])

    def test_an_RTL_failsafe_fails(self):
        t = self._clean()
        t[5]["mode"] = "RTL"
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["no failsafe / unexpected flight mode"])

    def test_a_track_with_no_attitude_cannot_pass(self):
        t = self._clean()
        for p in t:
            p.pop("roll"), p.pop("pitch")
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["no tilt beyond 30 deg"])

    def test_a_clean_flight_passes(self):
        res = grade(self._clean(), LAYOUT, target="b")
        self.assertTrue(res["pass"], res["checks"])
        self.assertLess(res["drop_error_m"], 0.1)

    def test_flying_over_red_ground_is_an_entry(self):
        t = self._clean()
        # Red main is world x 33..43, y 1.5..8.5 -> home-local x 35..45,
        # y -0.5..6.5. Fly straight across it.
        t.insert(3, {"t": 2.5, "x": 40.0, "y": 3.0, "z": 10.0, "armed": True,
                     "state": "SEARCH_QR"})
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["no red-zone entry"])
        self.assertEqual(len(res["red_zone_entries"]), 1)

    def test_leaving_the_geofence_fails(self):
        t = self._clean()
        t.insert(2, {"t": 1.5, "x": 70.0, "y": -12.0, "z": 10.0,
                     "armed": True, "state": "SEARCH_QR"})
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["inside geofence"])

    def test_sweeping_above_the_ceiling_fails(self):
        t = self._clean()
        next(p for p in t if p["x"] == 50)["z"] = 14.0     # over the zone
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["ceiling over zone"])

    def test_a_drop_off_the_pad_fails(self):
        t = self._clean()
        for p in t:
            if p["state"] == "WINCH_DROP":
                p["x"] += 3.0
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["drop within 1 m of pad"])

    def test_landing_away_from_home_fails(self):
        t = self._clean()
        t[-1]["x"] = 8.0
        res = grade(t, LAYOUT, target="b")
        self.assertFalse(res["checks"]["landed at take-off point"])


if __name__ == "__main__":
    unittest.main()
