#!/usr/bin/env python3
"""Fly one leg, around confirmed red ground, across many behaviour-tree ticks.

WHY THIS EXISTS

    Every transit stage in the tree commanded `mav.goto(x, y, z, yaw)` — a
    straight line to a destination — and not one of them asked whether that
    line crossed a restricted zone. The search sweep clipped its lanes and
    nothing else clipped anything.

    A watched flight confirmed 198 exclusion cells during the nadir sweep and
    the aircraft flew over red ground regardless. The detector was working the
    whole time; nothing downstream of the sweep was asking it. Restricted-zone
    avoidance is 10 marks, minus 5 per violation.

WHY IT IS A SHARED OBJECT RATHER THAN A METHOD ON Mav

    The tests need a vehicle they can drive. If the routing lived on Mav, the
    fake vehicle in the test suite would need its own copy of it, and the
    tests would then be grading the copy. That is not hypothetical here: the
    fake vehicle once hardcoded the QR offset's validity flag to zero, which
    made every target permanently invisible and quietly voided every test that
    depended on it.

    So the router is one object, used by the real stages and by the tests, and
    it asks the vehicle for only four things: `pos()`, `reached()`, `goto()`
    and an `exclusions` list.

WHAT IT DOES NOT DO

    It does not fall back to the straight line. A leg that cannot be flown
    legally is BLOCKED, with a reason the stage turns into an abort. Flying it
    anyway and trusting the ArduPilot exclusion fence to intervene would be
    scoring a violation on purpose and calling it defence in depth.
"""

from .search_planner import route_leg, path_hits_exclusion, routing_obstacles

RUNNING = "RUNNING"
ARRIVED = "ARRIVED"
BLOCKED = "BLOCKED"


class LegRouter:
    """One leg's worth of routing state. Own one per stage."""

    def __init__(self, clearance_m=1.5, tol=0.6, waypoint_tol=None):
        self.clearance_m = float(clearance_m)
        self.tol = float(tol)
        # Intermediate corners are steering hints, not places to be precise
        # about. Holding them to the arrival tolerance makes the aircraft
        # settle on each one and spends the mission clock, which is 15 marks
        # of its own.
        self.waypoint_tol = float(waypoint_tol if waypoint_tol is not None
                                  else max(1.5, tol * 2.5))
        self.blocked_reason = ""
        self._key = None
        self._wps = []
        self._i = 0
        # A refused leg has to STAY refused for as long as nothing has
        # changed. The cache is keyed on "same destination, same zones", and
        # without this the second tick of a blocked leg took the cache hit
        # into an empty waypoint list. A stage that reports FAILURE can still
        # be re-entered by the tree, so that second tick does happen.
        self._blocked = False

    def reset(self):
        self.blocked_reason = ""
        self._key = None
        self._wps = []
        self._i = 0
        self._blocked = False

    # ---- internals ---- #
    @staticmethod
    def _exclusions(mav):
        return list(getattr(mav, "exclusions", None) or [])

    def _plan(self, mav, x, y):
        """(Re)compute the route when the destination or the zones change."""
        ex = self._exclusions(mav)
        key = (round(float(x), 2), round(float(y), 2), len(ex))
        if key == self._key:
            return not self._blocked
        self._key = key
        self._blocked = False
        if not ex:
            self._wps = [(float(x), float(y))]
            self._i = 0
            self.blocked_reason = ""
            return True

        result = route_leg(mav.pos()[:2], (float(x), float(y)),
                           self.clearance_m, ex)
        if not result["ok"]:
            self._wps = []
            self._i = 0
            self._blocked = True
            self.blocked_reason = result["reason"]
            return False
        self._wps = list(result["waypoints"])
        self._i = 0
        self.blocked_reason = ""
        if result["detoured"]:
            mav.log(f"routing around {len(ex)} confirmed red zone(s): "
                    f"{len(self._wps)} leg(s), {result['length_m']:.0f} m "
                    f"to ({float(x):.1f}, {float(y):.1f})")
        return True

    # ---- the tick ---- #
    def fly(self, mav, x, y, z, yaw=0.0):
        """Command one tick of this leg. RUNNING / ARRIVED / BLOCKED."""
        if not self._plan(mav, x, y):
            return BLOCKED

        while self._i < len(self._wps) - 1:
            wx, wy = self._wps[self._i]
            if mav.reached(wx, wy, z, self.waypoint_tol):
                # Being near a corner does not mean it is safe to cut it.
                # Skip the waypoint only when the next commanded segment
                # still clears the same inflated obstacles used in planning.
                blocks = routing_obstacles(self._exclusions(mav), self.clearance_m)
                if path_hits_exclusion(
                        [mav.pos()[:2], self._wps[self._i + 1]], 0.0, blocks):
                    break
                self._i += 1
            else:
                break

        wx, wy = self._wps[self._i]
        mav.goto(wx, wy, z, yaw)
        last = self._i >= len(self._wps) - 1
        if last and mav.reached(wx, wy, z, self.tol):
            return ARRIVED
        return RUNNING
