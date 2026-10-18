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

import math

from .search_planner import (route_leg, path_hits_exclusion, routing_obstacles,
                             point_in_exclusion)

RUNNING = "RUNNING"
ARRIVED = "ARRIVED"
BLOCKED = "BLOCKED"


class LegRouter:
    """One leg's worth of routing state. Own one per stage."""

    def __init__(self, clearance_m=1.5, tol=0.6, waypoint_tol=None,
                 lead_m=0.0):
        self.clearance_m = float(clearance_m)
        self.tol = float(tol)
        # FLY THROUGH THE CORNERS. A position target is a place to stop at:
        # ArduPilot brakes to every one, so a detour of four legs was four
        # stops, and a quarter of the delivery sweep was spent nearly
        # stationary (my_world). With `lead_m` > 0 the command is a point that
        # far along the route -- past the corner, onto the next leg -- so the
        # aircraft only brakes for the real end of the leg. The chord to it is
        # checked against the same inflated zones, or the corner is used.
        self.lead_m = float(lead_m)
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
        key = (round(float(x), 2), round(float(y), 2),tuple(sorted(tuple(r) for r in ex)))
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

    def _carrot(self, mav, wx, wy):
        """The point `lead_m` ahead along the route from here, or the corner."""
        if self.lead_m <= 0.0 or self._i >= len(self._wps) - 1:
            return wx, wy
        px, py = mav.pos()[:2]
        # Finish the planned escape before applying look-ahead. Overlapping
        # observed margins may require several consecutive escape waypoints.
        blocks = routing_obstacles(self._exclusions(mav), self.clearance_m)
        if any(x0 <= px <= x1 and y0 <= py <= y1 for x0,x1,y0,y1 in blocks):
            return wx, wy
        left = self.lead_m - math.hypot(wx - px, wy - py)
        if left <= 0.0:
            return wx, wy
        ax, ay = wx, wy
        for bx, by in self._wps[self._i + 1:]:
            seg = math.hypot(bx - ax, by - ay)
            if seg >= left:
                k = left / seg
                ax, ay = ax + k * (bx - ax), ay + k * (by - ay)
                break
            left -= seg
            ax, ay = bx, by
        blocks = routing_obstacles(self._exclusions(mav), self.clearance_m)
        if blocks and path_hits_exclusion([(px, py), (ax, ay)], 0.0, blocks):
            return wx, wy
        return ax, ay

    # ---- the tick ---- #
    def target(self, mav, x, y, z):
        """Certified active segment endpoint, before a stage turns or moves."""
        if not self._plan(mav, x, y):
            return None

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
        cx, cy = self._carrot(mav, wx, wy)
        if not self._segment_safe(mav,(cx,cy)):
            # Aircraft drift and changed exclusions both invalidate a cached
            # segment. Recompute from measured position before commanding it.
            self._key = None
            if not self._plan(mav,x,y):
                return None
            wx,wy = self._wps[self._i]
            cx,cy = self._carrot(mav,wx,wy)
            if not self._segment_safe(mav,(cx,cy)):
                self.blocked_reason = 'active segment violates exclusion clearance'
                return None
        return cx,cy,wx,wy

    def fly(self, mav, x, y, z, yaw=0.0):
        """Command one tick of this leg. RUNNING / ARRIVED / BLOCKED."""
        target=self.target(mav,x,y,z)
        if target is None:
            self._hold(mav,yaw)
            return BLOCKED
        cx,cy,wx,wy=target
        mav.goto(cx, cy, z, yaw)
        last = self._i >= len(self._wps) - 1
        if last and mav.reached(wx, wy, z, self.tol):
            return ARRIVED
        return RUNNING

    @staticmethod
    def _hold(mav,yaw):
        fn=getattr(mav,'hold_position',None)
        if callable(fn):
            fn()

    def _segment_safe(self,mav,end,exclusions=None):
        px,py=mav.pos()[:2]
        raw=self._exclusions(mav) if exclusions is None else list(exclusions)
        if (not all(math.isfinite(v) for v in (px,py,*end))
                or any(len(rect)!=4 or not all(math.isfinite(v) for v in rect)
                       for rect in raw)):
            return False
        # Bounding boxes simplify route planning, but their added empty ground
        # is not an observed exclusion. An already issued segment may continue
        # only when both endpoints and its whole line clear EVERY original
        # observed rectangle inflated by the unchanged clearance. Starts inside
        # an actual margin still require the restricted nearest-edge escape.
        if (not point_in_exclusion(px,py,self.clearance_m,raw)
                and not point_in_exclusion(*end,self.clearance_m,raw)
                and not path_hits_exclusion([(px,py),end],self.clearance_m,raw)):
            return True
        blocks=routing_obstacles(raw,self.clearance_m)
        containing=[r for r in blocks if r[0]<=px<=r[1] and r[2]<=py<=r[3]]
        if not containing:
            return not path_hits_exclusion([(px,py),end],0,blocks)
        # A zone discovered inside the braking margin is already a clearance
        # failure. Permit only the planner's nearest-edge escape, never a
        # cached flight through the paint or a new zone.
        # route_leg leaves the first containing margin, then replans the
        # others. Their nearest exits can differ; requiring one command to
        # equal every exit strands the aircraft. Paint and other obstacles
        # are still checked below before accepting this escape segment.
        for x0,x1,y0,y1 in containing[:1]:
            candidates=[(x0-.25,py),(x1+.25,py),(px,y0-.25),(px,y1+.25)]
            nearest=min(candidates,key=lambda p:math.dist((px,py),p))
            if math.dist(end,nearest)>.01:
                # A streamed escape remains aimed at the same exit edge
                # while small cross-track motion changes its exact endpoint.
                # Permit continuation only through that nearest edge and
                # only when distance to paint in each containing margin is
                # nondecreasing along the whole straight segment. Squared
                # distance to a convex rectangle is convex; its initial
                # directional derivative suffices for that condition.
                axis=0 if candidates.index(nearest)<2 else 1
                if abs(end[axis]-nearest[axis])>.01:
                    return False
                vx,vy=end[0]-px,end[1]-py
                for rx0,rx1,ry0,ry1 in raw:
                    margin=self.clearance_m
                    if not (rx0-margin<=px<=rx1+margin and
                            ry0-margin<=py<=ry1+margin):
                        # Other zones retain the full inflated segment
                        # check below. Approaching a distant zone without
                        # entering its margin cannot invalidate this escape.
                        continue
                    dx=px-min(max(px,rx0),rx1)
                    dy=py-min(max(py,ry0),ry1)
                    if dx*vx+dy*vy < -1e-9:
                        return False
        other=[r for r in blocks if r not in containing]
        paint=[r for r in raw if not (r[0]<=px<=r[1] and r[2]<=py<=r[3])]
        return not (path_hits_exclusion([(px,py),end],0,other) or
                    path_hits_exclusion([(px,py),end],0,paint))
