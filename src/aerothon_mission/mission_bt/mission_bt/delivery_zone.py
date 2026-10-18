#!/usr/bin/env python3
"""Validate the organiser-supplied delivery-zone boundary.

The competition supplies four geographic corners.  The mission converts them
once about the FCU home and searches the resulting local ENU rectangle.  A
horizontal lidar free-space reading is deliberately absent from this module:
it says where nearby obstacles are, not where the payload field ends.
"""

import json
import math

from .geofence import global_to_local


def parse_boundary(payload):
    """Return ``([(lat, lon), ...], '')`` or ``(None, reason)``."""
    if not payload or not str(payload).strip():
        return None, "delivery-zone boundary is missing"
    try:
        value = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, ValueError, json.JSONDecodeError):
        return None, "delivery-zone boundary is not valid JSON"

    raw = value.get("vertices") if isinstance(value, dict) else None
    if not isinstance(raw, list) or len(raw) != 4:
        return None, "delivery-zone boundary must contain exactly four vertices"

    vertices = []
    for i, point in enumerate(raw):
        try:
            lat = float(point["lat"])
            lon = float(point["lon"])
        except (KeyError, TypeError, ValueError):
            return None, f"delivery-zone vertex {i + 1} needs numeric lat and lon"
        if not math.isfinite(lat) or not math.isfinite(lon):
            return None, f"delivery-zone vertex {i + 1} is not finite"
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            return None, f"delivery-zone vertex {i + 1} is outside WGS84 bounds"
        vertices.append((lat, lon))

    rounded = {(round(lat, 10), round(lon, 10)) for lat, lon in vertices}
    if len(rounded) != 4:
        return None, "delivery-zone boundary needs four distinct vertices"
    return vertices, ""


def parse_polygon(payload, what="geofence", min_vertices=3, max_vertices=64):
    """Return ``([(lat, lon), ...], '')`` or ``(None, reason)``.

    The arena geofence is a polygon, not necessarily a rectangle: the
    rulebook says only that its coordinates will be provided. Same JSON shape
    as the delivery-zone boundary, ``{"vertices": [{"lat":..,"lon":..}, ..]}``.
    """
    if not payload or not str(payload).strip():
        return None, f"{what} boundary is missing"
    try:
        value = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, ValueError, json.JSONDecodeError):
        return None, f"{what} boundary is not valid JSON"
    raw = value.get("vertices") if isinstance(value, dict) else None
    if not isinstance(raw, list) or not min_vertices <= len(raw) <= max_vertices:
        return None, (f"{what} boundary needs {min_vertices}..{max_vertices} "
                      "vertices")
    vertices = []
    for i, point in enumerate(raw):
        try:
            lat = float(point["lat"])
            lon = float(point["lon"])
        except (KeyError, TypeError, ValueError):
            return None, f"{what} vertex {i + 1} needs numeric lat and lon"
        if not (math.isfinite(lat) and math.isfinite(lon)
                and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return None, f"{what} vertex {i + 1} is not a valid WGS84 position"
        vertices.append((lat, lon))
    if len({(round(a, 10), round(b, 10)) for a, b in vertices}) != len(vertices):
        return None, f"{what} boundary has repeated vertices"
    return vertices, ""


def polygon_area(points):
    """Signed shoelace area of a local XY polygon (positive = CCW)."""
    area = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def point_in_polygon(x, y, points):
    """Ray-casting containment for a simple local XY polygon."""
    inside = False
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            cross = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < cross:
                inside = not inside
    return inside


def distance_to_polygon_edge(x, y, points):
    """Shortest distance from a point to any edge of the polygon."""
    best = float("inf")
    n = len(points)
    for i in range(n):
        ax, ay = points[i]
        bx, by = points[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def point_inside_with_margin(x, y, points, margin_m):
    return (point_in_polygon(x, y, points)
            and distance_to_polygon_edge(x, y, points) >= margin_m)


def polygon_to_local(vertices, home_lat, home_lon, min_area_m2=25.0):
    """WGS84 vertices -> local ENU points, or ``(None, reason)``."""
    if not vertices:
        return None, "geofence boundary is missing"
    if not (math.isfinite(home_lat) and math.isfinite(home_lon)):
        return None, "geofence needs a valid FCU home position"
    pts = [global_to_local(lat, lon, home_lat, home_lon) for lat, lon in vertices]
    if abs(polygon_area(pts)) < min_area_m2:
        return None, "geofence polygon encloses almost no area"
    return pts, ""


def boundary_to_local_zone(vertices, home_lat, home_lon,
                           corner_tolerance_m=0.75,
                           min_side_m=5.0, max_side_m=120.0):
    """Convert four WGS84 rectangle corners to ``(x0, x1, y0, y1)``.

    The selected search design is axis-aligned in local ENU.  Refuse a rotated
    or irregular polygon rather than searching its bounding box, which could
    place the aircraft outside the supplied geofence.
    """
    if vertices is None or len(vertices) != 4:
        return None, "delivery-zone boundary must contain four vertices"
    if not (math.isfinite(home_lat) and math.isfinite(home_lon)
            and -90.0 <= home_lat <= 90.0 and -180.0 <= home_lon <= 180.0):
        return None, "delivery-zone boundary needs a valid FCU home position"
    local = [global_to_local(lat, lon, home_lat, home_lon)
             for lat, lon in vertices]
    xs = [p[0] for p in local]
    ys = [p[1] for p in local]
    zone = (min(xs), max(xs), min(ys), max(ys))
    width, height = zone[1] - zone[0], zone[3] - zone[2]
    if width < min_side_m or height < min_side_m:
        return None, f"delivery-zone rectangle is too small ({width:.1f} x {height:.1f} m)"
    if width > max_side_m or height > max_side_m:
        return None, f"delivery-zone rectangle is too large ({width:.1f} x {height:.1f} m)"

    expected = {(zone[0], zone[2]), (zone[1], zone[2]),
                (zone[1], zone[3]), (zone[0], zone[3])}
    unmatched = set(expected)
    for x, y in local:
        nearest = min(unmatched, key=lambda p: math.hypot(x - p[0], y - p[1]),
                      default=None)
        if nearest is None or math.hypot(x - nearest[0], y - nearest[1]) > corner_tolerance_m:
            return None, "delivery-zone boundary is not an axis-aligned ENU rectangle"
        unmatched.remove(nearest)
    return zone, ""


def inset_zone(zone, clearance_m):
    """Rectangle the aircraft centre may occupy inside the supplied boundary."""
    x0, x1, y0, y1 = (float(v) for v in zone)
    c = float(clearance_m)
    result = (x0 + c, x1 - c, y0 + c, y1 - c)
    if result[0] >= result[1] or result[2] >= result[3]:
        raise ValueError(f"boundary has no room for {c:.1f} m clearance")
    return result


def nearest_point_in_zone(point, zone):
    """Clamp a local XY point to an axis-aligned closed rectangle."""
    x, y = point
    x0, x1, y0, y1 = zone
    return max(x0, min(x1, x)), max(y0, min(y1, y))


# ---- fence-only arenas ------------------------------------------------------
# The judges may give one polygon -- the geofence round the whole flying
# area, take-off, corridors and delivery field -- and no delivery-zone
# corners. The field is then the part of that polygon beyond the outbound
# corridor's exit, found in flight once the exit is known.

def clip_beyond(points, origin, heading_rad, margin_m=1.0):
    """The part of polygon `points` at least `margin_m` beyond the line
    through `origin` across `heading_rad` (Sutherland-Hodgman, one plane)."""
    hx, hy = math.cos(heading_rad), math.sin(heading_rad)
    ox, oy = origin

    def ahead(p):
        return (p[0] - ox) * hx + (p[1] - oy) * hy - margin_m

    out = []
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        da, db = ahead(a), ahead(b)
        if da >= 0:
            out.append(tuple(a))
        if (da >= 0) != (db >= 0):
            t = da / (da - db)
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def bounding_zone(points):
    """``(x0, x1, y0, y1)`` round a local XY polygon."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), max(xs), min(ys), max(ys))


def chord_inside(points, a, b, margin_m, step_m=0.25):
    """The longest stretch of segment a->b at least `margin_m` inside the
    polygon, as ``(start, end)``, or None. A sweep lane planned across the
    field's bounding box is trimmed to the fenced field this way."""
    length = math.dist(a, b)
    n = max(2, int(length / step_m) + 1)
    pts = [(a[0] + (b[0] - a[0]) * i / (n - 1), a[1] + (b[1] - a[1]) * i / (n - 1))
           for i in range(n)]
    best, run = None, []
    for p in pts + [None]:
        if p is not None and point_inside_with_margin(p[0], p[1], points, margin_m):
            run.append(p)
            continue
        if len(run) >= 2 and (best is None or math.dist(run[0], run[-1]) > math.dist(*best)):
            best = (run[0], run[-1])
        run = []
    return best


# WHAT THE JUDGES' POINTS ARE. The rulebook says "Delivery zone geo-fence
# coordinates will be provided to teams during Phase 2"; the team expects
# the points to enclose the whole flying area. Take-off decides: inside the
# polygon it is the arena fence, outside it is the delivery zone. Either way
# the operator types the same points once.
ROLE_ARENA = "arena"
ROLE_DELIVERY_ZONE = "delivery_zone"
# The fence the flight controller enforces when the points are the delivery
# zone: the zone and a square round take-off, hulled and grown outward. The
# corridors lie between the two; the margin covers side-by-side lanes.
SAFETY_FENCE_HOME_HALF_M = 5.0
SAFETY_FENCE_MARGIN_M = 10.0
# Sweep lanes inside an ARENA fence keep this far from its edge: the FC
# enforces that fence with FENCE_MARGIN 2 m, and Copter's fence avoidance
# brakes inside the margin. Inside a delivery zone 1 m suffices; its derived
# fence lies 10 m further out.
ARENA_FENCE_LANE_MARGIN_M = 3.0
ZONE_LANE_MARGIN_M = 1.0


def judges_polygon_role(points, home=(0.0, 0.0)):
    """ROLE_ARENA when take-off lies inside the polygon, else the zone."""
    return ROLE_ARENA if point_in_polygon(home[0], home[1], points) else ROLE_DELIVERY_ZONE


def convex_hull(points):
    """Counter-clockwise hull, Andrew's monotone chain."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def safety_fence(zone_points, home=(0.0, 0.0), home_half_m=SAFETY_FENCE_HOME_HALF_M,
                 margin_m=SAFETY_FENCE_MARGIN_M, sides=8):
    """Inclusion fence round a delivery-zone polygon and take-off, local XY.

    The hull of the zone and a square round take-off, grown by `margin_m`
    (an octagonal Minkowski sum, so every point of the hull is at least
    margin_m * cos(pi/sides) inside the result)."""
    hx, hy = home
    seed = list(zone_points) + [(hx + sx * home_half_m, hy + sy * home_half_m)
                                for sx in (-1, 1) for sy in (-1, 1)]
    r = margin_m / math.cos(math.pi / sides)
    grown = [(x + r * math.cos(2 * math.pi * k / sides + math.pi / sides),
              y + r * math.sin(2 * math.pi * k / sides + math.pi / sides))
             for x, y in convex_hull(seed) for k in range(sides)]
    return convex_hull(grown)
