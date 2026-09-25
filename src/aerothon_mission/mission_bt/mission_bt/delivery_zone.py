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
