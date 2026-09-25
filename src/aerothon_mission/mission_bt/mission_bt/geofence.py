#!/usr/bin/env python3
"""Building an ArduPilot geofence from what perception observed.

WHY THE FENCE IS THE AUTHORITATIVE BOUNDARY

    Red-zone avoidance in the search planner is a PLAN: it routes lanes around
    ground the camera mapped as red. Plans are wrong when perception is wrong,
    when the aircraft drifts, when a gust pushes it, or when an operator takes
    manual control. The fence is enforced by the flight controller below every
    one of those, which is why goal.md Q13/Q24 make it the backstop and why
    the camera layer is explicitly demoted to supplementary.

    A fence that was uploaded but not verified is worse than none, because it
    is believed. Everything here is built so that upload can be READ BACK and
    compared, not merely acknowledged.

FRAMES

    The mission works in the local ENU frame; ArduPilot fences are global
    lat/lon. Conversion is an equirectangular projection about the home
    position, which is accurate to well under a metre over an arena-sized area
    and is the same approximation MAVROS itself uses for local position.

Pure geometry plus message construction, no ROS spinning, so the fence can be
built and compared in a test without a flight controller.
"""

import math

EARTH_R = 6378137.0

# MAV_CMD fence commands.
FENCE_RETURN_POINT = 5000
FENCE_POLYGON_INCLUSION = 5001
FENCE_POLYGON_EXCLUSION = 5002
FENCE_CIRCLE_INCLUSION = 5003
FENCE_CIRCLE_EXCLUSION = 5004

FRAME_GLOBAL_REL_ALT = 3


def local_to_global(x, y, home_lat, home_lon):
    """Local ENU metres -> (lat, lon) about `home`.

    x is EAST, y is NORTH: the mission's local frame is ENU, and getting this
    pair the wrong way round rotates the entire fence 90 degrees about home —
    a failure that looks perfectly plausible on a map.
    """
    lat = home_lat + math.degrees(y / EARTH_R)
    lon = home_lon + math.degrees(x / (EARTH_R * math.cos(math.radians(home_lat))))
    return lat, lon


def global_to_local(lat, lon, home_lat, home_lon):
    """Inverse of local_to_global(), for comparing a read-back fence."""
    y = math.radians(lat - home_lat) * EARTH_R
    x = math.radians(lon - home_lon) * EARTH_R * math.cos(math.radians(home_lat))
    return x, y


def rect_vertices(rect):
    """(x0, x1, y0, y1) -> four corners, counter-clockwise.

    ArduPilot does not require a winding order for fence polygons, but a
    consistent one makes a read-back comparison a straight sequence compare
    instead of a set match.
    """
    x0, x1, y0, y1 = rect
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def build_fence(inclusion_rect, exclusion_rects, home_lat, home_lon,
                waypoint_cls):
    """Fence items: one inclusion polygon plus one exclusion per red zone.

    `waypoint_cls` is injected (mavros_msgs.msg.Waypoint) so this module can be
    imported and tested on a machine with no MAVROS installed.

    Every vertex of a polygon carries the SAME vertex count in param1 — that is
    how ArduPilot delimits one polygon from the next in a flat item list. Get
    it wrong and the vertices silently merge into one nonsensical shape.
    """
    items = []

    def polygon(rect, command):
        verts = rect_vertices(rect)
        for vx, vy in verts:
            lat, lon = local_to_global(vx, vy, home_lat, home_lon)
            w = waypoint_cls()
            w.frame = FRAME_GLOBAL_REL_ALT
            w.command = command
            w.is_current = False
            w.autocontinue = True
            w.param1 = float(len(verts))
            w.param2 = 0.0
            w.param3 = 0.0
            w.param4 = 0.0
            w.x_lat = float(lat)
            w.y_long = float(lon)
            w.z_alt = 0.0
            items.append(w)

    if inclusion_rect is not None:
        polygon(inclusion_rect, FENCE_POLYGON_INCLUSION)
    for rect in exclusion_rects:
        polygon(rect, FENCE_POLYGON_EXCLUSION)
    return items


def fence_signature(items, home_lat, home_lon):
    """A fence as (command, x_m, y_m) per item, in the LOCAL frame.

    Comparison is done in metres because degrees are unreadable and because
    the flight controller stores lat/lon as int32 at 1e-7 degrees — what comes
    back is never bit-identical to what went out.
    """
    sig = []
    for w in items:
        x, y = global_to_local(float(w.x_lat), float(w.y_long),
                               home_lat, home_lon)
        sig.append((int(w.command), x, y))
    return sig


def compare_fences(sent, received, home_lat, home_lon, tol_m=0.5):
    """(ok, reason) for an uploaded fence against what was read back.

    Compared with a TOLERANCE rather than by rounding: rounding puts an
    arbitrary boundary in the middle of the acceptable range, so two points
    11 cm apart can straddle it and fail while two points 99 cm apart pass.

    `tol_m` = 0.5 m is far below anything that matters for a fence and far
    above int32 quantisation (about 1 cm), so it catches a swapped axis, a
    wrong frame, a dropped vertex or a moved corner while ignoring encoding.
    """
    a = fence_signature(sent, home_lat, home_lon)
    b = fence_signature(received, home_lat, home_lon)
    if len(a) != len(b):
        return False, f"fence item count differs: sent {len(a)}, read back {len(b)}"
    for i, (ai, bi) in enumerate(zip(a, b)):
        if ai[0] != bi[0]:
            return False, (f"fence item {i} command differs: sent {ai[0]}, "
                           f"read back {bi[0]}")
        if abs(ai[1] - bi[1]) > tol_m or abs(ai[2] - bi[2]) > tol_m:
            return False, (f"fence item {i} differs: sent "
                           f"({ai[1]:.2f}, {ai[2]:.2f}) m, read back "
                           f"({bi[1]:.2f}, {bi[2]:.2f}) m")
    return True, ""


def inclusion_from_zone(zone, margin_m=5.0):
    """Arena inclusion fence around the observed operating area.

    Grown by `margin_m` because the fence must not fight the mission: a fence
    drawn exactly on the search zone would breach every time the aircraft
    overshoots a lane end by 30 cm.
    """
    x0, x1, y0, y1 = zone
    return (x0 - margin_m, x1 + margin_m, y0 - margin_m, y1 + margin_m)
