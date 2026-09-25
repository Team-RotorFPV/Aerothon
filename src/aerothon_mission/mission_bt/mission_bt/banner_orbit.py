"""Finding a banner the aircraft can only see edge-on.

THE CASE THIS EXISTS FOR

    A custom arena put the take-off pad 10.5 m NORTH of the corridor mouth,
    while the banner faces WEST down the lane. From the pad the board is seen
    almost exactly edge-on: a green sliver of ~2000 px with no lettering, which
    the detector correctly refuses to call the banner. AlignToBanner then swept
    a full turn, saw "green region too small" at every heading, and relocated
    along its ENTRY heading -- east, parallel to the board -- which can never
    bring its face into view. FindReturnBanner has the same blind spot on the
    way back.

    Nothing about that is exotic. On a real field the start pad is wherever the
    organisers put it, and a banner seen from behind or from the side is the
    worst case the search has to survive.

WHAT IT DOES

    The unreadable green region is still evidence: it is where the gate is.
    `green_fix` turns the detector's largest green region into a position
    (bearing from the camera; range from where the region meets the ground,
    else from its height, which foreshortening does not shrink). `orbit_plan`
    lays out vantage points on a circle round that position, one way round
    from the aircraft's own angle, each facing the centre, with arc waypoints
    between them so no leg cuts across the structure. At every vantage the
    ordinary stare-and-decide runs over a short fan of headings; the first
    face-on view identifies the banner and the stage carries on as before.

    It flies at the altitude the search started at, and every leg is checked
    against the geofence (with margin), red ground, and the lidar.
"""

import math

from mission_bt.delivery_zone import point_inside_with_margin
from mission_bt.scan_geometry import bearing_to_angle

BOARD_H_M = 1.15          # the banner board's height, the height-based fallback


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _now(mav):
    node = getattr(mav, "node", None)
    if node is None:
        return None
    return node.get_clock().now().nanoseconds * 1e-9


def outbound_structure(mav):
    """Segments the OUTBOUND gate and corridor occupy, once they are known.

    Coming back, the green corridor the aircraft has already flown is the
    biggest green thing in view, and orbiting it would find the outbound
    banner again. From the recorded banner position to where the corridor
    opened out.
    """
    ob = getattr(mav, "outbound_banner_xy", None)
    if ob is None:
        return []
    ex = getattr(mav, "corridor_exit_pose", None)
    end = (ex[0], ex[1]) if ex else ob
    return [(tuple(ob[:2]), tuple(end[:2]))]


def _dist_to_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def green_fix(mav, hfov_rad, max_age_s=1.0, exclude=(), exclude_m=3.0,
              board_h_m=BOARD_H_M):
    """Where the largest green region in view is, or None.

    Returns {"x", "y", "range", "area", "heading"} in the mission's local
    frame. Range, in order of trust:

      * GROUND CONTACT. With the camera's measured pitch and the aircraft's
        altitude, the region's bottom row is a ray that meets the ground where
        the structure stands. Not used when the region is cut off by the
        bottom of the frame (its real bottom is lower, i.e. nearer).
      * HEIGHT. A board seen edge-on loses width, not height, so its pixel
        height still gives range for a board of known height. A taller green
        structure reads NEARER than it is, which only tightens the orbit.
    """
    g = getattr(mav, "banner_green", None)
    if not g:
        return None
    now = _now(mav)
    if now is not None and now - float(g.get("t", now)) > max_age_s:
        return None
    x0, y0, w, h = g["px"]
    W, H = (g.get("wh") or [1280, 720])[:2]
    if h <= 0 or W <= 0:
        return None
    focal = 0.5 * float(W) / math.tan(0.5 * float(hfov_rad))
    heading = _wrap(mav.yaw() + bearing_to_angle(float(g["bearing"]), hfov_rad))

    rng = None
    cam = getattr(mav, "camera_state", None) or {}
    pitch = cam.get("actual_rad")
    alt = mav.alt()
    bottom = y0 + h
    if pitch is not None and alt > 0.5 and bottom < H - 2:
        down = -float(pitch) + math.atan((bottom - H / 2.0) / focal)
        if down > math.radians(3.0):
            rng = alt / math.tan(down)
    if rng is None:
        rng = focal * float(board_h_m) / float(h)
    rng = max(2.0, min(25.0, rng))

    px, py = mav.pos()[:2]
    gx, gy = px + rng * math.cos(heading), py + rng * math.sin(heading)
    for a, b in exclude:
        if _dist_to_segment((gx, gy), a, b) < exclude_m:
            return None
    return {"x": gx, "y": gy, "range": rng, "area": float(g.get("area", 0.0)),
            "heading": heading}


def orbit_plan(centre, here, radius, ok, step_rad=math.radians(45.0), n=7,
               arc_step_rad=math.radians(30.0)):
    """Vantage points round `centre`, facing it.

    One way round from the aircraft's own angle (a full turn of alternating
    vantages would fly most of the circle twice): the first way whose first
    vantage can be flown to, and if that way runs into the fence or red
    ground, the rest of the budget goes the other way. Each entry is
    {"at": (x, y), "face": yaw, "path": [(x, y), ...]} where `path` is the arc
    from the previous vantage (or from the aircraft) ending at "at".
    `ok(x, y)` says whether a point may be flown to.
    """
    cx, cy = centre
    a0 = math.atan2(here[1] - cy, here[0] - cx)

    def pt(a):
        return (cx + radius * math.cos(a), cy + radius * math.sin(a))

    def arc(a_from, a_to):
        span = a_to - a_from
        k = max(1, int(math.ceil(abs(span) / arc_step_rad)))
        return [pt(a_from + span * i / k) for i in range(1, k + 1)]

    def way(d, start_a, start_path, budget):
        out, a_prev, path = [], start_a, list(start_path)
        for k in range(1, budget + 1):
            a = a0 + d * k * step_rad
            leg = path + arc(a_prev, a)
            if not all(ok(*p) for p in leg):
                break
            at = pt(a)
            out.append({"at": at, "face": _wrap(math.atan2(cy - at[1], cx - at[0])),
                        "path": leg})
            a_prev, path = a, []
        return out

    # Onto the circle first, at the aircraft's own angle.
    entry = [pt(a0)]
    if not ok(*entry[0]):
        return []
    first = max((1, -1), key=lambda d: 1 if ok(*pt(a0 + d * step_rad)) else 0)
    plan = way(first, a0, entry, n)
    left = n - len(plan)
    if left > 0:
        # Back round the arc to the start angle, then the other way.
        back_from = a0 + first * len(plan) * step_rad
        back = arc(back_from, a0) if plan else entry
        if all(ok(*p) for p in back):
            plan += way(-first, a0, back if plan else entry, left)
    return plan


def fence_ok(mav, margin_m=2.0, on_red=None):
    """A point test for orbit_plan: inside the arena fence with margin, and
    off red ground. An unknown fence does not veto (the FC fence still does)."""
    fence = getattr(mav, "geofence_local", None)

    def ok(x, y):
        if fence and not point_inside_with_margin(x, y, fence, margin_m):
            return False
        return not (on_red and on_red(x, y))
    return ok


def leg_clear(mav, x, y, look_m=2.0, half_rad=math.radians(25.0)):
    """False when the lidar sees something within `look_m` in the direction
    of (x, y). No scan, no veto: the altitude is the primary protection."""
    scan = getattr(mav, "_scan", None)
    if scan is None or not getattr(scan, "ranges", None):
        return True
    px, py = mav.pos()[:2]
    if math.hypot(x - px, y - py) < 0.3:
        return True
    rel = _wrap(math.atan2(y - py, x - px) - mav.yaw())
    lo, hi = float(scan.range_min), float(scan.range_max)
    a = float(scan.angle_min)
    inc = float(scan.angle_increment)
    for i, r in enumerate(scan.ranges):
        if lo < r < hi and r < look_m and abs(_wrap(a + i * inc - rel)) <= half_rad:
            return False
    return True
