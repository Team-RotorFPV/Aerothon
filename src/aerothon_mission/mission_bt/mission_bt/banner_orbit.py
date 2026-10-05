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
    `green_fix` turns the likeliest of the detector's largest green regions
    into a position (bearing from the camera; range from the lidar, else from
    the region's height, which foreshortening does not shrink, or where it
    meets the ground if that is nearer). `orbit_plan`
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
SELF_M = 0.55             # lidar returns nearer than this are the airframe


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
              board_h_m=BOARD_H_M, lidar_half_rad=math.radians(3.0),
              refute=False):
    """Where the likeliest board among the green regions in view is, or None.

    Returns {"x", "y", "range", "area", "heading", "source", "cut", "edge"}
    in the mission's local frame. Range, in order of trust:

      * LIDAR. A return on the region's bearing is a measurement, not a model
        of what the green is. Only at gate height: above the walls the LD06's
        slice passes over everything.
      * THE NEARER OF GROUND CONTACT AND HEIGHT. With the camera's measured
        pitch and the aircraft's altitude, the region's bottom row is a ray
        that meets the ground. That is where a thing standing ON the ground
        is -- but the banner hangs 2.8 m up, and the ray under its bottom
        edge meets the ground far beyond it: on my_world, 11 m off, it read
        25 m from 5 m up and 165 m from gate height, and the real board was
        thrown away as too far to be the gate. A board seen edge-on loses
        width, not height, so its pixel height gives its range. A raised
        board's ground contact only ever reads long and a taller green
        thing's height only ever reads short: the nearer of the two is right
        for the board and errs towards a tighter orbit for anything else.
        Ground contact is not used when the region is cut off by the bottom
        of the frame (its real bottom is lower, i.e. nearer).

    WHICH REGION. The detector reports its few largest green regions, not just
    the largest: from my_world's take-off pad the delivery zone's grass, 13 m
    off, filled most of every frame the edge-on board was in, and the board
    never got to be the lead. A region spanning the whole width of the view
    is ground or a fence -- a 3.7 m board does that only from nearer than
    3.2 m, where it is read, not orbited -- and is no lead at all. One cut by
    a side of the frame (`edge`) has an unknown centre; whole ones win. One
    cut by the bottom and a side both is ground round the aircraft -- the
    grassed zone, on the way back -- and is no lead either.

    `refute`: the aircraft is at gate height, where the lidar's slice meets
    the gate's posts. A camera range cannot tell a flat patch of green from a
    small board hung nearer -- from 5 m up my_world's grass read as a board
    4.2 m off -- but the lidar can: a region it should see and does not is
    ground (see lidar_refutes), and is no lead.

    `cut` is True when the region runs off the bottom of the frame and no
    lidar range was had. Then the height is not the board's: on my_world the
    largest green was the outbound corridor's green FLOOR, running from the
    frame bottom up to the board, ~500 px "tall" -- a 3.2 m fix for a board
    11 m away, and an orbit round empty ground. A cut fix is a bearing, not
    a position.
    """
    g = getattr(mav, "banner_green", None)
    if not g:
        return None
    now = _now(mav)
    if now is not None and now - float(g.get("t", now)) > max_age_s:
        return None
    W, H = (g.get("wh") or [1280, 720])[:2]
    if W <= 0:
        return None
    regions = g.get("regions") or [
        {"px": g["px"], "area": float(g.get("area", 0.0)), "bearing": g.get("bearing")}]
    best = None
    for r in regions:
        f = _region_fix(mav, r, W, H, hfov_rad, board_h_m, lidar_half_rad)
        if f is None:
            continue
        if any(_dist_to_segment((f["x"], f["y"]), a, b) < exclude_m for a, b in exclude):
            continue
        if refute and f["source"] != "lidar" and lidar_refutes(mav, f):
            continue
        if better_green(f, best):
            best = f
    return best


def _region_fix(mav, r, W, H, hfov_rad, board_h_m, lidar_half_rad):
    """green_fix for one region {"px", "area", "bearing"?}, or None."""
    x0, y0, w, h = r["px"]
    if h <= 0:
        return None
    if x0 <= 1 and x0 + w >= W - 1:
        return None                       # wider than the view: not a board
    focal = 0.5 * float(W) / math.tan(0.5 * float(hfov_rad))
    bearing = r.get("bearing")
    if bearing is None:
        bearing = ((x0 + w / 2.0) - W / 2.0) / (W / 2.0)
    heading = _wrap(mav.yaw() + bearing_to_angle(float(bearing), hfov_rad))

    cam = getattr(mav, "camera_state", None) or {}
    pitch = cam.get("actual_rad")
    alt = mav.alt()
    bottom = y0 + h
    cut = bottom >= H - 2
    source = "lidar"
    rng = scan_min(mav, heading, lidar_half_rad)
    if rng is None:
        rng, source = focal * float(board_h_m) / float(h), "height"
        if pitch is not None and alt > 0.5 and not cut:
            down = -float(pitch) + math.atan((bottom - H / 2.0) / focal)
            if down > math.radians(3.0) and alt / math.tan(down) < rng:
                rng, source = alt / math.tan(down), "ground"
    rng = max(2.0, min(25.0, rng))
    edge = x0 <= 1 or x0 + w >= W - 1
    if cut and edge and source != "lidar":
        # Off the bottom AND a side: ground under and beside the aircraft
        # (the grassed zone, on the way back), with no range and no centre.
        return None

    px, py = mav.pos()[:2]
    return {"x": px + rng * math.cos(heading), "y": py + rng * math.sin(heading),
            "range": rng, "area": float(r.get("area", 0.0)),
            "heading": heading, "source": source,
            "cut": cut and source != "lidar",
            "edge": edge}


def lidar_refutes(mav, fix, board_w_m=3.7, slack=1.5, pad_m=1.0):
    """True when the lidar should see a gate where the camera put `fix` and
    sees nothing there.

    Only meaningful with the scan plane at gate height, where it meets the
    posts (they stand on the ground and rise above the board, whatever height
    it hangs at). The camera's range is allowed `slack` times over plus
    `pad_m`, and the sector spans the whole board and its posts. A fix the
    lidar cannot reach is never refuted: out of range is not evidence.
    """
    scan = getattr(mav, "_scan", None)
    if scan is None or not getattr(scan, "ranges", None):
        return False
    px, py = mav.pos()[:2]
    rng = math.hypot(fix["x"] - px, fix["y"] - py)
    reach = slack * rng + pad_m
    if reach >= float(scan.range_max):
        return False
    half = math.atan2(0.5 * board_w_m + 0.3, max(rng, 1.0))
    near = scan_min(mav, math.atan2(fix["y"] - py, fix["x"] - px), half)
    return near is None or near > reach


def better_green(f, best):
    """Whether fix `f` should replace `best`: one with a range beats one that
    is only a bearing (`cut`), a whole region beats one a side of the frame
    cuts off (`edge`), then the larger region wins."""
    if best is None:
        return True
    if bool(f.get("cut")) != bool(best.get("cut")):
        return not f.get("cut")
    if bool(f.get("edge")) != bool(best.get("edge")):
        return not f.get("edge")
    return f["area"] > best["area"]


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

    # Onto the circle first, at the aircraft's own angle, and look from
    # there: straight back from a board too close to read is where it is
    # likeliest to fit the frame, and the leg the aircraft arrived by.
    entry = pt(a0)
    if not ok(*entry):
        return []
    plan = [{"at": entry, "face": _wrap(a0 + math.pi), "path": [entry]}]
    first = max((1, -1), key=lambda d: 1 if ok(*pt(a0 + d * step_rad)) else 0)
    plan += way(first, a0, [], n - 1)
    left = n - len(plan)
    if left > 0:
        # Back round the arc to the start angle, then the other way.
        back = arc(a0 + first * (len(plan) - 1) * step_rad, a0)
        if all(ok(*p) for p in back):
            plan += way(-first, a0, back if len(plan) > 1 else [], left)
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


def leg_clear(mav, x, y, look_m=2.0, half_rad=math.radians(25.0), self_m=None):
    """False when the lidar sees something within `look_m` in the direction
    of (x, y). No scan, no veto: the altitude is the primary protection.

    Returns nearer than `self_m` are the aircraft itself, not an obstacle.
    Level, the LD06 sees the GPS mast at 0.22 m dead astern and the rear arms
    at 0.35 m (+-151 deg), measured in Gazebo. Banked into a leg the scan
    plane tilts through the landing gear and the hanging payload, and the
    farthest point of the airframe is 0.51 m from the lidar (CAD bounds
    +-0.29 m). Counting any of it blocked every orbit leg, two seconds in,
    whichever way the leg went (my_world, 00000171.BIN)."""
    px, py = mav.pos()[:2]
    if math.hypot(x - px, y - py) < 0.3:
        return True
    near = scan_min(mav, math.atan2(y - py, x - px), half_rad,
                    SELF_M if self_m is None else self_m)
    return near is None or near >= look_m


def scan_min(mav, heading, half_rad, self_m=None):
    """The nearest lidar return within `half_rad` of the local-frame
    `heading`, beyond the airframe's own reach; None with no scan or no
    return there."""
    scan = getattr(mav, "_scan", None)
    if scan is None or not getattr(scan, "ranges", None):
        return None
    rel = _wrap(heading - mav.yaw())
    lo = max(float(scan.range_min), SELF_M if self_m is None else self_m)
    hi = float(scan.range_max)
    a = float(scan.angle_min)
    inc = float(scan.angle_increment)
    best = None
    for i, r in enumerate(scan.ranges):
        if lo < r < hi and abs(_wrap(a + i * inc - rel)) <= half_rad:
            best = r if best is None else min(best, r)
    return best
