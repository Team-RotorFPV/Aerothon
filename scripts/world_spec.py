"""A user-built Mission 2 arena: the world spec, its geometry, its checks.

    tools/world_editor/  writes one of these (JSON, world frame, metres)
    materialize_world.py --world-spec FILE   turns it into the Gazebo world
                                             and the organiser inputs
    sim/check_track.py   grades a flight against the layout it produces

The seeded randomiser moves the arena within rules chosen in code. A spec is
chosen by a person, so every position, size and heading the mission meets
comes from outside the stack; nothing the mission flies to can be a constant
that happens to agree with the default arena.

WHAT A SPEC CAN MOVE, AND WHAT IT CANNOT

    Placed freely: the take-off area (pad, start QR and spawn together), the
    OUTBOUND corridor and the RETURN corridor (each its own pose, length,
    width and wall height; the return one "linked" beside the outbound one as
    in the rulebook drawing, or anywhere), every obstacle in the return
    corridor (position, size, height, heading, in the corridor's own frame),
    the delivery zone (axis-aligned, any size), the geofence (auto or any
    simple polygon), any number of red zones (any size, any heading), the
    five delivery pads, the green decoys, which pad the start QR names, and
    the QR edge lengths.

    Fixed shape: the banners (the board and its posts) and the take-off
    area's internal layout.

CORRIDOR FRAMES

    Outbound: origin at its banner (the entrance), +x down the lane towards
    the delivery zone, lane centred on y = 0.
    Return: origin at ITS banner (the entrance from the zone side), +x down
    the lane towards the far end, lane centred on y = 0. Obstacles are
    (u, v) = (along, across) in this frame.

Coordinates are Gazebo WORLD ENU. The mission's local frame is anchored at
the spawn point, so the materialiser converts what it publishes.
"""

import json
import math

SCHEMA = "aerothon-world/1"
PAD_LETTERS = ("a", "b", "c", "d", "e")

# ---- rigid structures, in their own frame (yaw 0) --------------------------
#
# Derived from mission2.sdf: the outbound banner at world (2, 2), both lanes
# 10 m x 3.5 m between 0.1 m walls 3.6 m tall, lane centres 4 m apart.
CORRIDOR_TEMPLATE_PIVOT = (2.0, 2.0)
DEFAULT_LANE = {"length": 10.0, "width": 3.5, "wall_height": 3.6}
WALL_T = 0.10                          # wall thickness
LANE_GAP = 0.30                        # between the two inner walls when linked
# The shipped return-lane obstacles: 0.35 m along, 1.45 m across, 3.4 m tall,
# alternating sides, in the return corridor frame (u along, v across).
SHIPPED_OBSTACLES = ((1.4, 1.05), (3.8, -1.05), (6.2, 1.05), (8.6, -1.05))
# Take-off frame: origin at the pad centre. In mission2.sdf the pad is at
# (-1, 0), the start QR at (-1, 2) and the vehicle spawns at (-2, 2).
TAKEOFF_TEMPLATE_CENTRE = (-1.0, 0.0)
TAKEOFF_PAD_SIZE = (5.0, 8.0)
START_QR_OFFSET = (0.0, 2.0)
SPAWN_OFFSET = (-1.0, 2.0)

HOME_FENCE_CLEARANCE_M = 2.0           # readiness: home >= 2 m inside the fence
BANNER_NEAR_RANGE_M = 9.6              # AlignToBanner: 0.8 x the 12 m lidar range
AUTO_FENCE_MARGIN_M = 6.0


def default_spec():
    """The shipped arena (mission2.sdf) as a spec."""
    return {
        "schema": SCHEMA,
        "name": "shipped",
        "takeoff": {"x": -1.0, "y": 0.0, "yaw_deg": 0.0},
        "corridor": {"x": 2.0, "y": 2.0, "yaw_deg": 0.0, **DEFAULT_LANE},
        "return_corridor": {"linked": True, "x": 12.0, "y": -2.0,
                            "yaw_deg": 180.0, **DEFAULT_LANE,
                            "obstacles": [
                                {"u": u, "v": v, "w": 0.35, "d": 1.45,
                                 "h": 3.4, "yaw_deg": 0.0}
                                for u, v in SHIPPED_OBSTACLES]},
        "delivery_zone": {"x": 32.0, "y": 0.0, "w": 40.0, "h": 30.0},
        "geofence": {"mode": "auto", "margin": AUTO_FENCE_MARGIN_M},
        "red_zones": [
            {"x": 38.0, "y": 5.0, "w": 10.0, "h": 7.0, "yaw_deg": 0.0},
            {"x": 29.0, "y": 10.0, "w": 6.0, "h": 4.0, "yaw_deg": 0.0},
            {"x": 40.0, "y": -11.0, "w": 7.0, "h": 4.0, "yaw_deg": 0.0},
        ],
        "pads": {"a": {"x": 21.0, "y": 10.0}, "b": {"x": 47.0, "y": 10.0},
                 "c": {"x": 23.0, "y": 1.0}, "d": {"x": 33.0, "y": -10.0},
                 "e": {"x": 45.0, "y": -6.0}},
        "decoys": [{"x": 8.0, "y": -14.0, "yaw_deg": 0.0},
                   {"x": 34.0, "y": 9.0, "yaw_deg": math.degrees(1.2)}],
        "start_target": "random",
        "qr": {"start_m": 2.2, "target_m": 3.0},
    }


def load(path):
    with open(path, encoding="utf-8") as f:
        return normalise(json.load(f))


def normalise(spec):
    """Fill anything a spec leaves out from the default, and coerce types."""
    base = default_spec()
    out = dict(base)
    for key, val in (spec or {}).items():
        out[key] = val
    for key in ("takeoff", "corridor", "return_corridor", "delivery_zone", "qr"):
        merged = dict(base[key])
        merged.update(out.get(key) or {})
        out[key] = merged
    out["return_corridor"]["obstacles"] = [
        {"yaw_deg": 0.0, **o} for o in out["return_corridor"].get("obstacles") or []]
    if out["return_corridor"].get("linked", True):
        # Linked: beside the outbound lane, pointing back, as in Figure 3.
        # Its pose is derived, so moving the outbound corridor moves it too.
        x, y, yaw = linked_return_pose(out)
        out["return_corridor"].update({"x": x, "y": y, "yaw_deg": yaw})
    out["geofence"] = dict(out.get("geofence") or base["geofence"])
    out["red_zones"] = [dict(r) for r in (out.get("red_zones") or [])]
    for r in out["red_zones"]:
        r.setdefault("yaw_deg", 0.0)
    out["pads"] = {k: dict(v) for k, v in (out.get("pads") or {}).items()}
    out["decoys"] = [dict(d) for d in (out.get("decoys") or [])]
    for d in out["decoys"]:
        d.setdefault("yaw_deg", 0.0)
    out["start_target"] = str(out.get("start_target", "random")).lower()
    return out


# ---- geometry ---------------------------------------------------------------

def _place(local, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = local
    return (x + lx * c - ly * s, y + lx * s + ly * c)


def rect_corners(cx, cy, w, h, yaw=0.0):
    return [_place(p, cx, cy, yaw) for p in
            ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]


def linked_return_pose(spec):
    """Where a linked return corridor sits: beside the outbound lane, to its
    starboard, its entrance level with the outbound EXIT, pointing back.
    (x, y, yaw_deg). For the shipped lanes this is (12, -2, 180)."""
    c, r = spec["corridor"], spec["return_corridor"]
    spacing = (float(c["width"]) + float(r["width"])) / 2 + 2 * WALL_T + LANE_GAP
    x, y = _place((float(c["length"]), -spacing), c["x"], c["y"],
                  math.radians(c["yaw_deg"]))
    yaw = (float(c["yaw_deg"]) + 180.0 + 180.0) % 360.0 - 180.0
    return round(x, 4), round(y, 4), yaw


def lane_polygon(c):
    """Outer footprint (wall faces) of one corridor dict."""
    L, W = float(c["length"]), float(c["width"])
    h = W / 2 + WALL_T
    yaw = math.radians(c["yaw_deg"])
    return [_place(p, c["x"], c["y"], yaw) for p in
            ((-0.1, -h), (L + 0.1, -h), (L + 0.1, h), (-0.1, h))]


def corridor_polygon(spec):
    """The OUTBOUND corridor's footprint (kept name: most checks mean it)."""
    return lane_polygon(spec["corridor"])


def return_polygon(spec):
    return lane_polygon(spec["return_corridor"])


def corridor_exit(spec):
    """Where the outbound lane opens out: its far end, on the centreline."""
    c = spec["corridor"]
    return _place((float(c["length"]), 0.0), c["x"], c["y"],
                  math.radians(c["yaw_deg"]))


def return_entrance(spec):
    r = spec["return_corridor"]
    return (float(r["x"]), float(r["y"]))


def obstacle_polygons(spec):
    """Every return-lane obstacle as a world polygon."""
    r = spec["return_corridor"]
    yaw = math.radians(r["yaw_deg"])
    out = []
    for o in r.get("obstacles") or []:
        cx, cy = _place((float(o["u"]), float(o["v"])), r["x"], r["y"], yaw)
        out.append(rect_corners(cx, cy, float(o["w"]), float(o["d"]),
                                yaw + math.radians(o.get("yaw_deg", 0.0))))
    return out


def takeoff_polygon(spec):
    t = spec["takeoff"]
    return rect_corners(t["x"], t["y"], *TAKEOFF_PAD_SIZE,
                        math.radians(t["yaw_deg"]))


def spawn_point(spec):
    t = spec["takeoff"]
    return _place(SPAWN_OFFSET, t["x"], t["y"], math.radians(t["yaw_deg"]))


def start_qr_point(spec):
    t = spec["takeoff"]
    return _place(START_QR_OFFSET, t["x"], t["y"], math.radians(t["yaw_deg"]))


def zone_rect(spec):
    z = spec["delivery_zone"]
    return (z["x"] - z["w"] / 2, z["x"] + z["w"] / 2,
            z["y"] - z["h"] / 2, z["y"] + z["h"] / 2)


def red_polygon(r):
    return rect_corners(r["x"], r["y"], r["w"], r["h"], math.radians(r["yaw_deg"]))


def pad_polygon(spec, letter):
    p = spec["pads"][letter]
    m = float(spec["qr"]["target_m"])
    return rect_corners(p["x"], p["y"], m, m, math.radians(p.get("yaw_deg", 0.0)))


def fence_polygon(spec):
    """The geofence as a CCW vertex list, auto-drawn or as the user gave it."""
    g = spec["geofence"]
    if g.get("mode") == "polygon" and len(g.get("vertices") or []) >= 3:
        pts = [(float(x), float(y)) for x, y in g["vertices"]]
        return pts if polygon_area(pts) > 0 else pts[::-1]
    m = float(g.get("margin", AUTO_FENCE_MARGIN_M))
    x0, x1, y0, y1 = zone_rect(spec)
    pts = (takeoff_polygon(spec) + corridor_polygon(spec) + return_polygon(spec)
           + [(x0, y0), (x1, y1)])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return [(min(xs) - m, min(ys) - m), (max(xs) + m, min(ys) - m),
            (max(xs) + m, max(ys) + m), (min(xs) - m, max(ys) + m)]


def polygon_area(pts):
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1]
                     - pts[(i + 1) % len(pts)][0] * pts[i][1]
                     for i in range(len(pts)))


def point_in_polygon(pt, poly):
    x, y = pt
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def distance_to_edge(pt, poly):
    return min(_seg_dist(pt, poly[i], poly[(i + 1) % len(poly)])
               for i in range(len(poly)))


def _segments_cross(p1, p2, p3, p4):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def polygons_overlap(a, b):
    """Convex-or-not polygons share area (edge crossing or containment)."""
    for i in range(len(a)):
        for j in range(len(b)):
            if _segments_cross(a[i], a[(i + 1) % len(a)], b[j], b[(j + 1) % len(b)]):
                return True
    return point_in_polygon(a[0], b) or point_in_polygon(b[0], a)


def polygon_gap(a, b):
    """Shortest distance between two polygons; 0 if they overlap."""
    if polygons_overlap(a, b):
        return 0.0
    return min(min(distance_to_edge(p, b) for p in a),
               min(distance_to_edge(p, a) for p in b))


def is_simple(poly):
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            if abs(i - j) <= 1 or {i, j} == {0, n - 1}:
                continue
            if _segments_cross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return False
    return True


# ---- checks -----------------------------------------------------------------

AIRFRAME_SPAN_M = 1.0        # Iris 0.8 m prop tip to tip, plus 0.1 m each side
CORRIDOR_ALT_M = 3.0         # mission corridor_alt
LIDAR_ABOVE_BODY_M = 0.235   # RPLidar C1 scan plane above base_link
AIRFRAME_BELOW_BODY_M = 0.35 # legs 0.195 m + sag/altitude error below base_link
PASS_COMFORT_M = 1.6         # the corridor navigator's 1.4 m passage strip + margin


def _lane_inset(c, m):
    """A corridor's footprint shrunk by m on every side."""
    L, W = float(c["length"]), float(c["width"])
    h = W / 2 + WALL_T - m
    yaw = math.radians(c["yaw_deg"])
    return [_place(p, c["x"], c["y"], yaw) for p in
            ((-0.1 + m, -h), (L + 0.1 - m, -h), (L + 0.1 - m, h), (-0.1 + m, h))]


def _check_obstacles(spec, errs, warns):
    """Every obstacle inside its lane, and every one leaving a way past."""
    r = spec["return_corridor"]
    L, W = float(r["length"]), float(r["width"])
    for i, o in enumerate(r.get("obstacles") or [], 1):
        w, d, h = float(o["w"]), float(o["d"]), float(o["h"])
        if min(w, d, h) <= 0:
            errs.append(f"obstacle {i} needs a positive size")
            continue
        a = math.radians(o.get("yaw_deg", 0.0))
        # Extent of the (turned) obstacle along and across the lane.
        du = abs(w * math.cos(a)) / 2 + abs(d * math.sin(a)) / 2
        dv = abs(w * math.sin(a)) / 2 + abs(d * math.cos(a)) / 2
        u, v = float(o["u"]), float(o["v"])
        if u - du < 0.0 or u + du > L:
            errs.append(f"obstacle {i} sticks out of the end of the return corridor")
        # Into the wall is fine (the shipped ones touch it); through it is not.
        if abs(v) + dv > W / 2 + WALL_T + 1e-6:
            errs.append(f"obstacle {i} goes through the return corridor's wall")
            continue
        free = max(W / 2 - (v + dv), (v - dv) + W / 2)
        if free < AIRFRAME_SPAN_M:
            errs.append(f"obstacle {i} blocks the return corridor: the widest "
                        f"gap past it is {free:.2f} m; it needs at least "
                        f"{AIRFRAME_SPAN_M:.1f} m (0.8 m airframe + clearance)")
        elif free < PASS_COMFORT_M:
            warns.append(f"obstacle {i} leaves only {free:.2f} m to fly past; "
                         f"the corridor navigator passes reliably with "
                         f"{PASS_COMFORT_M:.1f} m or more")
        # THE LIDAR IS ONE PLANE. At the corridor altitude it scans at
        # CORRIDOR_ALT_M + LIDAR_ABOVE_BODY_M; an obstacle whose top is below
        # that plane but above the airframe's underside is invisible to the
        # navigator and in the aircraft's way. Found live: a 3.2 m block,
        # 3.23 m scan plane, flown into at 3 m (pitch 48 deg).
        plane = CORRIDOR_ALT_M + LIDAR_ABOVE_BODY_M
        belly = CORRIDOR_ALT_M - AIRFRAME_BELOW_BODY_M
        if belly < h < plane + 0.1:
            errs.append(f"obstacle {i} is {h:g} m tall: its top is below the "
                        f"lidar's scan plane ({plane:.2f} m at the {CORRIDOR_ALT_M:g} m "
                        f"corridor altitude) but above the airframe's underside "
                        f"({belly:.2f} m) -- the aircraft cannot see it and would "
                        f"fly into it. Make it taller than {plane + 0.1:.2f} m or "
                        f"shorter than {belly:.2f} m")
        elif h <= belly:
            warns.append(f"obstacle {i} is {h:g} m tall; the corridor is flown "
                         f"at {CORRIDOR_ALT_M:g} m, so the aircraft passes over it")

def validate(spec):
    """(errors, warnings). An error is a world the mission cannot legally fly
    or the simulator cannot build; a warning is legal but worth a second look.
    """
    spec = normalise(spec)
    errs, warns = [], []
    z = spec["delivery_zone"]
    if not (z["w"] > 0 and z["h"] > 0):
        errs.append("delivery zone needs a positive width and height")
        return errs, warns
    zx0, zx1, zy0, zy1 = zone_rect(spec)
    zone_poly = [(zx0, zy0), (zx1, zy0), (zx1, zy1), (zx0, zy1)]

    missing = [l for l in PAD_LETTERS if l not in spec["pads"]]
    if missing:
        errs.append("missing delivery pad(s): " + ", ".join(m.upper() for m in missing))
    if spec["start_target"] not in PAD_LETTERS + ("random",):
        errs.append("start target must be a, b, c, d, e or random")
    for key in ("start_m", "target_m"):
        v = float(spec["qr"][key])
        if not 0.2 <= v <= 5.0:
            errs.append(f"QR size {key} = {v} m is outside 0.2-5 m")

    fence = fence_polygon(spec)
    if len(fence) < 3:
        errs.append("the geofence needs at least 3 vertices")
    elif not is_simple(fence):
        errs.append("the geofence polygon crosses itself")
    else:
        home = spawn_point(spec)
        if not point_in_polygon(home, fence):
            errs.append("the take-off point (FCU home) is outside the geofence")
        elif distance_to_edge(home, fence) < HOME_FENCE_CLEARANCE_M:
            errs.append(f"the take-off point is within {HOME_FENCE_CLEARANCE_M:.0f} m "
                        "of the geofence (readiness refuses to arm)")
        for name, poly in (("delivery zone", zone_poly),
                           ("outbound corridor", corridor_polygon(spec)),
                           ("return corridor", return_polygon(spec)),
                           ("take-off pad", takeoff_polygon(spec))):
            if not all(point_in_polygon(p, fence) for p in poly):
                errs.append(f"the {name} is not entirely inside the geofence")

    corridor = corridor_polygon(spec)
    ret = return_polygon(spec)
    takeoff = takeoff_polygon(spec)
    for name, c in (("outbound corridor", spec["corridor"]),
                    ("return corridor", spec["return_corridor"])):
        for key, lo, hi in (("length", 4.0, 40.0), ("width", 2.0, 8.0),
                            ("wall_height", 2.0, 8.0)):
            v = float(c[key])
            if not lo <= v <= hi:
                errs.append(f"the {name}'s {key.replace('_', ' ')} {v:g} m is "
                            f"outside {lo:g}-{hi:g} m")
    if polygons_overlap(corridor, takeoff):
        errs.append("the outbound corridor overlaps the take-off pad")
    if polygons_overlap(ret, takeoff):
        errs.append("the return corridor overlaps the take-off pad")
    if polygons_overlap(_lane_inset(spec["corridor"], 0.05),
                        _lane_inset(spec["return_corridor"], 0.05)):
        errs.append("the outbound and return corridors overlap")
    _check_obstacles(spec, errs, warns)
    # The mission finds the outbound banner FROM the take-off point (camera
    # sweep, then lidar square-up) and treats a banner beyond 80% of the 12 m
    # lidar range as the far gate. A world that breaks either is legal, but
    # the mission will not find its way in -- worth knowing before flying it.
    cc = spec["corridor"]
    hx, hy = spawn_point(spec)
    dx, dy = hx - cc["x"], hy - cc["y"]
    cy_ = math.radians(cc["yaw_deg"])
    along = dx * math.cos(cy_) + dy * math.sin(cy_)
    if along > -2.0:
        warns.append("the take-off point is not in front of the corridor "
                     "entrance; the outbound banner faces the corridor's -x side")
    if math.hypot(dx, dy) > BANNER_NEAR_RANGE_M:
        warns.append(f"the outbound banner is {math.hypot(dx, dy):.1f} m from the "
                     f"take-off point; the mission takes a banner within "
                     f"{BANNER_NEAR_RANGE_M:.1f} m as the near gate")
    # Inset 0.5 m: the shipped arena's walls end 0.1 m inside the field.
    for name, c in (("outbound", spec["corridor"]), ("return", spec["return_corridor"])):
        if polygons_overlap(_lane_inset(c, 0.5), zone_poly):
            warns.append(f"the {name} corridor overlaps the delivery zone; the "
                         "rulebook corridors lead INTO the zone, they do not sit "
                         "inside it")
    ex = corridor_exit(spec)
    if distance_to_edge(ex, zone_poly) > 6.0 and not point_in_polygon(ex, zone_poly):
        warns.append(f"the outbound corridor exit is "
                     f"{distance_to_edge(ex, zone_poly):.1f} m from the delivery "
                     "zone; the mission enters the zone from it")
    rin = return_entrance(spec)
    if distance_to_edge(rin, zone_poly) > 6.0 and not point_in_polygon(rin, zone_poly):
        warns.append(f"the return corridor entrance is "
                     f"{distance_to_edge(rin, zone_poly):.1f} m from the delivery "
                     "zone; the return lap starts from the zone")
    if polygon_gap(takeoff, zone_poly) < 1.0:
        warns.append("the take-off pad touches the delivery zone")

    for i, r in enumerate(spec["red_zones"], 1):
        if not (r["w"] > 0 and r["h"] > 0):
            errs.append(f"red zone {i} needs a positive width and height")
            continue
        rp = red_polygon(r)
        if polygons_overlap(rp, takeoff):
            errs.append(f"red zone {i} overlaps the take-off pad")
        if polygons_overlap(rp, corridor):
            errs.append(f"red zone {i} overlaps the outbound corridor")
        if polygons_overlap(rp, ret):
            errs.append(f"red zone {i} overlaps the return corridor")
        for where, pt in (("outbound corridor exit", ex),
                          ("return corridor entrance", rin)):
            gap = (0.0 if point_in_polygon(pt, rp)
                   else distance_to_edge(pt, rp))
            if gap < 3.0:
                warns.append(f"red zone {i} is {gap:.1f} m from the {where}; "
                             "the rulebook gives no other way through")
        if not all(point_in_polygon(p, zone_poly) for p in rp):
            warns.append(f"red zone {i} is not entirely inside the delivery zone")

    for l in PAD_LETTERS:
        if l not in spec["pads"]:
            continue
        pp = pad_polygon(spec, l)
        if not all(point_in_polygon(p, zone_poly) for p in pp):
            errs.append(f"pad {l.upper()} is not entirely inside the delivery zone")
        for i, r in enumerate(spec["red_zones"], 1):
            if r["w"] > 0 and r["h"] > 0 and polygon_gap(pp, red_polygon(r)) < 1.0:
                errs.append(f"pad {l.upper()} is within 1 m of red zone {i}; "
                            "it cannot be delivered to without a violation")
    placed = [(l, spec["pads"][l]) for l in PAD_LETTERS if l in spec["pads"]]
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            (la, a), (lb, b) = placed[i], placed[j]
            d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            if d < float(spec["qr"]["target_m"]) + 1.0:
                errs.append(f"pads {la.upper()} and {lb.upper()} overlap")
            elif d < 5.0:
                warns.append(f"pads {la.upper()} and {lb.upper()} are {d:.1f} m "
                             "apart; both can be in one camera frame")
    return errs, warns
