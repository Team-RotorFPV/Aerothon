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

    Fixed shape: the banners (the board and its posts; only the height the
    board hangs at moves) and the take-off area's internal layout.

CONDITIONS -- WHAT THE DAY THROWS AT IT

    "conditions" sets the weather, the sensors and the state of the printed
    markers the mission flies in (see conditions() and CONDITION_PRESETS):

        wind    Gazebo WindEffects: mean speed and heading, sinusoidal gusts,
                a meandering direction; drag calibrated to the airframe
        camera  sim_gazebo/corruptions.py severities (0-5) on every frame,
                plus dropped frames and delivery latency
        lidar   range noise, missing returns, false short returns
        wear    faded, dusty print on the banner and the QR pads (0-5)
        fcu     ArduPilot SITL sensor faults: GPS noise, a timed GPS glitch,
                baro noise and drift, IMU noise, a part-used battery

    Either a preset -- {"preset": "worst"} -- with any field overridden, or
    {"preset": "random"}: every factor drawn between calm and worst from the
    run's seed (domain randomisation), so a campaign of seeds covers the
    space instead of one point in it.

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
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))), "src", "aerothon_sim", "sim_gazebo"))
from sim_gazebo.corruptions import CAMERA_KEYS  # noqa: E402

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

# The banner board: 3.7 x 1.15 m on two posts. Its height is the one thing
# about it a spec moves (mission2.sdf hangs it at 2.805 m); the posts grow to
# carry it.
BOARD_BOTTOM_M = 2.805
BOARD_BOTTOM_RANGE_M = (1.8, 4.5)

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
        "banner": {"board_bottom_m": BOARD_BOTTOM_M},
        "conditions": {"preset": "calm"},
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
    for key in ("takeoff", "corridor", "return_corridor", "delivery_zone", "qr",
                "banner"):
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
    out["conditions"] = dict(out.get("conditions") or {"preset": "calm"})
    return out


# ---- conditions ---------------------------------------------------------------

_CALM = {
    "wind": {"speed": 0.0, "dir_deg": 0.0, "gust": 0.0, "gust_period_s": 6.0,
             "veer_deg": 0.0},
    "camera": {**{k: 0 for k in CAMERA_KEYS}, "frame_drop": 0.0, "latency_ms": 0},
    "lidar": {"noise_m": 0.0, "dropout": 0.0, "spurious": 0.0},
    "wear": {"banner": 0, "qr": 0},
    "fcu": {"gps_noise_m": 0.0, "gps_glitch_m": 0.0, "glitch_at_s": 150.0,
            "glitch_s": 5.0, "baro_noise_m": 0.0, "baro_drift_mps": 0.0,
            "imu_noise": 0.0, "battery_v": 16.8},
}

# The worst credible day. Wind: above ~8 m/s mean with 4 m/s gusts flying
# stops for every team. Camera: severity 2 of everything at once, the
# envelope sim/test_perception_corruption.py holds every detector to one at
# a time. GPS: a 5 m glitch for 5 s mid-search. Baro: 3 mm/s of drift, 2.7 m
# over a full 15 min -- a weather front is ~2 mm/s, the rest is a flight
# controller armed before it warmed up (docs/FIELD_READINESS.md). Battery: a
# pack already flown once, at 3.85 V/cell.
CONDITION_PRESETS = {
    "calm": _CALM,
    "field": {
        "wind": {"speed": 4.0, "gust": 2.0, "veer_deg": 10.0},
        "camera": {"noise": 1, "motion_blur": 1, "jpeg": 1, "vibration": 1,
                   "frame_drop": 0.05, "latency_ms": 60},
        "lidar": {"noise_m": 0.01, "dropout": 0.02, "spurious": 0.002},
        "wear": {"banner": 1, "qr": 1},
        "fcu": {"gps_noise_m": 0.5, "baro_noise_m": 0.2, "imu_noise": 0.5},
    },
    "worst": {
        "wind": {"speed": 8.0, "gust": 4.0, "veer_deg": 20.0},
        "camera": {"noise": 2, "motion_blur": 2, "defocus": 1, "haze": 2,
                   "exposure": -2, "glare": 2, "jpeg": 2, "lens_dust": 2,
                   "vibration": 2, "frame_drop": 0.2, "latency_ms": 150},
        "lidar": {"noise_m": 0.03, "dropout": 0.10, "spurious": 0.01},
        "wear": {"banner": 3, "qr": 3},
        "fcu": {"gps_noise_m": 1.5, "gps_glitch_m": 5.0, "baro_noise_m": 0.5,
                "baro_drift_mps": 0.003, "imu_noise": 1.0, "battery_v": 15.4},
    },
}


def _merge(base, over):
    out = {k: dict(v) for k, v in base.items()}
    for group, fields in (over or {}).items():
        if group in out and isinstance(fields, dict):
            out[group].update(fields)
    return out


def _randomised(rng):
    """Every factor uniform between calm and worst; wind from any heading."""
    worst = _merge(_CALM, CONDITION_PRESETS["worst"])
    out = {}
    for group, fields in _CALM.items():
        out[group] = {}
        for k, calm in fields.items():
            hi = worst[group][k]
            if isinstance(calm, int) and isinstance(hi, int):
                v = rng.randint(min(calm, hi), max(calm, hi))
            else:
                v = round(rng.uniform(min(calm, hi), max(calm, hi)), 3)
            out[group][k] = v
    out["wind"]["dir_deg"] = round(rng.uniform(0.0, 360.0), 1)
    out["fcu"]["glitch_at_s"] = round(rng.uniform(60.0, 300.0), 1)
    return out


def conditions(spec, seed=0):
    """The concrete conditions a spec asks for: every field filled in.

    `seed` only matters for {"preset": "random"}; the same seed gives the
    same day, so a failure can be flown again.
    """
    c = dict((spec or {}).get("conditions") or {})
    name = str(c.pop("preset", "calm")).lower()
    if name == "random":
        base = _randomised(random.Random(seed))
    else:
        base = _merge(_CALM, CONDITION_PRESETS.get(name, {}))
    return _merge(base, c)


_CONDITION_LIMITS = {
    ("wind", "speed"): (0.0, 15.0), ("wind", "gust"): (0.0, 10.0),
    ("wind", "gust_period_s"): (1.0, 120.0), ("wind", "veer_deg"): (0.0, 90.0),
    ("camera", "frame_drop"): (0.0, 0.9), ("camera", "latency_ms"): (0, 1000),
    ("lidar", "noise_m"): (0.0, 0.5), ("lidar", "dropout"): (0.0, 0.9),
    ("lidar", "spurious"): (0.0, 0.5),
    ("wear", "banner"): (0, 5), ("wear", "qr"): (0, 5),
    ("fcu", "gps_noise_m"): (0.0, 10.0), ("fcu", "gps_glitch_m"): (0.0, 50.0),
    ("fcu", "glitch_at_s"): (0.0, 1200.0), ("fcu", "glitch_s"): (0.0, 120.0),
    ("fcu", "baro_noise_m"): (0.0, 5.0), ("fcu", "baro_drift_mps"): (0.0, 0.5),
    ("fcu", "imu_noise"): (0.0, 5.0), ("fcu", "battery_v"): (12.0, 16.8),
}


def _check_conditions(spec, errs):
    c = spec.get("conditions") or {}
    name = str(c.get("preset", "calm")).lower()
    if name not in tuple(CONDITION_PRESETS) + ("random",):
        errs.append(f"conditions preset '{name}' is not one of "
                    f"{', '.join(tuple(CONDITION_PRESETS) + ('random',))}")
        return
    for group, fields in c.items():
        if group == "preset":
            continue
        if group not in _CALM or not isinstance(fields, dict):
            errs.append(f"conditions: unknown group '{group}'")
            continue
        for k in fields:
            if k not in _CALM[group]:
                errs.append(f"conditions.{group}: unknown field '{k}'")
    resolved = conditions(spec)
    for key in CAMERA_KEYS:
        lo = -5 if key == "exposure" else 0
        if not lo <= resolved["camera"][key] <= 5:
            errs.append(f"conditions.camera.{key} must be {lo}..5")
    for (group, k), (lo, hi) in _CONDITION_LIMITS.items():
        v = resolved[group][k]
        if not lo <= float(v) <= hi:
            errs.append(f"conditions.{group}.{k} = {v} is outside {lo:g}-{hi:g}")


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
LIDAR_ABOVE_BODY_M = 0.0815  # the team airframe's LD06 scan plane above base_link
# The corridors are flown under the board: DuckUnderBoard finds the altitude
# where the scan plane drops below its bottom edge and flies DUCK_MARGIN_M
# lower, never below DUCK_FLOOR_M (mission_tree.DuckUnderBoard).
DUCK_MARGIN_M, DUCK_FLOOR_M = 0.6, 1.2


def corridor_alt(spec):
    """The altitude the corridors are flown at under this spec's banner."""
    bottom = float(spec["banner"]["board_bottom_m"])
    return max(DUCK_FLOOR_M, bottom - LIDAR_ABOVE_BODY_M - DUCK_MARGIN_M)
AIRFRAME_BELOW_BODY_M = 0.35 # legs 0.195 m + sag/altitude error below base_link
PASS_COMFORT_M = 1.6         # the corridor navigator's 1.4 m passage strip + margin


def _lane_inset(c, m):
    """A corridor's footprint shrunk by m on every side."""
    L, W = float(c["length"]), float(c["width"])
    h = W / 2 + WALL_T - m
    yaw = math.radians(c["yaw_deg"])
    return [_place(p, c["x"], c["y"], yaw) for p in
            ((-0.1 + m, -h), (L + 0.1 - m, -h), (L + 0.1 - m, h), (-0.1 + m, h))]


TEAM_AIRFRAME_R_M = 0.36     # hub 0.242 m from centre + 0.12 m prop radius
NAVIGATOR_HALF_STRIP_M = 0.7  # velocity_controller passage_half_width


def lane_bottleneck(c, cell_m=0.05):
    """The widest clearance any path through a lane keeps, in metres.

    Obstacle by obstacle, every gap can be wide enough and the lane still be
    closed: two blocks on opposite sides 1.2 m apart along the lane leave a
    slot the airframe cannot turn through. So the lane is rasterised, every
    cell given its distance to the nearest wall or obstacle, and the answer is
    the best, over all paths from the banner end to the far end, of the
    smallest clearance on the path (a maximin path: cells joined in order of
    falling clearance until the two ends connect).
    """
    L, Wd = float(c["length"]), float(c["width"])
    nu, nv = int(L / cell_m), int(Wd / cell_m)
    polys = []
    for o in c.get("obstacles") or []:
        yaw = math.radians(o.get("yaw_deg", 0.0))
        polys.append(rect_corners(float(o["u"]), float(o["v"]), float(o["w"]),
                                  float(o["d"]), yaw))
    clear = []
    for i in range(nu):
        u = (i + 0.5) * cell_m
        row = []
        for j in range(nv):
            v = -Wd / 2 + (j + 0.5) * cell_m
            d = Wd / 2 - abs(v)
            for poly in polys:
                d = 0.0 if point_in_polygon((u, v), poly) else \
                    min(d, distance_to_edge((u, v), poly))
            row.append(d)
        clear.append(row)
    parent = list(range(nu * nv + 2))
    START, END = nu * nv, nu * nv + 1

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    order = sorted(((clear[i][j], i, j) for i in range(nu) for j in range(nv)),
                   reverse=True)
    seen = set()
    for d, i, j in order:
        k = i * nv + j
        seen.add(k)
        links = [START] if i == 0 else []
        links += [END] if i == nu - 1 else []
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ii, jj = i + di, j + dj
            if 0 <= ii < nu and 0 <= jj < nv and ii * nv + jj in seen:
                links.append(ii * nv + jj)
        for other in links:
            parent[find(k)] = find(other)
        if find(START) == find(END):
            return d
    return 0.0


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
        # alt + LIDAR_ABOVE_BODY_M; an obstacle whose top is below that plane
        # but above the airframe's underside is invisible to the navigator
        # and in the aircraft's way. Found live: a 3.2 m block, 3.23 m scan
        # plane, flown into at 3 m (pitch 48 deg).
        alt = corridor_alt(spec)
        plane = alt + LIDAR_ABOVE_BODY_M
        belly = alt - AIRFRAME_BELOW_BODY_M
        if belly < h < plane + 0.1:
            errs.append(f"obstacle {i} is {h:g} m tall: its top is below the "
                        f"lidar's scan plane ({plane:.2f} m at the {alt:.1f} m "
                        f"corridor altitude under the board) but above the "
                        f"airframe's underside ({belly:.2f} m) -- the aircraft "
                        f"cannot see it and would fly into it. Make it taller "
                        f"than {plane + 0.1:.2f} m or shorter than {belly:.2f} m")
        elif h <= belly:
            warns.append(f"obstacle {i} is {h:g} m tall; the corridor is flown "
                         f"at {alt:.1f} m, so the aircraft passes over it")
    if r.get("obstacles") and not errs:
        room = lane_bottleneck(r)
        if room < TEAM_AIRFRAME_R_M + 0.05:
            errs.append(f"the return corridor has no way through: the widest "
                        f"path keeps {room:.2f} m from every wall and obstacle, "
                        f"and the airframe needs {TEAM_AIRFRAME_R_M:.2f} m plus "
                        f"flying margin -- the obstacles close it between them "
                        f"even where each leaves a gap on its own")
        elif room < NAVIGATOR_HALF_STRIP_M:
            warns.append(f"the return corridor's tightest passage keeps "
                         f"{room:.2f} m either side of the path; the corridor "
                         f"navigator needs {NAVIGATOR_HALF_STRIP_M:.2f} m "
                         f"(velocity_controller passage_half_width) and will "
                         f"stop in front of it")

def validate(spec):
    """(errors, warnings). An error is a world the mission cannot legally fly
    or the simulator cannot build; a warning is legal but worth a second look.
    """
    spec = normalise(spec)
    errs, warns = [], []
    _check_conditions(spec, errs)
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
    bottom = float(spec["banner"]["board_bottom_m"])
    if not BOARD_BOTTOM_RANGE_M[0] <= bottom <= BOARD_BOTTOM_RANGE_M[1]:
        errs.append(f"banner board bottom {bottom:g} m is outside "
                    f"{BOARD_BOTTOM_RANGE_M[0]:g}-{BOARD_BOTTOM_RANGE_M[1]:g} m")
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
