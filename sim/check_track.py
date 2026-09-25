#!/usr/bin/env python3
"""Grade a flown Mission 2 track against the arena's ground truth.

    python3 sim/check_track.py --track /tmp/aerothon_track.csv \
        --layout /tmp/aerothon_arena_layout.json --target b

The mission never reads ground truth; this does, because this is where
knowing the answer is legitimate. It answers the rulebook's scored questions
from the aircraft's OWN recorded track, not from what the mission believed:

    restricted zones   did the airframe ever enter one (minus 5 each)?
    geofence           did it ever leave the organiser's boundary?
    ceiling            did it fly above ~10 m over the delivery zone?
    delivery           how far from the TRUE pad centre was the release?
    landing            how far from the take-off point did it come down?
    flight time        armed-to-disarmed, in simulated seconds, vs 900 s

Exit 0 when every graded item passes, 1 otherwise.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from materialize_world import BANNER_PIVOT, RED_ZONE_SIZES  # noqa: E402
import world_spec  # noqa: E402

# Corridor geometry in the SHIPPED corridor's own frame (mission2.sdf):
# walls x 1.9..12.1, lanes between y 0.25..3.75 (forward) and -3.75..-0.25
# (return), wall tops at 3.6 m. A randomised arena moves and turns the whole
# corridor rigidly about the forward banner; the grader undoes that move.
CORRIDOR_X = (1.9, 12.1)
FORWARD_LANE_Y = (0.25, 3.75)
RETURN_LANE_Y = (-3.75, -0.25)
WALL_TOP_M = 3.6
MAX_TILT_DEG = 30.0          # past this is a collision or a loss of control
FLIGHT_MODES = ("GUIDED", "LAND")

# Iris: 0.25 m arm + 0.127 m (10 in) prop radius. 0.3 let seed 1004 "pass"
# at 0.31 m from the paint with its props over the edge.
AIRFRAME_RADIUS_M = 0.4
CEILING_M = 10.5             # "approximately 10 m", with 0.5 m tolerance
DROP_TOL_M = 1.0             # a delivery inside 1 m of the pad centre passes
LAND_TOL_M = 1.5             # landed on the 5 x 8 m take-off pad, near home


def load_track(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            row = {"t": float(r["t_sim"]), "x": float(r["x"]),
                   "y": float(r["y"]), "z": float(r["z"]),
                   "armed": r["armed"] == "1", "state": r["state"]}
            if r.get("roll_deg") not in (None, ""):
                row["roll"] = float(r["roll_deg"])
                row["pitch"] = float(r["pitch_deg"])
                row["mode"] = r.get("mode", "")
            if r.get("payload_wx") not in (None, ""):
                row["pay"] = (float(r["payload_wx"]), float(r["payload_wy"]),
                              float(r["payload_wz"]))
            rows.append(row)
    return rows


def to_corridor(p, layout):
    """Home-local track point -> the shipped corridor's own frame."""
    hx, hy = layout.get("home_world", [0.0, 0.0])
    gx, gy, gyaw = layout.get("gate", [BANNER_PIVOT[0], BANNER_PIVOT[1], 0.0])
    wx, wy = p["x"] + hx, p["y"] + hy
    dx, dy = wx - gx, wy - gy
    c, s = math.cos(-gyaw), math.sin(-gyaw)
    return (BANNER_PIVOT[0] + dx * c - dy * s,
            BANNER_PIVOT[1] + dx * s + dy * c)


def lanes(layout):
    """(outbound, return) lanes as (x, y, yaw, length, width, wall height),
    WORLD frame, origin at each lane's banner and +x down it.

    A user-built arena records them ("corridors"). Older layouts only have the
    gate, and the shipped corridor is one rigid body about it: the outbound
    lane starts at the banner and the return lane's banner is 10 m down and
    4 m to starboard, pointing back.
    """
    if layout.get("corridors"):
        c = layout["corridors"]
        return tuple(tuple(c[k]) for k in ("outbound", "return"))
    gx, gy, gyaw = layout.get("gate", [BANNER_PIVOT[0], BANNER_PIVOT[1], 0.0])
    cs, sn = math.cos(gyaw), math.sin(gyaw)
    rx, ry = gx + 10.0 * cs + 4.0 * sn, gy + 10.0 * sn - 4.0 * cs
    return ((gx, gy, gyaw, 10.0, 3.5, WALL_TOP_M),
            (rx, ry, gyaw + math.pi, 10.0, 3.5, WALL_TOP_M))


def to_lane(p, layout, lane):
    """Home-local track point -> (along, across) in a lane's own frame."""
    hx, hy = layout.get("home_world", [0.0, 0.0])
    x0, y0, yaw = lane[:3]
    dx, dy = p["x"] + hx - x0, p["y"] + hy - y0
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def traversal(track, layout, start_i, outbound):
    """Find one lane traversal from `start_i`. Returns (dict, next_index).

    A traversal runs from crossing 0.5 m inside the lane's banner end to
    0.5 m short of its far end. Every sample on the way must be INSIDE that
    lane, between its walls and below their tops -- over the corridor or
    beside it is not through it.
    """
    lane = lanes(layout)[0 if outbound else 1]
    L, W, H = lane[3], lane[4], lane[5]
    enter_u, leave_u = 0.5, L - 0.5
    i, prev = start_i, None
    while i < len(track):
        u, _ = to_lane(track[i], layout, lane)
        if prev is not None and prev < enter_u <= u:
            break
        prev = u
        i += 1
    else:
        return {"found": False, "why": "never entered the lane"}, len(track)
    begin = i
    bad = []
    while i < len(track):
        p = track[i]
        u, v = to_lane(p, layout, lane)
        if u >= leave_u:
            return ({"found": True, "t": (track[begin]["t"], p["t"]),
                     "violations": bad[:5], "n_bad": len(bad)}, i)
        if 0.0 <= u <= L:
            if abs(v) > W / 2:
                bad.append({"t": p["t"], "why": f"outside the lane (across {v:+.2f})"})
            elif not (0.3 < p["z"] < H):
                bad.append({"t": p["t"], "why": f"z {p['z']:.2f} not below the "
                                                 "wall tops"})
        i += 1
    return {"found": False, "why": "entered but never came out the far end",
            "violations": bad[:5]}, len(track)


def rect_gap(poly, x, y):
    """Distance from (x, y) to a red zone's polygon; 0 inside it."""
    if world_spec.point_in_polygon((x, y), poly):
        return 0.0
    return world_spec.distance_to_edge((x, y), poly)


def red_polygons(layout):
    """Every red zone as a home-local polygon.

    A zone is [cx, cy] (a shipped/randomised zone, sized by name) or
    [cx, cy, w, h, yaw] (a user-built arena, any size and heading).
    """
    hx, hy = layout.get("home_world", [0.0, 0.0])
    out = []
    for name, v in layout.get("red_zones", {}).items():
        if len(v) >= 4:
            cx, cy, w, h = v[:4]
            yaw = v[4] if len(v) > 4 else 0.0
        else:
            cx, cy = v
            w, h = RED_ZONE_SIZES[name]
            yaw = 0.0
        out.append((name, world_spec.rect_corners(cx - hx, cy - hy, w, h, yaw)))
    return out


def fence_polygon(layout):
    """The geofence, home-local: the polygon if given, else the rectangle."""
    if layout.get("geofence_poly"):
        return [tuple(p) for p in layout["geofence_poly"]]
    fx0, fx1, fy0, fy1 = layout["geofence_rect"]
    return [(fx0, fy0), (fx1, fy0), (fx1, fy1), (fx0, fy1)]


def grade(track, layout, target=None):
    hx, hy = layout.get("home_world", [0.0, 0.0])
    reds = red_polygons(layout)
    fence = fence_polygon(layout)
    zx, zy, zw, zh = layout["delivery_zone_rect"]
    zone = (zx - zw / 2, zx + zw / 2, zy - zh / 2, zy + zh / 2)

    flying = [p for p in track if p["armed"] and p["z"] > 0.3]
    out = {"samples": len(track), "airborne_samples": len(flying)}

    # Restricted zones: count ENTRIES (contiguous runs), not samples.
    entries = []
    inside = {}
    for p in flying:
        for name, rect in reds:
            now = rect_gap(rect, p["x"], p["y"]) < AIRFRAME_RADIUS_M
            if now and not inside.get(name):
                entries.append({"zone": name, "t": p["t"], "at": (round(p["x"], 1),
                                round(p["y"], 1), round(p["z"], 1)),
                                "state": p["state"]})
            inside[name] = now
    out["red_zone_entries"] = entries
    out["closest_red_m"] = (round(min(rect_gap(r, p["x"], p["y"])
                                      for p in flying for _, r in reds), 2)
                            if flying and reds else None)

    out["geofence_exits"] = [
        {"t": p["t"], "at": (round(p["x"], 1), round(p["y"], 1))}
        for p in flying if not world_spec.point_in_polygon((p["x"], p["y"]), fence)]
    in_zone = [p for p in flying
               if zone[0] <= p["x"] <= zone[1] and zone[2] <= p["y"] <= zone[3]]
    out["max_alt_in_zone_m"] = (round(max(p["z"] for p in in_zone), 2)
                                if in_zone else None)
    out["max_alt_m"] = round(max(p["z"] for p in flying), 2) if flying else None

    armed_t = [p["t"] for p in track if p["armed"]]
    out["flight_time_s"] = (round(armed_t[-1] - armed_t[0], 1)
                            if armed_t else None)

    drops = [p for p in track if p["state"] == "WINCH_DROP"]
    if target and drops:
        px, py = layout["pads"][target.lower()]
        px, py = px - hx, py - hy
        # The release happens at the bottom of the drop, hovering at the drop
        # altitude: take the lowest WINCH_DROP samples.
        low = min(p["z"] for p in drops)
        at = [p for p in drops if p["z"] <= low + 0.3]
        mx = sum(p["x"] for p in at) / len(at)
        my = sum(p["y"] for p in at) / len(at)
        out["drop_error_m"] = round(math.hypot(mx - px, my - py), 2)
        out["drop_alt_m"] = round(sum(p["z"] for p in at) / len(at), 2)
    if track:
        last = track[-1]
        out["landed_from_home_m"] = round(math.hypot(last["x"], last["y"]), 2)
        out["final_armed"] = last["armed"]

    # ---- through the corridor, not over it or round it ---- #
    fwd, nxt = traversal(track, layout, 0, outbound=True)
    ret, _ = traversal(track, layout, nxt, outbound=False)
    out["corridor_outbound"] = fwd
    out["corridor_return"] = ret
    over = []
    for p in flying:
        for lane in lanes(layout):
            u, v = to_lane(p, layout, lane)
            if (-0.1 <= u <= lane[3] + 0.1 and abs(v) <= lane[4] / 2 + 0.1
                    and p["z"] >= lane[5]):
                over.append(p["t"])
                break
    out["over_corridor_samples"] = len(over)

    # ---- the return lane's obstacles: never touched ---- #
    obst = [([(x - hx, y - hy) for x, y in o["poly"]], o["h"])
            for o in layout.get("obstacles", [])]
    if obst:
        near = [rect_gap(poly, p["x"], p["y"]) for p in flying
                for poly, h in obst if p["z"] < h + 0.3]
        out["closest_obstacle_m"] = round(min(near), 2) if near else None

    # ---- in control the whole way ---- #
    tilted = [p for p in flying if "roll" in p
              and max(abs(p["roll"]), abs(p["pitch"])) > MAX_TILT_DEG]
    out["max_tilt_deg"] = (round(max(max(abs(p["roll"]), abs(p["pitch"]))
                                     for p in flying if "roll" in p), 1)
                           if any("roll" in p for p in flying) else None)
    out["tilt_events"] = [{"t": p["t"], "roll": p["roll"], "pitch": p["pitch"],
                           "state": p["state"]} for p in tilted[:5]]
    armed_modes = [p for p in track if p["armed"] and p.get("mode")]
    out["unexpected_modes"] = sorted({p["mode"] for p in armed_modes
                                      if p["mode"] not in FLIGHT_MODES})
    # LAND belongs at the end; LAND mid-mission is a failsafe.
    lands = [p["t"] for p in armed_modes if p["mode"] == "LAND"]
    t_end = armed_t[-1] if armed_t else 0.0
    out["early_land"] = bool(lands) and min(lands) < t_end - 90.0

    checks = {
        "no red-zone entry": not entries,
        "inside geofence": not out["geofence_exits"],
        "outbound lane flown THROUGH the corridor": (
            fwd.get("found", False) and not fwd.get("n_bad")),
        "return lane flown THROUGH the corridor": (
            ret.get("found", False) and not ret.get("n_bad")),
        "never above the corridor": not over,
        # No attitude recorded is not "no tilt": an old track cannot pass.
        "no tilt beyond 30 deg": out["max_tilt_deg"] is not None and not tilted,
        "no failsafe / unexpected flight mode": (
            not out["unexpected_modes"] and not out["early_land"]),
        "ceiling over zone": (out["max_alt_in_zone_m"] is None
                              or out["max_alt_in_zone_m"] <= CEILING_M),
        "flight within 900 s": (out["flight_time_s"] is not None
                                and out["flight_time_s"] <= 900.0),
    }
    if out.get("closest_obstacle_m") is not None:
        checks["no obstacle contact"] = out["closest_obstacle_m"] >= AIRFRAME_RADIUS_M
    if "drop_error_m" in out:
        checks["drop within 1 m of pad"] = out["drop_error_m"] <= DROP_TOL_M
    if "landed_from_home_m" in out:
        checks["landed at take-off point"] = (
            out["landed_from_home_m"] <= LAND_TOL_M and not out["final_armed"])

    # ---- the PAYLOAD itself (Gazebo ground truth), when it was recorded ---- #
    # The drone being over the pad is not a delivery; the payload lying on it
    # is. Released = it ended on the ground and away from where the aircraft
    # finished (which is home, not the pad): a payload still on the hook comes
    # home with the aircraft.
    pays = [p for p in track if p.get("pay")]
    if pays and track:
        px, py, pz = pays[-1]["pay"]
        px, py = px - hx, py - hy
        last = track[-1]
        apart = math.hypot(px - last["x"], py - last["y"])
        out["payload_final"] = [round(px, 2), round(py, 2), round(pz, 3)]
        out["payload_from_aircraft_m"] = round(apart, 2)
        checks["payload released (on the ground, off the aircraft)"] = (
            pz < 0.3 and apart > 2.0)
        if target:
            tx, ty = layout["pads"][target.lower()]
            err = math.hypot(px - (tx - hx), py - (ty - hy))
            out["payload_error_m"] = round(err, 2)
            checks["payload on the target pad (within 1 m)"] = err <= DROP_TOL_M
    out["checks"] = checks
    out["pass"] = all(checks.values())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", required=True)
    ap.add_argument("--layout", required=True)
    ap.add_argument("--target", default=None, help="delivery pad letter")
    args = ap.parse_args()
    layout = json.loads(Path(args.layout).read_text(encoding="utf-8"))
    res = grade(load_track(args.track), layout, args.target)
    print(json.dumps(res, indent=2))
    for name, ok in res["checks"].items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
