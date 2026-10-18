#!/usr/bin/env python3
"""Build a simulation world from the arena dimensions the operator enters.

The aircraft gets the arena's dimensions only from the GCS (gcs_operator.py
applies a saved preset). The simulated arena must be built from the SAME
values, or the run tests an aircraft told one arena while flying another. So
the world comes from the preset, laid out on a layout world's positions:

    python3 scripts/world_from_dimensions.py --dims out/dims.json \
        --layout sim/worlds/banner_both.json --out sim/worlds/clarified_both.json

What the organisers' clarifications fix, and how each lands in the world:

    corridors   side by side with a gap (linked); length, width, wall height
    banner      the corridor's width; hung from a rod between the walls with
                its top at the wall top, or laid in front of the mouth, or
                both; green where the aircraft enters, ORANGE at every other
                opening
    obstacles   in the exit (return) corridor, each reaching about half the
                corridor width from a wall, alternating sides
    red zone    one, of the given side, at the layout's first zone
    QR          the start and target sizes

The layout keeps the take-off, delivery zone and pads. The delivery zone is
moved clear of the corridor ends when it would begin within ZONE_CLEAR_M of
them: a laid sheet on the zone's grass merged with it, and the return
search stands 5 m off the gate on the zone side and looks down (issue 44).
"""
import argparse
import copy
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src/aerothon_mission/mission_bt"))

import world_spec as W                                  # noqa: E402
from mission_bt import arena_dimensions as A            # noqa: E402

ZONE_CLEAR_M = 9.0
OBSTACLE_ALONG_M = 0.35
OBSTACLE_HEIGHT_FRAC = 0.95         # of the wall height


def load_dims(args):
    if args.dims:
        with open(args.dims, encoding="utf-8") as f:
            data = json.load(f)
        return A.validate(data.get("dimensions", data))
    path = os.path.expanduser(args.presets_file)
    saved = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            saved = json.load(f)
    if args.preset in saved:
        return A.validate(saved[args.preset])
    if args.preset in A.BUILTIN_PRESETS:
        return A.validate(A.BUILTIN_PRESETS[args.preset])
    raise SystemExit(f"no preset {args.preset!r} in {path} or the built-ins")


def obstacles(length, width, wall_height, reach_frac=0.5):
    """Alternating blocks, each reaching half the corridor's width from a wall.

    Spaced as the shipped slalom is, 2.4 m apart from 1.4 m in -- not
    squeezed to fit, which closed a 5 m corridor -- so a 5 m corridor gets two
    and a 10 m one four.
    """
    reach = width * float(reach_frac)
    v = width / 2.0 - reach / 2.0
    out, u, side = [], 1.4, 1.0
    step = 2.4
    while u <= length - 1.0:
        out.append({"u": round(u, 3), "v": round(side * v, 3), "w": OBSTACLE_ALONG_M,
                    "d": round(reach, 3), "h": round(OBSTACLE_HEIGHT_FRAC * wall_height, 3),
                    "yaw_deg": 0.0})
        u += step
        side = -side
    return out


def build(dims, layout, name, colours=None):
    d = A.validate(dims)
    c, b, q, rz = d["corridor"], d["banner"], d["qr"], d["red_zone"]
    spec = copy.deepcopy(layout)
    spec["name"] = name
    lane = {"length": c["length_m"], "width": c["width_m"],
            "wall_height": c["wall_height_m"]}
    spec["corridor"] = {**{k: spec["corridor"][k] for k in ("x", "y", "yaw_deg")}, **lane}
    spec["return_corridor"] = {"linked": True, **lane,
                               "obstacles": obstacles(c["length_m"], c["width_m"],
                                                      c["wall_height_m"],
                                                      d["obstacle"]["reach_frac"])}
    # Rod-hung: the banner's top is the wall top.
    spec["banner"] = {
        "board_bottom_m": round(c["wall_height_m"] - b["height_m"], 4),
        "width_m": b["width_m"], "height_m": b["height_m"], "mount": "rod",
        "placement": b["placement"], "orange": True, "lettered": False,
        "colours": dict(colours or (layout.get("banner") or {}).get("colours") or {})}
    spec["qr"] = {"start_m": q["start_m"], "target_m": q["target_m"]}
    first = (layout.get("red_zones") or [{"x": 38.0, "y": 5.0}])[0]
    spec["red_zones"] = [{"x": first["x"], "y": first["y"], "w": rz["side_m"],
                          "h": rz["side_m"], "yaw_deg": 0.0}]
    spec["decoys"] = []

    # Keep the delivery zone clear of the corridor ends.
    norm = W.normalise(spec)
    ends = [W._place((float(l["length"]) * t, 0.0), l["x"], l["y"],
                     math.radians(l["yaw_deg"]))
            for l in (norm["corridor"], norm["return_corridor"]) for t in (0.0, 1.0)]
    z = spec["delivery_zone"]
    near_edge = float(z["x"]) - float(z["w"]) / 2.0
    far_end = max(x for x, _ in ends)
    shift = max(0.0, far_end + ZONE_CLEAR_M - near_edge)
    if shift > 0.0:
        z["x"] = round(float(z["x"]) + shift, 3)
        for pad in spec["pads"].values():
            pad["x"] = round(float(pad["x"]) + shift, 3)
        for r in spec["red_zones"]:
            r["x"] = round(float(r["x"]) + shift, 3)
    spec["_note"] = (f"Built by world_from_dimensions.py from the GCS dimensions "
                     f"(banner {b['width_m']:g} x {b['height_m']:g} m, {b['placement']}; "
                     f"QR {q['target_m']:g} m; red zone {rz['side_m']:.3f} m; walls "
                     f"{c['wall_height_m']:.3f} m; navigation {c['nav_height_m']:.3f} m)"
                     + (f"; delivery zone moved {shift:.1f} m clear of the corridors"
                        if shift else ""))
    return spec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--dims", help="dimensions JSON (as the GCS sends them)")
    src.add_argument("--preset", help="a saved or built-in preset name")
    ap.add_argument("--presets-file", default="~/.aerothon/dimension_presets.json")
    ap.add_argument("--layout", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--name", default=None)
    a = ap.parse_args(argv)
    layout = json.loads(a.layout.read_text(encoding="utf-8"))
    spec = build(load_dims(a), layout, a.name or a.out.stem)
    errs, warns = W.validate(W.normalise(spec))
    for w in warns:
        print(f"warning: {w}")
    if errs:
        raise SystemExit("REFUSED: " + "; ".join(errs))
    a.out.write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {a.out}: {spec['_note']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
