#!/usr/bin/env python3
"""Resolve runtime asset URIs in the competition SDF.

Gazebo is deliberately started by the master shell instead of as a child of
ROS launch. On the target workstation this avoids a Gazebo transport startup
stall and lets the launcher gate vehicle spawning on /gazebo/worlds.
"""

from argparse import ArgumentParser
from pathlib import Path
import json
import math
import os
import random
import shutil
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import world_spec  # noqa: E402


WHITE = "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>"
BLACK = "<material><ambient>0.002 0.002 0.002 1</ambient><diffuse>0.002 0.002 0.002 1</diffuse></material>"
GREEN = "<material><ambient>0.01 0.48 0.18 1</ambient><diffuse>0.01 0.62 0.24 1</diffuse></material>"

# Compact 5x7 block font used for signs. Geometry, unlike synchronized PBR
# textures, renders reliably in both the Gazebo GUI and simulated camera.
FONT = {
    "A": ["01110","10001","10001","11111","10001","10001","10001"],
    "B": ["11110","10001","10001","11110","10001","10001","11110"],
    "D": ["11110","10001","10001","10001","10001","10001","11110"],
    "E": ["11111","10000","10000","11110","10000","10000","11111"],
    "H": ["10001","10001","10001","11111","10001","10001","10001"],
    "N": ["10001","11001","11001","10101","10011","10011","10001"],
    "O": ["01110","10001","10001","10001","10001","10001","01110"],
    "R": ["11110","10001","10001","11110","10100","10010","10001"],
    "T": ["11111","00100","00100","00100","00100","00100","00100"],
    "Z": ["11111","00001","00010","00100","01000","10000","11111"],
    "0": ["01110","10001","10011","10101","11001","10001","01110"],
    "2": ["01110","10001","00001","00010","00100","01000","11111"],
    "6": ["00110","01000","10000","11110","10001","10001","01110"],
    " ": ["00000"] * 7,
}


def box_visual(name: str, pose: str, size: str, material: str) -> str:
    if len(pose.split()) == 3:
        pose = f"{pose} 0 0 0"
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}'
            f'</size></box></geometry>{material}</visual>')


def box_collision(name: str, pose: str, size: str) -> str:
    """A box the LIDAR can see. Visuals are invisible to a ray sensor.

    The banner board was emitted as a visual and nothing else, so the only
    solid parts of the gate were its two 18 cm posts and the ray sensor swept
    straight through the panel between them. A real banner is a surface; a
    simulator that models it as a hole is not modelling the thing being flown
    against.
    """
    if len(pose.split()) == 3:
        pose = f"{pose} 0 0 0"
    return (f'<collision name="{name}"><pose>{pose}</pose><geometry><box>'
            f'<size>{size}</size></box></geometry></collision>')


def qr_visuals(matrix: list[list[bool]], size: float, prefix: str) -> str:
    """White plate plus horizontally merged black QR module runs."""
    n = len(matrix)
    cell = size / n
    visuals = [box_visual(f"{prefix}_white_plate", "0 0 0", f"{size} {size} 0.04", WHITE)]
    idx = 0
    for row, modules in enumerate(matrix):
        col = 0
        while col < n:
            if not modules[col]:
                col += 1
                continue
            start = col
            while col < n and modules[col]:
                col += 1
            run = col - start
            x = -size / 2 + (start + run / 2) * cell
            y = size / 2 - (row + 0.5) * cell
            visuals.append(box_visual(
                f"{prefix}_black_{idx}", f"{x:.6f} {y:.6f} 0.026",
                f"{run * cell:.6f} {cell:.6f} 0.012", BLACK))
            idx += 1
    return "\n".join(visuals)


def bitmap_runs(text: str):
    rows = ["" for _ in range(7)]
    for char in text.upper():
        glyph = FONT[char]
        for row in range(7):
            rows[row] += glyph[row] + "0"
    return rows


def green_decoy_visuals() -> str:
    """Green things that are NOT the banner.

    perception_banner gates only on blob size and aspect, so any green
    rectangle passes and the GCS reports ALIGNED. With nothing green in the
    world except the banner itself, an identity check that simply returned
    True would pass every simulated test — a test measuring itself.

    These are the simulator stand-in for the physical decoy photographs
    (tarpaulin, grass, a green vehicle) that need a camera to capture. They are
    deliberately banner-like in colour and roughly plausible in size, so a
    detector that only looks at "is it a big green rectangle" WILL be fooled.
    """
    return "".join([
        # Broad grass-like green apron, low and wide.
        box_visual("decoy_grass_apron", "0 0 0.02", "9.0 5.0 0.04", GREEN),
        # A green tarpaulin propped roughly banner-sized and banner-shaped.
        box_visual("decoy_tarp", "0 0 1.30", "0.10 3.4 1.05", GREEN),
        # A smaller green panel at a different aspect ratio.
        box_visual("decoy_panel", "0 2.6 0.90", "0.10 1.2 1.20", GREEN),
    ])


def banner_geometry() -> str:
    """The lettered panel, as both a visual and a SOLID.

    WHY THE COLLISION IS PART OF THE BANNER AND NOT AN AFTERTHOUGHT

        Squareness to the gate is measured with the lidar: fit a line through
        the returns in the sector the camera points at, and the angle of that
        line to the nose is the misalignment. That measurement is only as good
        as what the ray sensor can hit. With the board emitted as a visual
        only, the gate presented two 18 cm posts and 3.7 m of nothing between
        them, which is not what a banner presents to a real C1.

        The line fit is written to work on either -- two isolated posts still
        define a line -- but the simulator should model the surface, not a
        hollow frame.
    """
    text = "AEROTHON"
    rows = bitmap_runs(text)
    cols = len(rows[0])
    cell_y, cell_z = 3.25 / cols, 0.115
    center_z = 3.38
    visuals = [
        box_collision("banner_board_collision", f"0 0 {center_z}",
                      "0.12 3.7 1.15"),
        box_visual("banner_board", f"0 0 {center_z}", "0.12 3.7 1.15", GREEN),
    ]
    # Raised white frame on both faces.
    for face, x in (("front", -0.071), ("back", 0.071)):
        visuals.extend([
            box_visual(f"banner_{face}_frame_top", f"{x} 0 {center_z + 0.50}", "0.022 3.48 0.07", WHITE),
            box_visual(f"banner_{face}_frame_bottom", f"{x} 0 {center_z - 0.50}", "0.022 3.48 0.07", WHITE),
            box_visual(f"banner_{face}_frame_left", f"{x} 1.705 {center_z}", "0.022 0.07 1.07", WHITE),
            box_visual(f"banner_{face}_frame_right", f"{x} -1.705 {center_z}", "0.022 0.07 1.07", WHITE),
        ])
    idx = 0
    for face, x, mirror in (("front", -0.071, False), ("back", 0.071, True)):
        for row, bits in enumerate(rows):
            source = bits[::-1] if mirror else bits
            col = 0
            while col < cols:
                if source[col] != "1":
                    col += 1
                    continue
                start = col
                while col < cols and source[col] == "1":
                    col += 1
                run = col - start
                y = 1.625 - (start + run / 2) * cell_y
                z = center_z + (3 - row) * cell_z
                visuals.append(box_visual(
                    f"banner_{face}_{idx}", f"{x:.3f} {y:.6f} {z:.6f}",
                    f"0.022 {run * cell_y:.6f} {cell_z:.6f}", WHITE))
                idx += 1
    return "\n".join(visuals)


def red_zone_visuals(width: float, height: float, prefix: str) -> str:
    """Raised white border and RED ZONE block text over the existing red base."""
    t = min(width, height) * 0.055
    z = 0.066
    visuals = [
        box_visual(f"{prefix}_border_n", f"0 {height/2-t/2:.4f} {z}", f"{width} {t} 0.022", WHITE),
        box_visual(f"{prefix}_border_s", f"0 {-height/2+t/2:.4f} {z}", f"{width} {t} 0.022", WHITE),
        box_visual(f"{prefix}_border_e", f"{width/2-t/2:.4f} 0 {z}", f"{t} {height} 0.022", WHITE),
        box_visual(f"{prefix}_border_w", f"{-width/2+t/2:.4f} 0 {z}", f"{t} {height} 0.022", WHITE),
    ]
    rows = bitmap_runs("RED ZONE")
    cols = len(rows[0])
    cell_x = width * 0.68 / cols
    cell_y = min(height * 0.09, cell_x * 1.35)
    idx = 0
    for row, bits in enumerate(rows):
        col = 0
        while col < cols:
            if bits[col] != "1":
                col += 1
                continue
            start = col
            while col < cols and bits[col] == "1":
                col += 1
            run = col - start
            x = -width * 0.34 + (start + run / 2) * cell_x
            y = (3 - row) * cell_y
            visuals.append(box_visual(
                f"{prefix}_text_{idx}", f"{x:.6f} {y:.6f} {z}",
                f"{run * cell_x:.6f} {cell_y:.6f} 0.022", WHITE))
            idx += 1
    return "\n".join(visuals)


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--assets", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    # MARKER SIZE IS THE DOMINANT UNKNOWN.
    #
    # The competition QR size has not been confirmed by the organisers, and it
    # is the dominant term in the search-altitude calculation: reliable
    # decoding needs a fixed number of pixels per QR module, so max stand-off
    # scales linearly with marker size (docs/QR_DECODE_ENVELOPE.md).
    #
    # The pads were previously hardcoded at 2.2 m (start) and 3.0 m (delivery),
    # which are far larger than any plausible real marker and quietly made the
    # simulation easy. Making size a parameter turns an external blocker into a
    # sweepable variable: the search strategy can be validated across the whole
    # plausible range now, and the real number just picks a point in it.
    parser.add_argument("--start-qr-size", type=float,
                        default=float(os.environ.get("AEROTHON_START_QR_M", 2.2)),
                        help="start pad edge length in metres")
    parser.add_argument("--target-qr-size", type=float,
                        default=float(os.environ.get("AEROTHON_TARGET_QR_M", 3.0)),
                        help="delivery pad edge length in metres")
    # WHICH delivery pad the start QR names.
    #
    # qr_start.png and qr_target_a.png were generated with the SAME payload, so
    # "the drone matched the target" was satisfied by a fixture coincidence:
    # any decode of any start pad matched delivery pad A, which is also the
    # first pad the lawnmower reaches. The search was never actually tested.
    #
    # The start pad now carries a CHOSEN target's payload, defaulting to a
    # random one per run, so the mission has to read it and then find that
    # specific pad. Pin it with AEROTHON_START_TARGET=C for a repeatable run.
    #
    # This reuses the existing matrices in qr_matrices.json rather than
    # regenerating PNGs, because the `qrcode` package is not installed here.
    parser.add_argument("--start-target", default=os.environ.get(
                            "AEROTHON_START_TARGET", "random"),
                        help="which delivery target the start pad names: "
                             "a|b|c|d|e or 'random'")
    parser.add_argument("--seed", type=int,
                        default=int(os.environ.get("AEROTHON_SEED", 0)),
                        help="seed for the random start target (0 = clock)")
    parser.add_argument("--randomise-arena", action="store_true",
                        default=os.environ.get("AEROTHON_RANDOM_ARENA", "0") == "1",
                        help="PHASE 11. Move the corridor, the delivery zone, "
                             "the red zones and the target pads to new places "
                             "for this run. If any hardcoded geometry survives "
                             "anywhere in the stack, this is what finds it: a "
                             "genuinely perception-driven mission does not care "
                             "which arena it is in.")
    parser.add_argument("--layout-out", type=Path, default=None,
                        help="write this arena's delivery-zone rectangle and "
                             "arena geofence as JSON")
    parser.add_argument("--world-spec", type=Path,
                        default=(Path(os.environ["AEROTHON_WORLD_SPEC"])
                                 if os.environ.get("AEROTHON_WORLD_SPEC") else None),
                        help="a user-built arena (tools/world_editor): every "
                             "position, size and heading comes from this file. "
                             "Overrides --randomise-arena, the start target "
                             "and the QR sizes.")
    args = parser.parse_args()

    # Ogre cannot resolve file:// URIs whose path contains spaces, even when
    # percent-encoded. Mirror the small generated texture set into /tmp.
    runtime_assets = Path(tempfile.gettempdir()) / "aerothon_m2_assets"
    runtime_assets.mkdir(parents=True, exist_ok=True)
    for asset in args.assets.iterdir():
        if asset.is_file():
            shutil.copy2(asset, runtime_assets / asset.name)

    world = args.source.read_text(encoding="utf-8")
    arena = None
    spec = None
    if args.world_spec:
        spec = world_spec.load(args.world_spec)
        errs, warns = world_spec.validate(spec)
        for w in warns:
            print(f"WORLD SPEC warning: {w}")
        if errs:
            raise SystemExit("WORLD SPEC refused:\n  " + "\n  ".join(errs))
        world, arena = apply_spec(world, spec)
        args.start_target = spec["start_target"]
        args.start_qr_size = float(spec["qr"]["start_m"])
        args.target_qr_size = float(spec["qr"]["target_m"])
        print("CUSTOM ARENA: " + json.dumps(arena, sort_keys=True))
    elif args.randomise_arena:
        arena = randomise_arena(world, random.Random(args.seed or None))
        world = arena.pop("world")
        print("RANDOMISED ARENA: " + json.dumps(arena, sort_keys=True))
    layout = arena if arena is not None else shipped_layout()
    # WORLD -> HOME-LOCAL. The mission's local frame is anchored at the FCU
    # home, which is where the vehicle spawns -- (-2, 2) in the shipped world,
    # not the world origin. Publishing world coordinates as local shifted the
    # delivery field and the geofence 2 m off the painted ones.
    layout = to_home_frame(layout, spawn_xy(world))
    if args.layout_out:
        # The organiser inputs for THIS arena (delivery-zone rectangle and
        # arena geofence), for the launcher to publish. Written whether or
        # not the arena was randomised, so the two cases cannot diverge.
        args.layout_out.write_text(json.dumps(layout, sort_keys=True),
                                   encoding="utf-8")
    matrices_path = args.assets / "qr_matrices.json"
    if not matrices_path.exists():
        raise SystemExit(f"Missing {matrices_path}; run generate_competition_assets.py")
    matrices = json.loads(matrices_path.read_text(encoding="utf-8"))

    choices = ["a", "b", "c", "d", "e"]
    if args.start_target.lower() in choices:
        start_letter = args.start_target.lower()
    else:
        rng = random.Random(args.seed or None)
        start_letter = rng.choice(choices)
    start_matrix = matrices[f"qr_target_{start_letter}.png"]
    payloads = (args.assets / "qr_payloads.txt").read_text(encoding="utf-8")
    start_payload = next(
        (line.split(":", 1)[1].strip() for line in payloads.splitlines()
         if line.startswith(f"qr_target_{start_letter}.png")), "?")
    print(f"start pad names delivery target {start_letter.upper()} "
          f"({start_payload})")
    replacements = {
        "@QR_START_VISUALS@": qr_visuals(start_matrix,
                                        args.start_qr_size, "start_qr"),
        "@QR_TARGET_A_VISUALS@": qr_visuals(matrices["qr_target_a.png"],
                                            args.target_qr_size, "target_a"),
        "@QR_TARGET_B_VISUALS@": qr_visuals(matrices["qr_target_b.png"],
                                            args.target_qr_size, "target_b"),
        "@QR_TARGET_C_VISUALS@": qr_visuals(matrices["qr_target_c.png"],
                                            args.target_qr_size, "target_c"),
        "@QR_TARGET_D_VISUALS@": qr_visuals(matrices["qr_target_d.png"],
                                            args.target_qr_size, "target_d"),
        "@QR_TARGET_E_VISUALS@": qr_visuals(matrices["qr_target_e.png"],
                                            args.target_qr_size, "target_e"),
        "@AEROTHON_BANNER_GEOMETRY@": banner_geometry(),
        "@GREEN_DECOY_VISUALS@": green_decoy_visuals(),
        "@RED_ZONE_MAIN_VISUALS@": red_zone_visuals(10.0, 7.0, "red_main"),
        "@RED_ZONE_NW_VISUALS@": red_zone_visuals(6.0, 4.0, "red_nw"),
        "@RED_ZONE_SOUTH_VISUALS@": red_zone_visuals(7.0, 4.0, "red_south"),
    }
    # A spec replaces the template's red zones and decoys with its own (any
    # number, including none), so their tokens may legitimately be gone.
    optional = ({"@RED_ZONE_MAIN_VISUALS@", "@RED_ZONE_NW_VISUALS@",
                 "@RED_ZONE_SOUTH_VISUALS@", "@GREEN_DECOY_VISUALS@"}
                if spec is not None else set())
    for token, geometry in replacements.items():
        if token not in world:
            if token in optional:
                continue
            raise SystemExit(f"World is missing geometry token {token}")
        world = world.replace(token, geometry)
    world = world.replace("@SIM_GAZEBO_ASSET_URI@", runtime_assets.as_uri())
    args.output.write_text(world, encoding="utf-8")


# --------------------------------------------------------------------------- #
# Phase 11 — randomised arenas
# --------------------------------------------------------------------------- #

def _set_pose(world: str, model: str, x, y, yaw=None) -> tuple[str, tuple]:
    """Rewrite the <pose> of a named model, keeping z/roll/pitch.

    Operates on the SDF text rather than a parsed tree because the world is a
    template full of @TOKEN@ placeholders that no XML parser will accept until
    they have been substituted.
    """
    marker = f'<model name="{model}">'
    i = world.index(marker)
    j = world.index("</model>", i)
    block = world[i:j]
    a = block.rindex("<pose>")
    b = block.index("</pose>", a)
    parts = block[a + 6:b].split()
    while len(parts) < 6:
        parts.append("0")
    old = tuple(float(v) for v in parts[:6])
    parts[0] = f"{x:.3f}"
    parts[1] = f"{y:.3f}"
    if yaw is not None:
        parts[5] = f"{yaw:.4f}"
    new_block = block[:a + 6] + " ".join(parts) + block[b:]
    return world[:i] + new_block + world[j:], old


def _rigid_move(world, model, dx, dy, dyaw, pivot):
    """Translate + rotate a model about `pivot`, KEEPING its offset from it.

    WHY THIS IS NOT JUST _set_pose

        The corridor is three separate models -- the banner, the wall block
        and the green surround -- and they are NOT co-located. In the shipped
        arena the banner sits at (2, 2) and the 10.2 m wall block is centred
        at (7, 0), five metres further in. That offset IS the corridor.

        The first version set all three to the gate position, collapsing them
        onto one point. Because the walls are centred, they then extended
        5.1 m BEHIND the banner: for seed 1003 the takeoff point ended up at
        corridor-local (-3.77, 1.72) -- inside the corridor channel -- and the
        aircraft never got off the ground (0.7 m, then disarm).

        An arena the aircraft cannot fly is not evidence about the aircraft.
        Moving the corridor as one rigid body randomises its placement while
        keeping it a corridor.
    """
    px, py = pivot
    marker = f'<model name="{model}">'
    i = world.index(marker)
    j = world.index("</model>", i)
    block = world[i:j]
    a = block.rindex("<pose>")
    b = block.index("</pose>", a)
    parts = block[a + 6:b].split()
    while len(parts) < 6:
        parts.append("0")
    ox, oy, oyaw = float(parts[0]), float(parts[1]), float(parts[5])
    rx, ry = ox - px, oy - py                     # offset from the pivot
    c, sn = math.cos(dyaw), math.sin(dyaw)
    nx = px + dx + rx * c - ry * sn
    ny = py + dy + rx * sn + ry * c
    parts[0] = f"{nx:.3f}"
    parts[1] = f"{ny:.3f}"
    parts[5] = f"{oyaw + dyaw:.4f}"
    new_block = block[:a + 6] + " ".join(parts) + block[b:]
    return world[:i] + new_block + world[j:], (nx, ny)


def _model_xy(world, model):
    marker = f'<model name="{model}">'
    i = world.index(marker)
    j = world.index("</model>", i)
    block = world[i:j]
    a = block.rindex("<pose>")
    b = block.index("</pose>", a)
    parts = block[a + 6:b].split()
    return float(parts[0]), float(parts[1])


# Shipped-arena geometry the layout helpers derive from.
ZONE_W, ZONE_H = 40.0, 30.0                     # delivery_zone_40x30
SHIPPED_ZONE_CENTRE = (32.0, 0.0)
BANNER_PIVOT = (2.0, 2.0)                       # forward_aerothon_banner
# corridor_walls outer footprint in the shipped arena: x 1.9..12.1, y +-3.85
CORRIDOR_CORNERS = ((1.9, -3.85), (12.1, -3.85), (12.1, 3.85), (1.9, 3.85))
TAKEOFF_CORNERS = ((-3.5, -4.0), (1.5, -4.0), (1.5, 4.0), (-3.5, 4.0))
RED_ZONE_SIZES = {"restricted_red_zone_main": (10.0, 7.0),
                  "restricted_red_zone_northwest": (6.0, 4.0),
                  "restricted_red_zone_south": (7.0, 4.0)}
FENCE_MARGIN_M = 6.0


def corridor_footprint(gate_x, gate_y, gate_yaw, pivot=BANNER_PIVOT):
    """Corridor corners after the same rigid move _rigid_move() applies.

    Order: entry-south, exit-south, exit-north, entry-north.
    """
    px, py = pivot
    c, s = math.cos(gate_yaw), math.sin(gate_yaw)
    out = []
    for x, y in CORRIDOR_CORNERS:
        rx, ry = x - px, y - py
        out.append((gate_x + rx * c - ry * s, gate_y + rx * s + ry * c))
    return out


def _rect_point_gap(rect, p):
    """Distance from a point to an axis-aligned rect (0 inside)."""
    x0, x1, y0, y1 = rect
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def arena_fence_rect(corridor, zone_centre, margin=FENCE_MARGIN_M):
    """The organiser's arena geofence for this layout, as (x0, x1, y0, y1).

    The rulebook promises the geofence coordinates; the simulator plays the
    organiser and draws them round everything the mission legitimately
    flies over -- the takeoff pad, the corridor and the delivery field --
    plus a margin, exactly as a venue fence would be.
    """
    zx, zy = zone_centre
    pts = list(TAKEOFF_CORNERS) + list(corridor) + [
        (zx - ZONE_W / 2, zy - ZONE_H / 2), (zx + ZONE_W / 2, zy + ZONE_H / 2)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return [round(min(xs) - margin, 2), round(max(xs) + margin, 2),
            round(min(ys) - margin, 2), round(max(ys) + margin, 2)]


def spawn_xy(world: str):
    """World (x, y) of the vehicle include -- the FCU home."""
    i = world.index("<name>aerothon_iris</name>")
    a = world.index("<pose>", i)
    b = world.index("</pose>", a)
    x, y = (float(v) for v in world[a + 6:b].split()[:2])
    return x, y


def to_home_frame(layout, home):
    """Copy of `layout` with its organiser inputs relative to `home`.

    Only the two published rectangles are converted (they become the FCU-
    relative inputs); the rest of the layout stays in world coordinates for
    ground-truth checks, and `home_world` records the offset.
    """
    hx, hy = home
    out = dict(layout)
    zx, zy, w, h = layout["delivery_zone_rect"]
    out["delivery_zone_rect"] = [round(zx - hx, 3), round(zy - hy, 3), w, h]
    x0, x1, y0, y1 = layout["geofence_rect"]
    out["geofence_rect"] = [round(x0 - hx, 3), round(x1 - hx, 3),
                            round(y0 - hy, 3), round(y1 - hy, 3)]
    if layout.get("geofence_poly"):
        out["geofence_poly"] = [[round(x - hx, 3), round(y - hy, 3)]
                                for x, y in layout["geofence_poly"]]
    out["home_world"] = [hx, hy]
    out["frame"] = ("delivery_zone_rect/geofence_rect/geofence_poly are "
                    "home-local ENU; gate, red_zones, pads, zone are world")
    return out


def shipped_layout():
    """Layout record for the un-randomised arena."""
    foot = corridor_footprint(BANNER_PIVOT[0], BANNER_PIVOT[1], 0.0)
    return {"delivery_zone_rect": [SHIPPED_ZONE_CENTRE[0], SHIPPED_ZONE_CENTRE[1],
                                   ZONE_W, ZONE_H],
            "geofence_rect": arena_fence_rect(foot, SHIPPED_ZONE_CENTRE),
            # Ground truth for sim/check_track.py, world coordinates, as in
            # mission2.sdf.
            "gate": [BANNER_PIVOT[0], BANNER_PIVOT[1], 0.0],
            "red_zones": {"restricted_red_zone_main": [38.0, 5.0],
                          "restricted_red_zone_northwest": [29.0, 10.0],
                          "restricted_red_zone_south": [40.0, -11.0]},
            "pads": {"a": [21.0, 10.0], "b": [47.0, 10.0], "c": [23.0, 1.0],
                     "d": [33.0, -10.0], "e": [45.0, -6.0]}}


def randomise_arena(world: str, rng) -> dict:
    """Move everything the mission is supposed to FIND rather than know.

    WHY THIS IS THE REAL ACCEPTANCE TEST (Phase 11)

        Every phase up to here removed a hardcoded coordinate and replaced it
        with perception. Each removal was checked against the ONE arena the
        coordinates came from, which cannot distinguish "derived from what the
        camera sees" from "derived from a different constant that happens to
        agree". Moving the arena can.

        Three separate live failures this session were only exposed because a
        hardcoded waypoint stopped dragging the aircraft to the right place:
        a corridor exit that fired before entry, a banner alignment that never
        approached, and a search that swept the takeoff pad. Those were found
        by accident. This finds them on purpose.

    What is NOT randomised: the takeoff pad and the start QR, because the
    aircraft spawns there and the mission legitimately begins from home (the
    one global reference the rules allow).
    """
    layout = {}

    # ---- corridor mouth: the banner gate moves and turns ---- #
    gate_x = rng.uniform(2.0, 6.0)
    gate_y = rng.uniform(-4.0, 4.0)
    gate_yaw = rng.uniform(-0.35, 0.35)
    # Move the whole corridor as ONE RIGID BODY about the banner, so the
    # wall block keeps its five-metre offset instead of being dragged back
    # over the takeoff point. See _rigid_move().
    pivot = _model_xy(world, "forward_aerothon_banner")
    dx, dy = gate_x - pivot[0], gate_y - pivot[1]
    # EVERY corridor component, not just the forward lane.
    #
    # The first version moved only the banner, the wall block and the forward
    # surround. The return lane's markers, its second banner, and -- fatally --
    # `return_static_obstacles` stayed at their original poses, so a rotated
    # forward corridor was driven straight through them.
    #
    # Seed 1002: obstacle `o4c`, a 0.35 x 1.45 x 3.4 m pillar, ended up at
    # (10.60, -3.05), 0.97 m from where the aircraft jammed at (9.7, -2.7),
    # and 3.4 m tall so it spanned the 3.0 m corridor altitude. The aircraft
    # was correctly aligned (heading -11.4 deg against a -11.2 deg corridor)
    # and the gap bearing read 0.0 the whole way in -- it simply flew into a
    # pillar the harness had put in its path. Ten seconds of backoff at
    # -0.35 m/s moved it zero metres; it pitched over to 45 degrees while
    # pinned.
    #
    # That accounted for three of the five Phase 11 failures. The corridor is
    # ONE structure and has to move as one rigid body.
    for part in ("forward_aerothon_banner", "corridor_walls",
                 "forward_corridor_green", "return_corridor_orange",
                 "return_static_obstacles", "return_aerothon_banner"):
        world, _ = _rigid_move(world, part, dx, dy, gate_yaw, pivot)
    layout["gate"] = [round(gate_x, 2), round(gate_y, 2), round(gate_yaw, 3)]

    # ---- delivery zone: BEYOND the corridor, and the corridor opens into it --
    #
    # It was placed at x 24..40 regardless of where the corridor ended, and a
    # 40 m field centred at x=24 reaches back to x=4 -- over the corridor
    # itself. Rulebook Figure 3 has the corridor lead INTO the delivery area,
    # so the field starts just past the corridor exit and spans its mouth.
    foot = corridor_footprint(gate_x, gate_y, gate_yaw)
    exit_pts = foot[1:3]
    exit_x = max(p[0] for p in exit_pts)
    x0 = exit_x + rng.uniform(0.5, 4.0)
    zx = x0 + ZONE_W / 2.0
    ey0 = min(p[1] for p in exit_pts)
    ey1 = max(p[1] for p in exit_pts)
    lo = max(-8.0, ey1 + 3.0 - ZONE_H / 2.0)
    hi = min(8.0, ey0 - 3.0 + ZONE_H / 2.0)
    zy = rng.uniform(lo, hi) if lo <= hi else 0.5 * (ey0 + ey1)
    world, _ = _set_pose(world, "delivery_zone_40x30", zx, zy)
    world, _ = _set_pose(world, "delivery_geofence_boundary", zx, zy)
    layout["zone"] = [round(zx, 2), round(zy, 2)]

    # ---- red zones: inside the zone, clear of the corridor mouth ---- #
    #
    # Anywhere in the field, including over a lane -- but not across the
    # corridor exit, where the aircraft must enter the field at corridor
    # altitude and the rulebook gives it no alternative way in.
    reds = {}
    red_rects = []
    for name, (w, h) in RED_ZONE_SIZES.items():
        for _ in range(200):
            rx = zx + rng.uniform(-(ZONE_W - w) / 2.0, (ZONE_W - w) / 2.0)
            ry = zy + rng.uniform(-(ZONE_H - h) / 2.0, (ZONE_H - h) / 2.0)
            rect = (rx - w / 2, rx + w / 2, ry - h / 2, ry + h / 2)
            if all(_rect_point_gap(rect, p) >= 6.0 for p in exit_pts):
                break
        world, _ = _set_pose(world, name, rx, ry)
        reds[name] = [round(rx, 2), round(ry, 2)]
        red_rects.append(rect)
    layout["red_zones"] = reds

    # ---- target pads: in the zone, deliverable, distinct ---- #
    #
    # A pad under red ground cannot be delivered to without a violation, and
    # two pads in one camera frame test the tie-break rather than the search.
    pads = {}
    placed = []
    for letter in "abcde":
        for _ in range(500):
            px = zx + rng.uniform(-16.0, 16.0)
            py = zy + rng.uniform(-11.0, 11.0)
            if all(_rect_point_gap(r, (px, py)) >= 3.5 for r in red_rects) and \
                    all(math.dist((px, py), q) >= 7.0 for q in placed):
                break
        world, _ = _set_pose(world, f"delivery_qr_target_{letter}", px, py)
        pads[letter] = [round(px, 2), round(py, 2)]
        placed.append((px, py))
    layout["pads"] = pads
    layout["delivery_zone_rect"] = [zx, zy, ZONE_W, ZONE_H]
    layout["geofence_rect"] = arena_fence_rect(foot, (zx, zy))

    # ---- decoys: moved so "reject non-banners" is exercised afresh ---- #
    for name in ("green_decoy_field", "green_decoy_zone"):
        world, _ = _set_pose(world, name,
                             rng.uniform(6.0, 30.0), rng.uniform(-18.0, 18.0))

    layout["world"] = world
    return layout


# --------------------------------------------------------------------------- #
# User-built arenas (tools/world_editor -> scripts/world_spec.py)
# --------------------------------------------------------------------------- #

RED_BASE_MATERIAL = ("<material><ambient>0.78 0.01 0.01 1</ambient>"
                     "<diffuse>0.95 0.02 0.02 1</diffuse></material>")
GREEN_FLOOR = ("<material><ambient>0.02 0.58 0.23 1</ambient>"
               "<diffuse>0.02 0.68 0.28 1</diffuse></material>")
ORANGE_FLOOR = ("<material><ambient>0.92 0.35 0.04 1</ambient>"
                "<diffuse>1.0 0.42 0.05 1</diffuse></material>")
WALL_MATERIAL = ("<material><ambient>0.82 0.86 0.90 1</ambient>"
                 "<diffuse>0.90 0.93 0.96 1</diffuse></material>")
OBSTACLE_MATERIAL = ("<material><ambient>0.015 0.015 0.018 1</ambient>"
                     "<diffuse>0.025 0.025 0.030 1</diffuse></material>")


def _box(name, pose, size, material, collide=True):
    geo = f"<geometry><box><size>{size}</size></box></geometry>"
    out = f'<visual name="{name}"><pose>{pose}</pose>{geo}{material}</visual>'
    if collide:
        out += f'<collision name="{name}_c"><pose>{pose}</pose>{geo}</collision>'
    return out


def lane_model(name, c, floor_material, obstacles=()):
    """One corridor: floor, two walls and any obstacles, in its own frame.

    Origin at the lane's banner end, +x down the lane, centred on y = 0 --
    the frame world_spec and check_track use. Walls are 0.2 m longer than the
    lane (0.1 m past each end), as in the shipped arena.
    """
    S = world_spec
    L, W, H = float(c["length"]), float(c["width"]), float(c["wall_height"])
    wy = W / 2 + S.WALL_T / 2
    parts = [
        _box("floor", f"{L / 2:.3f} 0 0.04 0 0 0", f"{L:.3f} {W:.3f} 0.08",
             floor_material),
        _box("wall_left", f"{L / 2:.3f} {wy:.3f} {H / 2:.3f} 0 0 0",
             f"{L + 0.2:.3f} {S.WALL_T:.3f} {H:.3f}", WALL_MATERIAL),
        _box("wall_right", f"{L / 2:.3f} {-wy:.3f} {H / 2:.3f} 0 0 0",
             f"{L + 0.2:.3f} {S.WALL_T:.3f} {H:.3f}", WALL_MATERIAL),
    ]
    for i, o in enumerate(obstacles, 1):
        h = float(o["h"])
        parts.append(_box(
            f"obstacle_{i}",
            f"{float(o['u']):.3f} {float(o['v']):.3f} {h / 2:.3f} 0 0 "
            f"{math.radians(float(o.get('yaw_deg', 0.0))):.4f}",
            f"{float(o['w']):.3f} {float(o['d']):.3f} {h:.3f}", OBSTACLE_MATERIAL))
    body = "\n        ".join(parts)
    return (f'\n    <model name="{name}">\n      <static>true</static>\n'
            f'      <link name="lane">\n        {body}\n      </link>\n'
            f'      <pose>{float(c["x"]):.3f} {float(c["y"]):.3f} 0 0 0 '
            f'{math.radians(float(c["yaw_deg"])):.4f}</pose>\n    </model>')


def _model_span(world, model):
    """(start, end) of a <model name=...> block, end past its </model>."""
    i = world.index(f'<model name="{model}">')
    # Nested models are not used in this world, so the first </model> closes it.
    j = world.index("</model>", i) + len("</model>")
    return i, j


def _remove_model(world, model):
    i, j = _model_span(world, model)
    return world[:i] + world[j:]


def _append_models(world, text):
    k = world.rindex("</world>")
    return world[:k] + text + "\n  " + world[k:]


def _set_include_pose(world, name, x, y, yaw):
    i = world.index(f"<name>{name}</name>")
    a = world.index("<pose>", i)
    b = world.index("</pose>", a)
    parts = world[a + 6:b].split()
    while len(parts) < 6:
        parts.append("0")
    parts[0], parts[1], parts[5] = f"{x:.3f}", f"{y:.3f}", f"{yaw:.4f}"
    return world[:a + 6] + " ".join(parts) + world[b:]


def _replace_in_model(world, model, old, new):
    i, j = _model_span(world, model)
    block = world[i:j]
    if old not in block:
        raise SystemExit(f"{model}: expected '{old}' in the template")
    return world[:i] + block.replace(old, new) + world[j:]


def red_zone_model(name, r, prefix):
    w, h = float(r["w"]), float(r["h"])
    yaw = math.radians(float(r.get("yaw_deg", 0.0)))
    return (f'\n    <model name="{name}">\n      <static>true</static>\n'
            f'      <link name="zone">\n'
            f'        <collision name="collision"><geometry><box><size>{w:.3f} '
            f'{h:.3f} 0.10</size></box></geometry></collision>\n'
            f'        <visual name="base"><geometry><box><size>{w:.3f} {h:.3f} '
            f'0.10</size></box></geometry>{RED_BASE_MATERIAL}</visual>\n'
            f'        {red_zone_visuals(w, h, prefix)}\n'
            f'      </link>\n'
            f'      <pose>{float(r["x"]):.3f} {float(r["y"]):.3f} 0.10 0 0 {yaw:.4f}</pose>\n'
            f'    </model>')


def zone_boundary_visuals(w, h):
    white = "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>"
    lines = (("north", 0.0, h / 2, w + 0.2, 0.20), ("south", 0.0, -h / 2, w + 0.2, 0.20),
             ("west", -w / 2, 0.0, 0.20, h + 0.2), ("east", w / 2, 0.0, 0.20, h + 0.2))
    return "\n        ".join(
        f'<visual name="{n}"><pose>{x:.3f} {y:.3f} 0.07 0 0 0</pose><geometry><box>'
        f'<size>{sx:.3f} {sy:.3f} 0.06</size></box></geometry>{white}</visual>'
        for n, x, y, sx, sy in lines)


def apply_spec(world: str, spec: dict):
    """Place a validated spec's arena into the template world.

    Returns (world, layout). The layout is in the same form randomise_arena()
    writes, plus `geofence_poly` and sized, headed red zones, so the launcher,
    publish_delivery_zone.py and check_track.py read either.
    """
    S = world_spec
    # ---- take-off area: pad, start QR and spawn as one rigid body ---- #
    t = spec["takeoff"]
    tyaw = math.radians(t["yaw_deg"])
    cx0, cy0 = S.TAKEOFF_TEMPLATE_CENTRE
    for part in ("takeoff_landing_zone", "start_qr_target_a"):
        world, _ = _rigid_move(world, part, t["x"] - cx0, t["y"] - cy0, tyaw,
                               S.TAKEOFF_TEMPLATE_CENTRE)
    sx, sy = S.spawn_point(spec)
    world = _set_include_pose(world, "aerothon_iris", sx, sy, tyaw)
    # The payload hangs under the airframe centre, so it goes where the spawn goes.
    world, _ = _set_pose(world, "aerothon_payload", sx, sy, tyaw)

    # ---- corridors: each built from the spec, each with its own banner ---- #
    c = spec["corridor"]
    r = spec["return_corridor"]
    cyaw = math.radians(c["yaw_deg"])
    ryaw = math.radians(r["yaw_deg"])
    for part in ("corridor_walls", "forward_corridor_green",
                 "return_corridor_orange", "return_static_obstacles"):
        world = _remove_model(world, part)
    world = _append_models(world,
                           lane_model("corridor_outbound", c, GREEN_FLOOR)
                           + lane_model("corridor_return", r, ORANGE_FLOOR,
                                        r.get("obstacles") or []))
    world, _ = _set_pose(world, "forward_aerothon_banner", c["x"], c["y"], cyaw)
    # The return banner faces the zone, i.e. back up its own lane.
    world, _ = _set_pose(world, "return_aerothon_banner", r["x"], r["y"],
                         ryaw + math.pi)

    # ---- delivery zone and its painted boundary, any size ---- #
    z = spec["delivery_zone"]
    zw, zh = float(z["w"]), float(z["h"])
    world = _replace_in_model(world, "delivery_zone_40x30", "<size>40 30 0.08</size>",
                              f"<size>{zw:.3f} {zh:.3f} 0.08</size>")
    world, _ = _set_pose(world, "delivery_zone_40x30", z["x"], z["y"])
    i, j = _model_span(world, "delivery_geofence_boundary")
    block = world[i:j]
    a = block.index('<link name="lines">') + len('<link name="lines">')
    b = block.index("</link>")
    block = block[:a] + "\n        " + zone_boundary_visuals(zw, zh) + "\n      " + block[b:]
    world = world[:i] + block + world[j:]
    world, _ = _set_pose(world, "delivery_geofence_boundary", z["x"], z["y"])

    # ---- red zones: any number, size and heading ---- #
    for name in ("restricted_red_zone_main", "restricted_red_zone_northwest",
                 "restricted_red_zone_south"):
        world = _remove_model(world, name)
    reds, red_text = {}, ""
    for n, rz in enumerate(spec["red_zones"], 1):
        name = f"restricted_red_zone_{n}"
        red_text += red_zone_model(name, rz, f"red_{n}")
        reds[name] = [round(float(rz["x"]), 3), round(float(rz["y"]), 3),
                      round(float(rz["w"]), 3), round(float(rz["h"]), 3),
                      round(math.radians(float(rz.get("yaw_deg", 0.0))), 5)]

    # ---- green decoys: clones of the template decoy group ---- #
    i, j = _model_span(world, "green_decoy_field")
    decoy_template = world[i:j]
    world = _remove_model(world, "green_decoy_field")
    world = _remove_model(world, "green_decoy_zone")
    decoy_text = ""
    for n, d in enumerate(spec["decoys"], 1):
        block = decoy_template.replace('<model name="green_decoy_field">',
                                       f'<model name="green_decoy_{n}">')
        tmp, _ = _set_pose(block, f"green_decoy_{n}", d["x"], d["y"],
                           math.radians(d.get("yaw_deg", 0.0)))
        decoy_text += "\n    " + tmp
    world = _append_models(world, red_text + decoy_text)

    # ---- delivery pads ---- #
    pads = {}
    for l in world_spec.PAD_LETTERS:
        p = spec["pads"][l]
        world, _ = _set_pose(world, f"delivery_qr_target_{l}", p["x"], p["y"],
                             math.radians(p.get("yaw_deg", 0.0)))
        pads[l] = [round(float(p["x"]), 3), round(float(p["y"]), 3)]

    # ---- ground under everything inside the fence, plus a margin ---- #
    fence = S.fence_polygon(spec)
    fx = [p[0] for p in fence]
    fy = [p[1] for p in fence]
    gx0, gx1, gy0, gy1 = min(fx) - 10, max(fx) + 10, min(fy) - 10, max(fy) + 10
    size = f"<size>{gx1 - gx0:.3f} {gy1 - gy0:.3f} 0.10</size>"
    pose = f"<pose>{(gx0 + gx1) / 2:.3f} {(gy0 + gy1) / 2:.3f} -0.05 0 0 0</pose>"
    world = _replace_in_model(world, "site_ground", "<size>72 44 0.10</size>", size)
    world = _replace_in_model(world, "site_ground", "<pose>22 0 -0.05 0 0 0</pose>", pose)

    layout = {
        "delivery_zone_rect": [float(z["x"]), float(z["y"]), zw, zh],
        "geofence_poly": [[round(x, 3), round(y, 3)] for x, y in fence],
        "geofence_rect": [round(min(fx), 3), round(max(fx), 3),
                          round(min(fy), 3), round(max(fy), 3)],
        "gate": [round(float(c["x"]), 3), round(float(c["y"]), 3), round(cyaw, 5)],
        # Both lanes as [x, y, yaw, length, width, wall height], world frame,
        # in the lane frames world_spec defines; check_track grades each pass
        # in its own lane.
        "corridors": {
            name: [round(float(d["x"]), 3), round(float(d["y"]), 3),
                   round(math.radians(float(d["yaw_deg"])), 5),
                   float(d["length"]), float(d["width"]), float(d["wall_height"])]
            for name, d in (("outbound", c), ("return", r))},
        "obstacles": [{"poly": [[round(x, 3), round(y, 3)] for x, y in poly],
                       "h": float(o["h"])}
                      for poly, o in zip(S.obstacle_polygons(spec),
                                         r.get("obstacles") or [])],
        "red_zones": reds,
        "pads": pads,
        "zone": [float(z["x"]), float(z["y"])],
        "spec": spec.get("name", ""),
    }
    return world, layout


# The entry point lives at the END of the file. It used to sit immediately
# after main(), above randomise_arena() and _set_pose() -- so `main()` ran
# before those names were bound and --randomise-arena died with
# "NameError: name 'randomise_arena' is not defined". Nothing caught it
# because the default path never calls them.
if __name__ == "__main__":
    main()
