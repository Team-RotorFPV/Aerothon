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
import tempfile


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


def banner_visuals() -> str:
    text = "AEROTHON"
    rows = bitmap_runs(text)
    cols = len(rows[0])
    cell_y, cell_z = 3.25 / cols, 0.115
    center_z = 3.38
    visuals = [
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
    if args.randomise_arena:
        arena = randomise_arena(world, random.Random(args.seed or None))
        world = arena.pop("world")
        print("RANDOMISED ARENA: " + json.dumps(arena, sort_keys=True))
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
        "@AEROTHON_BANNER_VISUALS@": banner_visuals(),
        "@GREEN_DECOY_VISUALS@": green_decoy_visuals(),
        "@RED_ZONE_MAIN_VISUALS@": red_zone_visuals(10.0, 7.0, "red_main"),
        "@RED_ZONE_NW_VISUALS@": red_zone_visuals(6.0, 4.0, "red_nw"),
        "@RED_ZONE_SOUTH_VISUALS@": red_zone_visuals(7.0, 4.0, "red_south"),
    }
    for token, geometry in replacements.items():
        if token not in world:
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

    # ---- delivery zone: somewhere beyond the corridor ---- #
    zx = rng.uniform(24.0, 40.0)
    zy = rng.uniform(-8.0, 8.0)
    world, _ = _set_pose(world, "delivery_zone_40x30", zx, zy)
    world, _ = _set_pose(world, "delivery_geofence_boundary", zx, zy)
    layout["zone"] = [round(zx, 2), round(zy, 2)]

    # ---- target pads: scattered inside the zone ---- #
    pads = {}
    for letter in "abcde":
        px = zx + rng.uniform(-16.0, 16.0)
        py = zy + rng.uniform(-11.0, 11.0)
        world, _ = _set_pose(world, f"delivery_qr_target_{letter}", px, py)
        pads[letter] = [round(px, 2), round(py, 2)]
    layout["pads"] = pads

    # ---- red zones: anywhere in the zone, including over a lane ---- #
    reds = {}
    for name in ("restricted_red_zone_main", "restricted_red_zone_northwest",
                 "restricted_red_zone_south"):
        rx = zx + rng.uniform(-15.0, 15.0)
        ry = zy + rng.uniform(-10.0, 10.0)
        world, _ = _set_pose(world, name, rx, ry)
        reds[name] = [round(rx, 2), round(ry, 2)]
    layout["red_zones"] = reds

    # ---- decoys: moved so "reject non-banners" is exercised afresh ---- #
    for name in ("green_decoy_field", "green_decoy_zone"):
        world, _ = _set_pose(world, name,
                             rng.uniform(6.0, 30.0), rng.uniform(-18.0, 18.0))

    layout["world"] = world
    return layout


# The entry point lives at the END of the file. It used to sit immediately
# after main(), above randomise_arena() and _set_pose() -- so `main()` ran
# before those names were bound and --randomise-arena died with
# "NameError: name 'randomise_arena' is not defined". Nothing caught it
# because the default path never calls them.
if __name__ == "__main__":
    main()
