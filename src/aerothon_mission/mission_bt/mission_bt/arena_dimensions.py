"""The arena's dimensions, entered at the GCS instead of fixed at launch.

The organisers give the arena in feet and inches and may change it between
practice and the competition: a 5 m or a 10 m corridor, a 1 m QR, a 5 ft red
zone. Every dimension the algorithms depend on is one field here, in metres,
validated once, shared by the GCS aggregator (which refuses bad input with a
reason) and by mission_bt (which rebuilds its tree with the values while the
aircraft is on the ground).

`mission_params(dims)` is the whole contract between the two: the mission
parameters each dimension sets. A field that sets none is still recorded,
because the operator needs one place that says what the arena is; the GCS
marks those "recorded only" so nobody believes they steer anything.
"""

import copy
import math

FT, INCH = 0.3048, 0.0254

# (group, key, label, default m, min, max, used_by) -- default is today's
# simulated arena, so nothing changes until an operator supplies values.
FIELDS = (
    ("corridor", "length_m", "Corridor length", 10.0, 2.0, 30.0,
     "clear distance needed to cross the gate"),
    ("corridor", "width_m", "Corridor width", 3.5, 1.0, 10.0, None),
    ("corridor", "wall_height_m", "Corridor wall height", 3.6, 1.0, 8.0,
     "upper bound on the navigation height"),
    ("corridor", "nav_height_m", "Navigation height", 3.0, 0.8, 7.0,
     "corridor flying altitude"),
    ("corridor", "spacing_m", "Corridor spacing, centre to centre", 4.0, 1.0, 30.0,
     "where the return lane is found"),
    ("hall", "length_m", "Hall length", 80.0, 10.0, 500.0, None),
    ("hall", "width_m", "Hall width", 45.0, 10.0, 500.0, None),
    ("hall", "ceiling_m", "Hall ceiling / max altitude", 30.0, 3.0, 120.0,
     "caps take-off, search and drop altitudes"),
    ("banner", "width_m", "Banner width", 3.7, 0.3, 10.0, "range to the banner"),
    ("banner", "height_m", "Banner height", 1.15, 0.2, 5.0, "range to the banner"),
    ("banner", "border_m", "Banner border", 3 * INCH, 0.0, 0.5, None),
    ("banner", "suspended_bottom_m", "Hung banner lower edge", 2.805, 0.5, 8.0,
     "the height the square-up scans the hung banner at"),
    ("qr", "target_m", "Target QR side", 2.2, 0.2, 5.0,
     "search altitude and landing"),
    ("qr", "start_m", "Start QR side", 2.2, 0.2, 5.0,
     "precision landing on the start QR"),
    ("qr", "border_m", "QR white border", 3 * INCH, 0.0, 0.5, None),
    ("red_zone", "side_m", "Red zone side", 5 * FT, 0.3, 20.0, None),
    ("red_zone", "border_m", "Red zone border", 0.5 * FT, 0.0, 2.0, None),
    ("payload", "length_m", "Payload length", 0.10, 0.02, 0.5,
     "drop confirmation blob size"),
    ("payload", "width_m", "Payload width", 0.055, 0.02, 0.5,
     "drop confirmation blob size"),
    ("payload", "mass_kg", "Payload mass (kg)", 0.10, 0.01, 2.0, None),
    # "Obstacles extend across approximately half the corridor width from a
    # wall", in the exit corridor only. The lidar avoids what is there; this
    # records the organisers' figure and builds the simulated slalom.
    ("obstacle", "reach_frac", "Obstacle reach, fraction of corridor width",
     0.5, 0.1, 0.9, None),
)
PLACEMENTS = ("suspended", "ground", "both")
RETURN_SIDES = ("right", "left")


def defaults():
    out = {}
    for group, key, _label, default, *_ in FIELDS:
        out.setdefault(group, {})[key] = default
    out["banner"]["placement"] = "suspended"
    out["corridor"]["return_side"] = "right"
    return out


def validate(supplied):
    """A complete, checked copy of `supplied` over the defaults, or ValueError.

    Missing fields take the defaults, so a preset written before a field
    existed still loads. Unknown groups or fields are REFUSED: a value the
    aircraft silently ignores is worse than one it rejects, because the
    operator believes it was taken.
    """
    if not isinstance(supplied, dict):
        raise ValueError("dimensions must be an object of groups")
    known = {}
    for group, key, *_ in FIELDS:
        known.setdefault(group, set()).add(key)
    known["banner"].add("placement")
    known["corridor"].add("return_side")
    out = defaults()
    for group, values in supplied.items():
        if group not in known:
            raise ValueError(f"unknown dimension group {group!r}")
        if not isinstance(values, dict):
            raise ValueError(f"{group} must be an object of fields")
        for key, value in values.items():
            if key not in known[group]:
                raise ValueError(f"unknown dimension {group}.{key}")
            out[group][key] = value
    for group, key, label, _default, lo, hi, _ in FIELDS:
        value = out[group][key]
        if isinstance(value, bool):
            raise ValueError(f"{label} must be a number")
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{label} must be a number") from None
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{label} {value:g} is outside {lo:g}-{hi:g}")
        out[group][key] = value
    if out["banner"]["placement"] not in PLACEMENTS:
        raise ValueError(f"banner placement must be one of {', '.join(PLACEMENTS)}")
    if out["corridor"]["return_side"] not in RETURN_SIDES:
        raise ValueError(f"return side must be one of {', '.join(RETURN_SIDES)}")
    c, h = out["corridor"], out["hall"]
    # Relationships a typo breaks and the aircraft would discover in the air.
    if c["nav_height_m"] >= c["wall_height_m"]:
        raise ValueError("navigation height must be below the corridor wall height")
    if c["nav_height_m"] >= h["ceiling_m"]:
        raise ValueError("navigation height must be below the hall ceiling")
    if c["spacing_m"] < c["width_m"]:
        raise ValueError("corridor spacing must be at least the corridor width")
    if out["banner"]["border_m"] * 2 >= min(out["banner"]["width_m"],
                                            out["banner"]["height_m"]):
        raise ValueError("banner border leaves no coloured panel")
    return out


def mission_params(dims, base):
    """The mission parameters these dimensions set, given the launch `base`.

    Only parameters with a measured meaning are touched. Altitudes keep the
    launch values unless the hall ceiling is lower, which caps them one metre
    under it.
    """
    d = validate(dims)
    c, h = d["corridor"], d["hall"]
    cap = h["ceiling_m"] - 1.0
    side = -1.0 if c["return_side"] == "right" else 1.0
    out = {
        "corridor_alt": c["nav_height_m"],
        "corridor_wall_m": c["wall_height_m"],
        "gate_advance_m": c["length_m"],
        "return_lane_offset_m": side * c["spacing_m"],
        "banner_w_m": d["banner"]["width_m"],
        "banner_h_m": d["banner"]["height_m"],
        "banner_bottom_m": d["banner"]["suspended_bottom_m"],
        "target_marker_m": d["qr"]["target_m"],
        "start_marker_m": d["qr"]["start_m"],
        "payload_size_m": max(d["payload"]["length_m"], d["payload"]["width_m"]),
        "banner_placement": d["banner"]["placement"],
    }
    for key in ("takeoff_alt", "search_alt", "drop_alt"):
        if key in base:
            out[key] = min(float(base[key]), cap)
    return out


def apply(base, dims):
    """`base` with the dimensions' parameters applied; base is not modified."""
    merged = copy.deepcopy(base)
    merged.update(mission_params(dims, base))
    return merged


# Presets every GCS starts with. Operator presets are saved beside these and
# may not reuse their names.
BUILTIN_PRESETS = {
    "Simulator arena": defaults(),
    # The organisers' clarifications. Corridor width and banner height are
    # not given: the width stays the simulator's 3.5 m (the banner is "equal
    # to the width of the corridor"), the height the rulebook board's 1.15 m.
    # A hung banner hangs from "a horizontal rod connecting the two corridor
    # walls", so its top is the 10 ft wall top and its lower edge 1.9 m --
    # under the 8 ft navigation height, so the gate crossing ducks.
    "SAE Aerothon 2026 (clarified)": {
        **defaults(),
        "corridor": {"length_m": 10.0, "width_m": 3.5, "wall_height_m": 10 * FT,
                     "nav_height_m": 8 * FT, "spacing_m": 4.0,
                     "return_side": "right"},
        "banner": {"width_m": 3.5, "height_m": 1.15, "border_m": 3 * INCH,
                   "suspended_bottom_m": 10 * FT - 1.15, "placement": "suspended"},
        "qr": {"target_m": 1.0, "start_m": 1.0, "border_m": 3 * INCH},
        "red_zone": {"side_m": 5 * FT, "border_m": 0.5 * FT},
        "payload": {"length_m": 0.10, "width_m": 0.055, "mass_kg": 0.10},
        "obstacle": {"reach_frac": 0.5},
    },
}
