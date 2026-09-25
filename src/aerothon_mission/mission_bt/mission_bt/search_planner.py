#!/usr/bin/env python3
"""Search geometry derived from the camera, not guessed.

WHAT THIS REPLACES (geometry audit A2, B1)

    search_alt = 10.0        "QR readable from 10 m"
    spacing    = 6.0         (and goal.md Q9 says 5.0 — they disagreed, and
                              neither came from the camera)

    Phase 1 measured the actual decode envelope (docs/QR_DECODE_ENVELOPE.md):
    reliable decoding needs about 5.3 pixels per QR module, and that floor held
    across a 3x change in resolution and a 4.4x change in marker size, so it
    can be used to size the search for any camera and any marker.

    At that floor a 0.4 m marker is readable from 1.3 m at 640x480 and 5.4 m at
    4K. `search_alt = 10.0` was never achievable for a realistic marker — the
    simulated pad is 2.2 m across, which is what made it look fine.

THE CONSEQUENCE: SWEEP HIGH, DESCEND TO DECODE

    A single altitude cannot both cover the zone and decode the payload:
    coverage wants height, decoding forbids it. So the search splits the job.

      * SWEEP altitude is set by how high the PAD can still be *detected* as a
        pad — a white quadrilateral is a far coarser target than 33 modules and
        survives at much lower pixel density.
      * DECODE altitude is set by the px/module floor and is only visited over
        a candidate.

    Lane spacing then follows from the sweep altitude and the camera's
    horizontal field of view, with an overlap factor — not from a constant.

Everything here is pure geometry with no ROS dependency, so it is testable
without a simulator and reusable by the GCS for showing the planned lanes.
"""

import math


def ground_width(altitude_m, hfov_rad):
    """Width of ground covered across the image at a given altitude."""
    return 2.0 * altitude_m * math.tan(hfov_rad / 2.0)


def px_per_module(altitude_m, image_width_px, hfov_rad, marker_m, modules):
    """Pixels per QR module for a marker seen from directly above."""
    gw = ground_width(altitude_m, hfov_rad)
    if gw <= 0:
        return float("inf")
    return (image_width_px / gw) * marker_m / modules


def max_decode_altitude(image_width_px, hfov_rad, marker_m, modules,
                        px_per_module_floor):
    """Highest altitude at which the payload can still be decoded.

    Inverts px_per_module(). This is the number Phase 6 needs and Phase 1
    measured; `search_alt = 10.0` was a guess with no basis.
    """
    return (image_width_px * marker_m) / (
        2.0 * math.tan(hfov_rad / 2.0) * modules * px_per_module_floor)


def max_detect_altitude(image_width_px, hfov_rad, marker_m, min_marker_px):
    """Highest altitude at which the PAD is still a usable blob.

    Detecting "a pale quadrilateral of about the right size" needs far fewer
    pixels than decoding 33 modules, which is what makes sweep-then-descend
    worth doing at all.
    """
    return (image_width_px * marker_m) / (
        2.0 * math.tan(hfov_rad / 2.0) * min_marker_px)


def vfov(hfov_rad, image_w_px, image_h_px):
    """Vertical field of view for a rectilinear camera of this aspect."""
    return 2.0 * math.atan(math.tan(hfov_rad / 2.0) *
                           float(image_h_px) / float(image_w_px))


def min_track_altitude(marker_m, hfov_rad, image_w_px, image_h_px,
                       margin=1.15):
    """Lowest altitude at which the WHOLE marker still fits in frame.

    The floor of the tracking envelope, and the counterpart to
    max_decode_altitude(): too high and the modules are too small to decode,
    too low and the marker no longer fits in the picture at all.

    WHY THIS IS NEEDED (Phase 9)

        `land_commit_alt` was 1.5 m. For the 2.2 m pad the marker overflows
        the frame below about 2.5 m, so a precision descent commanded to hold
        lock down to 1.5 m was being asked for something geometrically
        impossible. Live runs 12 and 14 both showed the same signature: the
        aircraft descended to ~3 m, lost the pad, climbed to re-acquire, and
        oscillated between 3.4 m and 4.1 m until the stage timed out and
        degraded.

        Nothing was wrong with the controller. It was chasing a floor that the
        camera could not reach.

    The limiting dimension is the SHORT one (vertical on a landscape sensor):
    a marker that fits across the frame but not down it is still clipped.
    `margin` keeps a little of the pad's surround visible rather than having
    it exactly touch the frame edge.
    """
    v = vfov(hfov_rad, image_w_px, image_h_px)
    return margin * marker_m / (2.0 * math.tan(v / 2.0))


def lane_spacing(altitude_m, hfov_rad, overlap=0.30):
    """Lane spacing giving `overlap` fractional overlap between passes.

    goal.md Q9 asserts 5.0 m and the code said 6.0 m. Neither is right in
    general: the correct spacing depends on how high you are and how wide the
    camera sees.
    """
    return ground_width(altitude_m, hfov_rad) * (1.0 - overlap)


def plan_lawnmower(zone, spacing, altitude_m, axis="x"):
    """Boustrophedon waypoints covering `zone` = (x0, x1, y0, y1).

    Lanes run along x and step in y. The first and last lanes are inset by half
    a spacing so the swept strip covers the zone edges rather than centring the
    outermost lane on them — a lane centred on the boundary wastes half its
    width outside the zone and leaves a gap inside it.
    """
    x0, x1, y0, y1 = zone
    if spacing <= 0:
        raise ValueError("spacing must be positive")
    if axis not in ("x", "y"):
        raise ValueError("axis must be 'x' or 'y'")
    if axis == "y":
        transposed = plan_lawnmower((y0, y1, x0, x1), spacing,
                                    altitude_m, axis="x")
        return [(y, x, z) for x, y, z in transposed]

    span = y1 - y0
    n_lanes = max(1, int(math.ceil(span / spacing)))
    # Distribute lanes evenly, inset from both edges.
    if n_lanes == 1:
        ys = [0.5 * (y0 + y1)]
    else:
        step = span / n_lanes
        ys = [y0 + step * (i + 0.5) for i in range(n_lanes)]

    waypoints = []
    for i, y in enumerate(ys):
        xs = (x1, x0) if i % 2 else (x0, x1)
        waypoints.append((xs[0], y, altitude_m))
        waypoints.append((xs[1], y, altitude_m))
    return waypoints


def coverage_fraction(zone, spacing, altitude_m, hfov_rad, samples=200,
                      axis="x"):
    """Fraction of the zone within half a swath of some lane.

    A coverage PROOF rather than an assertion that the lanes look about right:
    the sweep is only meaningful if every point of the delivery zone passes
    through the camera's footprint at least once.
    """
    x0, x1, y0, y1 = zone
    half_swath = ground_width(altitude_m, hfov_rad) / 2.0
    if axis == "y":
        return coverage_fraction((zone[2], zone[3], zone[0], zone[1]),
                                 spacing, altitude_m, hfov_rad, samples,
                                 axis="x")
    lanes = [wp[1] for wp in plan_lawnmower(
        zone, spacing, altitude_m, axis=axis)[::2]]
    if not lanes:
        return 0.0
    covered = 0
    for i in range(samples):
        y = y0 + (y1 - y0) * (i + 0.5) / samples
        if any(abs(y - ly) <= half_swath for ly in lanes):
            covered += 1
    return covered / samples


def plan_search(zone, image_width_px, hfov_rad, marker_m, modules,
                px_per_module_floor=5.3, min_marker_px=25.0,
                overlap=0.30, max_altitude=None, axis="x",
                swath_fov_rad=None):
    """Full search plan: sweep altitude, decode altitude, lanes, coverage.

    Returns a dict. `decode_alt` is where the aircraft must descend to over a
    candidate; `sweep_alt` is where it looks for candidates.

    `swath_fov_rad` is the field of view ACROSS the lanes, when that is not
    the horizontal one: a sweep flown crabbed, with the image's long axis
    along the lane for look-ahead, sweeps with the vertical FOV. Decode and
    detect altitudes still come from the image width and `hfov_rad`.
    """
    swath_fov = hfov_rad if swath_fov_rad is None else float(swath_fov_rad)
    decode_alt = max_decode_altitude(image_width_px, hfov_rad, marker_m,
                                     modules, px_per_module_floor)
    detect_alt = max_detect_altitude(image_width_px, hfov_rad, marker_m,
                                     min_marker_px)
    # Sweep as high as the PAD is still a usable blob, but never lower than the
    # altitude the payload is already decodable from -- descending below that
    # buys nothing and costs lanes.
    sweep_alt = max(detect_alt, decode_alt)
    # ...and then the ceiling, which WINS over both.
    #
    # This clamp used to come first and be undone by the max() above. With the
    # 2.2 m pad, decode_alt is 13.9 m, so a 10 m ceiling was silently raised to
    # 13.9 m and the aircraft swept above the rulebook's 10 m identification
    # altitude -- observed in live run 12 (MAVProxy reported "height 15").
    #
    # A regulatory ceiling is not a preference to be traded against image
    # resolution. If it forces the sweep below decode_alt, that is not a
    # problem: lower is strictly easier to decode, and descend_to_decode
    # correctly reports False because no further descent is needed.
    if max_altitude is not None:
        sweep_alt = min(sweep_alt, max_altitude)
    spacing = lane_spacing(sweep_alt, swath_fov, overlap)
    if axis == "auto":
        axis = "x" if zone[1] - zone[0] >= zone[3] - zone[2] else "y"
    waypoints = plan_lawnmower(zone, spacing, sweep_alt, axis=axis)
    return {
        "sweep_alt_m": sweep_alt,
        "decode_alt_m": decode_alt,
        "detect_alt_m": detect_alt,
        "lane_spacing_m": spacing,
        "swath_m": ground_width(sweep_alt, swath_fov),
        "swath_fov_rad": swath_fov,
        "waypoints": waypoints,
        "n_lanes": len(waypoints) // 2,
        "coverage": coverage_fraction(zone, spacing, sweep_alt, swath_fov,
                                      axis=axis),
        "lane_axis": axis,
        "descend_to_decode": sweep_alt > decode_alt + 1e-9,
    }


# --------------------------------------------------------------------------- #
# Observed zone extent (geometry audit A7, A8)
# --------------------------------------------------------------------------- #

def grow_zone(zone, direction_rad, step_m):
    """(grown_zone, new_band) after pushing `zone` out by `step_m`.

    WHY LATERAL GROWTH EXISTS

        frontier_strip() only ever advanced along the corridor heading. Seed
        1002 swept four strips out to x = 74.7 and never found pad E, because
        the pad was not further along the corridor -- it was off to the side.
        A search that can only go forward cannot cover a zone that is wider
        than the opening it was measured from.

    `new_band` is the ground the grown zone adds, so each expansion sweeps one
    band rather than re-flying everything covered so far. Taken as an
    axis-aligned bound: for a diagonal direction that over-covers slightly at
    the corners, which is the safe way to be wrong.
    """
    x0, x1, y0, y1 = zone
    dx = math.cos(direction_rad) * step_m
    dy = math.sin(direction_rad) * step_m
    nx0, nx1 = min(x0, x0 + dx), max(x1, x1 + dx)
    ny0, ny1 = min(y0, y0 + dy), max(y1, y1 + dy)
    grown = (nx0, nx1, ny0, ny1)

    if abs(dx) >= abs(dy):
        band = (x1, nx1, ny0, ny1) if dx > 0 else (nx0, x0, ny0, ny1)
    else:
        band = (nx0, nx1, y1, ny1) if dy > 0 else (nx0, nx1, ny0, y0)
    return grown, band


def zone_is_plausible(zone, min_side_m=5.0, max_side_m=120.0):
    """Reject a nonsense observation rather than sweeping it.

    A lidar that sees nothing returns max range in every direction, which would
    otherwise produce an enormous 'zone' and a sweep that never ends.
    """
    x0, x1, y0, y1 = zone
    w, h = x1 - x0, y1 - y0
    if w < min_side_m or h < min_side_m:
        return False, f"observed zone too small ({w:.1f} x {h:.1f} m)"
    if w > max_side_m or h > max_side_m:
        return False, f"observed zone implausibly large ({w:.1f} x {h:.1f} m)"
    return True, ""


# --------------------------------------------------------------------------- #
# Exclusion zones (Phase 7)
# --------------------------------------------------------------------------- #

def clip_lane(x0, x1, y, clearance_m, exclusions):
    """A lane split into the segments the AIRCRAFT may actually fly.

    The constraint a red zone imposes is on the airframe, not on the camera:
    the aircraft must not be over the zone. Seeing it is harmless and in fact
    necessary — that is how it got mapped.

    Clipping against the camera swath instead was the first thing tried here
    and it is wrong twice over. It forbids lanes that merely LOOK at red
    ground, and because the swath is far wider than the aircraft it carves out
    reachable ground nobody was forbidden to cover (measured: 0.81 coverage
    where 1.0 was achievable). `clearance_m` is the airframe half-span plus
    whatever margin the projection error deserves.

    Flying around is not the authoritative protection — the ArduPilot
    exclusion fence is — but a plan that knowingly routes over a red zone is
    indefensible whether or not the fence catches it.
    """
    blocked = []
    # Use the same merged, inflated obstacles as route_leg. Cutting only at
    # the painted edge produces endpoints that the router must refuse, so
    # the search silently skips the reachable part of each clipped lane.
    for ex0, ex1, ey0, ey1 in routing_obstacles(exclusions, clearance_m):
        if not ey0 <= y <= ey1:
            continue
        # Leave room beyond the inclusive forbidden boundary, just as the
        # router's corner waypoints do.
        a, b = max(x0, ex0 - 0.25), min(x1, ex1 + 0.25)
        if b > a:
            blocked.append((a, b))

    if not blocked:
        return [(x0, x1)]

    blocked.sort()
    merged = [blocked[0]]
    for a, b in blocked[1:]:
        if a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))

    segments = []
    cursor = x0
    for a, b in merged:
        if a > cursor:
            segments.append((cursor, a))
        cursor = max(cursor, b)
    if cursor < x1:
        segments.append((cursor, x1))
    return segments


def plan_lawnmower_excluding(zone, spacing, altitude_m, hfov_rad,
                             exclusions=(), min_segment_m=1.0,
                             clearance_m=1.0, axis="x"):
    """Boustrophedon waypoints that never route the AIRCRAFT over an exclusion.

    Same lane geometry as plan_lawnmower(); each lane is then cut where the
    airframe (plus `clearance_m`) would enter red ground, and stubs shorter
    than `min_segment_m` are dropped because flying a 30 cm segment costs more
    in settling time than the coverage is worth.
    """
    if axis == "y":
        swapped_zone = (zone[2], zone[3], zone[0], zone[1])
        swapped_exclusions = [(ey0, ey1, ex0, ex1)
                              for ex0, ex1, ey0, ey1 in exclusions]
        transposed = plan_lawnmower_excluding(
            swapped_zone, spacing, altitude_m, hfov_rad,
            exclusions=swapped_exclusions, min_segment_m=min_segment_m,
            clearance_m=clearance_m, axis="x")
        return [(y, x, z) for x, y, z in transposed]
    if axis != "x":
        raise ValueError("axis must be 'x' or 'y'")

    x0, x1, y0, y1 = zone
    lanes = plan_lawnmower(zone, spacing, altitude_m, axis="x")

    waypoints = []
    for i in range(0, len(lanes), 2):
        y = lanes[i][1]
        reverse = lanes[i][0] > lanes[i + 1][0]
        segments = clip_lane(x0, x1, y, clearance_m, exclusions)
        segments = [(a, b) for a, b in segments if b - a >= min_segment_m]
        if reverse:
            segments = [(b, a) for a, b in reversed(segments)]
        for a, b in segments:
            waypoints.append((a, y, altitude_m))
            waypoints.append((b, y, altitude_m))
    return waypoints


def coverage_fraction_excluding(zone, spacing, altitude_m, hfov_rad,
                                exclusions=(), samples=120, clearance_m=1.0,
                                axis="x"):
    """Fraction of the REACHABLE zone the clipped plan still covers.

    The denominator excludes red ground: a plan is not at fault for failing to
    cover a place it is forbidden to fly over. Without this the coverage proof
    would fall below 1.0 the moment a red zone existed, and the only way to
    keep it green would be to fly over the red — exactly backwards.
    """
    if axis == "y":
        swapped_zone = (zone[2], zone[3], zone[0], zone[1])
        swapped_exclusions = [(ey0, ey1, ex0, ex1)
                              for ex0, ex1, ey0, ey1 in exclusions]
        return coverage_fraction_excluding(
            swapped_zone, spacing, altitude_m, hfov_rad,
            exclusions=swapped_exclusions, samples=samples,
            clearance_m=clearance_m, axis="x")
    x0, x1, y0, y1 = zone
    half_swath = ground_width(altitude_m, hfov_rad) / 2.0
    wps = plan_lawnmower_excluding(zone, spacing, altitude_m, hfov_rad,
                                   exclusions, clearance_m=clearance_m,
                                   axis="x")
    segs = [(wps[i], wps[i + 1]) for i in range(0, len(wps), 2)]

    reachable = 0
    covered = 0
    for i in range(samples):
        x = x0 + (x1 - x0) * (i + 0.5) / samples
        for j in range(samples):
            y = y0 + (y1 - y0) * (j + 0.5) / samples
            if any(ex0 <= x <= ex1 and ey0 <= y <= ey1
                   for ex0, ex1, ey0, ey1 in exclusions):
                continue                  # forbidden, not owed coverage
            reachable += 1
            for (ax, ay, _), (bx, _, _) in segs:
                if abs(y - ay) <= half_swath and min(ax, bx) <= x <= max(ax, bx):
                    covered += 1
                    break
    return covered / reachable if reachable else 1.0


def plan_intersects_exclusions(waypoints, clearance_m, exclusions):
    """True if any planned leg would put the airframe inside an exclusion.

    The assertion the Phase 7 acceptance test makes. Checks the swept corridor
    of the aircraft (the flight line widened by `clearance_m`), which is the
    thing the rulebook forbids — not the camera swath, which is allowed and
    expected to see red ground.
    """
    for i in range(0, len(waypoints) - 1, 2):
        a, b = waypoints[i], waypoints[i + 1]
        if path_hits_exclusion([a[:2], b[:2]], clearance_m, exclusions):
            return True
    return False


# --------------------------------------------------------------------------- #
# Exclusion-aware transit legs
# --------------------------------------------------------------------------- #
#
# WHY THIS EXISTS
#
#     Everything above clips LANES. The sweep was the only part of the mission
#     that ever asked about red ground; every straight leg -- corridor exit to
#     the observed zone, approach to the matched pad, the reposition before
#     descent-to-decode, the return to the corridor mouth, go-home -- flew
#     point to point with no exclusion check at all.
#
#     A watched flight had 198 confirmed exclusion cells and still crossed red
#     ground. The detector was never the problem: nothing downstream of the
#     sweep asked it anything. This is that missing question, asked once, in
#     one place, by every leg.
#
# WHAT IT IS NOT
#
#     Not a general path planner. A visibility graph over the corners of the
#     merged obstacle boxes, which for the shape of a real arena -- a handful
#     of blobs of red ground on open field -- is both optimal and cheap. It
#     refuses rather than improvising when the answer would be a long way
#     round: a 27x detour is a time-limit failure, and the time limit is also
#     worth marks.

def _segment_hits_rect(ax, ay, bx, by, rect, eps=1e-9):
    """True if segment a->b enters the interior of an axis-aligned rect.

    Slab method. Grazing the boundary does not count -- the corner nodes the
    router steers through sit exactly on an inflated boundary, and a router
    that considered its own waypoints to be violations could never route.
    """
    x0, x1, y0, y1 = rect
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for d, lo, hi, p in ((dx, x0, x1, ax), (dy, y0, y1, ay)):
        if abs(d) < eps:
            if p <= lo + eps or p >= hi - eps:
                return False              # parallel and outside this slab
            continue
        n0, n1 = (lo - p) / d, (hi - p) / d
        if n0 > n1:
            n0, n1 = n1, n0
        t0, t1 = max(t0, n0), min(t1, n1)
        if t0 >= t1 - eps:
            return False
    return t1 > t0 + eps


def _inflate(rect, m):
    x0, x1, y0, y1 = rect
    return (x0 - m, x1 + m, y0 - m, y1 + m)


def _contains(rect, x, y, eps=0.0):
    x0, x1, y0, y1 = rect
    return (x0 - eps) <= x <= (x1 + eps) and (y0 - eps) <= y <= (y1 + eps)


def leg_hits_exclusion(a, b, clearance_m, exclusions):
    """True if flying a->b would put the AIRFRAME inside any exclusion.

    Unlike plan_intersects_exclusions(), which assumes the axis-aligned lanes
    it was written for, this handles a leg at any heading -- including one that
    only nicks a corner diagonally, which is what an axis-aligned check misses.
    """
    for ex in exclusions:
        if _segment_hits_rect(a[0], a[1], b[0], b[1],
                              _inflate(ex, clearance_m)):
            return True
    return False


def point_in_exclusion(x, y, clearance_m, exclusions):
    """True if HOLDING STATION here would sit the airframe on red ground.

    Distinct from leg_hits_exclusion() on a zero-length leg, and the
    distinction matters. A leg that begins inside a zone is allowed -- leaving
    is the correct response and refusing to move would hold the aircraft over
    the violation. Stopping there is never allowed. Descending onto a
    candidate is a commitment to sit above it for the whole descent.
    """
    return any(_contains(_inflate(ex, clearance_m), float(x), float(y))
               for ex in exclusions)


def path_hits_exclusion(points, clearance_m, exclusions):
    """True if any consecutive leg of a polyline enters an exclusion."""
    return any(leg_hits_exclusion(points[i], points[i + 1],
                                  clearance_m, exclusions)
               for i in range(len(points) - 1))


def _box_area(b):
    return max(0.0, b[1] - b[0]) * max(0.0, b[3] - b[2])


def _box_overlap(a, b):
    return (max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
            * max(0.0, min(a[3], b[3]) - max(a[2], b[2])))


def _exact_runs(boxes):
    """Lossless merges: boxes sharing an extent exactly and touching along it.

    A painted rectangle arrives as a grid of cells; row by row they share a y
    extent, and the rows then share an x extent, so it comes back as ONE box
    with not a square metre added.
    """
    changed = True
    while changed:
        changed = False
        for lo, hi, klo, khi in ((0, 1, 2, 3), (2, 3, 0, 1)):
            rows = {}
            for b in boxes:
                rows.setdefault((b[klo], b[khi]), []).append(b)
            out = []
            for group in rows.values():
                group.sort(key=lambda b: b[lo])
                cur = list(group[0])
                for b in group[1:]:
                    if b[lo] <= cur[hi]:
                        cur[hi] = max(cur[hi], b[hi])
                    else:
                        out.append(tuple(cur))
                        cur = list(b)
                out.append(tuple(cur))
            if len(out) < len(boxes):
                changed = True
            boxes = out
    return boxes


def merge_exclusions(exclusions, gap_m=0.0, max_waste=0.25):
    """Confirmed cells collapsed into the boxes the router steers around.

    The georeferencer confirms red ground cell by cell, so a single painted
    zone arrives as a hundred-odd little rectangles, and routing round each
    one is slow. They are merged, but NOT blindly into bounding boxes.

    WHAT THE BOUNDING-BOX MERGE COST

        Everything within `gap_m` of anything else used to become one
        bounding box. A custom arena chained three red zones along its
        delivery zone's northern edge -- each within 3 m of the next -- and
        the merge made one 25 m box of them that swallowed the strip north of
        the zones, target pad included. The sweep could never fly there, and
        the mission failed "without matching the target" with the pad in
        plain view of a lane it had been forbidden to fly.

    So: exact merges first (`_exact_runs`, no ground added), then a pair of
    boxes merges into its bounding box only if that adds at most `max_waste`
    of the result as ground that is not red. A rectangle is still one box;
    an L, or a chain of separate zones, stays several.

    NOTHING GETS THROUGH A GAP IT COULD NOT BEFORE. Boxes closer than `gap_m`
    stay separate but overlap once the router inflates them by the clearance
    (gap_m is twice it), and a leg through the overlap hits one of them.
    The merge was only ever what kept routing fast.
    """
    boxes = [tuple(float(v) for v in ex) for ex in exclusions if len(ex) == 4]
    if len(boxes) < 2:
        return boxes
    boxes = _exact_runs(boxes)
    half = gap_m / 2.0
    # (box, estimated red area inside it). The estimate subtracts the whole
    # overlap of two boxes, which can only UNDER-count red, i.e. over-count
    # waste: it errs toward keeping boxes apart, never toward forbidding more.
    items = [(b, _box_area(b)) for b in boxes]
    while len(items) > 1:
        best = None
        for i in range(len(items)):
            a, ra = items[i]
            ai = _inflate(a, half)
            for j in range(i + 1, len(items)):
                b, rb = items[j]
                bi = _inflate(b, half)
                if not (min(ai[1], bi[1]) >= max(ai[0], bi[0])
                        and min(ai[3], bi[3]) >= max(ai[2], bi[2])):
                    continue
                box = (min(a[0], b[0]), max(a[1], b[1]),
                       min(a[2], b[2]), max(a[3], b[3]))
                red = max(ra, rb, ra + rb - _box_overlap(a, b))
                area = _box_area(box)
                waste = 1.0 - red / area if area > 0 else 0.0
                if waste <= max_waste and (best is None or waste < best[0]):
                    best = (waste, i, j, box, min(red, area))
        if best is None:
            break
        _, i, j, box, red = best
        items = [it for k, it in enumerate(items) if k not in (i, j)]
        items.append((box, red))
    return [b for b, _ in items]


_OBSTACLE_CACHE = {}


def routing_obstacles(exclusions, clearance_m):
    """One obstacle interpretation for sweep endpoints and transit routes.

    Cached on its inputs: every lane clip, route and red-ground check in a
    tick asks with the same confirmed cells, and they only change when the
    georeferencer confirms more.
    """
    key = (tuple(tuple(float(v) for v in ex) for ex in exclusions),
           float(clearance_m))
    hit = _OBSTACLE_CACHE.get(key)
    if hit is None:
        hit = [_inflate(b, clearance_m)
               for b in merge_exclusions(exclusions, 2.0 * clearance_m)]
        if len(_OBSTACLE_CACHE) > 32:
            _OBSTACLE_CACHE.clear()
        _OBSTACLE_CACHE[key] = hit
    return list(hit)


def _dijkstra(nodes, edges, src, dst):
    import heapq
    dist = {src: 0.0}
    prev = {}
    seen = set()
    heap = [(0.0, src)]
    while heap:
        d, u = heapq.heappop(heap)
        if u in seen:
            continue
        seen.add(u)
        if u == dst:
            break
        for v, w in edges.get(u, ()):
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if dst not in dist:
        return None, float("inf")
    path, cur = [dst], dst
    while cur != src:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return [nodes[i] for i in path], dist[dst]


def route_leg(start, end, clearance_m, exclusions, max_detour_ratio=4.0,
              min_detour_allowance_m=25.0, corner_margin_m=0.25,
              max_obstacles=16, _depth=0):
    """Waypoints from `start` to `end` that keep the airframe off red ground.

    Returns a dict, in the style of plan_search():

        ok         False means DO NOT FLY THIS LEG. There is no route.
        waypoints  the legs to fly, excluding `start`, ending at `end`;
                   None when ok is False
        detoured   True if the straight line would have been a violation
        length_m   the routed distance
        reason     why it was refused, empty when ok

    Fails closed on purpose. The straight line is never the fallback: a leg
    that cannot be flown legally is an abort with a reason, not a violation
    with an excuse.
    """
    sx, sy = float(start[0]), float(start[1])
    ex_, ey_ = float(end[0]), float(end[1])
    direct = math.hypot(ex_ - sx, ey_ - sy)
    straight = {"ok": True, "waypoints": [(ex_, ey_)], "detoured": False,
                "length_m": direct, "reason": ""}

    if not exclusions:
        return straight

    # Cells closer together than the airframe are one obstacle: there is no
    # flying between them.
    blocks = routing_obstacles(exclusions, clearance_m)

    # A box the aircraft is already inside cannot be routed around, only left.
    # Refusing to move would hold it over the violation it is trying to end.
    #
    # LEFT BY THE NEAREST EDGE, not by the straight line to wherever the leg
    # was going. Dropping the box and flying straight at the destination is
    # what crossed the main red zone in live run 5: the aircraft entered the
    # inflated margin of a strip confirmed mid-leg, the box was ignored, and
    # the leg ran on diagonally through the paint. Step out the shortest way,
    # then route from there with every box in play.
    containing = [b for b in blocks if _contains(b, sx, sy)]
    if containing and _depth < 3:
        x0, x1, y0, y1 = containing[0]
        m = corner_margin_m
        exit_pt = min(((x0 - m, sy), (x1 + m, sy), (sx, y0 - m), (sx, y1 + m)),
                      key=lambda p: math.hypot(p[0] - sx, p[1] - sy))
        rest = route_leg(exit_pt, end, clearance_m, exclusions,
                         max_detour_ratio=max_detour_ratio,
                         min_detour_allowance_m=min_detour_allowance_m,
                         corner_margin_m=corner_margin_m,
                         max_obstacles=max_obstacles, _depth=_depth + 1)
        if not rest["ok"]:
            return rest
        step = math.hypot(exit_pt[0] - sx, exit_pt[1] - sy)
        return {"ok": True, "waypoints": [exit_pt] + list(rest["waypoints"]),
                "detoured": True, "length_m": step + rest["length_m"],
                "reason": ""}
    blocks = [b for b in blocks if not _contains(b, sx, sy)]
    if not blocks:
        return straight

    inside = [b for b in blocks if _contains(b, ex_, ey_)]
    if inside:
        return {"ok": False, "waypoints": None, "detoured": True,
                "length_m": float("inf"),
                "reason": (f"destination ({ex_:.1f}, {ey_:.1f}) lies inside "
                           f"{len(inside)} confirmed red zone(s)")}

    if not any(_segment_hits_rect(sx, sy, ex_, ey_, b) for b in blocks):
        return straight

    # Only the nearest obstacles are routed around; the finished path is then
    # checked against ALL of them, so trimming can cost a route but can never
    # buy a violation.
    def _near(b):
        cx, cy = (b[0] + b[1]) / 2.0, (b[2] + b[3]) / 2.0
        return math.hypot(cx - sx, cy - sy) + math.hypot(cx - ex_, cy - ey_)

    routing = sorted(blocks, key=_near)[:max_obstacles]

    m = corner_margin_m
    nodes = [(sx, sy), (ex_, ey_)]
    for x0, x1, y0, y1 in routing:
        nodes.extend([(x0 - m, y0 - m), (x1 + m, y0 - m),
                      (x0 - m, y1 + m), (x1 + m, y1 + m)])
    nodes = [n for i, n in enumerate(nodes)
             if i < 2 or not any(_contains(b, n[0], n[1]) for b in routing)]

    edges = {}
    for i in range(len(nodes)):
        ax, ay = nodes[i]
        for j in range(i + 1, len(nodes)):
            bx, by = nodes[j]
            if any(_segment_hits_rect(ax, ay, bx, by, b) for b in routing):
                continue
            w = math.hypot(bx - ax, by - ay)
            edges.setdefault(i, []).append((j, w))
            edges.setdefault(j, []).append((i, w))

    path, length = _dijkstra(nodes, edges, 0, 1)
    budget = max(direct * max_detour_ratio, direct + min_detour_allowance_m)
    if path is None:
        return {"ok": False, "waypoints": None, "detoured": True,
                "length_m": float("inf"),
                "reason": (f"no route to ({ex_:.1f}, {ey_:.1f}) around "
                           f"{len(blocks)} confirmed red zone(s)")}
    if length > budget:
        return {"ok": False, "waypoints": None, "detoured": True,
                "length_m": length,
                "reason": (f"the only route to ({ex_:.1f}, {ey_:.1f}) around "
                           f"{len(blocks)} confirmed red zone(s) is "
                           f"{length:.0f} m against a {direct:.0f} m direct "
                           f"leg; refusing rather than spending the clock")}
    if path_hits_exclusion(path, clearance_m, exclusions):
        return {"ok": False, "waypoints": None, "detoured": True,
                "length_m": length,
                "reason": (f"no route to ({ex_:.1f}, {ey_:.1f}) clear of "
                           f"{len(blocks)} confirmed red zone(s)")}
    return {"ok": True, "waypoints": [(round(x, 3), round(y, 3))
                                      for x, y in path[1:]],
            "detoured": True, "length_m": length, "reason": ""}
