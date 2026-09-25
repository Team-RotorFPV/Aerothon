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


def plan_lawnmower(zone, spacing, altitude_m):
    """Boustrophedon waypoints covering `zone` = (x0, x1, y0, y1).

    Lanes run along x and step in y. The first and last lanes are inset by half
    a spacing so the swept strip covers the zone edges rather than centring the
    outermost lane on them — a lane centred on the boundary wastes half its
    width outside the zone and leaves a gap inside it.
    """
    x0, x1, y0, y1 = zone
    if spacing <= 0:
        raise ValueError("spacing must be positive")

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


def coverage_fraction(zone, spacing, altitude_m, hfov_rad, samples=200):
    """Fraction of the zone within half a swath of some lane.

    A coverage PROOF rather than an assertion that the lanes look about right:
    the sweep is only meaningful if every point of the delivery zone passes
    through the camera's footprint at least once.
    """
    x0, x1, y0, y1 = zone
    half_swath = ground_width(altitude_m, hfov_rad) / 2.0
    lanes = [wp[1] for wp in plan_lawnmower(zone, spacing, altitude_m)[::2]]
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
                overlap=0.30, max_altitude=None):
    """Full search plan: sweep altitude, decode altitude, lanes, coverage.

    Returns a dict. `decode_alt` is where the aircraft must descend to over a
    candidate; `sweep_alt` is where it looks for candidates.
    """
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
    spacing = lane_spacing(sweep_alt, hfov_rad, overlap)
    waypoints = plan_lawnmower(zone, spacing, sweep_alt)
    return {
        "sweep_alt_m": sweep_alt,
        "decode_alt_m": decode_alt,
        "detect_alt_m": detect_alt,
        "lane_spacing_m": spacing,
        "swath_m": ground_width(sweep_alt, hfov_rad),
        "waypoints": waypoints,
        "n_lanes": len(waypoints) // 2,
        "coverage": coverage_fraction(zone, spacing, sweep_alt, hfov_rad),
        "descend_to_decode": sweep_alt > decode_alt + 1e-9,
    }


# --------------------------------------------------------------------------- #
# Observed zone extent (geometry audit A7, A8)
# --------------------------------------------------------------------------- #

def zone_from_observation(entry_xy, heading_rad, open_depth_m, open_width_m,
                          margin_m=1.0):
    """Delivery-zone bounds inferred from what the aircraft can see.

    `zone = (20, 52, -12, 12)` asserted the zone's position and size in advance
    (audit A8), and `zone_entry` asserted where it began (A7). Both are only
    true of the arena they were written for.

    On leaving the corridor the aircraft knows where it is, which way it is
    facing, and — from the lidar — how far the open area extends ahead and how
    wide it is. That is enough to bound the search region without being told.

    Returned in the same (x0, x1, y0, y1) form the lane planner already takes,
    so the sweep is unchanged; only the source of the numbers differs.

    `margin_m` insets the bounds so lanes do not run into the boundary the
    lidar just measured.
    """
    ex, ey = entry_xy
    depth = max(0.0, open_depth_m - margin_m)
    half_w = max(0.0, open_width_m / 2.0 - margin_m)

    # Axis-aligned bound of the swept-out region, which is what the boustrophedon
    # planner consumes. A rotated zone is handled by taking the bounding box;
    # over-covering slightly is safe, under-covering is not.
    import math as _m
    c, s_ = _m.cos(heading_rad), _m.sin(heading_rad)
    corners = []
    for along in (0.0, depth):
        for across in (-half_w, half_w):
            corners.append((ex + along * c - across * s_,
                            ey + along * s_ + across * c))
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return (min(xs), max(xs), min(ys), max(ys))


def extend_zone(zone, heading_rad, extra_m):
    """The zone with its far edge pushed `extra_m` further along `heading_rad`.

    WHY THIS EXISTS

        zone_from_observation() bounds the zone by what the LIDAR can see, and
        the lidar has a finite range. In the reference arena it reports a depth
        of 12 m for a delivery zone that is 40 m deep, so the "observed zone"
        is a 12 m window onto it -- x 16.5..27.9 of a real 12..52.

        That window happened to contain target C, which is the target every
        live run had been started with. Targets at x 33, 45 and 47, and all
        three red zones, lie outside it. The sweep would have reported
        "swept all lanes without matching the target" and the mission would
        have failed -- correctly, but for a reason nobody had looked at.

        The window is a FRONTIER, not the zone. This pushes it forward.

    Returned as an axis-aligned bound like every other zone here: the union of
    the window and the window translated along the heading. Over-covering is
    safe, under-covering is not.
    """
    x0, x1, y0, y1 = zone
    dx = math.cos(heading_rad) * extra_m
    dy = math.sin(heading_rad) * extra_m
    return (min(x0, x0 + dx), max(x1, x1 + dx),
            min(y0, y0 + dy), max(y1, y1 + dy))


def frontier_strip(zone, heading_rad, extra_m):
    """Only the NEW ground `extend_zone` adds -- the window translated forward.

    Sweeping the extended zone from scratch would re-fly ground already
    covered. This is the strip beyond the current frontier, so each expansion
    costs one strip rather than the whole search so far.
    """
    x0, x1, y0, y1 = zone
    dx = math.cos(heading_rad) * extra_m
    dy = math.sin(heading_rad) * extra_m
    return (x0 + dx, x1 + dx, y0 + dy, y1 + dy)


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

def _overlap(a0, a1, b0, b1):
    """Length of the overlap between two intervals (0 if disjoint)."""
    return max(0.0, min(a1, b1) - max(a0, b0))


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
    for ex0, ex1, ey0, ey1 in exclusions:
        if _overlap(y - clearance_m, y + clearance_m, ey0, ey1) <= 0.0:
            continue                      # not under this lane's swath
        a, b = max(x0, ex0), min(x1, ex1)
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
                             clearance_m=1.0):
    """Boustrophedon waypoints that never route the AIRCRAFT over an exclusion.

    Same lane geometry as plan_lawnmower(); each lane is then cut where the
    airframe (plus `clearance_m`) would enter red ground, and stubs shorter
    than `min_segment_m` are dropped because flying a 30 cm segment costs more
    in settling time than the coverage is worth.
    """
    x0, x1, y0, y1 = zone
    lanes = plan_lawnmower(zone, spacing, altitude_m)

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
                                exclusions=(), samples=120, clearance_m=1.0):
    """Fraction of the REACHABLE zone the clipped plan still covers.

    The denominator excludes red ground: a plan is not at fault for failing to
    cover a place it is forbidden to fly over. Without this the coverage proof
    would fall below 1.0 the moment a red zone existed, and the only way to
    keep it green would be to fly over the red — exactly backwards.
    """
    x0, x1, y0, y1 = zone
    half_swath = ground_width(altitude_m, hfov_rad) / 2.0
    wps = plan_lawnmower_excluding(zone, spacing, altitude_m, hfov_rad,
                                   exclusions, clearance_m=clearance_m)
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
        (ax, ay, _), (bx, _, _) = waypoints[i], waypoints[i + 1]
        for ex0, ex1, ey0, ey1 in exclusions:
            if (_overlap(min(ax, bx), max(ax, bx), ex0, ex1) > 0.0
                    and _overlap(ay - clearance_m, ay + clearance_m,
                                 ey0, ey1) > 0.0):
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


def merge_exclusions(exclusions, gap_m=0.0, max_passes=6):
    """Confirmed cells collapsed into the boxes the router steers around.

    The georeferencer confirms red ground cell by cell, so a single painted
    zone arrives as a hundred-odd little rectangles. Routing around each one
    is both slow and wrong: the aircraft cannot fit through a gap narrower
    than itself, so cells closer together than `gap_m` become one obstacle.

    The merge is to a BOUNDING BOX, so an L-shaped zone forbids the notch as
    well. That over-forbids: some flyable ground is given up. Over-forbidding
    costs coverage and under-forbidding costs marks, and only one of those is
    recoverable in the air.
    """
    boxes = [tuple(float(v) for v in ex) for ex in exclusions if len(ex) == 4]
    for _ in range(max_passes):
        n = len(boxes)
        if n < 2:
            break
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        half = gap_m / 2.0
        for i in range(n):
            ax0, ax1, ay0, ay1 = _inflate(boxes[i], half)
            for j in range(i + 1, n):
                bx0, bx1, by0, by1 = _inflate(boxes[j], half)
                if (min(ax1, bx1) >= max(ax0, bx0)
                        and min(ay1, by1) >= max(ay0, by0)):
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[ri] = rj

        groups = {}
        for i, box in enumerate(boxes):
            groups.setdefault(find(i), []).append(box)
        merged = [(min(b[0] for b in g), max(b[1] for b in g),
                   min(b[2] for b in g), max(b[3] for b in g))
                  for g in groups.values()]
        if len(merged) == len(boxes):
            return merged
        boxes = merged
    return boxes


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
              max_obstacles=16):
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
    blocks = [_inflate(b, clearance_m)
              for b in merge_exclusions(exclusions, 2.0 * clearance_m)]

    # A box the aircraft is already inside cannot be routed around, only left.
    # Refusing to move would hold it over the violation it is trying to end.
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
