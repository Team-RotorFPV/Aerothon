#!/usr/bin/env python3
"""Squareness to a flat surface, from one lidar scan. Pure arithmetic.

WHY THIS EXISTS

    "Am I perpendicular to that surface" was being answered with the aspect
    ratio of a camera-derived bounding box. That box includes the gate posts,
    so it plateaus at 1.88-1.91 however square the aircraft is; a threshold of
    2.00 was unreachable and a threshold of 1.75 was satisfied by one noisy
    narrowing at 1.4. Fourteen watched runs failed on it, twice by flying the
    aircraft out of the world.

    A lidar measures perpendicularity directly. Fit a line through the returns
    in the sector the camera points at; the angle of that line to the nose IS
    the misalignment, and the same fit gives the standoff for free.

WHY IT IS A SEPARATE MODULE

    No ROS types, no clock, no vehicle. Given a scan it returns a surface or a
    refusal, which means the geometry can be tested against a wall at 30
    degrees without starting a simulator. Modelled on `route_leg` in
    search_planner: a pure function with a structured result and an explicit
    refusal, so that "I could not measure this" is a value the caller has to
    handle rather than a zero it can accidentally act on.

FRAME

    +x out of the nose, +y to PORT, angles counter-clockwise from the nose --
    the ROS LaserScan convention the avoidance node already reads. Note that
    the CAMERA disagrees: image +x is to starboard. `bearing_to_angle` is the
    one place that disagreement is written down.
"""

import math


def bearing_to_angle(bearing, hfov_rad):
    """A camera bearing, [-1, 1] across the frame, as an angle off the nose.

    A bearing is a fraction of the image half-width, not of the half-FOV: the
    two differ by the projection, and treating them as the same under-reports
    every off-centre target. The lens is already known, so use it.

    Sign: image +x is to the RIGHT, and a positive angle here is to PORT, so
    the conversion flips. Getting this backwards searches the lidar sector on
    the opposite side of the aircraft from the banner.
    """
    half = float(hfov_rad) / 2.0
    return -math.atan(float(bearing) * math.tan(half))


def no_surface(reason):
    """A refusal, in the shape a successful fit has.

    Public because the commander refuses for reasons the geometry cannot know
    about -- no scan has arrived, the last one is stale -- and a caller that
    has to tell two shapes apart will eventually get it wrong.
    """
    return {"ok": False, "angle_rad": None, "range_m": None, "points": 0,
            "residual_m": None, "extent_m": 0.0, "reason": reason}


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _fit_line(points):
    """Total-least-squares line through `points`.

    Returns (centroid, direction, normal, offset, rms). `offset` is the signed
    distance from the ORIGIN to the line along `normal`; `rms` is the root
    mean square perpendicular residual.

    Total least squares rather than y-on-x: the surface may be seen edge-on,
    where a least-squares regression against either axis blows up. A banner
    seen at 80 degrees is exactly the case the measurement has to survive.
    """
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - cx, y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    # Principal axis of the scatter: the direction the surface runs in.
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    ux, uy = math.cos(theta), math.sin(theta)
    nx, ny = -uy, ux
    offset = nx * cx + ny * cy
    rms = math.sqrt(sum((nx * x + ny * y - offset) ** 2
                        for x, y in points) / n)
    return (cx, cy), (ux, uy), (nx, ny), offset, rms


def _extent(points):
    """How WIDE the returns are, across the line of sight.

    MEASURED, and the reason this is not the spread along the fitted line.
    Parked on the shipped arena, the fit was asked what it saw in each
    direction in turn:

        sector -75:  face  -90.0 deg  7.74 m  17 pts  "span" 2.24 m
        sector +15:  face  -90.9 deg  3.11 m   8 pts  "span" 2.22 m

    Eight returns do not span two metres of anything. Those fits had gone
    RADIAL -- the line runs toward the aircraft instead of across it -- so the
    spread along it is DEPTH wearing width's clothes, and a narrow object
    sailed through a guard meant to reject exactly that. In flight the same
    fits came back as an alternating +90/-90 measurement and the aircraft
    turned in circles for fourteen steps.

    Across the mean line of sight, depth cannot stand in for width: a post is
    18 cm wide however far through the scan it is smeared.
    """
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    bearing = math.hypot(cx, cy)
    if bearing < 1e-9:
        return 0.0
    ax, ay = -cy / bearing, cx / bearing      # across the line of sight
    projections = [x * ax + y * ay for x, y in points]
    return max(projections) - min(projections)


def _clusters(points, gap_m):
    """Split angularly-ordered returns wherever the surface breaks.

    Returns from one convex face arrive as a contiguous run of samples. A gate
    post, the panel, and a wall glimpsed through the opening are three runs
    separated by jumps in range, and treating them as one cloud is what lets
    something behind the gate bend the fit.
    """
    out, current = [], []
    for p in points:
        if current and math.dist(p, current[-1]) > gap_m:
            out.append(current)
            current = []
        current.append(p)
    if current:
        out.append(current)
    return out


def fit_surface(angle_min, angle_increment, ranges, bearing_rad,
                half_width_rad, range_min=0.05, range_max=12.0,
                min_points=6, min_extent_m=0.9, max_residual_m=0.08,
                gap_m=0.35, inlier_m=0.20, max_depth_m=6.0,
                max_obliquity_rad=math.radians(70.0),
                expected_range_m=None, range_tol_m=3.0):
    """The flat surface the camera is looking at, or an explicit refusal.

    Searches only the sector `bearing_rad` +/- `half_width_rad`, because the
    sector is what stops a corridor wall behind an open gate from being
    measured instead of the gate.

        ok          False means DO NOT ACT ON THIS. There is no measurement.
        angle_rad   signed angle from the nose to the surface's perpendicular
                    foot. Positive is to PORT: yaw by +angle_rad and the nose
                    is square to the face. None when ok is False.
        range_m     perpendicular distance to the fitted surface, which is the
                    standoff and is preserved by sliding along the face.
        points      how many returns the fit used
        residual_m  RMS perpendicular residual of those returns
        extent_m    how wide the returns are ACROSS the line of sight
        reason      why it was refused; empty when ok

    Fails closed, in the manner of `route_leg`: there is no "best guess"
    return. The aspect gate this replaces had a fallback, and the fallback is
    what fired on bad data and flew the aircraft out of the world.
    """
    half = abs(float(half_width_rad))
    inc = float(angle_increment)
    lo, hi = float(range_min), float(range_max)

    sector = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        r = float(r)
        if r != r or r in (float("inf"), float("-inf")):
            continue
        if not (lo < r < hi):
            continue
        a = angle_min + i * inc
        if abs(_wrap(a - bearing_rad)) > half:
            continue
        sector.append((r * math.cos(a), r * math.sin(a)))

    if len(sector) < min_points:
        return no_surface(
            f"only {len(sector)} lidar return(s) inside the "
            f"{math.degrees(2 * half):.0f} deg sector centred "
            f"{math.degrees(bearing_rad):+.0f} deg off the nose; "
            f"nothing to fit a surface to")

    # NEAREST FIRST. The gate stands in FRONT of whatever is behind it, which
    # is the only reason the camera can see it, so the nearest coherent run of
    # returns is the surface being asked about.
    groups = sorted(_clusters(sector, gap_m),
                    key=lambda g: sorted(math.hypot(x, y) for x, y in g)[len(g) // 2])
    nearest = min(math.hypot(x, y) for x, y in groups[0])

    chosen = list(groups[0])
    for group in groups[1:]:
        if min(math.hypot(x, y) for x, y in group) > nearest + max_depth_m:
            break
        trial = chosen + list(group)
        if len(trial) < 2:
            chosen = trial
            continue
        _, _, _, _, rms = _fit_line(trial)
        if rms <= max_residual_m:
            chosen = trial

    if len(chosen) < 2:
        return no_surface(
            f"the nearest surface at {nearest:.1f} m gave {len(chosen)} "
            f"return(s); a single return has no orientation")

    # One re-inclusion pass. The accumulation above starts from whatever
    # cluster happened to be nearest, which for a bare post pair is two or
    # three returns spanning 18 cm; sweeping the whole sector against the
    # resulting line recovers every return that really is on that surface and
    # stops a lucky pair of strays from defining the answer.
    _, _, normal, offset, _ = _fit_line(chosen)
    inliers = [p for p in sector
               if abs(normal[0] * p[0] + normal[1] * p[1] - offset) <= inlier_m]
    if len(inliers) >= len(chosen):
        chosen = inliers
    _, direction, normal, offset, rms = _fit_line(chosen)

    span = _extent(chosen)
    if len(chosen) < min_points:
        return no_surface(
            f"the surface {math.degrees(bearing_rad):+.0f} deg off the nose "
            f"drew only {len(chosen)} return(s), under the {min_points} a fit "
            f"is trusted on")
    if span < min_extent_m:
        return no_surface(
            f"the returns span only {span:.2f} m along the surface, under the "
            f"{min_extent_m:.2f} m needed to fix an orientation; a single post "
            f"is not a face")
    if rms > max_residual_m:
        return no_surface(
            f"the {len(chosen)} returns {math.degrees(bearing_rad):+.0f} deg "
            f"off the nose sit {rms * 100:.0f} cm from any straight line "
            f"(limit {max_residual_m * 100:.0f} cm); that is not a flat face")

    # The normal, pointed FROM the aircraft TOWARD the surface. `offset` is the
    # signed distance from the origin to the line along `normal`, so its sign
    # is exactly which way round that is.
    sign = 1.0 if offset >= 0.0 else -1.0
    mx, my = sign * normal[0], sign * normal[1]
    distance = abs(offset)

    angle = math.atan2(my, mx)
    # EDGE ON IS NOT A MEASUREMENT. A face at 90 degrees to the nose is a
    # surface running away alongside the aircraft, not one it is looking at --
    # and a banner the camera has just identified cannot be edge-on, because
    # then there would be nothing in frame to identify. Watched live: the
    # aircraft alternated between +90 and -90 for fourteen steps, turning to
    # face each reading and making the next one worse.
    if abs(angle) > float(max_obliquity_rad):
        return no_surface(
            f"the face found {math.degrees(bearing_rad):+.0f} deg off the "
            f"nose lies {math.degrees(angle):+.0f} deg to it, edge-on; that "
            f"is a surface running away alongside the aircraft, not one it "
            f"can square up to")

    if expected_range_m is not None and \
            abs(distance - float(expected_range_m)) > float(range_tol_m):
        return no_surface(
            f"the flat face found {math.degrees(bearing_rad):+.0f} deg off the "
            f"nose stands at {distance:.1f} m, not the {float(expected_range_m):.1f} m "
            f"the camera puts the banner at; refusing to square up to "
            f"something else")

    return {"ok": True,
            "angle_rad": angle,
            "range_m": distance,
            "points": len(chosen),
            "residual_m": rms,
            "extent_m": span,
            "reason": ""}


def gate_opening(angle_min, angle_increment, ranges, bearing_rad,
                 half_width_rad, need_clear_m=10.0, range_min=0.05,
                 range_max=12.0, gap_m=0.35, max_depth_m=0.75,
                 min_gap_m=1.0, min_cluster_points=2, corridor_m=1.5):
    """Is there a way THROUGH the gate at this height, or is the board here?

    WHY THIS EXISTS

        Squaring up and passing through are mutually exclusive altitudes, and
        that is not a tuning accident -- it is the geometry. The lidar can
        only measure the board's angle where the scan plane intersects the
        board, which is precisely the height at which the aircraft would fly
        into it. Measured on the shipped arena, the usable band for squareness
        was 2.5 to 3.5 m; the board spans 2.805 to 3.955. The band that makes
        the measurement possible is the band that makes the transit fatal.

        So the aircraft has to find the board's bottom edge and drop below it.
        The edge is not a number to write down -- the real gate will differ,
        and the standing constraint is no fixed arena geometry. It is a
        TRANSITION, and the lidar can see it:

            at board height   one wide continuous face, ~60 returns,
                              residual under 1 cm
            below the board   two narrow post clusters, ~11 returns, a hole
                              between them

        The altitude where the first becomes the second IS the bottom edge.

    Returns:

        open        True when there is a gap to fly through
        clusters    how many separate returns groups the sector holds
        gap_m       lateral width of the hole between them
        gate_m      forward distance to the two surfaces bracketing the hole;
                    None when no gate-like pair was measured
        clear_m     how far the flight line is clear, measured down a
                    corridor `corridor_m` wide -- the width the airframe
                    actually needs, not the width of the hole
        reason      why it is not open, empty when it is

    `open` is False both for a solid face and for a gap with something
    standing in it, and the reason says which. Neither is a thing to fly at.
    """
    half = abs(float(half_width_rad))
    inc = float(angle_increment)
    lo, hi = float(range_min), float(range_max)

    sector = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        r = float(r)
        if r != r or r in (float("inf"), float("-inf")) or not (lo < r < hi):
            continue
        a = angle_min + i * inc
        if abs(_wrap(a - bearing_rad)) > half:
            continue
        sector.append((r * math.cos(a), r * math.sin(a), _wrap(a - bearing_rad)))

    if not sector:
        # Nothing at all in the sector. That is not a measured opening: it is
        # the same reading the aircraft gets pointing at open sky, and the
        # whole point of this check is to distinguish those.
        return {"open": False, "clusters": 0, "gap_m": 0.0,
                "gate_m": None, "clear_m": 0.0,
                "reason": ("no returns in the sector at all, so neither the "
                           "board nor a way past it has been seen")}

    points = [(x, y) for x, y, _ in sector]
    groups = [g for g in _clusters(points, gap_m)
              if len(g) >= min_cluster_points]

    cos_b, sin_b = math.cos(bearing_rad), math.sin(bearing_rad)

    nearest_group = min(
        groups,
        key=lambda g: min(math.hypot(x, y) for x, y in g),
        default=None)
    nearest = (min(math.hypot(x, y) for x, y in nearest_group)
               if nearest_group else min(math.hypot(x, y) for x, y in points))

    # A wide nearest face is the board even when deeper corridor walls are
    # visible around its edges. Run 20 counted all of those deeper groups and
    # reported an impossible 8.2 m "gate" whose real posts are 3.8 m apart.
    nearest_span = (_extent(nearest_group)
                    if nearest_group and len(nearest_group) > 1 else 0.0)

    def _lat(p):
        return -p[0] * sin_b + p[1] * cos_b

    def _fwd(p):
        return p[0] * cos_b + p[1] * sin_b

    def _crosses_line(g):
        # ACROSS the flight line, not ALONG it. Where the corridor walls start
        # at the posts, each post and its wall return as ONE long group, over
        # a metre in extent, and the old span test called that "the board" at
        # every height down to the floor. A board spans the line the aircraft
        # means to fly; a wall runs beside it.
        #
        # AND AT ITS OWN FRONT. In the return lane the first slalom block
        # touches the outer wall, which starts at the gate post: post, wall
        # and block come back as one group whose inner end reaches the flight
        # line 1.2 m BEHIND the post. That is an obstacle past the gate, and
        # the clearance check below deals with it; a board crosses the line at
        # the depth of its posts (seed 1002, return lap).
        front = min(_fwd(p) for p in g)
        near = [_lat(p) for p in g if _fwd(p) <= front + float(max_depth_m)]
        return bool(near) and min(near) <= float(corridor_m) / 2.0 and \
            max(near) >= -float(corridor_m) / 2.0

    if nearest_group is not None and nearest_span >= float(min_gap_m) \
            and _crosses_line(nearest_group):
        return {"open": False, "clusters": 1, "gap_m": 0.0,
                "gate_m": None, "clear_m": 0.0,
                "reason": (f"one continuous surface {nearest_span:.1f} m across at "
                           f"{nearest:.1f} m; this is the board, not the way "
                           f"under it")}

    # The posts are the nearest same-depth pair bracketing the camera bearing.
    # "Outermost groups" is wrong once the corridor walls appear through the
    # gate. They are wider and deeper than the posts, so they inflate both the
    # measured gap and the supposed gate range.
    # Each group is judged by its NEAREST point -- the post, or the end of a
    # wall -- not its centroid. A post that runs on into its corridor wall has
    # a centroid metres deeper than the post itself, which both broke the
    # same-depth pairing and inflated the gate distance.
    def _front(g):
        p = min(g, key=_fwd)
        return _fwd(p), _lat(p), p

    pairs = []
    for i, a in enumerate(groups):
        af, al, ap = _front(a)
        for b in groups[i + 1:]:
            bf, bl, bp = _front(b)
            if af <= 0.0 or bf <= 0.0 or al * bl >= 0.0:
                continue
            if abs(af - bf) > float(max_depth_m):
                continue
            pairs.append((0.5 * (af + bf), a, b, ap, bp))

    if not pairs:
        blocking = [g for g in groups if _crosses_line(g)]
        what = ("one continuous surface" if blocking or len(groups) <= 1
                else f"{len(groups)} surface(s) beside the line, no pair")
        return {"open": False, "clusters": 1 if blocking else len(groups),
                "gap_m": 0.0, "gate_m": None, "clear_m": 0.0,
                "reason": (f"{what} {nearest_span:.1f} m across at "
                           f"{nearest:.1f} m; this is the board, not the way "
                           f"under it")}

    gate, a, b, ap, bp = min(pairs, key=lambda item: item[0])
    gap = math.hypot(bp[0] - ap[0], bp[1] - ap[1])

    # Distance to the gate itself, separate from how far the centreline stays
    # clear behind it. The return corridor puts its first slalom obstacle only
    # a short distance beyond the banner. Without both measurements, the
    # caller has to choose between declaring the whole 10 m open and losing
    # the board-to-post transition altogether.
    # IS THE PATH THE AIRCRAFT INTENDS TO FLY CLEAR?
    #
    # Asked directly, of the flight line, rather than inferred from which
    # cluster is which. An earlier version measured between the two clusters
    # bracketing the sector centre, and an obstacle standing in the mouth of
    # the gate -- nearer than the posts and dead ahead -- was mistaken for one
    # of the posts, so the gate read as open with the obstacle beside the
    # path. A corridor test cannot make that mistake: anything inside the
    # width the airframe needs, along the direction it means to travel, is in
    # the way whatever else it might be.
    ahead = []
    for x, y, _ in sector:
        forward = x * cos_b + y * sin_b
        lateral = -x * sin_b + y * cos_b
        if forward > 0.0 and abs(lateral) <= float(corridor_m) / 2.0:
            ahead.append(forward)
    clear = min(ahead) if ahead else float(range_max)

    if gap < float(min_gap_m):
        return {"open": False, "clusters": len(groups), "gap_m": gap,
                "gate_m": gate, "clear_m": clear,
                "reason": (f"the hole between the two nearest surfaces is "
                           f"only {gap:.1f} m across; too narrow to fly")}
    if clear < float(need_clear_m):
        return {"open": False, "clusters": len(groups), "gap_m": gap,
                "gate_m": gate, "clear_m": clear,
                "reason": (f"a {gap:.1f} m hole with something standing "
                           f"{clear:.1f} m into it, against the "
                           f"{float(need_clear_m):.0f} m the aircraft means "
                           f"to fly")}
    return {"open": True, "clusters": len(groups), "gap_m": gap,
            "gate_m": gate, "clear_m": clear, "reason": ""}
