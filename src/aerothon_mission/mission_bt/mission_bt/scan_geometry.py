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


def _extent(points, direction):
    ux, uy = direction
    projections = [x * ux + y * uy for x, y in points]
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
        extent_m    how far along the surface the returns span
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

    span = _extent(chosen, direction)
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

    if expected_range_m is not None and \
            abs(distance - float(expected_range_m)) > float(range_tol_m):
        return no_surface(
            f"the flat face found {math.degrees(bearing_rad):+.0f} deg off the "
            f"nose stands at {distance:.1f} m, not the {float(expected_range_m):.1f} m "
            f"the camera puts the banner at; refusing to square up to "
            f"something else")

    return {"ok": True,
            "angle_rad": math.atan2(my, mx),
            "range_m": distance,
            "points": len(chosen),
            "residual_m": rms,
            "extent_m": span,
            "reason": ""}
