#!/usr/bin/env python3
"""Projecting image detections onto the ground plane.

WHAT THIS REPLACES (Phase 7)

    perception_redzone published a single Bool: "red is visible somewhere in
    frame". That is not a position. A detection with no position cannot be
    avoided, cannot be routed around, and cannot be shown on a map — the
    mission could only ever react to it by being vaguely cautious.

    Worse, a Bool cannot distinguish the three states an operator needs:

        NOT VISIBLE   camera cannot see the ground there (wrong pose, too low)
        CLEAR         camera CAN see it and there is no red
        RED           red observed, here, at these coordinates

    The first two were the same value.

HOW A PIXEL BECOMES A COORDINATE

    A pinhole ray through the pixel, rotated into the local frame by the
    aircraft's yaw and the camera's pitch, intersected with the ground plane.
    The aircraft's altitude gives the scale — this is the same geometry the
    QR decode envelope uses, run backwards.

    Camera axes are derived rather than asserted, because asserting a sign is
    what flew the aircraft into a wall in Phase 2:

        optical axis   o = ( cos phi, 0, -sin phi)   phi = 0 forward, pi/2 down
        image right    r = ( 0, -1, 0)               body +y is LEFT in FLU
        image down     d = o x r = (-sin phi, 0, -cos phi)

    Check the two poses by hand:
      phi = 0     (forward): image-down is (0,0,-1) = world down. Correct.
      phi = pi/2  (nadir):   image-down is (-1,0,0) = BEHIND the aircraft,
                             which matches "flying forward moves the scene up
                             the image" — the mapping CenterOnQR relies on.

Pure geometry, no ROS: testable without a simulator, and reusable by the GCS
to draw what the aircraft believes it has seen.
"""

import math


def focal_px(image_width_px, hfov_rad):
    """Pinhole focal length in pixels from the horizontal field of view."""
    return (image_width_px / 2.0) / math.tan(hfov_rad / 2.0)


def camera_axes(pitch_down_rad):
    """(optical, right, down) unit vectors in body FLU for a camera pitch.

    `pitch_down_rad` is 0 looking forward and +pi/2 looking straight down,
    matching the named poses in camera_ctrl.
    """
    c, s = math.cos(pitch_down_rad), math.sin(pitch_down_rad)
    optical = (c, 0.0, -s)
    right = (0.0, -1.0, 0.0)
    down = (-s, 0.0, -c)
    return optical, right, down


def ground_point(u, v, image_wh, hfov_rad, altitude_m, aircraft_xy, yaw_rad,
                 pitch_down_rad):
    """Where pixel (u, v) meets the ground, in the local ENU frame.

    Returns (x, y) or None when the ray cannot reach the ground: pointing at
    or above the horizon, or the aircraft already on the deck. None means
    "unknown", never (0, 0) — a false origin is worse than no answer.
    """
    w, h = image_wh
    if altitude_m <= 0.0:
        return None

    f = focal_px(w, hfov_rad)
    xn = (u - w / 2.0) / f
    yn = (v - h / 2.0) / f

    o, r, d = camera_axes(pitch_down_rad)
    bx = o[0] + xn * r[0] + yn * d[0]
    by = o[1] + xn * r[1] + yn * d[1]
    bz = o[2] + xn * r[2] + yn * d[2]

    # Rotate body FLU into local ENU by yaw about z.
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    dx = bx * cy - by * sy
    dy = bx * sy + by * cy
    dz = bz

    if dz >= -1e-6:                      # at or above the horizon
        return None

    t = altitude_m / -dz
    ax, ay = aircraft_xy
    return (ax + t * dx, ay + t * dy)


def footprint(image_wh, hfov_rad, altitude_m, aircraft_xy, yaw_rad,
              pitch_down_rad):
    """Ground quadrilateral the camera can currently see, or None.

    This is what makes CLEAR distinguishable from NOT VISIBLE: "no red in the
    image" only means the ground is clear WHERE THE CAMERA WAS LOOKING, and
    that region is exactly this polygon. If any corner cannot reach the ground
    the view runs to the horizon and no bounded claim can be made.
    """
    w, h = image_wh
    corners = []
    # Pixel EDGES, not centres: the sensor spans [0, w], so using w - 1 would
    # report a footprint half a pixel small and off-centre.
    for u, v in ((0, 0), (w, 0), (w, h), (0, h)):
        g = ground_point(u, v, image_wh, hfov_rad, altitude_m, aircraft_xy,
                         yaw_rad, pitch_down_rad)
        if g is None:
            return None
        corners.append(g)
    return corners


def bbox(points):
    """(x0, x1, y0, y1) of a point list."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), max(xs), min(ys), max(ys))


# --------------------------------------------------------------------------- #
# Accumulating detections into exclusion zones
# --------------------------------------------------------------------------- #

class GroundGrid:
    """Occupancy of red ground, accumulated across frames.

    A single frame's detection is not trustworthy enough to plan around: HSV
    picks up a red jacket, a lens flare, a wet patch of clay. Requiring a cell
    to be hit `confirm_hits` times from (potentially) different viewpoints is
    what turns a detection into a zone.

    Deliberately a grid rather than a polygon tracker: cells are cheap, merge
    for free, and cannot produce the self-intersecting polygons that a naive
    contour-union generates when the aircraft circles a zone.
    """

    def __init__(self, cell_m=1.0, confirm_hits=3, decay=False):
        if cell_m <= 0:
            raise ValueError("cell_m must be positive")
        self.cell_m = float(cell_m)
        self.confirm_hits = int(confirm_hits)
        self.decay = decay
        self.hits = {}

    def _key(self, x, y):
        return (int(math.floor(x / self.cell_m)),
                int(math.floor(y / self.cell_m)))

    def add(self, points):
        """Record ground points observed as red."""
        for x, y in points:
            k = self._key(x, y)
            self.hits[k] = self.hits.get(k, 0) + 1

    def confirmed_cells(self):
        return [k for k, n in self.hits.items() if n >= self.confirm_hits]

    def cell_rect(self, key):
        i, j = key
        return (i * self.cell_m, (i + 1) * self.cell_m,
                j * self.cell_m, (j + 1) * self.cell_m)

    def exclusions(self, inflate_m=0.0):
        """Confirmed red as a list of (x0, x1, y0, y1) rectangles.

        `inflate_m` grows each rectangle so the aircraft keeps a margin: the
        grid marks where red WAS SEEN, and the boundary is only known to within
        a cell plus whatever the projection got wrong.
        """
        out = []
        for k in self.confirmed_cells():
            x0, x1, y0, y1 = self.cell_rect(k)
            out.append((x0 - inflate_m, x1 + inflate_m,
                        y0 - inflate_m, y1 + inflate_m))
        return out

    def area_m2(self):
        return len(self.confirmed_cells()) * self.cell_m ** 2
