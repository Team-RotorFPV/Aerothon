"""Measured LiDAR mounting and range calibration from repeated raw scans.

The GCS records measured XYZ/tilt, a background and four stable panel captures.
Independent direction/range checks qualify the result. The quick two-bearing
solver remains shared with scripts/check_sensors.py as a bench diagnostic.
"""

import math
import time
from collections import deque

import numpy as np

SELF_M = 0.3          # nearer than this is the airframe or the bracket
MAX_TURN_ERROR_DEG = 3.0


def nearest_bearing(scan, self_m=SELF_M):
    """(bearing_deg, range_m) of the nearest return in the scan's own frame,
    or None. Returns nearer than `self_m` are the airframe."""
    lo = max(float(self_m), float(getattr(scan, "range_min", 0.0)))
    best = None
    for i, r in enumerate(scan.ranges):
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r < lo or r > float(getattr(scan, 'range_max', math.inf)):
            continue
        if best is None or r < best[1]:
            best = (i, r)
    if best is None:
        return None
    a = scan.angle_min + best[0] * scan.angle_increment
    return math.degrees(math.atan2(math.sin(a), math.cos(a))), best[1]


def solve_mount(front_deg, port_deg):
    """(lidar_yaw_deg, mirrored, error_deg) that put `front` on the nose and
    `port` at +90: body = raw + yaw, or -raw + yaw when the driver runs
    clockwise. `error_deg` is how far the two captures are from 90 deg
    apart -- large means a box was not where it was said to be."""
    turn = (port_deg - front_deg + 180.0) % 360.0 - 180.0
    mirrored = turn < 0.0
    estimates = [front_deg, port_deg + 90] if mirrored else [-front_deg, 90 - port_deg]
    yaw = math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in estimates),
                                 sum(math.cos(math.radians(a)) for a in estimates)))
    return (yaw + 180.0) % 360.0 - 180.0, mirrored, abs(abs(turn) - 90.0)


def result(captures):
    """The calibration from {'ahead': (deg, m), 'port': (deg, m)}, or None
    while either is missing. Never claims a mount it cannot stand behind."""
    ahead, port = captures.get("ahead"), captures.get("port")
    if not ahead or not port:
        return None
    yaw, mirrored, err = solve_mount(ahead[0], port[0])
    ok = err <= MAX_TURN_ERROR_DEG
    return {"yaw_deg": round(yaw, 1), "mirrored": bool(mirrored),
            "error_deg": round(err, 1), "ok": ok,
            "reason": "" if ok else (
                f"the port box read {err:.0f} deg away from 90 deg off the nose; "
                "capture again with each box square to the aircraft")}


class CalibrationSession:
    """Measured extrinsics plus repeated, background-isolated panel captures.

    A horizontal scanner cannot infer its height or tilt from an unknown room.
    Those are operator measurements. Yaw, direction and range are independently
    checked against targets placed relative to the optical centre, parallel to
    the airframe axes. No GPS, nearest-point guess or flight truth is used.
    """
    MIN_SCANS = 15
    TARGETS = ('ahead', 'far', 'port', 'starboard')

    def __init__(self):
        self.history = deque(maxlen=30)
        self.geometry = None
        self.baseline = None
        self.captures = {}
        self.last_stamp = None

    def observe(self, scan, receipt=None):
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        if not math.isfinite(stamp) or stamp <= 0:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.history.clear()
            self.baseline = None
            self.captures.clear()
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        self.history.append((scan, time.monotonic() if receipt is None else receipt, stamp))

    def measure_geometry(self, values):
        geometry = {}
        for key, bound in (('x_m', 2.), ('y_m', 2.), ('z_m', 2.),
                           ('roll_deg', 3.), ('pitch_deg', 3.)):
            v = values.get(key)
            if isinstance(v, bool) or v is None:
                raise ValueError(f'enter measured {key}; automatic geometry is unavailable')
            v = float(v)
            if not math.isfinite(v) or abs(v) > bound:
                raise ValueError(f'{key} must be within +/-{bound}; level the scanner for 2D mapping')
            geometry[key] = v
        self.geometry = geometry
        self.baseline = None
        self.captures.clear()
        self.history.clear()

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        rows = [s for s in self.history if now - s[1] <= 2.5]
        if len(rows) < self.MIN_SCANS or now - rows[-1][1] > .4:
            raise ValueError('need 15 fresh, distinct scans; hold the aircraft and target still for 2 seconds')
        rows = rows[-self.MIN_SCANS:]
        if rows[-1][2] - rows[0][2] < 1.0 or max(np.diff([r[2] for r in rows])) > .3:
            raise ValueError('scan timestamps are too sparse or inconsistent; check the lidar rate')
        first = rows[0][0]
        key = lambda s: (len(s.ranges), s.angle_min, s.angle_increment, s.header.frame_id)
        if any(key(r[0]) != key(first) for r in rows):
            raise ValueError('raw scan geometry changed; capture a new background')
        n = len(first.ranges)
        if n < 90 or n > 4096 or not math.isfinite(first.angle_increment) or first.angle_increment == 0:
            raise ValueError('invalid raw scan geometry')
        ranges = np.asarray([r[0].ranges for r in rows], dtype=float)
        valid = np.isfinite(ranges) & (ranges >= max(SELF_M, first.range_min)) & (ranges <= first.range_max)
        ranges = np.where(valid, ranges, np.inf)
        return first, ranges, np.median(ranges, axis=0), rows[-1][2]

    def capture_background(self):
        if self.geometry is None:
            raise ValueError('record measured mounting position and tilt first')
        first, _, median, stamp = self.snapshot()
        self.baseline = (len(median), first.angle_min, first.angle_increment, median, stamp)
        self.captures.clear()

    def capture(self, where, distance_m):
        if where not in self.TARGETS:
            raise ValueError('capture ahead, far, port or starboard')
        if self.geometry is None or self.baseline is None:
            raise ValueError('record mount measurements and capture the empty background first')
        distance = float(distance_m)
        if not math.isfinite(distance) or not .4 <= distance <= 4.:
            raise ValueError('measure target distance from the lidar optical centre, between 0.4 and 4 m')
        first, ranges, median, stamp = self.snapshot()
        n, a0, inc, baseline, baseline_stamp = self.baseline
        if (len(median), first.angle_min, first.angle_increment) != (n, a0, inc):
            raise ValueError('scan geometry changed; capture the background again')
        if stamp - baseline_stamp < 1.2:
            raise ValueError('wait 2 seconds after placing the target')
        changed = np.isfinite(median) & (median >= .4) & (median <= 4.5) & (median < baseline - .15)
        # Connected components, including a panel crossing the scan wrap.
        indexes = np.flatnonzero(changed)
        groups = np.split(indexes, np.flatnonzero(np.diff(indexes) > 1) + 1)
        if len(groups) > 1 and len(groups[0]) and groups[0][0] == 0 and groups[-1][-1] == n - 1:
            groups = [np.concatenate((groups[-1], groups[0]))] + groups[1:-1]
        candidates = [g for g in groups if len(g) >= 5 and
                      1.5 <= math.degrees(abs(inc)) * len(g) <= 45.]
        if len(candidates) != 1:
            raise ValueError('need one isolated panel with at least 5 adjacent beams; remove moving clutter or widen the target')
        g = candidates[0]
        angles = a0 + g * inc
        bearing = math.atan2(float(np.sin(angles).mean()), float(np.cos(angles).mean()))
        relative = np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing))
        projected = ranges[:, g] * np.cos(relative)
        if np.any(np.isfinite(projected).sum(axis=1) < .8 * len(g)):
            raise ValueError('target returns are intermittent; use a wider matte panel')
        depths = np.median(projected, axis=1)
        depth = float(np.median(depths))
        spread = float(np.max(np.abs(depths - depth)))
        if not math.isfinite(depth) or spread > .025:
            raise ValueError('target or aircraft moved by more than 25 mm; hold still and recapture')
        # Verify that the target edges are stable in every individual scan.
        edges = []
        for row in ranges:
            present = changed & np.isfinite(row) & (row < baseline - .15)
            hit = np.array([i for i in g if present[i]], dtype=int)
            if len(hit) < .8 * len(g):
                raise ValueError('target shape is not stable across scans')
            aa = a0 + hit * inc
            edges.append(math.atan2(float(np.sin(aa).mean()), float(np.cos(aa).mean())))
        angular_spread = math.degrees(max(abs(math.atan2(math.sin(a - bearing), math.cos(a - bearing))) for a in edges))
        if angular_spread > 1.:
            raise ValueError('target bearing varies by more than 1 degree; hold still')
        self.captures[where] = {'bearing_deg': math.degrees(bearing), 'range_m': depth,
                                'distance_m': distance, 'samples': self.MIN_SCANS,
                                'beams': len(g), 'spread_m': spread,
                                'spread_deg': angular_spread, 'stamp': stamp}

    def result(self):
        if any(k not in self.captures for k in self.TARGETS):
            return None
        a, f, p, s = [self.captures[k] for k in self.TARGETS]
        yaw, mirrored, square_error = solve_mount(a['bearing_deg'], p['bearing_deg'])
        sign = -1 if mirrored else 1
        residuals = [abs((sign * c['bearing_deg'] + yaw - desired + 180) % 360 - 180)
                     for c, desired in ((a, 0), (f, 0), (p, 90), (s, -90))]
        if abs(f['distance_m'] - a['distance_m']) < .5 or abs(f['range_m'] - a['range_m']) < .4:
            return {'ok': False, 'reason': 'the far target must be at least 0.5 m farther than the ahead target'}
        scale = (f['distance_m'] - a['distance_m']) / (f['range_m'] - a['range_m'])
        bias = a['distance_m'] - scale * a['range_m']
        check_m = max(abs(scale * c['range_m'] + bias - c['distance_m']) for c in (p, s))
        ok = square_error <= 3 and max(residuals) <= 2 and .9 <= scale <= 1.1 and abs(bias) <= .1 and check_m <= .03
        return dict(self.geometry, yaw_deg=round(yaw, 3), mirrored=mirrored,
                    range_scale=scale, range_bias_m=bias, error_deg=round(max(residuals), 3),
                    validation_error_m=round(check_m, 4), ok=bool(ok),
                    reason='' if ok else 'independent direction/range checks failed; remeasure panel positions and recapture',
                    quality={'method': 'measured extrinsics, 15-scan background subtraction, 4 panels',
                             'captures': self.captures, 'measured_at': time.time()})
