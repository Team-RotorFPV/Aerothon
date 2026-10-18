"""Connected local paths from lidar observations, with no venue inputs.

Unknown rays are blocked. Clearance is a hard centre-to-surface distance,
including discretisation error, and is never shrunk near an obstacle.
"""
import heapq
import math

import numpy as np
from scipy.spatial import cKDTree


class LocalPlanner:
    def __init__(self, clearance=0.70, resolution=0.05, lookahead=5.0, occupied_radius=0.025):
        if clearance < 0.70:
            raise ValueError('minimum corridor clearance is 0.70 m')
        self.clearance = clearance
        self.resolution = resolution
        self.lookahead = lookahead
        if not 0<occupied_radius<=clearance:
            raise ValueError('invalid occupied aircraft footprint')
        self.occupied_radius = occupied_radius
        self.points = np.empty((0, 2))
        self.tree = None
        self.bearings = np.array([])
        self.ranges = np.array([])
        self.path = []
        self.reason = 'no observation'
        self.sensor_origin = np.zeros(2)

    def observe(self, bearings, ranges, max_range=12.0, sensor_origin=(0, 0), hits=None):
        b, r = np.asarray(bearings), np.asarray(ranges)
        valid = np.isfinite(b) & np.isfinite(r) & (r > 0)
        order = np.argsort(b[valid])
        self.bearings, self.ranges = b[valid][order], r[valid][order]
        self.sensor_origin = np.asarray(sensor_origin, dtype=float)
        hit = self.ranges < max_range - 0.001 if hits is None else np.asarray(hits)[valid][order]
        self.points = self.sensor_origin + np.column_stack((self.ranges[hit]*np.cos(self.bearings[hit]),
                                      self.ranges[hit]*np.sin(self.bearings[hit])))
        self.tree = cKDTree(self.points) if len(self.points) else None

    def distances(self, xy):
        xy = np.asarray(xy).reshape(-1, 2)
        return self.tree.query(xy)[0] if self.tree is not None else np.full(len(xy),np.inf)

    def observed(self, xy):
        xy = np.asarray(xy).reshape(-1, 2)
        if not len(self.bearings):
            return np.zeros(len(xy), dtype=bool)
        relative = xy-self.sensor_origin
        a = np.arctan2(relative[:, 1], relative[:, 0])
        i = np.searchsorted(self.bearings, a)
        lo, hi = np.clip(i-1, 0, len(self.bearings)-1), np.clip(i, 0, len(self.bearings)-1)
        dl, dh = np.abs(a-self.bearings[lo]), np.abs(a-self.bearings[hi])
        nearest = np.where(dl < dh, lo, hi)
        # A missing/masked sector must not interpolate into free space.
        close = np.minimum(dl, dh) <= math.radians(2.0)
        depth = self.ranges[nearest]
        return close & (np.linalg.norm(relative, axis=1) <= depth-0.025)

    def segment_clear(self, start, end):
        n = max(2, int(math.dist(start, end)/0.05)+2)
        xy = np.linspace(start, end, n)
        return self._clear_samples(xy)

    def _clear_samples(self, xy):
        """Prove clearance between samples without taxing a stationary origin.

        Distance to fixed observed returns is 1-Lipschitz. For an interval
        of length L with endpoint distances da and db, (da + db - L)/2 is
        a lower bound throughout that interval. Subdivide only ambiguous
        intervals, keeping the original observation checks and hard limit.
        A bounded refinement that cannot prove clearance remains blocked.
        """
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        if len(xy) < 2 or not np.isfinite(xy).all() or not self.known(xy).all():
            return False
        distances = self.distances(xy)
        if (distances < self.clearance).any():
            return False
        a, b = xy[:-1], xy[1:]
        da, db = distances[:-1], distances[1:]
        for _ in range(10):
            length = np.linalg.norm(b-a, axis=1)
            certified = (da+db-length)/2 >= self.clearance
            if certified.all():
                return True
            a, b = a[~certified], b[~certified]
            da, db = da[~certified], db[~certified]
            midpoint = (a+b)/2
            if not self.known(midpoint).all():
                return False
            dm = self.distances(midpoint)
            if (dm < self.clearance).any():
                return False
            a, b = np.concatenate((a, midpoint)), np.concatenate((midpoint, b))
            da, db = np.concatenate((da, dm)), np.concatenate((dm, db))
        return False

    def polyline_clear(self, points):
        """Batch the identical samples used by each segment certificate."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points) < 2 or not np.isfinite(points).all():
            return False
        samples = [np.linspace(a, b, max(2, int(math.dist(a, b)/0.05)+2))
                   for a, b in zip(points, points[1:])]
        xy = np.concatenate(samples)
        return self._clear_samples(xy)

    def known(self, xy):
        """Observed rays plus the narrow occupied link to the mounted sensor.

        A sensor on the nose cannot see the body-to-sensor link behind its
        origin. Connect through that existing footprint, capped by aircraft
        radius. This does not declare the rest of a masked sector free.
        Every accepted movement still needs the full surface clearance.
        """
        xy = np.asarray(xy).reshape(-1,2)
        length2 = float(self.sensor_origin @ self.sensor_origin)
        fraction = np.clip(xy @ self.sensor_origin/max(length2,1e-12),0,1)
        link_distance = np.linalg.norm(xy-fraction[:,None]*self.sensor_origin,axis=1)
        occupied = ((np.linalg.norm(xy,axis=1)<self.occupied_radius) &
                    (link_distance<0.025))
        return self.observed(xy) | occupied

    def plan(self, axis_bearing=0.0):
        """Dijkstra over observed cells; return a short certified polyline."""
        self.path = []
        if not self.segment_clear((0, 0), (0, 0)):
            self.reason = 'aircraft lacks required surface clearance'
            return []
        straight = (self.lookahead*math.cos(axis_bearing),
                    self.lookahead*math.sin(axis_bearing))
        if self.segment_clear((0,0),straight):
            self.path, self.reason = [(0,0),straight], 'connected lidar path'
            return self.path
        step = self.resolution
        n = int(self.lookahead/step)
        ij = np.array([(i, j) for i in range(-n, n+1) for j in range(-n, n+1)])
        rotation = np.array([[math.cos(axis_bearing),math.sin(axis_bearing)],
                             [-math.sin(axis_bearing),math.cos(axis_bearing)]])
        xy = ij*step @ rotation
        # Each chosen edge is checked at <= 5 cm spacing afterwards. Reserve
        # half that spacing instead of inflating by a whole grid-cell radius.
        margin = 0.025
        free = self.known(xy) & (self.distances(xy) >= self.clearance+margin)
        distances = self.distances(xy)
        travel_cost = {tuple(p): 1.0+0.10/max(0.05, float(d)-self.clearance)**2
                       for p,d in zip(ij[free],distances[free])}
        forward = ij[:,0]*step
        free &= forward >= -0.3
        allowed = {tuple(p) for p in ij[free]}
        allowed.add((0, 0))
        cost, parent = {(0, 0): 0.0}, {}
        queue = [(0.0, (0, 0))]
        # The corridor-aligned lattice need not align with a nose-mounted
        # sensor's occupied link. Connect the start to nearby observed cells
        # only through fully certified segments, instead of requiring an
        # axial grid step through the masked rear sector.
        bridge_radius = min(self.occupied_radius,
                            float(np.linalg.norm(self.sensor_origin))+2*step)
        if float(np.linalg.norm(self.sensor_origin))>0.025 and bridge_radius>0.025:
            for cell in allowed:
                if cell==(0,0) or math.hypot(*cell)*step>bridge_radius:
                    continue
                point=tuple(np.array(cell)*step @ rotation)
                if self.segment_clear((0,0),point):
                    value=math.hypot(*point)*travel_cost.get(cell,1.0)
                    cost[cell],parent[cell]=value,(0,0)
                    heapq.heappush(queue,(value,cell))
        best, score = (0, 0), 0.0
        directions = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        while queue:
            value, cell = heapq.heappop(queue)
            if value != cost[cell]:
                continue
            progress = cell[0]*step
            merit = progress-0.08*step*math.hypot(*cell)
            if merit > score:
                best, score = cell, merit
            for di, dj in directions:
                nxt = cell[0]+di, cell[1]+dj
                if nxt not in allowed:
                    continue
                if di and dj and ((cell[0]+di,cell[1]) not in allowed or
                                  (cell[0],cell[1]+dj) not in allowed):
                    continue
                candidate = value+step*math.hypot(di,dj)*travel_cost.get(nxt,1.0)
                if candidate < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = candidate, cell
                    heapq.heappush(queue,(candidate,nxt))
        if best == (0,0) or score < 0.3:
            self.reason = 'no connected observed path with required clearance'
            return []
        cells = [best]
        while cells[-1] != (0,0):
            cells.append(parent[cells[-1]])
        raw = [tuple(np.array(c)*step @ rotation) for c in reversed(cells)]
        # Shortcut only after checking every point of the new segment.
        path, k = [raw[0]], 0
        raw_clearance = self.distances(raw)
        while k < len(raw)-1:
            # The same three-tier rule the compiled planner uses: the normal
            # shortcut, then -- for an origin already inside the reserve,
            # which the first tier can never satisfy -- the longest shortcut
            # that gives away no clearance at all, and only then the original
            # allowance that keeps such an origin reachable. A shortcut
            # exists to lengthen the controller's lead, not to spend the
            # clearance the grid search preserved.
            def preserves_clearance(j, tier):
                running = min(raw_clearance[k:j+1])
                threshold = (max(self.clearance+margin, running-0.025), running,
                             running-0.025)[tier]
                samples = np.linspace(raw[k],raw[j],max(2,int(math.dist(raw[k],raw[j])/.05)+2))
                return (self.segment_clear(raw[k],raw[j]) and
                        min(self.distances(samples)) >= threshold)
            j = k+1
            for tier in range(3):
                candidate = len(raw)-1
                while candidate > k+1 and not preserves_clearance(candidate,tier):
                    candidate -= 1
                if candidate > k+1:
                    j = candidate
                    break
            if not self.segment_clear(raw[k],raw[j]):
                self.reason = 'segment failed continuous clearance check'
                return []
            path.append(raw[j]); k = j
        self.path, self.reason = path, 'connected lidar path'
        return path
