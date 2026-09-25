#!/usr/bin/env python3
"""Compare the red-zone map the aircraft has built with the painted truth.

    python3 sim/probe_redzone_map.py --layout /tmp/aerothon_arena_layout.json

Reads the latest /percep/redzone/detail, clusters its confirmed cells, and
prints each cluster's bounding box beside the true red rectangles in the
same home-local frame. A map that is shifted, mirrored or rotated shows up
as clusters that do not sit on the truth.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))


def clusters(rects, gap=0.01):
    groups = []
    for r in rects:
        placed = None
        for g in groups:
            if any(not (r[1] + gap < o[0] or o[1] + gap < r[0]
                        or r[3] + gap < o[2] or o[3] + gap < r[2]) for o in g):
                if placed is None:
                    g.append(r)
                    placed = g
                else:
                    placed.extend(g)
                    g.clear()
        if placed is None:
            groups.append([r])
        groups = [g for g in groups if g]
    return [(min(r[0] for r in g), max(r[1] for r in g),
             min(r[2] for r in g), max(r[3] for r in g), len(g)) for g in groups]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--inflate", type=float, default=1.0,
                    help="the node's inflate_m, removed before comparing")
    args = ap.parse_args()
    lay = json.loads(Path(args.layout).read_text())
    hx, hy = lay["home_world"]
    rclpy.init()
    node = Node("probe_redzone_map")
    last = {}
    node.create_subscription(String, "/percep/redzone/detail",
                             lambda m: last.update(json.loads(m.data)), 10)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    ex = [tuple(e) for e in last.get("exclusions", [])]
    k = args.inflate
    cells = [(a + k, b - k, c + k, d - k) for a, b, c, d in ex]
    print(f"status {last.get('status')}  cells {len(cells)}")
    for x0, x1, y0, y1, n in clusters(cells):
        print(f"  mapped  x {x0:6.1f}..{x1:6.1f}  y {y0:6.1f}..{y1:6.1f}  ({n} cells)")
    from check_track import red_polygons      # sized, headed zones too
    for name, poly in red_polygons(lay):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        print(f"  TRUTH   x {min(xs):6.1f}..{max(xs):6.1f}  "
              f"y {min(ys):6.1f}..{max(ys):6.1f}  {name}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
