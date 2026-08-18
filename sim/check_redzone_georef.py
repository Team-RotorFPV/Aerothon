#!/usr/bin/env python3
"""Measure georeferencing accuracy in flight, against the world's own truth.

WHY

    sim/test_redzone_georef.py proves the projection maths is right at nadir,
    and the exclusion x-extents have been eyeballed against the world file.
    Neither answers the question that matters: do the exclusions the aircraft
    ACTUALLY confirms in flight land on the red ground that is ACTUALLY there?

    It matters in both directions. An exclusion placed short makes the search
    avoid clear ground and costs coverage (live run 14's second strip fell to
    71%). An exclusion placed long lets the aircraft overfly red ground while
    believing it is clear, which is the one that loses points.

    `GroundGrid.inflate` biases toward over-covering, which is the safe
    direction — but "biased safe" is not "measured".

WHAT IT DOES

    Listens to /percep/redzone/detail for the confirmed exclusion set, parses
    the true red-zone rectangles out of the runtime SDF, and reports:

      recall     fraction of true red area covered by some exclusion
                 (low = the aircraft would fly over red ground)
      excess     exclusion area landing on ground that is not red, as a
                 multiple of the true red area
                 (high = coverage thrown away for nothing)

    Run it while a mission is sweeping, or just after one finishes — the grid
    is cumulative, so the exclusion set persists.

    source /opt/ros/jazzy/setup.bash
    python3 sim/check_redzone_georef.py
    python3 sim/check_redzone_georef.py --world /tmp/aerothon_mission2_runtime.sdf
"""

import argparse
import json
import re
import sys
import time

# Red-zone model name -> footprint in metres, read from the SDF <box><size>.
RED_MODELS = ("restricted_red_zone_main", "restricted_red_zone_northwest",
              "restricted_red_zone_south")


def true_red_rects(sdf_path):
    """[(x0, x1, y0, y1)] for each red zone, from pose + box size."""
    text = open(sdf_path, encoding="utf-8").read()
    rects = []
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name, body = m.group(1), m.group(2)
        if name not in RED_MODELS:
            continue
        size = re.search(r'<box><size>([\d.eE+-]+) ([\d.eE+-]+)', body)
        # The model <pose> is the LAST one in the block, after </link>.
        pose = re.findall(r'</link>\s*<pose>([^<]*)</pose>', body)
        if not size or not pose:
            continue
        w, h = float(size.group(1)), float(size.group(2))
        v = pose[-1].split()
        cx, cy = float(v[0]), float(v[1])
        rects.append((name, (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)))
    return rects


def grid_area(rects, bounds, step=0.25):
    """Area of the union of `rects`, by sampling. Rectangles overlap heavily
    (the grid confirms cell by cell), so summing them would double-count."""
    x0, x1, y0, y1 = bounds
    if x1 <= x0 or y1 <= y0:
        return 0.0, set()
    cells = set()
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    for i in range(nx):
        x = x0 + i * step
        for j in range(ny):
            y = y0 + j * step
            for r in rects:
                if r[0] <= x <= r[1] and r[2] <= y <= r[3]:
                    cells.add((i, j))
                    break
    return len(cells) * step * step, cells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="/tmp/aerothon_mission2_runtime.sdf")
    ap.add_argument("--listen", type=float, default=12.0)
    ap.add_argument("--min-recall", type=float, default=0.60)
    args = ap.parse_args()

    truth = true_red_rects(args.world)
    if not truth:
        print(f"no red zones found in {args.world}", file=sys.stderr)
        return 2
    print(f"true red zones in {args.world}:")
    for name, r in truth:
        print(f"  {name:34s} x {r[0]:7.2f}..{r[1]:7.2f}  y {r[2]:7.2f}..{r[3]:7.2f}")

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("redzone_georef_check")
    got = {}
    node.create_subscription(String, "/percep/redzone/detail",
                             lambda m: got.__setitem__("d", m.data), 10)
    t0 = time.time()
    while time.time() - t0 < args.listen:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "d" not in got:
        print("\nnothing published on /percep/redzone/detail", file=sys.stderr)
        return 2
    detail = json.loads(got["d"])
    got_rects = [tuple(e) for e in (detail.get("exclusions") or [])]
    print(f"\nconfirmed exclusions: {len(got_rects)}  "
          f"(status {detail.get('status')})")
    if not got_rects:
        print("no exclusions confirmed — fly a mission over the red zones first")
        return 1

    # Sample over everything either set touches.
    xs = [r[0] for _n, r in truth] + [r[0] for r in got_rects]
    xe = [r[1] for _n, r in truth] + [r[1] for r in got_rects]
    ys = [r[2] for _n, r in truth] + [r[2] for r in got_rects]
    ye = [r[3] for _n, r in truth] + [r[3] for r in got_rects]
    bounds = (min(xs) - 2, max(xe) + 2, min(ys) - 2, max(ye) + 2)

    true_area, true_cells = grid_area([r for _n, r in truth], bounds)
    got_area, got_cells = grid_area(got_rects, bounds)
    hit = len(true_cells & got_cells)
    recall = hit / len(true_cells) if true_cells else 0.0
    excess = (len(got_cells - true_cells) / len(true_cells)) if true_cells else 0.0

    print(f"\n  true red area      {true_area:8.1f} m^2")
    print(f"  excluded area      {got_area:8.1f} m^2")
    print(f"  RECALL             {recall:8.2f}   "
          f"(fraction of red ground actually excluded)")
    print(f"  EXCESS             {excess:8.2f}   "
          f"(clear ground excluded, as a multiple of the red area)")

    # Per-zone, so a single missed zone is not hidden by two good ones.
    print("\n  per zone:")
    for name, r in truth:
        _a, cells = grid_area([r], bounds)
        cov = len(cells & got_cells) / len(cells) if cells else 0.0
        seen = "seen" if cov > 0.05 else "NOT SEEN"
        print(f"    {name:34s} recall {cov:4.2f}  {seen}")

    ok = recall >= args.min_recall
    print(f"\n  VERDICT: {'ok' if ok else 'RECALL BELOW THRESHOLD'} "
          f"(threshold {args.min_recall:.2f})")
    print("  Note: a zone the aircraft never overflew cannot be detected; "
          "check the swept area before reading a low recall as a fault.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
