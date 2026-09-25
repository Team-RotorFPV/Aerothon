#!/usr/bin/env python3
"""Headless self-check for the Mission 2 logic — PASS/FAIL, no ROS needed.

Runs the same simulate() used by the animation and asserts the rulebook
success criteria (Q on scoring):
  * mission completes (lands)
  * the correct TARGET QR is reached (delivery)
  * the RED zone is never entered (-5 pt penalty avoided)
  * it finishes in a sane number of steps (within time)

Run:  python3 sim/check_mission.py   (exit code 0 = PASS)
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from mission_sim import simulate, TARGET, in_red  # noqa: E402


def main():
    hist = simulate()
    positions = [h[0] for h in hist]
    states = [h[1] for h in hist]

    final_state = states[-1]
    min_target = min(math.dist(p, TARGET) for p in positions)
    red_hits = sum(1 for p in positions if in_red(p[0], p[1]))
    reached_target = min_target < 1.4
    visited_states = set(states)
    expected = {"TAKEOFF", "CORRIDOR", "SEARCH", "DROP", "CORRIDOR_BACK", "LAND"}

    checks = [
        ("Mission lands",              final_state == "LAND"),
        ("Reaches TARGET QR",          reached_target),
        ("Never enters RED zone",      red_hits == 0),
        ("All key stages executed",    expected <= visited_states),
        ("Finishes within step budget", len(hist) < 900),
    ]

    print("=" * 52)
    print(" Mission 2 self-check")
    print("=" * 52)
    print(f"  final state          : {final_state}")
    print(f"  min dist to TARGET   : {min_target:.2f} m")
    print(f"  red-zone incursions  : {red_hits}")
    print(f"  recorded frames      : {len(hist)}")
    print("-" * 52)
    ok = True
    for name, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
        ok = ok and passed
    print("=" * 52)
    print(" RESULT:", "PASS ✅" if ok else "FAIL ❌")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
