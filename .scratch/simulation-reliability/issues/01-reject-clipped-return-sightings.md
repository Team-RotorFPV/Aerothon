# Reject clipped return-banner positions

Status: ready-for-agent
Resolution: implemented and verified on 2026-10-03
Parent: ../spec.md

`FindReturnBanner._sighting` clamped a clipped board's area range to the
5 m stand-off distance. `_found` then positioned the approach relative to
that fabricated board position. The next alignment ranged the real board
at about 14 m and pursued unrelated lidar surfaces.

The sighting now waits for a complete board. The existing level sweep and
perimeter vantages recover it. No range, safety or geometry limit was relaxed.

## Comments

The regression `test_a_clipped_distant_board_does_not_become_a_near_sighting`
failed with `(28.0, -3.0, -pi/2, 5.0)` before the fix, then passed. Seed 1
of `rb_rotated` changed from FAILED/FAIL to COMPLETED/PASS, 710.9 simulated
seconds, no contacts or fence breaches. Shipped seed 1 also completed/passed.
Evidence: `logs/continue_baseline_20261003` and `logs/continue_fix_20261003`.
