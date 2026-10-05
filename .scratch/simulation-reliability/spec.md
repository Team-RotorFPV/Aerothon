# Finish the interrupted custom-world reliability work

Inspected on 2026-10-03 at commit `210daa7`, with existing uncommitted
banner recovery, green-ground modelling, detector pacing and world changes.
Those changes were preserved.

The latest saved campaign, `logs/campaign_spec/summary.json`, completed and
passed 7 of 9 worlds. `rb_rotated` failed return alignment after placing a
clipped banner at a fabricated 5 m range. `my_world_offaxis` stalled in the
return corridor until battery-critical RTL.

## Acceptance

- A clipped banner does not provide a position for the return stand-off.
- The rotated and shipped worlds complete and pass independent track grading.
- A stationary aircraft with a visible gap reaches the existing bounded
  recovery ladder instead of resetting the stall every progress window.
- Movement after a stall starts a new progress window.
- Preserve the off-axis world as a failing geometry fixture. Do not widen it
  or reduce aircraft clearance to make its grade pass.
- Record headless, rendered SITL and physical-mechanism evidence separately.

Implementation and evidence are in [status.md](status.md) and the numbered
[issues](issues/) directory.
