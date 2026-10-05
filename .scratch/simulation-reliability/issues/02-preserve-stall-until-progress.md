# Preserve a corridor stall until movement earns a new window

Status: ready-for-agent
Resolution: implemented and verified on 2026-10-03
Parent: ../spec.md

`VelocityController.stalled` replaced its reference even when the aircraft
had not moved. This made a stall last one tick per 12 s window. CRUISE then
cleared the blocked counter, preventing the existing backoff ladder.

Retain the expired reference until displacement meets `min_progress_m`.
The existing backoff counts, speeds and STUCK outcome are unchanged.

## Comments

A 120 s virtual-time test with a stationary aircraft and a visible gap
never entered BACKOFF before the fix. Afterward it exhausts recovery and
reports STUCK. A second regression checks that measured movement resets it.
Corridor recovery/stress/yaw tests passed, 47 tests before the additional
movement-reset regression. The final targeted set passed 95 tests.

Full `my_world_offaxis` seed 1 now attempts all three backoffs and reports
`FAILED: ReturnCorridor: corridor navigator reported STUCK at x=12.76` at
766.6 s. The saved old run waited until battery-critical RTL at 1046.4 s.
This is a bounded failure, not successful corridor traversal.
Evidence: `logs/continue_stall_20261003`.
