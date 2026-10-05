# Resolve the off-axis chicane outside the navigator's envelope

Status: ready-for-agent
Resolution: open
Parent: ../spec.md

The fifth return-corridor obstacle in `sim/worlds/my_world_offaxis.json`
leaves 1.21 m past it. The tightest passage offers 0.45 m either side,
below the navigator's 0.70 m half-width requirement. Validation warns it
will stop in front of this chicane.
The watchdog now terminates that stall, but full mission grading still fails.

Investigate a planner improvement against the preserved world before
changing venue geometry or aircraft-clearance requirements. Such changes
need measured venue/airframe data and an explicit design choice.

Acceptance: the original off-axis geometry completes and passes independent
grading without contact, reduced clearance or a fabricated mission success.
Evidence: `logs/campaign_spec/my_world_offaxis_spec_s1`,
`logs/continue_stall_20261003/my_world_offaxis_spec_s1`.

## Comments

2026-10-03 rendered seed-1 rerun: banner search and alignment recovered,
but return-corridor recovery backed out through the entrance. The mission
reported FAILED at 796.8 simulated seconds, then RTL landed and disarmed.
Full-flight grading remains FAIL, with no obstacle contact. Recorded video
and complete trace: `logs/custom/my_world_offaxis_calm_s1.mp4` and
`logs/custom/my_world_offaxis_calm_s1_full_grade.txt`.
