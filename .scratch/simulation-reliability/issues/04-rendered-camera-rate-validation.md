# Verify detector-rate pacing in rendered Gazebo and SITL

Status: ready-for-agent
Resolution: open
Parent: ../spec.md

The last saved rendered `my_world` mission completed but failed
restricted-zone grading (`logs/custom/my_world_grade.txt`). The existing
uncommitted pacing change measures both QR and restricted-zone detector
rates and uses the slower detector when setting sweep speed. Its focused
regression passes; the headless detector publishes at an assumed cadence
and cannot qualify the real rendered-processing latency.

Next run, in a non-login WSL shell with system Python and ROS sourced:

```sh
source /opt/ros/jazzy/setup.bash
PATH=/usr/bin:/bin:$PATH bash scripts/run_custom_world.sh \
  sim/worlds/my_world.json --conditions calm --seed 20261003
```

Before launching, inspect existing simulator process groups and reserve the
live stack's ports/ROS domain. The live runner tears down matching older
stacks and must not interrupt a different active validation job.

Acceptance: COMPLETED plus every independent grading check, including no
restricted-zone entries, with recorded QR/red detector rates. Save the
rendered track, layout and stack logs separately from headless evidence.

## Comments

2026-10-03: the requested recorded rerun used the last failed off-axis
world, seed 1, calm, target B. Its complete rendered trace still enters
`restricted_red_zone_3` at simulation t=252.33 s during SEARCH_QR;
closest distance to restricted ground was 0.07 m. Existing detector pacing
is not qualified by this run. Evidence:
`logs/custom/my_world_offaxis_calm_s1_full_grade.txt` and `*_stack.log`.
The standard `my_world.json` acceptance run remains pending.
