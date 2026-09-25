# Gazebo and GCS debug, 2026-09-11 (second session)

Two defects were found by reading the launch path, fixed, and covered by
regressions that reproduce the defect before the fix. **No flight was run.**
The environment this session had access to is an isolated Linux workspace with
the repo mounted; it has no ROS, no Gazebo and no GPU, so nothing here is live
evidence. The run itself is still owed.

## 1. The launcher's vehicle had no sensors attached

`mission2.sdf` includes `model://aerothon_iris_c1_webcam`, which
`materialize_vehicle_model.py` generates from the installed upstream
`iris_with_gimbal`. The generator emitted `<parent>base_link</parent>` for the
lidar and webcam mounts unconditionally.

Upstream ships that model in two shapes. One merges the airframe links, so
`base_link` is a direct child. The one installed here `<include>`s
iris_with_standoffs, so the link is `iris_with_standoffs::base_link` -- and a
joint naming a link that does not exist is dropped by Gazebo. The world then
comes up with no `/scan` and no `/camera/image`, and nothing in the log names
the cause.

This was already known, but only half-fixed: `.scratch/open_gazebo.sh` patches
the generated SDF by hand after the fact. That script materialises the VIEWING
model. The model `launch_level6_sim.sh` actually flies went through the same
generator and was never patched. The two paths had silently diverged, which is
why the arena could look correct in a viewing session while a launched run had
no perception.

The same nesting scopes the gimbal joints as `gimbal::roll_joint`. Removing
plugins by the bare names `roll_joint`/`pitch_joint`/`yaw_joint` therefore
matched nothing, and PID controllers were left driving joints that had just
been removed.

The generator now detects which shape it was handed and scopes the mount
parents accordingly, and drops any plugin still driving a joint the variant no
longer has. The hand patch in `open_gazebo.sh` becomes a no-op rather than a
conflict.

Regression: `sim/test_vehicle_model_materialisation.py`, 5 tests. Against
`git show HEAD:scripts/materialize_vehicle_model.py` two of them fail, naming
the dangling `base_link` and the surviving `gimbal::roll_joint` controller.

## 2. Nothing supplied the delivery-zone boundary, so nothing could arm

`gcs_readiness` and `mav_commander` both require a four-corner delivery-zone
boundary on `/mission/delivery_zone` before `/mission_ready` can go true. The
aggregator accepts a `set_delivery_zone` command, but no frontend control sends
one and no launch file published one, so every simulated run stopped at
not-ready reporting "delivery-zone boundary is missing". The complaint was
correct; there was simply no organiser in the simulation.

`scripts/publish_delivery_zone.py` plays that part. It waits for the FCU home
position, converts the arena's delivery field from local ENU to WGS84 about it
using `mission_bt.geofence.local_to_global` -- the mission's own conversion,
not a second copy of the formula -- and publishes the corners latched.
`launch_level6_sim.sh` starts it; `AEROTHON_SUPPLY_DELIVERY_ZONE=0` turns it
off to drive the boundary from the GCS instead.

Defaults to the shipped arena's `delivery_zone_40x30` at centre (32, 0),
40 x 30 m. For a randomised arena pass that seed's centre through
`AEROTHON_DELIVERY_ZONE="cx,cy,40,30"`.

It refuses a home of (0, 0). That is MAVROS's uninitialised default rather than
a position, and a boundary georeferenced about it is a perfectly valid
rectangle -- the mission would accept it and search the Gulf of Guinea. The
refusal has to happen at the source.

Regression: `sim/test_delivery_zone_supply.py`, 6 tests. The load-bearing one
is not that it publishes JSON but that `parse_boundary` +
`boundary_to_local_zone` turn that JSON back into x 12..52, y -15..15 -- the
validator rejects corners more than 0.75 m off an axis-aligned rectangle, so a
slightly different earth model would be refused.

## 3. Preflight now checks the Python imports that actually failed

`preflight_stack.sh` checked ROS packages and no Python imports, so the missing
`websockets`, missing `py_trees` and the NumPy 2 / cv_bridge ABI break surfaced
late and in the wrong place: the aggregator died after Gazebo was already up,
and cv_bridge's binary-incompatibility message reads like a perception bug.
Three import checks now fail preflight by name.

Fixing them is environment work, not a code change:

    pip install --user websockets py_trees
    python3 -c "import numpy, cv_bridge"   # if this aborts, a user-site
                                           # NumPy 2 is shadowing the one
                                           # cv_bridge was built against

## Not done

- No preflight, no build, no launch, no flight. Every claim above is offline.
- `scripts/run_tests.sh` was not run; this session had no pytest and no network
  to install it. The two new files were run under `python3 -m unittest`.
- The frontend still has no delivery-zone control. The launcher now covers the
  simulated case, but an operator cannot supply a boundary by hand.
- Seed 1001 through delivery, return and landing remains the open acceptance
  gate, untouched by this session.
