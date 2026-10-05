# The CAD claw and the payload: contact tests

`sim/winch_bench.py` holds the team's CAD airframe at 5 m and runs the real
`winch_ctrl` (Gazebo backend) through lower, release and stow. The bench
removes the vehicle's `DetachableJoint`: nothing attaches or detaches the
payload. The claw's jaw angle is commanded from the CAD linkage geometry
(Gazebo cannot hold the closed linkage); whether the payload is carried,
released or lifted again is decided by Gazebo contact between the CAD jaw
meshes and the payload. The verdict (`assessment.json`) comes from the
payload's measured pose against the measured line length, never from what
`winch_ctrl` believes.

## What the claw can hold

Each jaw is a 2 mm plate in the x-z plane ending in a hook pocket. Measured
on the CAD (`cad_to_gazebo.py`, `airframe.json` "pockets"):

| | jaw a | jaw b |
|---|---|---|
| pocket centre (x, z) | 3.97, -123.85 mm | -5.30, -123.83 mm |
| pocket diameter | 3.26 mm | 3.25 mm |
| slot to the claw's centre | 2.88 mm | 2.87 mm |
| plate (y) | 1.5 to 3.55 mm | 3.5 to 5.55 mm |

A feature held by the claw has to cross the jaw plates, along y, at both
pockets. **A vertical plate with a round hole (rulebook Fig. 1 style) cannot
be held and released by this claw at any hole size:**

- with the hole in the jaws' plane, the claw's arms and links would have to
  pass through the plate: the earlier 17 mm / 4 mm trial started with jaw
  and link geometry inside the tab's material (z -120.5 to -113.4 mm), and
  its `DROPPED_EARLY` was Gazebo ejecting the overlap, not a slip;
- turned to cross the jaws, the shut fingertips are 7.2 mm apart and stop
  1.6 mm short of a 4 mm tab's hole; passing the claw through the hole
  instead traps the tab inside the linkage, and it can never be released.

That earlier run also had each jaw's collision mesh offset 6.5 mm below and
5.5 mm beside the jaw drawn (the tip box's pose was left on the mesh). Both
problems are fixed and guarded by `sim/test_physical_hook.py`.

What fits the pockets is a **flat ring of round wire**, lying horizontally on
two short legs, its centreline through both pocket centres (9.27 mm): one side
of the ring in each pocket. Wire of 1.6 mm (a 7.67 mm hole) passes the 2.87 mm
slot and leaves a 0.95 mm band of claw heights in which the open claw can be
lowered over the ring and closed round it without striking it; at 2.2 mm that
band is 0.05 mm.

## Results (1.6 mm ring, claw as drawn, 5 m)

**Loading from a stand: NOT HELD** (`logs/winch_ring`, three attempts, the
same each time). The claw, held open 35 degrees, comes down 25 mm over the
ring and closes round it; the line then takes in 5 mm. The payload rises
4.7 mm with the line, but hangs tilted about 0.8 degrees and 1.2 mm off
centre in y: one pocket is carrying it. About 0.2 s after the lift stops
the tilt jumps to 2-2.6 degrees, the ring slides out of the 2 mm pockets
and the payload drops back onto the stand.

**Payload started seated in both pockets: carried, released, then lifted
again** (`logs/winch_ring_hung`). Settled evenly in both pockets, it
follows the line down all 5 m by contact alone (never more than 0.7 mm
below the claw's hold). On touchdown the slack opens the jaws 17 degrees
and it is free. As the winch stows, taking up the slack shuts the jaws on
the ring again: in the full run the payload was lifted 0.56 m and then
fell back onto the pad. A second, shorter run was not lifted in its first
8 s of stow. The re-grab is not repeatable in either direction.

## What the mechanism needs

1. **Pockets that hold the wire along its length.** Each pocket is a 2 mm
   plate edge; a ring picked up unevenly walks out along y. The hooks need
   side walls or a wider seat (for example each jaw as a clevis of two
   plates, so both pockets span the full claw width), or a notch in the
   ring's wire that the pocket locates in.
2. **A latch that keeps the jaws open once they open**, reset when the claw
   is stowed. As drawn, taking up the slack shuts the tongs on the ring
   again whenever the claw rises straight up.
3. **Ground clearance.** On its gear the airframe holds the claw's tips
   46 mm above the ground; a payload hanging from them touches the ground
   first, the line goes slack and this claw opens before take-off.
4. **A payload eye matched to the claw**: a flat ring of about 1.6 mm wire
   with a 7.7 mm hole on two short legs, not a holed plate.

This is a simulation of the CAD geometry. It does not replace a bench test
of the real claw and payload.

## Running it

From WSL with ROS 2 Jazzy, the ArduPilot overlay and the workspace sourced:

```sh
python3 sim/winch_bench.py --claw as_drawn --out logs/winch_ring             # load from the stand
python3 sim/winch_bench.py --claw as_drawn --no-loading --out logs/winch_ring_hung
```

The bench refuses to start if the ring overlaps the jaw meshes or the open
claw cannot close round it. Outputs: `loading_closeup_2x_slow.mp4` (front and
oblique while loading), `contact.mp4`, `oblique.mp4` (cameras on the payload),
`claw.mp4`, `wide.mp4`, `mechanism.mp4`, `nadir.mp4`, `drop.mp4`,
`events.txt`, `timeline.csv` and `assessment.json`.
