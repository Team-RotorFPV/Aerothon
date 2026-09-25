# Move to recover the banner; yaw alone cannot

Status: ready-for-agent
Parent: ../spec.md

## What was watched

Run 16, seed 1001, GCS open, live.

The aircraft took off to 5.0 m, entered `BANNER_ALIGN`, identified the banner
cleanly at t+41 s, and then walked its altitude down: 5.0 at t+32, 3.7 at
t+40, 3.0 by t+48, where it stayed. From t+54 onward the detector refused
almost every frame, and the refusals name the reason:

    green region too small (5262 px, need 8533)
    green region too small (3416 px, need 8533)
    green region too small (3192 px, need 8533)
    green region too small (8139 px, need 8533)
    green region too small (1930 px, need 8533)

The board is shrinking in frame. Interleaved with those are four refusals at
board aspect 0.86 to 0.90 against the 0.9 floor.

The aircraft then sat at `[0.3, 0.0, 3.0]` from t+48 to t+80 without moving.

## The defect

`AlignToBanner` is constructed with `alt_floor_m=corridor_alt`, which is 3.0.
The aircraft descended to that floor during the align and lost sight of the
banner from the new position.

Once it had lost it, the only recovery available was the yaw sweep. **A yaw
sweep can only correct a bearing error.** This is a position error: from
`[0.3, 0.0, 3.0]` the banner is not visible at any heading, so rotating
through every heading in turn cannot find it. The aircraft is searching a
one-dimensional space for something that left it.

Fail-closed then did its job and refused to advance, which is correct
behaviour and the reason the aircraft did not fly out of the world this time.
But refusing forever is not the outcome wanted.

## What to build

A recovery search that moves. The existing yaw zigzag becomes the *inner*
loop, run unchanged at each vantage point. Around it goes an *outer* loop that
relocates the aircraft and runs the inner sweep again.

**Return to the last vantage point that worked, first.** The aircraft saw the
banner at t+41 and knows the pose it was holding at the time. A position that
demonstrably worked is a better guess than any pattern, and it is one setpoint
away. Record the pose on every confident identification and make "go back
there" the first recovery move. Only start a pattern if that fails.

**Then a pattern in range and altitude.** The refusal reasons
say which directions help. `green region too small` means the board subtends
too few pixels, so *closing range* helps and opening it does not. The lost
sighting followed a descent, so *climbing* helps. Fore-and-aft along the
bearing the banner was last seen on, combined with altitude steps, covers both.
The user asked for "back and forth"; the evidence says pair it with height.

**Bound it and say what it did.** A fixed number of vantage points, and a
give-up message naming how many were tried, where they were, and what the best
frame at each was. The current give-up message names only headings, which is
why this failure needed a live watch to diagnose.

**WITHDRAWN: "do not descend during align at all".** This said the descent
bought nothing and cost the sighting. Both halves are wrong, and the
measurements are in `evidence/lidar-square-on/static/FINDINGS.md`.

The descent buys the entire measurement. Parked on the shipped arena, the
lidar returns 0 finite ranges of 720 at 3.75 m and above, because the board
spans z 2.805 to 3.955 and the scan plane leaves it once the aircraft passes
about 3.72 m. Without descending there is nothing to fit a line through.

And the descent did not cost the sighting. Altitude against standoff, both
instruments, nine cells from 2.5 to 3.5 m and 3.5 to 6.9 m:

|  | standoff 3.5 m | standoff 5.0 m | standoff 6.9 m |
|---|---|---|---|
| 2.5 m | 51/51 ident | 55/55 ident | 57/57 ident |
| 3.0 m | 60/60 ident | 60/60 ident | 60/60 ident |
| 3.5 m | 34/34 ident | 58/58 ident | 59/59 ident |

100% identification in every cell, with the fit inside half a degree. The
usable band is 2.5 m to 3.5 m and the mission's own corridor altitude of 3.0 m
sits in the middle of it.

**The actual fault was the camera pointing.** The aircraft descended with the
camera still pitched 20 degrees down for a search from 5 m, which puts a board
at gate height out of the TOP of the frame. The readings above were taken with
the camera FORWARD. A wide band and a bad pointing angle, not a narrow band.

`alt_floor_m` was not being used as a target: the ladder tests
`az - step >= floor` and stops. It ran to the bottom because every rung
refused, and it kept refusing because the camera could not say where to look.
What was genuinely wrong is that the ladder kept descending AFTER a surface
had been found -- the aircraft measured a real face at 3.5 m (36 returns,
2 cm residual), turned toward it, measured nothing on the next tick because
the TURN had moved the sector, and read that as another reason to descend.

**A separate real constraint, also measured.** Past about 3 m off the gate's
centreline the detector stops identifying at all -- 15% of frames at 3 m off,
0% at 4 m off, refused for `green region too small` and `white components are
not in a horizontal band`. Any search that translates has to stay inside that,
so lateral steps are capped rather than free.

## Testing

Through the fake commander, as with the rest of the align stage.

The case that matters: a fake positioned where **no heading** reveals the
banner. Assert that the aircraft translates. A yaw-only implementation fails
this test, which is the point of writing it.

Then: the banner visible only from the remembered pose, and the aircraft
returning to it; the banner visible only after a climb; the banner visible
only after closing range; and nothing visible anywhere, which must give up
inside the bound with a message naming the vantage points tried.

The fake must model actuation lag, per the parent spec. A search that
translates is exactly the kind of code that converges against an
instant-following fake and oscillates against an airframe.

## Note on the aspect floor

Frames are being refused at board aspect 0.86, 0.86, 0.89 and 0.90 against the
0.9 floor. **Do not move the floor.** It was set from a measurement and the
parent spec explains why it looks wrong and is right. It is now marginal
rather than comfortable, so expect intermittent detection near the boundary
and design the search to tolerate it rather than tuning it away.
