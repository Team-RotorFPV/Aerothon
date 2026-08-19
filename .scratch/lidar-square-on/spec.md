# Square up to the banner with the lidar

Status: ready-for-agent

## Problem statement

The drone finds the banner and then fails to get in front of it.

Watched live across fourteen runs on seed 1001, the failure takes three
recognisable forms:

1. It orbits the wrong way, loses the banner off the edge of frame, and times
   out.
2. It slides sideways past the point of best view, thrashes direction, and
   gives up with "never came in front of the banner after 14 sideways steps".
3. It decides it is square when it is not, latches a waypoint 10 m ahead, and
   pitches into or past the banner. Twice this flew the aircraft out of the
   world.

Every one of these traces back to the same root cause: **squareness is being
inferred from how the banner looks to the camera** — the aspect ratio of a
bounding box derived from green pixels. That box includes the gate posts, so
it never gets slender no matter how square the aircraft is. Its measured
plateau is 1.88 to 1.91. A threshold of 2.00 was unreachable. Lowering the
threshold to 1.75 and adding peak detection only moved the failure: a single
noisy narrowing at aspect 1.4 now satisfies the gate and the aircraft advances
while badly off-axis.

The camera cannot answer "am I perpendicular to that surface". It is the wrong
instrument for the question.

## Solution

Use the lidar.

The aircraft carries an RPLidar C1: 720 samples over a full 360 degrees, 12 m
range, 10 Hz, 5 mm noise, fixed to the top of the airframe. Perpendicularity
to a flat surface is something a lidar measures directly. Fit a line through
the returns in the sector the camera says the banner occupies. The angle of
that line relative to the aircraft's nose *is* the misalignment, in degrees,
with no proxy and no tuning constant standing between the measurement and the
answer.

The same fit gives the standoff distance for free, so holding station off the
board becomes a single number to servo on rather than a guess.

The camera keeps the job it is good at: finding the banner, confirming it is
*the* banner by reading AEROTHON, and pointing at a bearing. The lidar takes
over the job the camera was bad at.

**When they disagree, the lidar wins.** This is the user's explicit decision.
If the camera says "banner at 20 degrees to port" and the lidar finds no flat
face in that sector, the aircraft does not advance. There is no fallback to
the aspect test. The aspect gate is deleted, not demoted, because a fallback
that fires on bad data is precisely how the aircraft flew out of the world.

## User stories

1. As a mission operator, I want the aircraft to measure its angle to the
   banner rather than infer it, so that "in front of the banner" means the
   same thing on every run.
2. As a mission operator, I want the aircraft to refuse to advance when it
   cannot measure squareness, so that it never commits a waypoint on a guess.
3. As a mission operator, I want the aircraft to hold its distance off the
   banner while orbiting, so that it does not drift out of the lidar's useful
   range.
4. As a mission operator, I want the orbit to be a genuine arc around the
   banner rather than a straight slide past it, so that the viewing angle
   actually changes with every step.
5. As a mission operator, I want the orbit to travel toward the side the
   banner is on, so that the banner stays in frame while the aircraft comes
   round to its face.
6. As a mission operator, I want a single reported number for how far off
   perpendicular the aircraft is, so that a failed run can be diagnosed from
   the log without re-flying it.
7. As a mission operator, I want the aircraft to stop orbiting the moment it
   is perpendicular, so that it does not overshoot a good position.
8. As a mission operator, I want the 10 m advance waypoint to be set only
   after perpendicularity is confirmed, so that the aircraft never pitches at
   a banner it has not squared up to.
9. As a mission operator, I want the advance to run through the existing
   avoidance routing, so that obstacles between the gate and the payload zone
   are still avoided.
10. As a mission operator, I want the aircraft to keep its heading during
    lateral steps, so that yaw and translation do not fight each other.
11. As a mission operator, I want the lidar sector searched to be derived from
    the camera bearing, so that the corridor wall behind the open gate is not
    mistaken for the banner.
12. As a mission operator, I want a flat surface found at a wildly different
    range from the camera's target to be rejected, so that a wall two metres
    behind the gate cannot masquerade as the banner.
13. As a mission operator, I want the squareness measurement to carry a
    quality figure, so that a fit through four noisy points is not trusted
    like a fit through forty.
14. As a mission operator, I want the aircraft to give up after a bounded
    number of orbit steps, so that a hopeless geometry ends the stage instead
    of hovering until the battery runs down.
15. As a mission operator, I want the give-up message to state the measured
    angle and range, so that the reason is legible without instrumentation.
16. As a developer, I want the lidar geometry to be a pure function of a scan,
    so that I can test squareness logic without a simulator.
17. As a developer, I want the mission tree to see the lidar through the
    existing commander seam, so that the number of places tests have to reach
    into stays at one.
18. As a developer, I want the fake vehicle used in tests to model actuation
    lag, so that per-tick correction bugs fail in the test suite rather than
    on a watched flight.
19. As a developer, I want the banner board to have collision geometry in the
    simulated world, so that the lidar sees the surface a real banner would
    present.
20. As a developer, I want the squareness algorithm to work identically on a
    solid board and on a bare pair of posts, so that it is not tuned to a
    simulator artifact.
21. As a developer, I want the aspect-ratio gate removed rather than kept as a
    fallback, so that no code path can advance on the measurement that has
    failed every run.
22. As a GCS operator, I want the measured angle and standoff shown while the
    aircraft is aligning, so that I can see it converging rather than guess.
23. As a mission operator, I want the aircraft to hold station and re-measure
    after each orbit step rather than measuring while still moving, so that
    the reading corresponds to where the aircraft actually is.
24. As a mission operator, I want the align stage to work at whatever altitude
    the corridor phase left the aircraft at, so that it does not depend on the
    banner being at one particular height.
25. As a mission operator, I want the run to reach the payload zone on seed
    1001 with the GUI up, so that I can watch the whole sequence end to end.
26. As a developer, I want each fix verified on a live flight and not only in
    the suite, so that a green suite is treated as permission to fly rather
    than as evidence the fix works.

## Implementation decisions

### Seams

One new seam, consumed through one existing seam. This is deliberate; the
codebase has been bitten repeatedly by tests that graded a copy of the logic
rather than the logic.

**New seam — a pure scan-geometry function.** Takes the raw scan (angle
minimum, angle increment, range array), plus a bearing and half-width naming
the sector to search. Returns the fitted surface: the signed angle between the
surface normal and the aircraft nose, the perpendicular distance to it, the
number of returns the fit used, and a residual. Returns a refusal, with a
reason string, when no surface is found. Pure arithmetic, no ROS types, no
clock. Modelled on `route_leg` in the search planner, which is the prior art
in this repo for a pure geometry function with a structured result.

**Existing seam — the mission commander.** The commander gains a subscription
to the scan topic and exposes the fitted surface to the behaviour tree, the
same way it already exposes the banner bearing and detail. The behaviour tree
never touches a ROS message. Tests drive the tree through the fake commander,
as they do today.

### Geometry

Fit a line, not a two-post special case. A line fit handles a solid board and
a pair of posts identically, so the algorithm survives whether or not the
board has collision geometry, and it is the algorithm that would work on the
real gate. Two isolated post clusters still define a line.

Reject the fit when the residual is large, when too few returns contributed,
or when the fitted range disagrees with the camera's estimate. Each rejection
carries its own reason string; "no surface found" alone is not diagnosable.

### Orbit

The orbit becomes an arc. Each step moves tangentially *and* corrects
radially, so the standoff stays inside a band. The tangential direction is
seeded from the side the camera saw the banner on, which is already tracked.
The radial correction comes from the lidar range, which is new. Range is no
longer inferred from apparent size.

Direction reversal on a narrowing measurement is removed. It existed to
compensate for a proxy that could not tell which way was better. The lidar
angle is signed, so the aircraft knows which way to go from the first reading.

### Gate

Squareness is a single condition: the measured angle is inside a tolerance and
the fit is trusted. There is no peak detection, no best-of-run fallback and no
aspect threshold. When the aircraft cannot measure, it does not advance.

The 10 m advance keeps its current structure: latch the target once on
confirmation, route it through avoidance, record the corridor exit on arrival.
Only the condition that releases it changes.

### World model

Add collision geometry to the banner board in the world materialiser. It
currently emits visuals only, so the board is invisible to the lidar. A real
banner is a solid surface. Adding the collision makes the simulator model the
thing being flown against rather than a hollow frame.

### Glossary

`CONTEXT.md` currently defines **Align** as "the aircraft has yawed to face an
identified banner". That is weaker than what the mission requires and does not
describe what the code does. Add **Square on** as a distinct term: the
aircraft is perpendicular to the banner's face, within tolerance, at a
measured standoff, as determined by the lidar. Align remains the yaw-only
state that precedes it.

## Testing decisions

A good test here asserts on behaviour visible outside the module: given a
scan, what surface is reported; given a sequence of commander states, where
does the tree command the aircraft and what does it refuse. Tests that assert
on internal state names, on which private method ran, or on the tuning
constants themselves are not wanted; they have to be rewritten every time a
constant moves and they caught none of the fourteen live failures.

**The scan-geometry function** is tested directly with synthetic scans: a wall
dead ahead, a wall at 30 degrees, a pair of posts, a post pair with one post
occluded, a sector containing nothing, a sector containing two surfaces at
different ranges, and scans carrying noise at the sensor's stated 5 mm. The
signed angle must come out with the correct sign; sign errors have cost this
project several flights and are worth their own explicit cases.

**The align stage** is tested through the fake commander, which is the
existing pattern in the banner sweep tests. Cases must include: the lidar
refusing while the camera is confident, which must not advance; the lidar
confirming while the camera has momentarily lost the banner; the orbit
converging from either side; and the standoff being held while the aircraft
circles.

**The fake commander must model lag.** This is the single most important
testing change in the spec. Every fake vehicle in this repo follows setpoints
instantly, and that is why an oscillation bug survived 789 tests and only
appeared on a watched flight. Give the fake a first-order lag on position and
heading, then re-run the existing align tests. Any test that only passes
against an instant-following fake was never testing what it claimed.

Mutation-test the squareness gate specifically: remove the refusal path and
confirm the suite goes red. The refusal path is the one that keeps the
aircraft inside the world.

## Out of scope

- Everything downstream of the gate. The corridor, red-zone routing in flight,
  the search sweep, the winch drop, delivery measurement, the return lap and
  landing are all still unflown. They are not this spec's problem, but note
  that this spec is what unblocks reaching them.
- Hardware. Simulator only, per the standing constraint.
- The banner detector itself. Detection reached 12 of 12 frames on seed 1001
  and is not the failing part. Do not retune it.
- The zigzag sweep. It works, finding the banner in two to four headings
  instead of eleven.

## Further notes

### Measurements a fresh reader cannot rederive cheaply

These cost flights to establish. Changing them without a measurement is how
this stage regressed twice.

- Banner detector `min_aspect` is **0.9**, and must stay there. The banner
  measures 1.10 to 1.22, so the old 1.2 threshold sat inside the real range
  and rejected the thing it was meant to accept. 0.9 looks sloppy and is
  correct. This single change took dwell quality from 8 of 12 frames to 12 of
  12.
- The banner is a **gate**: green posts, a lettered panel with **grey letters
  on green**, standing open in front of a grey corridor wall. The letters are
  *darker* than the board, V=132 against V=184. Any assumption that lettering
  is the bright part fails.
- Letters are recovered as the **holes in the closed board**, not by
  thresholding. The close kernel must be 0.20 of ROI height; 0.10 reaches only
  84% coverage.
- tesseract needs **psm 8**, single word. psm 7 reads `HERUT RUM`.
- Call the `tesseract` binary through subprocess. The Python environment is
  PEP 668 externally-managed and pytesseract is not installable.
- OCR runs **once per frame on a timer**, not per candidate per path. Per
  candidate it cost 517 ms per frame; once per frame it costs 12 to 21 ms.
- The derived board box **includes the gate posts**, which is why its aspect
  plateaus at 1.88 to 1.91 and why every aspect threshold above that was
  unreachable. This is the fact that motivates the whole spec.
- `pkill -f` matches the wrapper shell. Bracket every pattern.
- `flock` file descriptors leak into children. Close with `9>&-`.

### Recurring bug classes in this codebase

Named because they have each recurred more than once:

1. **Asserting instead of measuring.** Code that states a geometric fact it
   could have measured.
2. **Testing the calculation, not the wiring.** A correct function nobody
   calls.
3. **No fake vehicle has lag.** Per-tick proportional corrections converge
   against an instant-following fake and oscillate against a real airframe.
   The team calls this the receding carrot. It has appeared in the banner
   approach and again in centring.

### How to work

Fly it. A passing suite is permission to launch a run, not evidence the fix
works. Twice this session a change passed all 789 tests and then failed live.
Run the arena regression on seed 1001 with the GUI up and watch what the
aircraft actually does.

Keep the evidence pack per phase, per the standing constraint.
