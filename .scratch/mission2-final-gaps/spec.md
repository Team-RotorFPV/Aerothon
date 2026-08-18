# Spec: Mission 2 final scored gaps

Status: ready-for-agent

> **Implemented 2026-08-18.** Offline evidence in `VERIFICATION.md` §13.
> Not yet demonstrated in a live arena run.

## Problem Statement

I am flying the Mission 2 stack in the simulator with Mission Planner on one
screen and our own GCS on the other, and there are things I can see going wrong
that the mission itself reports as fine.

The aircraft flies over red ground. It should not even enter the airspace above
a restricted zone, and each violation is minus five of ten marks. When I watch
the telemetry the red-zone detector is clearly working — it confirms red ground
and builds exclusions — and the aircraft still crosses them.

The banner sweep oscillates. It yaws right, detects the banner, and immediately
yaws left again, so it never commits. And there are frames where I can plainly
see a banner in the camera pane and the stack rejects it, because the lettering
check needs the letters to be brighter than the board and in shadow they are
not.

When a QR is scanned the mission moves on instantly. I cannot tell afterwards
what was scanned, what decoded, what matched, or why anything was rejected —
that history exists only in a log I am not reading while flying. I want the
aircraft to hold over a marker long enough for me to see it happen, and I want
a panel that keeps the list.

Finally, a run can complete and still report `delivery UNMEASURED`. Payload
delivery accuracy is fifteen marks. A completed mission that cannot say how far
from the pad it dropped has not evidenced them.

## Solution

Six changes, each closing a gap that is either scored by the rulebook or
visible to the operator during a watched flight.

**Exclusions constrain every leg the airframe flies, not just the sweep.**
Today the search sweep clips its lanes against confirmed red ground and every
other leg — corridor exit to the observed zone, approach to the matched pad,
descent to decode, return to the corridor mouth, go-home — flies straight
through. Routing becomes a single exclusion-aware primitive that every leg
uses, and a leg with no clear route fails closed with that reason rather than
flying the straight line anyway.

**The banner sweep stops and stares.** Discrete yaw steps with a settle and a
five-second dwell at each, verdicts accumulated across the dwell rather than
acted on frame by frame, and a full turn as the bound before failing. The first
dwell that is *confident* wins. A failure names what each step saw and why it
was rejected.

> **Changed during implementation.** This originally said the best candidate
> across the whole turn should win. Built as first-confident-wins instead, for
> two reasons. It matches the instruction as given — "if it does not detect in
> those 5 secs it should yaw more left ... and continue until 360" makes the
> full turn the *limit*, not a mandate. And always completing twelve dwells
> costs about 90 seconds of a 15-minute window, twice per mission, to re-prove
> something already established. The decoy hazard the best-candidate rule was
> meant to cover is covered by the confidence floor instead, which is where it
> actually belonged: the 271° lock came from acting on weak evidence, not from
> looking too far.

**Lettering is read by two independent paths.** The existing brightness path
keeps working where it works; a stroke/edge path runs beside it for shadowed
and oblique views. Either path confirming is enough, and the detail topic says
which one did it.

**The GCS keeps a scan ledger.** An append-only list of every scan, decode,
identification and rejection, with matched entries tagged and rejected entries
carrying their reason, shown in a right-side panel.

**The aircraft hovers over each distinct decoded payload for five seconds**, so
a decode is something an operator watches happen.

**Delivery accuracy is measured or the reason it was not is specific.** The
release geometry keeps the pad in frame, and a lost lock at the instant of
release falls back to the last valid offset with its age rather than reporting
nothing.

## User Stories

### Restricted zones (10 marks, −5 per violation)

1. As a competition pilot, I want the aircraft to route around confirmed red
   ground on every leg it flies, so that I do not lose marks on a transit the
   planner never checked.
2. As a competition pilot, I want a leg with no clear route to abort with that
   reason, so that the stack never silently degrades to flying straight through
   an exclusion.
3. As a competition pilot, I want exclusion clearance applied to the airframe
   and not to the camera footprint, so that overflying red ground with the lens
   does not needlessly shrink my search coverage.
4. As a competition pilot, I want a red zone confirmed mid-leg to divert the leg
   currently being flown, so that a zone discovered late is still avoided.
5. As a competition pilot, I want the re-plan budget per strip to remain
   bounded, so that a steadily growing exclusion set cannot stall the sweep
   forever.
6. As a competition pilot, I want any waypoint still intersecting an exclusion
   after the re-plan budget is spent to be skipped rather than flown, so that
   exhausting the budget is not a licence to violate.
7. As an operator, I want the GCS to show the confirmed exclusion count and
   area while I fly, so that I can see avoidance is live rather than assumed.
8. As an operator, I want `NOT_VISIBLE` to stay visually distinct from `CLEAR`,
   so that "no ground view" never reads to me as "no hazard".
9. As a reviewer, I want in-flight evidence that the aircraft's own track
   stayed clear of every confirmed exclusion, so that the claim rests on a
   recording and not on a unit test.

### Banner sweep and identification (10 marks, shared with alignment)

10. As a competition pilot, I want the sweep to yaw in discrete steps and stop
    at each, so that the aircraft is stationary while the detector decides.
11. As a competition pilot, I want a five-second dwell at each step, so that a
    marginal banner has many frames to be confirmed from rather than one.
12. As a competition pilot, I want the verdict at a step to be taken over the
    whole dwell, so that a single flickering frame cannot end the sweep.
13. As a competition pilot, I want an identified banner to stay latched through
    a dropped frame, so that the aircraft stops yawing away from a banner it
    has already found.
14. As a competition pilot, I want the sweep to cover a full turn, so that a
    banner behind the aircraft's start heading is still found.
15. As a competition pilot, I want a heading to win only on a confident dwell,
    so that covering the full turn cannot make the aircraft align to the return
    gate or to a decoy.
16. As a competition pilot, I want a failed sweep to report what each step saw
    and why it was rejected, so that a failure tells me where to look.
17. As an operator, I want each dwell to be visible in the GCS as it happens,
    so that a stopped aircraft reads as deliberate rather than as a hang.

### Lettering in shadow

18. As a competition pilot, I want a banner in shadow to be identified, so that
    lighting does not decide whether I score the identification task.
19. As a competition pilot, I want a banner viewed obliquely to be identified,
    so that approach angle does not decide it either.
20. As a competition pilot, I want a stroke-based reading path to run beside
    the brightness path, so that either can confirm the lettering.
21. As a competition pilot, I want both paths graded by the same glyph
    classifier, so that adding a path cannot smuggle in a second, weaker
    definition of "reads as AEROTHON".
22. As a competition pilot, I want the glyph vocabulary to stay independent of
    the arena generator's font, so that a passing test is not the generator
    grading itself.
23. As an operator, I want the detail topic to say which path produced the
    read, so that I can tell a clean read from a rescued one.
24. As a competition pilot, I want a plain green board with no lettering to
    still be rejected, so that widening the reader does not widen it into
    accepting a tarpaulin.

### Scan ledger and hover

25. As an operator, I want an append-only list of everything scanned, so that I
    can review the run without reading a log.
26. As an operator, I want every decoded payload in that list, so that I can
    see what the aircraft actually read.
27. As an operator, I want entries that matched the delivery target tagged as
    matched, so that I can distinguish decoding from matching at a glance.
28. As an operator, I want rejected entries kept with their reason, so that a
    marker the stack refused is visible rather than absent.
29. As an operator, I want each entry timestamped and stamped with the mission
    stage it happened in, so that I can line the list up against the flight.
30. As an operator, I want the list on a right-side panel alongside the video,
    so that I can watch the feed and the history together.
31. As an operator, I want the panel to survive a websocket reconnect, so that
    a dropped connection does not erase the run's history.
32. As a competition pilot, I want the aircraft to hover five seconds over each
    distinct decoded payload, so that a decode is something I can watch happen.
33. As a competition pilot, I want the hover to fire once per distinct payload
    rather than per frame, so that re-seeing the same marker does not stall the
    mission.
34. As a competition pilot, I want the hover to be bounded and to hold
    altitude, so that a dwell cannot become a drift or a descent.

### Delivery accuracy (15 marks)

35. As a competition pilot, I want the release geometry to keep the pad inside
    the frame, so that the offset can be measured at the moment it matters.
36. As a competition pilot, I want release altitude checked against the
    altitude below which the pad cannot fit in frame, so that the measurement
    is not made geometrically impossible by the descent.
37. As a competition pilot, I want a lock lost at the instant of release to
    fall back to the last valid offset with its age, so that a good measurement
    a moment earlier is not thrown away.
38. As a competition pilot, I want the offset to remain perception-derived, so
    that the number is not read out of the simulator.
39. As a competition pilot, I want `UNMEASURED` to remain possible and honest,
    so that the stack never invents a zero it did not measure.
40. As an operator, I want the measured offset in the GCS mission panel, so
    that I can read the scored quantity during the flight.

### Regression and evidence

41. As a competition pilot, I want the main seed to complete end to end, so
    that the arena I demonstrate on is one I have seen work.
42. As a competition pilot, I want the return-side banner identified as
    reliably as the outbound one, so that the return leg is not a separate,
    weaker path.
43. As a reviewer, I want the randomised-arena regression to still pass on the
    arenas that passed before, so that closing these gaps did not open others.
44. As a reviewer, I want the in-flight recorder to be a first-class tool in
    the repo, so that the next diagnosis does not depend on a script in a temp
    directory.
45. As a reviewer, I want the recorder gated on mission stage, so that a probe
    can never sample a landed aircraft and report on a sweep.
46. As a reviewer, I want each fix mutation-checked, so that a passing suite is
    evidence the test would have caught the bug.

## Implementation Decisions

### Exclusion-aware routing is one primitive in the search planner

The search planner module already owns every exclusion computation — lane
clipping, exclusion-aware lawnmower planning, coverage under exclusions, and
the plan/exclusion intersection test. It is pure geometry with no ROS
dependency, and it is where this belongs.

Add **one** function that routes a single point-to-point leg around the
exclusion set given an airframe clearance, returning either a waypoint list
that detours around the union or an explicit "no route" result. Every mission
stage that currently commands a straight leg calls it: the transit to the
observed zone, the approach to the matched pad, the reposition for descent to
decode, the return to the corridor mouth, and the go-home leg.

The clearance is an **airframe** clearance. This is the distinction the domain
glossary records as load-bearing: a red zone constrains the aircraft, not the
camera, and clipping against the camera swath previously over-clipped the plan
and dropped coverage to 0.81.

"No route" is a fail-closed abort carrying the leg and the blocking exclusion
count, not a fallback to the straight line.

### The mid-strip re-plan bound stays, but exhausting it is not permission

The sweep re-plans when a newly confirmed exclusion intersects its remaining
waypoints, bounded per strip because the confirmed-cell count climbs steadily
while sweeping and an unbounded re-plan would restart the strip forever. The
bound is correct and stays.

What changes is what happens after it is spent: remaining waypoints are still
tested against the current exclusions, and any that intersect are **skipped**
rather than flown. Running out of re-plan budget degrades coverage; it must not
degrade compliance.

### The sweep becomes an explicit dwell state machine

Banner alignment is currently a continuous yaw that reacts frame by frame,
which is what produces the oscillation: the detector confirms, the stage
reacts, the aircraft moves, the detector drops, and it reverses. Replace it
with a state machine over discrete steps.

The state shape, from prototyping the step sequence against a fake vehicle:

```
STEP    -> command the next discrete yaw target
SETTLE  -> wait until heading error is inside tolerance and yaw rate is ~0
DWELL   -> hold heading for the dwell period, accumulating detector verdicts
DECIDE  -> score this step from the accumulated verdicts; record the reason
         -> more steps remain: STEP
         -> turn complete: pick the best-scoring step over the whole turn
CENTRE  -> yaw to the winning step's heading and fine-align
```

The step size and dwell come from parameters, defaulting to a step that covers
a full turn in twelve steps and a five-second dwell. Verdicts accumulate over
the dwell so no single frame decides a step, and an identification latches
across dropped frames.

**Covering a full turn requires a confidence floor on each dwell.** The
existing half-turn limit exists for a real reason — an unbounded sweep once
turned 271° and locked onto something to the south — so extending coverage
without changing anything else would reintroduce that failure.

The floor is what resolves it. That sweep locked onto a decoy because a
continuous yaw acts on the first frame that says yes, and over a long enough
sweep something greenish always will; the length was never the mechanism.
Requiring most of a five-second dwell to agree removes the mechanism, and once
it is removed the full turn is safe and the first confident dwell can win
outright. The comment recording the half-turn rationale must be updated rather
than deleted, because the hazard it describes still exists — only its cause has
been correctly identified.

Per-step reasons are retained and reported on failure, so a failed sweep says
what each of the twelve steps saw.

### Lettering gets a second path, not a looser threshold

The brightness path thresholds the lettering region relative to the board's own
median value and saturation. That is the right shape — it is scale- and
exposure-relative rather than absolute — but it requires letters markedly
brighter than the board, which shadowed and oblique views do not provide.

Add a **stroke/edge** path that finds lettering by stroke structure rather than
by brightness contrast, and run it beside the existing path. Either confirming
is sufficient. Both feed the **same** glyph classifier, which stays independent
of the arena generator's font: the classifier's glyph set was written
independently precisely because grading a reader with its own generator was the
trap that made four rounds of synthetic fixtures worthless.

The detail topic records which path produced the read, so a rescued
identification is distinguishable from a clean one in the GCS and in the logs.

Widening the reader must not widen acceptance of unlettered boards: a green
board with no lettering stays rejected, and that is a test obligation, not a
hope.

### The scan ledger is a new key in the aggregator snapshot

The aggregator holds one state dictionary broadcast to the GCS over the
websocket. The ledger is a new top-level key in it: an append-only list of
entries, each carrying the kind of observation, the payload or text where one
was recovered, whether it matched the delivery target, the rejection reason
where it was rejected, a timestamp, and the mission stage it occurred in.

Append-only with a bounded length. The aggregator, not the frontend, owns
de-duplication, so the same marker seen for two hundred frames is one entry
with a count rather than two hundred rows.

The frontend renders it as a right-side panel in the existing stage area, which
already switches between map, SLAM and video views by header tab. Matched
entries carry a visible tag; rejected entries show their reason.

The frontend type definitions and the aggregator's snapshot shape are already
held in sync by a contract test, and the ledger is covered by that same
mechanism — a key added to one and not the other must fail.

### The hover is per distinct payload

Five seconds over each **distinct** decoded payload. Firing per decode event
would stall the mission on a marker held in frame; firing once per distinct
payload string gives the operator the pause they asked for on every genuinely
new marker. The hover holds position and altitude using the altitude-hold path
already in the velocity controller, and is bounded so a dwell cannot become a
drift.

### Delivery measurement is protected by geometry, not by hope

The offset is derived from the QR detector's normalised frame-centre offset
converted to metres through altitude and field of view. That stays — reading
the pad's true pose from the simulator would measure the simulator.

Two changes. First, the release altitude is checked against the tracking floor,
the altitude below which the whole pad cannot fit in frame; releasing below it
makes the measurement geometrically impossible, which is the leading candidate
for the observed `UNMEASURED` on an otherwise complete run. Second, a lock lost
at the instant of release falls back to the most recent valid offset, reported
with its age, rather than discarding a good measurement taken moments earlier.

`UNMEASURED` remains a possible outcome with a specific reason attached. The
stack must never report a zero it did not measure.

### The in-flight recorder becomes a repo tool

The diagnoses that survived this work were produced by a mission-state-gated
recorder that only samples while the aircraft is in the stage under
investigation. The unqualified version of the same probe produced a wrong
answer, because sampling a landed aircraft reported no usable ground view and
looked like "the detector never works".

Promote it into the simulation tools directory as a first-class recorder with
the stage gate as a required argument, and use it as the evidence source for
red-zone compliance. Its output — the aircraft's own track against the
confirmed exclusion set — is the artefact that closes the red-zone item.

## Testing Decisions

### What makes a good test here

Three rules, each earned by a bug that got through:

**Measure, do not assert.** The single most repeated failure in this codebase
is a test or comment that states the desired outcome instead of observing it —
a hold-altitude comment above a line publishing zero velocity, a test named for
never sweeping below decode altitude that asserted the bug it was named
against. A test must read the value the system produced.

**Test the wiring, not the calculation.** Three separate defects shipped with
passing tests because the tests exercised a correct helper that nothing called
correctly. Assert on what was **commanded** — the setpoint published, the
waypoint list flown, the snapshot broadcast — not on what a helper returns in
isolation.

**Grade perception on rendered frames.** Hand-drawn fixtures failed four times:
they fragmented into one component per bitmap row and then eroded under the
morphological open, so they tested the fixture generator. Use frames rendered
from the arena, and keep the glyph vocabulary independent of the generator's
font.

Every fix is mutation-checked: revert it, confirm the suite goes red, restore
it. A fix whose test still passes when the fix is removed has not been tested.

### Seams

**Search planner — pure geometry.** The leg-routing primitive is tested here
directly: legs that need no detour, legs that must detour, legs with no route,
legs tangent to an exclusion at exactly the clearance, and clearance applied to
the airframe rather than the camera swath. Prior art is the existing planner
suite, which already covers lane clipping and plan/exclusion intersection.

**Behaviour-tree stage against a fake vehicle.** The dwell state machine, the
per-distinct-payload hover, exclusion-aware transit legs and the release-
altitude guard are tested by ticking the stage and asserting what the fake
vehicle **recorded** — the yaw targets commanded, the dwell durations held, the
waypoints flown, the abort reasons set. Prior art is the fail-closed stage
suite and the altitude-hold suite; the latter exists specifically because an
earlier version tested the correction calculation while the published setpoint
stayed at zero.

The fake vehicle's fidelity is itself a test obligation. It previously
hardcoded the QR offset's validity flag to zero, which made every target
permanently invisible and quietly voided the tests that depended on it.

**Banner identity on rendered frames.** The stroke path is graded on frames
captured from the arena under shadow and at oblique angles, asserting the
identity verdict and which path produced the read. The negative case — a green
board with no lettering — is as important as the positives. Prior art is the
existing identity and text suites.

**Aggregator snapshot and frontend contract.** The ledger is tested by driving
the aggregator's message handlers and asserting the broadcast snapshot:
de-duplication, matched tagging, rejection reasons retained, bounded length,
and survival across a reconnect. The existing contract test that keeps the
frontend types aligned with the snapshot keys covers the new key without
modification.

**Live arena regression as the acceptance seam.** Red-zone compliance cannot be
closed at any unit seam, and this is not a matter of thoroughness — it has
already happened. Unit tests passed while the aircraft overflew red ground, and
the diagnosis inverted only when the recorder sampled the aircraft at sweep
altitude instead of on the pad. The closing evidence for the red-zone item is a
recorded flight track checked against the confirmed exclusion set, on more than
one randomised arena, including the main seed.

Arena regressions must run one at a time. Two overlapping runs each tear down
the other's simulator and vehicle, which previously produced an entire results
table that had to be retracted. The regression script holds an exclusive lock
and must not leak its lock descriptor to child processes.

## Out of Scope

**The interactive world builder.** The page for placing arena components and
exporting a world is explicitly sequenced after this work, by the user's own
instruction: fix everything first, then build the builder for testing.

**The photo corpus.** Deferred by the user, and the three skipped tests that
depend on it stay skipped.

**Hardware.** Simulator only. No flight-controller, companion-computer or
camera hardware is available, so nothing here may depend on bench validation.

**Fixed arena geometry.** Out of scope permanently, not just for this spec. No
fix may introduce a hardcoded coordinate, extent or heading. The randomised
arena regression exists because passing on the shipped arena cannot distinguish
a perception-derived value from a constant that happens to agree with it.

**Reading ground truth from the simulator inside the mission.** Ground-truth
comparison belongs in the separate measurement tools, where knowing the answer
is legitimate.

## Further Notes

**Two findings in this area were retracted, and the retractions matter more
than the findings.** "Red zones are never detected" was wrong — detection
works, and confirmed exclusions were building steadily during the sweep. "The
main-seed run regressed at the winch drop" was wrong — that abort was the
operator pressing the abort control. Both are written up in the verification
record. Anyone picking this spec up should read those two sections before
forming a theory, because the surviving red-zone defect is specifically
*downstream* of detection: the exclusions exist and the aircraft crosses them
anyway.

**Five confident causal stories died to measurement during this work** —
harness contamination, a receding approach target, yaw authority, corridor
rotation, and the two retractions above. Each was plausible and each was wrong.
The discipline that resolved them is recorded in the verification document:
treat any diagnosis as provisional until it has been recorded in the state the
failure actually occurs in.

**The vocabulary is load-bearing.** Restricted zone, red zone and exclusion are
three different things — the rulebook's rule, the painted ground the camera
sees, and the georeferenced rectangle the planner routes around. Detect,
identify and align are three different claims, as are scan, decode and match,
and `NOT_VISIBLE` is not `CLEAR`. The glossary at the repository root is the
reference, and collapsing any of these pairs has already caused a defect.

**Scored weight, for prioritising.** Payload delivery accuracy is 15 marks,
autonomous return 20, target identification 10, restricted-zone avoidance 10 at
minus 5 per violation, landing 5, and mission completion within the 15-minute
window 15 proportionally. Restricted zones and delivery accuracy are the two
items here with marks directly attached and no current evidence.
