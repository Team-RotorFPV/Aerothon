# Context

Domain vocabulary for the AeroTHON 2026 Mission 2 stack. Glossary only — no
implementation detail, no decisions. Decisions live in `docs/adr/`, evidence
lives in `VERIFICATION.md`.

## Arena

**Restricted zone** — the rulebook's term for ground the aircraft may not fly
over. Scored: 10 marks, minus 5 per violation.

**Red zone** — the physical painted ground in the arena. What the camera sees.
A red zone is the *appearance* of a restricted zone.

**Exclusion** — a georeferenced rectangle in local ENU that the lane planner
routes around. What the mission *believes* about a red zone, derived from
confirmed camera observations.

> These three are not synonyms and the distinction is load-bearing. A red zone
> constrains the **airframe**, not the **camera**: flying the camera's
> footprint over red ground is harmless, flying the aircraft over it is a
> violation. Clipping lanes against the camera swath instead of the airframe
> clearance over-clipped the plan and dropped coverage to 0.81.

**Delivery zone** — the area beyond the corridor containing the target pads.

**Observed zone** — the portion of the delivery zone the aircraft has actually
measured. Always a subset of the delivery zone, and initially bounded by lidar
range, not by the zone's real extent.

**Frontier** — the edge of the observed zone. Search expands it forward and
laterally in bands.

**Strip / band** — the new ground one frontier expansion adds. Swept once.

## Perception

**Detect** — something of the right kind is present in frame.

**Identify** — that thing is confirmed to be *the* banner, by structure and
optionally by reading its lettering. Stronger than detect.

**Align** — the aircraft has yawed to face an identified banner. The rulebook
scores "detect ... and autonomously align" as one task.

**Scan** — point the camera at a QR marker and attempt a read.

**Decode** — recover the payload string from a scanned marker. A scan may
fail to decode.

**Match** — a decoded payload equals the delivery target named by the start
QR. Decoding is not matching.

**NOT_VISIBLE** — the red-zone detector has no usable ground view, so it
reports nothing about the ground. Distinct from CLEAR, which asserts the
ground was seen and is not red. Collapsing the two makes "no camera" read as
"no hazard".

## Mission

**Start QR** — the marker at the take-off point naming the delivery target.

**Target pad** — a marker in the delivery zone. Exactly one matches.

**Decode altitude** — the highest altitude the payload marker is still
readable from. Derived from the measured px-per-module floor.

**Sweep altitude** — the altitude the search flies at, set by pad
detectability and capped by the rulebook ceiling.

**Tracking floor** — the lowest altitude at which the whole pad still fits in
frame. Below it, visual lock is geometrically impossible.

**Standoff** — how far short of a target the aircraft stops so the target is
in the camera's field of view rather than beneath it.

## Evidence

**Nominal / shipped arena** — the single arena committed to the repo.

**Randomised arena** — an arena generated from a seed, with the corridor,
delivery zone, pads and red zones moved.

**Arena regression** — the full mission flown across several randomised
arenas. The acceptance test for the perception-driven claim: passing on the
shipped arena cannot distinguish "derived from perception" from "derived from
a constant that happens to agree".
## Search and navigation

**Routed leg** — a transit between two points, checked against the exclusion
set and detoured around it if the straight line would put the airframe inside
one. Distinct from a **clipped lane**, which is a sweep lane cut where it
enters an exclusion. Every leg is routed; only sweep lanes are clipped.

**Blocked leg** — a leg with no route within the detour budget, or whose
destination lies inside an exclusion. A blocked leg is an abort with a reason,
never a straight line flown anyway.

## Perception, continued

**Dwell** — the aircraft holds a heading, stationary, while the detector's
verdicts accumulate. A step of the sweep is decided on the whole dwell, never
on a single frame.

**Confident dwell** — a dwell in which most of the samples agreed. What makes
covering a full turn safe: the hazard of a long sweep was never its length, it
was acting on the first frame that said yes.

**Lettering path** — how the letters were separated from the board.
*Brightness* asks whether a pixel is brighter than the board; *stroke* asks
whether it is brighter than its immediate surroundings. A shadow changes the
first answer and not the second. Both feed the same glyph reader; either may
confirm.

## Evidence, continued

**Scan ledger** — the list of everything decoded, identified or refused during
a run, de-duplicated, with matches tagged and refusals carrying their reason.
One row per distinct observation; the repeat count is itself evidence, since it
separates a solid read from a single-frame blip.

**Stage-gated recording** — a measurement taken only while the mission reports
the state under investigation. An ungated probe of the red-zone detector caught
the aircraft on the pad and produced a confident wrong answer; the gate is what
makes a recording evidence about a stage rather than about whenever the probe
happened to look.
