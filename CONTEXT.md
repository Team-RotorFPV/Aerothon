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

**Delivery-zone boundary** — the closed geographic polygon supplied by the
organisers before flight. It is the authoritative search envelope for the
delivery zone and required mission input.

**Mission-ready boundary** — a delivery-zone boundary that is present, valid
and usable. Without one, the aircraft is not ready to arm.

**Arena geofence** — the organiser-supplied polygon the whole flight must stay
inside (rulebook: "Coordinates for the geo-fence boundary will be provided").
Distinct from the delivery-zone boundary, which bounds only the search. It is
uploaded to the flight controller, read back and enforced before arming;
exclusions are never added to it.

**Bottom-left start** — the lawnmower search always begins at the south-west
(min x, min y) corner of the delivery-zone boundary, whatever point the
corridor let the aircraft out at.

**Free-space window** — the open ground visible to a ranging sensor from one
pose. It describes immediate obstacle clearance, not the delivery-zone
boundary.

## Perception

**Detect** — something of the right kind is present in frame.

**Identify** — that thing is confirmed to be *the* banner, by structure and
optionally by reading its lettering. Stronger than detect.

**Align** — the aircraft has yawed to face an identified banner. The rulebook
scores "detect ... and autonomously align" as one task. Align says where the
nose points, nothing about where the aircraft is.

**Square on** — the aircraft is perpendicular to the banner's face, within
tolerance, at a measured standoff. Stronger than align, and measured by the
lidar rather than inferred from the camera. An aircraft can be aligned while
well off to one side; only a square-on aircraft may advance through the gate.

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

**Verified target** — a target pad whose payload matched repeatedly while the
aircraft was stationary, which the aircraft then centred over and reconfirmed.
Only a verified target may be used for payload delivery.

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

**Custom arena / world spec** — an arena a person placed by hand in the world
editor (`tools/world_editor`), saved as JSON in `sim/worlds/`: take-off area,
the outbound and return corridors (each with its own pose, length, width and
wall height; the return one *linked* beside the outbound one, as in the
rulebook drawing, or placed anywhere), the return corridor's obstacles, the
delivery zone, geofence, any number of red zones, the pads, decoys, the named
target and QR sizes. Unlike a randomised arena nothing about it is chosen by
the code, so it is the sharpest test for a surviving constant. Only the
banners and the take-off pad's internal layout keep their shape.

**Return gate search** — how the aircraft finds the return corridor without
being told where it is: a full turn at the stand-off first (the rulebook
layout ends here, on the first heading), then vantage points along the
delivery zone's edge, nearest the outbound exit first, each looking outward.
A sighting is the identified banner that is not the outbound one (whose
position was recorded on the way out) and is within the near range.

**Team airframe** — the vehicle the simulator flies by default
(`AEROTHON_AIRFRAME=cad`): the team's quad built from its CAD in
`Drone frame/` by `scripts/cad_to_gazebo.py` (meshes and measured mounts in
`models/aerothon_quad/airframe.json`) and `scripts/build_cad_vehicle.py`.
2.0 kg all-up, 2312 980 KV on 9450 props at 4S (4S2P 9000 mAh Li-ion),
Logitech C270 (48.8 deg HFOV) on a tilt servo, LD06 lidar on the raised
front mount, gravity-hook winch. `AEROTHON_AIRFRAME=iris` flies the older
ArduPilot Iris variant.

**Gravity hook** — the team's drop mechanism: a motor lowers the payload on
a hook that lets go by itself once the payload rests and the line goes slack.
The winch pays out past touchdown for that slack; "release" sends nothing on
the aircraft (in Gazebo the winch node detaches on slack).

**Edge-on orbit** — what both banner searches do when a full turn reads no
banner but did see green: the board is being seen from the side or behind,
where no lettering shows. The largest green region is taken as where the gate
stands (bearing from the camera, range from where it meets the ground or from
its height), and the aircraft flies vantage points round it at search
altitude, facing it, until the lettering reads. It is tried before the blind
relocation pattern and does not count against its budget. An empty heading
ends once it can no longer reach the dwell's hit floor, and the lettering can
be read mid-leg. With no green seen anywhere, a ring of vantage points round
the search's start is the last resort. `banner_orbit.py`.

**Confirmed delivery** — a drop the nadir camera has seen: after the release
and after the winch has wound back up, the payload is in frame on the ground,
at the pixel size its edge length has from that altitude. Distinct from the
winch's *released* flag, which only says the command was accepted. An
unconfirmed drop still flies home, but the outcome is DELIVERY_UNCONFIRMED,
not COMPLETED.
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
