# Align off the bounding box, immediately, and stop hedging

Status: ready-for-agent
Parent: ../spec.md

## What was watched

Run 16 onward, seed 1001, GCS open, live. In order:

1. The aircraft identifies the banner.
2. It yaws left and rolls left for no reason the log explains.
3. It comes back toward the banner.
4. It holds still, **continuing to detect the banner the whole time**, and
   never turns to face it.

Step 4 is the important one. Detection is working. The aircraft can see the
banner and is not acting on it.

## Three defects

### 1. Squareness gates the centring, when it should gate only the advance

The parent spec asked for the lidar to gate **the advance through the gate**.
What is built gates everything behind it, so with `/scan` arriving at 4.3 Hz
reading 0.00 m, the aircraft cannot even turn to face a banner it can see.

Separate the two. **Centring must run off the camera, unconditionally, on
every tick where the banner is identified.** It needs no lidar, no
perpendicularity, no confirmation. Point the nose at the thing. Only the
advance waits for the square-on confirmation.

Fail-closed is right for committing a waypoint. It is wrong for turning to
look at something.

### 2. The sweep runs before checking whether the banner is already visible

The unexplained yaw and roll at entry is the zigzag sweep searching for a
banner that is already in frame. **On entry, look first.** If the banner is
identified, go straight to centring. Sweep only when nothing is visible.

### 3. Too many paths

The user's words: "as soon as it detects the banner it should not be confused
and it should not try to do different things."

This is a criticism of the design, and it is fair. The stage accumulated peak
detection, best-of-turn fallback, direction reversal on narrowing, and an
aspect threshold, each added to rescue the previous one. Together they make
the aircraft look like it is guessing, because it is.

One behaviour per state. Given an identified banner, the response is always
the same response. Delete the alternatives rather than ordering them.

## What each instrument is for

The parent spec said the lidar wins when the two disagree. That still holds
**for perpendicularity and range**. It was never meant to mean the camera
stops being used. Split by job:

| Question | Instrument |
|---|---|
| Is that the banner? | Camera. Green region plus OCR reading AEROTHON. |
| Which way do I turn? | **Camera.** Bounding box midpoint against frame midpoint. |
| Am I perpendicular? | **Lidar** primary. Box geometry is the cross-check. |
| How far away am I? | **Lidar** range primary, box size as fallback. |
| May I advance? | Perpendicular within tolerance **and** range in band. |

Turning is a camera job and must never wait on the lidar. If the lidar is dead
the aircraft should still centre the banner in frame; it simply must not
advance through the gate.

The box geometry keeps a real role as the cross-check on perpendicularity,
which is what the user is pointing at when they say the box can tell you
whether you are at an angle. Report both numbers side by side in the log every
time they are measured, so a disagreement is visible rather than silent. When
they disagree, the lidar decides, per the parent spec. But a persistent
disagreement is a defect worth surfacing, not smoothing over.

## Centring, concretely

The user's definition: the bounding box midpoint should come into contact with
the camera midpoint.

That is a bearing error, and it is already published. Drive it to zero with
yaw. Discrete latched corrections as the parent spec describes, because
per-tick proportional correction against a lagging airframe is the receding
carrot and this project has met it three times.

Nothing about this needs the lidar. It should start on the first tick the
banner is identified and it should be the only thing happening until the
bearing is inside tolerance.

## Testing

Through the fake commander.

The case that pins defect 1: the lidar reporting nothing usable while the
camera reports a confident banner at a large bearing. Assert the aircraft
**yaws toward it**. The current build holds still, so this test fails today,
which is the point of writing it.

The case that pins defect 2: enter the stage with the banner already
identified dead ahead. Assert **no sweep motion is commanded at all**.

The case that pins defect 3: given an identified banner and a fixed state,
assert the same command is produced every time. Any path that produces a
different action from the same inputs is one of the alternatives that should
have been deleted.

Also: with the lidar dead, assert the aircraft centres and then **refuses to
advance**. Both halves matter. Centring without the lidar is required; advancing
without it is forbidden.

The fake must model actuation lag, per the parent spec.
