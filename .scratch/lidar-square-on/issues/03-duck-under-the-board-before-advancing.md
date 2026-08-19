# Duck under the board before advancing through the gate

Status: ready-for-agent
Parent: ../spec.md

## What was watched

The aircraft squared up to the banner correctly, then flew forward into the
board, deflected off it, went left, and left the world.

The user's read: "it is not even avoiding it by going beneath the banner using
the lidar."

## The defect, in two numbers

`GateAdvance` flies its 10 m at `alt=p['corridor_alt']`, which defaults to
**3.0 m**.

`evidence/lidar-square-on/static/FINDINGS.md` measures the board as spanning
**z 2.805 to 3.955**.

3.0 is inside 2.805 to 3.955. The aircraft commits a 10 m waypoint at exactly
board height and flies into the board. No perception failure is involved; the
squaring worked, and the aircraft then drove through the thing it had squared
up to.

The gate's opening is **below the board**, between the posts, from the ground
to 2.805 m. Nothing in the mission code knows that the opening is not where
the aircraft is standing.

## The trap this design walked into

The lidar's usable band was established as 2.5 to 3.5 m because that is where
the scan plane intersects the board. That is the same statement as "where the
aircraft is at board height".

**Measuring squareness and passing through the gate are mutually exclusive
altitudes.** The band that makes the measurement possible is the band that
makes the transit impossible. Tuning the align altitude into the middle of the
lidar band tuned the aircraft into the collision.

This was not visible while the aircraft was failing to square up, because it
never got far enough to advance.

## What to build

A descent between squaring and advancing. Three steps, in order:

1. **Square up at board height**, 3.0 to 3.5 m, where the lidar sees the
   board. Unchanged from today.
2. **Find the board's bottom edge by descending**, then drop a margin below
   it.
3. **Advance 10 m at the lower altitude**, then rejoin the corridor.

### Finding the edge without hardcoding it

Do not put 2.805 in the code. The standing constraint is no fixed arena
geometry, and the real gate will differ.

The lidar already distinguishes the two cases and `FINDINGS.md` records both
signatures:

- At board height: one wide continuous face, about 60 returns, residual under
  1 cm, span of order the board's width.
- Below the board: two narrow clusters, the posts, about 11 returns, residual
  4.5 cm.

The altitude at which one wide face becomes two narrow clusters **is** the
board's bottom edge, measured. Descend in steps, watch for that transition,
and take the altitude where it happens as the edge. Then descend a clearance
margin below it before advancing.

This is the user's own suggestion. They said the aircraft should go beneath
the banner using the lidar, and the lidar can find where beneath begins.

### Refuse rather than guess

If the transition is never observed, or the aircraft reaches a floor without
finding it, **refuse to advance**. Do not fall back to advancing at the align
altitude, which is the behaviour that put the aircraft into the board. Report
the altitudes tried and what the lidar returned at each.

The posts themselves give a second, independent check: below the board the
aircraft should see two clusters roughly 3.8 m apart with a gap between them,
and it is flying at the gap. If the forward sector between the posts is clear
to beyond the advance distance, the path is open. If it is not, something is
in the gate and the aircraft should not advance.

## Testing

Through the fake commander, with a fake whose lidar returns depend on
altitude, since that is the whole mechanism.

- Squared up at board height, assert the aircraft **descends** rather than
  advancing.
- Descending through the transition, assert it identifies the edge at the
  altitude where the wide face becomes two clusters.
- Below the edge with a clear gap, assert it advances.
- Below the edge with an obstruction between the posts, assert it refuses.
- Transition never observed down to the floor, assert it refuses and names the
  altitudes tried.
- **Regression: assert the aircraft never commits the advance waypoint while
  its altitude is inside the measured board span.** This is the test that
  would have caught the collision, and it is the one that must not be allowed
  to rot.

The fake must model actuation lag, per the parent spec.

## Note

Check whether the return leg has the same defect. There is a second
`GateAdvance` in the return corridor, also constructed with
`alt=p['corridor_alt']`. If the aircraft must duck under the board outbound,
it must duck under it coming back.
