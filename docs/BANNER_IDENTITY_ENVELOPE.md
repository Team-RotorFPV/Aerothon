# Banner identification envelope (measured)

`sim/measure_banner_identity.py`, against the real rendered fixture
`tests/fixtures/banner_sim_ambient_3m.png` captured from a live run.

## Why measure it

`banner_node` declares `max_detect_range_m = 25.0` and derives its minimum
blob area from it. That gate decides whether a green thing is **big enough to
be worth looking at**. It says nothing about whether the **lettering** — the
thing that actually decides identity — is resolvable at that range.

Only the area range was declared. The identity range was unknown, and it is
the binding one.

## What transfers

Not "it works to 30 m" — that is true only of this banner at this resolution.
What transfers is **pixels across a letter**:

```
focal_px      = image_width / (2 tan(hfov/2))
px_per_letter = focal_px * letter_width_m / range_m
```

so for any banner and any camera:

```
range_max = focal_px * letter_width_m / px_per_letter_floor
```

## Measurement

The fixture is rescaled inside a constant-size frame, so only the banner's
angular size changes and the area gate (a fraction of frame) is not confounded.
Halving the linear size is the same projection as doubling the range.

1280 x 720, 60° HFOV, simulated banner: 8 characters across 3.25 m
(letter width 0.406 m).

| range (m) | px/letter | identified | reason |
|---:|---:|:---:|---|
| 3.00 | 150.1 | yes | |
| 4.29 | 105.1 | yes | |
| 6.00 | 75.1 | yes | |
| 8.57 | 52.5 | yes | |
| 12.00 | 37.5 | yes | |
| 15.00 | 30.0 | yes | |
| 20.00 | 22.5 | yes | |
| 25.00 | 18.0 | yes | |
| **30.00** | **15.0** | **yes** | last success |
| 37.50 | 12.0 | no | green region too small (1134 px) |
| 50.00 | 9.0 | no | green region too small (24 px) |
| 60.00 | 7.5 | no | |
| 75.00 | 6.0 | no | |

**Identity floor: 15 px/letter.** For this banner and camera that is a 30 m
stand-off — and note that the failures at and beyond 37.5 m are reported as
*area* failures, not lettering failures. The area gate binds first. The
lettering check is not the limiting factor anywhere in the usable range.

## What this ruled out

Arena-regression seed 1001 moved the gate to 6.3 m from the aircraft (nominal
is 2.8 m) and the mission failed to identify the banner. The obvious
explanation — lettering too small at 2.2x the range — is **wrong**: at 6.3 m
the detector has about five times the resolution it needs.

Recorded because a plausible, wrong diagnosis nearly became a fix aimed at
nothing. The real seed 1001 behaviour is a marginal, non-deterministic
identification followed by `ApproachBanner: banner lost before reaching the
mouth` — see `VERIFICATION.md` §11b.

## Caveat

Measured against a **simulated** banner rendered by Gazebo. The real
competition banner's proportions, lettering and finish are unconfirmed
(geometry audit E2 remains open on the photo corpus). What the measurement
establishes is the *shape* of the relationship and the px/letter floor for
this detector, not a number to fly on without re-measuring against real
photographs.
