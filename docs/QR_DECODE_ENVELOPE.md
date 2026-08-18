# QR decode envelope

**Phase 1a.** Measured from the simulated camera, 2026-08-16.
Raw data: `docs/qr_decode_envelope.csv` · Harness: `sim/measure_qr_decode.py`

This document exists to replace a guess. `mission_tree.py` hardcodes
`search_alt = 10.0` (see `docs/GEOMETRY_AUDIT.md` A2) with no measured basis.
Here is the basis.

---

## What was measured

Aircraft flown to station directly above the start pad, camera confirmed NADIR
via `/camera/pose_state.settled`, 10 frames decoded per altitude with the same
`cv2.QRCodeDetector` the mission uses.

Simulated camera: **640 × 480, 60° HFOV**. Marker: **2.2 m across, 33 modules**
(version 4, ECC M).

| altitude | px / module | decode rate | marker px | payload |
|---:|---:|---:|---:|:--|
| 3 m | 12.32 | **100%** | 290 | correct |
| 5 m | 7.39 | **100%** | 182 | correct |
| 7 m | 5.28 | **90%** | 133 | correct |
| 10 m | 3.70 | 30% | 95 | correct |

**The decode cliff is between 5.3 and 3.7 px/module.** Take **≈5.3 px/module**
as the reliable floor. This matches the usual rule of thumb that a camera-based
QR reader wants 4–5 pixels per module.

### Off-axis, at 7 m

| lateral offset | fraction of half-FOV | decode rate |
|---:|---:|---:|
| 0.00 m | 0.00 | 90% |
| 1.01 m | 0.25 | **100%** |
| 2.02 m | 0.50 | 0% |
| 2.83 m | 0.70 | 0% |

Reliable only within roughly **the inner quarter of the half-FOV**. Beyond that
the oblique view defeats the detector even though the marker is still nominally
within frame. **This constrains the search pattern**: a lane plan that only
guarantees a marker appears *somewhere* in frame is not good enough — it has to
pass near frame centre. That tightens `docs/GEOMETRY_AUDIT.md` B1 (lane
spacing) beyond simple FOV coverage.

---

## The number Phase 6 actually needs

Altitude does not transfer between marker sizes; **pixels per module** does:

```
px_per_module(h) = image_width · marker_size / (2 · tan(HFOV/2) · modules · h)

h_max            = image_width · marker_size
                   / (2 · tan(HFOV/2) · modules · px_per_module_floor)
```

At the measured floor of 5.28 px/module:

| marker size | max nadir altitude, **640 × 480 / 60°** (as simulated) |
|---:|---:|
| 0.30 m | 1.0 m |
| 0.40 m | 1.3 m |
| 0.50 m | 1.6 m |
| 0.60 m | 1.9 m |
| 1.00 m | 3.2 m |
| 2.20 m | 7.0 m |

**`search_alt = 10.0` is not achievable for any plausible competition marker at
the simulated camera resolution.** It only works because the simulated pad is
2.2 m across — and even then only to 7 m.

---

## Second run: realistic marker, realistic resolution

The table above was measured at 640×480 against a 2.2 m pad — both unrealistic.
Both are now simulator parameters (`AEROTHON_CAMERA_W/H`,
`AEROTHON_START_QR_M`), so the experiment was repeated at **1920 × 1080** with a
**0.5 m** marker. Raw data: `docs/qr_decode_envelope_1080p_0.5m.csv`.

| altitude | px / module | decode rate | marker px |
|---:|---:|---:|---:|
| 1.5 m | 16.80 | **37%** ← see near-field note | 366 |
| 2.5 m | 10.08 | **100%** | 229 |
| 3.5 m | 7.20 | **100%** | 171 |
| 5.0 m | 5.04 | 88% | 127 |
| 7.0 m | 3.60 | 0% | – |

### The floor is a property of the detector, not the render size

| run | resolution | marker | measured floor |
|---|---|---|---|
| 1 | 640 × 480 | 2.2 m | 5.28 px/module |
| 2 | 1920 × 1080 | 0.5 m | 5.04 px/module |

**≈5.0–5.3 px/module in both**, across a 3× resolution change and a 4.4× marker
size change. That is strong evidence the px/module model transfers, and that
the number can be used to size the search altitude for whatever marker the
organisers specify.

**A 0.5 m marker is reliably decodable to ~3.5 m, and marginally to 5 m.**
Not 10 m.

### Near-field limit: position hold versus field of view

The 37% at 1.5 m is not a resolution problem — it has the *most* pixels per
module of any row. At 1.5 m the camera sees only `2 × 1.5 × tan30° = 1.73 m` of
ground, while the position-hold tolerance used to declare a station reached is
±0.6 m. The marker simply drifts out of frame.

So descend-to-decode has a **lower** bound as well as an upper one, set by
position-hold accuracy relative to FOV — roughly, do not descend below about
`3 × position_error / (2·tan(HFOV/2))`. Tightening the hold tolerance or
centring on the marker visually (the `qr_offset` servo built in Phase 3) raises
that floor.

### Off-axis tolerance collapses for small markers

At 2.5 m with the 0.5 m marker:

| lateral offset | fraction of half-FOV | decode rate |
|---:|---:|---:|
| 0.00 m | 0.00 | 88% |
| 0.36 m | 0.25 | **12%** |
| 0.72 m | 0.50 | 0% |

Compare the 2.2 m marker at 7 m, which still managed 100% at a quarter of the
half-FOV. A small marker must be **near frame centre**, not merely in frame.
This is a hard constraint on the search pattern and reinforces the
sweep-then-descend recommendation below.

---

## The simulated camera is the pessimistic case

The sim renders at **640 × 480**. The actual aircraft carries a **Logitech Brio**,
and `goal.md` Q14 specifies a **1080p** stream. Pixels per module scale linearly
with image width, so the simulation understates real performance by 3× at 1080p
and 6× at 4K — before accounting for the Brio's wider default FOV, which pushes
the other way.

Recomputing at the measured 5.28 px/module floor:

| marker | 640×480 / 60° | 1920×1080 / 78° | 3840×2160 / 78° |
|---:|---:|---:|---:|
| 0.30 m | 1.0 m | 2.0 m | 4.1 m |
| 0.40 m | 1.3 m | 2.7 m | 5.4 m |
| 0.50 m | 1.6 m | 3.4 m | 6.8 m |
| 0.60 m | 1.9 m | 4.1 m | 8.2 m |
| 1.00 m | 3.2 m | 6.8 m | 13.6 m |

Two consequences:

1. **Raise the simulated camera resolution to whatever will actually be
   captured**, or every perception result from Gazebo is pessimistic by 3–6×
   and no altitude derived from it is trustworthy. The cost is real: the
   simulator already runs at 0.55 real-time factor (`VERIFICATION.md` 2.2).
2. **Even at 4K, a 0.4 m marker is unreadable from 10 m.** The single-altitude
   lawnmower in `LawnmowerSearch` cannot work for a small marker at any
   sensible sweep height.

---

## Recommendation for Phase 6

A single-altitude sweep that must both *cover* the zone and *decode* the marker
is over-constrained: coverage wants altitude, decoding forbids it.

**Split the two jobs:**

1. **Sweep high to find candidate pads.** Detect the pad, not the payload — a
   white quadrilateral of roughly the right size against grass is a much
   coarser target than 33 modules, and survives at far lower px/module.
2. **Descend over each candidate to decode.** Only then does the px/module
   floor apply, and it applies at the altitude of your choosing.

This also matches the rulebook flow: find the matching QR, then descend to
deliver. The descent was going to happen anyway.

The alternative — sweeping at the decode altitude — multiplies sweep time by
roughly the square of the altitude ratio and is unlikely to fit `goal.md` Q1's
5–8 minute budget.

---

## Open input — and why it no longer blocks

**The competition marker size is still unconfirmed**, and it is the dominant
term in every number above. It is on the outstanding organiser-clarification
list in `CURRENT_PROGRESS_HANDOFF.md`.

**It no longer blocks development.** Marker size is now a simulator parameter
(`AEROTHON_START_QR_M`, `AEROTHON_TARGET_QR_M`), and the px/module floor has
been shown to hold across a 4.4× change in marker size and a 3× change in
resolution. So the search strategy can be built and validated across the whole
plausible range now, and the real number simply selects a point in it:

```bash
# validate the search across the plausible marker range
for s in 0.3 0.4 0.5 0.6 1.0; do
  AEROTHON_START_QR_M=$s AEROTHON_CAMERA_W=1920 AEROTHON_CAMERA_H=1080 \
  AEROTHON_HEADLESS=1 ./scripts/launch_level6_sim.sh
  python3 sim/measure_qr_decode.py --marker-size $s
done
```

The search-altitude logic Phase 6 builds must therefore take marker size as an
**input**, not a constant — that is the real requirement this measurement
produces.

## Still requires physical access (parked)

The real-image corpus (`docs/CORPUS_SHOT_LIST.md`) is the only remaining check
that cannot be done in simulation. It confirms whether the ≈5 px/module floor
survives a real lens, real sensor noise, motion blur and sunlight. **Expect it
to be worse, not better** — so treat the simulated altitudes as an optimistic
bound until it is shot.
