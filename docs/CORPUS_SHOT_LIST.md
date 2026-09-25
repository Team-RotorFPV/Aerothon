# Real-image corpus — shot list

**Phase 1b.** This is the one part of the plan I cannot do from here: it needs a printed marker, a real banner and a real camera. Everything else in Phase 1 is automated.

Photographs go in `tests/perception/corpus/`. The harness that consumes them is `tests/perception/test_real_corpus.py`, which **skips** until the folder has files — so nothing breaks before the shoot.

---

## Why this matters more than it looks

Two numbers come out of this, and Phase 6 is blocked on both:

1. **The px/module at which a real camera stops decoding.** Everything measured in Gazebo is measured against a rendered marker with no lens, no sensor noise, no motion blur and no sun. The simulated pad is also **2.2 m across** — far larger than any plausible competition marker — so simulated altitudes do not transfer at all.

2. **Whether the HSV green thresholds survive real light.** `perception_banner` currently uses `s_lo=90, v_lo=60`, tuned by eye against a rendered green rectangle. Direct sun and overcast are very different problems.

The conversion, once you have (1):

```
max_altitude = (image_width × marker_size) / (2 · tan(HFOV/2) · modules · px_per_module)
```

At 640 px, 60° HFOV, 33 modules, that is roughly `5.6 × marker_size / px_per_module` metres.

---

## Equipment

- The **Logitech Brio** that will fly (not a phone — lens and sensor are the point). Lock focus and exposure if you can; note if you cannot.
- A printed QR at the size you expect to use. If the organiser size is still unknown, print **two** sizes (e.g. 300 mm and 500 mm) — the harness records size per shot, so both are useful and the answer scales.
- The actual green banner, or the material you will use for it.
- A tape measure. **Measured** distances, not paced ones.

Generate the QR with the payload the mission expects:

```bash
python3 scripts/generate_competition_assets.py
```

That writes `qr_start.png` etc. to `src/aerothon_sim/sim_gazebo/materials/`. Print `qr_start.png` — payload `AEROTHON2026:M2:TARGET_A`. **Note:** this needs the `qrcode` Python package, which is not installed here:

```bash
pip3 install --user qrcode pillow
```

The already-generated PNGs in `materials/` are committed, so you can print one of those directly without installing anything.

---

## Naming

```
qr_<size_mm>_<distance_cm>_<angle_deg>_<lighting>_<n>.jpg
banner_<distance_cm>_<angle_deg>_<lighting>_<n>.jpg
```

`lighting` ∈ `indoor` · `overcast` · `sun` · `shade` · `dusk`

Examples:

```
qr_400_300_0_sun_01.jpg        400 mm marker, 3.00 m, head-on, direct sun
qr_400_500_30_overcast_02.jpg  400 mm marker, 5.00 m, 30° off-axis, overcast
banner_800_0_sun_01.jpg        banner at 8.00 m, head-on, direct sun
```

Then record the expected payloads in `tests/perception/corpus/expected.json`:

```json
{
  "qr_400_300_0_sun_01.jpg": "AEROTHON2026:M2:TARGET_A",
  "qr_400_500_0_sun_01.jpg": "AEROTHON2026:M2:TARGET_A"
}
```

A file with no entry is still decoded and reported — it just isn't asserted against an expected string.

---

## The QR shots

Camera **pointing at the marker face**, marker filling frame centre. Take **3 frames per condition** (the harness averages them; one lucky frame proves nothing).

### A. Distance sweep — the important one

Head-on (0°), in **direct sun** and again in **overcast**. Keep going until it stops decoding; the failure point *is* the measurement.

| distance | why |
|---|---|
| 1.0 m | reference, should always work |
| 2.0 m | |
| 3.0 m | |
| 4.0 m | |
| 5.0 m | |
| 6.0 m | |
| 8.0 m | expect failure for a 300–400 mm marker |
| 10.0 m | expect failure |

**Do not stop at the first failure** — take two or three more steps past it so the cliff edge is bracketed.

### B. Angle sweep

At a distance that decoded reliably in A (likely 2–3 m), tilt the marker (or move the camera) off the face normal:

| angle | 
|---|
| 0° |
| 15° |
| 30° |
| 45° |

This matters because the search sweep will rarely be perfectly nadir over a pad.

### C. Lighting

At one good distance, head-on, one set in each of: `indoor`, `overcast`, `sun`, `shade`. If you can manage `dusk`, it is the worst realistic case and worth having.

### D. Motion blur (optional but valuable)

Two or three shots **while walking** at roughly the search speed (2 m/s per goal.md Q9) at a distance that worked when static. Real search flight is not a tripod.

---

## The banner shots

The banner detector needs to survive light, and to **reject** things that are merely green.

| set | what |
|---|---|
| Distance | banner head-on at 3, 5, 8, 12 m — 3 frames each |
| Angle | at 5 m, at 0°, 30°, 45° |
| Lighting | one set each in `indoor`, `overcast`, `sun`, `shade` |
| **Decoys** | 3–5 frames of *other green things* — grass, a green shirt, a tree, a green car. Name these `banner_0_0_<lighting>_9x.jpg` and leave them out of `expected.json`. |

The decoys are the point of Phase 4's identity gate: right now any green rectangle passes, and the GCS reports `ALIGNED`. If grass passes the gate at 8 m, that is a finding worth having before the flight line.

---

## Minimum viable set

If time is short, this alone unblocks Phase 6:

- **QR distance sweep, head-on, in direct sun**, 1 m → failure + 2 steps past, 3 frames each.
- **Banner at 5 m plus 3 green decoys**, in the same light.

That is about 30 photographs and roughly twenty minutes with a tape measure.

---

## Then

```bash
python3 -m pytest tests/perception/test_real_corpus.py -v -s
```

It prints a decode-rate table by condition, the px/module envelope, and the implied maximum stand-off per marker size. Paste that output into `VERIFICATION.md` under Phase 1 — and tell me, so I can wire the measured altitude into the search planner instead of the current `search_alt = 10.0` guess.
