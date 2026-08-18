# GCS watch checklist

The automated half of the probe is `sim/test_gcs_contract.py`: it asserts that
every group the aggregator publishes has a TypeScript type and that the fields
worth 15 and 10 rulebook marks are actually rendered. It already caught two
real gaps — `ready_reasons` published but never displayed, and delivery
accuracy not shown at all.

What it cannot catch is a value that renders correctly and **reads** badly.
That is what this list is for. Tick it while the mission flies.

## Before arming

- [ ] **Interlock rows** — eleven items, each with a measured value, not just a
      tick. Force one to fail (pull the lidar, or set `min_sats` high) and
      confirm the row goes red *and* the `Blocking` line names it in words.
- [ ] **ARM stays disabled** while any row is red.
- [ ] **Waived items** show as waived rather than silently passing.
- [ ] **FCU / GPS / EKF badges** in the header agree with Mission Planner.

## Camera pane — the thing you asked to see

- [ ] Default feed is `ALL DETECTIONS` (`/percep/overlay`).
- [ ] **Status bar** at the top of the frame names the camera pose and what is
      currently seen.
- [ ] **QR box** appears the moment a marker is decoded, and carries the
      payload text.
- [ ] **Banner box** appears during alignment, labelled `BANNER`. If the text
      rescue fired it reads `BANNER [AEROTHON]` — that is the OCR confirming.
- [ ] A **rejected** green object draws in amber as `GREEN, NOT BANNER`, not
      green. The decoys exist to make this visible.
- [ ] Boxes **disappear** when the object leaves frame. A box that lingers is
      the stale-overlay failure (`max_box_age_s`).
- [ ] Switch to `BANNER ONLY` and `RAW CAMERA` and back — the per-detector
      feeds are kept for exactly this.

## During the mission

- [ ] **Checklist** advances in step with the actual stage.
- [ ] **Red zone** shows the tri-state: `NO GROUND VIEW` before the nadir
      camera sees anything, then `CLEAR` or `RESTRICTED`. It must never read
      `CLEAR` before the camera has looked.
- [ ] **Exclusions** count rises as red ground is georeferenced.
- [ ] **Front lidar / centering** move sensibly during corridor navigation.
- [ ] **SLAM view** builds a map that resembles the corridor.
- [ ] **Map view** track matches Mission Planner's.

## At delivery and landing

- [ ] **Delivery** shows a number in metres, not `—`. This is the 15-mark
      figure and it is new; if it reads `—` the QR was not in frame at
      release and that is worth knowing.
- [ ] **Landing** shows `PRECISE (committed at …)` or `DEGRADED (…)`.
- [ ] Mission result line matches what the aircraft actually did.

## Cross-check against Mission Planner

Watch both. They are independent paths to the same aircraft, so a disagreement
is a real finding.

- [ ] Altitude, heading, battery and mode agree.
- [ ] Geofence appears in Mission Planner after `fence verified` logs.
- [ ] Mission Planner shows no failsafes the GCS is not also showing.

## Known-unverified

- Phase 10's interlock has never been watched during a live mission — only
  each item forced individually in tests.
- Red-zone avoidance has never flown with a zone genuinely inside the lane
  plan; every run so far logged `avoiding 0 red zone(s)`. Lateral search
  expansion changes that, so this flight is the first real test of it.
