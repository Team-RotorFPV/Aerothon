"""Undo what haze, exposure and sensor noise do to colour, before any colour test.

The banner and red-zone detectors threshold hue, saturation and value. Those
thresholds hold for a well-exposed frame and nothing else: haze adds grey
airlight (saturation falls under the floor), a backlit or dusk frame is dark
(value falls under the floor), sensor noise breaks thin white lettering into
speckle. Measured on the camera corruptions of sim_gazebo/corruptions.py
(sim/test_perception_corruption.py), the banner was lost at haze severity 3,
at half exposure and at heavy noise; the red zone was lost at haze severity 5.

The fix is one joint black/white-point stretch and a 3x3 median:

  * JOINT, not per channel: one lookup table for B, G and R, so hue and the
    channel ratios a colour test depends on are preserved. It removes an
    additive grey (haze) and a multiplicative gain (exposure), which is what
    those two corruptions are.
  * The gain is capped, so a frame that genuinely has little contrast (a
    uniform field of grass) is not amplified into noise.
  * On a well-exposed frame the percentiles sit near 0 and 255 and the table
    is the identity.

The points are measured on a 160x90 thumbnail; the stretch itself is one
cv2.LUT. Together with the median, about 3 ms per 1280x720 frame on a Pi 5.
"""

import cv2
import numpy as np

THUMB = (160, 90)
MAX_GAIN = 4.0
MIN_SPAN = 40.0          # below this the frame has no usable contrast at all


def normalise(bgr):
    small = cv2.resize(bgr, THUMB, interpolation=cv2.INTER_AREA)
    lo = float(np.percentile(small.min(axis=2), 1))
    hi = float(np.percentile(small.max(axis=2), 99))
    if hi - lo >= MIN_SPAN:
        gain = min(255.0 / (hi - lo), MAX_GAIN)
        lut = np.clip((np.arange(256, dtype=np.float32) - lo) * gain,
                      0, 255).astype(np.uint8)
        bgr = cv2.LUT(bgr, lut)
    return cv2.medianBlur(bgr, 3)
