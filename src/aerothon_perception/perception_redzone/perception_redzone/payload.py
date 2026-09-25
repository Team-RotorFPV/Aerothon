"""Find the delivery payload in a camera frame.

The payload is a bright yellow box (0.12 x 0.12 x 0.08 m in simulation). No
other surface in the arena is in that hue band: the return lane is orange
(OpenCV hue ~8), red zones sit at 0/180, the field is green (~50), the pads
are black and white. So a saturated yellow blob in a nadir frame over the pad,
after the winch has let go and wound back up, is the payload on the ground.

Offsets use the QR detector's convention -- normalised to [-1, 1] of the half
frame, +x right, +y down -- so the mission converts both to metres the same
way and can measure the payload against the pad in one image.
"""

import cv2
import numpy as np

# OpenCV HSV: hue 0-180. Yellow ~30; orange (return lane) ~8 and red ~0/180
# are excluded by the lower bound.
YELLOW_LO = np.array([18, 120, 120], dtype=np.uint8)
YELLOW_HI = np.array([38, 255, 255], dtype=np.uint8)


def detect_payload(bgr, min_area_px=20):
    """Largest yellow blob in a BGR frame, or {"visible": False}."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {"visible": False, "img_w": w, "img_h": h}
    c = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < min_area_px:
        return {"visible": False, "img_w": w, "img_h": h}
    m = cv2.moments(c)
    cx = m["m10"] / m["m00"] if m["m00"] else 0.0
    cy = m["m01"] / m["m00"] if m["m00"] else 0.0
    _, _, bw, bh = cv2.boundingRect(c)
    return {"visible": True,
            "x": (cx - w / 2.0) / (w / 2.0), "y": (cy - h / 2.0) / (h / 2.0),
            "area_px": area, "w_px": int(bw), "h_px": int(bh),
            "img_w": w, "img_h": h}
