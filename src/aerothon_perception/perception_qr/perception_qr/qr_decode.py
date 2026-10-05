"""QR reading on a Pi 5 budget: zbar, then locate, then OpenCV on the crop.

Measured on rendered pads under the camera corruptions of
sim_gazebo/corruptions.py (sim/test_perception_corruption.py):

    zbar     survives motion blur, defocus and rolling-shutter vibration
             that defeat OpenCV, and costs about a third of OpenCV's time
             on a 1280x720 frame -- but fails on heavy sensor noise.
    OpenCV   survives heavy sensor noise that defeats zbar.

Neither alone covers what a vibrating aircraft sees in poor light. So read()
tries zbar on the whole frame; only if that finds nothing does it LOCATE a
marker by its three finder patterns (OpenCV, on a half-size frame -- the
finder squares are a code's largest features), and only if something is
located does OpenCV try to decode, on a crop around it. Most frames of a
search contain no marker at all, and those now cost zbar and a half-size
finder search instead of two full-frame decoders (62 -> ~30 ms per frame on
the test host; the Pi 5 is roughly half its speed).

A marker located but not decoded is returned as such: the finder patterns
survive blur and vibration that defeat decoding, and the mission stops over
it (DecodeHover) so the blur goes away. A located patch must look like a code
seen from above: near-square (MAX_SIDE_RATIO) and printed -- dark modules on
a white plate, a spread of at least MIN_CONTRAST grey levels with both tones
well represented. OpenCV's finder search alone reported a code in plain
grass texture in one frame in five, and in sun glare as a lopsided
quadrilateral half the frame across.

zbar is the libzbar0 system library behind pyzbar; without it every frame
goes to the locate step, and the node says so at start-up.
"""

import cv2
import numpy as np

try:
    from pyzbar import pyzbar
    _QR = [pyzbar.ZBarSymbol.QRCODE]
except (ImportError, OSError):      # OSError: pyzbar present, libzbar0 not
    pyzbar = None


MAX_SIDE_RATIO = 1.4        # longest side / shortest, a nadir view of a square
MIN_CONTRAST = 60           # grey levels between the patch's 10th and 90th percentiles
DARK_FRACTION = (0.2, 0.8)  # share of the patch darker than its midpoint


def zbar_available():
    return pyzbar is not None


class QrDecoder:
    def __init__(self):
        self._cv = cv2.QRCodeDetector()

    def read(self, bgr):
        """([(payload, quad)], unread quad or None); quads are 4x2 float32
        image corners."""
        found = self._zbar(bgr) if pyzbar is not None else []
        if found:
            return found, None
        quad = self.locate(bgr)
        if quad is None:
            return [], None
        found = self._opencv_crop(bgr, quad)
        return (found, None) if found else ([], quad)

    @staticmethod
    def _zbar(bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        out = []
        for r in pyzbar.decode(gray, symbols=_QR):
            pts = np.float32([(p.x, p.y) for p in r.polygon])
            if len(pts) != 4:
                pts = cv2.boxPoints(cv2.minAreaRect(pts)).astype(np.float32)
            payload = r.data.decode("utf-8", "replace")
            if payload:
                out.append((payload, pts))
        return out

    def locate(self, bgr):
        """4x2 float32 corners of a marker that can be seen, or None."""
        h, w = bgr.shape[:2]
        small = cv2.resize(bgr, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
        try:
            ok, pts = self._cv.detect(small)
        except cv2.error:
            return None
        if not ok or pts is None:
            return None
        quad = pts.reshape(4, 2).astype(np.float32) * 2.0
        return quad if _square(quad) and _looks_printed(bgr, quad) else None

    def _opencv_crop(self, bgr, quad, margin=0.25):
        """OpenCV's decoder on the located marker: first from the corners
        already found, then by its own detection on a crop around them."""
        try:
            text, _ = self._cv.decode(bgr, quad.reshape(1, 4, 2))
        except cv2.error:
            text = ""
        if text:
            return [(text, quad)]
        h, w = bgr.shape[:2]
        side = float(np.ptp(quad, axis=0).max())
        x0, y0 = np.maximum((quad.min(axis=0) - margin * side).astype(int), 0)
        x1, y1 = (quad.max(axis=0) + margin * side).astype(int)
        crop = bgr[y0:min(h, y1), x0:min(w, x1)]
        try:
            ok, infos, points, _ = self._cv.detectAndDecodeMulti(crop)
        except cv2.error:
            return []
        if not ok or points is None:
            return []
        return [(info, q.astype(np.float32) + np.float32([x0, y0]))
                for info, q in zip(infos, points) if info]


def _square(quad):
    sides = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
    return sides.min() > 0 and sides.max() / sides.min() <= MAX_SIDE_RATIO


def _looks_printed(bgr, quad):
    h, w = bgr.shape[:2]
    x0, y0 = np.clip(quad.min(axis=0).astype(int), 0, [w - 1, h - 1])
    x1, y1 = np.clip(quad.max(axis=0).astype(int), 0, [w - 1, h - 1])
    if x1 - x0 < 8 or y1 - y0 < 8:
        return False
    gray = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    lo, hi = np.percentile(gray, (10, 90))
    if hi - lo < MIN_CONTRAST:
        return False
    dark = float(np.mean(gray < (lo + hi) / 2.0))
    return DARK_FRACTION[0] <= dark <= DARK_FRACTION[1]
