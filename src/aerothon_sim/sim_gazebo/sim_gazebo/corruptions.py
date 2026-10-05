"""What the real world does to the camera and the lidar, applied in simulation.

A Gazebo camera renders a perfect pinhole image: no blur, no sensor noise, no
sun, no dust on the lens, no vibration. A Gazebo lidar returns clean ranges.
Every perception threshold tuned against those is tuned against a sensor that
does not exist. This module is the difference, applied between the simulator
and the stack (sim_gazebo/degrade_node.py) and in the offline perception
envelope tests (sim/test_perception_corruption.py), so both measure the same
thing.

CAMERA CORRUPTIONS follow the ImageNet-C convention (Hendrycks & Dietterich,
ICLR 2019): each is a severity 0..5, 0 = off, 5 = worst credible. Sizes are
in pixels at 1280 wide and scale with the frame, so a severity means the same
thing at any render size. Each models something the C270 will meet:

    motion_blur  aircraft moving/rotating during the exposure
    defocus      fixed-focus lens, fogged or smeared lens
    noise        sensor noise; worst at dusk and on a small cheap sensor
    haze         dust / heat haze / morning mist between lens and ground
    exposure     signed -5..5: auto-exposure fooled by sky or bright ground
    glare        sun in or near the frame, veiling flare
    jpeg         MJPEG/USB compression at the bandwidth the Pi grants
    lens_dust    dirt and droplets on the lens (fixed for the whole run)
    vibration    rolling-shutter "jello" from prop vibration

LIDAR CORRUPTIONS are in physical units: gaussian range noise (m), dropout
(fraction of beams returning nothing: dark or glancing surfaces, sunlight),
and spurious (fraction of beams returning a short false range: dust, insects,
sun speckle).

Everything is seeded, so a failing run can be replayed exactly.
"""

import math

import cv2
import numpy as np

REF_WIDTH = 1280.0

CAMERA_KEYS = ("motion_blur", "defocus", "noise", "haze", "exposure", "glare",
               "jpeg", "lens_dust", "vibration")

# Per-severity magnitudes, index 0..5.
_MOTION_PX = (0, 4, 8, 14, 22, 32)
_DEFOCUS_SIGMA = (0, 1.0, 1.8, 2.8, 4.0, 5.5)
_NOISE_DN = (0, 4, 8, 13, 20, 28)
_HAZE_T = (1.0, 0.85, 0.70, 0.55, 0.42, 0.30)
_GLARE = (0, 0.25, 0.40, 0.55, 0.70, 0.85)
_JPEG_Q = (100, 70, 45, 30, 18, 10)
_DUST_SPOTS = (0, 6, 12, 20, 30, 45)
_VIBRATION_PX = (0, 1, 2, 4, 7, 11)


def _sev(value, lo=0, hi=5):
    return int(max(lo, min(hi, round(float(value)))))


def _odd(n):
    n = max(1, int(round(n)))
    return n if n % 2 else n + 1


class CameraCorruptor:
    """Applies one run's camera conditions frame after frame.

    The lens dust and the sun's position are properties of the RUN (the dirt
    does not move between frames, the sun drifts slowly), so they are drawn
    once; blur direction, noise and vibration phase change every frame.
    """

    def __init__(self, conditions, seed=0):
        c = dict(conditions or {})
        self.sev = {k: _sev(c.get(k, 0), -5 if k == "exposure" else 0)
                    for k in CAMERA_KEYS}
        self.rng = np.random.default_rng(seed)
        self._dust = None            # (shape, alpha map) built on first frame
        self._sun = self.rng.uniform(0.0, 1.0, 2)
        self._frame = 0

    @property
    def active(self):
        return any(self.sev.values())

    def apply(self, img):
        """BGR uint8 in, BGR uint8 out (same size). A no-op when inactive."""
        if not self.active:
            return img
        self._frame += 1
        h, w = img.shape[:2]
        k = w / REF_WIDTH
        s = self.sev
        out = img
        if s["vibration"]:
            out = self._vibration(out, _VIBRATION_PX[s["vibration"]] * k)
        if s["motion_blur"]:
            out = self._motion_blur(out, _MOTION_PX[s["motion_blur"]] * k)
        if s["defocus"]:
            sig = _DEFOCUS_SIGMA[s["defocus"]] * k
            out = cv2.GaussianBlur(out, (0, 0), sig)
        f = out.astype(np.float32)
        if s["haze"]:
            t = _HAZE_T[s["haze"]]
            f = f * t + np.float32([205, 212, 218]) * (1.0 - t)
        if s["glare"]:
            f = self._glare(f, _GLARE[s["glare"]])
        if s["exposure"]:
            f = f * float(2.0 ** (0.35 * s["exposure"]))
        if s["lens_dust"]:
            f = self._lens_dust(f, _DUST_SPOTS[s["lens_dust"]])
        if s["noise"]:
            f = f + self.rng.normal(0.0, _NOISE_DN[s["noise"]], f.shape).astype(np.float32)
        out = np.clip(f, 0, 255).astype(np.uint8)
        if s["jpeg"]:
            ok, buf = cv2.imencode(".jpg", out,
                                   [cv2.IMWRITE_JPEG_QUALITY, _JPEG_Q[s["jpeg"]]])
            if ok:
                out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return out

    def _motion_blur(self, img, length):
        n = _odd(length)
        if n < 3:
            return img
        kern = np.zeros((n, n), np.float32)
        kern[n // 2, :] = 1.0
        ang = float(self.rng.uniform(0, 180))
        rot = cv2.getRotationMatrix2D((n / 2 - 0.5, n / 2 - 0.5), ang, 1.0)
        kern = cv2.warpAffine(kern, rot, (n, n))
        return cv2.filter2D(img, -1, kern / max(kern.sum(), 1e-6))

    def _vibration(self, img, amp):
        """Rolling shutter: each row is read at a different instant, so a
        vibrating camera shears the frame sinusoidally down its height."""
        if amp < 0.5:
            return img
        h, w = img.shape[:2]
        phase = self.rng.uniform(0, 2 * math.pi)
        cycles = self.rng.uniform(2.0, 4.0)
        rows = np.arange(h, dtype=np.float32)
        shift = (amp * np.sin(2 * math.pi * cycles * rows / h + phase)).astype(np.float32)
        map_x = np.tile(np.arange(w, dtype=np.float32), (h, 1)) + shift[:, None]
        map_y = np.repeat(rows[:, None], w, axis=1)
        return cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REFLECT)

    def _glare(self, f, strength):
        h, w = f.shape[:2]
        # The sun drifts across the frame as the aircraft turns.
        self._sun = (self._sun + self.rng.normal(0, 0.01, 2)) % 1.0
        cx, cy = self._sun[0] * w, self._sun[1] * h
        yy, xx = np.ogrid[:h, :w]
        r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (0.35 * w) ** 2
        blob = np.exp(-r2).astype(np.float32)
        veil = 0.35 * strength
        return f + 255.0 * (strength * blob + veil)[..., None] * \
            (1.0 - f / 255.0 * 0.5)

    def _lens_dust(self, f, spots):
        h, w = f.shape[:2]
        if self._dust is None or self._dust[0] != (h, w):
            alpha = np.zeros((h, w), np.float32)
            for _ in range(spots):
                r = int(self.rng.uniform(0.01, 0.05) * w)
                x, y = int(self.rng.uniform(0, w)), int(self.rng.uniform(0, h))
                cv2.circle(alpha, (x, y), r, float(self.rng.uniform(0.25, 0.6)), -1)
            alpha = cv2.GaussianBlur(alpha, (0, 0), 0.01 * w)
            self._dust = ((h, w), np.clip(alpha, 0, 0.8)[..., None])
        a = self._dust[1]
        return f * (1.0 - a) + np.float32(70) * a


class LidarCorruptor:
    """Range noise, missing returns and false short returns on a LaserScan."""

    def __init__(self, conditions, seed=0):
        c = dict(conditions or {})
        self.noise_m = max(0.0, float(c.get("noise_m", 0.0)))
        self.dropout = min(0.9, max(0.0, float(c.get("dropout", 0.0))))
        self.spurious = min(0.5, max(0.0, float(c.get("spurious", 0.0))))
        self.rng = np.random.default_rng(seed)

    @property
    def active(self):
        return bool(self.noise_m or self.dropout or self.spurious)

    def apply(self, ranges, range_min, range_max):
        """ranges: 1-D float array. Returns a new array. A dropped beam reads
        +inf (what the LD06 driver publishes for no return)."""
        r = np.asarray(ranges, dtype=np.float32).copy()
        if not self.active:
            return r
        n = r.size
        valid = np.isfinite(r)
        if self.noise_m:
            r[valid] += self.rng.normal(0.0, self.noise_m, int(valid.sum())).astype(np.float32)
        if self.dropout:
            r[self.rng.random(n) < self.dropout] = np.inf
        if self.spurious:
            hit = self.rng.random(n) < self.spurious
            r[hit] = self.rng.uniform(max(range_min, 0.05), 2.0, int(hit.sum()))
        r[np.isfinite(r)] = np.clip(r[np.isfinite(r)], range_min, range_max)
        return r
