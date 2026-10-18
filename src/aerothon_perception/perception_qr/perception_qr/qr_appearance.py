"""Match a QR by how it LOOKS, not by what it says.

WHY THIS EXISTS

    The organisers confirmed the start marker is 1 m square with a 3-inch
    white border, and that it is NOT settled whether the codes carry any
    encoded data at all. The requirement is that the aircraft "identify and
    match the QR code based on its visual appearance (image matching),
    without relying on decoded information".

    Decoding was the only identification the stack had. A code that carries
    nothing decodes to nothing, so on the day that path can return no
    evidence at all while the marker sits squarely in frame. This supplies
    the primary evidence instead: the pad marker whose PATTERN is the one the
    start marker showed. A decoded payload, when there is one, then confirms
    it (`qr_match.matches`).

HOW

    A located marker is rectified off its own four corners to a square and
    read as a grid of QR MODULES. The module count (21, 25 ... 41 for
    versions 1-6) is the one whose finder patterns and timing rows the image
    agrees with best, and the grid is turned so the corner WITHOUT a finder
    pattern is bottom-right -- so a pad photographed from the other side is
    the same grid.

    Only the DATA modules are compared. Every QR of a version shares its
    three finder patterns, timing rows and alignment pattern; a comparison
    that counts them scores two different codes as alike. The first version
    of this module compared a coarse 16x16 occupancy grid including those
    shared parts and scored different codes up to 0.92 against a 0.82 bar
    (logs/qr_appearance_study_20261014). On data modules alone two different
    codes agree about half the time and the same code nearly always.

    When the module grid cannot be read (too few pixels per module, a
    pattern that is not a standard QR) the coarse grid is the fallback, with
    the three finder corners masked out for the same reason. A coarse
    signature is only ever compared with another coarse one.
"""

from dataclasses import dataclass

import numpy as np

GRID = 16                 # coarse fallback, cells a side
MIN_SIDE_PX = 24          # below this a rectified marker is mush
# Agreeing data modules needed to call it the same pattern. Codes that
# differ by one character ("A"/"D") share their padding and agree on up to
# 0.894 of data modules; the same code under perspective, blur and noise
# scores 0.95-1.0 (logs/qr_appearance_study_20261014).
DEFAULT_THRESHOLD = 0.94
# The coarse fallback has no such measurement behind it, so it must agree
# more closely than its own different-code study showed (0.92 with corners).
COARSE_THRESHOLD = 0.93
SIZES = (21, 25, 29, 33, 37, 41)   # versions 1-6: one alignment pattern each
MIN_GRID_SCORE = 0.80     # finder + timing agreement needed to trust a module grid
WARP_PX = 8 * SIZES[-1]   # rectified canvas: 8 px per module at version 6

# The 7x7 finder pattern: dark ring, light ring, dark 3x3 centre.
_FINDER = np.ones((7, 7), np.uint8)
_FINDER[1:6, 1:6] = 0
_FINDER[2:5, 2:5] = 1


@dataclass(frozen=True)
class Signature:
    kind: str               # "modules" or "coarse"
    n: int                  # cells a side
    bits: np.ndarray        # 1 = dark
    mask: np.ndarray        # True where the cell is compared
    grey: np.ndarray = None  # the rectified marker, to re-read at another grid


def _order(quad):
    """The four corners as top-left, top-right, bottom-right, bottom-left."""
    pts = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    middle = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1]-middle[1], pts[:, 0]-middle[0])
    # Sorting by angle gives a consistent winding; rotate so the corner
    # nearest the top-left of the frame comes first.
    order = np.argsort(angles)
    pts = pts[order]
    start = int(np.argmin(pts.sum(axis=1)))
    return np.roll(pts, -start, axis=0)


def _rectify(bgr, quad):
    """The marker as a WARP_PX square grey image, or None if too small."""
    import cv2
    pts = _order(quad)
    side = max(np.linalg.norm(pts[0]-pts[1]), np.linalg.norm(pts[1]-pts[2]))
    if side < MIN_SIDE_PX:
        return None
    target = np.array([[0, 0], [WARP_PX-1, 0], [WARP_PX-1, WARP_PX-1],
                       [0, WARP_PX-1]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(pts.astype(np.float32), target)
    flat = cv2.warpPerspective(bgr, transform, (WARP_PX, WARP_PX))
    grey = cv2.cvtColor(flat, cv2.COLOR_BGR2GRAY) if flat.ndim == 3 else flat
    return grey


def _cells(grey, n):
    """Mean of the central half of each of n x n cells, as uint8."""
    import cv2
    step = grey.shape[0] / n
    inset = max(1, int(step * 0.25))
    out = np.empty((n, n), np.float32)
    integral = cv2.integral(grey.astype(np.float32))
    for r in range(n):
        y0, y1 = int(r*step) + inset, int((r+1)*step) - inset
        y1 = max(y1, y0 + 1)
        for c in range(n):
            x0, x1 = int(c*step) + inset, int((c+1)*step) - inset
            x1 = max(x1, x0 + 1)
            s = (integral[y1, x1] - integral[y0, x1] - integral[y1, x0]
                 + integral[y0, x0])
            out[r, c] = s / ((y1-y0)*(x1-x0))
    return np.clip(out, 0, 255).astype(np.uint8)


def _binarise(cells):
    import cv2
    _, dark = cv2.threshold(cells, 0, 1, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return dark.astype(np.uint8)


def _finder_scores(bits):
    n = bits.shape[0]
    corners = (bits[:7, :7], bits[:7, n-7:], bits[n-7:, n-7:], bits[n-7:, :7])
    return [float(np.mean(block == _FINDER)) for block in corners]


def _timing_score(bits):
    """Row and column 6 between the finders alternate dark/light."""
    n = bits.shape[0]
    expect = (np.arange(8, n-8) % 2 == 0).astype(np.uint8)
    return float((np.mean(bits[6, 8:n-8] == expect)
                  + np.mean(bits[8:n-8, 6] == expect)) / 2.0)


def _data_mask(n):
    mask = np.ones((n, n), bool)
    mask[:9, :9] = False            # finder, separator, format info
    mask[:9, n-8:] = False
    mask[n-8:, :9] = False
    mask[6, :] = False              # timing
    mask[:, 6] = False
    if n >= 25:                     # the single alignment pattern of v2-v6
        mask[n-9:n-4, n-9:n-4] = False
    return mask


def _module_signature(grey):
    best = None
    for n in SIZES:
        bits = _binarise(_cells(grey, n))
        scores = _finder_scores(bits)
        # Turn the grid so the corner without a finder is bottom-right
        # (index 2 in TL, TR, BR, BL order).
        missing = int(np.argmin(scores))
        bits = np.rot90(bits, k=(missing - 2))
        present = sorted(scores)[1:]
        if min(scores) > 0.75:      # four finders: not a QR grid at this n
            continue
        score = 0.6*float(np.mean(present)) + 0.4*_timing_score(bits)
        if best is None or score > best[0]:
            best = (score, n, bits)
    if best is None or best[0] < MIN_GRID_SCORE:
        return None
    _, n, bits = best
    return Signature("modules", n, bits.astype(np.uint8), _data_mask(n), grey)


# Registration search for re-reading a marker at a KNOWN module count, in
# fractions of a module and grid scales around the located corners.
SHIFTS = (-0.4, -0.2, 0.0, 0.2, 0.4)
SCALES = (0.95, 0.975, 1.0, 1.025, 1.05)
MIN_REGISTRATION = 0.78   # finder + timing agreement to trust a re-read


def _registered_bits(grey, n):
    """`grey` read as an n-module grid in canonical orientation, and its quality.

    A pad seen from the sweep is 3-5 px a module; its located corners can be
    off by a module, and reading it with no prior picks the wrong grid size.
    The start marker already says how many modules there are, so this reads
    at that count and searches a small rotation/shift/scale neighbourhood for
    the registration whose finder and timing patterns agree best. Only those
    FIXED parts drive the search, so it cannot tune itself to the data.
    """
    import cv2
    size = grey.shape[0]
    smooth = cv2.blur(grey, (3, 3)).astype(np.float32)
    centres = np.arange(n) + 0.5
    best_score, best_bits = -1.0, None
    for turns in range(4):
        image = np.rot90(smooth, turns)
        for scale in SCALES:
            cell = size * scale / n
            base = (size - size * scale) / 2.0
            for dy in SHIFTS:
                ys = np.clip((base + (centres + dy) * cell).astype(int), 0, size - 1)
                for dx in SHIFTS:
                    xs = np.clip((base + (centres + dx) * cell).astype(int), 0, size - 1)
                    cells = image[ys[:, None], xs[None, :]]
                    if cells.max() - cells.min() < 20:
                        continue
                    level = (np.percentile(cells, 25) + np.percentile(cells, 75)) / 2.0
                    dark = (cells < level).astype(np.uint8)
                    found = _finder_scores(dark)
                    score = (0.6 * float(np.mean([found[0], found[1], found[3]]))
                             + 0.4 * _timing_score(dark))
                    if score > best_score:
                        best_score, best_bits = score, dark
    return best_bits, best_score


def _coarse_signature(grey, grid=GRID):
    import cv2
    grey = cv2.GaussianBlur(grey, (3, 3), 0)
    cells = cv2.resize(grey, (grid, grid), interpolation=cv2.INTER_AREA)
    # Threshold against the marker's own mean: a code read in shade is the
    # same code, and an absolute level would be an assumption about the light.
    bits = (cells < float(cells.mean())).astype(np.uint8)
    corner = int(round(grid * 8 / 25))   # a finder + separator of a v2 code
    mask = np.ones((grid, grid), bool)
    for rows in (slice(0, corner), slice(grid-corner, grid)):
        for cols in (slice(0, corner), slice(grid-corner, grid)):
            mask[rows, cols] = False
    return Signature("coarse", grid, bits, mask, grey)


def signature(bgr, quad, grid=GRID):
    """A located marker as a comparable signature, or None if it is unusable."""
    grey = _rectify(bgr, quad)
    if grey is None:
        return None
    return _module_signature(grey) or _coarse_signature(grey, grid)


def agreement(left, right):
    """Fraction of agreeing compared cells. 0.0 if they cannot be compared.

    When either side is a module grid, the other is read at that grid's
    module count (`_registered_bits`) unless it already is one of that size:
    a pad the sweep sees too small to size by itself is still compared
    module for module with the start marker.
    """
    if left is None or right is None:
        return 0.0
    if left.kind == "modules" or right.kind == "modules":
        ref, other = (left, right) if left.kind == "modules" else (right, left)
        if other.kind == "modules" and other.n == ref.n:
            bits = other.bits
        elif other.grey is None:
            return 0.0
        else:
            bits, quality = _registered_bits(other.grey, ref.n)
            if bits is None or quality < MIN_REGISTRATION:
                return 0.0
        return round(float(np.mean(ref.bits[ref.mask] == bits[ref.mask])), 4)
    if left.n != right.n:
        return 0.0
    best = 0.0
    bits, mask = right.bits, right.mask
    for _ in range(4):
        both = left.mask & mask
        if both.any():
            best = max(best, float(np.mean(left.bits[both] == bits[both])))
        bits, mask = np.rot90(bits), np.rot90(mask)
    return round(best, 4)


def same_marker(left, right, threshold=DEFAULT_THRESHOLD,
                coarse_threshold=COARSE_THRESHOLD):
    """Whether two signatures are the same printed pattern."""
    score = agreement(left, right)
    coarse = (left is not None and right is not None
              and left.kind == "coarse" and right.kind == "coarse")
    bar = coarse_threshold if coarse else threshold
    return score >= float(bar), score
