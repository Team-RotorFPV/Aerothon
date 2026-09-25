#!/usr/bin/env python3
"""Read the word off a green board, and let the word decide.

WHY THIS REPLACES THE BLOB PIPELINE

    The identity check grew into a stack of heuristics, each added to rescue
    the case the last one broke:

        white components >= 3          then: which components are letters?
        components in a horizontal band  then: which band, when the wall is
                                               the same grey as the letters?
        board = band grown by 90%       then: the board comes out taller than
                                               wide and the aspect gate refuses
        board aspect 1.2-8.0            then: relax it, and tarpaulins get in
        stroke path beside brightness   then: it invents specks at frame edges

    Every one of those was a proxy for the only question that matters: does
    this green thing have AEROTHON written on it? A watched flight ended with
    the aircraft aligned to a 97x49 sliver of banner at the frame edge, which
    every proxy passed and the real question would have refused instantly.

    So the pipeline becomes: HSV finds green regions; this reads them; the
    reading decides.

TWO READERS

    tesseract, when it is installed -- a real OCR engine, and the right answer
    for a board carrying a word in an ordinary font.

    A template correlator otherwise, built from the same 5x7 glyph shapes the
    project already carries. It is weaker than tesseract on skewed views but
    needs no system package, which matters because the competition Pi may not
    have one either.

    The reader in use is reported, so a marginal read can be told from a
    confident one.
"""

import re
import shutil
import subprocess

import cv2
import numpy as np

from .glyphs import TEMPLATES, longest_in_sequence

TARGET = "AEROTHON"

# The BINARY, not the pytesseract wrapper.
#
# pytesseract is a thin shim that shells out to exactly this executable, and
# installing it here would mean writing into an externally-managed system
# Python (PEP 668) for no capability we do not already have. Calling the
# binary directly also keeps the deployment story honest: what the Pi needs is
# `apt install tesseract-ocr`, and nothing in a requirements file can express
# that.
TESSERACT = shutil.which("tesseract")


def tesseract_available():
    return TESSERACT is not None


def lettering_binary(roi_bgr, sat_max_ratio=0.55, min_side=48,
                     board_mask=None, close_frac=0.20):
    """Black text on white, ready for an OCR engine.

    The invariant that actually holds on a painted board is SATURATION: the
    lettering is close to neutral and the board is strongly coloured. Value is
    not invariant and assuming it was is what made the arena's own gate --
    grey letters at V=132 on green at V=184 -- unreadable from every angle,
    because the mask demanded letters BRIGHTER than the board.

    Upscaled because OCR engines want x-heights of roughly 20 px and a banner
    at range gives far less.
    """
    if roi_bgr is None or roi_bgr.size == 0:
        return None
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    # The reference has to be the BOARD's saturation, not the region's. The
    # arena's gate stands open in front of a grey wall, so half its bounding
    # box is unsaturated -- taking the region median makes the threshold
    # describe the wall and the whole box reads as "text". Measured: the real
    # banner then scored no better than a 97x48 speck.
    if board_mask is not None and np.count_nonzero(board_mask) > 0:
        # THE LETTERS ARE THE HOLES IN THE BOARD.
        #
        # No threshold at all. Close the board over its own lettering to get
        # the solid panel, then subtract the board: what is left is exactly
        # the shapes painted on it, whatever colour they happen to be. That
        # works for white-on-green and for the arena's grey-on-green without
        # knowing which it is looking at.
        #
        # Thresholding saturation was tried first and fragmented the glyphs
        # into their vertical strokes -- tesseract read one character out of
        # eight from it -- because antialiased edges sit between the board's
        # saturation and the paint's.
        # MEASURED on the rendered banner: at 0.10 of the region height the
        # close reaches only 84% coverage and recovers a fifth of the
        # lettering -- the vertical strokes and none of the horizontals, which
        # tesseract read as one character out of eight. 0.20 closes the panel
        # completely and recovers all of it. It stays well under the size of a
        # gate opening, which is what must NOT be filled.
        k = int(max(3, roi_bgr.shape[0] * float(close_frac))) | 1
        solid = cv2.morphologyEx(board_mask, cv2.MORPH_CLOSE,
                                 np.ones((k, k), np.uint8))
        mask = cv2.bitwise_and(solid, cv2.bitwise_not(board_mask))
    else:
        s_board = float(np.median(sat))
        thresh = max(40.0, s_board * float(sat_max_ratio))
        mask = (sat <= thresh).astype(np.uint8) * 255
    out = 255 - mask

    h, w = out.shape[:2]
    if min(h, w) < min_side and min(h, w) > 0:
        scale = float(min_side) / float(min(h, w))
        out = cv2.resize(out, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_CUBIC)
    return out


def rectify_lettering(binary, min_frac=0.0008, max_frac=0.06,
                      lo_aspect=0.12, hi_aspect=1.8, pad=6,
                      max_height_frac=0.40):
    """Crop and de-skew the lettering so an OCR engine can read it.

    A banner seen from the side puts its word on a diagonal, and the region it
    was found in also holds the posts and the gusset. Measured on the arena's
    gate, tesseract returned 'Pa' from the raw region in every page-segmentation
    mode it has.

    This is NOT identification -- nothing here decides whether the thing is a
    banner. It only presents the candidate text the right way up, and the
    reading still decides. That distinction is the whole point of the rewrite:
    the old pipeline used the same shape analysis to FIND the letters and to
    JUDGE them, so every improvement to one broke the other.
    """
    text = 255 - binary                      # components are the marks
    n, _, stats, _ = cv2.connectedComponentsWithStats(
        (text > 127).astype(np.uint8), connectivity=8)
    if n <= 1:
        return binary
    area = float(binary.shape[0] * binary.shape[1])
    pts = []
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        if not (area * min_frac <= a <= area * max_frac) or h < 2:
            continue
        if not (lo_aspect <= w / float(h) <= hi_aspect):
            continue
        # A LETTER IS NOT HALF THE REGION TALL. Measured on the gate, the
        # post's edge came through as 58x330 in a 586-tall region -- aspect
        # 0.18 and area 3.8%, inside every other bound -- and it dragged the
        # text rectangle from a flat band into a square, which read as 'aa'.
        if h > binary.shape[0] * max_height_frac:
            continue
        x0, y0 = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        pts.extend([(x0, y0), (x0 + w, y0), (x0, y0 + h), (x0 + w, y0 + h)])
    if len(pts) < 8:                          # fewer than two plausible marks
        return binary

    rect = cv2.minAreaRect(np.array(pts, dtype=np.float32))
    (cx, cy), (rw, rh), ang = rect
    if rw < rh:                               # keep the long side horizontal
        rw, rh = rh, rw
        ang += 90.0
    if rw < 8 or rh < 4:
        return binary
    m = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
    rot = cv2.warpAffine(binary, m, (binary.shape[1], binary.shape[0]),
                         flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                         borderValue=255)
    x0 = int(max(0, cx - rw / 2 - pad))
    x1 = int(min(rot.shape[1], cx + rw / 2 + pad))
    y0 = int(max(0, cy - rh / 2 - pad))
    y1 = int(min(rot.shape[0], cy + rh / 2 + pad))
    crop = rot[y0:y1, x0:x1]
    return crop if crop.size else binary


def _clean(text):
    return re.sub(r"[^A-Z]", "", (text or "").upper())


def _longest_run(text, target=TARGET):
    """Longest run of `target` appearing in order inside `text`.

    Delegates to the project's existing matcher: a second implementation of
    "does this spell AEROTHON" is a second definition of the answer.
    """
    return longest_in_sequence(text, target)


def read_with_tesseract(roi_bgr, board_mask=None):
    """(text, matched_letters) or (None, 0) if tesseract cannot be used."""
    if not tesseract_available():
        return None, 0
    img = lettering_binary(roi_bgr, board_mask=board_mask)
    if img is None:
        return None, 0
    # Three presentations, cheapest first, stopping at the first that reads.
    #
    #   raw         works when the board fills its own region (the rendered
    #               banner) and rectification would only crop off the margins
    #               tesseract needs
    #   rectified   needed when the word is a small skewed band inside a much
    #               larger region -- the arena's gate
    #   +180        a minAreaRect angle is 180-degree ambiguous, and nothing
    #               in the box says which way up the text is. Measured: the
    #               gate rectified upside down and read 'be'.
    candidates = [img]
    rect = rectify_lettering(img)
    if rect.shape != img.shape:
        candidates.append(rect)
        candidates.append(cv2.rotate(rect, cv2.ROTATE_180))

    best_text, best_run = "", 0
    for cand in candidates:
        if cand.shape[0] < 24:
            f = 32.0 / max(1, cand.shape[0])
            cand = cv2.resize(cand, None, fx=f, fy=f,
                              interpolation=cv2.INTER_CUBIC)
        ok, buf = cv2.imencode(".png", cand)
        if not ok:
            continue
        text, run = _run_tesseract(buf.tobytes())
        if run > best_run:
            best_text, best_run = text, run
        if best_run >= 5:
            break
    return best_text, best_run


def _run_tesseract(png_bytes):
    """One image through the engine. Returns (text, letters matched)."""
    best_text, best_run = "", 0
    # psm 8 = "treat the image as a single WORD", which is exactly what a
    # banner is. Measured on the rendered board:
    #
    #     psm 6  -> "HERUT RUM"
    #     psm 7  -> "HERUT RUM"      (single text LINE -- what was used first)
    #     psm 8  -> "AEROTHON"
    #     psm 13 -> "AEROTHON"
    #
    # The whitelist is deliberately NOT set: with psm 7 it turned the wrong
    # answer into a different wrong answer, and constraining the character set
    # stops the engine using letter shape to reject a bad segmentation.
    for psm in ("8", "13"):
        cmd = [TESSERACT, "stdin", "stdout", "--psm", psm, "--oem", "3"]
        try:
            out = subprocess.run(cmd, input=png_bytes,
                                 capture_output=True, timeout=4.0)
        except (OSError, subprocess.SubprocessError):
            return "", 0
        text = _clean(out.stdout.decode("utf-8", "ignore"))
        run = _longest_run(text)
        if run > best_run:
            best_text, best_run = text, run
        if best_run >= 5:
            break
    return best_text, best_run


def _word_template(word, cell=8):
    """The word drawn from the project's own 5x7 glyph shapes."""
    cols = []
    for ch in word:
        g = TEMPLATES.get(ch)
        if g is None:
            continue
        # The tables are "0"/"1" strings. Reading them as "#" produced a
        # BLANK template, and a constant template correlates perfectly with
        # anything -- every frame scored 1.000, including open sky. Caught
        # only because the negative fixture was in the probe.
        block = np.array([[255 if c == "1" else 0 for c in row] for row in g],
                         dtype=np.uint8)
        cols.append(cv2.resize(block, (5 * cell, 7 * cell),
                               interpolation=cv2.INTER_NEAREST))
        cols.append(np.zeros((7 * cell, cell), np.uint8))
    return np.hstack(cols[:-1]) if cols else None


def read_with_templates(roi_bgr, word=TARGET, min_score=0.28,
                        board_mask=None):
    """Dependency-free fallback: correlate the whole WORD, not blobs.

    Correlating the word as one shape is the point. Segmenting into blobs and
    classifying each is what produced eighteen 'letters' for an eight-letter
    banner; a single correlation cannot fragment.
    """
    img = lettering_binary(roi_bgr, board_mask=board_mask)
    if img is None:
        return None, 0.0
    scene = 255 - img                    # white text on black, like the template
    tpl = _word_template(word)
    if tpl is None:
        return None, 0.0

    best = 0.0
    sh, sw = scene.shape[:2]
    for frac in (0.95, 0.8, 0.65, 0.5, 0.38, 0.28):
        tw = max(16, int(sw * frac))
        th = max(8, int(tw * tpl.shape[0] / tpl.shape[1]))
        if th >= sh or tw >= sw:
            continue
        t = cv2.resize(tpl, (tw, th), interpolation=cv2.INTER_AREA)
        try:
            res = cv2.matchTemplate(scene, t, cv2.TM_CCOEFF_NORMED)
        except cv2.error:
            continue
        best = max(best, float(res.max()))
    return (word if best >= min_score else ""), best


def reads_banner(roi_bgr, word=TARGET, min_letters=5, min_score=0.28,
                 board_mask=None):
    """(is_banner, text, reader, score) for one green region.

    The single question the identity check should have been asking all along.
    """
    text, run = read_with_tesseract(roi_bgr, board_mask=board_mask)
    if text is not None:
        return run >= min_letters, text, "tesseract", float(run)
    text, score = read_with_templates(roi_bgr, word, min_score,
                                     board_mask=board_mask)
    return bool(text), text or "", "template", score
