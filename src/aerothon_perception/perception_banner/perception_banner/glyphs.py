#!/usr/bin/env python3
"""Read the lettering on the banner, as a CONFIRMING check only.

WHAT THIS IS FOR

    perception_banner validates the banner structurally: white content inside
    green, several separate white blobs in a horizontal band, then an aspect
    gate on the derived board. That verifies letter-like STRUCTURE and never
    letter IDENTITY -- "AEROTHON" and "XQZFBRT" are indistinguishable to it.

    Seed 1001 fails on the return lap for exactly that reason:

        board aspect 0.73 outside 1.2-8.0

    Seen from the delivery-zone side the board presents at an aspect the gate
    rejects, and the letters themselves are never consulted. The outbound
    identification of the same banner succeeds.

    So this reads the blobs that gate 3 has ALREADY segmented, and is allowed
    to RESCUE a rejection -- never to veto an acceptance. The worst case is
    therefore exactly today's behaviour.

WHY THE TEMPLATES ARE WRITTEN OUT HERE AND NOT IMPORTED

    scripts/materialize_world.py owns a FONT table that draws the simulated
    banner. Matching against that table would be a detector graded by its own
    generator -- the same trap that made the first synthetic banner fixtures
    worthless: they were drawn by the assumption the detector was making, and
    could not fail.

    These templates are ordinary 5x7 capitals. Most sans-serif capitals
    downsample to approximately this, which is the point: the check has to
    survive a real printed banner, not just the one the simulator draws.

NOT AN OCR ENGINE
    Fixed word, fixed alphabet, no dependency, roughly a millisecond. It
    answers "do these blobs spell AEROTHON?" and nothing else.
"""

import numpy as np

TARGET = "AEROTHON"

# 5x7 capitals. Independent of the world generator's table on purpose.
TEMPLATES = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "N": ["10001", "11001", "11001", "10101", "10011", "10011", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
}

GW, GH = 5, 7


def _as_array(rows):
    return np.array([[1 if c == "1" else 0 for c in r] for r in rows],
                    dtype=np.uint8)


_TEMPLATE_ARRAYS = {c: _as_array(r) for c, r in TEMPLATES.items()}


def normalise_glyph(patch):
    """A binary blob resampled to 5x7, the resolution the templates live at.

    Nearest-neighbour on purpose: these are strokes, and interpolating them
    produces grey that then needs a second threshold to undo.
    """
    if patch is None or patch.size == 0:
        return np.zeros((GH, GW), dtype=np.uint8)
    h, w = patch.shape[:2]
    ys = (np.arange(GH) * h // GH).clip(0, h - 1)
    xs = (np.arange(GW) * w // GW).clip(0, w - 1)
    cell_h = max(1, h // GH)
    cell_w = max(1, w // GW)
    out = np.zeros((GH, GW), dtype=np.uint8)
    for r, y in enumerate(ys):
        for c, x in enumerate(xs):
            block = patch[y:y + cell_h, x:x + cell_w]
            if block.size and float(np.count_nonzero(block)) / block.size > 0.4:
                out[r, c] = 1
    return out


def classify_glyph(patch):
    """(character, score in 0..1) for the best matching template."""
    g = normalise_glyph(patch)
    best, best_score = "?", 0.0
    for char, tmpl in _TEMPLATE_ARRAYS.items():
        score = float(np.count_nonzero(g == tmpl)) / float(GW * GH)
        if score > best_score:
            best, best_score = char, score
    return best, best_score


def read_glyphs(patches, min_glyph_score=0.72):
    """Left-to-right string from already-segmented letter patches.

    `patches` must be ordered left to right. A glyph the templates cannot
    match confidently becomes '?' rather than a guess, so a smear does not
    silently become an 'O'.
    """
    out = []
    for p in patches:
        char, score = classify_glyph(p)
        out.append(char if score >= min_glyph_score else "?")
    return "".join(out)


def longest_in_sequence(text, target=TARGET):
    """Longest run of `target`'s letters appearing IN ORDER within `text`.

    Subsequence rather than substring: a missed or smeared letter costs one
    from the count instead of breaking the match completely, which is what
    happens in practice at range and at an angle.
    """
    best = 0
    for start in range(len(text)):
        ti, count = 0, 0
        for ch in text[start:]:
            if ti < len(target) and ch == target[ti]:
                count += 1
                ti += 1
            elif ti < len(target) and ch == "?":
                ti += 1          # tolerate an unreadable glyph in place
        best = max(best, count)
    return best


def reads_as_target(patches, min_letters=5, min_glyph_score=0.72):
    """(confirmed, text, matched_count) for a run of letter patches.

    `min_letters` of TARGET's 8 must appear in sequence. Five was chosen so a
    banner with two letters lost to blur or occlusion still confirms, while a
    decoy with a couple of coincidental strokes does not.
    """
    text = read_glyphs(patches, min_glyph_score)
    matched = longest_in_sequence(text)
    return matched >= min_letters, text, matched
