#!/usr/bin/env python3
"""The banner's lettering is READ, as a confirming check only.

WHAT THIS ADDS

    The structural gates verify letter-like STRUCTURE and never letter
    IDENTITY: "AEROTHON" and "XQZFBRT" are indistinguishable to them. Seed
    1001 fails the return lap on

        board aspect 0.73 outside 1.2-8.0

    -- the board seen from the delivery-zone side presents at an aspect the
    gate refuses, while the lettering is legible and never consulted.

    So the blobs gate 3 already segments are read, and may RESCUE a rejection.
    Never veto an acceptance: a check that can only turn NO into YES cannot
    regress the outbound identification that works on every arena today.

WHY THE TEMPLATES ARE NOT IMPORTED FROM THE WORLD GENERATOR

    scripts/materialize_world.py owns the FONT that draws the simulated
    banner. Grading the reader with its own generator is the trap that made
    the first synthetic banner fixtures worthless -- drawn by the assumption
    the detector was making, unable to fail. perception_banner.glyphs writes
    ordinary 5x7 capitals instead, and the cross-check below asserts the two
    tables are genuinely independent.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_banner_text.py -v
"""

import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_perception",
                                "perception_banner"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from perception_banner.glyphs import (      # noqa: E402
    TARGET, TEMPLATES, classify_glyph, longest_in_sequence, normalise_glyph,
    read_glyphs, reads_as_target,
)


def render(rows, scale=6, noise=0.0, rng=None):
    """A letter blob as the camera would deliver it: bigger than 5x7."""
    a = np.array([[1 if c == "1" else 0 for c in r] for r in rows],
                 dtype=np.uint8)
    big = np.kron(a, np.ones((scale, scale), dtype=np.uint8))
    if noise > 0:
        rng = rng or np.random.default_rng(0)
        flip = rng.random(big.shape) < noise
        big = np.where(flip, 1 - big, big).astype(np.uint8)
    return big * 255


def word_patches(word, **kw):
    return [render(TEMPLATES[c], **kw) for c in word]


class GlyphTests(unittest.TestCase):

    def test_every_target_letter_has_a_template(self):
        for ch in set(TARGET):
            self.assertIn(ch, TEMPLATES, f"no template for {ch}")

    def test_each_letter_classifies_as_itself(self):
        for ch, rows in TEMPLATES.items():
            got, score = classify_glyph(render(rows))
            self.assertEqual(got, ch, f"{ch} read as {got} ({score:.2f})")

    def test_resampling_survives_an_odd_size(self):
        """Real blobs are never a clean multiple of 5x7.

        Resized, not cropped: cutting a letter in half is mutilation, and a
        reader that still called it an R would be guessing.
        """
        import cv2
        odd = cv2.resize(render(TEMPLATES["R"]), (23, 41),
                         interpolation=cv2.INTER_NEAREST)
        got, _ = classify_glyph(odd)
        self.assertEqual(got, "R")

    def test_every_letter_survives_an_odd_size(self):
        import cv2
        for ch, rows in TEMPLATES.items():
            odd = cv2.resize(render(rows), (19, 27),
                             interpolation=cv2.INTER_NEAREST)
            got, _ = classify_glyph(odd)
            self.assertEqual(got, ch, f"{ch} misread as {got} when resized")

    def test_an_empty_patch_is_not_a_letter(self):
        self.assertEqual(normalise_glyph(np.zeros((0, 0), np.uint8)).sum(), 0)

    def test_a_blank_blob_does_not_confidently_read(self):
        text = read_glyphs([np.zeros((30, 20), np.uint8)])
        self.assertEqual(text, "?")


class ReadWordTests(unittest.TestCase):

    def test_the_word_reads(self):
        ok, text, n = reads_as_target(word_patches(TARGET))
        self.assertTrue(ok, f"read {text!r} ({n} letters)")
        self.assertEqual(text, TARGET)

    def test_it_survives_two_unreadable_letters(self):
        """Blur and occlusion at range; five of eight is the bar."""
        patches = word_patches(TARGET)
        patches[2] = np.zeros_like(patches[2])
        patches[5] = np.zeros_like(patches[5])
        ok, text, n = reads_as_target(patches)
        self.assertTrue(ok, f"read {text!r} ({n})")

    def test_it_survives_speckle_noise(self):
        rng = np.random.default_rng(3)
        ok, text, n = reads_as_target(word_patches(TARGET, noise=0.04, rng=rng))
        self.assertTrue(ok, f"read {text!r} ({n})")

    def test_a_DECOY_does_not_read_as_the_banner(self):
        """The check that matters. The simulated decoys are green and
        banner-shaped; if random strokes confirmed as text the rescue would
        hand every tarp an acceptance."""
        rng = np.random.default_rng(11)
        blobs = [(rng.random((30, 20)) < 0.5).astype(np.uint8) * 255
                 for _ in range(8)]
        ok, text, n = reads_as_target(blobs)
        self.assertFalse(ok, f"random strokes read as the banner: {text!r} ({n})")

    def test_a_blank_board_does_not_read(self):
        ok, _, _ = reads_as_target([np.zeros((30, 20), np.uint8)] * 8)
        self.assertFalse(ok)

    def test_too_few_blobs_cannot_confirm(self):
        ok, _, _ = reads_as_target(word_patches("AER"))
        self.assertFalse(ok)

    def test_a_DIFFERENT_word_of_the_same_letters_does_not_confirm(self):
        """Order is the signal, not letter frequency."""
        ok, text, n = reads_as_target(word_patches("NOHTOREA"))
        self.assertFalse(ok, f"{text!r} confirmed with {n} in sequence")


class SequenceTests(unittest.TestCase):

    def test_exact(self):
        self.assertEqual(longest_in_sequence("AEROTHON"), 8)

    def test_letters_out_of_order_score_low(self):
        self.assertLess(longest_in_sequence("NOHTOREA"), 5)

    def test_an_unreadable_glyph_costs_one_letter_not_the_match(self):
        self.assertGreaterEqual(longest_in_sequence("AER?THON"), 7)

    def test_embedded_in_other_text(self):
        self.assertEqual(longest_in_sequence("XXAEROTHONXX"), 8)


class IndependenceFromTheGeneratorTests(unittest.TestCase):
    """A reader graded by its own generator proves nothing."""

    def test_the_reader_does_not_import_the_world_generator(self):
        """Checks IMPORTS, not prose -- the docstring names the generator
        deliberately, to explain why it is not imported."""
        import ast
        import perception_banner.glyphs as g
        tree = ast.parse(open(g.__file__).read())
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported += [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                imported.append(node.module or "")
        for name in imported:
            self.assertNotIn("materialize_world", name)
            self.assertNotIn("scripts", name)

    def test_the_reader_still_reads_the_GENERATOR_glyphs(self):
        """Independent tables, but they must agree on what a letter looks
        like -- otherwise the reader cannot read the simulated banner."""
        from materialize_world import FONT
        for ch in set(TARGET):
            got, score = classify_glyph(render(FONT[ch]))
            self.assertEqual(got, ch,
                             f"generator's {ch} read as {got} ({score:.2f})")


if __name__ == "__main__":
    unittest.main(verbosity=2)
