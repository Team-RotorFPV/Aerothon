#!/usr/bin/env python3
"""The start QR and the pad QR need not carry the same string.

The rulebook promises "delivery location information" at the start and "the
QR code corresponding to" it in the zone, nothing more (4.2.4). Every format
the organisers could plausibly print is pinned here, with the ones that must
NOT match -- a wrong pad costs more than none.

    python3 -m pytest sim/test_qr_match.py -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src", "aerothon_perception", "perception_qr"))

from perception_qr.qr_match import matches  # noqa: E402

MATCH = (
    ("AEROTHON2026:M2:TARGET_B", "AEROTHON2026:M2:TARGET_B"),   # the simulator
    ("Deliver to Target B", "TARGET-B"),
    ("Deliver to Target B", "B"),
    ("AEROTHON2026:M2:TARGET_B", "B"),
    ("TGT-A17", "A17"),
    ("Target 01", "TARGET 1"),
    ("Target A", "target a"),
    ("Deliver to location C, north field", "Location C"),
)
NO_MATCH = (
    ("AEROTHON2026:M2:TARGET_B", "AEROTHON2026:M2:TARGET_A"),
    ("B", "B2"),                       # the pad says more than the start
    ("B", "TARGET B NORTH"),
    ("TGT-A17", "A18"),
    ("Location C", "Location D"),
    ("Deliver to Target B", "TARGET"),  # nothing left to identify
    ("", "B"),
    ("B", ""),
)


class QrMatchTests(unittest.TestCase):

    def test_every_plausible_format_matches_its_pad(self):
        for start, pad in MATCH:
            with self.subTest(start=start, pad=pad):
                self.assertTrue(matches(start, pad))

    def test_no_other_pad_matches(self):
        for start, pad in NO_MATCH:
            with self.subTest(start=start, pad=pad):
                self.assertFalse(matches(start, pad))

    def test_the_simulated_pads_are_told_apart(self):
        pads = [f"AEROTHON2026:M2:TARGET_{c}" for c in "ABCDE"]
        for want in pads:
            self.assertEqual([p for p in pads if matches(want, p)], [want])


if __name__ == "__main__":
    unittest.main()
