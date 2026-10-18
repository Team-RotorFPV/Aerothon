"""Appearance matching on real QR codes, as the delivery decision uses it.

The organisers may print codes that carry nothing readable, so the pad is
chosen by its PATTERN against the start marker. The hard case is two codes
that differ by one character: they share every padding module, and the first
coarse-grid signature scored them up to 0.92 against a 0.82 bar
(logs/qr_appearance_study_20261014). These checks pin the module-grid
signature: the same code through perspective, rotation, blur and exposure
matches; a one-character-different code never does.
"""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
if not hasattr(cv2, "QRCodeEncoder"):
    pytest.skip("OpenCV without QRCodeEncoder", allow_module_level=True)

from perception_qr import qr_appearance as qa                     # noqa: E402
from perception_qr.qr_decode import QrDecoder                     # noqa: E402
from perception_qr.qr_match import APPEARANCE_ONLY, matches       # noqa: E402

CODES = ["A", "B", "C", "D", "TARGET A", "TARGET B", "PAD 4", "ZONE 2 PAD 1"]


def printed(text, side=400):
    """The code with a 3-inch-on-1-m white border, as the organisers lay it."""
    code = cv2.QRCodeEncoder.create().encode(text)
    code = cv2.resize(code, (side, side), interpolation=cv2.INTER_NEAREST)
    border = int(side * 0.0762)
    return cv2.copyMakeBorder(code, border, border, border, border,
                              cv2.BORDER_CONSTANT, value=255)


def photographed(sheet, px, rng, angle=0.0, keystone=0.0, blur=0.0,
                 gain=1.0, noise=0.0, turns=0):
    sheet = np.rot90(sheet, turns).copy()
    h, w = 720, 1280
    ground = (np.full((h, w), 90, np.uint8)
              + rng.integers(0, 30, (h, w), dtype=np.uint8))
    s = sheet.shape[0]
    src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
    a = np.deg2rad(angle)
    turn = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], float) * px / 2
    corners[:, 0] *= np.array([1 - keystone, 1 - keystone, 1 + keystone, 1 + keystone])
    dst = (corners @ turn.T + [w / 2, h / 2]).astype(np.float32)
    img = cv2.warpPerspective(sheet, cv2.getPerspectiveTransform(src, dst), (w, h),
                              dst=ground.copy(), borderMode=cv2.BORDER_TRANSPARENT)
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    img = np.clip(img.astype(float) * gain + rng.normal(0, noise, img.shape), 0, 255)
    return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)


def signed(img):
    decoded, located = QrDecoder().read(img)
    quad = decoded[0][1] if decoded else located
    assert quad is not None, "the test image should locate"
    return qa.signature(img, quad)


@pytest.fixture(scope="module")
def references():
    rng = np.random.default_rng(7)
    return {code: signed(photographed(printed(code), 200, rng, angle=10,
                                      keystone=0.05, blur=0.6, noise=3))
            for code in CODES}


def test_the_start_marker_reads_as_a_module_grid(references):
    for code, ref in references.items():
        assert ref is not None and ref.kind == "modules", code


def test_the_same_code_matches_from_the_sweep(references):
    rng = np.random.default_rng(11)
    for code in CODES:
        for px, turns in ((120, 1), (150, 2), (180, 3)):
            seen = signed(photographed(printed(code), px, rng,
                                       angle=rng.uniform(0, 360),
                                       keystone=0.08, blur=1.0, gain=0.8,
                                       noise=6, turns=turns))
            same, score = qa.same_marker(references[code], seen)
            assert same, f"{code} at {px}px scored {score}"


def test_a_one_character_different_code_never_matches(references):
    rng = np.random.default_rng(13)
    for ref_code in CODES:
        for code in CODES:
            if code == ref_code:
                continue
            seen = signed(photographed(printed(code), 150, rng,
                                       angle=rng.uniform(0, 360),
                                       keystone=0.06, blur=0.8, noise=5))
            same, score = qa.same_marker(references[ref_code], seen)
            assert not same, f"{code} matched {ref_code} at {score}"


def test_different_kinds_and_sizes_never_compare():
    a = qa.Signature("modules", 21, np.zeros((21, 21), np.uint8), np.ones((21, 21), bool))
    b = qa.Signature("modules", 25, np.zeros((25, 25), np.uint8), np.ones((25, 25), bool))
    c = qa.Signature("coarse", 21, np.zeros((21, 21), np.uint8), np.ones((21, 21), bool))
    assert qa.agreement(a, b) == 0.0
    assert qa.agreement(a, c) == 0.0


def test_the_appearance_only_target_matches_no_payload():
    assert not matches(APPEARANCE_ONLY, "A")
    assert not matches(APPEARANCE_ONLY, APPEARANCE_ONLY)
    assert not matches(APPEARANCE_ONLY, "START MARKER APPEARANCE")


def test_a_pad_too_small_to_size_is_read_at_the_start_markers_grid(references):
    """The registered re-read: the pad's own grid reading is withheld (as if
    the sweep saw it too small to size), so only re-reading it at the start
    marker's module count can compare it."""
    rng = np.random.default_rng(17)
    for code, other in (("B", "C"), ("TARGET A", "TARGET B")):
        for text, should in ((code, True), (other, False)):
            seen = signed(photographed(printed(text), 110, rng, angle=rng.uniform(0, 360),
                                       keystone=0.05, blur=0.9, noise=5))
            unsized = qa.Signature("coarse", qa.GRID, seen.bits[:qa.GRID, :qa.GRID],
                                   np.ones((qa.GRID, qa.GRID), bool), seen.grey)
            same, score = qa.same_marker(references[code], unsized)
            assert same is should, f"{text} against {code} scored {score}"
