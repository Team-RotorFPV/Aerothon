"""The organisers' clarifications, as checks.

Banners carry no text, their shades arrive as HEX before the flight, they may
be ground-mounted or suspended, and the QR must be matched by appearance
because it is unsettled whether it encodes anything. Each check here fails on
the behaviour that preceded the clarification, so a pass means the change is
actually in place rather than merely written down.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))

from perception_banner import colours                                # noqa: E402
from perception_qr import qr_appearance                              # noqa: E402


# ---- the supplied colours ----------------------------------------------- #

def test_a_hex_colour_becomes_a_hue_band_around_itself():
    hue, sat, val = colours.hsv_of('#07A03D')
    assert 50 <= hue <= 80, 'that green should land in the green hues'
    (lo, hi), = colours.ranges('#07A03D')
    assert lo[0] < hue < hi[0]
    # Saturation and value are left wide on purpose: wear, shade and exposure
    # move them, and pinning them to the supplied colour rejects the banner.
    assert lo[1] <= colours.DEFAULT_S_FLOOR and hi[1] == 255
    assert lo[2] <= colours.DEFAULT_V_FLOOR and hi[2] == 255


def test_a_red_hue_window_wraps_the_circle_into_two_bands():
    bands = colours.ranges('#C4161C')
    assert len(bands) == 2, 'red sits at 0/180 and needs both sides'
    assert any(int(lo[0]) == 0 for lo, _ in bands)
    assert any(int(hi[0]) == 179 for _, hi in bands)


@pytest.mark.parametrize('bad', ['', 'green', '#12345', '#GGGGGG', None, 42,
                                 '#1234567'])
def test_anything_that_is_not_a_hex_colour_is_refused(bad):
    with pytest.raises(ValueError):
        colours.parse_hex(bad)


def test_an_unknown_colour_name_is_refused_rather_than_dropped():
    # A colour the aircraft silently ignores is worse than one it rejects:
    # the operator believes it was taken.
    with pytest.raises(ValueError):
        colours.validate({'green': '#07A03D', 'turquoise': '#00FFEE'})
    assert colours.validate({'green': '#07a03d'}) == {'green': '#07A03D'}


def test_the_supplied_green_matches_that_green_and_not_the_orange():
    import cv2
    green = np.full((8, 8, 3), colours.parse_hex('#07A03D')[::-1], np.uint8)
    orange = np.full((8, 8, 3), colours.parse_hex('#EB5A0A')[::-1], np.uint8)
    bands = colours.ranges('#07A03D')
    assert colours.mask(cv2.cvtColor(green, cv2.COLOR_BGR2HSV), bands).all()
    assert not colours.mask(cv2.cvtColor(orange, cv2.COLOR_BGR2HSV), bands).any()

def test_banner_masks_follow_changed_hex_and_never_use_an_unsupplied_default():
    from perception_banner.banner_node import BannerNode
    node = object.__new__(BannerNode)
    node._colours = {}
    original = np.full((32,32,3), colours.parse_hex('#07A03D')[::-1], np.uint8)
    assert not node.colour_mask(original).any()
    node._colours = {'green':'#245FBC','orange':'#D68C21','red':'#BB235F'}
    entered = np.full((32,32,3), colours.parse_hex(node._colours['green'])[::-1], np.uint8)
    assert node.colour_mask(entered).all()
    assert not node.colour_mask(original).any()

def test_restricted_zone_mask_follows_entered_hex_and_has_no_default():
    from perception_redzone.redzone_node import RedZoneNode
    node = object.__new__(RedZoneNode)
    node._g = lambda key: 60
    node._drop_narrow = lambda mask: mask
    node._red_hex = ''
    old = np.full((32,32,3), colours.parse_hex('#C4161C')[::-1], np.uint8)
    assert not node.red_mask(old).any()
    node._red_hex = '#245FBC'
    node._supplied = {'red':'#245FBC','green':'#07A03D','orange':'#D68C21'}
    entered = np.full((32,32,3), colours.parse_hex(node._red_hex)[::-1], np.uint8)
    assert node.red_mask(entered).any()
    assert not node.red_mask(old).any()


# ---- appearance-first QR ------------------------------------------------- #

def marker(seed, side=160):
    """A printed-looking marker: a coarse random pattern with a quiet zone."""
    rng = np.random.default_rng(seed)
    cells = rng.integers(0, 2, size=(10, 10), dtype=np.uint8) * 255
    import cv2
    body = cv2.resize(cells, (side-40, side-40), interpolation=cv2.INTER_NEAREST)
    frame = np.full((side, side), 255, np.uint8)
    frame[20:side-20, 20:side-20] = body
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


QUAD = np.array([[20, 20], [139, 20], [139, 139], [20, 139]], dtype=np.float32)


def test_the_same_marker_matches_itself_through_a_turn_and_a_rescale():
    import cv2
    first = qr_appearance.signature(marker(7), QUAD)
    bigger = cv2.resize(marker(7), (320, 320), interpolation=cv2.INTER_LINEAR)
    turned = cv2.rotate(bigger, cv2.ROTATE_90_CLOCKWISE)
    second = qr_appearance.signature(
        turned, np.array([[40, 40], [279, 40], [279, 279], [40, 279]], np.float32))
    same, score = qr_appearance.same_marker(first, second)
    assert same, f'the same code at another size and turn scored {score}'


def test_a_different_marker_does_not_match():
    first = qr_appearance.signature(marker(7), QUAD)
    other = qr_appearance.signature(marker(99), QUAD)
    same, score = qr_appearance.same_marker(first, other)
    assert not same, f'two different codes scored {score}'
    assert score < qr_appearance.DEFAULT_THRESHOLD


def test_a_marker_too_small_to_rectify_yields_no_signature():
    tiny = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
    assert qr_appearance.signature(marker(7), tiny) is None
    # ...and an absent signature can never match anything.
    assert qr_appearance.agreement(None, qr_appearance.signature(marker(7), QUAD)) == 0.0


# ---- the arena the organisers describe ---------------------------------- #

def test_the_banner_is_built_with_no_lettering_but_keeps_its_white_border():
    import materialize_world as MW
    panel = MW.banner_geometry(0, 2.805)
    assert 'banner_front_frame_top' in panel, 'the 3-inch border is the identity'
    assert 'banner_front_0"' not in panel, 'the banners carry no text'
    lettered = MW.banner_geometry(0, 2.805, lettered=True)
    assert 'banner_front_0"' in lettered, 'the old panel must still be buildable'


def test_a_ground_banner_lies_flat_with_a_border_and_no_collision():
    import materialize_world as MW
    laid = MW.ground_banner_geometry(0)
    assert 'ground_banner_panel' in laid and 'ground_banner_border_n' in laid
    # A sheet on the ground is not something the aircraft can hit; a collision
    # box here would put a phantom obstacle across the corridor mouth.
    assert 'collision' not in laid


def test_a_supplied_hex_paints_the_banner_it_was_supplied_for():
    import world_spec as W
    ambient, diffuse = W.print_colour('#07A03D')
    assert diffuse == pytest.approx((0.0275, 0.6275, 0.2392), abs=1e-3)
    assert all(a < d for a, d in zip(ambient, diffuse) if d > 0)


@pytest.mark.parametrize('placement', ['suspended', 'ground', 'both'])
def test_every_banner_placement_the_organisers_allow_validates(placement):
    import world_spec as W
    spec = W.normalise({'banner': {'placement': placement}})
    assert W.validate(spec)[0] == []


def test_an_unknown_placement_or_colour_is_refused():
    import world_spec as W
    errors, _ = W.validate(W.normalise({'banner': {'placement': 'sideways'}}))
    assert any('placement' in e for e in errors)
    errors, _ = W.validate(W.normalise({'banner': {'colours': {'green': 'teal'}}}))
    assert any('not #RRGGBB' in e for e in errors)


def test_the_flown_worlds_carry_no_fake_green_banners():
    """The organisers' arena has nothing green in it but the banners.

    One world keeps the decoys on purpose, so "reject a plain coloured sheet"
    is still measured against something rather than assumed.
    """
    planted = {p.stem: len(json.loads(p.read_text()).get('decoys') or [])
               for p in sorted((ROOT/'sim/worlds').glob('*.json'))}
    assert planted.pop('banner_decoys') > 0, 'the negative control needs decoys'
    assert not {k: v for k, v in planted.items() if v}, 'flown worlds must have none'


def test_the_three_placement_cases_exist_and_differ_only_in_placement():
    specs = {name: json.loads((ROOT/f'sim/worlds/{name}.json').read_text())
             for name in ('banner_ground', 'banner_suspended', 'banner_both')}
    assert {s['banner']['placement'] for s in specs.values()} == {
        'ground', 'suspended', 'both'}
    for spec in specs.values():
        assert spec['banner']['lettered'] is False
        assert spec['banner']['colours']['green'].startswith('#')
        assert spec['decoys'] == []


# ---- supplied shades never take each other -------------------------------- #

SUPPLIED = {'green': '#07A03D', 'orange': '#EB5A0A', 'red': '#C4161C'}


def patch_of(hex_colour):
    import cv2
    bgr = np.full((8, 8, 3), colours.parse_hex(hex_colour)[::-1], np.uint8)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


def test_the_red_zone_band_does_not_take_the_orange_banner_or_floor():
    # Rendered regression: at +-25 the red band covered orange (12 hue steps
    # away), the return lane's orange floor read as a red zone at the return
    # mouth, and the gate crossing was refused there.
    red = colours.separated_ranges('red', SUPPLIED)
    assert colours.mask(patch_of('#C4161C'), red).all()
    assert not colours.mask(patch_of('#EB5A0A'), red).any()
    assert colours.mask(patch_of('#EB5A0A'), colours.ranges('#C4161C')).any(), \
        'the unseparated band is the defect this guards'


def test_the_orange_band_does_not_take_the_red_zone():
    orange = colours.separated_ranges('orange', SUPPLIED)
    assert colours.mask(patch_of('#EB5A0A'), orange).all()
    assert not colours.mask(patch_of('#C4161C'), orange).any()


def test_green_far_from_both_keeps_its_full_tolerance():
    assert [tuple(map(int, lo)) + tuple(map(int, hi)) for lo, hi in
            colours.separated_ranges('green', SUPPLIED)] == \
           [tuple(map(int, lo)) + tuple(map(int, hi)) for lo, hi in
            colours.ranges('#07A03D')]


def test_the_corridor_floors_are_unmarked_so_only_the_banners_carry_colour(tmp_path):
    # Rendered regression: the outbound lane's green floor between two white
    # walls is a framed green panel to a text-free detector, and the aircraft
    # squared up on it between the corridors. The clarified arena marks lanes
    # with banners only.
    import subprocess
    import materialize_world as MW
    out = tmp_path / 'world.sdf'
    proc = subprocess.run(
        [sys.executable, str(ROOT/'scripts/materialize_world.py'),
         '--source', str(ROOT/'src/aerothon_sim/sim_gazebo/worlds/mission2.sdf'),
         '--assets', str(ROOT/'src/aerothon_sim/sim_gazebo/materials'),
         '--output', str(out),
         '--world-spec', str(ROOT/'sim/worlds/banner_ground.json')],
        capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr[-2000:]
    world = out.read_text()
    assert MW.GREEN_FLOOR not in world and MW.ORANGE_FLOOR not in world
    assert world.count(MW.NEUTRAL_FLOOR) >= 2


def test_nothing_at_the_gate_is_painted_banner_green_except_the_banner():
    # Rendered regression: the gate posts were green, touched the laid sheet's
    # corners, and merged with it into one region that failed the border test
    # -- so the GROUND glance never identified the laid banner.
    import re
    sdf = (ROOT/'src/aerothon_sim/sim_gazebo/worlds/mission2.sdf').read_text()
    posts = re.findall(r'<visual name="(?:left|right)_post">.*?<diffuse>([^<]*)</diffuse>', sdf)
    assert len(posts) == 4
    for rgb in posts:
        r, g, b, _ = map(float, rgb.split())
        assert abs(r - g) < 0.1 and abs(g - b) < 0.1, f'post is coloured: {rgb}'


def test_a_laid_banner_lies_in_front_of_each_mouth_not_under_the_floor(tmp_path):
    # Rendered regression: centred on the gate line the sheet lay half under
    # the lane's floor slab, showed no far border, and was never identified.
    import re
    import subprocess
    import materialize_world as MW
    out = tmp_path / 'world.sdf'
    proc = subprocess.run(
        [sys.executable, str(ROOT/'scripts/materialize_world.py'),
         '--source', str(ROOT/'src/aerothon_sim/sim_gazebo/worlds/mission2.sdf'),
         '--assets', str(ROOT/'src/aerothon_sim/sim_gazebo/materials'),
         '--output', str(out),
         '--world-spec', str(ROOT/'sim/worlds/banner_ground.json')],
        capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr[-2000:]
    world = out.read_text()
    assert MW.LAID_BANNER_TOKEN not in world
    for lane in ('corridor_outbound', 'corridor_return'):
        body = re.search(r'<model name="%s">(.*?)</model>' % lane, world, re.S).group(1)
        x = float(re.search(r'<visual name="ground_banner_panel"><pose>([-\d.]+)', body).group(1))
        assert x + 1.15 / 2 < 0.0, f'{lane}: sheet reaches past the mouth (x={x})'
