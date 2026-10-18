"""Ground, suspended or both: the gate aligns to whichever banner is there.

The organisers may hang the banner, lay it flat in front of the corridor, or
do both. These pin the two halves of handling that: the headless arena builds
what each placement actually puts in the world, and the mission tree pays for
a full ground-banner alignment only after a glance has seen one.

    python3 -m pytest sim/test_banner_placement_alignment.py -v
"""
import os
import sys
from unittest.mock import MagicMock

import py_trees
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))

import world_spec as W                                      # noqa: E402
from headless_world import Arena                            # noqa: E402
from mission_bt.mission_tree import (                       # noqa: E402
    AlignToBanner, BannerPlacement, GlanceForBanner, SetCameraPose,
    UntilIdentified, align_to_every_banner)

S = py_trees.common.Status


# ---- the arena each placement builds ------------------------------------ #

def arena(placement):
    return Arena(W.load(os.path.join(HERE, "worlds", f"banner_{placement}.json")), "b")


@pytest.mark.parametrize("placement,hung,laid", [
    ("suspended", True, False), ("ground", False, True), ("both", True, True)])
def test_each_placement_builds_exactly_its_banners(placement, hung, laid):
    a = arena(placement)
    boards = [b for b in a.boxes if b[3] == "board"]
    assert len(boards) == (2 if hung else 0)
    for gate in a.banners:
        assert gate["hung"] is hung
        assert (gate["laid"] is not None) is laid


def test_a_laid_banner_is_never_something_the_aircraft_can_hit():
    # A sheet on the ground has no collision; a box there would be a phantom
    # obstacle across the corridor mouth (materialize_world says the same).
    a = arena("ground")
    assert not [b for b in a.boxes if b[3] == "board"]
    assert all(len(gate["laid"]) == 4 for gate in a.banners)


def laid_view(eye_back_m, alt_m, pitch_deg):
    """What the headless camera makes of the outbound laid sheet."""
    import math
    import types
    from headless_world import Camera, HeadlessWorld
    gate = arena("ground").banners[0]
    f = gate["facing"]
    eye = (gate["xy"][0] - eye_back_m * math.cos(f),
           gate["xy"][1] - eye_back_m * math.sin(f), alt_m)
    world = types.SimpleNamespace(camera=Camera(1.0472), yaw=f)
    world._ground_green_view = lambda *a: HeadlessWorld._ground_green_view(world, *a)
    return HeadlessWorld._laid_view(world, gate["laid"], eye, math.radians(pitch_deg))


def test_a_laid_banner_ahead_is_wholly_in_frame_from_the_hung_view():
    # Regression: a signed polygon area made the in-frame fraction -1.0, so
    # a sheet squarely in view was never identified and the ground-only
    # world swept a full turn seeing nothing.
    from headless_world import BANNER_MAX_GROUND_INCIDENCE, BANNER_MIN_BOARD_PX
    bbox, _, inc, px_h, frac_in, dist = laid_view(7.0, 3.0, 20.0)
    assert frac_in == pytest.approx(1.0, abs=0.01)
    assert inc <= BANNER_MAX_GROUND_INCIDENCE and px_h >= BANNER_MIN_BOARD_PX


def test_a_laid_banner_foreshortens_past_the_detectors_aspect_gate_from_afar():
    # The real detector refuses a strip wider than max_aspect; from 7 m out
    # in the hung view a laid sheet is that strip, while the 45 deg ground
    # view from 5 m sees it squarely enough to identify.
    from headless_world import BANNER_MAX_ASPECT
    far, *_ = laid_view(7.0, 3.0, 20.0)
    near, *_ = laid_view(5.0, 3.0, 45.0)
    assert far[2] > BANNER_MAX_ASPECT * far[3]
    assert near[2] <= BANNER_MAX_ASPECT * near[3]


def test_a_laid_banner_too_far_out_is_too_grazing_to_identify():
    from headless_world import BANNER_MAX_GROUND_INCIDENCE
    _, _, inc, _, _, _ = laid_view(14.0, 3.0, 20.0)
    assert inc > BANNER_MAX_GROUND_INCIDENCE


# ---- the glance --------------------------------------------------------- #

class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def glance(hits):
    """Tick a glance over a 3 s dwell, 10 ticks per second."""
    mav, clock = MagicMock(), Clock()
    seen = iter(hits)
    mav.banner_identified.side_effect = lambda: next(seen, False)
    leaf = GlanceForBanner("g", mav, clock=clock, dwell_s=3.0)
    leaf.setup_with_descendants()
    status = None
    for _ in range(100):
        leaf.tick_once()
        status = leaf.status
        if status != S.RUNNING:
            return status
        clock.t += 0.1
    return status


def test_a_glance_that_sees_a_banner_succeeds():
    assert glance([True] * 40) == S.SUCCESS


def test_a_glance_at_bare_ground_fails_after_the_dwell_not_a_sweep():
    assert glance([False] * 40) == S.FAILURE


def test_one_stray_frame_is_not_a_banner():
    assert glance([True] + [False] * 39) == S.FAILURE


# ---- the budget on a view that sees nothing ----------------------------- #

class Forever(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("forever")
        self.stopped = False

    def update(self):
        return S.RUNNING

    def terminate(self, new_status):
        self.stopped = self.stopped or new_status == S.INVALID


def budgeted(identified_at=None, seconds=200):
    mav, clock, child = MagicMock(), Clock(), Forever()
    mav.banner_identified.side_effect = lambda: (
        identified_at is not None and clock.t >= identified_at)
    node = UntilIdentified("budget", child, mav, clock=clock, budget_s=90.0)
    node.setup_with_descendants()
    for _ in range(seconds * 10):
        node.tick_once()
        if node.status != S.RUNNING:
            break
        clock.t += 0.1
    return node.status, clock.t, child.stopped


def test_a_view_that_identifies_nothing_is_given_up_at_the_budget():
    status, t, stopped = budgeted()
    assert status == S.FAILURE and 90.0 <= t <= 91.0
    assert stopped, "the abandoned search must be stopped, not left running"


class Wanderer(py_trees.behaviour.Behaviour):
    """A search that walks the aircraft onto the gate and never finds it."""
    def __init__(self, mav):
        super().__init__("wander")
        self.mav = mav

    def update(self):
        self.mav.goto(6.3, -3.4, 3.0, 0.0)
        return S.RUNNING


def test_a_given_up_view_hands_over_from_where_it_started_going_over_the_top():
    mav, clock = MagicMock(), Clock()
    state = {"p": (0.4, -3.2, 3.0), "yaw": 0.1}
    mav.pos.side_effect = lambda: state["p"]
    mav.yaw.side_effect = lambda: state["yaw"]
    mav.banner_identified.return_value = False
    flown = []

    def goto(x, y, z, yaw):          # a perfect autopilot: arrive at once
        flown.append((x, y, z))
        state["p"] = (x, y, z)
    mav.goto.side_effect = goto
    node = UntilIdentified("budget", Wanderer(mav), mav, clock=clock, budget_s=90.0)
    node.setup_with_descendants()
    for _ in range(2000):
        node.tick_once()
        if node.status != S.RUNNING:
            break
        clock.t += 0.1
    assert node.status == S.FAILURE
    assert state["p"] == pytest.approx((0.4, -3.2, 3.0))
    back = flown[flown.index((6.3, -3.4, 3.0), -1) + 1:] if False else         [f for f in flown if f != (6.3, -3.4, 3.0)]
    # Up over the posts first, across at that height, then down.
    assert back[0] == pytest.approx((6.3, -3.4, 5.0))
    assert back[-1] == pytest.approx((0.4, -3.2, 3.0))
    assert all(z >= 5.0 for _, _, z in back[:-1])


def test_a_view_that_has_identified_anything_keeps_its_full_recovery():
    status, t, _ = budgeted(identified_at=30.0)
    assert status == S.RUNNING and t > 150.0


# ---- the tree ----------------------------------------------------------- #

def structure(label="Outbound"):
    nodes, = align_to_every_banner(MagicMock(), Clock(), {}, label)
    return nodes


def test_the_gate_tries_hung_then_ground_then_hung_without_a_budget():
    root = structure()
    assert isinstance(root, py_trees.composites.Selector)
    hung, ground, full = root.children
    assert hung.name == "OutboundHungBannerPass"
    assert ground.name == "OutboundGroundOnlyPass"
    assert full.name == "OutboundHungBannerFullPass"
    assert [c.pose for c in ground.children if isinstance(c, SetCameraPose)] == [
        "GROUND", "BANNER"]
    assert any(isinstance(c, AlignToBanner) for c in ground.children)
    # The first hung pass is budgeted; the last resort is not.
    assert any(isinstance(c, UntilIdentified) for c in hung.children)
    assert any(isinstance(c, AlignToBanner) for c in full.children)


def test_every_ground_pass_searches_and_recovers_looking_down():
    # Recovery used to level the camera (FORWARD) or return it to BANNER,
    # and from either a sheet lying on the ground is out of frame.
    hung, ground, full = structure().children
    optional = next(c for c in hung.children
                    if isinstance(c, py_trees.decorators.FailureIsSuccess))
    for leaf in (optional.decorated.children[-1],
                 next(c for c in ground.children if isinstance(c, AlignToBanner))):
        assert leaf.search_camera_pose == "GROUND"
        assert leaf.low_camera_pose == "GROUND"
    budget = next(c for c in hung.children if isinstance(c, UntilIdentified))
    assert budget.decorated.search_camera_pose == "BANNER"
    assert next(c for c in full.children
                if isinstance(c, AlignToBanner)).search_camera_pose == "BANNER"


def test_the_second_banner_is_aligned_to_only_after_a_glance_saw_it():
    hung, _, _ = structure().children
    optional = next(c for c in hung.children
                    if isinstance(c, py_trees.decorators.FailureIsSuccess))
    kinds = [type(c) for c in optional.decorated.children]
    assert kinds == [SetCameraPose, GlanceForBanner, AlignToBanner]
    # ...and the camera is put back to the hung-banner view either way.
    cameras = [c for c in hung.children if isinstance(c, SetCameraPose)]
    assert cameras[-1].pose == "BANNER"


# ---- the camera pose the ground view uses -------------------------------- #

def test_the_ground_pose_exists_and_the_banner_detector_runs_in_it():
    from camera_ctrl.camera_ctrl_node import NAMED_POSES_DEG
    from perception_banner.banner_node import CAMERA_POSES
    assert NAMED_POSES_DEG["GROUND"] == -35.0
    assert "GROUND" in CAMERA_POSES
    # ALIGN is the QR stage's pose; a banner pass there would be blind.
    assert "ALIGN" not in CAMERA_POSES


def test_the_headless_banner_is_gated_exactly_like_the_real_detector():
    from headless_world import BANNER_CAMERA_POSES
    from perception_banner.banner_node import CAMERA_POSES
    assert tuple(BANNER_CAMERA_POSES) == tuple(CAMERA_POSES)


# ---- ranging a sheet that lies flat -------------------------------------- #

def ranged(pose, below=0.0, alt=3.0, area_px=20000.0):
    import math
    import types
    mav = MagicMock()
    mav.camera_state = {"requested_rad": math.radians(
        {"GROUND": -30.0, "BANNER": -20.0}[pose])}
    mav.banner = types.SimpleNamespace(x=0.0, y=below, z=1.0)
    mav.alt.return_value = alt
    leaf = AlignToBanner(mav, clock=Clock(), search_camera_pose=pose)
    return leaf.range_from_area(area_px)


def test_a_laid_sheet_is_ranged_where_its_ray_meets_the_ground():
    import math
    # Frame centre at 30 deg down from 3 m: 3 / tan 30.
    assert ranged("GROUND") == pytest.approx(3.0 / math.tan(math.radians(30.0)), rel=1e-3)
    # Lower in the frame is nearer.
    assert ranged("GROUND", below=0.5) < ranged("GROUND")


def test_a_hung_board_is_still_ranged_from_its_area():
    # The hung-board pass keeps the face-on area model it was tuned with.
    assert ranged("BANNER", area_px=20000.0) != ranged("GROUND", area_px=20000.0)
    assert ranged("BANNER", area_px=80000.0) == pytest.approx(
        ranged("BANNER", area_px=20000.0) / 2.0, rel=1e-6)


def test_the_headless_laid_aspect_limits_are_the_real_detectors():
    # banner_node refuses a box outside min_aspect..max_aspect however clear
    # its border; the headless sheet must be refused by the same numbers.
    import re
    from headless_world import BANNER_MAX_ASPECT, BANNER_MIN_ASPECT
    source = open(os.path.join(os.path.dirname(HERE), "src", "aerothon_perception",
                               "perception_banner", "perception_banner",
                               "banner_node.py"), encoding="utf-8").read()
    lo, hi = re.search(r"p\('min_aspect', ([\d.]+)\); p\('max_aspect', ([\d.]+)\)",
                       source).groups()
    assert (BANNER_MIN_ASPECT, BANNER_MAX_ASPECT) == (float(lo), float(hi))


# ---- what the outbound gate learnt, carried to the return gate ----------- #

def run_hint(learned, note=None):
    import types
    mav = types.SimpleNamespace(banner_placement_learned=learned)
    leaf = BannerPlacement("hint", mav, note=note)
    leaf.setup_with_descendants()
    leaf.tick_once()
    return leaf.status, mav.banner_placement_learned


def test_after_a_ground_only_gate_the_hung_view_is_skipped_not_forbidden():
    assert run_hint("ground")[0] == S.FAILURE
    assert run_hint("hung")[0] == S.SUCCESS
    assert run_hint(None)[0] == S.SUCCESS
    # The last resort still runs the hung view: the hint only reorders.
    hung, ground, full = structure().children
    assert isinstance(hung.children[0], BannerPlacement)
    assert not any(isinstance(c, BannerPlacement) and c.note is None
                   for c in full.children)


def test_each_pass_records_what_it_aligned_on():
    assert run_hint(None, note="ground") == (S.SUCCESS, "ground")
    hung, ground, full = structure().children
    assert hung.children[-1].note == "hung"
    assert ground.children[-1].note == "ground"
    assert full.children[-1].note == "hung"


def test_in_the_ground_pose_a_sheet_seen_end_on_is_not_identified():
    # From beside the corridor a laid sheet boxes up nearly square; across the
    # mouth it is a wide strip. Only the second is a mouth to square up on.
    from perception_banner.banner_node import GROUND_MIN_ASPECT
    from headless_world import BANNER_GROUND_MIN_ASPECT
    import math
    import types
    from headless_world import Camera, HeadlessWorld
    assert GROUND_MIN_ASPECT == BANNER_GROUND_MIN_ASPECT == 1.5
    gate = arena("ground").banners[0]
    pitch = math.radians(35.0)

    def aspect(eye, yaw):
        world = types.SimpleNamespace(camera=Camera(1.0472), yaw=yaw)
        world._ground_green_view = lambda *a: HeadlessWorld._ground_green_view(world, *a)
        bbox, *_ = HeadlessWorld._laid_view(world, gate["laid"], eye, pitch)
        return bbox[2] / bbox[3]

    f = gate["facing"]
    front = (gate["xy"][0] - 5.0 * math.cos(f), gate["xy"][1] - 5.0 * math.sin(f), 3.0)
    side = (gate["xy"][0] + 5.0 * math.cos(f + math.pi / 2),
            gate["xy"][1] + 5.0 * math.sin(f + math.pi / 2), 3.0)
    assert aspect(front, f) >= GROUND_MIN_ASPECT
    assert aspect(side, f - math.pi / 2) < GROUND_MIN_ASPECT


# ---- issue 43: a laid gate approached from beside the corridor ------------ #

def test_beside_the_corridor_the_side_wall_sends_the_aircraft_onto_the_axis():
    # Gate mouth at the origin, corridor running +x, walls at y = +-1.75.
    # The aircraft came from the west (outside), drifted round to the north
    # side, and its lidar fits the north wall 3 m away while the camera has
    # the laid sheet dead ahead.
    import math
    leaf = AlignToBanner(MagicMock(), clock=Clock(), search_camera_pose="GROUND")
    leaf._entry_xy = (-6.0, 0.5)
    x, y = 3.0, 4.75
    yaw = math.atan2(0.0 - y, 0.0 - x)                 # facing the sheet
    fit = {"ok": True, "range_m": 3.0, "angle_rad": -math.pi / 2 - yaw}
    vx, vy, face = leaf.laid_mouth_vantage((x, y, yaw), fit,
                                           math.hypot(x, y), 0.0)
    assert vy == pytest.approx(0.0, abs=1e-6), "on the corridor axis"
    assert vx < -leaf.min_standoff + 1e-6, "outside the mouth, a standoff back"
    assert face == pytest.approx(0.0, abs=1e-6), "looking down the corridor"


def test_the_outside_of_the_gate_is_the_side_the_pass_came_from():
    import math
    leaf = AlignToBanner(MagicMock(), clock=Clock(), search_camera_pose="GROUND")
    leaf._entry_xy = (14.0, -0.5)                       # came from the east
    x, y = 3.0, 4.75
    yaw = math.atan2(-y, -x)
    fit = {"ok": True, "range_m": 3.0, "angle_rad": -math.pi / 2 - yaw}
    vx, vy, face = leaf.laid_mouth_vantage((x, y, yaw), fit, math.hypot(x, y), 0.0)
    assert vx > 0 and abs(vy) < 1e-6
    assert abs(abs(face) - math.pi) < 1e-6


def test_a_face_beyond_the_sheet_is_not_read_as_a_side_wall():
    # Return gate, headless: a face at 8.9 m with the sheet at 5.1 m was read
    # as a side wall and sent the aircraft across the return lane.
    mav = MagicMock()
    mav.pos.return_value = (20.0, -15.0, 3.0)
    mav.yaw.return_value = 3.1
    mav.banner_bearing.return_value = 0.0
    leaf = AlignToBanner(mav, clock=Clock(), search_camera_pose="GROUND")
    far = {"ok": True, "range_m": 8.9, "angle_rad": 0.2}
    assert leaf._laid_mouth_vantage(far, 5.1) is None
    near = {"ok": True, "range_m": 2.0, "angle_rad": 1.4}
    assert leaf._laid_mouth_vantage(near, 5.1) is not None


def test_on_the_axis_a_laid_gate_squares_up_by_the_walls_running_past():
    # A laid sheet leaves the mouth open, so the lidar sees only side walls
    # edge-on -- refused as a face, accepted here as the corridor direction.
    import math
    import types
    mav = MagicMock()
    mav.banner_identified.return_value = True
    mav.banner_bearing.return_value = 0.02
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    mav.alt.return_value = 3.4

    def surface_ahead(bearing, half, expected_range_m=None, **kw):
        if kw.get("max_obliquity_rad", 0) > math.radians(90):
            return {"ok": True, "angle_rad": math.radians(88.0), "range_m": 1.85,
                    "points": 30, "reason": ""}
        return {"ok": False, "angle_rad": None, "range_m": None, "points": 0,
                "reason": "edge-on"}
    mav.surface_ahead.side_effect = surface_ahead
    leaf = AlignToBanner(mav, clock=Clock(), search_camera_pose="GROUND")
    leaf._sector_bearing, leaf._standoff = 0.0, None
    fit = leaf._laid_axis_fit()
    assert fit["ok"] and fit["laid_axis"]
    assert math.degrees(fit["angle_rad"]) == pytest.approx(-2.0)
    assert fit["range_m"] == pytest.approx(3.4 / math.tan(math.radians(35.0)), rel=1e-3)
    # A hung-board pass never takes this route.
    mav.banner_bearing.return_value = 0.5
    assert leaf._laid_axis_fit() is None


def test_the_ground_pose_area_floor_allows_for_a_sheet_seen_foreshortened():
    import math
    import types
    from camera_ctrl.camera_ctrl_node import NAMED_POSES_DEG
    from perception_banner.banner_node import GROUND_POSE_DEPRESSION_DEG
    from perception_banner import banner_node
    assert GROUND_POSE_DEPRESSION_DEG == -NAMED_POSES_DEG["GROUND"]
    params = {"min_area_frac": 0.0, "max_detect_range_m": 12.0,
              "camera_hfov": 1.0472, "banner_width_m": 3.7,
              "banner_height_m": 1.15, "area_safety": 0.5}
    node = types.SimpleNamespace(_g=params.get,
                                 gate=types.SimpleNamespace(requested="BANNER"))
    face_on = banner_node.BannerNode.min_area_px(node, 1280, 720)
    node.gate.requested = "GROUND"
    laid = banner_node.BannerNode.min_area_px(node, 1280, 720)
    assert laid == pytest.approx(face_on * math.sin(math.radians(35.0)))
    assert laid < 12400 < face_on, "the rendered sheet's 12,400 px now passes"


def test_the_ground_pose_accepts_the_long_strip_a_laid_sheet_makes_close_in():
    from perception_banner.banner_node import GROUND_MAX_ASPECT
    from headless_world import BANNER_GROUND_MAX_ASPECT, BANNER_MAX_ASPECT
    assert GROUND_MAX_ASPECT == BANNER_GROUND_MAX_ASPECT == 16.0
    # Rendered, 4.9 m out at 3.1 m up: 949 x 80 px.
    assert BANNER_MAX_ASPECT < 949 / 80 <= GROUND_MAX_ASPECT


def _laid_leaf(walls_visible):
    import math
    import types
    mav = MagicMock()
    mav.pos.return_value = (21.9, -15.1, 3.0)
    mav.yaw.return_value = math.radians(-175.0)
    mav.banner_identified.return_value = True
    mav.banner_bearing.return_value = 0.0
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    mav.alt.return_value = 3.0

    def surface_ahead(bearing, half, expected_range_m=None, **kw):
        if walls_visible and kw.get("max_obliquity_rad", 0) > math.radians(90):
            return {"ok": True, "angle_rad": math.radians(-80.0), "range_m": 1.9,
                    "points": 30, "reason": ""}
        return {"ok": True, "angle_rad": math.radians(-12.0), "range_m": 6.7,
                "points": 20, "reason": ""}
    mav.surface_ahead.side_effect = surface_ahead
    leaf = AlignToBanner(mav, clock=Clock(), search_camera_pose="GROUND")
    leaf._sector_bearing, leaf._standoff = 0.0, None
    return leaf


@pytest.mark.parametrize("walls_visible", [False, True])
def test_a_face_beyond_the_mouth_squares_up_instead_of_relocating(walls_visible):
    # Calm return gate, headless: the first slalom obstacle 1 m inside the
    # mouth was refused six times as "not the mouth" and the stage looped.
    import math
    leaf = _laid_leaf(walls_visible)
    leaf._relocate = MagicMock(side_effect=AssertionError("relocated"))
    laid = leaf.ground_range()
    out = leaf._laid_face({"ok": True, "angle_rad": math.radians(-12.0),
                           "range_m": 6.7, "points": 20, "reason": ""})
    assert isinstance(out, dict) and out["ok"]
    assert out["range_m"] == pytest.approx(laid)
    expected = 10.0 if walls_visible else -12.0
    assert math.degrees(out["angle_rad"]) == pytest.approx(expected)


def test_a_face_nearer_than_the_sheet_is_still_refused():
    import math
    leaf = _laid_leaf(False)
    leaf._relocate = MagicMock(return_value="relocated")
    leaf._laid_mouth_vantage = MagicMock(return_value=None)
    out = leaf._laid_face({"ok": True, "angle_rad": math.radians(80.0),
                           "range_m": 1.0, "points": 20, "reason": ""})
    assert out == "relocated"
