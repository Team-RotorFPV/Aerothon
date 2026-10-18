"""Phase 2: the arena the organisers clarified, entered at the GCS.

A banner whose height is not given (perhaps a foot tall, hung from the rod at
the 10 ft wall top), an orange banner beside every green one, and a detector
that takes the banner's size from the operator instead of a launch file.

    python3 -m pytest sim/test_phase2_arena.py -v
"""
import json
import os
import sys
import types
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))

from perception_banner import banner_shape, colours           # noqa: E402
from perception_banner.banner_node import BannerNode          # noqa: E402

SUPPLIED = {"green": "#07A03D", "orange": "#EB5A0A", "red": "#C4161C"}


# ---- a banner thinner than the board the limits were measured on -------- #

@pytest.mark.parametrize("height_m", [0.3048, 0.457, 1.15])
def test_a_face_on_banner_of_any_height_passes_the_upright_limit(height_m):
    w = 3.5
    assert w / height_m < banner_shape.max_aspect(w, height_m)


def test_the_limits_never_tighten_below_what_was_measured():
    assert banner_shape.max_aspect(3.7, 1.15) == pytest.approx(8.0)
    assert banner_shape.max_aspect(2.0, 1.0) == pytest.approx(8.0)
    assert banner_shape.max_aspect(3.7, 1.15, ground=True) == pytest.approx(16.0)


def test_a_foot_tall_laid_sheet_seen_from_the_ground_pose_is_accepted():
    import math
    # Face-on from the 35 deg GROUND pose a laid sheet foreshortens by the
    # sine of the look-down angle: 3.5 x 0.3 m reads about 20:1.
    seen = 3.5 / (0.3048 * math.sin(math.radians(35.0)))
    assert seen > banner_shape.GROUND_MAX
    assert seen < banner_shape.max_aspect(3.5, 0.3048, ground=True)


def test_bad_sizes_fall_back_to_the_measured_limits():
    assert banner_shape.stretch(None, 1.0) == 1.0
    assert banner_shape.stretch(3.5, 0.0) == 1.0


# ---- the detector takes the banner's size from the GCS ------------------ #

def _node():
    node = types.SimpleNamespace(set_parameters=MagicMock(),
                                 get_logger=lambda: MagicMock())
    return node


def _applied(applied=True, w=3.5, h=0.3048):
    return types.SimpleNamespace(data=json.dumps(
        {"applied": applied, "dimensions": {"banner": {"width_m": w, "height_m": h}}}))


def test_the_banner_size_follows_the_dimensions_the_mission_applied():
    node = _node()
    BannerNode._on_arena_dimensions(node, _applied())
    (params,), _ = node.set_parameters.call_args
    got = {p.name: p.value for p in params}
    assert got == {"banner_width_m": 3.5, "banner_height_m": 0.3048}


def test_dimensions_the_mission_refused_do_not_resize_the_detector():
    node = _node()
    BannerNode._on_arena_dimensions(node, _applied(applied=False))
    BannerNode._on_arena_dimensions(node, types.SimpleNamespace(data="not json"))
    BannerNode._on_arena_dimensions(node, _applied(h=0.0))
    node.set_parameters.assert_not_called()


# ---- green is the banner to fly to; orange never a candidate ------------ #

def _patch(hex_code):
    bgr = np.full((8, 8, 3), colours.parse_hex(hex_code)[::-1], np.uint8)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


def test_with_colours_supplied_the_search_mask_takes_green_and_not_orange():
    node = types.SimpleNamespace(_colours=dict(SUPPLIED))
    bands = BannerNode.colour_ranges(node)
    assert colours.mask(_patch(SUPPLIED["green"]), bands).all()
    assert not colours.mask(_patch(SUPPLIED["orange"]), bands).any()


def test_orange_is_still_named_when_asked_about_a_panel():
    node = types.SimpleNamespace(_colours=dict(SUPPLIED))
    node.colour_ranges = lambda name=None: BannerNode.colour_ranges(node, name)
    panel = np.full((20, 40, 3), colours.parse_hex(SUPPLIED["orange"])[::-1], np.uint8)
    name, frac = BannerNode.dominant_colour(node, panel)
    assert name == "orange" and frac > 0.9


# ---- the world is built from the same dimensions the GCS applies -------- #

import world_spec as W                                        # noqa: E402
import world_from_dimensions as WFD                           # noqa: E402
from mission_bt import arena_dimensions as A                  # noqa: E402


def _clarified(height_m=0.3048, placement="both"):
    d = A.validate(A.BUILTIN_PRESETS["SAE Aerothon 2026 (clarified)"])
    wall = d["corridor"]["wall_height_m"]
    d["banner"].update(height_m=height_m, suspended_bottom_m=wall - height_m,
                       placement=placement)
    return d


def _layout():
    with open(os.path.join(HERE, "worlds", "banner_both.json"), encoding="utf-8") as f:
        return json.load(f)


@pytest.mark.parametrize("height_m", [0.3048, 1.15])
def test_the_world_matches_the_dimensions_entered(height_m):
    d = _clarified(height_m)
    spec = W.normalise(WFD.build(d, _layout(), "t"))
    assert W.validate(spec)[0] == []
    c, r, b = spec["corridor"], spec["return_corridor"], spec["banner"]
    assert r["linked"] is True and r["width"] == c["width"] == 3.5
    assert c["wall_height"] == pytest.approx(3.048)
    # Hung from the rod at the wall top, as wide as the corridor.
    assert b["mount"] == "rod" and b["orange"] is True
    assert b["board_bottom_m"] + b["height_m"] == pytest.approx(c["wall_height"])
    assert b["width_m"] == c["width"]
    assert spec["qr"] == {"start_m": 1.0, "target_m": 1.0}
    assert len(spec["red_zones"]) == 1
    assert spec["red_zones"][0]["w"] == pytest.approx(1.524)
    # Obstacles in the exit corridor reach half its width from a wall.
    for o in r["obstacles"]:
        assert o["d"] == pytest.approx(c["width"] / 2)
        assert abs(o["v"]) + o["d"] / 2 == pytest.approx(c["width"] / 2)


def test_the_delivery_zone_is_kept_clear_of_the_corridor_ends():
    spec = W.normalise(WFD.build(_clarified(), _layout(), "t"))
    z = spec["delivery_zone"]
    import math
    ends = [W._place((float(l["length"]), 0.0), l["x"], l["y"], math.radians(l["yaw_deg"]))
            for l in (spec["corridor"], spec["return_corridor"])]
    assert z["x"] - z["w"] / 2 >= max(x for x, _ in ends) + WFD.ZONE_CLEAR_M - 1e-6


def test_a_rod_hung_banner_above_the_walls_is_refused():
    spec = W.normalise(WFD.build(_clarified(), _layout(), "t"))
    spec["banner"]["board_bottom_m"] = 2.9
    errs, _ = W.validate(spec)
    assert any("above" in e for e in errs)


def test_the_headless_arena_builds_the_rod_hung_banners_and_the_orange_ones():
    from headless_world import Arena
    spec = W.normalise(WFD.build(_clarified(0.3048, "both"), _layout(), "t"))
    a = Arena(spec, "b")
    assert not [box for box in a.boxes if box[3] == "post"], "the walls carry it"
    boards = [box for box in a.boxes if box[3] == "board"]
    assert len(boards) == 4, "green at each entrance, orange at each far end"
    for _, z0, z1, _ in boards:
        assert z1 == pytest.approx(3.048) and z1 - z0 == pytest.approx(0.3048)
    assert a.board_w == 3.5 and a.board_h == pytest.approx(0.3048)


# ---- the hung-banner square-up finds a thin banner ---------------------- #

def _align(banner_h_m, banner_bottom_m=None, z=5.0):
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.low_known_xy = [(0.0, 0.0)]                   # it has been low right here
    leaf = AlignToBanner(mav, clock=lambda: 0.0, banner_w_m=3.5, banner_h_m=banner_h_m,
                         banner_bottom_m=banner_bottom_m, alt_floor_m=2.4384)
    leaf._anchor = (0.0, 0.0, z)
    leaf._surface = None
    leaf._descents = 0
    leaf._lower_camera = lambda: None
    return leaf


def test_the_descent_never_steps_over_a_thin_banner():
    leaf = _align(0.3048)
    leaf._no_surface({"ok": False, "points": 0, "reason": "nothing"})
    _, _, z = leaf._anchor
    assert 5.0 - z <= 0.8 * 0.3048 + 1e-9


def test_with_the_banner_height_from_the_gcs_it_goes_straight_to_it():
    leaf = _align(0.3048, banner_bottom_m=3.048 - 0.3048)
    leaf._no_surface({"ok": False, "points": 0, "reason": "nothing"})
    _, _, z = leaf._anchor
    lidar = z + leaf.lidar_above_body_m
    assert 3.048 - 0.3048 < lidar < 3.048, "the scan plane cuts the banner"


# ---- the precision landing on a 1 m start QR ---------------------------- #

def test_the_landing_commits_when_a_turned_start_qr_leaves_the_frame_near_the_floor():
    # Smoke run: a 1 m start QR 25 deg off square lost at 2.53 m, 2.25 m commit.
    from types import SimpleNamespace as NS
    import py_trees
    from mission_bt.mission_tree import PrecisionDescent

    class Mav:
        def __init__(self):
            self.at = (0.0, 0.0, 2.53)
            self.home_marker_xy = (0.0, 0.0)
            self.qr_offset = NS(x=0.0, y=0.0, z=1.0)
            self.gotos, self.logs = [], []
        def pos(self): return self.at
        def yaw(self): return 0.0
        def home_local_xy(self): return (0.0, 0.0)
        def qr_visible(self): return self.qr_offset.z > 0.0
        def goto(self, *a): self.gotos.append(a)
        def log(self, *a, **k): self.logs.append(a)

    mav = Mav()
    leaf = PrecisionDescent(mav, commit_alt=1.5, marker_m=1.0, hfov_rad=0.851919,
                            lost_grace=3)
    leaf.initialise()
    leaf.update()                                   # centred
    mav.qr_offset = NS(x=0.0, y=0.0, z=0.0)         # the turned marker left the frame
    for _ in range(4):
        leaf.update()
    assert all(g[2] <= 2.53 + 1e-9 for g in mav.gotos), "never climbs back"
    assert leaf.update() is py_trees.common.Status.SUCCESS
    assert str(mav.landing_precision).startswith("PRECISE")


def test_the_landing_uses_the_start_qr_size_from_the_gcs():
    d = A.validate(A.BUILTIN_PRESETS["SAE Aerothon 2026 (clarified)"])
    assert A.mission_params(d, {})["start_marker_m"] == 1.0


# ---- an orbit leg refuses a wall alongside, not only ahead -------------- #

def _scan_mav(ranges_by_deg, yaw=0.0):
    import math
    n = 360
    ranges = [float("inf")] * n
    for deg, r in ranges_by_deg.items():
        ranges[int(deg) % n] = r
    scan = types.SimpleNamespace(ranges=ranges, range_min=0.05, range_max=12.0,
                                 angle_min=0.0, angle_increment=math.radians(1.0))
    mav = MagicMock()
    mav._scan = scan
    mav.pos.return_value = (0.0, 0.0, 3.0)
    mav.yaw.return_value = yaw
    return mav


def test_a_leg_beside_a_wall_is_refused():
    from mission_bt.banner_orbit import leg_clear
    # Leg due +x; the wall runs alongside at 0.7 m on the left (90 deg).
    beside = _scan_mav({88: 0.72, 90: 0.7, 92: 0.72})
    assert not leg_clear(beside, 5.0, 0.0)


def test_open_flanks_and_the_airframe_itself_do_not_refuse_a_leg():
    from mission_bt.banner_orbit import leg_clear
    assert leg_clear(_scan_mav({90: 2.0, 270: 1.5}), 5.0, 0.0)
    assert leg_clear(_scan_mav({180: 0.22, 151: 0.35}), 5.0, 0.0), "mast and arms"


# ---- a GROUND pass backs away from green it cannot identify ------------- #

def test_the_ground_pass_backs_away_from_cut_green_before_orbiting():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (1.0, -0.5, 5.0)
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND")
    leaf._green = {"x": 6.0, "y": -3.0, "range": 5.6, "heading": math.radians(-30),
                   "source": "camera", "cut": True}
    leaf._restart_sweep_at = MagicMock(return_value="moving")
    assert leaf._next_orbit_vantage(5.0) == "moving"
    (nx, ny, nz, hd), _ = leaf._restart_sweep_at.call_args
    assert math.hypot(nx - 1.0, ny + 0.5) == pytest.approx(3.0)
    assert (nx - 1.0) < 0 < (ny + 0.5), "away from the green, not round it"
    assert nz == 5.0
    leaf._next_orbit_vantage(5.0)
    leaf._restart_sweep_at.reset_mock()
    assert leaf._ground_backoff(5.0) is None, "bounded: two back-offs"


def test_a_hung_pass_does_not_back_off():
    from mission_bt.mission_tree import AlignToBanner
    leaf = AlignToBanner(MagicMock(), clock=lambda: 0.0)
    leaf._green = {"x": 6.0, "y": -3.0, "range": 5.6, "heading": 0.0, "source": "camera"}
    assert leaf._ground_backoff(5.0) is None


# ---- the return gate's zone side, whatever gap the venue leaves --------- #

def _finder(gate_heading=0.0):
    from mission_bt.mission_tree import FindReturnBanner
    mav = MagicMock()
    mav.gate_heading = gate_heading
    mav.outbound_banner_xy = (0.0, 0.0)
    # The zone starts 9 m beyond the 10 m corridors.
    mav.delivery_search_zone = lambda c: (19.0 + c, 59.0 - c, -15.0 + c, 15.0 - c)  # c<0 grows it
    return FindReturnBanner(mav, alt=5.0, clock=lambda: 0.0, corridor_length_m=10.0)


def test_a_stand_off_beyond_the_corridors_is_the_zone_side_however_far_the_zone():
    f = _finder()
    assert f._zone_side(15.0, -4.0), "5 m past the return entrance"
    assert not f._zone_side(5.0, -4.0), "inside the return lane is its back"


def test_without_the_outbound_axis_the_zone_box_still_decides():
    f = _finder()
    f.mav.gate_heading = None
    assert not f._zone_side(15.0, -4.0)
    assert f._zone_side(17.0, -4.0)


# ---- a laid return banner is looked at from where it is in frame -------- #

def test_the_return_search_stands_farther_off_a_ground_mounted_gate():
    f = _finder()
    assert f._standoff() == f.standoff_m
    f.mav.banner_placement_learned = "ground"
    assert f._standoff() >= 7.5


def test_a_ground_pass_relocates_back_along_the_bearing():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (20.0, -6.0, 5.0)
    mav.yaw.return_value = math.pi
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND")
    leaf._seen_yaw = math.pi                 # facing the gate, -x
    leaf._restart_sweep_at = MagicMock(return_value="moving")
    leaf._relocate("nothing in view")
    (nx, ny, nz, _), _ = leaf._restart_sweep_at.call_args
    assert nx > 20.0, "stepped back, away from the sheet"


def test_a_far_laid_sheet_is_closed_in_on_while_descending_to_square_up():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (23.2, -6.8, 5.0)
    mav.yaw.return_value = math.pi
    mav.low_known_xy = [(23.2, -6.8)]                 # been low here before
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND",
                         alt_floor_m=2.4384)
    leaf.ground_range = lambda: 7.5
    leaf._begin_square()
    ax, ay, az = leaf._anchor
    target = 0.5 * (leaf.min_standoff + leaf.max_standoff)
    assert 23.2 - ax == pytest.approx(7.5 - target)
    assert az == pytest.approx(2.4384)


# ---- a laid square-up measures past the mouth, widest face first -------- #

def test_past_the_mouth_the_widest_face_wins_over_a_stub_of_wall_end():
    import math
    from mission_bt.scan_geometry import fit_surface
    inc = math.radians(1.0)
    n = 360
    ranges = [float("inf")] * n
    a0 = -math.pi

    def put(deg, r):
        ranges[int(round((math.radians(deg) - a0) / inc)) % n] = r

    # A stub of wall end, 0.4 m of it, at 5.6 m on the left...
    for k, deg in enumerate(range(-26, -22)):
        put(deg, 5.6 + 0.1 * k)
    # ...and a 1.6 m obstacle face square across the lane at 6.2 m.
    for deg in range(-1, 15):
        put(deg, 6.2 / math.cos(math.radians(deg)))
    near = fit_surface(a0, inc, ranges, 0.0, math.radians(35))
    assert not near["ok"]
    far = fit_surface(a0, inc, ranges, 0.0, math.radians(35),
                      min_range_m=5.0, widest_first=True)
    assert far["ok"] and far["range_m"] == pytest.approx(6.2, abs=0.05)
    assert abs(far["angle_rad"]) < math.radians(2.0)


def test_a_flickering_identification_does_not_forget_where_the_sheet_is():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (20.0, -6.8, 2.4)
    mav.alt.return_value = 2.4
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND")
    seen = leaf._laid_range_now()
    assert seen == pytest.approx(2.4 / math.tan(math.radians(35.0)))
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=0.5)     # dropped a frame
    mav.pos.return_value = (19.5, -6.8, 2.4)                    # and moved 0.5 m in
    assert leaf._laid_range_now() == pytest.approx(seen - 0.5)


def test_a_square_up_step_into_the_next_corridor_is_refused():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = _scan_mav({88: 0.72, 90: 0.7, 92: 0.72})    # a wall closing in on the left
    leaf = AlignToBanner(mav, clock=lambda: 0.0)
    leaf._sq_phase = leaf.MOVING
    leaf._anchor = (0.0, 2.2, 3.0)                      # a step 2.2 m to port
    leaf._relocate = MagicMock(return_value="relocated")
    assert leaf._square() == "relocated"
    x, y, _ = leaf._anchor
    assert (x, y) == (0.0, 0.0), "held where it was"


def test_past_the_mouth_an_edge_on_wall_gives_the_corridor_direction():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (21.6, -6.8, 2.4)
    mav.alt.return_value = 2.4
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    mav.banner_identified.return_value = True
    mav.banner_bearing.return_value = 0.0

    def surface_ahead(bearing, half, expected_range_m=None, **kw):
        if kw.get("min_range_m") is not None:          # the past-the-sheet retry
            return {"ok": True, "angle_rad": math.radians(-85.0), "range_m": 1.9,
                    "points": 40, "reason": ""}
        return {"ok": False, "angle_rad": None, "range_m": None, "points": 4,
                "reason": "the returns span only 0.47 m"}
    mav.surface_ahead.side_effect = surface_ahead
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND")
    leaf._last_good_bearing = 0.0
    leaf._laid_axis_fit = lambda: None
    fit = leaf._measure_surface()
    assert fit["ok"] and fit["laid_axis"]
    assert math.degrees(fit["angle_rad"]) == pytest.approx(5.0)
    assert fit["range_m"] == pytest.approx(leaf.ground_range())


# ---- never straight down onto what the lidar cannot see ----------------- #

def _low_mav(pos, low=((0.0, 0.0),)):
    mav = MagicMock()
    mav.pos.return_value = pos
    mav.low_known_xy = list(low)
    return mav


def test_a_descent_past_the_wall_tops_goes_down_where_it_has_been_low():
    from mission_bt.mission_tree import AlignToBanner
    mav = _low_mav((7.44, -5.4, 3.95))               # over the gap between lanes
    leaf = AlignToBanner(mav, clock=lambda: 0.0, wall_top_m=3.048)
    legs = leaf._down_where_low([(7.44, -5.4, 2.44)])
    assert legs[0] == (0.0, 0.0, 3.95), "across high to home first"
    assert legs[1] == (0.0, 0.0, 2.44), "down there"
    assert legs[-1] == (7.44, -5.4, 2.44), "then across low, lidar-guarded"


def test_a_known_clear_spot_or_a_high_leg_is_left_alone():
    from mission_bt.mission_tree import AlignToBanner, note_low
    mav = _low_mav((7.44, -5.4, 3.95), low=((0.0, 0.0), (7.0, -5.0)))
    leaf = AlignToBanner(mav, clock=lambda: 0.0, wall_top_m=3.048)
    assert leaf._down_where_low([(7.44, -5.4, 2.44)]) == [(7.44, -5.4, 2.44)]
    assert leaf._down_where_low([(20.0, 0.0, 5.0)]) == [(20.0, 0.0, 5.0)]
    mav.pos.return_value = (12.0, 3.0, 2.0)
    note_low(mav, 3.048)
    assert (12.0, 3.0) in mav.low_known_xy


def test_a_route_down_where_it_is_clear_is_not_cut_short_by_a_sighting():
    from mission_bt.mission_tree import AlignToBanner
    mav = _low_mav((23.2, -6.8, 5.0), low=((21.1, -2.6),))
    mav.yaw.return_value = 3.14
    leaf = AlignToBanner(mav, clock=lambda: 0.0, wall_top_m=3.048)
    leaf._restart_sweep_at(21.7, -6.8, 2.44, 3.14)
    assert leaf._mouth_leg, "the sighting interrupt is off for the route"
    assert leaf._anchor[:2] == (21.1, -2.6)


# ---- gate-height work stays under a hung banner ------------------------- #

def _align_floors(extra):
    import math
    from mission_bt.mission_tree import AlignToBanner, build_root
    from test_fail_closed_stages import FakeMav
    params = {'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
              'image_width_px': 1280, 'camera_hfov': 1.0472, 'target_marker_m': 1.0,
              'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
              'zone_boundary_clearance': 1.0, 'corridor_alt': 2.4384,
              'scan_floor_alt': 2.0, 'land_commit_alt': 1.5,
              'banner_sweep_limit': math.pi, **extra}
    root = build_root(FakeMav(), MagicMock(), params)
    return {leaf.alt_floor_m for leaf in root.iterate() if isinstance(leaf, AlignToBanner)}


def test_with_a_foot_tall_hung_banner_the_gate_work_stays_half_a_metre_under_it():
    floors = _align_floors({'banner_bottom_m': 3.048 - 0.3048, 'banner_h_m': 0.3048})
    assert floors and all(f == pytest.approx(3.048 - 0.3048 - 0.5) for f in floors)


def test_without_the_banner_edge_the_gate_work_is_at_the_navigation_height():
    assert all(f == pytest.approx(2.4384) for f in _align_floors({}))
    # The 1.15 m banner spans the scan plane at 2.44 m: the lidar sees it.
    assert all(f == pytest.approx(2.4384) for f in _align_floors(
        {'banner_bottom_m': 1.898, 'banner_h_m': 1.15}))
    # Laid on the ground, nothing hangs: the edge the GCS records steers nothing.
    assert all(f == pytest.approx(2.4384) for f in _align_floors(
        {'banner_bottom_m': 2.7432, 'banner_placement': 'ground'}))


def test_a_gate_floor_under_the_takeoff_guard_moves_the_guard_down():
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.airborne_floor = 2.0
    leaf = AlignToBanner(mav, clock=lambda: 0.0, alt_floor_m=1.5)
    leaf.initialise()
    (floor,), _ = mav.set_airborne_floor.call_args
    assert 0.5 <= floor < 1.5
    mav2 = MagicMock()
    mav2.airborne_floor = 1.0
    AlignToBanner(mav2, clock=lambda: 0.0, alt_floor_m=2.4).initialise()
    mav2.set_airborne_floor.assert_not_called()


# ---- a laid sheet seen at a slant is identified along its outline ------- #

@pytest.fixture
def detector():
    """A BannerNode in its own rclpy context, shut down afterwards: a context
    left open broke a later file's rclpy.init() in the full suite."""
    import rclpy
    rclpy.init()
    made = []
    yield lambda **kw: made.append(_detector(**kw)) or made[-1]
    for node in made:
        node.destroy_node()
    rclpy.shutdown()


def _detector(height_m=1.15):
    from std_msgs.msg import String
    node = BannerNode()
    node._colours = {'green':'#07A03D','orange':'#EB5A0A','red':'#C4161C'}
    node._on_banner_colours(String(data=json.dumps({"colours": SUPPLIED})))
    node._on_arena_dimensions(String(data=json.dumps(
        {"applied": True, "dimensions": {"banner": {"width_m": 3.5, "height_m": height_m}}})))
    node.gate = types.SimpleNamespace(requested="GROUND")
    return node


def _slanted_sheet(shadow=False, corner=False):
    """A green panel in a 7 px white border, as a slanted quadrilateral on
    grey floor -- a laid sheet seen from off the corridor's axis."""
    img = np.full((720, 1280, 3), (160, 165, 165), np.uint8)
    outer = np.array([[440, 355], [560, 430], [915, 232], [795, 182]], np.int32)
    inner = np.array([[460, 357], [563, 421], [897, 230], [797, 191]], np.int32)
    if corner:
        outer, inner = outer + [700, -200], inner + [700, -200]
    green = tuple(int(c) for c in colours.parse_hex(SUPPLIED["green"])[::-1])
    cv2.fillConvexPoly(img, outer, (245, 245, 245))
    cv2.fillConvexPoly(img, inner, green)
    if shadow:   # a post's shadow over the far short edge, border included
        cv2.fillConvexPoly(img, np.array([[880, 215], [930, 240], [900, 270], [850, 245]], np.int32),
                           (80, 82, 82))
        cv2.fillConvexPoly(img, np.array([[860, 222], [890, 236], [880, 250], [850, 236]], np.int32),
                           (20, 70, 30))
    return img


def _bbox(node, img):
    cnts, _ = cv2.findContours(node.colour_mask(img), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return cv2.boundingRect(max(cnts, key=cv2.contourArea))


@pytest.mark.parametrize("shadow", [False, True])
def test_a_slanted_framed_sheet_is_identified_along_its_outline(shadow, detector):
    node = detector()
    img = _slanted_sheet(shadow=shadow)
    ok, reason, info, _ = node._identify_bordered(img, _bbox(node, img))
    assert ok, reason
    assert info["identity_path"] == "border-outline"


def test_a_sheet_run_into_a_frame_corner_is_not_identified(detector):
    node = detector()
    img = _slanted_sheet(corner=True)
    ok, reason, _, _ = node._identify_bordered(img, _bbox(node, img))
    assert not ok


def test_a_dropped_frame_does_not_let_a_far_face_through_as_the_standoff():
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = MagicMock()
    mav.pos.return_value = (0.3, -3.3, 2.4)
    mav.alt.return_value = 2.4
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND")
    sheet = leaf._laid_range_now()
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=0.5)       # dropped
    leaf._laid_axis_fit = lambda: None
    out = leaf._laid_face({"ok": True, "angle_rad": 0.05, "range_m": 7.2,
                           "points": 20, "reason": ""})
    assert out["range_m"] == pytest.approx(sheet), "the sheet's range, not the far face's"


# ---- a laid sheet identified from above is carried at gate height ------- #

def _carry_mav():
    import math
    mav = MagicMock()
    mav.pos.return_value = (-0.8, 2.0, 5.0)
    mav.alt.return_value = 5.0
    mav.yaw.return_value = math.radians(-50.0)
    mav.camera_state = {"requested_rad": math.radians(-35.0)}
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    mav.banner_identified.return_value = True
    mav.banner_bearing.return_value = 0.0
    mav.banner_clipped = False
    mav.node = None                                  # no clock: freshness not checked
    return mav


def _sheet_leaf(mav):
    from mission_bt.mission_tree import AlignToBanner
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND",
                         alt_floor_m=2.4384)
    leaf.ground_range()                              # identified from 5 m up
    assert leaf._sheet_xy is not None
    return leaf


def _on_axis(mav, leaf, green_cols):
    """On the corridor axis at gate height, facing the sheet; detector says green
    (not identified) over the image columns `green_cols`."""
    import math
    sx, sy = leaf._sheet_xy
    mav.pos.return_value = (sx - 4.2, sy, 2.44)
    mav.alt.return_value = 2.44
    mav.yaw.return_value = 0.0
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=0.5)
    mav.banner_identified.return_value = False
    mav.banner_green = {"px": [green_cols[0], 300, green_cols[1] - green_cols[0], 200],
                        "wh": [1280, 720], "t": 0.0, "regions": []}


def test_a_laid_sheet_identified_from_above_is_held_at_gate_height():
    # Rendered, ground-only: identified 100/100 from 5 m, refused at 2.4 m on the
    # axis (overflowing, then too foreshortened), and the stage ran out of places.
    mav = _carry_mav()
    leaf = _sheet_leaf(mav)
    _on_axis(mav, leaf, (0, 1280))                   # overflowing the frame
    assert leaf._ident()
    assert abs(leaf._bearing_now()) < 0.01
    assert leaf.ground_range() == pytest.approx(4.2, abs=0.01)


def test_green_away_from_where_the_sheet_lies_is_not_carried():
    mav = _carry_mav()
    leaf = _sheet_leaf(mav)
    _on_axis(mav, leaf, (1000, 1280))                # green off to the right only
    assert not leaf._ident()
    assert leaf.ground_range() is None


def test_above_the_walls_the_camera_must_read_the_sheet_itself():
    mav = _carry_mav()
    leaf = _sheet_leaf(mav)
    _on_axis(mav, leaf, (0, 1280))
    sx, sy = leaf._sheet_xy
    mav.pos.return_value = (sx - 4.2, sy, 5.0)
    assert not leaf._ident()


def test_a_hung_pass_carries_nothing():
    from mission_bt.mission_tree import AlignToBanner
    mav = _carry_mav()
    leaf = AlignToBanner(mav, clock=lambda: 0.0, alt_floor_m=2.4384)
    leaf._sheet_xy = (3.0, 0.0)
    mav.banner_identified.return_value = False
    mav.banner_green = {"px": [0, 300, 1280, 200], "wh": [1280, 720], "t": 0.0}
    mav.pos.return_value = (0.0, 0.0, 2.44)
    mav.yaw.return_value = 0.0
    assert not leaf._ident()


def test_a_clipped_box_takes_the_sheets_bearing_not_the_boxs():
    mav = _carry_mav()
    leaf = _sheet_leaf(mav)
    _on_axis(mav, leaf, (0, 1280))
    mav.banner_identified.return_value = True
    mav.banner_clipped = True
    mav.banner_bearing.return_value = 0.4            # the cut box's centre
    assert abs(leaf._bearing_now()) < 0.01


# ---- a ground-only arena's return sheet is looked for on the ground ------ #

def test_on_a_ground_only_arena_the_return_search_looks_down_from_look_height():
    import math
    f = _finder()
    f.mav.pos.return_value = (30.0, 0.0, 5.0)
    f.mav.yaw.return_value = math.pi
    f.initialise()
    assert f._look == f.look_pose == "FORWARD", "a hung arena: unchanged"
    f.mav.banner_placement_learned = "ground"
    f.initialise()
    assert f._look == "GROUND" and f._second_look == "BANNER"


def test_a_laid_return_sheet_is_ranged_where_it_meets_the_ground():
    import math
    f = _finder()
    f.mav.banner_placement_learned = "ground"
    f.mav.pos.return_value = (30.0, 0.0, 5.0)
    f.mav.alt.return_value = 5.0
    f.mav.yaw.return_value = math.pi
    f.mav.banner_identified.return_value = True
    f.mav.banner_clipped = False
    f.mav.banner_board_area = 900.0                 # foreshortened: reads ~50 m by area
    f.mav.banner_bearing.return_value = 0.0
    f.mav.banner_observation = None
    f.mav.camera_state = {"requested_rad": math.radians(-35.0)}
    f.mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
    f.mav.outbound_banner_xy = None
    f.initialise()
    bx, by, th, rng = f._sighting()
    assert rng == pytest.approx(5.0 / math.tan(math.radians(35.0)))
    assert bx == pytest.approx(30.0 - rng)


def test_the_return_square_up_carries_the_sheet_the_return_search_ranged():
    # Rendered p2r9, ground-only: the return square-up read the sheet on 49/82
    # frames with nothing noted to carry it; four sweeps passed 900 s.
    import math
    from mission_bt.mission_tree import AlignToBanner
    mav = _carry_mav()
    mav.laid_sheet_xy = (17.9, -6.4)                  # FindReturnBanner's ground range
    leaf = AlignToBanner(mav, clock=lambda: 0.0, search_camera_pose="GROUND",
                         alt_floor_m=2.4384)
    assert leaf._sheet_xy is None
    mav.pos.return_value = (22.1, -6.4, 2.44)
    mav.alt.return_value = 2.44
    mav.yaw.return_value = math.pi
    mav.banner = types.SimpleNamespace(x=0.0, y=0.0, z=0.5)
    mav.banner_identified.return_value = False
    mav.banner_green = {"px": [500, 300, 300, 200], "wh": [1280, 720], "t": 0.0}
    assert leaf._ident()
    assert leaf.ground_range() == pytest.approx(4.2)


def test_a_clipped_sighting_notes_the_sheet_only_when_nothing_better_is_known():
    mav = _carry_mav()
    mav.banner_clipped = True
    leaf = _sheet_leaf(mav)                            # first sighting, clipped: kept
    first = leaf._sheet_xy
    mav.pos.return_value = (0.0, 0.0, 5.0)
    leaf.ground_range()
    assert leaf._sheet_xy == first, "a later clipped sighting does not move it"


# ---- the slalom steers at a waypoint it can move toward ----------------- #

def test_a_short_first_waypoint_does_not_steer_straight_at_the_obstacle():
    # Rendered p2r10: first waypoint 0.1 m dead ahead, the turn after it.
    import math
    from avoidance.velocity_controller import steer_point
    path = [(0.0, 0.0), (0.1, 0.0), (0.15, -0.8), (0.95, -1.15)]
    x, y = steer_point(path, lambda a, b: True, 0.3)
    assert (x, y) == (0.15, -0.8)
    assert math.degrees(math.atan2(y, x)) < -45.0


def test_an_unclear_short_cut_keeps_the_planners_first_waypoint():
    from avoidance.velocity_controller import steer_point
    path = [(0.0, 0.0), (0.1, 0.0), (0.15, -0.8)]
    assert steer_point(path, lambda a, b: False, 0.3) == (0.1, 0.0)
    assert steer_point([(0.0, 0.0), (0.9, 0.2), (1.5, 1.0)], lambda a, b: True, 0.3) == (0.9, 0.2), "an ordinary first step is kept"
