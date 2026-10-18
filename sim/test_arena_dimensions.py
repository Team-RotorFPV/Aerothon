"""Arena dimensions entered at the GCS, as checks.

Every dimension the algorithms use is one validated field; the GCS refuses
bad input with a reason; presets persist; and mission_bt rebuilds with new
dimensions only on the ground.

    python3 -m pytest sim/test_arena_dimensions.py -v
"""
import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/aerothon_mission/mission_bt'))

from mission_bt import arena_dimensions as A                      # noqa: E402


# ---- the fields ----------------------------------------------------------- #

def test_the_defaults_are_the_launch_values_so_nothing_changes_unasked():
    d = A.validate({})
    assert d["banner"]["width_m"] == 3.7 and d["banner"]["height_m"] == 1.15
    assert d["qr"]["target_m"] == 2.2 and d["corridor"]["nav_height_m"] == 3.0
    params = A.mission_params({}, {"takeoff_alt": 5.0, "search_alt": 10.0,
                                   "drop_alt": 5.0})
    assert params["corridor_alt"] == 3.0 and params["return_lane_offset_m"] == -4.0
    assert params["takeoff_alt"] == 5.0 and params["search_alt"] == 10.0


def test_each_dimension_sets_the_parameter_it_means():
    dims = {"corridor": {"length_m": 5.0, "nav_height_m": 2.438,
                         "wall_height_m": 3.048, "spacing_m": 5.0,
                         "return_side": "left"},
            "banner": {"width_m": 3.0, "height_m": 1.0, "placement": "ground"},
            "qr": {"target_m": 1.0},
            "payload": {"length_m": 0.10, "width_m": 0.055}}
    p = A.mission_params(dims, {})
    assert p == {"corridor_alt": 2.438, "gate_advance_m": 5.0,
                 "return_lane_offset_m": 5.0, "banner_w_m": 3.0,
                 "banner_h_m": 1.0, "target_marker_m": 1.0,
                 "banner_bottom_m": 2.805, "start_marker_m": 2.2,
                 "corridor_wall_m": 3.048,
                 "payload_size_m": 0.10, "banner_placement": "ground"}


def test_a_low_ceiling_caps_every_altitude_a_metre_under_it():
    p = A.mission_params({"hall": {"ceiling_m": 6.0}},
                         {"takeoff_alt": 5.0, "search_alt": 10.0, "drop_alt": 5.0})
    assert (p["takeoff_alt"], p["search_alt"], p["drop_alt"]) == (5.0, 5.0, 5.0)


@pytest.mark.parametrize("dims,why", [
    ({"corridor": {"nav_height_m": 3.7}}, "below the corridor wall"),
    ({"corridor": {"width_m": 5.0}}, "spacing must be at least"),
    ({"qr": {"target_m": 0.01}}, "outside"),
    ({"qr": {"target_m": "big"}}, "must be a number"),
    ({"qr": {"target_m": True}}, "must be a number"),
    ({"qr": {"target_m": float("nan")}}, "outside"),
    ({"banner": {"placement": "sideways"}}, "placement"),
    ({"corridor": {"return_side": "up"}}, "return side"),
    ({"banner": {"border_m": 0.6}}, "outside"),
    ({"hall": {"ceiling_m": 2.5}}, "outside"),
])
def test_a_bad_value_is_refused_with_its_reason(dims, why):
    with pytest.raises(ValueError, match=why):
        A.validate(dims)


def test_an_unknown_field_is_refused_rather_than_ignored():
    with pytest.raises(ValueError, match="unknown dimension"):
        A.validate({"qr": {"colour": "black"}})
    with pytest.raises(ValueError, match="unknown dimension group"):
        A.validate({"tent": {}})


def test_every_builtin_preset_is_valid_and_the_clarified_one_is_the_rulebook():
    for name, dims in A.BUILTIN_PRESETS.items():
        A.validate(dims)
    sae = A.validate(A.BUILTIN_PRESETS["SAE Aerothon 2026 (clarified)"])
    assert sae["qr"]["target_m"] == 1.0
    assert sae["corridor"]["wall_height_m"] == pytest.approx(3.048)
    assert sae["corridor"]["nav_height_m"] == pytest.approx(2.438, abs=1e-3)
    assert sae["red_zone"]["side_m"] == pytest.approx(1.524)
    assert sae["banner"]["border_m"] == pytest.approx(0.0762)
    # Hung from a rod between the walls: within them, and below the
    # navigation height, so the crossing must duck under it.
    b, c = sae["banner"], sae["corridor"]
    assert b["suspended_bottom_m"] + b["height_m"] == pytest.approx(c["wall_height_m"])
    assert b["suspended_bottom_m"] < c["nav_height_m"]
    assert b["width_m"] == c["width_m"]
    assert sae["payload"]["width_m"] == pytest.approx(0.055)


# ---- the GCS -------------------------------------------------------------- #

@pytest.fixture
def aggregator(tmp_path):
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.create_client'), \
         patch('rclpy.node.Node.create_timer'), \
         patch('rclpy.node.Node.declare_parameter'), \
         patch('rclpy.node.Node.get_parameter',
               return_value=MagicMock(value=str(tmp_path/'presets.json'))), \
         patch('rclpy.node.Node.get_logger'):
        from gcs_aggregator.aggregator import Aggregator
        node = Aggregator()
    node.pub_arena_dimensions = MagicMock()
    node.get_logger = MagicMock()
    return node


def test_the_gcs_sends_validated_dimensions_and_refuses_bad_ones(aggregator):
    ok, _ = aggregator._dispatch("set_arena_dimensions",
                                 {"dimensions": {"qr": {"target_m": 1.0}}})
    assert ok == "accepted"
    sent = json.loads(aggregator.pub_arena_dimensions.publish.call_args[0][0].data)
    assert sent["dimensions"]["qr"]["target_m"] == 1.0
    result, reason = aggregator._dispatch(
        "set_arena_dimensions", {"dimensions": {"corridor": {"nav_height_m": 9.0}}})
    assert result == "rejected" and "outside" in reason


def test_the_gcs_will_not_change_dimensions_in_the_air(aggregator):
    aggregator.state["mission"]["armed"] = True
    result, reason = aggregator._dispatch("set_arena_dimensions", {"dimensions": {}})
    assert result == "rejected" and "disarmed" in reason
    aggregator.pub_arena_dimensions.publish.assert_not_called()


def test_presets_are_saved_listed_and_deleted_and_survive_a_restart(aggregator, tmp_path):
    dims = {"qr": {"target_m": 1.0}}
    assert aggregator._dispatch("save_dimension_preset",
                                {"name": "Practice hall", "dimensions": dims})[0] == "accepted"
    saved = aggregator.state["dimension_presets"]["saved"]
    assert saved["Practice hall"]["qr"]["target_m"] == 1.0
    on_disk = json.loads((tmp_path/'presets.json').read_text())
    assert "Practice hall" in on_disk
    # A fresh aggregator reads it back.
    assert aggregator._load_presets()["saved"]["Practice hall"]["qr"]["target_m"] == 1.0
    assert aggregator._dispatch("delete_dimension_preset",
                                {"name": "Practice hall"})[0] == "accepted"
    assert "Practice hall" not in aggregator.state["dimension_presets"]["saved"]


def test_builtin_presets_cannot_be_overwritten_or_deleted(aggregator):
    name = "SAE Aerothon 2026 (clarified)"
    assert aggregator._dispatch("save_dimension_preset",
                                {"name": name, "dimensions": {}})[0] == "rejected"
    assert aggregator._dispatch("delete_dimension_preset", {"name": name})[0] == "rejected"
    assert name in aggregator.state["dimension_presets"]["builtin"]


def test_an_invalid_preset_is_not_saved(aggregator, tmp_path):
    result, _ = aggregator._dispatch("save_dimension_preset", {
        "name": "bad", "dimensions": {"qr": {"target_m": -1}}})
    assert result == "rejected" and not (tmp_path/'presets.json').exists()


def test_a_corrupt_preset_file_does_not_take_the_others_down(aggregator, tmp_path):
    (tmp_path/'presets.json').write_text(json.dumps(
        {"good": {"qr": {"target_m": 1.0}}, "broken": {"qr": {"target_m": "x"}}}))
    saved = aggregator._load_presets()["saved"]
    assert list(saved) == ["good"]


# ---- the mission ---------------------------------------------------------- #

class Node:
    def __init__(self):
        self.subs, self.pubs = {}, {}
        self.logger = MagicMock()

    def create_publisher(self, _type, topic, _qos):
        pub = MagicMock()
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, _type, topic, callback, _qos):
        self.subs[topic] = callback

    def get_logger(self):
        return self.logger


def wired(state="WAITING", armed=False):
    from mission_bt.mission_tree import attach_arena_dimensions
    node, mav = Node(), MagicMock()
    mav.state.armed = armed
    built = []
    old = MagicMock()
    holder = {"tree": old}

    def build(params):
        built.append(params)
        return MagicMock()
    attach_arena_dimensions(node, mav, {"takeoff_alt": 5.0, "search_alt": 10.0,
                                        "drop_alt": 5.0, "corridor_alt": 3.0},
                            holder, build, {"value": state})
    return node, built, old


def send(node, dims):
    msg = MagicMock()
    msg.data = json.dumps({"dimensions": dims})
    node.subs["/mission/arena_dimensions"](msg)
    return json.loads(node.pubs["/mission/arena_dimensions/applied"]
                      .publish.call_args[0][0].data)


def test_on_the_ground_the_mission_rebuilds_with_the_new_dimensions():
    node, built, old = wired()
    reply = send(node, {"corridor": {"nav_height_m": 2.438, "wall_height_m": 3.048}})
    assert reply["applied"] and built[-1]["corridor_alt"] == 2.438
    old.shutdown.assert_called_once()


@pytest.mark.parametrize("state,armed", [("CORRIDOR_NAV", True), ("TAKEOFF", False),
                                         ("WAITING", True)])
def test_in_flight_the_mission_keeps_the_dimensions_it_took_off_with(state, armed):
    node, built, old = wired(state, armed)
    reply = send(node, {"qr": {"target_m": 1.0}})
    assert not reply["applied"] and "land first" in reply["reason"]
    assert built == [] and not old.shutdown.called


def test_the_mission_refuses_what_the_gcs_would_have_refused():
    node, built, _ = wired()
    reply = send(node, {"qr": {"target_m": 99.0}})
    assert not reply["applied"] and built == []
