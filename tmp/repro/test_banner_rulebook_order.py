import math
import os
import sys

import py_trees

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "src", "aerothon_mission", "mission_bt"))
sys.path.insert(0, os.path.join(REPO, "sim"))

from test_banner_sweep import Clock, GateMav
from mission_bt.mission_tree import AlignToBanner


def test_alignment_leaf_does_not_descend_before_it_succeeds():
    """Rulebook: identify and align before descending to corridor altitude."""
    clock = Clock()
    mav = GateMav(
        gate=(6.0, 0.0),
        face_rad=math.pi,
        start=(0.0, 0.0, 5.0),
        gate_top_m=4.0,
    )
    stage = AlignToBanner(
        mav,
        clock=clock,
        dwell_s=0.3,
        align_dwell_s=0.2,
        hfov_rad=mav.hfov,
        alt_floor_m=3.0,
    )
    stage.initialise()

    for _ in range(1_000):
        status = stage.update()
        clock.advance(0.1)
        lower_commands = [command for command in mav.gotos if command[2] < 4.9]
        if lower_commands or status is not py_trees.common.Status.RUNNING:
            break

    assert not lower_commands, (
        "AlignToBanner commanded a descent before reporting alignment success: "
        f"{lower_commands[0]}"
    )
