#!/usr/bin/env python3
"""Replay saved claw contact traces without starting Gazebo or hardware.

Exit 1 means at least one saved contact trial failed. This is a simulation
baseline, never evidence of a physical bench pass.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from winch_bench import assess_drop, ring_geometry, LEG_H  # noqa: E402


def audit(trials, payload_height):
    airframe = json.loads((ROOT / "src/aerothon_sim/sim_gazebo/models/"
                           "aerothon_quad/airframe.json").read_text())
    ring = ring_geometry(airframe["claw"])
    # Base-link height with the CAD landing gear resting on a level plane.
    base_height = -airframe["gear_bottom_z"]
    payload_bottom = base_height + ring["z"] - LEG_H - payload_height
    report = {
        "basis": "saved Gazebo contact traces and CAD geometry; physical hardware unverified",
        "payload_height_m": payload_height,
        "simulated_attachment": "horizontal wire ring; real attachment not confirmed",
        "cad_stowed_payload_ground_clearance_m": round(payload_bottom, 6),
        "ground_clearance_positive": payload_bottom > 0,
        "trials": [],
    }
    for trial in trials:
        source = trial / "timeline.csv"
        if not source.is_file():
            raise FileNotFoundError(source)
        with tempfile.TemporaryDirectory(prefix="claw-replay-") as folder:
            replay = Path(folder)
            shutil.copyfile(source, replay / "timeline.csv")
            verdict = assess_drop(replay, payload_height)
        report["trials"].append({"source": str(source), **verdict})
    report["saved_contact_trials_pass"] = all(
        item["result"] == "RELEASED_ON_GROUND" for item in report["trials"])
    report["physical_bench_status"] = "NOT_TESTED"
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", nargs="*", type=Path,
                        default=[ROOT / "logs/winch_ring", ROOT / "logs/winch_ring_hung"])
    parser.add_argument("--payload-height-m", type=float, default=0.05)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if not 0 < args.payload_height_m < 1:
        parser.error("payload height must be between 0 and 1 metre")
    report = audit(args.trials, args.payload_height_m)
    rendered = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
    print(rendered, end="")
    return 0 if report["saved_contact_trials_pass"] and report["ground_clearance_positive"] else 1


if __name__ == "__main__":
    sys.exit(main())
