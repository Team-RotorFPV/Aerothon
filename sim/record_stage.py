#!/usr/bin/env python3
"""Record what the aircraft sees and does DURING one mission stage.

WHY THIS IS A TOOL AND NOT A SCRATCH SCRIPT

    Five confident causal stories died to measurement in this project --
    harness contamination, a receding approach target, yaw authority, corridor
    rotation, and the two written up as retractions in VERIFICATION.md 12.3
    and 12.4. Every one was plausible and every one was wrong.

    What killed each of them was recording the system in the state the failure
    actually occurs in. That is not a figure of speech. The red-zone diagnosis
    inverted completely between two runs of the same probe:

        probing whenever it happened to look, which caught the aircraft on the
        pad:            "altitude -0.0 m too low to project"
                        -> "the detector never works"      (WRONG)

        probing only while the mission reported SEARCH_QR, at 10 m with the
        camera settled at -91 deg:
                        "RED RED  excl=198  red_frac=0.47"
                        -> "the detector works; the plan ignores it"

    The unqualified probe was one edit away from being filed as a fourth wrong
    diagnosis. So the stage gate is a REQUIRED argument here: there is no way
    to run this tool without saying what state the answer is about.

WHAT IT RECORDS

    Per sample, gated on the mission stage:

      pose      where the aircraft is, and how high
      camera    commanded and achieved pitch, and whether it has settled
      redzone   the tri-state, the reason, and the confirmed exclusion count
      banner    identified or not, what was read, and by which path
      qr        accepted payload and whether it matched

    And on exit, the thing the red-zone claim actually rests on: whether the
    aircraft's own recorded track ever entered a confirmed exclusion.

USAGE

    source /opt/ros/jazzy/setup.bash
    python3 sim/record_stage.py --stage SEARCH_QR
    python3 sim/record_stage.py --stage CORRIDOR_NAV --timeout 300
    python3 sim/record_stage.py --stage ANY --interval 1.0 --csv /tmp/run.csv
"""

import argparse
import csv
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

sys.path.insert(0, __file__.rsplit("/", 2)[0]
                + "/src/aerothon_mission/mission_bt")


def _clearance_breaches(track, exclusions, clearance_m):
    """Samples of the flown track that sat inside a confirmed exclusion.

    The airframe, not the camera. A red zone constrains where the aircraft may
    be, not what the lens may see -- seeing it is how it got mapped.
    """
    from mission_bt.search_planner import point_in_exclusion
    return [(x, y) for x, y, _ in track
            if point_in_exclusion(x, y, clearance_m, exclusions)]


class Recorder(Node):
    def __init__(self, stage, interval, clearance_m):
        super().__init__("stage_recorder")
        self.stage = stage
        self.interval = float(interval)
        self.clearance_m = float(clearance_m)
        self.s = {}
        self.track = []
        self.exclusions = []
        self.rows = []
        self._last = 0.0

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose, qos_profile_sensor_data)
        for topic, key in (("/camera/pose_state", "cam"),
                           ("/percep/redzone/detail", "red"),
                           ("/percep/banner/detail", "banner"),
                           ("/percep/qr/detail", "qr"),
                           # What the aircraft believes about its angle to the
                           # banner. The stage decides on this one number, so a
                           # run artifact that does not carry it cannot say why
                           # the stage did what it did.
                           ("/mission/square_on", "square"),
                           ("/mission/state", "state")):
            self.create_subscription(
                String, topic, lambda m, k=key: self.s.update({k: m.data}), 10)

    def _pose(self, m):
        p = m.pose.position
        self.s["xyz"] = (p.x, p.y, p.z)

    def _j(self, key):
        try:
            return json.loads(self.s.get(key) or "{}")
        except (ValueError, TypeError):
            return {}

    def in_stage(self):
        if self.stage == "ANY":
            return True
        return self.s.get("state") == self.stage

    def sample(self):
        now = time.time()
        if now - self._last < self.interval:
            return
        self._last = now
        if not self.in_stage() or "xyz" not in self.s:
            return

        x, y, z = self.s["xyz"]
        cam, red = self._j("cam"), self._j("red")
        ban, qr = self._j("banner"), self._j("qr")
        sq = self._j("square")
        ex = [tuple(v) for v in (red.get("exclusions") or []) if len(v) == 4]
        if len(ex) >= len(self.exclusions):
            self.exclusions = ex
        self.track.append((x, y, z))

        row = {
            "t": round(now - self.rows[0]["_t0"], 1) if self.rows else 0.0,
            "_t0": self.rows[0]["_t0"] if self.rows else now,
            "stage": self.s.get("state", "?"),
            "x": round(x, 2), "y": round(y, 2), "alt": round(z, 2),
            "cam_deg": round(math.degrees(float(cam.get("actual_rad", 0.0))), 1),
            "cam_settled": bool(cam.get("settled")),
            "red": red.get("status", "?"),
            "red_reason": str(red.get("reason", ""))[:44],
            "excl": len(ex),
            "red_area_m2": red.get("confirmed_area_m2", 0.0),
            "banner": bool(ban.get("identified")),
            "banner_text": ban.get("text", ""),
            "banner_via": ban.get("lettering_path", ""),
            "banner_reason": str(ban.get("reason", ""))[:44],
            "qr": qr.get("accepted", ""),
            "qr_matched": bool(qr.get("matched")),
            "sq_ok": sq.get("ok"),
            "sq_angle_deg": sq.get("angle_deg"),
            "sq_standoff_m": sq.get("standoff_m"),
            "sq_points": sq.get("points"),
            "sq_reason": str(sq.get("reason", ""))[:52],
        }
        self.rows.append(row)
        print(f"{row['stage']:12s} alt={row['alt']:5.1f} "
              f"cam={row['cam_deg']:6.1f} settled={row['cam_settled']!s:5s} | "
              f"RED {row['red']:11s} excl={row['excl']:4d} | "
              f"BANNER {row['banner']!s:5s} {row['banner_text']:10s} "
              f"{row['banner_via']:10s} | QR {row['qr'][:18]:18s} "
              f"{'MATCH' if row['qr_matched'] else ''}", flush=True)
        if sq:
            print(f"{'':12s} SQUARE ok={row['sq_ok']!s:5s} "
                  f"angle={row['sq_angle_deg']!s:>7s} deg  "
                  f"standoff={row['sq_standoff_m']!s:>6s} m  "
                  f"pts={row['sq_points']!s:>4s}  {row['sq_reason']}",
                  flush=True)

    def report(self):
        print("\n" + "=" * 74)
        if not self.rows:
            print(f"*** never observed stage {self.stage} -- "
                  f"NOTHING WAS MEASURED ***")
            print("Do not draw a conclusion from this run. A probe that never "
                  "saw the stage\nis not evidence that the stage is fine.")
            return 1
        print(f"{len(self.rows)} sample(s) in stage {self.stage}")
        alts = [r["alt"] for r in self.rows]
        print(f"altitude      {min(alts):.1f} .. {max(alts):.1f} m")
        print(f"exclusions    {self.rows[0]['excl']} -> "
              f"{self.rows[-1]['excl']} confirmed")
        reds = {r["red"] for r in self.rows}
        print(f"red-zone      {', '.join(sorted(reds))}")

        if not self.exclusions:
            print("\nNo exclusions were ever confirmed, so this run says "
                  "nothing about\navoidance either way.")
            return 0

        breaches = _clearance_breaches(self.track, self.exclusions,
                                       self.clearance_m)
        print(f"\nAIRFRAME vs {len(self.exclusions)} confirmed exclusion(s), "
              f"{self.clearance_m:.1f} m clearance:")
        if breaches:
            print(f"  VIOLATION: {len(breaches)} of {len(self.track)} track "
                  f"samples were inside a restricted zone")
            for x, y in breaches[:6]:
                print(f"    ({x:.1f}, {y:.1f})")
            return 2
        print(f"  clear: none of {len(self.track)} track samples entered one")
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # REQUIRED, deliberately. See the module docstring: an ungated probe
    # produced a confident wrong answer about this exact subsystem.
    ap.add_argument("--stage", required=True,
                    help="mission state to record in, or ANY "
                         "(e.g. SEARCH_QR, CORRIDOR_NAV, BANNER_ALIGN)")
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--clearance", type=float, default=1.5,
                    help="airframe half-span plus margin, metres")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()

    rclpy.init()
    rec = Recorder(a.stage, a.interval, a.clearance)
    t0 = time.time()
    try:
        while time.time() - t0 < a.timeout:
            rclpy.spin_once(rec, timeout_sec=0.2)
            rec.sample()
    except KeyboardInterrupt:
        pass

    rc = rec.report()
    if a.csv and rec.rows:
        cols = [k for k in rec.rows[0] if not k.startswith("_")]
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rec.rows)
        print(f"\nwrote {a.csv}")
    rec.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
