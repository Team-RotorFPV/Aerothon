#!/usr/bin/env python3
"""Fly the real Mission 2 stack in the headless world, then grade it.

    python3 sim/fly_headless.py sim/worlds/my_world.json
    python3 sim/fly_headless.py sim/worlds/my_world.json --conditions worst --seed 3
    python3 sim/fly_headless.py sim/worlds/*.json --conditions random --seeds 1-20 -j 3

Each run gets its own ROS domain, so runs go in parallel (-j). The mission
tree, corridor navigator, camera controller and winch controller are the
real nodes, on simulated time; sim/headless_world.py plays everything else.
Each run's outputs land in --out/NAME_CONDITIONS_sSEED/ and are graded by
sim/check_track.py, the same grader as a Gazebo run. A summary table closes.

Exit 0 only if every run completed AND passed its grade.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "scripts"))

import world_spec as W                                      # noqa: E402
from headless_world import organiser_inputs                 # noqa: E402

PKGS = [("aerothon_mission", "mission_bt"), ("aerothon_avoidance", "avoidance"),
        ("aerothon_perception", "camera_ctrl"), ("aerothon_payload", "winch_ctrl"),
        ("aerothon_perception", "perception_qr"),
        ("aerothon_perception", "perception_redzone"),
        ("aerothon_perception", "perception_banner"), ("aerothon_sim", "sim_gazebo")]
LIVE = set()          # process groups of every run in flight


def stop_all(signum, _frame):
    """Killed mid-campaign: take every run's processes with us. Left behind,
    they keep flying on their ROS domain and join the next campaign's runs."""
    for pid in list(LIVE):
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)


def node_cmd(module, params):
    args = ["--ros-args", "-p", "use_sim_time:=true"]
    for k, v in params.items():
        args += ["-p", f"{k}:={v}"]
    return [sys.executable, "-c", f"from {module} import main; main()"] + args


def fly(spec_path, conditions, seed, out_root, rtf, max_sim_s, domain):
    spec = W.load(spec_path)
    name = f"{spec_path.stem}_{conditions or 'spec'}_s{seed}"
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)
    zone, fence = organiser_inputs(spec)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               AEROTHON_DELIVERY_ZONE=zone, AEROTHON_GEOFENCE=fence,
               PYTHONPATH=os.pathsep.join(
                   [str(ROOT / "src" / a / b) for a, b in PKGS]
                   + [os.environ.get("PYTHONPATH", "")]))
    world = [sys.executable, str(ROOT / "sim/headless_world.py"), str(spec_path),
             "--seed", str(seed), "--out", str(out), "--rtf", str(rtf),
             "--max-sim-s", str(max_sim_s)]
    if conditions:
        world += ["--conditions", conditions]
    stack = [
        node_cmd("mission_bt.mission_tree", {
            "camera_hfov": 0.851919,
            "target_marker_m": float(spec["qr"]["target_m"])}),
        node_cmd("avoidance.velocity_controller", {"scan_topic": "/scan"}),
        node_cmd("camera_ctrl.camera_ctrl_node", {"backend": "sim"}),
        node_cmd("winch_ctrl.winch_node", {"backend": "gazebo"}),
        [sys.executable, str(ROOT / "scripts/publish_delivery_zone.py")],
    ]
    log = open(out / "stack.log", "w", encoding="utf-8")
    t0 = time.monotonic()
    procs = [subprocess.Popen(world, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)]
    time.sleep(1.0)
    procs += [subprocess.Popen(c, env=env, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True) for c in stack]
    LIVE.update(p.pid for p in procs)
    try:
        procs[0].wait(timeout=max_sim_s / max(0.2, rtf) * 3 + 120)
    except subprocess.TimeoutExpired:
        pass
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    LIVE.difference_update(p.pid for p in procs)
    log.close()

    run = json.loads((out / "run.json").read_text()) if (out / "run.json").exists() else {}
    grade = subprocess.run(
        [sys.executable, str(ROOT / "sim/check_track.py"), "--track", str(out / "track.csv"),
         "--layout", str(out / "layout.json"), "--target", run.get("start_target", "a")],
        capture_output=True, text=True)
    (out / "grade.txt").write_text(grade.stdout + grade.stderr)
    result = (run.get("result") or {}).get("state", "NO_OUTCOME")
    fails = [line.strip()[7:] for line in grade.stdout.splitlines() if "[FAIL]" in line]
    return {"run": name, "result": result, "grade": "PASS" if grade.returncode == 0 else "FAIL",
            "fails": fails, "contacts": len(run.get("contacts", [])),
            "fence": run.get("fence_breaches", 0), "sim_s": run.get("sim_s"),
            "wall_s": round(time.monotonic() - t0), "reason":
            ((run.get("result") or {}).get("reason") or "")[:120]}


def seeds(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("specs", nargs="+", type=Path)
    ap.add_argument("--conditions", default=None,
                    help="calm|field|worst|random (default: each spec's own)")
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--seed", dest="seeds")
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("--rtf", type=float, default=2.0)
    ap.add_argument("--max-sim-s", type=float, default=1200.0)
    ap.add_argument("--domain-base", type=int, default=100,
                    help="first ROS domain; give concurrent campaigns disjoint "
                         "ranges or their runs hear each other")
    ap.add_argument("--out", type=Path, default=ROOT / "logs" / "headless")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, stop_all)
    signal.signal(signal.SIGINT, stop_all)
    specs = []
    for path in args.specs:
        errs, warns = W.validate(W.load(path))
        for w in warns:
            print(f"{path.stem}: warning: {w}")
        if errs:
            print(f"{path.stem}: REFUSED: " + "; ".join(errs))
        else:
            specs.append(path)
    runs = [(s, n) for s in specs for n in seeds(args.seeds)]
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(
            lambda iv: fly(iv[1][0], args.conditions, iv[1][1], args.out, args.rtf,
                           args.max_sim_s, args.domain_base + iv[0] % 100),
            enumerate(runs)))
    ok = 0
    print(f"\n{'run':44s} {'result':10s} {'grade':5s} {'hits':>4s} {'fence':>5s} "
          f"{'sim s':>6s}  failing checks / reason")
    for r in results:
        good = r["result"] == "COMPLETED" and r["grade"] == "PASS"
        ok += good
        print(f"{r['run']:44s} {r['result']:10s} {r['grade']:5s} {r['contacts']:4d} "
              f"{r['fence']:5d} {r['sim_s'] or 0:6.0f}  "
              f"{'; '.join(r['fails']) or ('' if good else r['reason'])}")
    print(f"\n{ok}/{len(results)} runs COMPLETED and graded PASS")
    (args.out / "summary.json").write_text(json.dumps(results, indent=1))
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
