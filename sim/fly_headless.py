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
import math
import os
import signal
from sensor_cadence import CAMERA_CAPTURE_HZ, LIDAR_SCAN_HZ
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "scripts"))

import world_spec as W                                      # noqa: E402
from campaign_record import navigation_grade, source_manifest
from headless_world import organiser_inputs                 # noqa: E402

PKGS = [("aerothon_mission", "mission_bt"), ("aerothon_avoidance", "avoidance"),
        ("aerothon_perception", "camera_ctrl"), ("aerothon_payload", "winch_ctrl"),
        ("aerothon_perception", "perception_qr"),
        ("aerothon_perception", "perception_redzone"),
        ("aerothon_perception", "perception_banner"), ("aerothon_sim", "sim_gazebo"),
        ("aerothon_gcs", "gcs_aggregator")]
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


def fly(spec_path, conditions, seed, out_root, rtf, max_sim_s, domain, navigation_only=False,
        injected_command_delay_s=.4, gcs_preset=None, gcs_create_from=None,
        gcs_create_from_file=None, fence_only=None):
    if not math.isfinite(injected_command_delay_s) or injected_command_delay_s<0:
        raise ValueError('Injected command delay must be finite and nonnegative')
    spec = W.load(spec_path)
    name = f"{spec_path.stem}_{conditions or 'spec'}_s{seed}"
    out = out_root / name
    out.mkdir(parents=True, exist_ok=False)
    (out/'world_spec.json').write_bytes(spec_path.read_bytes())
    (out/'configuration.json').write_text(json.dumps(dict(world=str(spec_path),seed=seed,
        conditions=conditions,rtf=rtf,max_sim_s=max_sim_s,ros_domain=domain,
        fidelity='object-level navigation; no rendered images or motor telemetry',
        navigation_only=navigation_only,delivery_exercised=not navigation_only,
        vel_command_delay_s=injected_command_delay_s,vel_response_s=0.6,
        injected_command_delay_s=injected_command_delay_s,
        artificial_transport_delay_injected=injected_command_delay_s>0,
        arena_dimensions=('GCS preset ' + repr(gcs_preset)) if gcs_preset
            else 'launch parameters from the world spec',
        camera_capture_hz=CAMERA_CAPTURE_HZ,lidar_scan_hz=LIDAR_SCAN_HZ,
        fence_only=fence_only),indent=2))
    (out/'source_manifest.json').write_text(json.dumps(source_manifest(ROOT),indent=2))
    zone, fence = organiser_inputs(spec)
    # The judges' polygon alone, local ENU "x,y;x,y;...": no delivery-zone
    # corners reach the aircraft, as at the competition.
    fence_env = {"AEROTHON_FENCE_ONLY": "1"} if fence_only else {}
    fence = fence_only or fence
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               AEROTHON_DELIVERY_ZONE=zone, AEROTHON_GEOFENCE=fence, **fence_env,
               AEROTHON_GCS_WS_PORT=str(18700 + domain),
               PYTHONPATH=os.pathsep.join(
                   [str(ROOT / "src" / a / b) for a, b in PKGS]
                   + [os.environ.get("PYTHONPATH", "")]))
    world = [sys.executable, str(ROOT / "sim/headless_world.py"), str(spec_path),
             "--seed", str(seed), "--out", str(out), "--rtf", str(rtf),
             "--max-sim-s", str(max_sim_s), "--vel-command-delay-s", str(injected_command_delay_s)]
    if conditions:
        world += ["--conditions", conditions]
    # THE OPERATOR'S PATH. With a preset, the arena's dimensions reach the
    # aircraft only as the competition delivers them: typed at the GCS, saved,
    # applied over the aggregator before START -- never as launch parameters.
    mission_params = {"camera_hfov": 0.851919}
    # A frozen load profile, as the launch file passes it on the aircraft.
    # Without one the drop is refused (fail closed) and no return lap flies.
    if os.environ.get("AEROTHON_DELIVERY_CALIBRATION"):
        mission_params["delivery_load_calibration"] = os.environ["AEROTHON_DELIVERY_CALIBRATION"]
    gcs = []
    if gcs_preset:
        world += ["--start-after-dimensions"]
        gcs = [node_cmd("gcs_aggregator.aggregator", {}),
               [sys.executable, str(ROOT / "scripts/gcs_operator.py"),
                "--preset", gcs_preset, "--record", str(out / "gcs_dimensions.json")]
               + (["--create-from", gcs_create_from] if gcs_create_from else [])
               + (["--create-from-file", str(gcs_create_from_file)]
                  if gcs_create_from_file else [])]
    else:
        mission_params["target_marker_m"] = float(spec["qr"]["target_m"])
    stack = [
        [sys.executable,str(ROOT/'sim/record_delivery.py'),'--out',str(out),
         '--timeout',str(max_sim_s/max(.2,rtf)*3+120)],
        node_cmd("sim.navigation_mission" if navigation_only else "mission_bt.mission_tree",
                 mission_params),
        node_cmd("avoidance.velocity_controller", {"scan_topic": "/scan",
                                                  "vel_command_delay_s": float(injected_command_delay_s)}),
        node_cmd("camera_ctrl.camera_ctrl_node", {"backend": "sim"}),
        node_cmd("winch_ctrl.winch_node", {"backend": "gazebo"}),
        [sys.executable, str(ROOT / "scripts/publish_delivery_zone.py")],
    ] + gcs
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
    nav={}
    if (out/'track.csv').exists() and (out/'layout.json').exists():
        from check_track import load_track
        # The world trajectory is simulator truth, kept separate from onboard.jsonl.
        (out/'evaluation_truth_track.csv').write_bytes((out/'track.csv').read_bytes())
        nav=navigation_grade(load_track(str(out/'track.csv')),
            json.loads((out/'layout.json').read_text()),run.get('start_target','a'),
            independent_contacts=run.get('contacts'))
        (out/'navigation_grade.json').write_text(json.dumps(nav,indent=2))
    grade = subprocess.run(
        [sys.executable, str(ROOT / "sim/check_track.py"), "--track", str(out / "track.csv"),
         "--layout", str(out / "layout.json"), "--target", run.get("start_target", "a")],
        capture_output=True, text=True)
    (out / "grade.txt").write_text(grade.stdout + grade.stderr)
    result = (run.get("result") or {}).get("state", "NO_OUTCOME")
    fails = [line.strip()[7:] for line in grade.stdout.splitlines() if "[FAIL]" in line]
    return {"run": name, "result": result, "grade": "PASS" if grade.returncode == 0 else "FAIL",
            "navigation_pass":result in ('COMPLETED','DELIVERY_UNCONFIRMED') and nav.get('pass_navigation',False),
            "delivery_exercised":not navigation_only,
            "navigation_failures":[k for k,v in nav.get('checks',{}).items() if not v],
            "rendered_perception_qualified":False,"motor_delivery_qualified":False,
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
    ap.add_argument('--navigation-only',action='store_true',help='omit unqualified delivery in object fixture; never qualifies a full mission')
    ap.add_argument("--max-sim-s", type=float, default=1200.0)
    ap.add_argument("--domain-base", type=int, default=100,
                    help="first ROS domain; give concurrent campaigns disjoint "
                         "ranges or their runs hear each other")
    ap.add_argument("--out", type=Path, default=ROOT / "logs" / "headless")
    ap.add_argument("--gcs-preset", default=None,
                    help="apply the arena dimensions through the GCS from this "
                         "saved preset before START (the competition's path)")
    ap.add_argument("--gcs-create-from", default=None,
                    help="built-in preset to save --gcs-preset from if missing")
    ap.add_argument("--gcs-create-from-file", type=Path, default=None,
                    help="dimensions JSON to save --gcs-preset from if missing")
    ap.add_argument("--fence-only", default=None, metavar="X,Y;X,Y;...",
                    help="give the aircraft only this local geofence polygon "
                         "(the judges' points), no delivery-zone corners")
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
                           args.max_sim_s, args.domain_base + iv[0] % 100,args.navigation_only,
                           gcs_preset=args.gcs_preset, gcs_create_from=args.gcs_create_from,
                           gcs_create_from_file=args.gcs_create_from_file,
                           fence_only=args.fence_only),
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
