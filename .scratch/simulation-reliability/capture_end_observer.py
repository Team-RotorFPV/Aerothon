"""Recording helper: keep the existing encoder running until disarm.

Read-only flight telemetry. Only the recorder controller is held/resumed;
the simulator, flight controller, camera GUI and encoder keep running.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
import rclpy
from run_mission_live import MissionRunner


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch-pid", required=True, type=int)
    ap.add_argument("--encoder-pid", required=True, type=int)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()
    cmd = Path(f"/proc/{args.watch_pid}/cmdline").read_bytes()
    if b"scripts/watch_mission_gui.sh" not in cmd:
        raise RuntimeError("PID is not this flight's recording controller")
    rclpy.init(args=["--ros-args", "-r", "__node:=capture_end_observer"])
    n = MissionRunner()
    held = False
    started = time.monotonic()
    outcome = "timeout"
    try:
        while rclpy.ok() and time.monotonic() - started < 5400:
            rclpy.spin_once(n, timeout_sec=0.1)
            encoder_alive = Path(f"/proc/{args.encoder_pid}").exists()
            if not encoder_alive:
                outcome = "encoder stopped; allowing segment recovery"
                break
            if n.state.connected and n.state.armed and not held:
                os.kill(args.watch_pid, signal.SIGSTOP)
                held = True
                print("recording controller held; encoder continues", flush=True)
            if n.result is not None and n.state.connected and not n.state.armed:
                outcome = "terminal mission result and FCU disarmed"
                n.spin(2.0)
                break
    finally:
        if held:
            os.kill(args.watch_pid, signal.SIGCONT)
            print("recording controller resumed", flush=True)
        n.write_track(str(args.out.with_suffix(".csv")))
        p = n.pose.pose.position
        args.out.with_suffix(".json").write_text(json.dumps({
            "capture_end": outcome, "result": n.result,
            "armed": bool(n.state.armed), "mode": n.state.mode,
            "position_local": [p.x, p.y, p.z],
            "sim_s": n.get_clock().now().nanoseconds * 1e-9,
            "samples": len(n.track),
        }, indent=2))
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
