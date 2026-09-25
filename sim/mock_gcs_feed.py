#!/usr/bin/env python3
"""Mock GCS feed — drives the GCS with the simulated mission, no ROS.

Serves the gcs_aggregator WebSocket schema on ws://0.0.0.0:8765, replaying
sim/mission_sim.py. Now also sends attitude (roll/pitch/yaw) for the HUD and a
downsampled 2D lidar scan so the GCS can build a live SLAM map.

Run:  python3 sim/mock_gcs_feed.py
"""
import asyncio
import json
import math
import os
import sys
import time

import websockets

sys.path.insert(0, os.path.dirname(__file__))
from mission_sim import simulate, raycast, LIDAR_MAX  # noqa: E402

HIST = simulate()
N = len(HIST)
NRAYS = 72

STATE_MAP = {
    "TAKEOFF": "TAKEOFF", "TO_CORRIDOR": "GOTO_CORRIDOR", "CORRIDOR": "CORRIDOR_NAV",
    "TO_ZONE": "ENTER_ZONE", "SEARCH": "SEARCH_QR", "DROP": "WINCH_DROP",
    "RETURN_TO_CORRIDOR": "RETURN", "CORRIDOR_BACK": "RETURN_CORRIDOR", "LAND": "LAND",
}
ALT_TARGET = {"TAKEOFF": 5, "TO_CORRIDOR": 5, "CORRIDOR": 3, "TO_ZONE": 3,
              "SEARCH": 10, "DROP": 5, "RETURN_TO_CORRIDOR": 10, "CORRIDOR_BACK": 3, "LAND": 0}

# ---- precompute attitude, speeds, smoothed alt, and lidar scans ---- #
def _precompute():
    yaw = [0.0] * N; gs = [0.0] * N; roll = [0.0] * N; pitch = [0.0] * N
    alt_s = [5.0] * N; scans = [None] * N
    for i in range(N):
        pos, state, t, _ = HIST[i]
        if i > 0:
            p0 = HIST[i - 1][0]; dt = max(1e-3, t - HIST[i - 1][2])
            vx, vy = (pos[0] - p0[0]) / dt, (pos[1] - p0[1]) / dt
            sp = math.hypot(vx, vy); gs[i] = sp
            yaw[i] = math.atan2(vy, vx) if sp > 0.15 else yaw[i - 1]
            dyaw = (yaw[i] - yaw[i - 1] + math.pi) % (2 * math.pi) - math.pi
            roll[i] = max(-35, min(35, -22 * dyaw / dt))
            alt_s[i] = alt_s[i - 1] + (ALT_TARGET.get(state, 5) - alt_s[i - 1]) * 0.12
            vs = (alt_s[i] - alt_s[i - 1]) / dt
            pitch[i] = max(-15, min(15, -vs * 7 + (sp - 1.0) * 2))
        # lidar scan (world-frame ray-cast); None == no return (beyond range)
        ranges = []
        for k in range(NRAYS):
            a = -math.pi + 2 * math.pi * k / NRAYS
            r = raycast((pos[0], pos[1]), a)
            ranges.append(None if r >= LIDAR_MAX - 0.01 else round(r, 2))
        scans[i] = ranges
    return yaw, gs, roll, pitch, alt_s, scans

YAW, GS, ROLL, PITCH, ALT_S, SCANS = _precompute()


def checklist(reached):
    order = ["TAKEOFF", "TO_CORRIDOR", "CORRIDOR", "TO_ZONE", "SEARCH",
             "DROP", "CORRIDOR_BACK", "LAND"]
    keys = ["takeoff", "start_qr", "banner", "corridor", "target_id", "drop", "return", "land"]
    return {key: (st in reached) for st, key in zip(order, keys)}


LAT0, LON0 = 12.9716, 77.5946   # takeoff datum
def _to_gps(x, y):
    dlat = y / 111320.0
    dlon = x / (111320.0 * math.cos(math.radians(LAT0)))
    return round(LAT0 + dlat, 7), round(LON0 + dlon, 7)


def frame(i, reached):
    pos, state, t, front = HIST[i]
    lat, lon = _to_gps(pos[0], pos[1])
    matched = state in ("DROP", "RETURN_TO_CORRIDOR", "CORRIDOR_BACK", "LAND")
    return {
        "v": 1, "kind": "telemetry", "t": time.time(),
        "data": {
            "mission": {"selected": "mission2", "state": STATE_MAP.get(state, state),
                        "armed": not (state == "LAND" and i >= N - 1), "mode": "GUIDED",
                        "elapsed": round(t, 1)},
            "flight": {"x": round(float(pos[0]), 2), "y": round(float(pos[1]), 2),
                       "alt": round(ALT_S[i], 2), "gs": round(GS[i], 2),
                       "roll_deg": round(ROLL[i], 1), "pitch_deg": round(PITCH[i], 1),
                       "yaw_deg": round((math.degrees(YAW[i]) + 360) % 360, 1)},
            "gps": {"lat": lat, "lon": lon, "sats": 16, "fix": "3D"},
            "power": {"volt": round(16.0 - 0.6 * (i / N), 2), "pct": round(100 - 30 * (i / N))},
            "nav": {"front_m": round(float(front), 2), "centering_err": 0.0,
                    "cmd_vx": round(GS[i], 2)},
            "percep": {"start_qr": "TGT-A17" if reached != {"TAKEOFF"} else "",
                       "target_match": matched, "banner": state == "CORRIDOR",
                       "redzone_visible": False},
            "safety": {"ready": True, "ekf": True, "geofence": "INSIDE"},
            "checklist": checklist(reached),
            "scan": {"yaw_deg": round((math.degrees(YAW[i]) + 360) % 360, 1),
                     "ranges": SCANS[i]},
        },
    }


async def handler(websocket, *_):
    print("GCS client connected")
    reached = set()

    async def consume():
        try:
            async for raw in websocket:
                msg = json.loads(raw)
                if msg.get("kind") == "command":
                    d = msg["data"]
                    print(f"  << command: {d.get('cmd')} {d.get('args')}")
                    await websocket.send(json.dumps({
                        "v": 1, "kind": "ack", "t": time.time(),
                        "data": {"cmd_id": d.get("cmd_id"), "cmd": d.get("cmd"),
                                 "result": "accepted"}}))
        except Exception:  # noqa: BLE001
            pass

    asyncio.ensure_future(consume())
    try:
        while True:
            for i in range(N):
                reached.add(HIST[i][1])
                await websocket.send(json.dumps(frame(i, set(reached))))
                await asyncio.sleep(0.1)
            await asyncio.sleep(1.2)
            reached.clear()
    except websockets.ConnectionClosed:
        print("GCS client disconnected")


async def main():
    print(f"mock GCS feed on ws://0.0.0.0:8765  ({N} frames w/ attitude+lidar, replay loop)")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
