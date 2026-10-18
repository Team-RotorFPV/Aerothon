#!/usr/bin/env python3
"""The GCS operator's arena-dimension steps, scripted for simulation.

At the competition the organisers give the arena's dimensions on the day and
the operator types them into the GCS -- once, saved as a preset -- and applies
them before pressing START. A simulated run does exactly that, over the same
WebSocket the GCS app uses, so the dimensions reach the aircraft by the
operator's path and no other:

    1. connect to the aggregator;
    2. if the named preset is not saved yet, save it (from a built-in preset
       or a JSON file) -- what the operator does the first time;
    3. load it and send `set_arena_dimensions`;
    4. wait until mission_bt reports the dimensions APPLIED, and refuse to
       report success on anything less.

    python3 scripts/gcs_operator.py --preset "Aerothon 2026 sim" \
        --create-from "SAE Aerothon 2026 (clarified)" --record out/gcs_dimensions.json

Exit status 0 only when the aircraft confirmed the dimensions it was sent.
"""
import argparse
import asyncio
import json
import os
import sys
import time

import websockets


def command(cmd, args, cmd_id):
    return json.dumps({"v": 1, "kind": "command",
                       "data": {"cmd": cmd, "cmd_id": cmd_id, "args": args}})


def close(a, b, tol=1e-6):
    """Same dimensions, field by field (numbers to `tol`)."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


class Operator:
    def __init__(self, ws, log):
        self.ws, self.log = ws, log
        self.state = {}
        self.acks = {}
        self._n = 0

    async def pump(self, until, timeout):
        """Read messages until `until()` is true; False on timeout."""
        end = time.monotonic() + timeout
        while not until():
            left = end - time.monotonic()
            if left <= 0:
                return False
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=left)
            except asyncio.TimeoutError:
                return False
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if msg.get("kind") == "telemetry":
                self.state = msg.get("data") or {}
            elif msg.get("kind") == "ack":
                d = msg.get("data") or {}
                self.acks[d.get("cmd_id")] = d
        return True

    async def send(self, cmd, args, timeout=10.0):
        self._n += 1
        cid = f"op{self._n}"
        await self.ws.send(command(cmd, args, cid))
        if not await self.pump(lambda: cid in self.acks, timeout):
            raise RuntimeError(f"{cmd}: no acknowledgement in {timeout:.0f} s")
        ack = self.acks[cid]
        if ack.get("result") != "accepted":
            raise RuntimeError(f"{cmd} rejected: {ack.get('reason')}")
        self.log(f"{cmd}: accepted")

    def presets(self):
        p = self.state.get("dimension_presets") or {}
        return p.get("builtin") or {}, p.get("saved") or {}


async def run(a):
    log = (lambda s: print(f"[gcs_operator] {s}", flush=True))
    end = time.monotonic() + a.connect_timeout
    while True:
        try:
            ws = await websockets.connect(a.url, max_size=None, open_timeout=5)
            break
        except (OSError, asyncio.TimeoutError, websockets.WebSocketException) as error:
            if time.monotonic() > end:
                raise RuntimeError(f"no GCS aggregator at {a.url}: {error}") from None
            await asyncio.sleep(1.0)
    try:
        op = Operator(ws, log)
        if not await op.pump(lambda: "dimension_presets" in op.state, 30.0):
            raise RuntimeError("the aggregator sent no state with dimension presets")
        builtin, saved = op.presets()

        # The first time: enter the dimensions and save them under a name.
        if a.preset not in saved:
            if a.create_from_file:
                with open(a.create_from_file, encoding="utf-8") as f:
                    dims = json.load(f)
                dims = dims.get("dimensions", dims)
                source = a.create_from_file
            elif a.create_from in builtin:
                dims, source = builtin[a.create_from], f"built-in {a.create_from!r}"
            else:
                raise RuntimeError(f"no saved preset {a.preset!r} and nothing to create it from")
            await op.send("save_dimension_preset", {"name": a.preset, "dimensions": dims})
            if not await op.pump(lambda: a.preset in op.presets()[1], 10.0):
                raise RuntimeError(f"preset {a.preset!r} saved but not listed")
            log(f"saved preset {a.preset!r} from {source}")
        dims = op.presets()[1][a.preset]

        # Every run: load the preset and apply it, then wait for the aircraft.
        await op.send("set_arena_dimensions", {"dimensions": dims})

        def applied():
            d = op.state.get("arena_dimensions") or {}
            return bool(d.get("ok")) and close(d.get("applied"), dims)

        def refused():
            d = op.state.get("arena_dimensions") or {}
            return d.get("ok") is False and bool(d.get("reason"))

        await op.pump(lambda: applied() or refused(), a.apply_timeout)
        result = op.state.get("arena_dimensions") or {}
        record = {"preset": a.preset, "url": a.url, "sent": dims,
                  "applied": result.get("applied"), "params": result.get("params"),
                  "ok": applied(), "reason": result.get("reason", "")}
        if a.record:
            os.makedirs(os.path.dirname(os.path.abspath(a.record)), exist_ok=True)
            with open(a.record, "w", encoding="utf-8") as f:
                json.dump(record, f, indent=2)
        if not applied():
            raise RuntimeError("the aircraft did not apply the dimensions: "
                               + (result.get("reason") or "no confirmation"))
        log(f"aircraft applied {a.preset!r}: {json.dumps(result.get('params'))}")
    finally:
        await ws.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    port = os.environ.get("AEROTHON_GCS_WS_PORT", "8765")
    ap.add_argument("--url", default=f"ws://127.0.0.1:{port}")
    ap.add_argument("--preset", required=True, help="saved operator preset to apply")
    ap.add_argument("--create-from", default="SAE Aerothon 2026 (clarified)",
                    help="built-in preset to save it from when it does not exist yet")
    ap.add_argument("--create-from-file", help="JSON dimensions to save it from instead")
    ap.add_argument("--record", help="write what was sent and applied here")
    ap.add_argument("--connect-timeout", type=float, default=120.0)
    ap.add_argument("--apply-timeout", type=float, default=60.0)
    a = ap.parse_args(argv)
    try:
        asyncio.run(run(a))
    except RuntimeError as error:
        print(f"[gcs_operator] FAILED: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
