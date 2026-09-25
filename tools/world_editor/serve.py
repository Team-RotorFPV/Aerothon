#!/usr/bin/env python3
"""Serve the world editor, keep its worlds, and fly them.

    python3 tools/world_editor/serve.py            # http://127.0.0.1:8777/

Run it inside WSL (the simulator lives there); the Windows browser reaches it
at the same address. Standard library only.

    GET  /                      the editor
    GET  /api/worlds            saved worlds (sim/worlds/*.json)
    GET  /api/worlds/NAME       one world
    POST /api/worlds/NAME       save (validated; refused worlds are still saved
                                and reported, so work is never lost)
    POST /api/validate          errors and warnings from scripts/world_spec.py
    POST /api/run/NAME?gui=1&record=1
                                fly NAME with scripts/run_custom_world.sh
    GET  /api/run               the run in progress, or the last one
"""

import argparse
import importlib
import json
import re
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
WORLDS = ROOT / "sim" / "worlds"
LOGS = ROOT / "logs" / "custom"
sys.path.insert(0, str(ROOT / "scripts"))
import world_spec  # noqa: E402

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
LIVE_LOG = Path("/tmp/aerothon_live_mission.log")
RUN = {"proc": None, "name": "", "started": 0.0}


def validate(spec):
    """world_spec's verdict, from the file as it is NOW.

    Imported once, a rule added to world_spec.py while the server ran never
    reached it: the server passed a world that run_custom_world.sh (a fresh
    import) then refused, and the editor sat on "starting the simulator".
    """
    importlib.reload(world_spec)
    return world_spec.validate(spec)


def tail(path, n=8):
    try:
        return path.read_text(errors="replace").splitlines()[-n:]
    except OSError:
        return []


def last_line(path, needle):
    try:
        lines = [l for l in path.read_text(errors="replace").splitlines() if needle in l]
    except OSError:
        return ""
    return lines[-1] if lines else ""


def run_status():
    p, name = RUN["proc"], RUN["name"]
    out = {"running": bool(p and p.poll() is None), "name": name}
    if not name:
        return out
    out["elapsed_s"] = time.time() - RUN["started"] if out["running"] else None
    # The live log is shared with every run; until this run's stack rewrites
    # it, it still holds the LAST run's result.
    try:
        fresh = LIVE_LOG.stat().st_mtime >= RUN["started"]
    except OSError:
        fresh = False
    if not fresh:
        if out["running"]:
            out["state"], out["result"] = "starting the simulator", ""
            out["log_tail"] = tail(LOGS / f"{name}.out", 6)
        else:
            # Ended before the stack ever wrote the live log: the runner
            # refused or crashed, and its own log says why.
            out["state"] = ""
            out["result"] = (f"FAILED TO START (runner exit code "
                             f"{RUN['proc'].returncode})")
            out["log_tail"] = (tail(LOGS / f"{name}.runner.log")
                               or tail(LOGS / f"{name}.out"))
        return out
    state = last_line(LIVE_LOG, "Mission state ->")
    out["state"] = state.split("Mission state ->")[-1].strip() if state else ""
    result = last_line(LIVE_LOG, "Mission result:")
    out["result"] = result.split("Mission result:")[-1].strip() if result else ""
    grade = LOGS / f"{name}_grade.txt"
    if grade.exists() and not out["running"]:
        out["grade"] = [l.strip() for l in grade.read_text().splitlines()
                        if l.strip().startswith("[")]
    video = LOGS / f"{name}.mp4"
    if video.exists() and not out["running"]:
        out["video"] = str(video.relative_to(ROOT))
    run_out = LOGS / f"{name}.out"
    if run_out.exists():
        out["log_tail"] = run_out.read_text(errors="replace").splitlines()[-6:]
    return out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            body = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/worlds":
            WORLDS.mkdir(parents=True, exist_ok=True)
            self._json(200, {"worlds": sorted(p.stem for p in WORLDS.glob("*.json"))})
        elif path.startswith("/api/worlds/"):
            name = unquote(path.rsplit("/", 1)[1])
            f = WORLDS / f"{name}.json"
            if not NAME_RE.match(name) or not f.exists():
                self._json(404, {"error": "no such world"})
            else:
                self._json(200, json.loads(f.read_text()))
        elif path == "/api/run":
            self._json(200, run_status())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        url = urlparse(self.path)
        path = url.path
        if path == "/api/validate":
            errs, warns = validate(self._body())
            self._json(200, {"errors": errs, "warnings": warns})
        elif path.startswith("/api/worlds/"):
            name = unquote(path.rsplit("/", 1)[1])
            if not NAME_RE.match(name):
                self._json(400, {"error": "names are letters, digits, _ and - only"})
                return
            spec = self._body()
            spec["name"] = name
            spec["schema"] = world_spec.SCHEMA
            WORLDS.mkdir(parents=True, exist_ok=True)
            f = WORLDS / f"{name}.json"
            f.write_text(json.dumps(spec, indent=2) + "\n")
            errs, warns = validate(spec)
            self._json(200, {"path": str(f.relative_to(ROOT)), "errors": errs,
                             "warnings": warns})
        elif path.startswith("/api/run/"):
            name = unquote(path.rsplit("/", 1)[1])
            f = WORLDS / f"{name}.json"
            if RUN["proc"] and RUN["proc"].poll() is None:
                self._json(409, {"error": f"'{RUN['name']}' is still flying"})
                return
            if not NAME_RE.match(name) or not f.exists():
                self._json(404, {"error": "save the world first"})
                return
            errs, _ = validate(json.loads(f.read_text()))
            if errs:
                self._json(400, {"error": "the world has errors: " + "; ".join(errs)})
                return
            q = parse_qs(url.query)
            args = ["bash", str(ROOT / "scripts" / "run_custom_world.sh"), str(f)]
            if q.get("gui", ["0"])[0] == "1":
                args.append("--gui")
            if q.get("record", ["0"])[0] == "1":
                args.append("--record")
            LOGS.mkdir(parents=True, exist_ok=True)
            RUN["proc"] = subprocess.Popen(
                args, cwd=ROOT, stdout=open(LOGS / f"{name}.runner.log", "w"),
                stderr=subprocess.STDOUT, start_new_session=True)
            RUN["name"], RUN["started"] = name, time.time()
            self._json(200, {"started": name})
        else:
            self._json(404, {"error": "not found"})


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8777)
    args = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"world editor on http://127.0.0.1:{args.port}/  (worlds in {WORLDS})")
    srv.serve_forever()


if __name__ == "__main__":
    main()
