"""Web host and aggregator failure modes found by the 16 Oct 2026 audit.

Each would have stranded the operator in the field: a GCS crash-looping on a
power-cut profiles.json, a refused Start wedging every later Start and flight
command, a plain-text 500 shown as a JSON parse error, and one non-finite
telemetry value (front_m = inf with nothing ahead, normal outdoors) making
the browser drop every frame.
"""
import asyncio
import json
import math
import time
import aiohttp

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from aerothon_runtime.web import Host, errors


def test_old_open_gcs_pages_receive_the_original_array_map():
    from aerothon_runtime.web import map_for_legacy_client
    envelope={'kind':'map','data':{'w':4,'h':2,'res':.05,'encoding':'rle','rle':[-1,2,0,3,100,3]}}
    decoded=json.loads(map_for_legacy_client(envelope))
    assert decoded['data']['data']==[-1,-1,0,0,0,100,100,100]
    assert 'encoding' not in decoded['data']
    assert decoded['data']['res']==.05


@pytest.mark.asyncio
async def test_map_backpressure_does_not_silently_remove_browser_telemetry(tmp_path, monkeypatch):
    from aerothon_runtime.web import application
    gate=asyncio.Event()
    original=web.WebSocketResponse.send_str

    async def slow_map(self,data,*args,**kwargs):
        if self._req.path=='/ws' and '"kind": "map"' in data:
            await asyncio.sleep(.2)
        await original(self,data,*args,**kwargs)

    monkeypatch.setattr(web.WebSocketResponse,'send_str',slow_map)
    async def upstream(request):
        ws=web.WebSocketResponse();await ws.prepare(request)
        await gate.wait()
        await ws.send_str(json.dumps({'kind':'map','data':{'w':500,'h':500,'data':[0]*250000}}))
        await ws.send_str(json.dumps({'kind':'telemetry','data':{'phase':'after map'}}))
        async for _ in ws:pass
        return ws
    source=web.Application();source.router.add_get('/upstream',upstream)
    async with TestServer(source) as server:
        h=Host(tmp_path,tmp_path/'relay-state',aggregator=str(server.make_url('/upstream')))
        async with TestClient(TestServer(application(h))) as client:
            async with client.ws_connect('/ws') as browser:
                gate.set();received=[]
                for _ in range(2):
                    try:message=await browser.receive(timeout=1.)
                    except asyncio.TimeoutError:break
                    if message.type==aiohttp.WSMsgType.TEXT:received.append(json.loads(message.data))
                assert any(m['kind']=='map' for m in received),'large map was cancelled after 100 ms'
                assert any(m.get('data',{}).get('phase')=='after map' for m in received),'the map transfer silently killed later telemetry'


def test_corrupt_profiles_file_is_set_aside_not_fatal(tmp_path):
    state = tmp_path / "state"
    state.mkdir()
    (state / "profiles.json").write_text('{"drone": {"id": "drone", ')     # torn write
    h = Host(tmp_path, state)
    assert h.profiles == {}
    assert not (state / "profiles.json").exists()
    assert list(state.glob("profiles.unreadable-*.json"))


def test_an_invalid_saved_profile_is_dropped_and_the_rest_kept(tmp_path):
    state = tmp_path / "state"
    state.mkdir()
    good = dict(id="sim", host="laptop.local", gateway="http://laptop.local:8090")
    (state / "profiles.json").write_text(json.dumps({"sim": good, "bad": {"id": "$(rm -rf)"}}))
    h = Host(tmp_path, state)
    assert list(h.profiles) == ["sim"]
    assert h.profiles["sim"]["venue_file"] == ""        # older profiles get defaults


@pytest.mark.asyncio
async def test_unexpected_errors_reach_the_ui_as_json():
    async def boom(request):
        raise RuntimeError("runtime exited during disconnect")
    app = web.Application(middlewares=[errors])
    app.router.add_get("/x", boom)
    async with TestServer(app) as server:
        from aiohttp import ClientSession
        async with ClientSession() as s, s.get(server.make_url("/x")) as r:
            assert r.status == 500
            assert "runtime exited" in (await r.json())["error"]


@pytest.mark.asyncio
async def test_a_start_the_runtime_refuses_releases_the_reservation(tmp_path):
    fresh = {"mission": {"armed": False}, "safety": {"ready": True, "fcu_connected": True},
             "runtime_health": {"fresh": True}, "sample_age_s": {"state": 0}}

    async def aggregator(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        await ws.send_str(json.dumps({"kind": "telemetry", "data": fresh}))
        async for m in ws:
            cmd = json.loads(m.data)["data"]
            await ws.send_str(json.dumps({"kind": "ack", "data": {
                "cmd_id": cmd["cmd_id"], "result": "rejected",
                "reason": "execution supervisor stale"}}))
            # keep telemetry fresh while the host processes the ack
            await ws.send_str(json.dumps({"kind": "telemetry", "data": fresh}))
        return ws

    app = web.Application()
    app.router.add_get("/", aggregator)
    async with TestServer(app) as server:
        h = Host(tmp_path, tmp_path / "state", aggregator=str(server.make_url("/")))
        await h.start(None)
        try:
            for _ in range(50):
                if h.internal_ws is not None and h.fresh():
                    break
                await asyncio.sleep(0.05)
            assert h.fresh()
            r = await h.command({"command_id": "start_refused_1", "cmd": "start",
                                 "args": {"task": "qr"}})
            assert r["state"] == "Submitted"
            for _ in range(50):
                if h.start_reserved is None:
                    break
                await asyncio.sleep(0.05)
            assert h.start_reserved is None, "a refused Start wedged later Starts"
            assert h.execution["state"] == "Failed"
            assert "supervisor stale" in h.execution["reason"]
        finally:
            await h.close(None)


def test_non_finite_telemetry_is_valid_browser_json():
    from gcs_aggregator.aggregator import _finite_json
    data = {"nav": {"front_m": float("inf"), "cmd_vx": float("nan"), "ok": 1.5},
            "scan": {"ranges": [1.0, float("inf"), None]}, "flag": True}
    clean = _finite_json(data)
    text = json.dumps(clean, allow_nan=False)          # raises on inf/nan
    assert json.loads(text)["nav"] == {"front_m": None, "cmd_vx": None, "ok": 1.5}
    assert json.loads(text)["scan"]["ranges"] == [1.0, None, None]
    assert math.isinf(data["nav"]["front_m"]), "the live state is not modified"


def test_camera_rate_is_measured_without_deserializing_frames():
    """The aggregator reads only the Header stamp from the raw CDR bytes."""
    import rclpy.serialization
    from sensor_msgs.msg import Image
    from gcs_aggregator.sensor_metrics import SensorMetric, cdr_header_stamp
    img = Image(); img.header.stamp.sec = 1792157061; img.header.stamp.nanosec = 73767454
    img.height, img.width, img.encoding, img.step = 2, 2, "rgb8", 6
    img.data = bytes(12)
    raw = rclpy.serialization.serialize_message(img)
    assert cdr_header_stamp(raw) == (1792157061, 73767454)
    m = SensorMetric(); m.observe_raw(raw)
    assert m.capture == 1792157061 + 73767454 / 1e9 and len(m.receipts) == 1
    assert cdr_header_stamp(b"\x00\x05" + bytes(10)) is None      # unknown encapsulation
    assert cdr_header_stamp(b"\x00\x01") is None                  # truncated
