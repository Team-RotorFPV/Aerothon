#!/usr/bin/env python3
"""The Pi's device link to the laptop simulator survives a dropped stream.

On a congested LAN the gateway closed the /sensors stream once and the Pi
never reopened it: the vehicle clock froze and readiness stayed stale until
an operator reconnected by hand. A drop on the SAME gateway session is now
reopened; a changed session is still a fault that needs an explicit
reconnect. These run the real driver against a local fake gateway.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_remote_device_link.py -v
"""

import asyncio
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "aerothon_runtime"))

rclpy = pytest.importorskip("rclpy")
web = pytest.importorskip("aiohttp.web")

from aerothon_runtime import remote_devices                     # noqa: E402
from aerothon_runtime.device_wire import pack                    # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("remote_devices_link_test", parameter_overrides=[
        rclpy.parameter.Parameter("gateway", value="http://127.0.0.1:0")])
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def clock_packet(session, t):
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    msg = Clock()
    msg.clock.sec, msg.clock.nanosec = int(t), int((t % 1) * 1e9)
    return pack("clock", session, serialize_message(msg))


class FakeGateway:
    """Manifest plus a /sensors stream that drops its first connection."""

    def __init__(self):
        self.session = "s1"
        self.sensor_connections = 0
        self.t = 100.0

    def manifest(self):
        return {"v": 1, "session": self.session, "system_id": 1,
                "component_id": 1, "fcu_ok": True}

    async def start(self):
        app = web.Application()

        async def manifest(request):
            return web.json_response(self.manifest())

        async def sensors(request):
            ws = web.WebSocketResponse(heartbeat=3)
            await ws.prepare(request)
            self.sensor_connections += 1
            first = self.sensor_connections == 1
            for _ in range(3 if first else 200):
                if ws.closed:
                    break
                self.t += 0.05
                await ws.send_bytes(clock_packet(self.session, self.t))
                await asyncio.sleep(0.02)
            if first:
                # The congested-link failure: the stream goes away mid-run.
                request.transport.close()
            return ws

        async def camera(request):
            ws = web.WebSocketResponse(heartbeat=3)
            await ws.prepare(request)
            while not ws.closed:
                await asyncio.sleep(0.05)
            return ws

        app.router.add_get("/manifest", manifest)
        app.router.add_get("/sensors", sensors)
        app.router.add_get("/camera", camera)
        self.runner = web.AppRunner(app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, "127.0.0.1", 0)
        await site.start()
        port = site._server.sockets[0].getsockname()[1]
        return f"http://127.0.0.1:{port}"


async def fly(driver, gateway, seconds, during=None):
    driver.url = await gateway.start()
    task = asyncio.create_task(driver.run())
    try:
        await asyncio.sleep(seconds / 2)
        if during:
            during()
        await asyncio.sleep(seconds / 2)
    finally:
        driver.shutdown.set()
        await asyncio.wait_for(task, 5)
        await gateway.runner.cleanup()


def test_a_dropped_sensor_stream_is_reopened_on_the_same_session(node):
    driver = remote_devices.RemoteDevices(node)
    gateway = FakeGateway()
    asyncio.run(fly(driver, gateway, 3.0))
    assert gateway.sensor_connections >= 2
    assert driver.reopened.get("/sensors", 0) >= 1
    assert driver.fault == ""
    # The clock kept advancing after the drop, past the first connection's 3 samples.
    assert driver.clock > 100.0 + 3 * 0.05 + 1e-6


def test_a_changed_gateway_session_is_still_a_fault(node):
    driver = remote_devices.RemoteDevices(node)
    gateway = FakeGateway()

    def restart():
        gateway.session = "s2"

    asyncio.run(fly(driver, gateway, 2.0, during=restart))
    assert "session changed" in driver.fault


def test_identity_reads_stale_when_the_manifest_goes_quiet(node):
    import json
    import time
    from std_msgs.msg import String
    driver = remote_devices.RemoteDevices(node)
    seen = []
    node.create_subscription(String, "/devices/status", lambda m: seen.append(json.loads(m.data)), 10)
    driver.fault = ""
    driver.manifest = FakeGateway().manifest()
    driver.manifest_at = time.monotonic() - remote_devices.MANIFEST_FRESH_S - 0.5
    driver.publish_status()
    rclpy.spin_once(node, timeout_sec=0.5)
    assert seen and seen[-1]["identity"]["ok"] is False
