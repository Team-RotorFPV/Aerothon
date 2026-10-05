"""Read one local GCS telemetry snapshot; sends no flight commands."""
import asyncio
import json
import websockets


async def main():
    async with websockets.connect("ws://127.0.0.1:8765", open_timeout=5) as ws:
        while True:
            packet = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
            if packet.get("kind") == "telemetry":
                state = packet["data"]
                print(json.dumps({k: state[k] for k in ("mission", "flight", "power")}))
                return


asyncio.run(main())
