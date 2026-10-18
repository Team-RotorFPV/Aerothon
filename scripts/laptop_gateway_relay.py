#!/usr/bin/env python3
"""Expose WSL's two explicit gateway TCP services on the laptop intranet IP."""
import argparse
import asyncio

async def relay(reader, writer, target, port):
    upstream = None
    try:
        r, upstream = await asyncio.wait_for(asyncio.open_connection(target, port), 3)
        async def pump(source, sink):
            while data := await source.read(65536):
                sink.write(data)
                await sink.drain()
        tasks = [asyncio.create_task(pump(reader, upstream)), asyncio.create_task(pump(r, writer))]
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
    except (OSError, asyncio.TimeoutError):
        pass
    finally:
        writer.close()
        if upstream:
            upstream.close()

async def run(args):
    servers = []
    for port in (args.mavlink_port,args.sensor_port):
        servers.append(await asyncio.start_server(lambda r,w,p=port: relay(r,w,args.target,p),args.bind,port))
    print(f'Laptop gateway relay {args.bind} -> {args.target}; FCU {args.mavlink_port}, sensors {args.sensor_port}',flush=True)
    async with servers[0], servers[1]:
        await asyncio.gather(*(s.serve_forever() for s in servers))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bind',required=True,help='Laptop intranet IP')
    p.add_argument('--target',required=True,help='Current WSL IP from hostname -I')
    p.add_argument('--mavlink-port',type=int,default=5762)
    p.add_argument('--sensor-port',type=int,default=8090)
    args=p.parse_args()
    try: asyncio.run(run(args))
    except KeyboardInterrupt: pass

if __name__=='__main__': main()
