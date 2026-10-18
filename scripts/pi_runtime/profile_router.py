#!/usr/bin/env python3
"""Native Pi UART owner, following an explicitly selected vehicle profile."""
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time


def routing_settings(p, env):
    """A saved network profile releases the UART. No profile uses runtime.env.

    Routing has no props or mission-start dependency. The host service can
    receive and forward FC heartbeats before Docker, ROS or a browser exists.
    """
    if p and p.get('transport') != 'serial':
        return None
    device = str(p.get('device') or env.get('AEROTHON_FCU_DEVICE', ''))
    if not device.startswith('/dev/') or '..' in device:
        return None
    baud = int(p.get('baud', env.get('AEROTHON_FCU_BAUD', '115200')))
    if baud < 9600 or baud > 3000000:
        return None
    gcs_host = str(p.get('gcs_host') or env.get('AEROTHON_GCS_HOST', '')).strip()
    if gcs_host and not all(c.isalnum() or c in '.:-' for c in gcs_host):
        return None
    return device, baud, gcs_host, p.get('gcs_broadcast', True) is not False

def main():
    source = Path('/home/pi5/.aerothon/vehicle_connection.json')
    proc = None
    signature = None
    stopping = False
    started, delay, retry_at = 0.0, 0.5, 0.0
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping:
            try:
                p = json.loads(source.read_text()) if source.exists() else {}
                wanted = routing_settings(p, os.environ)
                if wanted and not (Path(wanted[0]).exists() and stat.S_ISCHR(Path(wanted[0]).stat().st_mode)):
                    wanted = None
            except (OSError, ValueError, TypeError):
                wanted = None
            died = proc is not None and proc.poll() is not None
            if died and signature == wanted:
                # A router that dies at once (port held by a getty, pyserial
                # missing) was relaunched every 0.5 s: a flapping UART and two
                # journal lines a second. Back off, up to 30 s, and say why.
                quick = time.monotonic() - started < 5.0
                delay = min(30.0, delay * 2) if quick else 0.5
                if quick:
                    print(f'UART owner exited (code {proc.returncode}) {time.monotonic() - started:.1f} s '
                          f'after start; retrying in {delay:.0f} s', flush=True)
                proc = None
                retry_at = time.monotonic() + delay
            if signature != wanted or (proc is None and wanted and time.monotonic() >= retry_at):
                if proc and proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
                if signature != wanted:
                    delay, retry_at = 0.5, 0.0
                proc = None
                signature = wanted
                if wanted:
                    started = time.monotonic()
                    # MAVROS on the Pi and a GCS port (14550) for the laptop;
                    # with gcs_host the FC's MAVLink is also pushed to it.
                    cmd = ['/usr/bin/python3','/home/pi5/aerothon_ws/scripts/mav_router.py',
                        '--fcu-serial',wanted[0],'--fcu-baud',str(wanted[1]),'--mavros-port','14555','--gcs-port','14550']
                    if wanted[2]:
                        cmd += ['--gcs-host', wanted[2], '--gcs-out-port', '14550']
                    if wanted[3]:
                        cmd += ['--gcs-broadcast']
                    proc = subprocess.Popen(cmd)
                    print(f'UART owner started: {wanted[0]} at {wanted[1]}'
                          + (f', MAVLink to {wanted[2]}:14550' if wanted[2] else '')
                          + (', LAN broadcast :14550' if wanted[3] else ''), flush=True)
            time.sleep(.5)
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)

if __name__ == '__main__':
    main()
