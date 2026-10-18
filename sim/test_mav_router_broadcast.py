"""The Pi router forwards the FC's MAVLink to every Mission Planner on the LAN.

Mission Planner in UDP mode only listens on 14550 and waits for the vehicle.
The router used to push to one configured laptop IP (192.168.0.100), so after
DHCP gave the laptop .101 nothing arrived (16 Oct 2026). It now also
broadcasts on each operator LAN, drops its own broadcast echo (which would
otherwise be written back into the FC's UART), and does not duplicate packets
to a listener the broadcast already reaches.
"""
import importlib.util
import ipaddress
import socket
import threading
import time
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "mav_router", Path(__file__).resolve().parent.parent / "scripts" / "mav_router.py")
router = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(router)
except ImportError as e:                               # pyserial absent
    pytest.skip(f"mav_router dependencies missing: {e}", allow_module_level=True)

PI = """1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever
3: wlan0    inet 192.168.0.102/24 brd 192.168.0.255 scope global dynamic noprefixroute wlan0\\       valid_lft 6807sec
4: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0\\       valid_lft forever"""


def test_only_operator_lans_are_broadcast_to():
    lans = router.lan_interfaces(PI)
    assert lans == [("192.168.0.102", ipaddress.ip_network("192.168.0.0/24"), "192.168.0.255")]


def test_every_lan_listener_is_reached_without_configuration():
    lans = router.lan_interfaces(PI)
    assert router.gcs_destinations(set(), set(), lans, 14550, True) == {("192.168.0.255", 14550)}


def test_a_listener_reached_by_broadcast_is_not_sent_duplicates():
    lans = router.lan_interfaces(PI)
    learned = {("192.168.0.101", 14550),      # Mission Planner UDP (listen) mode
               ("192.168.0.101", 53021),      # Mission Planner UDPCl: ephemeral port
               ("10.8.0.5", 14550)}           # off-LAN (VPN) client
    assert router.gcs_destinations(learned, {("192.168.0.100", 14550)}, lans, 14550, True) == {
        ("192.168.0.255", 14550), ("192.168.0.101", 53021), ("10.8.0.5", 14550),
        ("192.168.0.100", 14550)}


def test_without_broadcast_the_old_unicast_behaviour_holds():
    learned = {("192.168.0.101", 14550)}
    assert router.gcs_destinations(learned, set(), [], 14550, False) == learned


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_own_broadcast_echo_is_recognised():
    lans = router.lan_interfaces(PI)
    assert router.own_echo(("192.168.0.102", 14550), lans, 14550)         # our broadcast
    assert not router.own_echo(("192.168.0.101", 14550), lans, 14550)     # Mission Planner
    assert not router.own_echo(("192.168.0.102", 40000), lans, 14550)     # another local app
    assert not router.own_echo(("192.168.0.102", 14550), [], 14550)       # broadcast off


def test_gcs_commands_still_reach_the_fcu_with_broadcast_on(monkeypatch):
    gcs_port, mavros_port, fcu_port = free_port(), free_port(), free_port()
    lo = [("127.0.0.1", ipaddress.ip_network("127.0.0.0/8"), "127.255.255.255")]
    monkeypatch.setattr(router, "read_lan_interfaces", lambda: lo)
    threading.Thread(target=router.run_router, daemon=True, kwargs=dict(
        fcu_port=fcu_port, mavros_port=mavros_port, gcs_port=gcs_port,
        gcs_broadcast=True)).start()
    time.sleep(0.3)
    fcu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fcu.bind(("127.0.0.1", 0)); fcu.settimeout(0.5)
    fcu.sendto(b"\xfeFCU-HELLO", ("127.0.0.1", fcu_port))     # router learns the FCU
    time.sleep(0.2)
    gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gcs.bind(("127.0.0.1", 0)); gcs.settimeout(0.5)
    gcs.sendto(b"\xfeGCS-CMD", ("127.0.0.1", gcs_port))
    assert fcu.recv(4096) == b"\xfeGCS-CMD"
    fcu.sendto(b"\xfeTELEMETRY", ("127.0.0.1", fcu_port))     # and telemetry comes back
    assert gcs.recv(4096) == b"\xfeTELEMETRY"


def test_bytewise_uart_input_does_not_create_a_wifi_datagram_storm():
    import os
    import subprocess
    import sys
    master,slave=os.openpty()
    gcs=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    gcs.bind(('127.0.0.1',0));gcs.settimeout(.1)
    process=subprocess.Popen([sys.executable,str(Path(router.__file__)),
        '--fcu-serial',os.ttyname(slave),'--fcu-baud','115200',
        '--gcs-port',str(free_port()),'--mavros-port',str(free_port()),
        '--gcs-host','127.0.0.1','--gcs-out-port',str(gcs.getsockname()[1])],
        stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
    try:
        startup=[]
        for line in process.stderr:
            startup.append(line)
            if 'Mission Planner output:' in line:break
        assert any('Mission Planner output:' in line for line in startup),''.join(startup)
        assert process.poll() is None
        expected=bytes(range(240))
        for byte in expected:
            os.write(master,bytes((byte,)))
            time.sleep(.0005)
        packets=[];end=time.monotonic()+1.
        while sum(map(len,packets))<len(expected) and time.monotonic()<end:
            try:packets.append(gcs.recv(4096))
            except socket.timeout:pass
        assert b''.join(packets)==expected,'telemetry bytes changed or disappeared'
        assert len(packets)<=25,f'{len(packets)} UDP packets from 240 UART bytes congests Wi-Fi'
        assert all(len(p)<=1200 for p in packets)
    finally:
        process.terminate();process.wait(timeout=3)
        gcs.close();os.close(master);os.close(slave)
