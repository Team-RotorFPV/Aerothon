#!/usr/bin/env python3
"""High-performance bi-directional MAVLink UDP router.

Routes MAVLink telemetry between:
  1) ArduPilot SITL / Serial FCU (default: listen on UDP 127.0.0.1:14550 or connect to SITL 5760/14550)
  2) Local MAVROS (UDP 127.0.0.1:14555)
  3) External GCS / Mission Planner / QGroundControl (UDP 0.0.0.0:14550 or 14551)
  4) Companion Computer / GCS Aggregator (UDP 127.0.0.1:14552)

Usage:
  python3 scripts/mav_router.py --fcu-in 14560 --mavros-out 14555 --gcs-out 14550
"""
import argparse
import logging
import select
import socket
import sys
import time

import serial

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (MAVRouter) %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("mav_router")


def run_router(fcu_port=14560, mavros_port=14555, gcs_port=14550,
               gcs_host=None, gcs_out_port=None, extra_ports=None,
               fcu_serial=None, fcu_baud=921600):
    if extra_ports is None:
        extra_ports = [14551, 14552]

    # Use UDP for SITL, or open the Pixhawk telemetry device directly on the Pi.
    if fcu_serial:
        fcu_link = serial.Serial(
            fcu_serial, baudrate=fcu_baud, timeout=0, write_timeout=0)
        fcu_sock = None
    else:
        fcu_link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fcu_link.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        fcu_link.bind(("0.0.0.0", fcu_port))
        fcu_link.setblocking(False)
        fcu_sock = fcu_link

    # Socket listening for GCS / Mission Planner connections
    gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gcs_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gcs_sock.bind(("0.0.0.0", gcs_port))
    gcs_sock.setblocking(False)

    # Socket for MAVROS communication
    mavros_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mavros_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Separate router command ingress from MAVROS telemetry ingress. Binding
    # both processes to 14555 with SO_REUSEADDR silently drops packets.
    mavros_sock.bind(("127.0.0.1", mavros_port + 1))
    mavros_sock.setblocking(False)

    # Registered client endpoints (IP, port)
    clients = {"mavros": ("127.0.0.1", mavros_port)}
    configured_gcs = set()
    if gcs_host:
        configured_gcs.add((gcs_host, gcs_out_port or gcs_port))
    
    # Active endpoints that have sent packets to us
    active_gcs_senders = set()
    active_fcu_senders = set()
    active_mavros_senders = set()

    logger.info("=========================================================")
    logger.info(" MAVLink High-Speed UDP Router Active")
    logger.info("=========================================================")
    if fcu_serial:
        logger.info(f"  FCU Link              : serial {fcu_serial} @ {fcu_baud}")
    else:
        logger.info(f"  FCU Ingress Port      : UDP 0.0.0.0:{fcu_port}")
    logger.info(f"  MAVROS Port           : UDP 127.0.0.1:{mavros_port}")
    logger.info(f"  Mission Planner / GCS : UDP 0.0.0.0:{gcs_port}")
    for host, port in configured_gcs:
        logger.info(f"  Mission Planner output: UDP {host}:{port}")
    logger.info("=========================================================")

    sockets = [fcu_link, gcs_sock, mavros_sock]
    packet_count = 0
    t_last_stat = time.time()

    try:
        while True:
            readable, _, _ = select.select(sockets, [], [], 0.01)

            for s in readable:
                try:
                    if fcu_serial and s is fcu_link:
                        data = fcu_link.read(4096)
                        addr = ("serial", fcu_serial)
                    else:
                        data, addr = s.recvfrom(4096)
                except Exception:
                    continue

                if not data:
                    continue

                packet_count += 1

                # 1. Packet from FCU (ArduPilot SITL / Pixhawk) -> Route to MAVROS + All connected GCS
                if s is fcu_link:
                    active_fcu_senders.add(addr)
                    # Forward to MAVROS
                    try:
                        mavros_sock.sendto(data, clients["mavros"])
                    except Exception:
                        pass
                    # Forward to all GCS clients (Mission Planner / QGC)
                    for g_addr in active_gcs_senders | configured_gcs:
                        try:
                            gcs_sock.sendto(data, g_addr)
                        except Exception:
                            pass

                # 2. Packet from GCS (Mission Planner / QGC commands) -> Route to FCU
                elif s is gcs_sock:
                    active_gcs_senders.add(addr)
                    try:
                        if fcu_serial:
                            fcu_link.write(data)
                        else:
                            for f_addr in active_fcu_senders:
                                fcu_sock.sendto(data, f_addr)
                    except Exception:
                        pass

                # 3. Packet from MAVROS -> Route to FCU
                elif s is mavros_sock:
                    active_mavros_senders.add(addr)
                    try:
                        if fcu_serial:
                            fcu_link.write(data)
                        else:
                            for f_addr in active_fcu_senders:
                                fcu_sock.sendto(data, f_addr)
                    except Exception:
                        pass

            if time.time() - t_last_stat > 10.0:
                logger.info(f"Router status: {packet_count} pkts routed | "
                            f"{len(active_fcu_senders)} FCU, "
                            f"{len(active_mavros_senders)} MAVROS, "
                            f"{len(active_gcs_senders)} GCS endpoints active.")
                t_last_stat = time.time()

    except KeyboardInterrupt:
        logger.info("MAVRouter stopped by user.")
    finally:
        fcu_link.close()
        gcs_sock.close()
        mavros_sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAVLink Multi-Endpoint Router")
    parser.add_argument("--fcu-in", type=int, default=14560, help="Port to receive MAVLink from SITL/FCU")
    parser.add_argument("--fcu-serial", help="Pixhawk serial device (for example /dev/ttyAMA0 or /dev/ttyACM0)")
    parser.add_argument("--fcu-baud", type=int, default=921600, help="Pixhawk serial baud rate")
    parser.add_argument("--mavros-port", type=int, default=14555, help="Port for MAVROS connection")
    parser.add_argument("--gcs-port", type=int, default=14550, help="Port for Mission Planner / QGC")
    parser.add_argument("--gcs-host", help="Laptop IP receiving Mission Planner MAVLink")
    parser.add_argument("--gcs-out-port", type=int, help="Laptop UDP listen port (default: --gcs-port)")
    args = parser.parse_args()

    run_router(fcu_port=args.fcu_in, mavros_port=args.mavros_port,
               gcs_port=args.gcs_port, gcs_host=args.gcs_host,
               gcs_out_port=args.gcs_out_port, fcu_serial=args.fcu_serial,
               fcu_baud=args.fcu_baud)
