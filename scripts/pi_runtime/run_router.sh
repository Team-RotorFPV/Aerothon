#!/usr/bin/env bash
set -euo pipefail
source /home/pi5/aerothon-deploy/runtime.env
export AEROTHON_FCU_DEVICE AEROTHON_FCU_BAUD
exec /usr/bin/python3 /home/pi5/aerothon-deploy/profile_router.py
