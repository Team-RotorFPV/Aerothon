#!/usr/bin/env bash
# Laptop: simulation and device gateway only. Production nodes stay on the Pi.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
if pgrep -af '(^|/)(arducopter|gz sim).*' >/dev/null; then
    echo 'A simulator is already running. Finish recovery/disarm before starting another.' >&2
    exit 1
fi
export AEROTHON_LAPTOP_SERVICES_ONLY=1 AEROTHON_NO_REAP=1
export AEROTHON_SUPPLY_DELIVERY_ZONE=0 AEROTHON_SUPPLY_BANNER_COLOURS=0
export AEROTHON_LOCALHOST_DDS=1 ROS_DOMAIN_ID=43
exec bash "$root/scripts/launch_level6_sim.sh" "$@"
