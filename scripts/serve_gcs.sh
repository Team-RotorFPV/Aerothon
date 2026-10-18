#!/usr/bin/env bash
# Serve the prebuilt revised GCS on the Pi. No npm, Node or Tauri required.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
web_root="$root/gcs"
if [[ ! -f "$web_root/index.html" ]]; then
    web_root="$root/src/aerothon_gcs/tauri_app/dist"
fi
[[ -f "$web_root/index.html" ]] || { echo 'Built GCS missing; build and package on the development machine.' >&2; exit 1; }
[[ ! -f /opt/ros/jazzy/setup.bash ]] || { set +u; source /opt/ros/jazzy/setup.bash; set -u; }
[[ ! -f "$root/install/setup.bash" ]] || { set +u; source "$root/install/setup.bash"; set -u; }
export PYTHONPATH="$root/src/aerothon_runtime:${PYTHONPATH:-}"
exec python3 -m aerothon_runtime.web --static "$web_root" --port "${AEROTHON_GCS_HTTP_PORT:-8899}" --bind "${AEROTHON_GCS_HTTP_BIND:-0.0.0.0}"
