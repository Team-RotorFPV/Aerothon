#!/usr/bin/env python3
"""Cache a small Esri World Imagery tile pyramid for offline GCS use."""

from __future__ import annotations

import argparse
import math
import time
import urllib.request
from pathlib import Path


def tile_xy(lat: float, lon: float, zoom: int) -> tuple[int, int]:
    scale = 2**zoom
    x = int((lon + 180.0) / 360.0 * scale)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * scale)
    return x, y


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lat", type=float, default=-35.36324)
    parser.add_argument("--lon", type=float, default=149.1652)
    parser.add_argument("--min-zoom", type=int, default=14)
    parser.add_argument("--max-zoom", type=int, default=19)
    parser.add_argument("--radius", type=int, default=1)
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "src/aerothon_gcs/tauri_app/public/satellite",
    )
    args = parser.parse_args()

    total = 0
    for zoom in range(args.min_zoom, args.max_zoom + 1):
        center_x, center_y = tile_xy(args.lat, args.lon, zoom)
        for x in range(center_x - args.radius, center_x + args.radius + 1):
            for y in range(center_y - args.radius, center_y + args.radius + 1):
                destination = args.output / str(zoom) / str(x) / f"{y}.jpg"
                if not args.force and destination.exists() and destination.stat().st_size > 100:
                    continue
                destination.parent.mkdir(parents=True, exist_ok=True)
                request = urllib.request.Request(
                    "https://services.arcgisonline.com/ArcGIS/rest/services/"
                    f"World_Imagery/MapServer/tile/{zoom}/{y}/{x}",
                    headers={"User-Agent": "AeroTHON-GCS/1.0"},
                )
                with urllib.request.urlopen(request, timeout=20) as response:
                    destination.write_bytes(response.read())
                total += 1
                time.sleep(0.05)
    print(f"Cached {total} new map tiles in {args.output}")


if __name__ == "__main__":
    main()
