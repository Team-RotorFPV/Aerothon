#!/usr/bin/env python3
"""Fetch the offline satellite regions in scripts/map_regions.json.

Tiles land in OUT/{z}/{x}/{y}.jpg, the layout the web host serves from its
state-dir `map-tiles` cache. Copy OUT there on the Pi. Re-running resumes:
valid tiles already present are skipped. Smallest regions first, then by zoom.

    python scripts/download_map_tiles.py --out <dir> [--workers 8]

The imagery is Esri World Imagery; its terms of use govern bulk download.
"""
import argparse
import json
import math
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests

ROOT = Path(__file__).resolve().parents[1]
REGIONS = ROOT / 'scripts' / 'map_regions.json'
URL = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'


def tile_range(bounds, z):
    """(x0, x1, y0, y1) inclusive tile indices covering [w, s, e, n] at zoom z."""
    w, s, e, n = bounds
    count = 2 ** z

    def x(lon):
        return min(count - 1, max(0, int((lon + 180.0) / 360.0 * count)))

    def y(lat):
        r = math.radians(lat)
        return min(count - 1, max(0, int((1 - math.log(math.tan(r) + 1 / math.cos(r)) / math.pi) / 2 * count)))
    return x(w), x(e), y(n), y(s)


def tiles(regions):
    for region in sorted(regions, key=lambda r: tile_count([r])):
        for z in range(region['minzoom'], region['maxzoom'] + 1):
            x0, x1, y0, y1 = tile_range(region['bounds'], z)
            for tx in range(x0, x1 + 1):
                for ty in range(y0, y1 + 1):
                    yield z, tx, ty


def tile_count(regions):
    total = 0
    for region in regions:
        for z in range(region['minzoom'], region['maxzoom'] + 1):
            x0, x1, y0, y1 = tile_range(region['bounds'], z)
            total += (x1 - x0 + 1) * (y1 - y0 + 1)
    return total


def is_jpeg(data):
    return data.startswith(b'\xff\xd8\xff') and data.endswith(b'\xff\xd9')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=8)
    args = parser.parse_args()
    regions = json.loads(REGIONS.read_text(encoding='utf-8'))['regions']
    total = tile_count(regions)
    local = threading.local()
    lock = threading.Lock()
    stats = dict(done=0, fetched=0, skipped=0, failed=0, bytes=0)
    failures = []
    started = time.monotonic()

    def fetch(job):
        z, x, y = job
        path = args.out / str(z) / str(x) / f'{y}.jpg'
        outcome = 'skipped'
        if not (path.is_file() and is_jpeg(path.read_bytes())):
            session = getattr(local, 'session', None)
            if session is None:
                session = local.session = requests.Session()
                session.headers['User-Agent'] = 'AeroTHON-GCS/1.0 offline cache'
            outcome = 'failed'
            for attempt in range(5):
                try:
                    r = session.get(URL.format(z=z, x=x, y=y), timeout=20)
                    if r.status_code == 200 and is_jpeg(r.content):
                        path.parent.mkdir(parents=True, exist_ok=True)
                        tmp = path.with_suffix('.tmp')
                        tmp.write_bytes(r.content)
                        tmp.replace(path)
                        outcome = 'fetched'
                        break
                    if r.status_code in (400, 404):
                        break
                except requests.RequestException:
                    pass
                time.sleep(2 ** attempt)
        with lock:
            stats['done'] += 1
            stats[outcome] += 1
            if outcome == 'fetched':
                stats['bytes'] += path.stat().st_size
            if outcome == 'failed':
                failures.append([z, x, y])
            if stats['done'] % 2000 == 0 or stats['done'] == total:
                rate = stats['done'] / max(1e-6, time.monotonic() - started)
                print(f"{stats['done']}/{total} tiles, {stats['fetched']} fetched, "
                      f"{stats['skipped']} present, {stats['failed']} failed, "
                      f"{stats['bytes'] / 1e9:.2f} GB new, {rate:.0f}/s, z{z}", flush=True)

    with ThreadPoolExecutor(args.workers) as pool:
        for _ in pool.map(fetch, tiles(regions), chunksize=64):
            pass
    report = dict(stats, total=total, regions=regions, failures=failures[:1000],
                  seconds=round(time.monotonic() - started, 1))
    (args.out / 'download_report.json').write_text(json.dumps(report, indent=1), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('regions', 'failures')}))
    return 1 if stats['failed'] else 0


if __name__ == '__main__':
    sys.exit(main())
