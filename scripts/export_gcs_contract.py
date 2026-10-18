"""Export the production arena schema for the disconnected browser editor."""
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def contract():
    source = ROOT / 'src/aerothon_mission/mission_bt/mission_bt/arena_dimensions.py'
    spec = importlib.util.spec_from_file_location('arena_contract', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {
        'fields': [dict(zip(('group', 'key', 'label', 'default', 'min', 'max', 'used_by'), field))
                   for field in module.FIELDS],
        'defaults': module.defaults(), 'presets': module.BUILTIN_PRESETS,
        'placements': module.PLACEMENTS, 'return_sides': module.RETURN_SIDES,
    }


def satellite_coverage():
    tiles = ROOT / 'src/aerothon_gcs/tauri_app/public/satellite'
    zooms = sorted(int(p.name) for p in tiles.iterdir() if p.is_dir() and p.name.isdigit())
    z = zooms[0]
    points = [(int(p.parent.name), int(p.stem)) for p in (tiles / str(z)).glob('*/*.jpg')]
    west, east = min(x for x, _ in points), max(x for x, _ in points) + 1
    north, south = min(y for _, y in points), max(y for _, y in points) + 1
    lon = lambda x: x / 2**z * 360.0 - 180.0
    lat = lambda y: math.degrees(math.atan(math.sinh(math.pi * (1 - 2*y/2**z))))
    bundled = dict(name='Simulator site (bundled)',
                   bounds=[lon(west), lat(south), lon(east), lat(north)],
                   minzoom=z, maxzoom=zooms[-1])
    cached = json.loads((ROOT / 'scripts/map_regions.json').read_text(encoding='utf-8'))
    # Drawn bottom to top: coarse regions (low minzoom) under detail regions.
    regions = sorted([bundled, *cached['regions']], key=lambda r: r['minzoom'])
    return dict(initial=cached['initial'], regions=regions)


if __name__ == '__main__':
    destination = ROOT / 'src/aerothon_gcs/tauri_app/src/arenaSchema.json'
    destination.write_text(json.dumps(contract(), indent=2) + '\n', encoding='utf-8')
    destination.with_name('satelliteCoverage.json').write_text(json.dumps(satellite_coverage(), indent=2) + '\n', encoding='utf-8')
