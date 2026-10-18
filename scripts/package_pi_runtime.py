#!/usr/bin/env python3
"""Build a source-and-built-GCS bundle for the aircraft, without development data.

Run on the development machine after building the revised GCS. Nothing is
installed, uploaded, started or armed by this command. The ZIP contains exactly
two maintained Markdown files and a SHA256 inventory for its payload.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (
    'src/aerothon_runtime',
    'src/aerothon_slam_grid',
    'src/aerothon_mission/mission_bringup',
    'src/aerothon_mission/mission_bt',
    'src/aerothon_mission/uav_description',
    'src/aerothon_avoidance/avoidance',
    'src/aerothon_payload/winch_ctrl',
    'src/aerothon_perception/camera_ctrl',
    'src/aerothon_perception/perception_qr',
    'src/aerothon_perception/perception_banner',
    'src/aerothon_perception/perception_redzone',
    'src/aerothon_perception/perception_overlay',
    'src/aerothon_gcs/gcs_aggregator',
)
TOOLS = (
    'scripts/mav_router.py', 'scripts/check_sensors.py',
    'scripts/measure_vel_response.py', 'scripts/calibrate_delivery_load.py',
    'scripts/check_pi_runtime.sh', 'scripts/serve_gcs.sh',
    'scripts/pi_runtime/Dockerfile', 'scripts/pi_runtime/console.launch.py',
    'scripts/pi_runtime/Dockerfile.dependencies',
    'scripts/pi_runtime/ldlidar-pthread.patch',
    'scripts/pi_runtime/runtime.env', 'scripts/pi_runtime/run_container.sh',
    'scripts/pi_runtime/run_router.sh', 'scripts/pi_runtime/install_services.sh',
    'scripts/pi_runtime/profile_router.py', 'scripts/pi_runtime/stop_runtime.py',
)
EXCLUDED = {'__pycache__', 'node_modules', 'build', 'install', 'log',
            'logs', 'test', 'tests', 'dist', 'target', '.pytest_cache', '.git'}


def payload(root):
    files = {}
    for rel in ('MASTER_DOCUMENTATION.md', 'AGENTS.md', *TOOLS):
        path = root / rel
        if not path.is_file():
            raise ValueError(f'Required runtime file missing: {rel}')
        files[rel] = path
    for rel in PACKAGES:
        if not (root / rel / 'package.xml').is_file():
            raise ValueError(f'Required runtime package missing: {rel}')
        for current, dirs, names in os.walk(root / rel):
            dirs[:] = sorted(d for d in dirs if d not in EXCLUDED)
            for name in sorted(names):
                path = Path(current) / name
                if path.suffix in ('.pyc', '.pyo', '.md'):
                    continue
                files[path.relative_to(root).as_posix()] = path
    dist = root / 'src/aerothon_gcs/tauri_app/dist'
    if not (dist / 'index.html').is_file() or not (dist / 'assets').is_dir():
        raise ValueError('Build the revised GCS with npm run build before packaging')
    for path in sorted(dist.rglob('*')):
        if path.is_file() and path.suffix != '.md':
            files['gcs/' + path.relative_to(dist).as_posix()] = path
    if not any(p.startswith('gcs/satellite/') for p in files):
        raise ValueError('Offline satellite tiles are missing from the GCS build')
    return files


def build(root, out):
    files = payload(root)
    inventory = {rel: hashlib.sha256(path.read_bytes()).hexdigest()
                 for rel, path in sorted(files.items())}
    manifest = dict(schema='aerothon-pi-runtime/1', files=inventory,
                    local_ros_packages=[Path(p).name for p in PACKAGES],
                    includes_simulation=False, includes_historical_evidence=False,
                    hardware_qualified=False, encoder_backend_implemented=False,
                    camera_servo_output=9,
                    configuration='Ship defaults; measure and configure on the aircraft.')
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, 'x', compression=zipfile.ZIP_DEFLATED) as z:
        for rel, path in sorted(files.items()):
            info = zipfile.ZipInfo(rel)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = ((0o100755 if path.suffix == '.sh' else 0o100644) << 16)
            z.writestr(info, path.read_bytes())
        z.writestr('runtime_manifest.json', json.dumps(manifest, indent=2) + '\n')
    with zipfile.ZipFile(out) as z:
        if z.testzip() is not None:
            raise ValueError('ZIP integrity check failed')
        for rel, digest in inventory.items():
            if hashlib.sha256(z.read(rel)).hexdigest() != digest:
                raise ValueError(f'Bundle hash mismatch: {rel}')
        if sorted(p for p in z.namelist() if p.endswith('.md')) != ['AGENTS.md', 'MASTER_DOCUMENTATION.md']:
            raise ValueError('Bundle must contain exactly the two maintained Markdown files')
    return dict(bundle=str(out), files=len(files), bytes=out.stat().st_size,
                sha256=hashlib.sha256(out.read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True, help='New ZIP path; never overwrites')
    args = parser.parse_args()
    print(json.dumps(build(ROOT, args.out.resolve()), indent=2))


if __name__ == '__main__':
    main()
