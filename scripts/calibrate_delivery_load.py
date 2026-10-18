#!/usr/bin/env python3
"""Freeze load calibration from independent loaded/unloaded FCU hover traces."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/aerothon_mission/mission_bt'))
from mission_bt.delivery_observer import calibration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--loaded', type=Path, required=True)
    p.add_argument('--unloaded', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    try:
        configs = [json.loads((f.parent/'sampling_config.json').read_text())
                   for f in (a.loaded, a.unloaded)]
        if configs[0]['mode'] != 'loaded' or configs[1]['mode'] != 'unloaded':
            raise ValueError('calibration requires separately labelled loaded and unloaded fixtures')
        if any(configs[0][k] != configs[1][k] for k in ('motor_channels', 'pwm_min', 'pwm_max')):
            raise ValueError('calibration motor output configurations differ')
        profile = calibration(json.loads(a.loaded.read_text()), json.loads(a.unloaded.read_text()))
    except (OSError, ValueError) as e:
        print(f'UNQUALIFIED: {e}', file=sys.stderr)
        return 1
    profile.update(loaded_recording=str(a.loaded), unloaded_recording=str(a.unloaded))
    profile['motor_config'] = {k: configs[0][k] for k in ('motor_channels', 'pwm_min', 'pwm_max')}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(profile, indent=2)+'\n')
    print(json.dumps(profile))
    return 0


if __name__ == '__main__': sys.exit(main())
