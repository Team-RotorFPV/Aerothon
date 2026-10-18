#!/usr/bin/env python3
"""Export evaluator camera captures with recorded mission states and native time."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2


def records(path):
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', type=Path)
    args = parser.parse_args()
    run, view = args.mission, args.mission / 'overview'
    summary = json.loads((run / 'summary.json').read_text())
    if summary.get('final_armed') is not False:
        raise SystemExit('Connected disarm must be recorded before exporting the final video')
    onboard = sorted(records(run / 'onboard.jsonl'), key=lambda row: row['stamp'])
    armed = [row for row in onboard if row.get('flight', {}).get('data', {}).get('armed')]
    if not armed:
        raise SystemExit('No armed flight was recorded')
    start = armed[0]['stamp']
    disarms = [row['stamp'] for row in onboard if row['stamp'] > start
               and row.get('flight', {}).get('data', {}).get('connected')
               and row.get('flight', {}).get('data', {}).get('armed') is False]
    if not disarms:
        raise SystemExit('Fresh connected disarm not found')
    end = disarms[0]
    frames = sorted(records(view / 'frames.jsonl'), key=lambda row: row['stamp'])
    selected = [row for row in frames if start - 2 <= row['stamp'] <= end + 2]
    if not selected or selected[-1]['stamp'] < end:
        raise SystemExit('Overview does not cover disarm')
    times = [row['stamp'] for row in onboard]
    native = sorted(records(run / 'evaluation_vehicle_truth.jsonl'), key=lambda row: row['stamp'])
    native_times = [row['stamp'] for row in native]
    camera = json.loads((view / 'configuration.json').read_text())
    cx, cy, cz = camera['pose'][:3]
    focal = camera['width'] / (2 * math.tan(camera['hfov_rad'] / 2))
    rendered = view / 'annotated'
    rendered.mkdir(exist_ok=False)
    lines = []
    for index, frame in enumerate(selected):
        stamp = frame['stamp']
        row = onboard[max(0, bisect.bisect_right(times, stamp) - 1)]
        image = cv2.imread(str(view / frame['file']))
        if image is None:
            raise SystemExit(f'Missing capture: {frame["file"]}')
        state = str(row.get('mission', 'STARTUP')).replace('_', ' ')
        terminal = ' | DISARMED' if stamp >= end else ''
        label = f'CLARIFIED ARENA | FIELD | {max(0, stamp-start):06.1f} s | {state}{terminal}'
        cv2.rectangle(image, (0, 0), (image.shape[1], 50), (23, 27, 31), -1)
        cv2.putText(image, label, (22, 34), cv2.FONT_HERSHEY_SIMPLEX, .8,
                    (240, 240, 240), 2, cv2.LINE_AA)
        if stamp >= end:
            cv2.putText(image, f'RECORDED RESULT: {summary.get("grade")}', (22, 83),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (240, 240, 240), 2, cv2.LINE_AA)
        truth_index = bisect.bisect_right(native_times, stamp) - 1
        if truth_index >= 0 and stamp - native[truth_index]['stamp'] <= .7:
            truth = native[truth_index]
            distance = cz - truth['z']
            if distance > 0:
                px = round(camera['width'] / 2 + (truth['x'] - cx) * focal / distance)
                py = round(camera['height'] / 2 - (truth['y'] - cy) * focal / distance)
                cv2.circle(image, (px, py), 12, (0, 220, 255), 2, cv2.LINE_AA)
                cv2.putText(image, 'Aircraft', (px + 16, py - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
        name = f'frame_{index:06d}.jpg'
        cv2.imwrite(str(rendered / name), image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        duration = selected[index + 1]['stamp'] - stamp if index + 1 < len(selected) else 2.0
        if duration <= 0:
            raise SystemExit('Non-monotonic native camera stamps')
        lines.extend([f"file 'annotated/{name}'", f'duration {duration:.9f}'])
    lines.append(f"file 'annotated/frame_{len(selected)-1:06d}.jpg'")
    concat = view / 'timeline.ffconcat'
    concat.write_text('\n'.join(lines) + '\n')
    video = view / 'world_overview.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-n', '-f', 'concat', '-safe', '0',
                    '-i', str(concat), '-vf', 'fps=10', '-c:v', 'libx264',
                    '-preset', 'veryfast', '-crf', '21', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(video)], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(video), '-f', 'null', '-'], check=True)
    receipt = dict(video=str(video), sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
                   grade=summary.get('grade'), acceptance_pass=summary.get('acceptance_pass'),
                   capture_count=len(selected), first_arm=start, connected_disarm=end,
                   time_basis='Native Gazebo stamps; frames held until next actual capture',
                   maximum_capture_gap_s=max(b['stamp']-a['stamp'] for a, b in zip(selected, selected[1:])),
                   evaluation_only=True, actual_gazebo_camera=True)
    (view / 'export_receipt.json').write_text(json.dumps(receipt, indent=2))
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
