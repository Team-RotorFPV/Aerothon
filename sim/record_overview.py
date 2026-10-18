#!/usr/bin/env python3
"""Record a native Gazebo overview camera. Evaluation only; no ROS output."""
import argparse
import json
import math
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', type=Path, required=True)
    args = parser.parse_args()
    out = args.mission / 'overview'
    out.mkdir(exist_ok=False)
    frames = out / 'frames'
    frames.mkdir()
    layout = json.loads((args.mission / 'layout.json').read_text())
    home = layout['home_world']
    poly = [(x + home[0], y + home[1]) for x, y in layout['geofence_poly']]
    xs, ys = zip(*poly)
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    width, height, hfov = 1920, 1080, 0.90
    span = max(max(xs) - min(xs), (max(ys) - min(ys)) * width / height)
    z = 1.12 * span / (2 * math.tan(hfov / 2))
    topic = '/evaluation/overview/image'
    sdf = f'''<sdf version="1.9"><model name="evaluation_overview_camera">
<static>true</static><pose>{cx} {cy} {z} 0 {math.pi/2} {math.pi/2}</pose>
<link name="camera_link"><sensor name="overhead" type="camera">
<always_on>true</always_on><update_rate>3</update_rate><topic>{topic}</topic>
<camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{width}</width>
<height>{height}</height><format>R8G8B8</format></image>
<clip><near>0.1</near><far>300</far></clip></camera></sensor></link></model></sdf>'''
    (out / 'camera.sdf').write_text(sdf)
    config = dict(view='native Gazebo overview', topic=topic,
                  pose=[cx, cy, z, 0, math.pi / 2, math.pi / 2],
                  world=layout.get('spec'), width=width, height=height,
                  hfov_rad=hfov, capture_hz=3, ros_publications=False,
                  flight_input=False)
    (out / 'configuration.json').write_text(json.dumps(config, indent=2))
    native = Node()
    req = EntityFactory(sdf=sdf, name='evaluation_overview_camera', allow_renaming=False)
    ok, response = native.request('/world/mission2/create', req, EntityFactory, Boolean, 5000)
    if not ok or not response.data:
        raise SystemExit('Evaluator overview camera could not be created')
    pending = queue.Queue(maxsize=12)
    closing = threading.Event()
    stats = dict(received=0, dropped=0, written=0)

    def receive(msg):
        if closing.is_set():
            return
        stats['received'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        try:
            pending.put_nowait((stamp, msg.width, msg.height, msg.step, bytes(msg.data)))
        except queue.Full:
            stats['dropped'] += 1

    if not native.subscribe(Image, topic, receive):
        raise SystemExit('Evaluator overview subscription failed')
    started, last_stamp = time.monotonic(), None
    try:
        with (out / 'frames.jsonl').open('x') as ledger:
            while time.monotonic() - started < 7200:
                try:
                    stamp, w, h, step, data = pending.get(timeout=1)
                except queue.Empty:
                    if (args.mission / 'recorded_result.json').exists():
                        break
                    continue
                if stamp == last_stamp:
                    continue
                last_stamp = stamp
                rgb = np.frombuffer(data, np.uint8).reshape(h, step)[:, :w * 3].reshape(h, w, 3)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                rel = f'frames/frame_{stats["written"]:06d}.jpg'
                if not cv2.imwrite(str(out / rel), bgr, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                    raise RuntimeError('Overview frame write failed')
                ledger.write(json.dumps(dict(stamp=stamp, file=rel)) + '\n')
                ledger.flush()
                stats['written'] += 1
                if stats['written'] == 1:
                    cv2.imwrite(str(out / 'preview.png'), bgr)
                    (out / 'ready.json').write_text(json.dumps(dict(stamp=stamp, topic=topic)))
                    print('Native overview ready', flush=True)
                if (args.mission / 'recorded_result.json').exists():
                    break
    except KeyboardInterrupt:
        pass
    finally:
        closing.set()
        native.unsubscribe(topic)
        (out / 'capture_summary.json').write_text(json.dumps(stats, indent=2))
        print(json.dumps(stats), flush=True)


if __name__ == '__main__':
    main()
