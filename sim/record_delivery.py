#!/usr/bin/env python3
"""Passive onboard evidence recorder for a full rendered mission."""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, RCOut, PositionTarget
from std_msgs.msg import String
from qualification_evidence import fresh_flight_record,recorded_disarm


def json_sensor_values(value):
    if isinstance(value,dict):
        return {key:json_sensor_values(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):
        return [json_sensor_values(item) for item in value]
    if isinstance(value,float) and not math.isfinite(value):
        return str(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=6000)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('mission_onboard_evidence', parameter_overrides=[Parameter('use_sim_time', value=True)])
    bridge = CvBridge()
    values, latest = {}, [None]
    trace = (args.out/'onboard.jsonl').open('w')
    commands = (args.out/'winch_commands.jsonl').open('w')
    flight_commands = (args.out/'flight_commands.jsonl').open('w')
    detector_events = (args.out/'detector_events.jsonl').open('w')
    camera_events = (args.out/'camera_capture_events.jsonl').open('w')
    frames = (args.out/'camera_frames.jsonl').open('w')
    lidar = (args.out/'onboard_lidar.jsonl').open('w')
    last_scan = [None]
    frame_index = 0
    writer = None
    def now():
        return node.get_clock().now().nanoseconds*1e-9
    def capture(key, msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        values[key] = dict(received_stamp=now(), received_wall_s=time.monotonic(),
                           data=message_to_ordereddict(msg))
    def detail(key, msg):
        try: values[key] = json.loads(msg.data)
        except ValueError: values[key] = msg.data
        if key=='winch':
            received=now()
            values['winch_capture']=dict(received_stamp=received,data=values[key])
            # Preserve this actual arrival before the next 5 Hz video tick.
            # Two independent 5 Hz clocks otherwise alias the newest winch
            # receipt and can make fresh target-surface contacts look stale.
            trace.write(json.dumps(dict(stamp=received,**values))+'\n')
            trace.flush()
        if key in ('qr_timing','red_timing'):
            detector_events.write(json.dumps(dict(detector=key,received_stamp=now(),
                received_wall_s=time.monotonic(),
                detail=values[key],mission=values.get('mission')))+'\n')
            detector_events.flush()
    def command(msg):
        commands.write(json.dumps(dict(stamp=now(), command=msg.data))+'\n')
        commands.flush()
    def flight_command(topic,msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        flight_commands.write(json.dumps(dict(stamp=now(),topic=topic,
            data=message_to_ordereddict(msg)))+'\n')
        flight_commands.flush()
    for topic,typ in [('/mavros/setpoint_position/local',PoseStamped),
                       ('/mavros/setpoint_raw/local',PositionTarget),
                       ('/camera/set_pose',String)]:
        node.create_subscription(typ,topic,lambda m,t=topic:flight_command(t,m),10)
    for topic, typ, key in [('/mavros/state', State, 'flight'),
                            ('/mavros/rc/out', RCOut, 'motor_outputs'),
                            ('/mavros/imu/data', Imu, 'imu'),
                            ('/mavros/local_position/pose', PoseStamped, 'pose'),
                            ('/mavros/local_position/velocity_local', TwistStamped, 'velocity')]:
        node.create_subscription(typ, topic, lambda m, k=key: capture(k, m), qos_profile_sensor_data)
    for topic, key in [('/percep/payload', 'payload'), ('/winch/status', 'winch'),
                       ('/camera/pose_state','camera_state'),
                       ('/percep/banner/detail','banner_detail'),
                       ('/percep/qr/detail','qr_timing'),('/percep/redzone/detail','red_timing'),
                       ('/mission/search_timing','search_timing'),('/avoidance/detail','avoidance'),
                       ('/mission/delivery_observation', 'observation'), ('/mission/state', 'mission')]:
        node.create_subscription(String, topic, lambda m, k=key: detail(k, m), 10)
    node.create_subscription(String, '/mission/result', lambda m: detail('result', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/winch/cmd', command, 10)
    def camera_frame(msg):
        latest[0]=msg
        camera_events.write(json.dumps(dict(capture_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            received_stamp=now(),received_wall_s=time.monotonic(),
            mission=values.get('mission')))+'\n')
        camera_events.flush()
    node.create_subscription(Image, '/camera/image', camera_frame, qos_profile_sensor_data)
    def lidar_frame(msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==last_scan[0] and stamp!=(0,0):return
        last_scan[0]=stamp
        data=json_sensor_values(message_to_ordereddict(msg))
        # Keep the original invalid/no-return semantics in standard JSON.
        # Strings decode with float(value) for an exact offline scan replay.
        lidar.write(json.dumps(dict(received_stamp=now(),received_wall_s=time.monotonic(),
            data=data,nonfinite_encoding='float strings'),allow_nan=False)+'\n')
        lidar.flush()
    node.create_subscription(LaserScan,'/scan',lidar_frame,qos_profile_sensor_data)
    start, last_tick, last_image, ended = time.monotonic(), -1, None, None
    armed_record=None
    try:
        while time.monotonic()-start < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            stamp = now()
            if stamp-last_tick < .2: continue
            last_tick = stamp
            row=dict(stamp=stamp, **values)
            trace.write(json.dumps(row)+'\n'); trace.flush()
            if fresh_flight_record(row) is not None and row['flight']['data']['armed']:
                armed_record=row
            msg = latest[0]
            if msg is not None:
                frame_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if frame_stamp != last_image:
                    last_image = frame_stamp
                    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                    det = values.get('payload', {})
                    if isinstance(det, dict) and det.get('visible') and det.get('bbox'):
                        x,y,w,h = det['bbox']; cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                    if writer is None:
                        writer = cv2.VideoWriter(str(args.out/'onboard.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                                 5, (frame.shape[1],frame.shape[0]))
                    lines = [f"{stamp:.1f}s {values.get('mission', 'starting')}", str(values.get('observation', ''))]
                    for i,line in enumerate(lines):
                        cv2.putText(frame,line[:150],(10,25+25*i),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                    writer.write(frame)
                    frames.write(json.dumps(dict(index=frame_index,
                        capture_stamp=frame_stamp[0]+frame_stamp[1]*1e-9,
                        recorded_stamp=stamp,mission=values.get('mission')))+'\n')
                    frames.flush(); frame_index+=1
            disarm=recorded_disarm([armed_record,row]) if armed_record is not None else None
            if 'result' in values and disarm is not None:
                if ended is None: ended=time.monotonic()
                if time.monotonic()-ended > 3: break
            else:ended=None
    finally:
        trace.close(); commands.close(); frames.close(); flight_commands.close()
        detector_events.close(); camera_events.close(); lidar.close()
        if writer: writer.release()
        (args.out/'recorded_result.json').write_text(json.dumps(values.get('result'),indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': main()
