#!/usr/bin/env python3
"""Drone-controlled hover/drop trial. Inputs are camera and MAVROS only.

Run against an already launched, idle SITL stack. Modes loaded/unloaded
record separate calibration fixtures; delivery uses the frozen profile.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/aerothon_mission/mission_bt'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('loaded', 'unloaded', 'delivery'), required=True)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--alt', type=float, default=5.0)
    parser.add_argument('--camera-hfov', type=float, default=0.851919,
                        help='radians; match the configured C270 camera')
    parser.add_argument('--payload-size', type=float, default=0.10,
                        help='longest visible payload edge in metres')
    parser.add_argument('--hover-seconds', type=float, default=20.0)
    parser.add_argument('--wall-timeout', type=float, default=900.0)
    parser.add_argument('--no-video',action='store_true',help='use a separate passive recorder for camera evidence')
    args = parser.parse_args()
    if not 2 < args.alt <= 5.5:
        parser.error('demo altitude must be above 2 m and within the 5.5 m winch reach')
    if args.mode == 'delivery' and not args.profile:
        parser.error('--profile required for delivery')
    args.out.mkdir(parents=True, exist_ok=True)
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from mission_bt.mav_commander import Mav
    from mission_bt.delivery_observer import stable
    from mission_bt.mission_tree import WinchDrop

    rclpy.init()
    node = Node('delivery_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
    mav = Mav(node)
    if args.profile:
        profile = json.loads(args.profile.read_text())
        if profile.get('motor_config') != dict(motor_channels=list(mav.motor_channels),
                                                pwm_min=mav.motor_pwm_min, pwm_max=mav.motor_pwm_max):
            node.destroy_node(); rclpy.shutdown()
            parser.error('profile does not match the configured motor outputs')
        mav.delivery_load_profile = profile
    bridge = CvBridge()
    latest = [None]
    last_command = [None]
    def record_command(msg):
        command = dict(stamp=node.get_clock().now().nanoseconds*1e-9, command=msg.data)
        last_command[0] = command
        command_trace.write(json.dumps(command)+'\n'); command_trace.flush()
    command_trace = (args.out/'winch_commands.jsonl').open('w')
    node.create_subscription(String, '/winch/cmd', record_command, 10)
    if not args.no_video:
        node.create_subscription(Image, '/camera/image', lambda m: latest.__setitem__(0, m),
                                 qos_profile_sensor_data)
    stage_pub=node.create_publisher(String,'/mission/state',10)
    trace = (args.out/'onboard.jsonl').open('w')
    writer = None
    last_image = None
    efforts, seen_motor = [], None
    phase, flight_start, hover_start = 'WAIT', None, None
    origin, leaf = None, None
    started_wall = time.monotonic()
    last_tick = -1
    result = {'status': 'INCOMPLETE', 'mode': args.mode}
    landing = False
    try:
        while time.monotonic()-started_wall < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            now = node.get_clock().now().nanoseconds*1e-9
            if now-last_tick < .2:
                continue
            last_tick = now
            if phase == 'WAIT':
                if not mav.state.connected or mav.alt() is None:
                    continue
                if flight_start is None:
                    flight_start = now
                if now-flight_start < 15:
                    continue
                mav.mission_started = True
                mav.set_camera_pose('NADIR')
                if mav.state.mode != 'GUIDED':
                    mav.set_mode('GUIDED'); continue
                if not mav.state.armed:
                    mav.arm(); continue
                origin = mav.pos()[:2]
                mav.takeoff(args.alt)
                phase = 'TAKEOFF'
            if phase == 'TAKEOFF':
                if now-flight_start > 120:
                    result = {'status': 'FAILED', 'reason': 'takeoff timeout'}
                    phase = 'LAND'
                elif mav.alt() > args.alt-.4:
                    phase, hover_start = 'HOVER', now
            if phase in ('HOVER', 'DELIVERY'):
                mav.goto(*origin, args.alt, mav.yaw())
                mav.set_camera_pose('NADIR')
                if mav.abort_requested or not mav.state.armed or mav.state.mode != 'GUIDED':
                    result = {'status': 'FAILED', 'reason': mav.abort_reason or 'flight interrupted'}
                    phase = 'LAND'
            t = mav.delivery_telemetry()
            det = mav.payload_seen()
            if phase == 'HOVER' and t and stable(t, now) and t['stamp'] != seen_motor:
                seen_motor = t['stamp']; efforts.append(t['effort'])
            if phase == 'HOVER' and now-hover_start >= args.hover_seconds:
                if args.mode == 'delivery':
                    leaf = WinchDrop(mav, args.alt, args.alt,
                                     hfov_rad=args.camera_hfov, payload_size_m=args.payload_size)
                    leaf.initialise()
                    leaf.drop_x, leaf.drop_y = origin
                    leaf.release_alt = args.alt
                    leaf.phase = 1
                    phase = 'DELIVERY'
                else:
                    result = {'status': 'RECORDED', 'stable_samples': len(efforts)}
                    phase = 'LAND'
            observation = None
            if phase == 'DELIVERY':
                leaf.update()
                observation = leaf.observer.status()
                if leaf.observer.state in ('DONE', 'FAILED', 'RECOVERED'):
                    result = {'status': leaf.observer.state, **observation}
                    phase = 'LAND'
            if phase == 'LAND':
                mav.pub_result.publish(String(data=json.dumps({
                    **result, 'state': 'DELIVERY_TRIAL',
                    'trial_scope': 'focused observer trial; full mission not exercised'})))
                if not landing:
                    mav.winch('stop'); mav.land(); landing = True
                if not mav.state.armed:
                    break
            stage_pub.publish(String(data='WINCH_DROP' if phase=='DELIVERY' else phase))
            trace.write(json.dumps(dict(stamp=now, phase=phase, telemetry=t,
                                        camera=det, winch=mav.winch_status,
                                        winch_command=last_command[0],
                                        observation=observation))+'\n')
            trace.flush()
            msg = latest[0]
            if msg is not None:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if stamp != last_image:
                    last_image = stamp
                    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                    if det and det.get('visible') and det.get('bbox'):
                        bx, by, bw, bh = det['bbox']
                        cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
                    if writer is None:
                        writer = cv2.VideoWriter(str(args.out/'onboard.avi'),
                                                cv2.VideoWriter_fourcc(*'MJPG'), 5,
                                                (frame.shape[1], frame.shape[0]))
                    lines = [f'{now:.1f}s {phase}', str(observation or result)]
                    if det:
                        lines.append(f"payload size {max(det.get('w_px',0),det.get('h_px',0))} px; frame {det.get('stamp')}")
                    for i, line in enumerate(lines):
                        cv2.putText(frame, line[:125], (10, 25+25*i),
                                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                    writer.write(frame)
    finally:
        if mav.state.armed:
            mav.winch('stop'); mav.land()
            until = time.monotonic()+180
            while mav.state.armed and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.1)
        result['final_armed'] = bool(mav.state.armed)
        mav.pub_result.publish(String(data=json.dumps({
            **result, 'state': 'DELIVERY_TRIAL',
            'trial_scope': 'focused observer trial; full mission not exercised'})))
        trace.close()
        command_trace.close()
        if writer: writer.release()
        (args.out/'effort_samples.json').write_text(json.dumps(efforts, indent=2)+'\n')
        (args.out/'sampling_config.json').write_text(json.dumps(dict(
            motor_channels=list(mav.motor_channels), pwm_min=mav.motor_pwm_min,
            pwm_max=mav.motor_pwm_max, camera_hfov=args.camera_hfov,
            payload_size_m=args.payload_size, mode=args.mode), indent=2)+'\n')
        (args.out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if result['status'] in ('RECORDED', 'DONE', 'RECOVERED') and not result['final_armed'] else 1


if __name__ == '__main__': sys.exit(main())
