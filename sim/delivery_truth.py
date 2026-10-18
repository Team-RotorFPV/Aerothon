#!/usr/bin/env python3
"""Independent evaluation recorder. Never publishes or controls the drone."""
import argparse
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    p.add_argument('--vehicle-out',default=None)
    args = p.parse_args()
    rclpy.init()
    node = Node('delivery_evaluation_only', parameter_overrides=[Parameter('use_sim_time', value=True)])
    native=None
    vehicle_file=None
    stopped=[False]
    guard=threading.Lock()
    if args.vehicle_out:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        native=GzNode()
        vehicle_file=open(args.vehicle_out,'w')
        last=[-1.0]
        def vehicle(msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            if stamp-last[0]<.05:
                return
            for pose in msg.pose:
                if pose.name=='aerothon_iris':
                    with guard:
                        if stopped[0]:return
                        last[0]=stamp
                        vehicle_file.write(json.dumps(dict(stamp=stamp,model=pose.name,
                            x=pose.position.x,y=pose.position.y,z=pose.position.z,
                            quaternion=[pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z]))+'\n')
                        vehicle_file.flush()
                    return
        native.subscribe(Pose_V,'/world/mission2/pose/info',vehicle)
        native.subscribe(Pose_V,'/world/mission2/dynamic_pose/info',vehicle)
    with open(args.out, 'w') as f:
        def sample(m):
            f.write(json.dumps(dict(stamp=node.get_clock().now().nanoseconds*1e-9,
                                    x=m.position.x, y=m.position.y, z=m.position.z))+'\n')
            f.flush()
        node.create_subscription(Pose, '/sim/payload_pose', sample, qos_profile_sensor_data)
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
    with guard:
        stopped[0]=True
        if vehicle_file:vehicle_file.close()
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
