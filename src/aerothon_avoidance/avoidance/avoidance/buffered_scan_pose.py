"""Select paired lidar captures without discarding a still-fresh paired scan."""
import math
from types import MethodType
from avoidance.paired_scan_pose import capture_pose,install as install_pairing


def install(node):
    install_pairing(node)
    scan_callback=node._on_scan
    pose_callback=node._on_pose
    node._pending_paired_scan=None
    node._paired_scan_stamp=-math.inf
    node._scan_pairing_reason='no paired capture'

    def promote():
        pending=node._pending_paired_scan
        if pending is None:return
        stamp,message=pending
        age=node._now()-stamp
        if age<0:
            node._scan_pairing_reason='awaiting capture clock'
            return
        if age>float(node._g('stale_scan_s')):
            node._pending_paired_scan=None
            node._scan_pairing_reason='pending capture stale'
            return
        paired=capture_pose(node._pose_history,stamp)
        if paired is None:
            node._scan_pairing_reason='awaiting pose bracket; retaining paired scan while fresh'
            return
        scan_callback(message)
        node._scan_pose,node._scan_orientation=paired
        node._paired_scan_stamp=stamp
        node._pending_paired_scan=None
        node._scan_pairing_reason='capture paired'

    def on_scan(_node,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not math.isfinite(stamp) or stamp<0:
            _node.scan=_node._scan_t=_node._scan_pose=_node._scan_orientation=None
            _node._pending_paired_scan=None
            _node._scan_pairing_reason='invalid capture stamp'
            return
        pending=_node._pending_paired_scan
        newest=max(_node._paired_scan_stamp,pending[0] if pending else -math.inf)
        if stamp<=newest:
            return
        _node._pending_paired_scan=(stamp,message)
        promote()

    def on_pose(_node,message):
        pose_callback(message)
        promote()

    # ROS subscriptions retain the bound callbacks created in __init__. Wire
    # those public callback fields as well as direct calls used by regressions.
    wired=[]
    for subscription in node.subscriptions:
        if subscription.callback==scan_callback:
            subscription.callback=MethodType(on_scan,node);wired.append('scan')
        elif subscription.callback==pose_callback:
            subscription.callback=MethodType(on_pose,node);wired.append('pose')
    node._on_scan=MethodType(on_scan,node)
    node._on_pose=MethodType(on_pose,node)
    assert sorted(wired)==['pose','scan'],wired
