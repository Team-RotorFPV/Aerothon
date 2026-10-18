"""Exercise the operator driver's landing loop with MAVROS link loss."""
import json
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import corridor_trial


@pytest.mark.parametrize('landing_state', ['disconnected', 'stale', 'disarmed', 'no_pose'])
def test_driver_requires_fresh_connected_disarm(tmp_path, monkeypatch, landing_state):
    import rclpy
    import rclpy.node
    import mission_bt.mav_commander

    clock = NS(sim=1., wall=0.)
    states = []

    class Node:
        def __init__(self, *args, **kwargs): self.callbacks = []; self.mav = None
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=round(clock.sim*1e9)))
        def create_subscription(self, message, topic, callback, qos):
            if topic == '/mavros/state': self.callbacks.append(callback)
        def create_publisher(self, *args): return NS(publish=lambda message: None)
        def destroy_node(self): pass

    class Mav:
        def __init__(self, node):
            node.mav = self; self.node = node; self.state = State(connected=True, mode='GUIDED')
            self._pos = (0., 0., 0.); self._yaw = 0.; self.landing = False
            self.avoid_detail = dict(corridor_exited=True, axis_progress_m=5.)
            self.abort_requested = False; self.pub_result = NS(publish=lambda message: None)
        def alt(self): return self._pos[2]
        def pos(self): return self._pos

        @property
        def pose(self):
            """The onboard local position the landing contract is written from."""
            message = PoseStamped()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = self._pos
            stamp = 0. if landing_state == 'no_pose' else clock.sim
            message.header.stamp.sec = int(stamp)
            message.header.stamp.nanosec = round((stamp % 1)*1e9)
            return message
        def yaw(self): return self._yaw
        def arm(self): self.state.armed = True
        def set_mode(self, mode): self.state.mode = mode
        def takeoff(self, z): self._pos = (0., 0., z)
        def goto(self, x, y, z, yaw): self._pos = (x, y, z); self._yaw = yaw
        def reached(self, *args): return True
        def set_camera_pose(self, *args): pass
        def enable_avoidance(self, *args, **kwargs): pass
        def avoidance_stuck(self): return False
        def land(self):
            self.landing = True
            self.state = State(connected=landing_state != 'disconnected', armed=False, mode='LAND')
            if landing_state != 'stale': self.state.header.stamp.sec = int(clock.sim)
            for callback in self.node.callbacks: callback(self.state)

    def spin(node, timeout_sec):
        clock.sim += 1.
        if not node.mav.landing:
            node.mav.state.header.stamp.sec = int(clock.sim)
        for callback in node.callbacks: callback(node.mav.state)
        states.append(node.mav.state.connected)

    def monotonic(): clock.wall += 1.; return clock.wall

    fixture = tmp_path/'entry.json'
    fixture.write_text(json.dumps(dict(x=5., y=0., alt=1.65, yaw=0., approach=dict(x=0., y=0.))))
    monkeypatch.setattr(corridor_trial.sys, 'argv', ['corridor_trial', '--entry', str(fixture), '--out', str(tmp_path)])
    monkeypatch.setattr(corridor_trial.time, 'monotonic', monotonic)
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    monkeypatch.setattr(rclpy.node, 'Node', Node)
    monkeypatch.setattr(mission_bt.mav_commander, 'Mav', Mav)
    rc = corridor_trial.main()
    result = json.loads((tmp_path/'trial_result.json').read_text())
    # The declared land-in-place is written at the LAND command, from the
    # onboard pose alone. A stale pose records the failure, never an anchor.
    contract = json.loads((tmp_path/'landing_contract.json').read_text())
    assert contract['frame'] == 'home_local_enu' and contract['radius_m'] == 1.5
    if landing_state == 'no_pose':
        assert contract['anchor'] is None and 'failure' in contract
    else:
        assert contract['anchor'] == [5., 0.] and contract['pose_capture_stamp'] > 0
    if landing_state in ('disarmed', 'no_pose'):
        assert rc == 0 and result['final_armed'] is False
    else:
        assert rc == 1 and result['final_armed'] is not False
