"""A certificate cannot authorize movement after its computation budget expires."""
import numpy as np
from test_corridor_clearance_contract import controller
from test_local_planner import wall_scan
from rclpy.parameter import Parameter

def computation_budget(c):
    if c.has_parameter('motion_computation_budget_s'):
        c.set_parameters([Parameter('motion_computation_budget_s', value=.2)])


def test_expired_certificate_cannot_publish_travel(controller):
    c, sent = controller
    computation_budget(c)
    c._planner.observe(*wall_scan(1.75, 1.75))
    c._compensated = lambda vx,vy: (vx,vy)
    check = c._planner.polyline_clear
    clock = [1.]
    c._now = lambda: clock[0]
    def delayed_check(*args):
        result = check(*args)
        clock[0] = 1.25
        return result
    c._planner.polyline_clear = delayed_check
    c._publish(.8, 0., 12., 0., {})
    assert sent[-1].velocity.x == sent[-1].velocity.y == sent[-1].yaw_rate == 0.
    assert c.last_detail['fault'] == 'motion certificate computation deadline exceeded'


def test_prompt_certificate_retains_command_and_full_clearance(controller):
    c, sent = controller
    computation_budget(c)
    c._planner.observe(*wall_scan(1.75, 1.75))
    c._compensated = lambda vx,vy: (vx,vy)
    c._publish(.8, 0., 12., 0., {})
    assert sent[-1].velocity.x > 0.
    assert c.last_detail['required_clearance_m'] == .70


def test_pose_aging_during_certificate_stops_publication(controller):
    c, sent = controller
    computation_budget(c)
    c._planner.observe(*wall_scan(1.75, 1.75))
    c._compensated = lambda vx,vy: (vx,vy)
    check = c._planner.polyline_clear
    def age_pose(*args):
        result = check(*args)
        c._pose_t = -1.
        return result
    c._planner.polyline_clear = age_pose
    c._publish(.8, 0., 12., 0., {})
    assert sent[-1].velocity.x == sent[-1].velocity.y == 0.
    assert c.last_detail['fault'] == 'post-certificate flight observations stale'


def test_deadline_stop_preserves_packets_already_transmitted(controller):
    c, sent = controller
    computation_budget(c)
    c._planner.observe(*wall_scan(1.75, 1.75))
    c._compensated = lambda vx,vy: (vx,vy)
    c._command_history.append((.8,np.array([.4,0.])))
    c._sent_heading[.8] = 0.
    check = c._planner.polyline_clear
    clock = [1.]
    c._now = lambda: clock[0]
    def delayed_check(*args):
        result = check(*args)
        clock[0] = 1.25
        return result
    c._planner.polyline_clear = delayed_check
    c._publish(.8,0.,12.,0.,{})
    assert c._command_history[0][0] == .8
    assert c._sent_heading[.8] == 0.
    assert sent[-1].velocity.x == sent[-1].velocity.y == 0.
