"""The real certificate must bound response error with no transport queue."""
import copy
import numpy as np
from rclpy.parameter import Parameter
from test_corridor_clearance_contract import controller
from test_local_planner import wall_scan
from test_headless_land_position_hold import world,DT


def test_helpful_response_error_cannot_authorize_unclear_braking_tail(controller):
    c,sent=controller
    c.set_parameters([Parameter('motion_computation_budget_s',value=.20)])
    # A 6 cm/s residual can be a response-model mismatch, rather than wind
    # that will cancel this issued packet. There is no artificial queue.
    c._planner.observe(*wall_scan(.735,1.75))
    c._wind_world=np.array([0.,-.06]);c._wind_rate_world=np.zeros(2)
    c._ground_world=np.zeros(2);c._compensated=lambda vx,vy:(0.,.06)
    c._publish(0.,0.,2.,0.,{})
    w=world();w.p[:2]=0.;w.v[:]=0.;w.wind=lambda t:(0.,0.)
    assert w.args.vel_command_delay_s==0. and w.args.vel_response_s==.6
    w._on_sp_raw(sent[-1]);closest=.735
    for _ in range(round(.6/DT)):
        w.t+=DT;w._step_airframe();closest=min(closest,.735-w.p[1])
    stop=copy.deepcopy(sent[-1]);stop.velocity.x=stop.velocity.y=0.
    w._on_sp_raw(stop)
    for _ in range(round(3.6/DT)):
        w.t+=DT;w._step_airframe();closest=min(closest,.735-w.p[1])
    assert closest>=.70, f'normal-response braking tail crossed the required clearance: {closest}'
