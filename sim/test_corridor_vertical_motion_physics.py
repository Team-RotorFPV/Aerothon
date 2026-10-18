"""Actual published vertical command through the unchanged airframe."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import numpy as np
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Bool,Float32
from test_altitude_hold import make_controller
from test_headless_land_position_hold import world,DT
ROOT=Path(__file__).resolve().parents[1]


def test_return_corridor_remains_above_ground_through_recorded_baro_wander():
    f=json.loads((ROOT/'sim/fixtures/corridor_return_altitude_mild_s4_20261008.json').read_text())
    w=world();w.t=f['epoch'];epoch=w.t;w.p=np.array([0.,0.,f['truth_alt']]);w.v=np.array([0.,0.,f['initial_vz']]);w.ground_velocity=w.v.copy();w.wind=lambda t:(0.,0.);w.gps_err[:]=0.
    errors=np.array(f['errors']);c=make_controller();c._now=lambda:w.t
    params=c._g;overrides={'stale_scan_s':.5,'drift_filter_s':1.,'motion_computation_budget_s':.2}
    c._g=lambda n:overrides[n] if n in overrides else params(n)
    c.get_clock=lambda:NS(now=lambda:NS(to_msg=w.stamp))
    c.pub_sp=NS(publish=w._on_sp_raw);c.pub_status=MagicMock();c.pub_detail=MagicMock();c.state='CRUISE';c._left_m=c._right_m=2.
    minimum=w.p[2];next_obs=next_command=epoch;enabled=False
    for _ in range(round(30/DT)):
        w.baro_err=float(np.interp(w.t,errors[:,0],errors[:,1]));est=w.est()
        if w.t>=next_obs:
            pose=PoseStamped();pose.header.stamp=w.stamp();pose.pose.orientation.w=1.;pose.pose.position.z=float(est[2]);c._on_pose(pose)
            velocity=TwistStamped();velocity.header.stamp=w.stamp();velocity.twist.linear.z=float(w.ground_velocity[2]);c._on_velocity(velocity)
            next_obs=w.t+.03-1e-9
        if not enabled:
            c._on_enable(Bool(data=True));c._on_hold_alt(Float32(data=f['hold_alt']));enabled=True
        if w.t>=next_command:
            c._on_hold_alt(Float32(data=f['hold_alt']))
            c._publish(0.,0.,0.,0.,{'fault':'evaluation of vertical publication only'})
            next_command=w.t+.05-1e-9
        w.t+=DT;w._step_airframe();minimum=min(minimum,float(w.p[2]))
    print(dict(minimum_height_m=minimum,final_height_m=float(w.p[2]),normal_response_s=w.args.vel_response_s,extra_queue_s=w.args.vel_command_delay_s))
    assert minimum>.30
    assert w.args.vel_response_s==.6 and w.args.vel_command_delay_s==0.
