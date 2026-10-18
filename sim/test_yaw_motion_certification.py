"""A translation certificate must include the yaw command sent with it."""
import math
import numpy as np
from rclpy.parameter import Parameter
from test_corridor_clearance_contract import controller
from test_local_planner import wall_scan
from geometry_msgs.msg import PoseStamped


def turning_pose_history(c):
    c._pose_history.clear()
    for stamp,yaw in ((.8,-.1),(1.,0.)):
        c._now=lambda stamp=stamp:stamp
        pose=PoseStamped();pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);c._on_pose(pose)
    c._now=lambda:1.


def test_delayed_body_translation_and_yaw_leave_required_clearance(controller):
    c,sent=controller
    c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    turning_pose_history(c)
    c._planner.observe(*wall_scan(.78,1.75))
    c._ground_world=np.zeros(2);c._wind_world=np.zeros(2);c._wind_rate_world=np.zeros(2)
    c._compensated=lambda vx,vy:(vx,vy)
    c._publish(.8,0.,12.,1.,{})
    sp=sent[-1]
    velocity=np.zeros(2);point=np.zeros(2);dt=.005
    for t in np.arange(0.,5.,dt):
        if t<.4:goal=np.zeros(2)
        elif t<1.0:
            # The aircraft can already be turning while the packet travels.
            # ArduPilot converts body velocity once, at receipt. The following
            # response and stop retain that received earth-frame vector.
            yaw=sp.yaw_rate*.4;co,si=math.cos(yaw),math.sin(yaw)
            goal=np.array([co*sp.velocity.x-si*sp.velocity.y,si*sp.velocity.x+co*sp.velocity.y])
        else:goal=np.zeros(2)
        change=(goal-velocity)*dt/.6
        if t>=1.0:change*=min(1.,.5*dt/max(1e-12,float(np.linalg.norm(change))))
        velocity+=change;point+=velocity*dt
        assert .78-point[1]>=.70, 'transport-time yaw redirects certified body translation into a surface'


def test_received_body_packet_uses_heading_at_arrival_in_observer(controller):
    c,_=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    turning_pose_history(c);c._command_history.append((.4,np.array([1.,0.])))
    # The packet was sent at heading -0.1 rad and received at heading 0.
    c._pose_history.appendleft((.4,0.,0.,-.1,(0.,0.,math.sin(-.05),math.cos(-.05))))
    command=c._effective_command(.8)
    np.testing.assert_allclose(command,[1.,0.],atol=1e-12)
    # Now use a packet sent before the observed turn and received afterward.
    turning_pose_history(c)
    c._command_history.clear();c._command_history.append((.6,np.array([1.,0.])))
    c._pose_history.appendleft((.6,0.,0.,-.1,(0.,0.,math.sin(-.05),math.cos(-.05))))
    np.testing.assert_allclose(c._effective_command(1.),[math.cos(.1),math.sin(.1)],atol=1e-12)


def test_pending_body_pulse_cannot_be_declared_brakeable_during_turn(controller):
    c,sent=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    turning_pose_history(c);c._planner.observe(*wall_scan(.73,1.75))
    c._ground_world=np.zeros(2);c._wind_world=np.zeros(2);c._wind_rate_world=np.zeros(2)
    c._compensated=lambda vx,vy:(vx,vy);c._command_history.append((.8,np.array([2.,0.])))
    c._publish(.8,0.,12.,1.,{})
    assert c.last_detail.get('braking_clearance_available') is False
    assert sent[-1].velocity.x==sent[-1].velocity.y==0.


def test_in_flight_yaw_packet_changes_future_receipt_heading(controller):
    c,_=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    c._yaw_command_history.append((.8,.5))
    assert c._heading_at(1.1)==0.
    assert abs(c._heading_at(1.4)-.1)<1e-12
    c._yaw_command_history.append((.8,-.5))
    assert abs(c._heading_at(1.4)+.1)<1e-12


def test_stale_stop_retains_yaw_packets_already_in_transit(controller):
    c,_=controller;c.set_parameters([Parameter('vel_command_delay_s',value=.4)])
    c._yaw_command_history.append((.8,.5));c._command_history.append((.8,np.array([.8,0.])))
    c._sent_heading[.8]=0.
    c._publish(0.,0.,0.,0.,{'fault':'scan stale'})
    assert c._sent_heading[.8]==0.
    assert abs(c._heading_at(1.3)-.05)<1e-12
