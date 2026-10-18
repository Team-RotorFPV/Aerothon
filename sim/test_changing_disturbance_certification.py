"""Certify compensation against the disturbance evolution it compensates."""
import numpy as np
import pytest
from rclpy.parameter import Parameter
from test_corridor_clearance_contract import controller
from test_local_planner import wall_scan


@pytest.mark.parametrize('half_width,accepted',[(.85,False),(1.05,True)])
def test_forecast_compensation_requires_clearance_for_response_uncertainty(controller,half_width,accepted):
    c, sent = controller
    c.set_parameters([Parameter('vel_command_delay_s', value=0.)])
    c._planner.observe(*wall_scan(half_width,half_width))
    c._ground_world = np.zeros(2)
    c._wind_world = np.zeros(2)
    c._wind_rate_world = np.array([0., .5])
    c._compensated = lambda vx, vy: (0., -.3)
    c._publish(0., 0., 2., 0., {})
    # At .85 m the forecast is safe only if the estimated change happens.
    # With response error instead, the same packet's braking tail is unsafe.
    # Keep that narrow case as a refusal, and also prove compensation remains
    # available when both predictions have the unchanged .70 m clearance.
    if accepted:
        assert sent[-1].velocity.y == pytest.approx(-.3)
    else:
        assert sent[-1].velocity.y == pytest.approx(0.)
    assert c.last_detail['required_clearance_m']==.7


def test_unsafe_growing_disturbance_still_requires_full_clearance(controller):
    c, sent = controller
    c.set_parameters([Parameter('vel_command_delay_s', value=0.)])
    c._planner.observe(*wall_scan(.75, .75))
    c._ground_world = np.zeros(2)
    c._wind_world = np.zeros(2)
    c._wind_rate_world = np.array([0., .8])
    c._compensated = lambda vx, vy: (0., -.48)
    c._publish(0., 0., 2., 0., {})
    assert sent[-1].velocity.y != pytest.approx(-.48)
    assert c.last_detail['required_clearance_m'] == .7
