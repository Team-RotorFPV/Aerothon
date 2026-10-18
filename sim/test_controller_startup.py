"""Execute the proposed normal entry point, including capture-driver cleanup."""
import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import pytest

from avoidance import velocity_controller as controller
from avoidance import buffered_scan_pose, independent_ros_clock_candidate, capture_inbox, motion_forecast_trace

@pytest.mark.parametrize('recording', [False, True])
@pytest.mark.parametrize('capture_failure', [False, True])
def test_normal_startup_and_partial_failure_cleanup(recording, capture_failure):
    events = []
    node = SimpleNamespace(destroy_node=lambda: events.append('destroy'))
    def driver(kind, fail=False):
        def create(value):
            assert value is node
            events.append(kind)
            if fail:
                raise RuntimeError('capture startup failed')
            return SimpleNamespace(close=lambda: events.append(kind+'.close'))
        return create
    def trace(value,path):
        assert value is node and path == 'explicit-diagnostic.json'
        events.append('trace')
        return SimpleNamespace(close=lambda: events.append('trace.close'))
    def spin(value):
        assert value is node
        events.append('spin')
        raise KeyboardInterrupt
    environment = dict(os.environ)
    environment.pop('AEROTHON_MOTION_FORECAST_OUT', None)
    if recording:
        environment['AEROTHON_MOTION_FORECAST_OUT'] = 'explicit-diagnostic.json'
    with patch.dict(os.environ, environment, clear=True), \
         patch.object(controller, 'VelocityController', return_value=node), \
         patch.object(controller.rclpy, 'init', side_effect=lambda: events.append('init')), \
         patch.object(controller.rclpy, 'spin', side_effect=spin), \
         patch.object(controller.rclpy, 'ok', return_value=True), \
         patch.object(controller.rclpy, 'shutdown', side_effect=lambda: events.append('shutdown')), \
         patch.object(buffered_scan_pose, 'install', side_effect=lambda value: events.append('pairing')), \
         patch.object(independent_ros_clock_candidate, 'IndependentRosClock', driver('clock')), \
         patch.object(capture_inbox, 'CaptureInbox', driver('capture', capture_failure)), \
         patch.object(motion_forecast_trace, 'MotionForecastTrace', trace):
        if capture_failure:
            with pytest.raises(RuntimeError, match='capture startup failed'):
                controller.main()
        else:
            controller.main()
    expected = ['init','pairing','clock','capture']
    if not capture_failure:
        if recording:
            expected.append('trace')
        expected.append('spin')
        if recording:
            expected.append('trace.close')
        expected.append('capture.close')
    assert events == expected+['clock.close','destroy','shutdown']
