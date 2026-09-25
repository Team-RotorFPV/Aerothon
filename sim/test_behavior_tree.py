import sys
import unittest
import math
from unittest.mock import MagicMock

# Substitute mavros_msgs ONLY when it is genuinely unavailable (e.g. a laptop
# without ROS). Checking `'mavros_msgs' not in sys.modules` was wrong: on a
# machine that HAS mavros_msgs the module simply is not imported yet, so this
# installed a MagicMock into sys.modules for the whole pytest session and every
# later test file silently received mocks instead of real messages. Because
# pytest collects alphabetically, that broke sim/test_phase0_rails.py — 22
# tests failed on `pytest sim/` while passing when run on their own.
try:
    import mavros_msgs.msg  # noqa: F401
    import mavros_msgs.srv  # noqa: F401
except ImportError:
    m_msgs = MagicMock()
    sys.modules['mavros_msgs'] = m_msgs
    sys.modules['mavros_msgs.msg'] = m_msgs
    sys.modules['mavros_msgs.srv'] = m_msgs

from mission_bt.mission_tree import (
    CheckAbortTriggered,
    StageAwareAbort,
    SetModeArm,
    Takeoff,
    WinchDrop,
    LawnmowerSearch,
    build_root,
)
import py_trees


class _Done:
    """A service future that has already completed."""

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class MockMav:
    def __init__(self):
        self.state = MagicMock(connected=True, armed=False, mode="STABILIZE")
        self.battery = MagicMock(voltage=15.2, percentage=0.85)
        self.pose = MagicMock()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.abort_requested = False
        self.abort_reason = ""
        self.abort_latched = False
        self.mission_started = True  # test the active mission branch; real M2 starts only from GCS
        # Phase 0 rails (see sim/test_phase0_rails.py for the real-object tests)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.attitude_over_limit = False
        self.low_alt_fault = False
        self.airborne_floor = None
        self.winch_status = {}
        self.target_override = ""
        self.qr_streak = 0
        self.mission_failed = False
        self.expect_disarm_flag = False
        self.results = []
        self.qr_decoded = ""
        self.qr_matched = False
        self.last_mode = None
        self.armed_cmd = None
        self.avoidance_enabled = None
        self.winch_cmds = []
        # Organiser inputs: delivery-zone boundary and arena geofence.
        self.delivery_zone_local = (12.0, 52.0, -15.0, 15.0)
        self.geofence_local = [(-9.5, -21.0), (58.0, -21.0), (58.0, 21.0),
                               (-9.5, 21.0)]
        self.fence_readback = None
        self.fence_verified = False
        self.fence_reason = ""
        self.params = {}

    def delivery_search_zone(self, clearance_m):
        x0, x1, y0, y1 = self.delivery_zone_local
        c = clearance_m
        return (x0 + c, x1 - c, y0 + c, y1 - c)

    def home_global(self):
        return (-35.3632621, 149.1652374)

    def home_local_xy(self):
        return (0.0, 0.0)

    def push_fence(self, items):
        self.fence_readback = list(items)      # FC echoes it back intact
        return _Done(MagicMock(success=True))

    def set_param(self, name, value):
        self.params[name] = value
        return _Done(MagicMock(success=True))

    def yaw(self):
        return 0.0

    def log(self, msg, warn=False):
        pass

    def connected(self):
        return self.state.connected

    def battery_critical(self, min_volt=13.2, min_pct=0.15):
        if self.battery.voltage > 0.0 and self.battery.voltage < min_volt:
            return True
        if self.battery.percentage > 0.0 and self.battery.percentage < min_pct:
            return True
        return False

    def set_mode(self, mode):
        self.last_mode = mode

    def arm(self, val):
        self.armed_cmd = val

    def takeoff(self, alt):
        pass

    def land(self):
        self.last_mode = "LAND"

    def goto(self, x, y, z, yaw=0.0):
        pass

    def pos(self):
        return (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z)

    def reached(self, x, y, z, tol=0.6):
        px, py, pz = self.pos()
        return math.dist((px, py, pz), (x, y, z)) < tol

    def alt(self):
        return self.pose.pose.position.z

    def enable_avoidance(self, on, hold_alt=None):
        self.avoidance_enabled = on

    def winch(self, cmd):
        self.winch_cmds.append(cmd)

    # ---- Phase 0 rails ---- #
    def attitude_excessive(self):
        return self.attitude_over_limit

    def low_altitude_fault(self, samples_required=8):
        return self.low_alt_fault

    def set_airborne_floor(self, f):
        self.airborne_floor = f

    def clear_airborne_floor(self):
        self.airborne_floor = None

    def qr_confident(self, frames):
        return bool(self.qr_decoded) and self.qr_streak >= frames

    def expect_disarm(self, value=True):
        self.expect_disarm_flag = bool(value)

    def publish_result(self, state, reason=""):
        if not self.results:
            self.results.append((state, reason))

    def consume_reset(self):
        return None


class TestMissionBT(unittest.TestCase):
    def setUp(self):
        self.mav = MockMav()
        self.node = MagicMock()
        self.defaults = {
            'takeoff_alt': 5.0, 'search_alt': 10.0, 'drop_alt': 5.0,
            'image_width_px': 1280, 'camera_hfov': 1.0472,
            'target_marker_m': 2.2, 'qr_modules': 33,
            'px_per_module_floor': 5.3, 'lane_overlap': 0.30,
            'zone_margin': 1.0, 'corridor_alt': 3.0,
            'waypoint_tol': 0.8, 'drop_tol': 0.5, 'scan_floor_alt': 2.0,
        }

    def test_guard_healthy_does_not_abort(self):
        """When system is healthy, CheckAbortTriggered returns FAILURE, so AbortBranch fails and Mission runs."""
        guard = CheckAbortTriggered(self.mav, self.node)
        status = guard.update()
        self.assertEqual(status, py_trees.common.Status.FAILURE)

    def test_guard_disconnected_aborts(self):
        """An FCU link down for the whole grace trips the abort."""
        now = [100.0]
        self.mav.state.connected = False
        guard = CheckAbortTriggered(self.mav, self.node, clock=lambda: now[0])
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        now[0] += guard.link_grace_s
        self.assertEqual(guard.update(), py_trees.common.Status.SUCCESS)
        self.assertIn("FCU disconnected", guard.feedback_message)

    def test_guard_rides_out_a_heartbeat_blip(self):
        """Arena 1001, batch E: MAVROS declared the link lost and had it back
        0.27 s later, with the aircraft hovering armed in GUIDED. That must
        not end the mission."""
        now = [100.0]
        guard = CheckAbortTriggered(self.mav, self.node, clock=lambda: now[0])
        self.mav.state.connected = False
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        now[0] += 0.27
        self.mav.state.connected = True
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        # A later drop gets its own full grace, not what is left of the first.
        now[0] += 10.0
        self.mav.state.connected = False
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        now[0] += guard.link_grace_s * 0.9
        self.assertEqual(guard.update(), py_trees.common.Status.FAILURE)
        self.assertFalse(self.mav.abort_latched)

    def test_land_does_not_complete_on_a_dropped_link(self):
        """A dropped link reads as armed=False; that is not a landing."""
        from mission_bt.mission_tree import Land
        land = Land(self.mav)
        land.initialise()
        self.mav.state.armed = False
        self.mav.state.connected = False
        self.assertEqual(land.update(), py_trees.common.Status.RUNNING)
        self.assertEqual(self.mav.results, [])
        self.mav.state.connected = True
        self.assertEqual(land.update(), py_trees.common.Status.SUCCESS)
        self.assertEqual(self.mav.results[0][0], "COMPLETED")

    def test_guard_battery_critical_aborts(self):
        """When battery drops below threshold, CheckAbortTriggered returns SUCCESS."""
        self.mav.battery.voltage = 12.8
        guard = CheckAbortTriggered(self.mav, self.node)
        status = guard.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_guard_explicit_abort(self):
        """When abort is requested from GCS, CheckAbortTriggered returns SUCCESS."""
        self.mav.abort_requested = True
        guard = CheckAbortTriggered(self.mav, self.node)
        status = guard.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_root_tree_ticks_mission_when_healthy(self):
        """Verify the full tree ticks into the mission branch when healthy without premature RTL."""
        root = build_root(self.mav, self.node, self.defaults)
        root.setup_with_descendants()
        
        # The pre-flight stages (delivery zone, fence push/read-back/params)
        # take a few ticks; SetModeArm must then ask for GUIDED.
        for _ in range(10):
            root.tick_once()
            if self.mav.last_mode:
                break
        self.assertEqual(self.mav.last_mode, "GUIDED")
        self.assertNotEqual(self.mav.last_mode, "RTL")
        self.assertEqual(self.mav.params.get("FENCE_ENABLE"), 1)
        self.assertTrue(self.mav.fence_verified)

    def test_root_tree_preempts_to_abort_when_aborted(self):
        """Verify that when an abort is triggered, the tree immediately invokes StageAwareAbort."""
        root = build_root(self.mav, self.node, self.defaults)
        root.setup_with_descendants()
        
        self.mav.abort_requested = True
        root.tick_once()
        self.assertEqual(self.mav.last_mode, "LAND")  # low alt -> land

    def test_winch_drop_latches_coordinates(self):
        """Verify WinchDrop latches drop_x and drop_y upon initialise."""
        self.mav.pose.pose.position.x = 35.0
        self.mav.pose.pose.position.y = -6.0
        self.mav.pose.pose.position.z = 10.0
        
        drop = WinchDrop(self.mav, drop_alt=5.0, cruise_alt=10.0)
        drop.initialise()
        self.assertEqual(drop.drop_x, 35.0)
        self.assertEqual(drop.drop_y, -6.0)


if __name__ == '__main__':
    unittest.main()
