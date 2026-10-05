#!/usr/bin/env python3
"""The simulated vehicle is the team's airframe, built from its CAD.

What is checked is what the physics and the perception stack depend on, not
the mesh: rotors at the CAD motor hubs in ArduPilot's quad-X order and spin,
thrust scaled to the flown prop, the LD06 and C270 at their CAD mounts with
their own specs, the lidar blind to the props, the claw where the CAD
draws it, and the TF tree (uav.urdf.xacro) agreeing with the Gazebo model.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest sim/test_cad_vehicle.py -v
"""

import json
import math
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
AIRFRAME = ROOT / "src/aerothon_sim/sim_gazebo/models/aerothon_quad"
UPSTREAM = Path("/home/sarthak/aerothon_stack/src/ardupilot_gazebo/models")


@unittest.skipUnless((UPSTREAM / "iris_with_ardupilot" / "model.sdf").exists(),
                     "needs the upstream ardupilot_gazebo models")
class CadVehicleTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import build_cad_vehicle as B
        cls.B = B
        cls.af = json.loads((AIRFRAME / "airframe.json").read_text())
        cls.tmp = tempfile.TemporaryDirectory()
        mdir, cls.info = B.build(AIRFRAME, UPSTREAM, Path(cls.tmp.name))
        cls.model = ET.parse(mdir / "model.sdf").getroot().find("model")
        cls.mdir = mdir

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def link_pose(self, name):
        return [float(v) for v in self.model.find(f"link[@name='{name}']/pose").text.split()]

    def test_rotors_are_at_the_cad_hubs_in_quad_x_order(self):
        order = ["Front Right", "Back Left", "Front Left", "Back Right"]
        for i, corner in enumerate(order):
            x, y, z = self.link_pose(f"rotor_{i}")[:3]
            hub = self.af["motors"][corner]["hub"]
            self.assertAlmostEqual(x, hub[0], places=4)
            self.assertAlmostEqual(y, hub[1], places=4)
            self.assertAlmostEqual(z, self.af["prop_plane_z"], places=4)
        # ArduPilot motors 1..4: FR and BL spin one way, FL and BR the other.
        plug = self.model.find("plugin[@name='ArduPilotPlugin']")
        mul = {int(c.get("channel")): float(c.find("multiplier").text)
               for c in plug.findall("control")}
        self.assertEqual(sorted(mul), [0, 1, 2, 3])
        self.assertGreater(mul[0] * mul[1], 0)
        self.assertGreater(mul[2] * mul[3], 0)
        self.assertLess(mul[0] * mul[2], 0)

    def test_maximum_thrust_is_the_flown_props(self):
        """Four rotors at top speed lift ~4.7 kg: thrust-to-weight ~2.3 on
        2.0 kg, what a 2312 980 KV gives on a 9450 at 4S."""
        lds = [p for p in self.model.findall("plugin")
               if "lift-drag" in p.get("filename", "")]
        self.assertEqual(len(lds), 8)
        w = self.info["w_max_rad_s"]
        total = 0.0
        for p in lds:
            cp = abs(float(p.find("cp").text.split()[0]))
            area = float(p.find("area").text)
            cla = float(p.find("cla").text)
            a0 = float(p.find("a0").text)
            total += 0.5 * 1.2041 * (w * cp) ** 2 * area * cla * a0
        self.assertAlmostEqual(total / 9.81 / 2.0, 2.34, delta=0.1)

    def test_rotor_torque_per_thrust_is_a_real_props(self):
        """(cda/cla)*cp is the rotor's yaw torque per newton of thrust. The
        Iris's blade gives 0.0019 m, a sixth of a real 9x4.5, and the yaw
        demand of every sweep step took the thrust with it."""
        for p in self.model.findall("plugin"):
            if "lift-drag" not in p.get("filename", ""):
                continue
            ratio = (float(p.find("cda").text) / float(p.find("cla").text)
                     * abs(float(p.find("cp").text.split()[0])))
            self.assertAlmostEqual(ratio, 0.0125, delta=0.0005)

    def test_mass_is_the_all_up_weight(self):
        m = sum(float(l.find("inertial/mass").text) for l in self.model.findall("link"))
        self.assertAlmostEqual(m, 2.0, delta=0.04)

    def test_the_lidar_is_an_ld06_at_its_mount_and_blind_to_the_props(self):
        scan = self.model.find("link[@name='base_scan']")
        x, y, z = self.link_pose("base_scan")[:3]
        self.assertAlmostEqual(x, self.af["lidar"][0][0], places=3)
        self.assertAlmostEqual(z, self.af["lidar_scan_z"], places=3)
        lidar = scan.find("sensor/lidar")
        self.assertEqual(lidar.find("scan/horizontal/samples").text, "450")
        self.assertEqual(float(lidar.find("range/max").text), 12.0)
        mask = int(lidar.find("visibility_mask").text)
        for i in range(4):
            flags = int(self.model.find(f"link[@name='rotor_{i}']/visual/visibility_flags").text)
            self.assertEqual(mask & flags, 0, "the lidar would see the props")
        # ...and the scan plane really is barely above the prop disc.
        self.assertLess(self.af["lidar_scan_z"] - self.af["prop_top_z"], 0.01)

    def test_the_camera_is_a_c270_on_the_tilt_servo(self):
        cam = self.model.find("link[@name='webcam_link']/sensor/camera")
        self.assertAlmostEqual(float(cam.find("horizontal_fov").text),
                               math.radians(48.8), delta=0.005)
        j = self.model.find("joint[@name='webcam_pitch_joint']")
        self.assertEqual(j.find("axis/xyz").text, "0 -1 0")
        self.assertAlmostEqual(self.link_pose("webcam_link")[0], self.af["camera"][0][0], places=3)

    def test_the_tilt_servo_is_stiff_enough_to_settle(self):
        """At the Iris's gains (P 8, I 0.1) the C270 sat 2.3 deg short of the
        banner pose and 1.6 deg short of nadir, both outside camera_ctrl's
        2 deg settle tolerance: the mission failed "camera did not reach
        BANNER" on two flights. Measured on the vehicle alone at P 40 / I 4:
        within 0.3 deg."""
        ctl = next(p for p in self.model.findall("plugin")
                   if (p.findtext("joint_name") or "") == "webcam_pitch_joint")
        self.assertGreaterEqual(float(ctl.find("p_gain").text), 40)
        self.assertGreaterEqual(float(ctl.find("i_gain").text), 4)

    def test_the_claw_is_where_the_cad_draws_it(self):
        """The line holds the claw's top pin and the payload hangs from its
        jaws. (The hook used to be a point placed 3.3 cm above the CAD's, so
        that a payload on it cleared the ground; on the drawn claw the
        rulebook payload hangs below the skids -- see sim/winch_bench.py.)"""
        claw = self.af["claw"]
        for link, pin in (("winch_hook", "top_pin"), ("claw_pivot", "centre_pin"),
                          ("claw_link_a", "top_pin"), ("claw_jaw_a", "centre_pin")):
            for a, b in zip(self.link_pose(link)[:3], claw[pin]):
                self.assertAlmostEqual(a, b, places=4, msg=link)
        dj = next(p for p in self.model.findall("plugin")
                  if "DetachableJoint" in p.get("name", ""))
        self.assertEqual(dj.findtext("parent_link"), "claw_pivot")
        self.assertTrue(dj.findtext("attach_topic"))

    def test_the_tf_tree_agrees_with_the_gazebo_model(self):
        xacro = ROOT / "src/aerothon_mission/uav_description/urdf/uav.urdf.xacro"
        try:
            urdf = subprocess.run(["xacro", str(xacro)], capture_output=True,
                                  text=True, check=True).stdout
        except (OSError, subprocess.CalledProcessError):
            self.skipTest("xacro not available")
        r = ET.fromstring(urdf)
        def origin(joint):
            return [float(v) for v in r.find(f"joint[@name='{joint}']/origin").get("xyz").split()]
        for got, link in ((origin("rplidar_c1_mount"), "base_scan"),
                          (origin("webcam_servo_mount"), "webcam_servo_base")):
            want = self.link_pose(link)[:3]
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b, delta=0.002, msg=link)

    def test_it_is_valid_sdf(self):
        try:
            out = subprocess.run(["gz", "sdf", "-k", str(self.mdir / "model.sdf")],
                                 capture_output=True, text=True, timeout=60)
        except (OSError, subprocess.TimeoutExpired):
            self.skipTest("gz not available")
        self.assertIn("Valid", out.stdout + out.stderr)


if __name__ == "__main__":
    unittest.main()


class AirframeTuningTests(unittest.TestCase):
    """The team airframe flies on its own ArduCopter tuning, not the Iris's."""

    def params(self):
        out = {}
        for line in (ROOT / "src/aerothon_sim/sim_gazebo/config/aerothon_quad.parm").read_text().splitlines():
            line = line.split("#", 1)[0].split()
            if len(line) == 2:
                out[line[0]] = float(line[1])
        return out

    def test_it_hovers_at_its_own_throttle(self):
        """The Iris's learned 0.324 against the ~0.48 this frame needs left the
        altitude loop's integrator carrying 15 % of throttle all flight."""
        self.assertAlmostEqual(self.params()["MOT_THST_HOVER"], 0.48, delta=0.03)

    def test_rate_gains_are_raised_for_its_authority(self):
        p = self.params()
        self.assertGreater(p["ATC_RAT_RLL_P"], 0.135)
        self.assertGreater(p["ATC_RAT_PIT_P"], 0.135)
        self.assertLess(p["ATC_ACCEL_R_MAX"], 110000)

    def test_yaw_is_capped_so_it_cannot_take_the_thrust(self):
        """00000169.BIN: a 45 deg sweep step saturated a motor and the frame
        sank 4.9 m to the ground. Yaw demand is limited to what 9.45 in props
        can deliver, and thrust has priority."""
        p = self.params()
        self.assertLessEqual(p["ATC_ACCEL_Y_MAX"], 27000)
        self.assertTrue(0 < p["ATC_RATE_Y_MAX"] <= 60)
        self.assertLess(p["MOT_YAW_HEADROOM"], 200)
        self.assertLessEqual(p["ATC_RAT_YAW_P"], 0.4)

    def test_the_launch_loads_it_and_a_stale_eeprom_cannot_override_it(self):
        launch = (ROOT / "src/aerothon_sim/sim_gazebo/launch/sim_full.launch.py").read_text()
        self.assertIn("aerothon_quad.parm", launch)
        sh = (ROOT / "scripts/launch_level6_sim.sh").read_text()
        self.assertIn('rm -f "$PWD/eeprom.bin"', sh)
