#!/usr/bin/env python3
"""Red-zone detection that produces COORDINATES, not a boolean.

WHAT WAS WRONG (Phase 7)

    The node published one Bool — "red is visible somewhere in frame" — plus
    the fraction of the image it covered. Three separate problems:

      1. NO POSITION. A detection you cannot locate cannot be routed around,
         cannot be uploaded as a fence, and cannot be drawn on a map. The only
         possible response was to be vaguely careful.

      2. "NOT VISIBLE" AND "CLEAR" WERE THE SAME VALUE. `false` meant both
         "the camera is pointed at the sky / too low / not streaming" and "the
         camera can see that ground and it is clean". An operator cannot act
         on that, and neither can the mission.

      3. A SINGLE FRAME DECIDED. One red-ish blob — a jacket, a flare, wet
         clay — was enough.

WHAT IT DOES NOW

    Projects the red contours through the camera pose onto the ground plane
    (perception_redzone.georef), accumulates them across frames in a grid, and
    publishes confirmed exclusion rectangles in the local frame. Coverage
    status is explicit: NOT_VISIBLE / CLEAR / RED.

    The ArduPilot exclusion fence remains the authoritative boundary; this is
    the layer that tells the fence and the search planner WHERE.

Topics
  sub  <image_topic>              sensor_msgs/Image
  sub  /mavros/local_position/pose geometry_msgs/PoseStamped
  sub  /camera/pose_state          std_msgs/String   (camera_ctrl)
  pub  /percep/redzone             std_msgs/Bool     red in view (compatibility)
  pub  /percep/redzone/area        std_msgs/Float32  red fraction of frame
  pub  /percep/redzone/detail      std_msgs/String   JSON: status, exclusions
  pub  /percep/redzone/annotated   sensor_msgs/Image
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String

from perception_redzone.georef import GroundGrid, bbox, footprint, ground_point

NOT_VISIBLE = "NOT_VISIBLE"
CLEAR = "CLEAR"
RED = "RED"


class RedZoneNode(Node):
    def __init__(self):
        super().__init__('perception_redzone')
        p = self.declare_parameter
        p('image_topic', '/image_raw')
        p('min_area_frac', 0.002)      # smallest contour worth projecting
        p('s_lo', 90)
        p('v_lo', 60)
        # ---- geometry ----
        p('camera_hfov', 1.0472)
        p('nadir_pitch_tol_rad', 0.35)  # how far off nadir still georeferences
        p('min_altitude_m', 1.5)        # below this the projection is useless
        # ---- accumulation ----
        p('cell_m', 1.0)
        p('confirm_hits', 3)
        p('inflate_m', 1.0)
        p('samples_per_contour', 24)

        self.bridge = CvBridge()
        self.grid = GroundGrid(cell_m=float(self._g('cell_m')),
                               confirm_hits=int(self._g('confirm_hits')))
        self._pose = None
        self._cam_pitch = None
        self._hfov = float(self._g('camera_hfov'))

        topic = self._g('image_topic')
        self.create_subscription(Image, topic, self.on_image, 5)
        self.create_subscription(CameraInfo, '/camera_info', self.on_info, 5)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(String, '/camera/pose_state',
                                 self.on_camera_state, 10)

        self.pub = self.create_publisher(Bool, '/percep/redzone', 10)
        self.pub_area = self.create_publisher(Float32, '/percep/redzone/area', 10)
        self.pub_detail = self.create_publisher(String, '/percep/redzone/detail', 10)
        self.pub_annot = self.create_publisher(Image, '/percep/redzone/annotated', 5)
        self.get_logger().info(
            f"perception_redzone up (georeferenced); image_topic={topic}")

    def _g(self, n):
        return self.get_parameter(n).value

    # ------------------------------------------------------------------ #
    def on_info(self, m: CameraInfo):
        """Prefer the camera's own intrinsics over the declared HFOV.

        `m.k` is a numpy array: truth-testing it raises "truth value of an
        array is ambiguous", which killed a sibling node on its first message
        and was only caught by a live run.
        """
        if m.width > 0 and len(m.k) >= 1 and float(m.k[0]) > 0.0:
            self._hfov = 2.0 * math.atan((m.width / 2.0) / float(m.k[0]))

    def on_pose(self, m: PoseStamped):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self._pose = (m.pose.position.x, m.pose.position.y,
                      m.pose.position.z, yaw)

    def on_camera_state(self, m: String):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        # camera_ctrl reports the angle it has READ BACK from /joint_states,
        # not the one it commanded. Only a settled pose is worth projecting
        # through: mid-slew the geometry is a guess.
        if not d.get("settled", False) or d.get("stale", False):
            self._cam_pitch = None
            return
        angle = d.get("actual_rad")
        if angle is not None:
            # camera_ctrl's convention is negative = down (goal.md Q18);
            # georef takes positive-down.
            self._cam_pitch = -float(angle)

    # ------------------------------------------------------------------ #
    def can_georeference(self):
        """(ok, reason) — whether a bounded ground claim can be made at all."""
        if self._pose is None:
            return False, "no aircraft pose"
        if self._cam_pitch is None:
            return False, "camera pose unknown or still moving"
        alt = self._pose[2]
        if alt < float(self._g('min_altitude_m')):
            return False, f"altitude {alt:.1f} m too low to project"
        off_nadir = abs(self._cam_pitch - math.pi / 2)
        if off_nadir > float(self._g('nadir_pitch_tol_rad')):
            return False, (f"camera {math.degrees(off_nadir):.0f} deg off nadir; "
                           "view runs to the horizon")
        return True, ""

    def red_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        s_lo, v_lo = int(self._g('s_lo')), int(self._g('v_lo'))
        # Red wraps the hue circle -> two bands.
        m1 = cv2.inRange(hsv, (0, s_lo, v_lo), (10, 255, 255))
        m2 = cv2.inRange(hsv, (170, s_lo, v_lo), (179, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    def project_contour(self, contour, wh):
        """Ground points for a contour, skipping rays that miss the ground."""
        x, y, w, h = cv2.boundingRect(contour)
        n = max(2, int(self._g('samples_per_contour')) // 4)
        px, py, pz, yaw = self._pose
        pts = []
        for i in range(n + 1):
            for u, v in ((x + w * i / n, y), (x + w * i / n, y + h),
                         (x, y + h * i / n), (x + w, y + h * i / n)):
                g = ground_point(u, v, wh, self._hfov, pz, (px, py), yaw,
                                 self._cam_pitch)
                if g is not None:
                    pts.append(g)
        return pts

    # ------------------------------------------------------------------ #
    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]
        mask = self.red_mask(frame)
        frac = float(np.count_nonzero(mask)) / mask.size
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self._g('min_area_frac')) * w * h
        cnts = [c for c in cnts if cv2.contourArea(c) >= min_area]

        ok, why = self.can_georeference()
        detail = {"status": NOT_VISIBLE, "reason": why,
                  "red_frac": round(frac, 5), "blobs": len(cnts),
                  "exclusions": [], "confirmed_area_m2": 0.0,
                  "observed": None}

        if ok:
            px, py, pz, yaw = self._pose
            fp = footprint((w, h), self._hfov, pz, (px, py), yaw,
                           self._cam_pitch)
            detail["observed"] = [round(v, 2) for v in bbox(fp)] if fp else None
            # CLEAR is now a real claim: this bounded patch of ground was
            # looked at, and there was no red on it.
            detail["status"] = RED if cnts else CLEAR
            detail["reason"] = ""
            for c in cnts:
                self.grid.add(self.project_contour(c, (w, h)))

        detail["exclusions"] = [[round(v, 2) for v in ex]
                                for ex in self.grid.exclusions(
                                    inflate_m=float(self._g('inflate_m')))]
        detail["confirmed_area_m2"] = round(self.grid.area_m2(), 1)

        colour = (0, 0, 255) if cnts else (0, 200, 0)
        cv2.drawContours(frame, cnts, -1, colour, 2)
        cv2.putText(frame, detail["status"], (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

        self.pub.publish(Bool(data=bool(cnts)))
        self.pub_area.publish(Float32(data=frac))
        self.pub_detail.publish(String(data=json.dumps(detail)))
        try:
            self.pub_annot.publish(self.bridge.cv2_to_imgmsg(frame,
                                                             encoding='bgr8'))
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = RedZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
