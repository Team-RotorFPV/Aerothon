#!/usr/bin/env python3
"""Supply the delivery-zone boundary the mission refuses to fly without.

The competition hands the team four geographic corners before flight, and the
mission treats a missing or invalid boundary as not-ready: `gcs_readiness`
reports `delivery_zone` unknown, `/mission_ready` stays false, and the aircraft
will not arm.  Nothing in the simulator played the organiser's part, so every
simulated run stopped there with a correct but unhelpful complaint.

This node is that organiser.  It waits for the FCU home position -- the one
global reference the rules allow -- converts the arena's delivery field from
local ENU into WGS84 about that home, and publishes the four corners latched on
``/mission/delivery_zone``.

It is a SIMULATION INPUT, not mission logic.  On a real flight the corners come
from the organisers and this node is not launched.

The rectangle defaults to the shipped arena's `delivery_zone_40x30` model:
centre (32, 0) in the world, which is (34, -2) about the FCU home at world
(-2, 2); 40 m across by 30 m deep.  A randomised arena moves
it, and `materialize_world.py --randomise-arena` prints that seed's centre;
pass it through ``AEROTHON_DELIVERY_ZONE="cx,cy,width,height"``.

The ENU->WGS84 conversion is imported from `mission_bt.geofence` rather than
rewritten here on purpose: the mission converts the corners straight back, and
`boundary_to_local_zone` rejects a polygon whose corners miss an axis-aligned
rectangle by more than 0.75 m.  Two nearly-identical formulas would be a slow
way to discover that.
"""

import json
import math
import os

from mission_bt.geofence import local_to_global

# HOME-LOCAL, not world: the vehicle (and so the FCU home) spawns at world
# (-2, 2), so the shipped field at world (32, 0) is home-local (34, -2).
DEFAULT_ZONE = (34.0, -2.0, 40.0, 30.0)
TOPIC = "/mission/delivery_zone"
# The ARENA geofence the rulebook says will be provided, as home-local
# (x0, x1, y0, y1). The default is the shipped arena's, from
# materialize_world.shipped_layout(); every launch passes the arena's own via
# AEROTHON_GEOFENCE="x0,x1,y0,y1".
GEOFENCE_TOPIC = "/mission/geofence"
DEFAULT_GEOFENCE = (-7.5, 60.0, -23.0, 19.0)


def geofence_from_env(environ=None):
    """Read ``AEROTHON_GEOFENCE``, local ENU.

    ``x0,x1,y0,y1`` is a rectangle and comes back as that 4-tuple; a vertex
    list ``x,y;x,y;x,y;...`` (a user-built arena's polygon) comes back as a
    list of (x, y).
    """
    environ = os.environ if environ is None else environ
    raw = (environ.get("AEROTHON_GEOFENCE") or "").strip()
    if not raw:
        return DEFAULT_GEOFENCE
    if ";" in raw:
        verts = []
        for pair in raw.split(";"):
            if not pair.strip():
                continue
            xy = [float(v) for v in pair.split(",")]
            if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
                raise ValueError("AEROTHON_GEOFENCE vertices must be 'x,y;x,y;...'")
            verts.append((xy[0], xy[1]))
        if len(verts) < 3:
            raise ValueError("AEROTHON_GEOFENCE polygon needs at least 3 vertices")
        return verts
    parts = [float(v) for v in raw.split(",")]
    if len(parts) != 4 or not all(math.isfinite(v) for v in parts):
        raise ValueError("AEROTHON_GEOFENCE must be 'x0,x1,y0,y1'")
    x0, x1, y0, y1 = parts
    if x0 >= x1 or y0 >= y1:
        raise ValueError("AEROTHON_GEOFENCE must have x0 < x1 and y0 < y1")
    return x0, x1, y0, y1


def fence_vertices(fence):
    """A rectangle (x0, x1, y0, y1) or a vertex list, as a vertex list."""
    if len(fence) == 4 and not isinstance(fence[0], (tuple, list)):
        x0, x1, y0, y1 = fence
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [tuple(v) for v in fence]


def geofence_payload(fence, home_lat, home_lon):
    """The geofence as the mission's polygon JSON (ring order)."""
    vertices = []
    for x, y in fence_vertices(fence):
        lat, lon = local_to_global(x, y, home_lat, home_lon)
        vertices.append({"lat": lat, "lon": lon})
    return json.dumps({"vertices": vertices}, separators=(",", ":"))


def zone_from_env(environ=None):
    """Read ``AEROTHON_DELIVERY_ZONE`` as ``(cx, cy, width, height)``."""
    environ = os.environ if environ is None else environ
    raw = (environ.get("AEROTHON_DELIVERY_ZONE") or "").strip()
    if not raw:
        return DEFAULT_ZONE
    parts = raw.split(",")
    if len(parts) != 4:
        raise ValueError("AEROTHON_DELIVERY_ZONE must be 'cx,cy,width,height'")
    try:
        cx, cy, width, height = (float(v) for v in parts)
    except ValueError:
        raise ValueError("AEROTHON_DELIVERY_ZONE values must be numbers")
    if not all(math.isfinite(v) for v in (cx, cy, width, height)):
        raise ValueError("AEROTHON_DELIVERY_ZONE values must be finite")
    if width <= 0.0 or height <= 0.0:
        raise ValueError("delivery-zone width and height must be positive")
    return cx, cy, width, height


def boundary_payload(cx, cy, width, height, home_lat, home_lon):
    """Four corners of the local ENU rectangle as the mission's boundary JSON.

    Corners are emitted in ring order (SW, SE, NE, NW) so the payload reads as
    a polygon rather than an unordered set.
    """
    half_w, half_h = width / 2.0, height / 2.0
    corners = ((cx - half_w, cy - half_h), (cx + half_w, cy - half_h),
               (cx + half_w, cy + half_h), (cx - half_w, cy + half_h))
    vertices = []
    for x, y in corners:
        lat, lon = local_to_global(x, y, home_lat, home_lon)
        vertices.append({"lat": lat, "lon": lon})
    return json.dumps({"vertices": vertices}, separators=(",", ":"))


def home_is_usable(home_lat, home_lon):
    """Reject the uninitialised MAVROS home instead of georeferencing about it.

    A home of exactly (0, 0) is MAVROS's default, not a position.  Publishing a
    boundary about null island hands the mission a rectangle in the Gulf of
    Guinea, which it accepts -- it is a valid rectangle -- and then searches.
    """
    if not (math.isfinite(home_lat) and math.isfinite(home_lon)):
        return False
    return not (abs(home_lat) < 1e-7 and abs(home_lon) < 1e-7)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from mavros_msgs.msg import HomePosition

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class DeliveryZoneSource(Node):
        def __init__(self):
            super().__init__("aerothon_delivery_zone_source")
            self.cx, self.cy, self.width, self.height = zone_from_env()
            self.pub = self.create_publisher(String, TOPIC, latched)
            self.fence = geofence_from_env()
            self.pub_fence = self.create_publisher(String, GEOFENCE_TOPIC,
                                                   latched)
            self.sent = False
            self.create_subscription(HomePosition,
                                     "/mavros/home_position/home",
                                     self._on_home, latched)
            self.get_logger().info(
                "waiting for FCU home to georeference the delivery zone "
                f"(centre {self.cx} {self.cy}, {self.width} x {self.height} m)")

        def _on_home(self, msg):
            if self.sent:
                return
            home_lat = float(msg.geo.latitude)
            home_lon = float(msg.geo.longitude)
            if not home_is_usable(home_lat, home_lon):
                self.get_logger().warn(
                    f"FCU home ({home_lat}, {home_lon}) is not a usable "
                    "position yet; still waiting")
                return
            self.pub.publish(String(data=boundary_payload(
                self.cx, self.cy, self.width, self.height, home_lat, home_lon)))
            self.pub_fence.publish(String(data=geofence_payload(
                self.fence, home_lat, home_lon)))
            verts = fence_vertices(self.fence)
            self.get_logger().info(
                "published arena geofence ({} vertices): x {:.1f}..{:.1f}, "
                "y {:.1f}..{:.1f} m".format(
                    len(verts), min(v[0] for v in verts), max(v[0] for v in verts),
                    min(v[1] for v in verts), max(v[1] for v in verts)))
            self.sent = True
            half_w, half_h = self.width / 2.0, self.height / 2.0
            self.get_logger().info(
                f"published delivery-zone boundary about home {home_lat:.7f} "
                f"{home_lon:.7f}: x {self.cx - half_w:.1f}..{self.cx + half_w:.1f}, "
                f"y {self.cy - half_h:.1f}..{self.cy + half_h:.1f} m")

    rclpy.init()
    node = DeliveryZoneSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
