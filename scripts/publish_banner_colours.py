#!/usr/bin/env python3
"""Supply the banner colours the detectors refuse to fly without.

The organisers give each banner's shade as a HEX code before the competition,
and the banners carry no text, so colour is what tells the aircraft which
corridor it may use. `gcs_readiness` reports `banner_colours` unknown until
they arrive, `/mission_ready` stays false, and the aircraft will not arm.

This node is that organiser. It is a SIMULATION INPUT, not mission logic: on
a real flight the shades come from the organisers through the GCS field and
this node is not launched. The defaults are the colours the simulator itself
prints, so a normal simulated run is not blocked waiting for a human.

    AEROTHON_BANNER_COLOURS='{"green":"#07A03D","orange":"#EB5A0A"}'
    AEROTHON_BANNER_COLOURS='green=#07A03D,orange=#EB5A0A,red=#C4161C'

Set AEROTHON_SUPPLY_BANNER_COLOURS=0 to drive them from the GCS instead.
"""

import json
import os
import sys

# The shades the simulator prints today: PRINT_GREEN's diffuse, the return
# lane's orange, and the restricted zone's red. Writing them as HEX here is
# what makes the whole path -- operator value to detector mask -- the same
# path in simulation as on the day.
DEFAULTS = {"green": "#07A03D", "orange": "#EB5A0A", "red": "#C4161C"}
TOPIC = "/mission/banner_colours"


def supplied(text):
    """Parse the env override: JSON object, or name=#RRGGBB comma list."""
    text = (text or "").strip()
    if not text:
        return dict(DEFAULTS)
    if text.startswith("{"):
        return json.loads(text)
    out = {}
    for part in text.split(","):
        if not part.strip():
            continue
        name, _, value = part.partition("=")
        if not value:
            raise ValueError(f"{part!r} is not name=#RRGGBB")
        out[name.strip().lower()] = value.strip()
    return out


def main():
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src/aerothon_perception/perception_banner"))
    from perception_banner.colours import validate

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String

    try:
        colours = validate(supplied(os.environ.get("AEROTHON_BANNER_COLOURS")))
    except (ValueError, TypeError, json.JSONDecodeError) as error:
        raise SystemExit(f"AEROTHON_BANNER_COLOURS refused: {error}")

    rclpy.init()
    node = Node("banner_colour_source")
    # Latched, so every detector that comes up later still receives them.
    publisher = node.create_publisher(
        String, TOPIC,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(String(data=json.dumps({"colours": colours},
                                             separators=(",", ":"))))
    node.get_logger().info(
        "published banner colours: "
        + ", ".join(f"{k}={v}" for k, v in sorted(colours.items())))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
