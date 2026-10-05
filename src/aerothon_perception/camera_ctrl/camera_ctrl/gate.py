"""Is the camera pointed where a detector has anything to see?

Every detector used to run on every frame, whatever the camera was looking
at: the banner detector over the nadir view of the grass all through the
delivery search (58 ms a frame on the test host, about twice that on the Pi 5),
the QR reader into the sky ahead through the corridor. On a Pi 5 that is most
of a core spent measuring nothing.

A detector asks `open()` before its work. It follows camera_ctrl's own
/camera/pose_state: open while the REQUESTED pose is one it serves, and open
whenever that state is missing or stale -- a detector must never go blind
because the camera controller did.
"""

import json

from std_msgs.msg import String


class CameraGate:
    def __init__(self, node, poses, stale_s=2.0):
        self.node = node
        self.poses = frozenset(poses)
        self.stale_s = float(stale_s)
        self._requested = None
        self._t = None
        node.create_subscription(String, "/camera/pose_state", self._on_state, 10)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, m):
        try:
            self._requested = json.loads(m.data).get("requested") or None
        except (ValueError, TypeError, AttributeError):
            return
        self._t = self._now()

    def open(self):
        if self._requested is None or self._t is None \
                or self._now() - self._t > self.stale_s:
            return True
        return self._requested in self.poses
