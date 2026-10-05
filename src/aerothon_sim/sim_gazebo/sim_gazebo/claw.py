"""The dropping mechanism's scissor claw: how far it opens for a given slack.

The claw is a pair of ice tongs (scripts/cad_to_gazebo.py, MECH_MOVING). The
line holds the TOP PIN; a link runs from it to the upper end of each JAW; the
jaws cross on a CENTRE PIN and their curled tips hold the payload's eyelet.

    loaded   the line pulls the top pin up, the links pull the jaws' upper
             ends in, the tips close under the eyelet
    slack    the payload rests, the top pin comes down toward the centre pin,
             the links push the upper ends out and the tips spread apart

It is a closed linkage, which Gazebo's physics cannot hold, so the model
carries it as a tree of driven joints (build_cad_vehicle.py) and this module
works out one consistent set of joint positions for an opening angle. Pins run
along y; everything moves in the x-z plane. Joint angles are about +y, which
turns +x toward -z: the opposite sense to counter-clockwise in the x-z plane.
"""

import math


def _rot(p, c, a):
    """p turned counter-clockwise (x-z plane) by a about c."""
    dx, dz = p[0] - c[0], p[1] - c[1]
    return (c[0] + dx * math.cos(a) - dz * math.sin(a),
            c[1] + dx * math.sin(a) + dz * math.cos(a))


def _ang(v):
    return math.atan2(v[1], v[0])


class Claw:
    def __init__(self, geo, open_max_deg=35.0, release_deg=15.0):
        xz = lambda p: (float(p[0]), float(p[2]))          # noqa: E731
        self.T0, self.P0 = xz(geo["top_pin"]), xz(geo["centre_pin"])
        self.A0, self.B0 = xz(geo["pin_a"]), xz(geo["pin_b"])
        self.La = math.dist(self.T0, self.A0)
        self.open_max = math.radians(open_max_deg)
        # The tips have spread past the eyelet's wire.
        self.release = math.radians(release_deg)

    def pose(self, phi):
        """Joint positions with the jaws opened by phi (rad, 0 = as drawn).

        drop     how far the top pin has come down toward the centre pin: the
                 line's travel while the claw rests and opens
        pivot    the centre pin's travel up the hanger (prismatic, +z), = drop
        jaw_a/b, link_a/b   revolute, about +y
        """
        A = _rot(self.A0, self.P0, +phi)       # jaw a's upper pin swings out
        B = _rot(self.B0, self.P0, -phi)       # jaw b's, the mirror image
        dx = self.T0[0] - A[0]
        T = (self.T0[0], A[1] + math.sqrt(max(self.La ** 2 - dx * dx, 0.0)))
        drop = self.T0[1] - T[1]
        la = _ang((A[0] - T[0], A[1] - T[1])) - _ang((self.A0[0] - self.T0[0],
                                                       self.A0[1] - self.T0[1]))
        lb = _ang((B[0] - T[0], B[1] - T[1])) - _ang((self.B0[0] - self.T0[0],
                                                       self.B0[1] - self.T0[1]))
        return {"drop": drop, "pivot": drop, "jaw_a": -phi, "jaw_b": phi,
                "link_a": -la, "link_b": -lb}

    @property
    def drop_max(self):
        return self.pose(self.open_max)["drop"]

    def phi_for_drop(self, drop):
        """The opening at which the top pin has come down by `drop`."""
        if drop <= 0.0:
            return 0.0
        if drop >= self.drop_max:
            return self.open_max
        lo, hi = 0.0, self.open_max
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.pose(mid)["drop"] < drop:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)
