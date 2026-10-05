#!/usr/bin/env python3
"""Hold station for a few seconds over each DISTINCT decoded marker.

WHY

    The mission read a marker and moved on within the same tick. Watched
    live, a decode was invisible: the payload appeared in a log line and the
    aircraft was already elsewhere. The operator asked for a hover "on the qr
    for 5 seconds even after scanning it", and for it on every marker decoded,
    not only the one that matches.

WHY PER DISTINCT PAYLOAD RATHER THAN PER DECODE

    A marker held in frame decodes on every frame. Hovering per decode would
    hold the aircraft over the first pad it saw until the mission timed out --
    15 marks are scored on finishing inside 15 minutes. Hovering once per
    payload gives the operator the pause on every genuinely new marker and
    costs five seconds each.

UNREADABLE MARKERS

    A marker the camera can SEE but not READ -- motion blur from the sweep,
    rolling-shutter jello from the props, a smeared lens -- is reported by
    qr_node with offset z = UNREAD. Measured under the camera corruptions of
    sim_gazebo/corruptions.py, the decoders lose a marker to motion blur and
    vibration while its finder patterns still locate it, and a stationary
    camera removes the motion blur. So such a marker gets one hold, over
    where it is on the ground (`locate`), for up to `unread_s`: the sweep
    stops, the frames stop smearing, and the hold ends the moment it reads.
    Once per `unread_cell_m` of ground, so a marker that will never read (not
    a competition marker, or genuinely illegible) costs one hold, not the
    mission.

WHY IT IS AN OBJECT RATHER THAN A STAGE

    A hover has to happen INSIDE stages that are doing something else -- the
    sweep, the descent -- without interrupting their own state. A stage would
    have to be spliced into the tree at every point a marker might be read,
    and would then need to hand control back to whatever it interrupted.
"""

import time

# qr_node's offset z for a marker located but not decoded.
UNREAD = 0.25


class DecodeHover:
    """Own one per stage. Call tick() first; hold if it says to."""

    def __init__(self, hover_s=5.0, clock=None, enabled=True, unread_s=0.0,
                 locate=None, unread_cell_m=3.0):
        self.hover_s = float(hover_s)
        self.clock = clock or time.monotonic
        self.enabled = bool(enabled)
        self.unread_s = float(unread_s)
        self.locate = locate
        self.unread_cell_m = float(unread_cell_m)
        self.seen = set()
        self.seen_unread = set()
        self._mission = None
        self._t0 = None
        self._span = 0.0
        self._payload = ""
        self._hold = None

    def reset(self):
        """Forget the current hover, but NOT which payloads have been seen.

        Deliberate: re-entering the sweep after a descent must not hover again
        on the marker that caused the descent.
        """
        self._t0 = None
        self._payload = ""
        self._hold = None

    def begin(self, mission):
        """A stage (re)starting: `reset` within the same mission, `forget`
        when `mission` -- Mav.mission_seq -- says it is a new one."""
        if mission != self._mission:
            self._mission = mission
            self.forget()
        else:
            self.reset()

    def forget(self):
        """Full reset, including history. For a new mission, not a new stage."""
        self.seen.clear()
        self.seen_unread.clear()
        self.reset()

    @property
    def payload(self):
        return self._payload

    def tick(self, mav):
        """True if the caller should hold station this tick.

        Commands the hold itself, so a caller cannot accidentally hover and
        keep flying at the same time. `enabled` governs the hover over a
        decoded payload; the hold over an unreadable marker is governed by
        `unread_s` alone, because it is how a marker gets read at all.
        """
        if self._t0 is not None:
            reading = self._payload == "" and self._new_payload(mav)
            if not reading and self.clock() - self._t0 < self._span:
                mav.goto(*self._hold)
                return True
            # Time up -- or the unreadable marker being held over just read,
            # which starts that payload's own hover below.
            self.reset()
            if not reading:
                return False

        payload = self._new_payload(mav) if self.enabled else ""
        if payload:
            self.seen.add(payload)
            x, y, z = mav.pos()
            self._start(payload, (x, y, z, mav.yaw()), self.hover_s)
            mav.goto(*self._hold)
            if hasattr(mav, "log"):
                mav.log(f"decoded '{payload}': holding {self.hover_s:.0f} s over it")
            return True

        spot = self._unread_spot(mav)
        if spot is None:
            return False
        self.seen_unread.add(self._cell(spot))
        _, _, z = mav.pos()
        self._start("", (spot[0], spot[1], z, mav.yaw()), self.unread_s)
        mav.goto(*self._hold)
        if hasattr(mav, "log"):
            mav.log(f"marker at ({spot[0]:.1f}, {spot[1]:.1f}) seen but not "
                    f"read: holding up to {self.unread_s:.0f} s over it")
        return True

    def _start(self, payload, hold, span):
        self._payload = payload
        self._hold = hold
        self._span = float(span)
        self._t0 = self.clock()

    def _new_payload(self, mav):
        payload = str(getattr(mav, "qr_decoded", "") or "")
        return payload if payload and payload not in self.seen else ""

    def _cell(self, xy):
        c = self.unread_cell_m
        return (round(xy[0] / c), round(xy[1] / c))

    def _unread_spot(self, mav):
        """Ground (x, y) of an unreadable marker not yet held over, or None."""
        if self.unread_s <= 0.0 or self.locate is None:
            return None
        off = getattr(mav, "qr_offset", None)
        if off is None or not 0.0 < float(getattr(off, "z", 0.0)) <= UNREAD:
            return None
        spot = self.locate(mav)
        if spot is None or self._cell(spot) in self.seen_unread:
            return None
        return spot
