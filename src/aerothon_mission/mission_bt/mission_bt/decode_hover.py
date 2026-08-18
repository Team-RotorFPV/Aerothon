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

WHY IT IS AN OBJECT RATHER THAN A STAGE

    A hover has to happen INSIDE stages that are doing something else -- the
    sweep, the descent -- without interrupting their own state. A stage would
    have to be spliced into the tree at every point a marker might be read,
    and would then need to hand control back to whatever it interrupted.
"""

import time


class DecodeHover:
    """Own one per stage. Call tick() first; hold if it says to."""

    def __init__(self, hover_s=5.0, clock=None, enabled=True):
        self.hover_s = float(hover_s)
        self.clock = clock or time.monotonic
        self.enabled = bool(enabled)
        self.seen = set()
        self._t0 = None
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

    def forget(self):
        """Full reset, including history. For a new mission, not a new stage."""
        self.seen.clear()
        self.reset()

    @property
    def payload(self):
        return self._payload

    def tick(self, mav):
        """True if the caller should hold station this tick.

        Commands the hold itself, so a caller cannot accidentally hover and
        keep flying at the same time.
        """
        if not self.enabled:
            return False

        if self._t0 is not None:
            if self.clock() - self._t0 < self.hover_s:
                mav.goto(*self._hold)
                return True
            self.reset()
            return False

        payload = str(getattr(mav, "qr_decoded", "") or "")
        if not payload or payload in self.seen:
            return False

        self.seen.add(payload)
        self._payload = payload
        self._t0 = self.clock()
        x, y, z = mav.pos()
        self._hold = (x, y, z, mav.yaw())
        mav.goto(*self._hold)
        if hasattr(mav, "log"):
            mav.log(f"decoded '{payload}': holding {self.hover_s:.0f} s over it")
        return True
