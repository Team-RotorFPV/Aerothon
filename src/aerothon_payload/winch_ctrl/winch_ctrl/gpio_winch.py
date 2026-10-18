"""Pi GPIO winch: BTS7960 H-bridge drive and N20 quadrature encoder travel.

The selected winch is an N20 12 V gear motor with a two-channel encoder,
driven by a BTS7960 from the Pi (master guide section 6; wiring in
mission_bringup/config/hardware_wiring.yaml). This module is the backend
`winch_ctrl` uses when `backend:=gpio`:

  * DRIVE. Lowering drives RPWM, stowing LPWM, each at `drive_duty_pct`.
    The inactive direction is written to zero BEFORE the active one starts,
    and stop writes both PWMs to zero and drops both enables.
  * TRAVEL. Both encoder channels are decoded in quadrature (4 counts per
    encoder cycle). Line travel is counts x 2 pi r / counts_per_output_rev,
    signed by `lower_sign`. These are MEASURED values; with either missing
    the backend refuses to start rather than invent a payout.
  * STALL. Driven with no counts for `stall_timeout_s` stops the motor. Near
    the top of a stow (within `stow_window_m`) that is the hook against its
    stop: the stow reference, and the count is re-zeroed there. Anywhere
    else it is a fault: a jammed spool, a dead encoder or no motor power.
  * DIRECTION. Counting the wrong way while driven is a fault (swapped motor
    leads or encoder phases), never silently corrected.

Encoder travel is measured line movement only. It is not touchdown, payload
detachment or delivery; the delivery observer decides those.
"""

import math
import threading
import time

# (previous AB << 2 | current AB) -> step. Invalid double transitions are 0.
_QUADRATURE = (0, +1, -1, 0,
               -1, 0, 0, +1,
               +1, 0, 0, -1,
               0, -1, +1, 0)


class LgpioDriver:
    """The BTS7960 and encoder on Pi GPIO through lgpio (Pi 5 compatible)."""

    def __init__(self, chip, rpwm, lpwm, r_en, l_en, enc_a, enc_b, pwm_hz):
        import lgpio
        self.lg = lgpio
        self.pins = dict(rpwm=rpwm, lpwm=lpwm, r_en=r_en, l_en=l_en)
        self.enc = (enc_a, enc_b)
        self.pwm_hz = float(pwm_hz)
        self.h = lgpio.gpiochip_open(int(chip))
        for pin in self.pins.values():
            lgpio.gpio_claim_output(self.h, pin, 0)
        for pin in self.enc:
            lgpio.gpio_claim_alert(self.h, pin, lgpio.BOTH_EDGES)
        self._callbacks = []

    def on_edge(self, handler):
        # lgpio queues edge reports and hands them to Python in batches from a
        # thread. Re-reading the pins in the callback sees the batch's FINAL
        # state for every edge (counts lost); decode from each report's level.
        for pin in self.enc:
            self._callbacks.append(self.lg.callback(
                self.h, pin, self.lg.BOTH_EDGES,
                lambda chip, gpio, level, tick: handler(gpio, level)))

    def read_ab(self):
        a, b = (self.lg.gpio_read(self.h, pin) for pin in self.enc)
        return (a << 1) | b

    def drive(self, direction, duty_pct):
        p, lg = self.pins, self.lg
        if direction == 0:
            lg.tx_pwm(self.h, p["rpwm"], self.pwm_hz, 0)
            lg.tx_pwm(self.h, p["lpwm"], self.pwm_hz, 0)
            lg.gpio_write(self.h, p["r_en"], 0)
            lg.gpio_write(self.h, p["l_en"], 0)
            return
        # Inactive side to zero first; never both PWMs high.
        lg.tx_pwm(self.h, p["lpwm" if direction > 0 else "rpwm"], self.pwm_hz, 0)
        lg.gpio_write(self.h, p["r_en"], 1)
        lg.gpio_write(self.h, p["l_en"], 1)
        lg.tx_pwm(self.h, p["rpwm" if direction > 0 else "lpwm"], self.pwm_hz,
                  float(duty_pct))

    def close(self):
        try:
            self.drive(0, 0)
        finally:
            for cb in self._callbacks:
                cb.cancel()
            self.lg.gpiochip_close(self.h)


class EncoderWinch:
    """Drive state, measured travel and stall/direction faults."""

    def __init__(self, driver, counts_per_output_rev, spool_radius_m,
                 lower_sign=1, drive_duty_pct=60.0, stall_timeout_s=0.6,
                 stow_window_m=0.15, reverse_fault_counts=8, clock=time.monotonic):
        counts_per_output_rev = float(counts_per_output_rev or 0.0)
        spool_radius_m = float(spool_radius_m or 0.0)
        if not (math.isfinite(counts_per_output_rev) and counts_per_output_rev > 0):
            raise ValueError("encoder counts per output revolution not measured")
        if not (math.isfinite(spool_radius_m) and spool_radius_m > 0):
            raise ValueError("spool effective radius not measured")
        if int(lower_sign) not in (-1, 1):
            raise ValueError("encoder lower_sign must be +1 or -1")
        if not 0 < float(drive_duty_pct) <= 100:
            raise ValueError("drive duty must be within 0-100 %")
        self.driver = driver
        self.m_per_count = 2 * math.pi * spool_radius_m / counts_per_output_rev
        self.lower_sign = int(lower_sign)
        self.duty = float(drive_duty_pct)
        self.stall_timeout_s = float(stall_timeout_s)
        self.stow_window_m = float(stow_window_m)
        self.reverse_fault_counts = int(reverse_fault_counts)
        self.clock = clock
        self._lock = threading.Lock()
        self._count = 0
        self._invalid = 0
        self._ab = driver.read_ab()
        self.direction = 0              # +1 lowering, -1 stowing, 0 stopped
        self.fault = ""
        self._moved_t = None
        self._start_count = 0
        driver.drive(0, 0)
        driver.on_edge(self._edge)

    # -- encoder ------------------------------------------------------- #
    def _edge(self, gpio=None, level=None):
        enc = getattr(self.driver, "enc", None)
        if gpio is not None and enc and level in (0, 1) and gpio in enc:
            # A is the high bit, B the low bit (read_ab's layout).
            shift = 1 if gpio == enc[0] else 0
            with self._lock:
                ab = (self._ab & ~(1 << shift)) | (int(level) << shift)
        elif gpio is not None and level not in (0, 1):
            return              # lgpio watchdog report (level 2): no edge
        else:
            ab = self.driver.read_ab()
        with self._lock:
            step = _QUADRATURE[(self._ab << 2) | ab]
            if step == 0 and ab != self._ab:
                self._invalid += 1
            self._count += step
            self._ab = ab
            self._moved_t = self.clock()

    @property
    def counts(self):
        with self._lock:
            return self._count

    @property
    def invalid_transitions(self):
        with self._lock:
            return self._invalid

    def payout_m(self):
        return self.lower_sign * self.counts * self.m_per_count

    # -- drive --------------------------------------------------------- #
    def command(self, action):
        """`lower`, `stow` or `stop`. A latched fault allows only stop."""
        direction = {"lower": 1, "stow": -1, "stop": 0}[action]
        if self.fault and direction:
            return False
        self.direction = direction
        self._start_count = self.counts
        self._moved_t = self.clock()
        self.driver.drive(direction, self.duty if direction else 0)
        return True

    def _halt(self, reason=""):
        self.direction = 0
        self.driver.drive(0, 0)
        if reason:
            self.fault = reason

    def update(self):
        """Check a driven spool for stall or reversal. Returns an event or None."""
        if not self.direction:
            return None
        moved = (self.counts - self._start_count) * self.lower_sign
        if moved * self.direction <= -self.reverse_fault_counts:
            self._halt("encoder counts opposite to the drive; check motor "
                       "leads, encoder phase order and lower_sign")
            return "fault"
        if self.clock() - self._moved_t < self.stall_timeout_s:
            return None
        if self.direction < 0 and self.payout_m() <= self.stow_window_m:
            # Hook against its stop: the stow reference.
            self._halt()
            with self._lock:
                self._count = 0
            return "stowed"
        self._halt("no encoder motion while driven: jammed spool, missing "
                   "encoder signal or no motor power")
        return "fault"

    def stop(self):
        self._halt()

    def close(self):
        self._halt()
        close = getattr(self.driver, "close", None)
        if close:
            close()
