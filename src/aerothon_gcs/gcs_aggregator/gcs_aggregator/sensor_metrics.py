"""Receipt rate and capture age for display; never flight decision inputs."""
import math
import struct
import time
from collections import deque


class SensorMetric:
    def __init__(self):
        self.receipts = deque(maxlen=120)
        self.capture = None

    def observe(self, header):
        self.receipts.append(time.monotonic())
        self.capture = header.stamp.sec + header.stamp.nanosec / 1e9

    def observe_raw(self, data):
        """As observe(), from a still-serialized message whose first field is
        a std_msgs/Header. Deserializing a 1280x720 Image only to read its
        stamp cost ~0.4 of a Pi 5 core per subscriber at 30 Hz (16 Oct)."""
        self.receipts.append(time.monotonic())
        stamp = cdr_header_stamp(data)
        if stamp is not None:
            self.capture = stamp[0] + stamp[1] / 1e9

    def snapshot(self, clock, now=None):
        now = time.monotonic() if now is None else now
        stamps = [t for t in self.receipts if now - t < 3.0]
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else None
        receipt_age = max(0., now - self.receipts[-1]) if self.receipts else None
        capture_age = clock - self.capture if clock is not None and self.capture is not None else None
        if capture_age is not None and (not math.isfinite(capture_age) or capture_age < 0 or self.capture <= 0):
            capture_age = None
        return dict(ok=receipt_age is not None and receipt_age < 3 and capture_age is not None and capture_age < 3,
                    hz=hz, receipt_age_s=receipt_age, capture_age_s=capture_age)


def cdr_header_stamp(data):
    """(sec, nanosec) of the Header at the start of a CDR-serialized message,
    or None. A ROS 2 serialized message starts with a 4-byte encapsulation
    (0x0001 little-endian, 0x0000 big-endian), then Header.stamp."""
    raw = bytes(data[:12])
    if len(raw) < 12 or raw[0] != 0:
        return None
    order = {1: "<", 0: ">"}.get(raw[1])
    if order is None:
        return None
    sec, nanosec = struct.unpack(order + "iI", raw[4:12])
    return (sec, nanosec) if nanosec < 1_000_000_000 else None
