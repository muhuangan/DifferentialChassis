"""Pure Python speed mapping and the fixed 12-byte UART packet."""

import math
import struct

FRAME = struct.Struct('<2sii2s')
HEADER = b'\xAA\x55'
TAIL = b'\x0D\x0A'


def pack_speeds(left_rpm, right_rpm):
    """Signed int32 rpm, little-endian. Raises on values outside int32."""
    return FRAME.pack(HEADER, left_rpm, right_rpm, TAIL)


def map_speeds(linear, angular, kv, kw, rpm_limit, left_direction, right_direction):
    """Approximate mapping; no calibrated wheel diameter or track width."""
    left = kv * linear - kw * angular
    right = kv * linear + kw * angular
    if not all(math.isfinite(v) for v in (linear, angular, left, right)):
        raise ValueError('Non-finite velocity command')
    left = round(max(-rpm_limit, min(rpm_limit, left))) * left_direction
    right = round(max(-rpm_limit, min(rpm_limit, right))) * right_direction
    return int(left), int(right)


class CommandState:
    """Keep only the newest wheel command; stale commands become zero."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.speeds = (0, 0)
        self.received_at = None

    def update(self, speeds, now):
        self.speeds = speeds
        self.received_at = now

    def current(self, now):
        if self.received_at is None or now - self.received_at >= self.timeout:
            return (0, 0)
        return self.speeds
