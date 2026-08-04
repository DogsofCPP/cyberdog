"""
Utility helpers for the CyberDog race system.
"""
import math
import time
from collections import deque


class RateKeeper:
    """Simple rate limiter: yields at approximately the requested Hz."""

    def __init__(self, hz=30):
        self._dt = 1.0 / hz
        self._last = time.perf_counter()

    def sleep(self):
        elapsed = time.perf_counter() - self._last
        if elapsed < self._dt:
            time.sleep(self._dt - elapsed)
        self._last = time.perf_counter()


class MovingAverage:
    """Thread-safe moving average over a fixed window."""

    def __init__(self, window=5):
        self._window = window
        self._buf = deque(maxlen=window)

    def update(self, value):
        self._buf.append(value)

    @property
    def value(self):
        if not self._buf:
            return 0.0
        return sum(self._buf) / len(self._buf)

    def reset(self):
        self._buf.clear()


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value, lo, hi):
    """Clamp value to [lo, hi]."""
    return max(lo, min(hi, value))


def deg_to_rad(deg):
    return deg * math.pi / 180.0


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


class Timer:
    """Context manager / simple timer."""

    def __init__(self, name=""):
        self.name = name
        self._start = None

    def __enter__(self):
        self._start = time.perf_counter()
        return self

    def __exit__(self, *args):
        elapsed = time.perf_counter() - self._start
        print(f"[Timer{(' ' + self.name) if self.name else ''}] "
              f"{elapsed:.3f}s")

    @property
    def elapsed(self):
        if self._start is None:
            return 0.0
        return time.perf_counter() - self._start
