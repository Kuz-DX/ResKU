"""Thread-safe one-slot handoff; processing remains serialized by the ROS group."""
from threading import Lock


class LatestPair:
    def __init__(self):
        self._lock = Lock()
        self._pair = None
        self._stamp = None
        self.out_of_order = 0
        self.received = 0
        self.overwritten = 0

    def put(self, color, depth, received_at, stamp_ns=None):
        with self._lock:
            self.received += 1
            if (self._pair is not None and stamp_ns is not None and self._stamp is not None
                    and stamp_ns <= self._stamp):
                self.out_of_order += 1
                return
            self._stamp = stamp_ns
            self.overwritten += int(self._pair is not None)
            self._pair = (color, depth, received_at)

    def take(self):
        with self._lock:
            pair, self._pair = self._pair, None
            return pair
