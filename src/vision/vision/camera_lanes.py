"""Thread-safe latest-frame slots and weighted camera-lane scheduling."""

from __future__ import annotations

import threading
from collections import Counter
from typing import Any, Iterable


CAMERA_NAMES = frozenset({'drive', 'arm', 'left', 'right'})


class LatestFrameSlots:
    """Keep at most one newest item per camera and count overwrites."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Serializes lane cursors, rate reservations, and slot removal across
        # worker threads. Camera callbacks never take this lock.
        self._scheduler_lock = threading.RLock()
        self._slots: dict[str, Any] = {}
        self._overwrites: Counter[str] = Counter()

    def put(self, camera: str, item: Any) -> None:
        with self._lock:
            if camera in self._slots:
                self._overwrites[camera] += 1
            self._slots[camera] = item

    def has(self, camera: str) -> bool:
        with self._lock:
            return camera in self._slots

    def take(self, camera: str) -> Any | None:
        with self._lock:
            return self._slots.pop(camera, None)

    def take_overwrite_counts(self) -> dict[str, int]:
        """Return and reset overwrite counts accumulated since the last call."""
        with self._lock:
            counts = dict(self._overwrites)
            self._overwrites.clear()
            return counts


def parse_pattern(text: str, enabled: Iterable[str]) -> list[str]:
    """Parse a comma-separated pattern, validating before filtering cameras."""
    tokens = [token.strip().lower() for token in str(text).split(',') if token.strip()]
    unknown = set(tokens) - CAMERA_NAMES
    if unknown:
        raise ValueError(f'Unknown camera names in lane pattern: {sorted(unknown)}')
    enabled_set = set(enabled)
    return [camera for camera in tokens if camera in enabled_set]


def validate_lane_assignments(
    lane_patterns: dict[str, list[str]], enabled: Iterable[str]
) -> None:
    """Require exclusive lane ownership and coverage for every enabled camera."""
    owners: dict[str, str] = {}
    for lane_name, pattern in lane_patterns.items():
        for camera in set(pattern):
            previous = owners.get(camera)
            if previous is not None and previous != lane_name:
                raise ValueError(
                    f"Camera '{camera}' is assigned to both {previous} and {lane_name}."
                )
            owners[camera] = lane_name
    uncovered = set(enabled) - set(owners)
    if uncovered:
        raise ValueError(
            f'Active cameras missing from lane patterns: {sorted(uncovered)}'
        )


class Lane:
    """Weighted pattern cursor and per-camera rate reservations for one lane."""

    def __init__(self, name: str, pattern: list[str], max_fps: float = 0.0) -> None:
        if max_fps < 0.0:
            raise ValueError('lane max_fps must not be negative.')
        self.name = str(name)
        self.pattern = list(pattern)
        self.max_fps = float(max_fps)
        self.cursor = 0
        self._last_selected: dict[str, float] = {}

    def _rate_limited(self, camera: str, now: float) -> bool:
        if self.max_fps <= 0.0:
            return False
        previous = self._last_selected.get(camera)
        return previous is not None and now - previous < 1.0 / self.max_fps

    def _select_locked(
        self, slots: LatestFrameSlots, now: float
    ) -> tuple[str, Any] | None:
        if not self.pattern:
            return None
        for _ in range(len(self.pattern)):
            camera = self.pattern[self.cursor]
            self.cursor = (self.cursor + 1) % len(self.pattern)
            if self._rate_limited(camera, now):
                continue
            item = slots.take(camera)
            if item is None:
                continue
            # Reserve the rate-limit interval at selection time so another
            # worker cannot immediately steal a newer frame for this camera.
            self._last_selected[camera] = now
            return camera, item
        return None

    def select(
        self, slots: LatestFrameSlots, now: float
    ) -> tuple[str, Any] | None:
        """Select one job while protecting this lane from stealing workers."""
        with slots._scheduler_lock:
            return self._select_locked(slots, now)


def next_job(
    lane: Lane,
    lanes: list[Lane],
    slots: LatestFrameSlots,
    work_stealing: bool,
    now: float,
) -> tuple[str, Lane, Any] | None:
    """Select from the worker's lane, optionally advancing another lane."""
    with slots._scheduler_lock:
        selected = lane._select_locked(slots, now)
        if selected is not None:
            camera, item = selected
            return camera, lane, item
        if not work_stealing:
            return None
        for owner in lanes:
            if owner is lane:
                continue
            selected = owner._select_locked(slots, now)
            if selected is not None:
                camera, item = selected
                return camera, owner, item
    return None


__all__ = [
    'CAMERA_NAMES',
    'Lane',
    'LatestFrameSlots',
    'next_job',
    'parse_pattern',
    'validate_lane_assignments',
]
