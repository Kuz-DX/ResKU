"""연속 프레임 상태 확정을 위한 ROS 비의존 유틸리티."""

from collections import deque


class ConsecutiveStateDebouncer:
    """같은 non-unknown 상태가 N회 연속 들어오면 그 상태를 확정한다."""

    def __init__(self, confirm_frames: int):
        if confirm_frames < 1:
            raise ValueError('confirm_frames must be at least 1')
        self._history: deque[str] = deque(maxlen=confirm_frames)
        self._confirm_frames = confirm_frames

    def update(self, raw_state: str) -> str | None:
        """상태 이력을 갱신하고 확정 상태 또는 None을 반환한다.

        unknown은 연속 판정이 끊긴 것으로 보고 이력을 즉시 초기화한다.
        """
        if raw_state == 'unknown':
            self._history.clear()
            return None

        self._history.append(raw_state)
        if len(self._history) < self._confirm_frames:
            return None
        first = self._history[0]
        if all(state == first for state in self._history):
            return first
        return None

    def reset(self) -> None:
        """이력을 비운다 — 외부에서 확정 상태를 강제로 풀 때(예: 락 해제 후
        재확정 대기로 되돌릴 때) 호출한다. update()가 unknown을 받았을 때와
        동일하게 이력만 지운다."""
        self._history.clear()
