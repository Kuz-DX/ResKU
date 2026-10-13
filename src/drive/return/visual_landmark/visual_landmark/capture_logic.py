"""Landmark capture sequencing -- pure logic, no rclpy (unit-testable).

'moving' 모드: 누른 뒤 도착한 첫 프레임을 바로 keyframe으로 (정지 안 함).
'stop' 모드 촬영 순서 (UI /path/landmark 버튼 한 번):
    WAIT_STILL  : odom |vx|, |wz|가 문턱 아래로 still_time_s 동안 유지될 때까지
    SETTLE      : 정지 직후 차체/마운트 잔진동이 가라앉을 settle_s 대기
                  (rolling shutter라 흔들리는 중에 찍으면 영상이 휜다)
    WAIT_FRAME  : SETTLE이 끝난 "이후에 도착한" 첫 프레임을 keyframe으로 사용
    -> 저장 후 IDLE
전체가 timeout_s 안에 안 끝나면 실패로 끝내고 hold를 푼다.
시간은 전부 float 초(호출자가 넘김).
"""
import math

IDLE = 'IDLE'
WAIT_STILL = 'WAIT_STILL'
SETTLE = 'SETTLE'
WAIT_FRAME = 'WAIT_FRAME'


class CaptureSequencer:
    def __init__(self, still_vx: float = 0.02, still_wz: float = 0.02,
                 still_time_s: float = 0.5, settle_s: float = 0.3, timeout_s: float = 5.0):
        self.still_vx = still_vx
        self.still_wz = still_wz
        self.still_time_s = still_time_s
        self.settle_s = settle_s
        self.timeout_s = timeout_s
        self.phase = IDLE
        self._start = None
        self._still_since = None
        self._settle_until = None
        self._frame_after = None
        self._wait_still = True

    @property
    def busy(self) -> bool:
        return self.phase != IDLE

    def start(self, now: float, wait_still: bool = True):
        """wait_still=False ('moving' 모드): 정지/정착 대기 없이 누른 뒤 첫 프레임을 쓴다."""
        self._start = now
        self._still_since = None
        self._settle_until = None
        if wait_still:
            self.phase = WAIT_STILL
            self._frame_after = None
        else:
            self.phase = WAIT_FRAME
            self._frame_after = now
        self._wait_still = wait_still

    def finish(self):
        self.phase = IDLE

    def on_odom(self, now: float, vx: float, wz: float):
        still = abs(vx) < self.still_vx and abs(wz) < self.still_wz
        if not still:
            self._still_since = None
            if self.phase in (SETTLE, WAIT_FRAME) and self._wait_still:
                # 정착 중에 다시 움직였다 -> 정지 확인부터 다시
                self.phase = WAIT_STILL
            return
        if self._still_since is None:
            self._still_since = now

    def tick(self, now: float):
        """Returns None while progressing, or a failure reason string."""
        if self.phase == IDLE:
            return None
        if now - self._start >= self.timeout_s:
            reason = f'timeout in {self.phase} after {self.timeout_s:.1f}s'
            self.phase = IDLE
            return reason
        if self.phase == WAIT_STILL and self._still_since is not None \
                and now - self._still_since >= self.still_time_s:
            self.phase = SETTLE
            self._settle_until = now + self.settle_s
        if self.phase == SETTLE and now >= self._settle_until:
            self.phase = WAIT_FRAME
            self._frame_after = now
        return None

    def accept_frame(self, arrival: float) -> bool:
        """True if a frame that arrived at `arrival` should become the keyframe."""
        return self.phase == WAIT_FRAME and arrival >= self._frame_after


class PressDetector:
    """drive_cmd_mux/capture_hold.py의 PressDetector와 같은 규칙 (두 노드가 같은
    눌림을 같은 방식으로 세야 하므로 동작을 바꾸면 양쪽 다 바꿀 것).
    False->True, 또는 True가 rearm_s 이상 끊겼다가 다시 오면 새 눌림."""

    def __init__(self, rearm_s: float = 0.5):
        self.rearm_s = rearm_s
        self._prev = False
        self._last_true = None

    def update(self, now: float, value: bool) -> bool:
        pressed = False
        if value:
            gap = None if self._last_true is None else now - self._last_true
            pressed = (not self._prev) or (gap is not None and gap >= self.rearm_s)
            self._last_true = now
        self._prev = value
        return pressed


class Odometer:
    """Distance travelled along the recorded route (mission frame), for s_along_path.

    /recorded_path의 point index는 점 병합(_process_path) 때문에 녹화 중에
    바뀔 수 있어서 landmark를 경로에 연결할 때 index 대신 누적 거리를 쓴다.
    max_step_m보다 큰 한 번의 점프(odom 재시작 등)는 거리에 넣지 않는다.
    """

    def __init__(self, max_step_m: float = 0.5):
        self.max_step_m = max_step_m
        self.reset()

    def reset(self):
        self.s = 0.0
        self._prev = None

    def update(self, x: float, y: float) -> float:
        if self._prev is not None:
            step = math.hypot(x - self._prev[0], y - self._prev[1])
            if step <= self.max_step_m:
                self.s += step
        self._prev = (x, y)
        return self.s
