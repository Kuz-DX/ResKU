"""Landmark-capture hold: decides whether manual driving must be forced to zero.

[2026-10-13 신규] UI의 /path/landmark 버튼(rising edge)을 누르면 즉시 정지해서
visual_landmark_node가 후방 카메라 keyframe을 찍을 수 있게 한다. 정지는
mux가 직접 건다 -- 촬영 노드가 죽어 있어도 버튼을 누르면 바로 멈추도록.

해제 조건 (어느 하나라도):
  - visual_landmark_node가 /visual_landmark/hold=False(촬영 끝)를 보냄
  - 촬영 노드 응답 없음: 버튼 후 grace_s 동안 hold=True가 한 번도 안 오거나,
    마지막 hold=True 이후 lease_s가 지남 (노드가 죽은 경우)
  - 버튼 후 max_hold_s 경과 (어떤 경우에도 조이스틱이 영영 막히지 않게)

시간은 전부 float 초(호출자가 넘김) -- rclpy 없이 단위 테스트 가능.
"""


class PressDetector:
    """Bool 버튼 토픽의 '새 눌림' 판정.

    False->True 변화, 또는 True가 rearm_s 이상 끊겼다가 다시 오는 경우를 눌림으로
    본다. UI/조이스틱이 True만 보내고 False를 빼먹어도 다음 누름이 동작하게 하려는
    것 -- landmark는 같은 상태(MANUAL_RECORDING) 안에서 여러 번 누르므로
    return_state_machine처럼 "상태 재진입 시 초기화"로 해결할 수 없다.
    """

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


class CaptureHold:
    def __init__(self, grace_s: float = 1.0, lease_s: float = 0.5, max_hold_s: float = 6.0):
        self.grace_s = grace_s
        self.lease_s = lease_s
        self.max_hold_s = max_hold_s
        self._start = None
        self._last_lease = None

    def press(self, now: float):
        """UI 버튼 rising edge. 이미 hold 중이면 무시(타이머 연장 안 함)."""
        if self.active(now):
            return
        self._start = now
        self._last_lease = None

    def lease(self, now: float, holding: bool):
        """visual_landmark_node의 /visual_landmark/hold 수신."""
        if self._start is None:
            return
        if holding:
            self._last_lease = now
        else:
            self.release()

    def release(self):
        self._start = None
        self._last_lease = None

    def active(self, now: float) -> bool:
        if self._start is None:
            return False
        elapsed = now - self._start
        if elapsed >= self.max_hold_s:
            self.release()
            return False
        if self._last_lease is None:
            alive = elapsed < self.grace_s
        else:
            alive = now - self._last_lease < self.lease_s
        if not alive:
            self.release()
        return alive
