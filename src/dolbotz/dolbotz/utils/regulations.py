"""
대회 규정집에 명시된 물리 치수 상수 — ROS 의존성 없음.

지금은 어느 노드/로직에도 아직 연결 안 됐다(기록만 해두는 단계). 잠재적
활용처는 이 모듈이 아니라 각 노드 쪽 검토가 필요하다 — 예를 들어
flat_drive.py의 bev_mask_to_centerline_path()/slope_decision.py의 좌우
ROI 분리 로직에 이 값을 "세그멘테이션 마스크가 그럴듯한 폭으로 잡혔는지"
검증하는 용도로 연결할 수 있다.
"""

# 규정집 기준 트랙 폭 [m] — 914.4mm(36in). 로봇 자체 폭(516mm,
# slope_decision.py의 track_width_m)과는 다른 값이니 혼동 주의.
TRACK_WIDTH_M = 0.9144
