"""카메라 등 센서 스트림 전용 QoS 공용 상수.

[2026-08-31] 팀원 공유 — ROS2 공식 권장(SensorDataQoS)대로 카메라류는
BEST_EFFORT(재전송 없이 최신 샘플 우선)를 쓰되, 기본 큐 깊이(rclpy의
qos_profile_sensor_data는 depth=5)보다 더 작게(depth=1) 잡는 게 낫다는
피드백을 반영한다. depth=5면 구독 측 처리가 잠깐 밀렸을 때 오래된 프레임
5개를 순서대로 처리하며 실시간에서 계속 뒤처지는데, depth=1이면 항상
"지금 막 온 가장 최신 프레임"만 남고 그 전 건 버려져서 뒤처짐이 누적되지
않는다 — spring_ifof.py의 _submit()이 처리 중이면 새 프레임을 아예 드롭하는
것과 같은 취지(연산 이미지 노드들의 "최신 프레임 우선" 설계와 일관됨).

reliability/durability는 qos_profile_sensor_data와 동일(BEST_EFFORT/
VOLATILE)하게 유지 — DDS QoS 호환 규칙상 "구독자가 요구하는 수준 <=
발행자가 제공하는 수준"이면 매칭되므로, BEST_EFFORT 구독자는 RELIABLE
발행자(realsense2_camera, usb_cam 등 기본 RELIABLE 발행)와도 문제없이
연결된다(이 리포가 이미 qos_profile_sensor_data로 그렇게 써왔음) — 반대로
RELIABLE 구독자가 BEST_EFFORT 발행자를 구독하려는 조합이 비호환 사례다.
depth(큐 크기)는 이 매칭 규칙과 무관한 순수 로컬 버퍼 설정이라, reliability/
durability는 건드리지 않고 depth만 바꿨다.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# 카메라 이미지/CameraInfo/IMU 등 고빈도 센서 구독에 공용으로 쓴다 --
# qos_profile_sensor_data(rclpy 기본, depth=5)와 동일하되 depth만 1로.
SENSOR_DATA_QOS_DEPTH1 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
