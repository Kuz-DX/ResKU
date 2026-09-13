#!/usr/bin/env python3
"""
test_frame_chain.py

direct_odom_node의 quaternion/frame chain(q_world_base = q_world_imu *
inverse(q_base_imu))을 재검증하는 독립 스크립트. ROS 노드를 띄우지 않고
direct_odom.direct_odom_node의 순수 함수만 가져다 쓴다 -- colcon test로
자동 실행되진 않고(이 패키지 CMakeLists에 pytest 등록 안 함, 최소 개선
범위 밖), `python3 test_frame_chain.py`로 직접 실행.

실행:
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # 또는 이 파일 fallback으로 src 경로 사용
    python3 src/drive/autonomous/direct_odom/test/test_frame_chain.py

검증 두 가지 층위 (섞으면 순환 논증이 되므로 분리):

  (A) "순수 회전 수학이 맞는가" -- roll-only/pitch-only/yaw-only/복합
      케이스에서 setRPY와 같은 공식(직접 구현, tf2 안 불러옴)으로
      만든 quaternion을 rotate_vector에 넣고, 손으로 유도한 회전행렬
      기댓값(R = Rz(yaw)*Ry(pitch)*Rx(roll), intrinsic ZYX)과 비교한다.
      mount offset compose 로직은 아예 안 쓴다 -- 순수 rotate_vector/quat_mul
      프리미티브만 검증.

  (B) "mount offset 보정 체인이 맞는가" -- 실제 reduced_odom_bringup.launch.py 마운트
      오프셋(roll=0.0555, pitch=0.0134, yaw=0)을 q_base_imu로 놓고, (A)의
      5개 케이스를 "진짜 base_link 자세"라고 가정해서 IMU가 봤을 법한
      값을 합성(q_world_imu = q_true_base * q_base_imu)한 다음,
      direct_odom_node의 실제 프로덕션 함수로 역산해서 q_true_base를
      되찾는지 검증한다. 대수적으로는 q ⊗ q_off ⊗ q_off^-1 = q라 항상
      성립하는 self-consistency 테스트지만(구현 버그 -- 곱셈 순서/conjugate
      방향 실수를 잡아냄), "관례 자체가 맞는가"는 (A)와 tf2 message 문서
      (TransformStamped.msg: "rotation of child_frame_id from
      header.frame_id")로 별도 확인한다 -- direct_odom_node.py 상단
      docstring 참고.
"""
import math
import os
import sys

try:
    from direct_odom.direct_odom_node import (
        quat_mul, quat_conjugate, quat_normalize, rotate_vector, quat_angle_deg,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from direct_odom.direct_odom_node import (
        quat_mul, quat_conjugate, quat_normalize, rotate_vector, quat_angle_deg,
    )


def rpy_to_quat(roll, pitch, yaw):
    """tf2::Quaternion::setRPY와 동일한 표준 intrinsic-ZYX 공식.
    myahrs_driver_node.cpp가 실제로 이 공식(tf2 구현)을 쓰므로, 테스트
    합성 IMU 메시지도 같은 공식으로 만들어야 비교가 의미 있다."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def expected_forward_vector(roll, pitch, yaw):
    """R(q)*[1,0,0]을 R = Rz(yaw)*Ry(pitch)*Rx(roll)로 손으로 전개한 닫힌 형태.
    Rx(roll)은 x축 벡터의 고유벡터라 [1,0,0]에 대해 항등(roll은 forward
    방향에 영향 없음) -- 그래서 아래엔 roll이 안 나온다(의도적, case 1/4에서
    이 사실 자체를 확인)."""
    return (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        -math.sin(pitch),
    )


CASES = [
    ('roll only',    0.3, 0.0, 0.0),
    ('pitch only',   0.0, 0.2, 0.0),
    ('yaw only',     0.0, 0.0, 1.2),
    ('roll+pitch',   0.3, 0.2, 0.0),
    ('yaw+pitch',    0.0, 0.2, 1.2),
]

# reduced_odom_bringup.launch.py의 base_to_imu_tf 실측값 (하드코딩 아님 -- 실제 배포된 값을
# 그대로 가져온 것. 이 값이 바뀌면 이 테스트도 같이 갱신해야 함).
MOUNT_ROLL = 0.0555
MOUNT_PITCH = 0.0134
MOUNT_YAW = 0.0

TOL_DEG = 1e-6
TOL_VEC = 1e-9


def vec_close(a, b, tol):
    return all(abs(ai - bi) < tol for ai, bi in zip(a, b))


def main():
    failures = 0

    print('=== (A) 순수 회전 수학 검증 (mount offset 없음) ===')
    print(f'{"case":14s} {"expected fwd":>28s} {"computed fwd":>28s}  ok?')
    for name, roll, pitch, yaw in CASES:
        q = quat_normalize(rpy_to_quat(roll, pitch, yaw))
        computed = rotate_vector(q, (1.0, 0.0, 0.0))
        expected = expected_forward_vector(roll, pitch, yaw)
        ok = vec_close(computed, expected, TOL_VEC)
        failures += 0 if ok else 1
        print(f'{name:14s} {str(tuple(round(v, 6) for v in expected)):>28s} '
              f'{str(tuple(round(v, 6) for v in computed)):>28s}  {"OK" if ok else "FAIL"}')

    print()
    print('=== (B) mount-offset 보정 체인 round-trip (실제 reduced_odom_bringup.launch.py 오프셋 사용) ===')
    q_base_imu = quat_normalize(rpy_to_quat(MOUNT_ROLL, MOUNT_PITCH, MOUNT_YAW))
    print(f'{"case":14s} {"true fwd":>28s} {"recovered fwd":>28s} {"angle err(deg)":>15s}  ok?')
    for name, roll, pitch, yaw in CASES:
        q_true_base = quat_normalize(rpy_to_quat(roll, pitch, yaw))
        # IMU가 봤을 값 = "진짜 base 자세"에 마운트 오프셋을 얹은 것.
        q_world_imu_synth = quat_normalize(quat_mul(q_true_base, q_base_imu))
        # 프로덕션 코드와 동일한 식.
        q_recovered = quat_normalize(
            quat_mul(q_world_imu_synth, quat_conjugate(q_base_imu)))

        angle_err = quat_angle_deg(q_recovered, q_true_base)
        true_fwd = rotate_vector(q_true_base, (1.0, 0.0, 0.0))
        recovered_fwd = rotate_vector(q_recovered, (1.0, 0.0, 0.0))
        ok = angle_err < TOL_DEG and vec_close(true_fwd, recovered_fwd, TOL_VEC)
        failures += 0 if ok else 1
        print(f'{name:14s} {str(tuple(round(v, 6) for v in true_fwd)):>28s} '
              f'{str(tuple(round(v, 6) for v in recovered_fwd)):>28s} '
              f'{angle_err:15.9f}  {"OK" if ok else "FAIL"}')

    print()
    print('=== (C) 평지 sanity: 로봇이 실제로 수평이면 IMU는 마운트 tilt 자체를 절대값으로 보고해야 함 ===')
    # 로봇이 완전 평지(true base 자세=identity)일 때, IMU 자체는 물리적으로
    # 마운트만큼 기울어져 있으므로 orientation으로 그 tilt를 그대로 보고한다.
    # (이걸 반대로 "IMU가 flat을 보고한다"고 가정하면 마운트 보정이 검증되지
    # 않음 -- 실제로 이전 세션에서 이 실수를 했다가 정정한 케이스.)
    q_true_base = (0.0, 0.0, 0.0, 1.0)
    q_world_imu_flat_robot = q_base_imu  # 로봇이 평지 -> IMU reading == 마운트 오프셋 그 자체
    q_recovered = quat_normalize(
        quat_mul(q_world_imu_flat_robot, quat_conjugate(q_base_imu)))
    angle_err = quat_angle_deg(q_recovered, q_true_base)
    ok = angle_err < TOL_DEG
    failures += 0 if ok else 1
    print(f'recovered q_world_base = {tuple(round(v, 9) for v in q_recovered)} '
          f'(expect identity (0,0,0,1)), angle_err={angle_err:.9f}deg  {"OK" if ok else "FAIL"}')

    print()
    if failures:
        print(f'{failures} case(s) FAILED')
        sys.exit(1)
    else:
        print('all cases OK')


if __name__ == '__main__':
    main()
