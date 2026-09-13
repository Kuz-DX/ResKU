#!/usr/bin/env bash
#
# record_reduced_odom_validation.sh
#
# reduced_odom(5-state [x,y,yaw,vx,wz] EKF, production odometry -- src/drive/
# autonomous/reduced_odom) 평지 실차 검증용 rosbag 기록 스크립트.
#
# 전제: reduced_odom_bringup.launch.py(또는 autonomous.launch.py -- rmd_x8_driver_node +
# myahrs_driver_node + reduced_odom_node를 전부 포함하는 쪽)가 이미 떠 있어야
# 함. manual 조종 스택(can_driver + manual_joy_control, /motor_speed_cmd)은
# rmd_x8_driver_node/reduced_odom_node를 아예 안 띄우므로 이 검증에 쓸 수
# 없다 -- todrive.md "미션 구조상... 두 모드는 완전히 독립된 노드 구현" 참고.
# 컨트롤된 시험 주행(직선/회전/사각형)을 위해 MPPI/인지 파이프라인 없이
# /cmd_vel(geometry_msgs/Twist)을 직접 퍼블리시하고 싶다면 예:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# (설치 안 돼 있으면 `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`로 대체)
#
# 사용:
#   ./record_reduced_odom_validation.sh [output_dir]
#
# output_dir 기본값: $HOME/dolbotz_bags (repo(source directory) 밖 -- 큰 bag을
# src/ 트리 안에 두지 않기 위함). 환경변수 DOLBOTZ_BAG_DIR로도 override 가능.
#
# bag 이름: reduced_odom_validation_YYYYMMDD_HHMMSS (자동, output_dir 하위에 생성)

set -euo pipefail

OUTPUT_DIR="${1:-${DOLBOTZ_BAG_DIR:-$HOME/dolbotz_bags}}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_DIR%/}/reduced_odom_validation_${STAMP}"

# 반드시 있어야 하는 topic (reduced_odom_node 검증의 최소 조건, section 5).
REQUIRED_TOPICS=(
  /wheel/odom
  /imu
  /odometry/filtered
  /odometry/diagnostics
  /tf
  /tf_static
)

# 있으면 같이 기록(코드 확인 결과 rmd_x8_driver_node가 실제로 publish함 --
# left/right wheel raw velocity(joint_states.velocity), motor 진단, 실제 모터
# 입력 command). 없어도 기록은 진행하되 경고만 남긴다.
OPTIONAL_TOPICS=(
  /wheel/joint_states   # left_wheel_joint/right_wheel_joint velocity(rad/s), effort=전류(A)
  /wheel/motor_status   # 모터 통신/에러 상태 (DiagnosticArray)
  /cmd_vel              # rmd_x8_driver_node가 실제로 받는 최종 구동 명령
  /cmd_vel_safety       # stability_monitor_node 긴급정지 (평상시 무발행)
)

# 확인했지만 이 로봇에서 record 대상이 아닌 것 (section 2/11 D 참고):
#   - raw gyro (angular_velocity) / raw magnetometer: myahrs_driver_node.cpp가
#     "$RPY,..." fused orientation만 파싱/발행하고, angular_velocity_covariance[0]
#     =-1 / linear_acceleration_covariance[0]=-1로 "데이터 없음"을 명시함.
#     별도 topic으로도 발행 안 함 -- 이 드라이버로는 raw gyro/mag를 bag에
#     담을 수 없다(디바이스 자체에 magnetometer가 없다는 뜻은 아님, ROS로
#     노출을 안 할 뿐).

echo "== reduced_odom validation bag record =="
echo "output: ${BAG_PATH}"
echo

echo "-- 필수 topic 확인 --"
missing=0
for t in "${REQUIRED_TOPICS[@]}"; do
  if ros2 topic info "$t" >/dev/null 2>&1; then
    echo "  OK   $t"
  else
    echo "  MISSING $t"
    missing=1
  fi
done

if [ "$missing" -ne 0 ]; then
  echo
  echo "필수 topic이 안 보입니다. reduced_odom_bringup.launch.py(또는 autonomous.launch.py)가"
  echo "떠 있는지 먼저 확인하세요:"
  echo "  ros2 launch robot_bringup reduced_odom_bringup.launch.py"
  echo "기록을 중단합니다."
  exit 1
fi

echo
echo "-- 선택 topic 확인 (없어도 기록은 진행) --"
record_optional=()
for t in "${OPTIONAL_TOPICS[@]}"; do
  if ros2 topic info "$t" >/dev/null 2>&1; then
    echo "  OK   $t"
    record_optional+=("$t")
  else
    echo "  없음 $t (건너뜀)"
  fi
done

mkdir -p "$OUTPUT_DIR"

echo
echo "-- 기록 시작: ${BAG_PATH} --"
echo "topics: ${REQUIRED_TOPICS[*]} ${record_optional[*]}"
echo "(Ctrl+C로 중지)"
echo

exec ros2 bag record -o "$BAG_PATH" "${REQUIRED_TOPICS[@]}" "${record_optional[@]}"
