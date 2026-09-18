#!/usr/bin/env bash
#
# record_manual_drive.sh
#
# record_autonomous_drive.sh와 같은 패턴(REQUIRED/OPTIONAL topic 확인 후
# 기록)의 manual+return 미션용 rosbag 스크립트.
#
# [manual+return 통합 이후 갱신] 예전엔 can_driver_node가 CAN 응답을 안 읽어서
# odom->base_link TF/odometry/filtered/wheel odom을 구조적으로 기록할 수
# 없었지만, 이제 rmd_x8_driver_node가 manual/return 공용 CAN 드라이버라 이
# 전부 정상 발행된다. 그래서 REQUIRED/OPTIONAL 토픽 목록을 새 아키텍처
# (/motor_speed_cmd_manual, /odometry/filtered, /recorded_path, /mission/return/state 등)
# 에 맞게 갱신했다.
#
# [2026 사용자 요청] "무조건 rosbag 기록이 시작되도록" -- 필수 토픽이 아직 안
# 보여도(노드가 조금 늦게 뜨는 중이거나, 원격 PC 조이스틱을 아직 안 켠 경우 등)
# 더 이상 여기서 exit 1로 기록을 막지 않는다. 경고만 찍고 그대로
# `ros2 bag record`를 시작한다 -- ros2 bag record 자체가 아직 없는 토픽은
# 나중에 생기면 그때부터 알아서 기록을 시작하는 방식으로 동작하므로, 미리
# 다 떠 있어야 할 필요가 없다.
#
# 전제: robot_bringup manual_return_bringup.launch.py가 먼저 떠 있어야 함(로봇
# PC), 그리고 원격 PC에서 manual_joy_control/manual_control.launch.py도 떠
# 있어야 /motor_speed_cmd_manual이 옴 (두 PC 모두 같은 ROS_DOMAIN_ID/
# ROS_LOCALHOST_ONLY=0 필요).
#
# 사용:
#   ./record_manual_drive.sh [output_dir]
#
# output_dir 기본값: $HOME/dolbotz_bags. 환경변수 DOLBOTZ_BAG_DIR로도 override 가능.
# bag 이름: manual_drive_YYYYMMDD_HHMMSS (자동, output_dir 하위에 생성)

set -uo pipefail

OUTPUT_DIR="${1:-${DOLBOTZ_BAG_DIR:-$HOME/dolbotz_bags}}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_DIR%/}/manual_drive_${STAMP}"

# 주행 카메라 스트림이 이미 떠 있는지 판단할 때 보는 대표 topic.
# base_link->imu_link/camera_link TF, odom->base_link TF는 이제
# manual_return_bringup.launch.py 자체가 채워주므로 여기서 더 이상 확인/대신
# 켜주지 않는다 (manual_drive_sensors.launch.py도 카메라만 남기고 축소됨).
CAMERA_CHECK_TOPIC="/drive/camera/color/image_raw/compressed"

SENSORS_LAUNCH_PID=""

cleanup() {
  if [ -n "$SENSORS_LAUNCH_PID" ] && kill -0 "$SENSORS_LAUNCH_PID" 2>/dev/null; then
    echo
    echo "-- manual_drive_sensors.launch.py(PID $SENSORS_LAUNCH_PID) 종료 중 --"
    kill "$SENSORS_LAUNCH_PID" 2>/dev/null || true
    wait "$SENSORS_LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

camera_up() {
  ros2 topic info "$CAMERA_CHECK_TOPIC" >/dev/null 2>&1
}

echo "== manual+return drive bag record =="
echo "output: ${BAG_PATH}"
echo

if camera_up; then
  echo "-- 주행 카메라 이미 떠 있음, 그대로 사용 --"
else
  echo "-- 주행 카메라가 안 보여서 manual_drive_sensors.launch.py를 대신 띄웁니다 --"
  ros2 launch robot_bringup manual_drive_sensors.launch.py >/tmp/manual_drive_sensors.log 2>&1 &
  SENSORS_LAUNCH_PID=$!

  echo "   기동 대기 중 (최대 15초)..."
  ok=0
  for _ in $(seq 1 15); do
    if camera_up; then
      ok=1
      break
    fi
    sleep 1
  done
  if [ "$ok" -ne 1 ]; then
    echo
    echo "카메라가 15초 안에 안 떴습니다 -- 로그 확인: /tmp/manual_drive_sensors.log"
    echo "(카메라가 이미 다른 프로세스에서 열려있거나, USB 연결 문제일 수 있음)"
    echo "그래도 기록은 계속 진행합니다 (무조건 시작 -- 카메라 없이라도 주행/복귀"
    echo "미션 토픽은 기록됨)."
  else
    echo "   카메라 확인됨 (PID $SENSORS_LAUNCH_PID)"
  fi
fi

# 메뉴얼+복귀 미션의 핵심 토픽. 하나라도 안 보여도 더 이상 기록을 막지 않는다
# (아래 경고만 찍고 계속 진행) -- ros2 bag record는 나중에 토픽이 생겨도 그때부터
# 알아서 기록하기 시작한다.
CORE_TOPICS=(
  /tf_static
  /tf
  /imu                     # myAHRS+, manual_return_bringup.launch.py가 항상 띄움
  /motor_speed_cmd_manual  # manual_joy_control_node (원격 PC 필요, dps)
  /cmd_vel_return          # return_state_machine_node
  /cmd_vel                 # drive_cmd_mux_node 최종 출력
  /wheel/odom              # rmd_x8_driver_node -- 이제 manual 주행 중에도 나옴
  /wheel/joint_states
  /wheel/motor_status
  /odometry/filtered       # reduced_odom_node -- 이제 manual 주행 중에도 나옴
  /mission/return/state
  /mission/return/trigger
  /mission/origin_pose
  /recorded_path_raw
  /recorded_path
  /return_path
)

echo
echo "-- 핵심 topic 확인 (안 보여도 기록은 진행) --"
missing=0
for t in "${CORE_TOPICS[@]}"; do
  if ros2 topic info "$t" >/dev/null 2>&1; then
    echo "  OK      $t"
  else
    echo "  MISSING $t"
    missing=1
  fi
done

if [ "$missing" -ne 0 ]; then
  echo
  echo "일부 핵심 topic이 아직 안 보입니다 (경고만, 기록은 계속 진행):"
  echo "  1) [로봇 PC]   ros2 launch robot_bringup manual_return_bringup.launch.py"
  echo "  2) [원격 PC]   ros2 launch manual_joy_control manual_control.launch.py"
  echo "  3) 두 PC가 같은 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY=0인지"
  echo "위 토픽들은 나중에 뜨면 ros2 bag record가 그때부터 알아서 기록합니다."
fi

echo
echo "-- 선택 topic 확인 (없어도 기록은 진행) --"
OPTIONAL_TOPICS=(
  /joy                                                       # 원격 PC joy_node 원본 입력
  /cmd_vel_return_path                                       # return_path_follower_node
  /drive/camera/imu                                          # RealSense D455 자체 IMU
  /drive/camera/color/image_raw/compressed
  /drive/camera/color/camera_info
  /drive/camera/aligned_depth_to_color/image_raw/compressedDepth
  /drive/camera/aligned_depth_to_color/camera_info
)
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
echo "topics: ${CORE_TOPICS[*]} ${record_optional[*]}"
echo "(Ctrl+C로 중지 -- 이 스크립트가 띄운 카메라도 같이 정리됩니다)"
echo

ros2 bag record -o "$BAG_PATH" "${CORE_TOPICS[@]}" "${record_optional[@]}"
