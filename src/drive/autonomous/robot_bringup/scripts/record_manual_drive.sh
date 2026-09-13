#!/usr/bin/env bash
#
# record_manual_drive.sh
#
# record_autonomous_drive.sh와 같은 패턴(REQUIRED/OPTIONAL topic 확인 후
# 기록)의 메뉴얼 주행용 rosbag 스크립트.
#
# [2026-09-04 신규, 사용자 요청] "메뉴얼 주행할 때도 TF/카메라를 딸 수 있게,
# 안 켜져있으면 억지로 켜서라도" -- can_driver/manual.launch.py(로봇 PC 쪽
# 메뉴얼 주행)는 모터 구동/IMU 긴급정지만 신경 쓰고 TF나 카메라는 원래 안
# 띄운다. 이 스크립트는 그 둘이 안 보이면 manual_drive_sensors.launch.py를
# 대신 띄워서 채운 다음 기록을 시작한다.
#
# [중요, 구조적 한계] odom->base_link 동적 TF, /odometry/filtered, /wheel/odom
# 은 이 스크립트로도 채울 수 없다 -- can_driver_node(메뉴얼 주행 모터
# 드라이버)는 CAN 응답을 안 읽어서 애초에 바퀴 오도메트리를 발행하지 않고,
# 그걸 발행하는 rmd_x8_driver_node(reduced_odom_bringup.launch.py)는
# can_driver_node와 같은 CAN 채널/ID를 잡아서 동시에 띄우면 실물에서 충돌한다
# (manual_drive_sensors.launch.py 상단 docstring 참고). 그래서 이 셋은
# REQUIRED/OPTIONAL 어디에도 넣지 않았다 -- 자율주행 경로 재현이 필요하면
# record_autonomous_drive.sh를 쓸 것.
#
# 전제: can_driver/manual.launch.py가 먼저 떠 있어야 함(로봇 PC), 그리고
# 원격 PC에서 manual_joy_control/manual_control.launch.py도 떠 있어야
# /motor_speed_cmd가 옴(두 PC 모두 같은 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY=0
# 필요 -- can_driver/manual.launch.py 상단 docstring 참고).
#
# 사용:
#   ./record_manual_drive.sh [output_dir]
#
# output_dir 기본값: $HOME/dolbotz_bags. 환경변수 DOLBOTZ_BAG_DIR로도 override 가능.
# bag 이름: manual_drive_YYYYMMDD_HHMMSS (자동, output_dir 하위에 생성)

set -euo pipefail

OUTPUT_DIR="${1:-${DOLBOTZ_BAG_DIR:-$HOME/dolbotz_bags}}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_DIR%/}/manual_drive_${STAMP}"

# TF/카메라가 이미 떠 있는지 판단할 때 보는 대표 topic 하나씩.
SENSOR_CHECK_TOPICS=(
  /tf_static
  /drive/camera/color/image_raw/compressed
)

SENSORS_LAUNCH_PID=""

# 스크립트가 어떻게 끝나든(정상 종료/Ctrl+C/에러) 우리가 직접 띄운
# manual_drive_sensors.launch.py는 같이 정리한다 -- 안 그러면 카메라
# 프로세스가 백그라운드에 orphan으로 남는다.
cleanup() {
  if [ -n "$SENSORS_LAUNCH_PID" ] && kill -0 "$SENSORS_LAUNCH_PID" 2>/dev/null; then
    echo
    echo "-- manual_drive_sensors.launch.py(PID $SENSORS_LAUNCH_PID) 종료 중 --"
    kill "$SENSORS_LAUNCH_PID" 2>/dev/null || true
    wait "$SENSORS_LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sensors_up() {
  for t in "${SENSOR_CHECK_TOPICS[@]}"; do
    ros2 topic info "$t" >/dev/null 2>&1 || return 1
  done
  return 0
}

echo "== manual drive bag record =="
echo "output: ${BAG_PATH}"
echo

if sensors_up; then
  echo "-- TF/카메라 이미 떠 있음, 그대로 사용 --"
else
  echo "-- TF/카메라가 안 보여서 manual_drive_sensors.launch.py를 대신 띄웁니다 --"
  echo "   (can_driver_node와 CAN을 공유하지 않는 부분만 -- 상단 docstring 참고)"
  ros2 launch robot_bringup manual_drive_sensors.launch.py >/tmp/manual_drive_sensors.log 2>&1 &
  SENSORS_LAUNCH_PID=$!

  echo "   기동 대기 중 (최대 15초)..."
  ok=0
  for _ in $(seq 1 15); do
    if sensors_up; then
      ok=1
      break
    fi
    sleep 1
  done
  if [ "$ok" -ne 1 ]; then
    echo
    echo "TF/카메라가 15초 안에 안 떴습니다 -- 로그 확인: /tmp/manual_drive_sensors.log"
    echo "(카메라가 이미 다른 프로세스에서 열려있거나, USB 연결 문제일 수 있음)"
    exit 1
  fi
  echo "   TF/카메라 확인됨 (PID $SENSORS_LAUNCH_PID)"
fi

# 메뉴얼 주행 자체의 최소 조건 -- 하나라도 없으면 기록을 시작하지 않는다.
REQUIRED_TOPICS=(
  /tf_static
  /imu                   # myAHRS+, can_driver/manual.launch.py(enable_stability_monitor 기본 true)
  /motor_speed_cmd       # manual_joy_control_node -> can_driver_node(원격 PC 필요)
)

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
  echo "필수 topic이 안 보입니다. 아래를 확인하세요:"
  echo "  1) [로봇 PC]   ros2 launch can_driver manual.launch.py"
  echo "  2) [원격 PC]   ros2 launch manual_joy_control manual_control.launch.py"
  echo "  3) 두 PC가 같은 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY=0인지"
  echo "기록을 중단합니다."
  exit 1
fi

echo
echo "-- 선택 topic 확인 (없어도 기록은 진행) --"
OPTIONAL_TOPICS=(
  /motor_speed_cmd_safety                                   # manual_stability_node 긴급정지 override
  /joy                                                       # 원격 PC joy_node 원본 입력
  /tf                                                        # 보통 없음(구조적 한계, 상단 docstring 참고) -- 혹시 떠있으면 같이 기록
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
echo "topics: ${REQUIRED_TOPICS[*]} ${record_optional[*]}"
echo "(Ctrl+C로 중지 -- 이 스크립트가 띄운 카메라도 같이 정리됩니다)"
echo

ros2 bag record -o "$BAG_PATH" "${REQUIRED_TOPICS[@]}" "${record_optional[@]}"
