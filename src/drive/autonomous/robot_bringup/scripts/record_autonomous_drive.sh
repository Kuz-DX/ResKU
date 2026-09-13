#!/usr/bin/env bash
#
# record_autonomous_drive.sh
#
# 자율주행 스택(TF/odom/경로추종/경사대응) 전체를 재현/분석 가능하게
# 기록하는 rosbag 스크립트. reduced_odom/scripts/record_reduced_odom_validation.sh
# 와 동일한 패턴(REQUIRED/OPTIONAL topic 확인 후 기록)을 따른다.
#
# [2026-08-30 신규, 사용자 요청] "봄구간에서 바로 자율주행에 필요한 rosbag"
# -- 특정 미션 전용이 아니라, path_relay_node/slope_traverse_node(또는
# slope_traverse_blend_node)/reduced_odom_node를 쓰는 자율주행 launch라면
# 어느 미션(봄/겨울 등)에서 실행하든 동일하게 유효하다. TF, odom, /path를
# 특히 놓치지 않도록 전부 REQUIRED_TOPICS에 넣었다. [2026-09-02, 사용자
# 요청] 카메라 color/depth 스트림도 OPTIONAL_TOPICS에 추가해 bag 용량이
# 커지더라도 같이 기록하도록 확장.
#
# 전제: 아래 두 가지가 먼저 떠 있어야 함:
#   1) 자율주행 launch (하나만) -- 예:
#        ros2 launch robot_bringup autonomous.launch.py         (PlanA, 일반 트랙)
#        ros2 launch robot_bringup autonomous_blend.launch.py   (PlanB, TF 미사용 pure pursuit)
#        ros2 launch robot_bringup mission_winter_drive.launch.py
#      (reduced_odom_bringup.launch.py를 내부에서 포함하므로 /imu, /odometry/filtered,
#      TF 등은 이걸로 이미 뜸)
#   2) 인지팀 쪽 perception 파이프라인 -- /path, /terrain/slope_side_signal,
#      /drive/camera/imu 등은 이게 따로 떠 있어야 발행됨(dolbotz 패키지,
#      이 repo 기준 정확한 launch 파일명은 인지팀 확인 필요).
#
# [2026-09-02, 사용자 요청] 이미지/뎁스 스트림(카메라 color/depth)도 이제
# OPTIONAL_TOPICS에 포함해서 같이 담는다 -- bag 용량이 커지는 건 감수.
# 토픽 이름은 실제 구독 코드 기준(압축 전송, slope_decision.py/flat_drive.py
# 참고) -- 인지 파이프라인이 raw(비압축)로 바꾸면 여기도 맞춰서 바꿀 것.
#
# 사용:
#   ./record_autonomous_drive.sh [output_dir]
#
# output_dir 기본값: $HOME/dolbotz_bags (repo 밖 -- 큰 bag을 src/ 트리 안에
# 두지 않기 위함). 환경변수 DOLBOTZ_BAG_DIR로도 override 가능.
#
# bag 이름: autonomous_drive_YYYYMMDD_HHMMSS (자동, output_dir 하위에 생성)

set -euo pipefail

OUTPUT_DIR="${1:-${DOLBOTZ_BAG_DIR:-$HOME/dolbotz_bags}}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_DIR%/}/autonomous_drive_${STAMP}"

# 반드시 있어야 하는 topic -- TF/odom/path 재현에 필요한 최소 조건.
# 하나라도 없으면 기록을 시작하지 않는다(사용자 요청: "TF, odom이랑
# /path를 발행하는 것들 모두 다 딸 수 있게").
REQUIRED_TOPICS=(
  /tf
  /tf_static
  /odometry/filtered   # reduced_odom_node 출력(EKF), odom->base_link TF도 여기서 나옴
  /wheel/odom           # rmd_x8_driver_node 원본 휠 오도메트리(reduced_odom_node 입력)
  /odometry/diagnostics
  /imu                  # myAHRS+
  /path                 # 인지팀 slope_decision.py 최종 경로 -- perception 파이프라인 필요
)

# 있으면 같이 기록(자율주행 launch/perception 파이프라인 구성에 따라 없을
# 수 있음 -- 없어도 경고만 남기고 기록은 진행).
OPTIONAL_TOPICS=(
  /drive/camera/imu              # RealSense D455 자체 IMU (perception 파이프라인 필요)
  # [2026-09-02 신규] 카메라 color/depth 스트림 -- bag 용량 커지는 것
  # 감수하고 전부 기록(사용자 요청). 압축 전송 토픽(실제 구독 코드 기준).
  /drive/camera/color/image_raw/compressed
  /drive/camera/color/camera_info
  /drive/camera/aligned_depth_to_color/image_raw/compressedDepth
  /drive/camera/aligned_depth_to_color/camera_info
  /terrain/slope_side_signal     # 인지팀 signed left/right 경사 신호
  /terrain/side_slope_angle_deg  # 경사각 모니터링용(연속값)
  /drive/status                  # slope_decision.py 상태 로그
  /cmd_vel_auto                  # controller_server(MPPI) 출력
  /cmd_vel_safety                # slope_traverse_node/stability_monitor_node 최우선 override
  /cmd_vel                       # rmd_x8_driver_node가 실제로 받는 최종 구동 명령
  /wheel/joint_states             # left/right wheel velocity(rad/s), effort=전류(A)
  /wheel/motor_status             # 모터 통신/에러 상태(DiagnosticArray)
  /slope_traverse/debug           # slope_traverse_node(PlanA/B 공통 토픽명) 디버그
  /drive/slope_traverse_state     # slope_traverse_node 상태 문자열
)

echo "== autonomous drive bag record =="
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
  echo "필수 topic이 안 보입니다. 아래 둘 다 먼저 떠 있는지 확인하세요:"
  echo "  1) 자율주행 launch, 예: ros2 launch robot_bringup autonomous.launch.py"
  echo "  2) 인지팀 perception 파이프라인 (/path, /terrain/... 발행 -- dolbotz 패키지)"
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
