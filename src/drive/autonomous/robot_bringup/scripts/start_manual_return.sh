#!/usr/bin/env bash
#
# start_manual_return.sh  (로봇 PC 전용)
#
# 이전 실행에서 남은 manual+return 미션 프로세스를 먼저 정리한 뒤
# manual_return_bringup.launch.py를 실행한다.
#
# 왜 필요한가: 이전 launch가 Ctrl+C로 깔끔하게 안 끝나서 rmd_x8_driver_node 같은
# 노드가 남아 있으면, 새 실행과 같은 CAN 버스(모터 명령/응답)와 IMU 시리얼 포트를
# 나눠 쓰게 된다. 남은 드라이버가 0 속도 명령을 계속 보내면서 새 명령과 섞이면
# 로봇이 안 움직이거나, 모터 통신두절 보호 설정(comm_timeout_protection) 확인이
# 실패할 수 있다.
#
# 사용:
#   bash start_manual_return.sh [launch 인자...]     정리 후 실행
#   bash start_manual_return.sh --cleanup-only        정리만 하고 종료
#
#   예) bash start_manual_return.sh turn_direction:=right
#
# 종료 방식: 먼저 SIGINT(정상 종료 -- 드라이버가 모터에 정지 명령을 보냄)를 보내고
# 최대 5초 기다린 뒤, 그래도 남은 것만 SIGKILL 한다.
#
# 주의: 이 스크립트는 "이 미션의 노드"만 골라서 종료한다(경로에 lib/<패키지>/<실행파일>
# 이 들어간 프로세스와 robot_bringup launch 프로세스). 같은 이름의 노드를 쓰는
# 자율주행(autonomous.launch.py)이나 가상 테스트(manual_return_sim)도 같이 종료되며,
# 이 미션은 그것들과 CAN을 같이 쓸 수 없으므로 의도한 동작이다.

set -u

# 경로 전체(lib/<pkg>/<exe>)로 매칭해서 우연히 파일명만 같은 편집기/로그 뷰어 등은
# 건드리지 않는다.
PATTERNS=(
  "bin/ros2 launch robot_bringup manual_return_bringup"
  "bin/ros2 launch robot_bringup autonomous"
  "lib/rmd_x8_driver/rmd_x8_driver_node"
  "lib/myahrs_driver/myahrs_driver_node"
  "lib/reduced_odom/reduced_odom_node"
  "lib/drive_cmd_mux/drive_cmd_mux_node"
  "lib/return_navigation/manual_path_recorder_node"
  "lib/return_navigation/return_state_machine_node"
  "lib/return_navigation/return_path_follower_node"
)

find_leftovers() {
  local pat pid
  for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
      # 자기 자신과 부모 셸은 절대 대상에서 제외
      if [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ]; then
        echo "$pid"
      fi
    done
  done | sort -un
}

cleanup() {
  local pids
  pids=$(find_leftovers)
  if [ -z "$pids" ]; then
    echo "[cleanup] 남은 프로세스 없음"
    return 0
  fi

  echo "[cleanup] 남은 프로세스를 종료합니다:"
  # shellcheck disable=SC2086
  ps -o pid=,etime=,args= -p $(echo $pids | tr ' ' ',') | cut -c1-150 | sed 's/^/    /'

  # shellcheck disable=SC2086
  kill -INT $pids 2>/dev/null

  local i
  for i in 1 2 3 4 5 6 7 8 9 10; do
    sleep 0.5
    pids=$(find_leftovers)
    [ -z "$pids" ] && break
  done

  if [ -n "$pids" ]; then
    echo "[cleanup] 5초 안에 안 끝난 프로세스를 강제 종료(SIGKILL): $(echo $pids | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill -KILL $pids 2>/dev/null
    sleep 1
    pids=$(find_leftovers)
  fi

  if [ -n "$pids" ]; then
    echo "[cleanup] 실패: 아직 남아 있습니다 -> $(echo $pids | tr '\n' ' ')" >&2
    return 1
  fi

  # CAN 소켓/IMU 시리얼 포트가 완전히 풀릴 시간을 준다
  sleep 1
  echo "[cleanup] 완료"
  return 0
}

cleanup || exit 1

if [ "${1:-}" = "--cleanup-only" ]; then
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"

# ROS setup 스크립트는 정의 안 된 변수를 참조하므로 set -u를 잠시 끈다
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u

echo "[start] ros2 launch robot_bringup manual_return_bringup.launch.py $*"
exec ros2 launch robot_bringup manual_return_bringup.launch.py "$@"
