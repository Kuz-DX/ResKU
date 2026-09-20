# Manual + Return 미션 실행 명령어

manual 주행 중 경로를 기록하고, RETURN 트리거 후 자동으로 출발지까지
복귀하는 미션의 실행 명령어 모음. 아키텍처/토픽 상세는 `topic.md`,
`howtorun.md`, `LOCAL_ODOM_RETURN_PATH.md` 참고.

**[업데이트] Corner-aware Pure Pursuit + Rotate-To-Heading** —
`return_path_follower_node`는 경로 진행방향이 `corner_detection_angle_deg`
(기본 45°) 이상 바뀌는 지점을 **임시 목표(코너 꼭짓점)**로 둔다.
꼭짓점까지 곡선 없이 직진하며 감속하고, 도착(`corner_reach_distance_m`
기본 0.08m 이내)하면 **정지 → 제자리 회전 → 다음 코너/목표로 재출발**한다
(기존엔 꼭짓점 앞에서 먼저 곡선으로 꺾은 뒤 또 회전해서 좌우로 흔들렸음).
내부적으로 `TRACK_PATH`/`ROTATE_TO_PATH` 두 상태를 오가며, 시작 자세가
경로와 `rotate_enter_angle_deg`(42°) 이상 어긋나 있으면 그 경우도 회전
후 출발한다. 실행 명령어 자체는 그대로다 — 내부 파라미터만 추가됐다.
recorder는 제자리 회전 중 같은 위치에 겹쳐 찍힌 점(`min_point_spacing_m`
기본 0.1m 이내)을 하나로 합쳐서 경로 방향이 튀지 않게 한다.

**[실차 튜닝]** 실차 로그에서 스키드 조향 제자리 회전이 명령의 약 25~40%만
나오는 것이 확인되어(예: `TURN_180`이 35초), 회전 게인/속도 한도를 올렸다:
`turn_kp` 2.0, `turn_w_max_radps` 0.9, `turn_yaw_tolerance_rad` 0.087(5°),
`rotate_kp` 2.0, `rotate_max_angular_speed_radps` 0.9,
`rotate_min_angular_speed_radps` 0.15, 복귀 직진 속도 `linear_speed_mps` 0.2. 또한 RETURN 직전 180° 정렬 회전은
방향이 우연에 맡겨져 있던 것을 **고정**했다(기본 `left`=반시계):

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py turn_direction:=right   # 시계 방향
```

벽이 있는 쪽으로 돌지 않게 상황에 맞춰 고른다.

## 1. 로봇 PC — CAN 인터페이스 켜기

```bash
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
ip -details link show can_drive
```

## 2. 로봇 PC — manual+return 통합 미션 실행

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py
```

`rmd_x8_driver`(CAN 유일 소유, `/wheel/odom`) + `myahrs_driver`(`/imu`) +
`reduced_odom`(`/odometry/filtered`) + `drive_cmd_mux` +
`manual_path_recorder` + `return_state_machine` + `return_path_follower`가
한 번에 뜬다.


## 3. 원격 PC — 조이스틱

```bash
ros2 launch manual_joy_control manual_control.launch.py
```


## 4. rosbag 기록 (선택, 로봇 PC)

```bash
bash src/drive/autonomous/robot_bringup/scripts/record_manual_drive.sh
```

## 5. 확인용 명령어

```bash
# 오도메트리 (manual 주행 중에도 나와야 정상)
ros2 topic echo /wheel/odom
ros2 topic echo /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_link

# 조종/최종 명령
ros2 topic echo /motor_speed_cmd_manual   # 조이스틱 출력 (dps)
ros2 topic echo /cmd_vel                  # mux 최종 출력 (Twist) -> rmd_x8_driver

# 복귀 미션 상태
ros2 topic echo /mission/return/state     # IDLE/MANUAL_RECORDING/.../FINISHED
ros2 topic echo /recorded_path            # 누적 기록 경로
ros2 topic echo /return_path              # RETURN 트리거 후 1회 발행되는 복귀 경로

# 코너 회전 전환 확인 -- return_path_follower_node를 띄운 터미널 콘솔에서
# 아래 로그 줄이 코너마다 한 쌍씩(진입/복귀) 찍히는지 확인
#   "TRACK_PATH -> ROTATE_TO_PATH (heading_error=...)"
#   "ROTATE_TO_PATH -> TRACK_PATH (yaw_error=...)"
```

## 6. RETURN 트리거

조이스틱 버튼 8번(기본값, 미검증 — 실기에서 확인 필요). 수동으로
트리거하려면:

```bash
ros2 topic pub -1 /mission/return/trigger std_msgs/msg/Bool "{data: true}"
```

## 7. 실차 전 가상환경 테스트 (RViz + 키보드 조종)

실제 CAN/IMU/조이스틱 없이, `rmd_x8_driver`/`reduced_odom`/`drive_cmd_mux`/
`return_navigation` 전체를 **실제 코드 그대로** 가상 CAN(`vcan0`) 위에서
구동하고 RViz로 지켜볼 수 있다. 모터/IMU만 `manual_return_sim`의 가짜
하드웨어로 대체된다. **터미널 2개**가 필요하다 (시뮬레이션 / 키보드).

### 7-1. vcan0 준비 (최초 1회, 재부팅/세션 초기화 시 다시 필요)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0        # UP 이면 정상
```

### 7-2. [터미널 1] 시뮬레이션 + RViz 실행

코드를 수정했다면 먼저 빌드한다:

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-select return_navigation manual_return_sim
```

실행 (RViz가 자동으로 같이 뜬다):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manual_return_sim sim_manual_return.launch.py
```

`rmd_x8_driver_node`(vcan0) + `sim_rmd_x8_hardware`(가짜 모터+IMU) +
`reduced_odom_node` + `drive_cmd_mux` + `manual_path_recorder` +
`return_state_machine` + `return_path_follower` + `sim_debug_viz`(경로
누적 시각화용 `odom->mission` TF) + RViz가 한 번에 뜬다. RViz에는
초록(기록 경로)/주황(계획 복귀 경로)/하늘색(실제 복귀 궤적)이 사이클마다
누적되어 겹쳐 보인다.

**RViz만 닫았다가 다시 띄울 때** (시뮬레이션은 그대로 두고):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d install/manual_return_sim/share/manual_return_sim/config/sim_manual_return.rviz
```

### 7-2-1. 초기화하고 다시 켜기 (경로 기록/RViz 화면 리셋)

RViz의 누적 경로(초록/주황/하늘색)와 기록된 경로는 시뮬레이션 노드가
들고 있으므로, **RViz만 다시 켜서는 초기화되지 않는다.** 시뮬레이션 전체를
껐다가 다시 켜야 한다.

**1) 종료** — 터미널 1에서 `Ctrl+C`. 그래도 노드/RViz가 남아있을 수
있으니(고아 프로세스가 남으면 다음 실행과 겹쳐서 RViz가 2개 뜨거나 노드가
중복된다) 아래 명령으로 정리한다. 대괄호(`[r]viz2` 등)는 이 명령이 자기
자신을 죽이지 않게 하는 것이니 그대로 복사해서 쓴다:

```bash
pkill -f "[r]os2 launch manual_return_sim|[r]viz2|[r]md_x8_driver_node|[r]educed_odom_node|[d]rive_cmd_mux_node|[m]anual_path_recorder_node|[r]eturn_state_machine_node|[r]eturn_path_follower_node|[s]im_rmd_x8_hardware|[s]im_debug_viz|[s]tatic_transform_publisher"
sleep 3
# 0 이 나오면 깨끗하게 정리된 것
pgrep -f '[r]viz2 -d|[_]node --ros-args|[s]im_rmd_x8_hardware|[s]im_debug_viz|[s]tatic_transform_publisher' | wc -l
```

(`keyboard_teleop`은 종료되지 않는다. 시뮬레이션이 재시작되면 상태줄이
멈출 수 있으니 그 터미널에서 `x`로 끝내고 다시 실행한다.)

**2) 다시 켜기** — 터미널 1:

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manual_return_sim sim_manual_return.launch.py
```

`vcan0`은 위 정리로 사라지지 않으니 다시 만들 필요 없다(재부팅했다면
7-1부터). 코드를 고쳤다면 실행 전에 7-2의 `colcon build`를 먼저 한다.

### 7-3. [터미널 2] 키보드로 조종

**별도 터미널**(실제 TTY 필요)에서:

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run manual_return_sim keyboard_teleop
```

| 키 | 동작 |
|---|---|
| `w` / `s` | 전진 / 후진 |
| `a` / `d` | 제자리 좌/우회전 |
| `q` / `e` | 전진 좌/우 커브 |
| `z` / `c` | 후진 좌/우 커브 |
| `space` | 정지 |
| `+` / `-` | 속도 ±10 dps |
| `r` | 정지 후 RETURN 트리거 |
| `x` | 종료 |

키를 누르면 다른 키를 누르거나 `space`를 누를 때까지 유지된다. 화면
아래 상태줄에 현재 미션 상태와 좌/우 dps가 실시간으로 표시된다.
직각/지그재그 경로를 그려본 뒤 `r`로 복귀시켜 코너에서 정지 후
제자리 회전하는지 RViz로 확인한다.

### 7-4. 코너 회전 동작 확인 (선택)

```bash
# 코너마다 진입/복귀 로그가 한 쌍씩 나와야 정상 (코너 수와 비슷한 횟수)
#   TRACK_PATH -> ROTATE_TO_PATH (heading_error=...)
#   ROTATE_TO_PATH -> TRACK_PATH (yaw_error=...)
# 코너마다 "reached corner N" 로그가 한 번씩만 나와야 정상 (같은 코너 반복 = 이상).
ros2 topic echo /cmd_vel_return_path      # 회전 중 linear.x 가 0 인지 확인
```

## 7-5. 실차 모니터링 (로컬 PC에서 RViz + 초 단위 yaw)

로봇 PC에서는 `manual_return_bringup`만 켜고, **로컬 PC**에서 아래 두 개를 띄우면
로봇 토픽(`/odometry/filtered`, `/mission/*`, `/return_path`)을 그대로 받아 그린다
(두 PC의 `ROS_DOMAIN_ID`가 같아야 함). 코드 수정 후에는 로컬 PC에서
`colcon build --packages-select manual_return_sim` 먼저.

```bash
# 터미널 A: 경로/궤적/yaw 마커 발행 + odom->mission TF (debug 전용)
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run manual_return_sim sim_debug_viz

# 터미널 B: RViz
rviz2 -d install/manual_return_sim/share/manual_return_sim/config/sim_manual_return.rviz
```

TURN_180 / FOLLOW_RETURN_PATH 동안 **1초마다** 실제 궤적 위에 노란 점과
글자가 찍히고, 같은 내용이 터미널 A에 표로 출력된다:

```
  t(s)  state                 yaw(deg) path(deg)  err(deg)   xte(m)     v(m/s)  w(rad/s)
     3  FOLLOW_RETURN_PATH         30.4      11.0     -19.4     +0.05       0.20     +0.10
```

- `yaw`: 로봇 현재 yaw (mission 프레임), `path`: 가장 가까운 계획 구간의 진행방향
- `err` = `path - yaw` (양수면 경로 방향이 로봇 기준 왼쪽), `xte`: 경로까지의 거리
  (양수 = 로봇이 경로 왼쪽), `v`/`w`: follower가 낸 명령
- 로봇 옆의 하늘색 글자는 실시간 상태/yaw/명령(0.2초 갱신)
- 라벨은 사이클마다 누적된다(최대 600개). 지우려면 터미널 A를 재시작.

## 8. 동시 실행 금지

`autonomous.launch.py`(MPPI, 2단계 평가용 보존)와
`manual_return_bringup.launch.py`는 **절대 동시에 실행하지 말 것** — 같은
CAN 버스/`/cmd_vel`을 두고 충돌한다.
