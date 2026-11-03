# Tool Manipulator Bringup

팔의 실기·모의 구동, 조이스틱 수동 구동, 설정 점검과 캘리브레이션 도구를 모아 둔
패키지다. Managed 경로 재생과 AprilTag 도킹 실행기를 제공한다.

## 남긴 실행 경로

| 목적 | 실행 파일 | 모터 명령 | 시각화 |
| --- | --- | --- | --- |
| 조이스틱 수동 구동 | `manual.launch.py` | O | 선택 |
| IK/Manual 통합 모드 | `ik_manual.launch.py` | O | X |
| 실기 controller + MoveIt | `real_control.launch.py` | O | X |
| mock MoveIt/RViz | `mock.launch.py` | mock만 | O |
| 원격 PC의 `/joy` 발행 | `remote_joy.launch.py` | X | X |
| 주행 가능 자세 감시 | `arm_drive_guard.launch.py` | X | X |

`manual`은 오늘 pull 받은 Servo 없는 수동 제어 구현이다. 세부 버튼 매핑, 시작 복귀,
안전 조건과 rosbag 진단 방법은 [manual_controller.md](docs/manual_controller.md)를 따른다.
현재 `manual.launch.py`의 일반 수동 조작은 기존 충돌 검사 비활성 동작을 유지한다.
버튼식 각도 정렬은 MoveIt 장면으로 전체 경로를 검사하며, 사용자 요청에 따라 핑거 두 링크만 제외한다.
모터 명령은 계속 `manual.py`가 담당한다.
`manual.launch.py`는 기본적으로 실기(`real_hardware:=true`)를 사용한다.
모의 하드웨어로 실행할 때는 `real_hardware:=false`를 명시한다.

`manual.launch.py`의 L2+△(툴0), L2+○(툴1)는 사용자가 근처까지 접근한 뒤
AprilTag 기반 각도 정렬을 한 번 요청한다. 완료 후 자동 위치 보정은 하지 않는다. 기존 manual 키 매핑을 유지한다. 조작 순서,
도킹 보조 동작과 임의 파라미터는 [manual_docking_assist.md](docs/manual_docking_assist.md)를 따른다.
아래의 IK/Manual 자동 경로·도킹 설명은 `ik_manual.launch.py`에 해당한다.

## 제어 모드 기반 구조

`arm_bringup_common.py`는 독립 실행 launch가 아니라 모든 팔 제어 모드가 공유하는
내부 구성 모듈이다. 다음 항목을 한 경로에서 생성한다.

- 동일한 URDF/xacro mapping과 `robot_description`
- 실기 `hardware.yaml` preflight
- 단일 `ros2_control_node`와 `robot_state_publisher`
- 공통 controller 설정과 controller spawner

현재 선택 가능한 모드는 다음처럼 실행한다.

| 모드 | 실행 명령 | 활성 팔 controller |
| --- | --- | --- |
| Manual | `manual.launch.py` | `manual_position_controller` |
| IK/Manual managed | `ik_manual.launch.py` | `manual_position_controller` |
| trajectory/MoveIt | `real_control.launch.py` | `arm_controller` |

세 launch는 같은 공통 base를 사용한다. `ik_manual.launch.py` 안에서는 Mode Manager가
MANUAL/TCP_IK/TOOL_AUTO를 중재하지만, 다른 launch와의 동시 실행은 지원하지 않는다.
반드시 기존 팔 launch를 완전히 종료한 뒤 다른 모드를 실행한다.
`ik_manual.launch.py`도 첫 Options arm 진입에서 RMD 시작 복구 완료 후 DXL
base/roll/yaw를 0 rad로 정렬한다. 이후 현재 자세의 TCP 진입 조건을 검사한다.
L1은 deadman 전용이며 자세 이동을 시작하지 않는다. 현재 자세가 부적합할 때만
R2는 현재 정지 자세의 TCP 진입 가능 여부만 검사하며 움직임을 요청하지 않는다.

```bash
# 빌드 전에는 소스 launch를 직접 지정한다.
ros2 launch src/arm/tool_manipulator_bringup/launch/ik_manual.launch.py
```

## L2 경로 이후 AprilTag 도킹

`ik_manual.launch.py`는 기본적으로 `enable_tool_docking:=true`,
`enable_docking_motion:=true`,
`docking_camera_frame:=arm_camera_color_optical_frame`를 사용한다.
launch 자체는 툴 경로를 시작하지 않는다. 카메라와 AprilTag 검출기는 별도로 실행한다.

```text
□ 단독       → MoveIt HOME
L2 + □       → 현재 → HOME → BACK, BACK 피드백 확인
L2 + △/○/×  → 각각 tool0/tool1/tool2 선택
```

툴 선택은 시작 복구와 DXL 정렬을 완료한 arm의 신선한 정지 HOLD에서 받는다.
현재 BACK이면 바로 툴 접근을 계획한다. BACK이 아니면 선택을 보관하고
현재→HOME→BACK을 실행한 뒤 자동으로 이어간다. 이미 실행 중인 경로가 있으면
새 요청은 무시한다. BACK은 현재 관절 피드백의 최대 오차 `0.03 rad`로 확인한다.

접근 목표는 `config/cartesian_targets.yaml`의 `pre_joint_goal_rad`를 우선한다.
base는 제한 안에서 현재 관절값에 가까운 `2π` 동치각을 선택한다.
MoveIt은 경로 계획과 장면 충돌 검사만 제공하고, 실제 명령은 Mode Manager만 발행한다.
계획 경로는 관절 변화량 최대 `0.005 rad` 간격으로 다시 충돌 검사한 뒤 실행한다.
계획·IK·충돌 검사 실패, 오래된 피드백, 제어 소유권 변경, 명시적 취소는 이동을 중단한다.

### 툴0·툴1의 관절 경로 도킹

두 툴은 `fixed_joint_goal` 실행기와 다음 체결·복귀 흐름을 공유한다.

```text
BACK → 관측 자세 → 정지 후 새 태그 확인 → 전체 관절 접근 경로 검사/실행
  → 최종 관절 오차·정지 확인 → /arm/docking=true 한 번 발행 → 3초 대기
  → 실제 실행한 접근 경로를 역순으로 검사/실행 → BACK → HOME
```

`require_l2_hold: false`이므로 요청 이후 L2를 계속 누를 필요는 없다.
3초 대기는 체결 ACK를 필수 완료 조건으로 사용하지 않는다. 대기 중에도 피드백,
HOLD 소유권, Arduino ERROR/UNKNOWN 상태와 후퇴 전 정지 조건을 검사한다.
추가 EE 회전이나 별도의 직선 후퇴는 수행하지 않는다. 후퇴·BACK·HOME 경로가
검증되지 않으면 현재 자세에서 정지하며 임의의 대체 경로로 이동하지 않는다.

툴0은 안전 경유점을 거쳐 관측 자세로 간다. 태그0의 정지 상태 표본 8개로 저장 기준과의
위치 차이를 한 번 확정한다. 표본 분산 한계는 `3 mm`, 보정량 한계는 `10 mm`다.
전체 남은 접근점에 같은 위치 보정량을 적용하고 IK로 다시 계산한다. 실측 외곽 접근점과
마지막 `70 mm` 축방향 샘플을 사용하며, 전체 관측→도킹 구간을 직선으로 만들지 않는다.
역순 후퇴 뒤에는 저장된 복귀 경유점을 거쳐 BACK→HOME으로 이동한다.

툴1의 최종 도킹 관절값은 다음으로 확정했다. 순서는
`base, shoulder, elbow, wrist_pitch, wrist_roll, wrist_yaw`, 단위는 rad다.

```yaml
final: [-3.136990711226343, 0.20682150637447397, 1.1557570519759337,
  -1.5262903776058194, -0.3390097541229333, -1.5017671913398365]
```

관측 자세는 사용자가 새로 제공할 예정이므로 이전 관측점은 실행 목표에서 제외했다.
`cartesian_targets.yaml`의 tool1은 `executable: false`이며, 도킹 설정에도
`blocked_reason`을 두어 새 관측값을 반영하기 전에는 툴1 요청이 HOME/BACK staging이나
도킹 이동을 시작하지 않는다. 관측점이 들어오면 최종점에 맞춘 접근·후퇴·복귀 경로와
태그 보정을 함께 통합한다. 현재 최종점은 `0.02 rad` 계획 여유를 만족한다.

실제 툴박스2와 URDF 위치가 다르다는 사용자 확인에 따라, 툴1의 최종 진입 경로에만
`final_entry_ignored_collision_links: [tool_case_y_plus]`를 적용한다.
이는 해당 구간의 상태 유효성 응답에서 툴박스2와의 접촉을 제외하며 다른 물체와의 충돌,
자체 충돌, 관절 제한은 계속 검사한다. 관측점·HOME/BACK 경로와 역순 후퇴에는 이 예외를
전달하지 않으며 SRDF의 전역 충돌 행렬도 수정하지 않는다. 후퇴 때에도 모델 불일치가
남아 있으면 현재 정책에서는 차단될 수 있다.

툴0은 `tag36h11` ID 0, 툴1은 ID 1이며 검은 사각형의 실측 변 길이는 모두 `20 mm`다.
입력은 `/arm/apriltag/centers` (`std_msgs/String`, stamp/frame_id/detections/corners_px)와
`/arm/camera/color/camera_info`다. 두 입력의 optical frame이 일치하고 영상 시각의
`base_actuator ← optical frame` TF가 있어야 한다. 태그 확인은 관측 자세에서 정지한 뒤
수신한 새 영상으로 시작한다. 취득 timeout은 `10초`이며 낮은 품질, 프레임 불일치,
오래된 영상과 TF 누락은 정상 확인 표본으로 세지 않는다.

`actuation_completion: delay`, `actuation_delay_s: 3.0`,
`return_home_after_back: true`, `retreat_mode: reverse_docking_path`가 공통 완료 정책이다.
기존 `camera_normal_advance`, `tag_tcp_stages`, 피드백 기반 체결과 TCP 축 후퇴 구현은
별도 설정에서 사용할 수 있다. `ik_manual_test.launch.py`의 독립 테스트 설정은 유지한다.
`enable_docking_motion:=false`이면 관측 자세에서 최종 도킹 이동과 체결 없이 HOLD한다.

```bash
ros2 service call /arm/tool_docking/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /arm/tool_trajectory/status
```

UI의 `/arm/tool{0,1,2}_docking` 또는 `/arm/tool{0,1,2}_release`는 Bool 상승 에지로
요청한다. 같은 요청을 다시 보내려면 먼저 false로 내려 재무장한다. UI 요청은 알려진
체결 상태를 먼저 확인한다. `actuation_enabled: true`인 경로는 시작할 때부터
`/arm/docking` subscriber와 `/arm/docked` publisher가 있어야 한다.

## 수동 구동

### mock, 시각화 없음

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py real_hardware:=false
```

### mock, RViz 포함

```bash
ros2 launch tool_manipulator_bringup manual.launch.py real_hardware:=false launch_rviz:=true
```

### 실기, 시각화 없음

잭스의 작업공간(`~/ResKU`)에서 매뉴얼을 실행하는 터미널은 ROS domain을 99로 맞춘다.
`hardware_config`를 생략하면 이 터미널에서 source한 패키지의
`config/hardware.yaml`을 자동으로 사용한다.

잭스의 기존 셸에는 `dolbotZ` 패키지 경로가 먼저 잡힐 수 있다. 실기와 원격 조이스틱은
`cau`로 CAN을 켠 뒤 아래 스크립트로 실행한다. 스크립트가 깨끗한 ROS 환경을 만들고
두 필수 패키지가 `ResKU`에서 선택되는지 검사한다.
소스를 새로 동기화한 뒤에는 잭스에서 `tool_manipulator_bringup`을 한 번 빌드한다.
매번 실행할 때 다시 빌드할 필요는 없다.

```bash
cau
bash ~/ResKU/util/start_manual_arm_remote_joy.bash
```

아래 직접 실행 예제는 ROS 환경에 다른 작업공간이 섞이지 않은 터미널에서만 사용한다.

```bash
cd ~/ResKU
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py
```

실기에서는 `hardware.yaml` 사전 검사를 통과해야 하며, 시작 시 RMD 3축을 각 소프트
리밋 안쪽 2도로 저속 복귀한다. 처음 Options로 arm을 선택하면 DXL의 base/roll/yaw
3축을 현재 자세에서 0 rad로 저속 정렬한다. 정렬 중 수동 입력은 차단되며, 완료 후
조작 입력을 중립으로 놓아야 수동 제어가 열린다. 수동 경로는 충돌 검사를 하지 않는다. 다른 hardware
launch나 읽기 전용 bridge와 동시에 실행하지 않는다.
DXL 정렬 제한 시간은 시작 각도와 0.09 rad/s 속도로 계산한 예상 시간에 15초를 더한다.

로봇이 아닌 조이스틱 연결 PC에서 `/joy`만 발행하려면 두 PC의 ROS domain을 99로
맞추고 `ROS_LOCALHOST_ONLY=0`을 설정한다. 이때 잭스의 `manual.launch.py`에는
`launch_joy:=false`를 준다. 각 PC의 실행 터미널에서 설정한다.

```bash
# 조이스틱 연결 PC
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0

# 잭스
cd ~/ResKU
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py \
  launch_joy:=false
```

## MoveIt 구동

### mock + RViz

```bash
# task/tool ID: 0=gripper, 1=drill, 2=supply_box handled by gripper, 99=no mounted tool
ros2 launch tool_manipulator_bringup mock.launch.py tool_id:=0 use_mesh:=true
```

### 실기, RViz 없음

```bash
ros2 launch tool_manipulator_bringup real_control.launch.py \
  hardware_config:=$HOME/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml \
  tool_id:=0 use_mesh:=true
```

기본값은 controller 활성화 뒤 SRDF `home`으로 이동한다. 자동 이동을 막으려면
`move_home_on_start:=false`, 최소 이동 시간을 바꾸려면 `home_duration:=10.0`을 추가한다.
이 시작 이동은 joint trajectory이며 MoveIt 충돌 경로 계획은 수행하지 않는다.

실기 MoveIt에 RViz가 필요하면 위 launch를 실행한 상태에서 별도 터미널에 다음을 실행한다.

```bash
rviz2 -d $(ros2 pkg prefix tool_manipulator_moveit_config)/share/\
tool_manipulator_moveit_config/config/moveit.rviz
```

### named pose

팔 내부 MANUAL/TCP_IK/TOOL_AUTO 모드 중재는 별도
[Control Mode Manager](docs/control_mode_manager.md)를 참고한다.
기존 아래 named-pose 직접 실행 명령은 managed stack과 동시에 사용하지 않는다.

```bash
# pose 목록과 dry-run
bash ~/ResKU/utils/move_to_pose.sh --list
bash ~/ResKU/utils/move_to_pose.sh home --dry-run

# controller가 실행 중일 때 실제 이동
bash ~/ResKU/utils/move_to_pose.sh home --duration 8.0
```

현재 `ee_joint`는 velocity mode이므로 `real_control`과 `manual` 모두
`ee_velocity_controller`를 사용한다. position trajectory 방식의 구형
`move_gripper.py`는 현재 하드웨어 모드와 맞지 않아 제거했다.

## 출력·확인 도구

이 절의 도구는 별도 언급이 없는 한 모터 명령을 보내지 않는다.

### 손으로 경로 티칭

팔을 중력 낙하하지 않도록 물리적으로 받친 다음 모든 명령 컨트롤러를 내리고,
Dynamixel 토크와 RMD 구동을 해제한다. `joint_state_broadcaster`는 유지되어
다이나믹셀 4축과 RMD 3축의 위치를 계속 읽는다.

```bash
ros2 run tool_manipulator_bringup enter_teach_mode.py --yes
ros2 run tool_manipulator_bringup capture_tool_trajectory.py
```

두 번째 명령에서 Enter를 누를 때마다 `time_from_start_sec`와 6축
`positions_rad`가 출력된다. 출력값을 검토해 `config/tool0_trajectory.yaml` 등의
`points`에 저장한다. 읽기모드는 의도적으로 자동 복귀하지 않는다. 다시 구동하려면
현재 launch를 완전히 종료하고 팔을 지지한 상태에서 재시작한다. 실행 중 개별 torque
서비스로 다시 켜면 이전 명령이 재사용될 수 있으므로 금지한다.

### 설정 검사

```bash
ros2 run tool_manipulator_bringup check_hardware_config.py
ros2 run tool_manipulator_bringup check_hardware_config.py --show-values
ros2 run tool_manipulator_bringup read_hardware_safety_params.py --dry-run
```

### 현재 관절 위치와 raw 값

실기 controller를 끄고 읽기 전용 bridge만 실행한다.

```bash
# 터미널 A
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: Enter를 누를 때마다 현재 값 출력
ros2 run tool_manipulator_bringup capture_arm_pose.py

# raw encoder/pulse 출력
ros2 run tool_manipulator_bringup capture_arm_raw.py
ros2 run tool_manipulator_bringup capture_rmd_calibration.py
```

### world 좌표와 TF에서 역산한 관절값

이 파일은 의도적으로 패키지 실행 파일로 설치하지 않은 독립 점검 스크립트다.
`robot_state_publisher`와 TF가 실행 중일 때 시작하고, 출력하고 싶은 순간 Enter를 누른다.
각 joint/link와 `cam_link`의 world 좌표를 출력한다.

```bash
python3 ~/ResKU/src/arm/tool_manipulator_bringup/scripts/joint_world_coordinates.py
```

### 현재 위치를 영점으로 저장

이 명령만 `--apply`를 붙이면 `hardware.yaml`을 수정한다. 먼저 미리보기를 확인한다.
실기 제어 launch인 `manual`, `real_control`, `ik_manual`, `ik_manual_test`는
선택된 ROS 패키지가 속한 작업공간의 `src/arm/tool_manipulator_bringup/config/hardware.yaml`을
우선 사용한다. 소스가 없는 배포에서는 설치된 설정을 사용한다.
`hardware_config:=/절대/경로/hardware.yaml`으로 명시한 경로가 우선한다.

저장 후 실행 중인 실기 제어 노드를 종료하고 다시 시작해야 반영된다.
시작 로그의 `Arm hardware config: ...; base_joint zero_raw=...`를 확인한다.
2026-10-31 베이스 측정값은 `zero_raw=1561`이며, 하드웨어의
`position calibration: zero_offset=-0.747048643700 rad`와 대응한다.
Options 첫 진입은 DXL을 관절각 0으로 이동시키므로 기계적 영점에서 시작할 때
실제 제어기의 `/joint_states` 베이스 값이 0 근처인지 먼저 확인한다.

```bash
ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml

ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml --apply
```

## 유지 파일 기준

`trajectory_diagnostics.py`와 관련 문서·테스트는 DXL 수동 구동 진단
기록이므로 실행 진입점은 아니어도 분석 자료로 유지한다.

- `launch/`: `manual`, `ik_manual`, `real_control`, `mock`, `remote_joy`, `arm_drive_guard`
- `scripts/`: 실제 구동에 필요한 모듈과 읽기/검사/캘리브레이션 도구
- `config/`: `hardware`, `manual_controllers`, `servo` 안전 margin, RViz 설정
- `test/`: manual, named pose, 안전·영점·trajectory와 URDF 정적 검사

과거 영점 기준의 `d/l` raw 상수를 현재 `hardware.yaml`로 변환하던 테스트는 제거했다.
현재 기준 검증값이 필요하면 실제 자세를 다시 캡처하여 raw와 `/joint_states`를 함께
기록하고, 당시 사용한 `hardware.yaml`을 명시한다. 기존 hardware 백업은 보정 이력
확인용이므로 실행 설정은 아니지만 유지한다.

## 현재 기준으로 남은 작업

1. `manual.launch.py`가 사용하는 현재 `hardware.yaml` 상태에서 대표 수동 자세를
   다시 캡처한다. 같은 시각의 raw 값과 `/joint_states` rad 값을 함께 저장한다.
2. 재캡처 값으로 관절 제한·영점 변환 검증을 추가한다. 과거 raw 숫자를 현재 영점에
   그대로 대입하거나 각도를 임의로 wrap하지 않는다.
3. `ik_manual.launch.py real_hardware:=false`에서 시작 복귀, MANUAL/TCP_IK 전환,
   Deadman과 충돌 거부를 Mock 검증한다.
4. 별도 승인 후 실기에서 저속 TCP 이동을 검증한다. `manual.launch.py`와
   `ik_manual.launch.py`는 동시에 실행하지 않는다.
5. Tool Change에 사용할 PRE와 최종 TCP 자세는 위 보정이 확정된 뒤 YAML에 티칭한다.

ROS 패키지를 다시 빌드하기 전까지 설치 공간에는 삭제한 예전 실행 파일이 남아 있을 수
있다. 소스 기준 진입점은 이 README의 목록만 사용한다.
