# Tool Manipulator Bringup

팔의 실기·모의 구동, 조이스틱 수동 구동, 설정 점검과 캘리브레이션 도구를 모아 둔
패키지다. Managed 경로 재생과 선택적으로 활성화하는 AprilTag 도킹 실행기를 제공한다.

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

```bash
# 빌드 전에는 소스 launch를 직접 지정한다.
ros2 launch src/arm/tool_manipulator_bringup/launch/ik_manual.launch.py
```

## 선택 기능: L2 경로 이후 AprilTag 도킹

기존 실행 명령과 L2 경로 0/1/2 재생은 그대로 유지한다.
`enable_tool_docking:=false`가 기본이며, 이 경우 기존 `tool_trajectory_joy.py`를
실행한다. 옵션을 켜면 `tool_docking_coordinator.py` 하나로 교체한다. 두 실행기를
동시에 띄우지 않는다. 새 실행기도 `tool_docking.yaml`에서 `enabled: false`인
경로는 기존 방식으로 재생하고 HOLD로 끝낸다.

`enable_tool_docking:=true`만 지정하면 활성 경로의 저장 궤적 이후 태그와 목표 오차를
`preview` 상태로 출력하지만 보정 이동이나 Arduino 구동은 실행하지 않는다. 실측·preview 검증까지
통과한 뒤에만 `enable_docking_motion:=true`를 추가해 실제 보정을 별도로 무장한다.

도킹을 활성화한 경로의 순서는 다음과 같다.

```text
L2 + 경로 선택 → 기존 관절 경로 검증/재생 → 실제 완료 및 HOLD
  → 태그 획득 → coarse 목표 → fine 목표 → dock 목표
  → 서로 다른 영상/관절 피드백에서 정렬·정지 연속 확인
  → `/arm/docking` Bool 펄스 → 새 `/arm/docked` 완료 상태 확인 → HOLD 유지
```

정렬은 매번 `검출 → 제한된 TCP 보정 → IK → 관절 경로 충돌 검사 → 실행 → 재검출`로
진행한다. 기존 Mode Manager만 모터 명령을 발행한다. 세 단계는 모두 태그 기준 TCP의
6D 목표이며, coarse/fine은 삽입 전 여유 공간, dock은 최종 삽입 위치로 직접 측정해
정한다. 경로 YAML의 마지막 점도 **접촉 전, 태그가 보이는 위치**여야 한다.
이미 삽입까지 포함한 경로 뒤에 도킹을 그대로 덧붙이지 않는다.

`config/tool_docking.yaml`의 경로 0 템플릿부터 다음 값을 채운다.

- 실제 `tag_id`, 검은 정사각형의 실측 `tag_size_m`.
- `stages`의 coarse/fine/dock 각각 `xyz_m`와 `xyzw`.
  이는 `T_tag_tcp`이며 카메라 좌표나 base 좌표가 아니다. quaternion 순서는 x/y/z/w.
- Arduino와 확인한 `actuation_timeout_s`. 최초 정렬 검증은
  `actuation_enabled: false`로 수행한다.
- `desired_docked: true`는 체결, `false`는 해지 경로다.
- 기구에 맞는 위치/각도/속도 허용값과 보정 범위. 제공된 값은 실기 검증값이 아니다.

필수값을 채운 후 경로 0만 `enabled: true`로 설정한다. 경로 1/2는 비활성으로 두면
기존 동작을 유지하며, 추후 경로 0 설정 전체를 복사하고 각 도구의 실측값을 넣는다.
경로 번호와 tag ID가 같다고 가정하지 않는다.

```bash
# 아래 프레임 이름은 실제 보정된 카메라 optical frame으로 교체한다.
ros2 launch src/arm/tool_manipulator_bringup/launch/ik_manual.launch.py \
  enable_tool_docking:=true \
  docking_camera_frame:=YOUR_CALIBRATED_OPTICAL_FRAME
```

카메라/검출기는 별도로 실행한다. 입력은 현재 vision의
`/arm/apriltag/centers` (`std_msgs/String`, stamp/frame_id/detections/corners_px 포함)와
`/arm/camera/color/camera_info`다. `CameraInfo`와 영상의 optical frame이 일치하고,
영상 시각의 `base_actuator ← optical frame` TF가 있어야 한다. 저장소의 측정 도구
`tool_change_min/scripts/measure_apriltag_docking.py`와 같은 코너 순서로 PnP를 계산한다.
태그 획득 후 검출 유실, 낮은 품질, 오래된 영상, TF 누락, 관절 피드백 유실, 소유권 변경,
보정 한계 초과, 충돌 검사/IK 실패는 중단 사유다. 마지막 검출값으로 계속 접근하지 않는다.

Arduino 명령과 완료 상태는 `std_msgs/Bool`을 사용한다. 체결은 `/arm/docking`에
`true` 후 즉시 `false`, 해지는 `false` 후 즉시 `true`를 발행하는 트리거 펄스다.
`/arm/docked`의 명령 이후 새 피드백이 체결 시 `true`, 해지 시 `false`가 되어야 완료로
인정한다. 명령 전에 남아 있던 동일 상태나 반대 상태는 완료로 인정하지 않는다. 대기 중
위치 이탈·timeout·취소는 팔을 정지시키며 자동 재시도나 반대 펄스, 자동 후퇴를 하지 않는다.
Bool 계약에는 STOP 명령이 없으므로 통신 단절과 비상 정지는 Arduino 자체 watchdog이
담당해야 한다. 시각적 도킹 판정은 물리적인 접촉/삽입 센서 확인을 대신하지 않으며,
실제 완료는 `/arm/docked` 피드백으로 구분한다.

```bash
ros2 service call /arm/tool_docking/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /arm/tool_trajectory/status
```

도킹 노드는 전용 취소 서비스와 기존 `/arm/cancel`에 의한 모드/epoch 변경을 감시한다.
도킹 노드 종료 시 launch도 종료한다. 프로세스 강제 종료/통신 단절에서 Arduino의 동작을
정지시키는 자체 watchdog은 펌웨어에 필요하다. 성공 시에도 자동 후퇴하지 않고 HOLD를 유지한다.
도킹이 활성화된 경로는 L2를 deadman으로 사용한다. 저장 경로 재생부터 정렬·체결까지 L2를
계속 눌러야 하며, L2를 놓거나 `/joy`가 0.3초 이상 끊기면 취소하고 자동 후퇴하지 않는다.
`actuation_enabled: true`인 경로는 시작 시점부터 `/arm/docking` subscriber와
`/arm/docked` publisher가
모두 보여야 한다.

오프라인 검증(ROS 패키지 빌드 및 실기 구동 없음):

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/arm/tool_manipulator_bringup/test/test_tool_docking.py \
  src/arm/tool_manipulator_bringup/test/test_tool_trajectory_joy.py
```

## 수동 구동

### mock, 시각화 없음

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py
```

### mock, RViz 포함

```bash
ros2 launch tool_manipulator_bringup manual.launch.py launch_rviz:=true
```

### 실기, 시각화 없음

잭스의 작업공간(`~/dolbotZ`)에서 매뉴얼을 실행하는 터미널은 ROS domain을 99로 맞춘다.
`hardware_config`를 생략하면 이 터미널에서 source한 패키지의
`config/hardware.yaml`을 자동으로 사용한다.

```bash
cd ~/dolbotZ
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py real_hardware:=true
```

실기에서는 `hardware.yaml` 사전 검사를 통과해야 하며, 시작 시 RMD 3축을 각 소프트
리밋 안쪽 2도로 저속 복귀한다. 수동 경로는 충돌 검사를 하지 않는다. 다른 hardware
launch나 읽기 전용 bridge와 동시에 실행하지 않는다.

로봇이 아닌 조이스틱 연결 PC에서 `/joy`만 발행하려면 두 PC의 ROS domain을 99로
맞추고 `ROS_LOCALHOST_ONLY=0`을 설정한다. 이때 잭스의 `manual.launch.py`에는
`launch_joy:=false`를 준다. 각 PC의 실행 터미널에서 설정한다.

```bash
# 조이스틱 연결 PC
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0

# 잭스
cd ~/dolbotZ
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py \
  real_hardware:=true launch_joy:=false
```

## MoveIt 구동

### mock + RViz

```bash
# tool_id: 0=gripper, 1=drill, 2=Phillips bit, 99=no tool
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
