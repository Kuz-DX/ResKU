# reduced_odom 평지 실차 검증 절차

`reduced_odom_node` (5-state EKF, state = `[x, y, yaw, vx, wz]`)는 현재
production odometry다 -- `reduced_odom_bringup.launch.py`에서 `odom -> base_link` TF와
`/odometry/filtered`를 발행하는 유일한 노드고, generic 15-state
`robot_localization`은 더 이상 같이 띄우지 않는다.

**이 문서는 평지 검증만 다룬다.** 현재 실차 환경에 경사로가 없어서 경사
주행 테스트는 이번 범위에서 제외했다 -- 별도 문서(추후)에서 다룰 것.

이 문서는 estimator(Q/R/gate)를 바꾸지 않는다. 여기서 하는 건 (1) 충돌 없는
manual validation bringup을 구성하고 (2) bag을 정확히 기록하고 (3) 무엇을
봐야 하는지 정리하는 것뿐이다. Q/R/gate 튜닝은 이 bag들을 실제로 분석한 뒤
별도 작업으로 진행한다.

## -1. Manual vs Autonomous 경로 추적, 그리고 왜 둘을 같이 못 띄우는가

### 기존 manual 경로 (코드 확인)

```text
Joystick(/dev/input/js0)
  -> joy_node (pkg 'joy')                         /joy (sensor_msgs/Joy)
  -> manual_joy_control_node                       /motor_speed_cmd (std_msgs/Float32MultiArray, [left_dps, right_dps])
  -> can_driver_node                               CAN(channel='can_drive', left_can_id=1, right_can_id=2) -> Motors
```

`manual_joy_control_node.py`: `/motor_speed_cmd`는 **Twist가 아니라
Float32MultiArray(좌/우 dps)** 다 -- tank-drive 스타일이라 `/cmd_vel`
경로와 애초에 호환 안 됨. `can_driver_node.py`: `self.bus =
can.interface.Bus(channel=self._can_channel, bustype='socketcan')`,
기본 `can_channel='can_drive'`, `left_can_id=1`/`right_can_id=2`. **odometry
를 전혀 publish하지 않는다** (Odometry publisher 자체가 코드에 없음).

### Autonomous / reduced_odom 경로 (코드 확인)

```text
(MPPI 등 상위 명령원, 이번 validation에서는 안 씀)
  -> /cmd_vel (geometry_msgs/Twist)
  -> rmd_x8_driver_node                            CAN(channel='can_drive', left_motor_can_id=1, right_motor_can_id=2) -> Motors
       -> /wheel/odom, /wheel/joint_states, /wheel/motor_status

/imu (myahrs_driver_node) + /wheel/odom
  -> reduced_odom_node -> /odometry/filtered, /odometry/diagnostics, odom->base_link TF
```

### CAN/motor ownership 충돌 (코드 근거)

`can_driver_node.py`와 `rmd_x8_driver_node.py` 둘 다:
- 같은 방식(`can.interface.Bus(channel=..., bustype='socketcan')`)으로
  **같은 기본 CAN channel(`'can_drive'`)**에 소켓을 연다.
- **같은 모터 CAN ID**를 쓴다 (`left_can_id=1`/`right_can_id=2` vs
  `left_motor_can_id=1`/`right_motor_can_id=2` -- `can_driver_node.py` 주석에
  "`1 -> 0x141`"로 실제 arbitration ID까지 명시돼 있고, `rmd_x8_driver_node.py`도
  `proto.motor_send_id(channel.can_id)`로 동일 ID 체계를 씀).

SocketCAN 자체는 브로드캐스트 버스라 두 프로세스가 같은 인터페이스에 소켓을
여는 것 자체는 OS 레벨에서 막히지 않는다 -- 그래서 "당장 에러가 나서
못 뜬다"가 아니라, **두 프로세스가 동시에 같은 물리 모터(0x141/0x142)에
독립적으로 속도 명령을 계속 내보내면서 서로 경합**하는, 훨씬 조용하고
위험한 문제가 생긴다: 마지막에 버스에 도착한 명령이 이기는 식으로 모터
속도가 예측 불가능하게 튀거나, 한쪽 노드의 CAN reply 처리 로직
(`rmd_x8_driver_node.py`의 `_on_can_message`/`Notifier`)이 다른 쪽 노드가
유발한 응답 프레임까지 같이 받는 상황이 생긴다. **따라서 `can_driver_node`와
`rmd_x8_driver_node`를 동시에 실행하는 것은 명확히 금지한다.**

### 결론: manual_joy_control/can_driver로는 reduced_odom 검증이 불가능

`/wheel/odom`을 내는 노드가 `rmd_x8_driver_node` 하나뿐인데, 그게 뜨려면
`can_driver_node`를 띄우면 안 된다(위 충돌). 그렇다고 `rmd_x8_driver_node`만
따로 띄우고 명령을 안 주면 로봇을 못 움직이니 검증 자체가 안 된다. 그래서
이번 작업에서 만든 게 아래 4번 `reduced_odom_validation.launch.py`다 --
`can_driver_node`/`manual_joy_control_node`는 아예 빼고, `rmd_x8_driver_node`가
받을 수 있는 `/cmd_vel`(Twist)로 조이스틱을 직접 연결한다.

## 0. 실제 topic 구조 (코드/launch 기준 확인, 추측 아님)

| 분류 | topic | 타입 | 소스 |
|---|---|---|---|
| wheel odometry | `/wheel/odom` | `nav_msgs/Odometry` | `rmd_x8_driver_node.py` (`twist.linear.x`=차동휠 평균 전진속도, `twist.angular.z`=스키드조향 wz) |
| IMU | `/imu` | `sensor_msgs/Imu` | `myahrs_driver_node.cpp` (`orientation`만 유효, 아래 참고) |
| production 5-state EKF 출력 | `/odometry/filtered` | `nav_msgs/Odometry` | `reduced_odom_node` |
| production EKF 진단 | `/odometry/diagnostics` | `diagnostic_msgs/DiagnosticArray`, 1Hz | `reduced_odom_node` |
| TF (동적) | `/tf` (`odom`->`base_link`) | | `reduced_odom_node` |
| TF (정적) | `/tf_static` (`base_link`->`imu_link`, `base_link`->`camera_link`) | | `reduced_odom_bringup.launch.py`의 `static_transform_publisher` 2개 |
| 좌/우 wheel raw 속도 | `/wheel/joint_states` | `sensor_msgs/JointState` | `rmd_x8_driver_node.py` -- `name=[left_wheel_joint,right_wheel_joint]`, `velocity`=rad/s, `effort`=전류(A, Nm 아님 주의) |
| 모터 피드백/진단 | `/wheel/motor_status` | `diagnostic_msgs/DiagnosticArray` | `rmd_x8_driver_node.py` |
| 실제 구동 명령 | `/cmd_vel` | `geometry_msgs/Twist` | `rmd_x8_driver_node`가 구독(=최종 입력). `current_ramp_node` 통과 후 값(`todrive.md`) |
| 긴급정지 명령 | `/cmd_vel_safety` | `geometry_msgs/Twist` | `stability_monitor_node`, 평상시 무발행 |

**raw gyro / raw magnetometer는 이 시스템에 없다.** `myahrs_driver_node.cpp`는
myAHRS+가 보내는 `"$RPY,<seq>,<roll>,<pitch>,<yaw>*<checksum>"` fused
orientation 문자열만 파싱해서 `/imu`로 발행하고, `angular_velocity_covariance[0]
= -1.0`/`linear_acceleration_covariance[0] = -1.0`로 "이 필드 데이터 없음"을
명시적으로 표시한다. 별도의 raw gyro나 magnetometer topic도 발행하지 않는다.
**주의: 이건 "myAHRS+ 디바이스에 magnetometer가 없다"는 뜻이 아니다** --
myAHRS+는 magnetometer를 포함하는 정식 AHRS이고, 이전 세션에서 "나침반이
없어서 yaw가 gyro drift만 가진다"고 적었던 건 틀린 전제였다(정정:
`direct_odom_node.py` 상단 docstring 참고). 정확한 사실은 "디바이스 자체엔
있을 가능성이 높지만, **이 드라이버가 raw 값을 ROS로 노출하지 않는다**"는
것 -- 즉 yaw 오차 원인에는 gyro drift뿐 아니라 magnetometer 기반 오차
(모터/배터리 자기장 교란, hard/soft-iron distortion, 캘리브레이션 오차)도
포함해서 봐야 하지만, 이 오차들을 원인별로 직접 분리 계측할 raw 데이터가
지금 시스템엔 없다. 아래 Test 2가 그나마 간접적으로 이걸 들여다보는
방법이다.

`cmd_vel`은 **오직 `rmd_x8_driver_node`가 살아있을 때만 의미가 있다** --
manual 조종(`can_driver` + `manual_joy_control`, `/motor_speed_cmd`)은 완전히
다른 노드/토픽 스택이고, CAN/motor를 공유하는 두 드라이버 노드를 동시에
띄우면 충돌이 난다(위 -1번 참고). **reduced_odom 검증은 `reduced_odom_bringup.launch.py`
계열(아래 4번 `reduced_odom_validation.launch.py`)로 로봇을 움직여야 한다 --
manual 스택으로는 `/wheel/odom` 자체가 안 나온다.**

## 1. reduced_odom의 wz 측정 구조 (코드 기준 확인)

`reduced_odom_node.cpp::wheelCb()`가 유일한 wz measurement source다:

```cpp
const double wz = msg->twist.twist.angular.z;   // == /wheel/odom의 twist.angular.z
...
ekf_.correct(4, wz, r_wz_, wheel_gate_, ...);
```

그리고 `/wheel/odom`의 `twist.angular.z`는 `rmd_x8_driver_node.py`에서
좌/우 wheel differential로 계산된 값이다(`w = (v_right - v_left) /
track_width`). **즉 지금 wz는 순수히 "wheel 기반"이다 -- IMU
angular_velocity는 애초에 데이터가 없어서(`-1` covariance) fusion 후보에도
안 들어간다.** `/wheel/joint_states`의 `velocity`(좌/우 wheel rad/s)를 bag에
같이 담아두면, 사후에 `(v_right - v_left)/track_width`를 직접 재계산해서
`/wheel/odom`의 wz, `/odometry/filtered`의 wz(EKF 추정치)와 3자 비교가
가능하다 -- Test 4에서 이걸 본다.

## 2. validation bringup 실행: `reduced_odom_validation.launch.py`

**분석 결과 `reduced_odom_bringup.launch.py` 자체는 이미 sensor + `rmd_x8_driver` +
`reduced_odom`만 띄우는 최소 launch였다**(Nav2/MPPI/robot_localization/
can_driver 없음, 코드 확인 -- 아래 3번). 그래서 이 파일은 그대로 두고, 빠진
조각(조이스틱 -> `/cmd_vel`)만 새 validation 전용 launch에서 추가했다 --
기존 production/manual 파일은 **하나도 수정하지 않았다.**

```bash
ros2 launch src/drive/autonomous/reduced_odom/launch/reduced_odom_validation.launch.py
# CAN dry-run: can_interface:=vcan0 추가
```

이 launch가 띄우는 것 (전부 기존 패키지 재사용, 새 노드는 launch 파일
자체 하나뿐):

```text
rmd_x8_driver_node, myahrs_driver_node, static TF x2, reduced_odom_node   <- reduced_odom_bringup.launch.py 그대로 include
joy_node (pkg 'joy')                                                     <- manual_control.launch.py와 동일하게 재사용
teleop_twist_joy_node (pkg 'teleop_twist_joy')                            <- 표준 패키지, 새 노드 안 만듦
stability_monitor_node (pkg 'robot_bringup')                              <- 기존 하드웨어 안전장치 재사용
```

`joy`/`teleop_twist_joy` 둘 다 이 환경에 이미 설치돼 있는 걸 확인했다
(`ros-humble-teleop-twist-joy`, `ros-humble-joy`). axis/버튼 매핑은
`manual_joy_control_node.py`의 실측 검증된 매핑(축1=왼쪽 스틱 상하 ->
`linear.x`, 축3=오른쪽 스틱 좌우 -> `angular.z`, 버튼6=L2)을 그대로
재사용했다 -- 추측 아님. 속도 상한(`max_linear_mps=0.2`, `max_angular_radps=0.5`)도
`todrive.md`/`our_mppi_params.yaml`의 기존 첫 실차 시험값을 그대로 가져왔다.
`enable_button`(L2)을 누르고 있어야만 `/cmd_vel`이 나간다(deadman switch) --
스틱이 실수로 건드려져도 안 움직인다.

`can_driver`/`manual_joy_control_node`는 포함하지 않았다(-1번 CAN 충돌 근거).
`current_ramp_node`(autonomous 전용 소프트 전류 램프)도 의도적으로 뺐다
(`auto_cmd_relay_node`는 이후 프로젝트에서 아예 제거됨, 2026-08-25) --
teleop 속도 상한 + `rmd_x8_driver_node` 자체의
`max_wheel_speed_dps` 하드 클램프가 이미 이중으로 걸려 있어서다. 전류
프로파일까지 검증하고 싶다면 알려주면 추가하겠다(이번 범위 밖으로 판단).

실행 확인(실제로 8개 프로세스 전부 정상 기동하는 것까지 확인함, CAN/IMU
장치가 없는 개발 환경이라 그 두 드라이버만 장치 접근 단계에서 실패 -- 그
외 6개 노드는 전부 정상 시작):

```bash
ros2 node list
```

Nav2/MPPI/`can_driver`/`manual_joy_control`/`robot_localization` 관련 노드가
하나도 없어야 한다.

### 이 launch만으로 부족할 때 (조이스틱 없이 스크립트로 정밀 기동만 필요할 때)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 20
```
같이 `/cmd_vel`을 직접 퍼블리시할 수도 있다(이 launch가 이미 떠 있는 상태에서는
`teleop_twist_joy_node`도 같은 topic에 publish하므로, 이 방법을 쓸 거면 launch에서
teleop 노드를 빼거나 조이스틱 enable 버튼을 안 눌러서 teleop 쪽이 아무것도
안 내보내게 해야 publisher가 2개로 안 겹친다 -- 8번 참고).

## 3. `reduced_odom_bringup.launch.py`가 실제로 무엇을 띄우는지 (코드 확인)

`reduced_odom_bringup.launch.py`의 `LaunchDescription`을 그대로 읽으면 5개 액션뿐이다:
`DeclareLaunchArgument` x2, `rmd_x8_driver_node`, `myahrs_driver_node`,
`static_transform_publisher` x2(`base_link->imu_link`, `base_link->camera_link`),
`reduced_odom_node`. **Nav2/MPPI/controller_server/behavior_server/planner_server/
autonomous command publisher/기존 robot_localization/manual CAN driver, 그 중
아무것도 없다.** 그래서 이번 작업에서 새 launch를 만들 때 `reduced_odom_bringup.launch.py`
자체는 전혀 수정하지 않고 `IncludeLaunchDescription`으로 그대로 가져다
썼다.

## 4. bag 기록

```bash
./src/drive/autonomous/reduced_odom/scripts/record_reduced_odom_validation.sh
# 또는 출력 위치 지정: ./record_reduced_odom_validation.sh /path/to/bags
```

스크립트가 필수 topic(`/wheel/odom`, `/imu`, `/odometry/filtered`,
`/odometry/diagnostics`, `/tf`, `/tf_static`) 존재를 먼저 확인하고, 하나라도
없으면 기록을 시작하지 않고 종료한다. `/wheel/joint_states`,
`/wheel/motor_status`, `/cmd_vel`, `/cmd_vel_safety`는 있으면 자동으로 같이
기록한다. bag은 기본적으로 `$HOME/dolbotz_bags/`(repo 밖, `DOLBOTZ_BAG_DIR`
환경변수로 변경 가능)에 `reduced_odom_validation_YYYYMMDD_HHMMSS/`로 저장된다.
**이번 turn에서 record script 자체는 수정하지 않았다** -- `reduced_odom_validation.launch.py`가
내는 topic 집합이 지난 turn에 이미 만들어둔 필수/선택 목록과 정확히 같아서
바꿀 이유가 없었다.

## 5. 시작 전 충돌/중복 점검

`reduced_odom_validation.launch.py` 기동 직후, bag 기록을 시작하기 전에
확인:

```bash
ros2 node list
# can_driver_node, manual_joy_control_node, controller_server,
# planner_server, behavior_server, ekf_filter_node(robot_localization) 중
# 아무것도 없어야 한다.

ros2 topic info /cmd_vel -v
# publisher가 정확히 1개(teleop_twist_joy_node)여야 한다.

ros2 topic info /cmd_vel_safety -v
# publisher가 정확히 1개(stability_monitor_node)여야 한다 -- slope_traverse_node는
# path_control.launch.py 전용이라 이 launch엔 없음.

ros2 topic echo /tf --once
ros2 topic info /tf -v
# odom->base_link publisher가 reduced_odom_node 하나뿐이어야 한다
# (robot_localization의 ekf_filter_node가 같이 떠 있으면 두 번째 발행자가 생김).
```

목표: `manual command source = 1`(teleop_twist_joy), `motor command
consumer = rmd_x8_driver_node 하나`, `unexpected autonomous publisher = 0`.

## 6. 저속 smoke test (bag 기록 전에 먼저)

### Step 1 -- 정지 상태 센서 확인
`/wheel/odom`, `/imu`, `/odometry/filtered`, `/odometry/diagnostics`가
정상 수신되는지(`ros2 topic hz`), `level=OK`인지 확인.

### Step 2 -- 아주 조금 전진
`enable_button`(L2)을 누른 채 왼쪽 스틱을 살짝만 미는 정도. 확인:
`cmd_vel.linear.x > 0`, `/wheel/odom` vx > 0, `/odometry/filtered` vx > 0,
`/odometry/filtered`의 x가 올바른 방향으로 증가하는지.

### Step 3 -- 아주 작은 좌/우 회전
오른쪽 스틱을 살짝만 좌/우로. 확인: `cmd_vel.angular.z` 부호, wheel-derived
wz(`/wheel/odom` twist.angular.z) 부호, EKF wz(`/odometry/filtered`) 부호,
yaw 변화 부호가 전부 일관되는지.

**부호가 하나라도 반대면 본격적인 bag 테스트 전에 멈추고 frame/sign 문제부터
잡을 것** -- 이 launch 자체를 더 수정할지, `left_motor_sign`/`right_motor_sign`
류의 기존 파라미터를 조정할지는 실제로 부호가 틀린 게 확인된 뒤 사용자와
상의해서 결정한다(이번 작업 범위에서 미리 손대지 않음).

## 7. 진단 필드 (`/odometry/diagnostics`, `DiagnosticArray`, 1Hz)

이번 작업에서 아래 항목을 **estimator 판정 로직 변경 없이** 추가했다
(`reduced_ekf.hpp::correct()`에 순수 out-parameter만 추가, 기존 4개
gtest(`test_reduced_ekf.cpp`) 전부 통과 확인):

| key | 의미 |
|---|---|
| `wheel_messages`, `imu_messages` | 누적 수신 메시지 수 |
| `wheel_rejected`, `imu_rejected` | (기존) 누적 거부 수 |
| `wheel_vx_rejected`, `wheel_wz_rejected` | (신규) `wheel_rejected`를 vx/wz 채널별로 분해 |
| `yaw_innovation_rejected` | (기존) yaw gate 거부 수 |
| `timestamp_anomalies` | (기존) 음수/역행/과대 dt |
| `wheel_stale`, `imu_stale` | (신규, 0/1) 각 센서 개별 stale 여부 -- 기존엔 `level`/`message`에 합쳐져 있어서 둘 중 뭐가 죽었는지 diagnostics만으로 구분 불가했음 |
| `vx`, `wz`, `yaw` | (기존) 현재 state 추정치 |
| `Pxx`, `Pyy`, `Pyaw`, `Pvx`, `Pwz` | (신규) covariance 대각 -- 이미 `/odometry/filtered`의 covariance 필드에 있던 값을 diagnostics에서도 바로 보이게 노출만 함 |
| `yaw_innovation`, `yaw_innovation_S` | (신규) 마지막 yaw correct()의 innovation과 S(=P+R) -- R/gate 튜닝용. gate에 걸려 reject된 샘플도 이 값은 채워짐 |
| `wheel_vx_innovation`, `wheel_vx_innovation_S` | (신규) 위와 동일, vx 채널 |
| `wheel_wz_innovation`, `wheel_wz_innovation_S` | (신규) 위와 동일, wz 채널 |

## 8. covariance ceiling: 이번 작업에서 추가 안 함 (의도적)

절대 XY 위치 센서가 없는 구성이라 `Pxx`/`Pyy`가 시간이 갈수록 커지는 것
자체는 (예전 15-state EKF의 T⁵ 폭주와 달리) 이 5-state 모델에서는 훨씬
완만할 것으로 예상되지만, 정상적인 현상일 수 있다. 이번 bag 분석에서는
아래만 확인하고 임계값/ceiling은 넣지 않는다:

- `Pxx`/`Pyy`가 **폭발적으로**(수 초 단위로 몇 자릿수씩) 증가하는가, 아니면
  완만한가
- `Pyaw`/`Pvx`/`Pwz`는 (yaw는 IMU로, vx/wz는 wheel로 계속 보정되므로) 발산하지
  않고 어떤 값 근처에서 안정되는가
- `NaN`/`Inf`가 나오는가 (`ReducedEkf::correct()`가 업데이트 후
  `x_.allFinite() && P_.allFinite()`를 체크해서 false를 리턴하긴 하지만,
  **그 시점에 이미 `x_`/`P_` 멤버 자체는 덮어써진 뒤라는 점 관찰됨** --
  reject 리턴이 "이전 상태로 롤백"을 보장하지 않는다. 이번 작업에서 estimator
  코드는 안 건드리기로 했으니 고치지 않았지만, bag에서 이 케이스가 실제
  발생하는지 확인할 가치는 있다.)
- negative variance(대각 성분이 음수)가 나오는가

## 9. 평지 실차 테스트 절차

공통: 매 테스트 시작 전 `/odometry/diagnostics`의 `wheel_stale`/`imu_stale`이
둘 다 0인지, `level`이 `OK`인지 확인.

### Test 0 -- 센서 상태 확인 (~30초 정지)
확인: `/imu`, `/wheel/odom` 정상 수신(`ros2 topic hz`), `/odometry/diagnostics`
level=OK, `/tf`(`odom`->`base_link`) 정상, `/tf_static`
(`base_link`->`imu_link`, `base_link`->`camera_link`) 존재, 두 토픽 timestamp가
서로/wall clock과 크게 어긋나지 않는지.

### Test 1 -- 장시간 정지 (2~5분)
확인: `/odometry/filtered`의 x/y drift, `/wheel/odom` vx의 정지 노이즈 수준,
wz 노이즈, yaw drift, `Pxx/Pyy/Pyaw/Pvx/Pwz` 시간에 따른 거동. 가능하면 모터
enable 상태 변화(활성화 시점)도 같이 기록해서 이후 분석에서 구간을 나눌 수
있게 한다.

### Test 2 -- 모터 자기장 영향 (위치/자세 고정)
순서: motor OFF -> motor ON -> 구동계 활성화 -> (가능하면) 전류 증가 ->
motor OFF. 로봇은 움직이지 않아야 한다. 확인: AHRS yaw(`/imu`, `/odometry/filtered`의
yaw), `yaw_innovation`/`yaw_innovation_rejected`(diagnostics) -- 로봇이
안 움직이는데 yaw가 변하거나 innovation이 튀면 자기장 교란 의심.
raw magnetometer가 없어서(0번 참고) 직접 원인 분리는 안 되고, "정지 상태인데
yaw가 변했다"는 간접 증거만 남길 수 있다.

### Test 3 -- 평지 직선 주행 (가능하면 10m 이상, 저속 1회 + 중속 1회)
확인: lateral drift, heading drift, wheel scale consistency(=`/wheel/odom`
vx 적분 거리 vs 실측 거리), final position displacement.

### Test 4 -- 좌/우 회전 (평지, 정속)
확인: `wheel-derived wz`(`/wheel/odom` twist.angular.z, 또는
`/wheel/joint_states`로 직접 재계산) vs `estimated wz`(`/odometry/filtered`)
비교, yaw tracking, skid-steer slip 영향. **IMU 기반 wz는 없다**(위 1번
참고) -- 3자 비교가 아니라 2자(wheel vs EKF estimate) 비교가 된다는 점
유의.

### Test 5 -- 360도 회전 (제자리 또는 최소 반경, 좌/우 각각)
확인: 시작/종료 yaw 차이, 시작/종료 XY 차이, wheel wz vs EKF wz.

### Test 6 -- 사각형 또는 왕복 경로 (경사 테스트 대신 추가)
예: 직진 -> 90도 회전 -> 반복 -> 시작점 복귀, 또는 10m 직진 -> 180도 회전 ->
복귀. 확인: closed-loop position consistency(시작점 복귀 오차), heading
누적 오차, 좌/우 회전 비대칭.

### Test 7 -- 센서 dropout (안전할 때만, 생략 가능)
IMU 또는 wheel odom을 의도적으로 잠시 차단. 확인: `wheel_stale`/`imu_stale`
플래그 전환, `/odometry/filtered` pose jump 유무, 복구 거동.

## 10. 누락된 sensor source (요약)

- raw gyro (`angular_velocity`): 드라이버가 노출 안 함(위 0번)
- raw magnetometer: 드라이버가 노출 안 함(디바이스 자체 유무는 불명 -- 이
  드라이버로는 확인 불가)
- IMU 기반 wz: 위 이유로 없음 -- wz는 전적으로 wheel differential

## 11. 최종 확인된 구조 (실제 node/topic, `ros2 node list`/`ros2 launch --show-args`로 검증)

```text
Joystick(/dev/input/js0)
  |
  v
joy_node (pkg 'joy')                              --> /joy (sensor_msgs/Joy)
  |
  v
teleop_twist_joy_node (pkg 'teleop_twist_joy')     --> /cmd_vel (geometry_msgs/Twist)
  |                                                    (enable_button=L2 안 누르면 무발행)
  v
rmd_x8_driver_node
  |-- Motors (CAN 'can_drive', id 1/2)
  |-- /wheel/odom (nav_msgs/Odometry)
  \-- /wheel/joint_states (sensor_msgs/JointState)
           |
           v
      reduced_odom_node  <-- /imu (myahrs_driver_node)
           |-- /odometry/filtered (nav_msgs/Odometry)
           |-- /odometry/diagnostics (diagnostic_msgs/DiagnosticArray, 1Hz)
           \-- odom -> base_link TF (유일한 publisher)

stability_monitor_node (모터 STALE/ERROR 감지) --> /cmd_vel_safety (평상시 무발행)
static_transform_publisher x2                  --> /tf_static (base_link->imu_link, base_link->camera_link)
```

`can_driver_node`, `manual_joy_control_node`, Nav2/MPPI 관련 노드, 기존
`robot_localization`(`ekf_filter_node`)은 이 구조 어디에도 없다 -- §5의
`ros2 node list`/`ros2 topic info -v` 점검으로 실제 기동 시 확인할 것.

## 12. 답변 (질문 A~G)

**A. 왜 기존 manual stack에서는 reduced_odom 검증이 불가능했는가?**
`/wheel/odom`을 내는 노드가 `rmd_x8_driver_node` 하나뿐인데, manual
stack(`can_driver_node`)은 이 노드를 아예 안 띄우고 완전히 다른
프로토콜(`/motor_speed_cmd`, Float32MultiArray)로 모터를 직접 구동하기
때문이다. `can_driver_node`는 odometry를 전혀 publish하지 않는다(코드
확인, Odometry publisher 없음).

**B. `can_driver`와 `rmd_x8_driver`를 동시에 실행하면 실제 충돌 위험이
있는가? (코드 근거)**
있다. 둘 다 `can.interface.Bus(channel=..., bustype='socketcan')`로 같은
기본 채널(`'can_drive'`)에 소켓을 열고, 같은 모터 CAN ID(left=1/right=2,
0x141/0x142)로 속도 명령을 보낸다(-1번 섹션 코드 인용 참고). 소켓이 열리는
것 자체는 막히지 않지만, 두 프로세스가 같은 모터에 독립적으로 계속 명령을
경합시키는 실제 위험이 있다.

**C. `reduced_odom_bringup.launch.py`만으로 이미 필요한 sensor + driver + estimator를 실행할
수 있는가?**
그렇다(§3). `rmd_x8_driver_node` + `myahrs_driver_node` + static TF 2개 +
`reduced_odom_node` 5개 액션뿐이라, 이 파일은 수정 없이 그대로
`IncludeLaunchDescription`으로 재사용했다.

**D. 수동 joystick을 어떤 경로로 `/cmd_vel` 또는 실제 motor command 입력에
연결했는가?**
`joy_node`(표준 `joy` 패키지) -> `teleop_twist_joy_node`(표준
`teleop_twist_joy` 패키지) -> `/cmd_vel` -> `rmd_x8_driver_node`. 새 노드를
만들지 않고 이 환경에 이미 설치된 표준 패키지 2개만 조합했다.

**E. validation 중 motor command publisher는 정확히 몇 개인가?**
`/cmd_vel`에는 `teleop_twist_joy_node` 하나. `/cmd_vel_safety`에는
`stability_monitor_node` 하나(평상시 무발행, 모터 fault 시에만). §5의
`ros2 topic info -v`로 실제 기동 시 재확인 절차를 남겨뒀다.

**F. `odom -> base_link` TF publisher는 정확히 하나인가?**
그렇다 -- `reduced_odom_node` 하나뿐이다(`robot_localization`의
`ekf_filter_node`는 production에서 이미 빠졌고 이 validation launch에도
없음). §5에서 실제 기동 시 재확인하도록 절차를 남겼다.

**G. Nav2/MPPI 없이 reduced_odom 실차 bag을 정상적으로 수집할 수 있는가?**
그렇다 -- `reduced_odom_validation.launch.py` + 기존
`record_reduced_odom_validation.sh`만으로 §4에 정리한 전체 topic 집합을
수집할 수 있다. 실제 CAN/IMU/조이스틱 하드웨어가 없는 개발 환경이라 완전한
end-to-end 기동은 확인 못 했지만, `rmd_x8_driver_node`/`myahrs_driver_node`가
장치 접근 단계에서만 실패하고(예상된 결과, vcan0/실제 IMU 포트가 이
환경에 없어서) 나머지 6개 프로세스(`joy_node`, `teleop_twist_joy_node`,
`static_transform_publisher` x2, `reduced_odom_node`,
`stability_monitor_node`)는 전부 정상 기동하는 것까지 확인했다.
