# robot_arm_bringup

`army_manipulator` URDF와 하나의 `ros2_control_node`를 사용하는 수동 미션용
로봇팔 bringup입니다. AUTO 미션은 별도 launch로 분리하며, 이 launch에서는
MoveIt, `maru_ik_node`, AUTO trajectory controller를 실행하지 않습니다.

```bash
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py
# Drive가 joy_node를 이미 실행한 경우
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py launch_joy:=false
# 하드웨어 없이 검증
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py use_mock_hardware:=true
```

## 원격 PC 조이스틱 소유권

조이스틱이 로봇 PC가 아닌 원격 PC에 연결된 구성에서는
`joy_node`도 원격 PC에서 실행해야 합니다. 두 PC에는 동일하게
`ROS_DOMAIN_ID=99`, `ROS_LOCALHOST_ONLY=0`을 설정합니다.

Arm만 실행할 때:

```bash
# 원격 PC: /joy의 유일한 publisher
ros2 launch robot_arm_bringup remote_joy.launch.py

# 로봇 PC: 원격 /joy를 구독하므로 로컬 joy_node는 끄기
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py launch_joy:=false
```

Drive와 Arm을 동시에 실행할 때:

```bash
# 원격 PC: Drive가 joy_node 소유
ros2 launch manual_joy_control manual_control.launch.py

# 로봇 PC: Arm은 /joy 구독만 함
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py launch_joy:=false
```

Drive의 `manual_control.launch.py`와 Arm의 `remote_joy.launch.py`를 동시에
실행하지 않습니다. 정상 상태에서 `ros2 topic info /joy -v`의
publisher count는 1입니다.

## 수동 모드 전환

```text
DRIVE(OFF) <--- 9번 ---> MANUAL_EE
                          |
                          | 세모 toggle
                          v
                     MANUAL_100
```

- Drive에서 9번 버튼을 누르면 항상 기본 허브인 `MANUAL_EE`로 진입합니다.
- `MANUAL_EE`에서 세모 버튼을 누르면 `MANUAL_100`, 다시 세모를 누르면
  `MANUAL_EE`로 돌아옵니다.
- 9번 버튼은 `OFF`와 `MANUAL_EE` 사이에서만 동작합니다. DRIVE로 복귀하려면
  반드시 먼저 `MANUAL_EE`로 전환해야 합니다.
- 기존 AUTO 전환용 8번 버튼은 이 수동 launch에서 비활성화되어 있습니다.
- `/control/active_target`은 Drive와의 기존 `drive`/`arm` 계약을 유지합니다.

## 단일 하드웨어와 controller 소유권

관절명은 MoveIt 모델과 동일합니다.

```text
base_joint, shoulder_joint, elbow_joint, wrist_joint, gripper_joint
```

하나의 controller manager가 수동 제어용 controller를 로드합니다.

| 모드 | 활성 controller |
|---|---|---|
| OFF/MANUAL_EE/MANUAL_100 | `position_controller` |

AUTO용 `arm_controller`와 `gripper_controller`는 이 launch에서 로드하지 않습니다.

## MANUAL_100

```text
gamepad_position_controller -> /manual_joint_commands --┐
manual_gripper_controller   -> /manual_gripper_command --┤
joint_command_mux -> /position_controller/commands ------┘
```

`joint_command_mux`는 AUTO 외의 모드에서만 발행합니다. `MANUAL_EE` 진입 시에는
실측 자세를 그대로 유지합니다. Circle을 누르면 버튼 순간 shoulder
실측값을 기준으로 카메라 거치대 충돌 회피 순서를 선택합니다.

- `shoulder > +0.03 rad`: shoulder `+30°` → wrist가 `+75°` 미만일 때만
  `+75°`까지 이동(이미 `+75°` 이상이면 현재각 홀드) → wrist 현재각을 유지하며
  elbow home → base `0` → shoulder home → 마지막에 wrist home (`+1.51844 rad`)
- `shoulder <= +0.03 rad`: wrist home → elbow home → shoulder home → base `0`

정식 MANUAL_EE home은 base `0`, shoulder `-0.55851`, elbow `+1.58825`,
wrist `+1.51844 rad`입니다. 이와 별개인 운반용 hold pose는 base `0`,
shoulder `-0.76044`, elbow `+1.63830`, wrist `+1.53290 rad`이며 그리퍼는
마지막 파지 위치를 유지합니다. 각 단계에서 목표과 `0.03 rad` 이내인
관절은 이동을 생략하고, 이동하지 않는 관절은 단계 시작 실측값으로
고정합니다. 목표 `±0.02 rad`에서 0.3초를 유지해야 다음 단계로
넘어가며, 단계가 60초를 넘으면 실측 자세를 홀드하고 EE 조작을 계속
차단합니다. 전체 복귀 완료 후에만 새 EE 입력을 적용합니다.
`MANUAL_100`은 별도 home 이동 없이 바로 조작할 수 있습니다.

MANUAL_EE에서 Circle은 home pose, Square는 운반용 hold pose 이동을
시작합니다. PS 버튼은 현재 자세 일시정지/재개 토글이며 home/hold 이동
중에도 동작합니다.

| 기능 | 조작 |
|---|---|
| shoulder +/- | 오른쪽 스틱 위/아래 |
| elbow +/- | 왼쪽 스틱 위/아래 |
| wrist +/- | 십자키 위/아래 |
| base +/- | R1/L1 |
| gripper | O/네모 |

## MANUAL_EE

`ee_manual_controller`는 조이스틱 입력을 `base_actuator` 기준 TCP 병진 속도로
해석하고, URDF에서 계산한 위치 Jacobian의 damped least-squares 역해를 통해
`/manual_ee_joint_commands` 관절 목표로 변환합니다. 오른쪽 스틱 상하는 현재
베이스 방향 기준 전진/후진, 왼쪽 스틱 상하는 수직 상승/하강을 명령합니다.
L1/R1은 베이스를 회전합니다. 관절 제한과 최종 속도 제한은 EE 노드와
`joint_command_mux`에서 각각 적용합니다.

진입 중에는 `joint_command_mux`가 home 이동을 독점하며
`/control/manual_ee_ready`를 false로 유지합니다. Home 도달 후 이 신호가 true가
되면 EE 노드는 최신 `/joint_states`에서 IK 목표를 다시 초기화한 뒤 조작 명령을
발행합니다.

MANUAL_EE 노드는 기본 bringup에서 활성화되어 있습니다. 실물 구동 전에는 반드시
낮은 속도에서 joystick 각 축과 실제 `wrist_link` 이동 방향이 일치하는지 확인합니다.

현재 MANUAL_EE는 독립 differential IK 노드에서 position 목표를 생성합니다.
AUTO의 기존 상위제어 체계를 MANUAL_EE와 공용화하려면 `maru_ik_node`의 입력과
MoveIt 실행 출력을 분리하는 상위제어 수정이 필요하므로, 해당 구조 변경은 보류되어
있습니다. AUTO 상위제어 코드는 변경하지 않았습니다.

## AUTO (별도 launch로 분리 예정)

```text
/arm/target_point
  -> maru_ik_node
  -> MoveIt move_group
  -> arm_controller / gripper_controller
  -> ros2_control hardware
```

위 파이프라인은 기존 설계 참고용입니다. 현재 대표 수동 launch에서는 실행되지
않으며, 카메라 검출을 포함한 AUTO 미션 launch가 별도로 소유해야 합니다.

실물 운용 전에는 controller 목록과 상태를 확인합니다.

```bash
ros2 control list_controllers
ros2 topic echo /control/mode
ros2 topic echo /control/active_target
```
