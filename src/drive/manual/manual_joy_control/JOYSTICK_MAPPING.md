# 주행 조이스틱 매핑

현재 `manual_joy_control_node.py`의 기본 파라미터 기준이다.
버튼/축 번호는 0부터 시작하며 실제 `/joy` 출력과 다르면 장치 매핑을 확인한다.
이 문서는 차량 주행용이다. 팔 조작은 `src/arm/tool_manipulator_bringup/docs/manual_controller.md`를 참고한다.

| 조작 | ROS 입력 | 기능 |
|---|---|---|
| 왼쪽 스틱 상하 | axes[1] | 왼쪽 바퀴 속도 비례 조절 |
| 오른쪽 스틱 상하 | axes[4] | 오른쪽 바퀴 속도 비례 조절 |
| △ 세모 유지 | buttons[2] | 전진 |
| × 엑스 유지 | buttons[0] | 후진 |
| □ 네모 유지 | buttons[3] | 제자리 좌회전 |
| ○ 동그라미 유지 | buttons[1] | 제자리 우회전 |
| L1 누름 | buttons[4] | 왼쪽 설정 속도 +10dps |
| L2 누름 | buttons[6] | 왼쪽 설정 속도 −10dps |
| R1 누름 | buttons[5] | 오른쪽 설정 속도 +10dps |
| R2 누름 | buttons[7] | 오른쪽 설정 속도 −10dps |
| L3 유지 | buttons[11] | 차동 시험: 왼쪽 250 / 오른쪽 200dps |
| R3 유지 | buttons[12] | 차동 시험: 왼쪽 200 / 오른쪽 250dps |
| PS 누름 | buttons[10] | Record 요청 `/path/record` |
| Share 누름 | buttons[8] | Return 요청 `/path/return` |
| D-pad ↑ | axes[7] > 0.5 | 수동 랜드마크 촬영 `/path/landmark` |
| D-pad ↓ | axes[7] < −0.5 | 복귀 중 MANUAL 요청 |
| D-pad → | axes[6] < −0.5 | 복귀 중 AUTO 요청 |
| Options | buttons[9] | 팔 노드가 drive ↔ arm 포커스를 전환 |

기본 좌우 속도는 각각 200dps, 설정 범위 0–800dps, 스틱 deadzone은 0.05다.
속도 조정은 버튼을 새로 누를 때 한 단계씩 적용된다. 차동 시험은 설정 속도와 별개다.
주행 우선순위는 포커스 제한 → 차동 시험 → 전후진/회전 버튼 → 스틱 → 정지다.
전후진은 좌우 설정 속도를 각각 쓰므로 두 설정값이 다르면 직진하지 않을 수 있다.

## 포커스·미션 조건

- `/control/active_target=drive`일 때 주행 입력을 처리한다. arm 포커스에서는 0을 발행한다.
- Options는 주행 조이노드 자체가 처리하지 않으므로 팔의 포커스 관리 노드가 필요하다.
- 포커스를 drive로 바꾸는 것과 복귀 미션의 MANUAL/AUTO 전환은 별개다.
- Record는 odometry가 준비된 IDLE에서 시작한다.
- Return은 충분한 경로점이 쌓인 WAIT_RETURN_COMMAND에서 시작하며 실제 회전·복귀 주행을 시작한다.
- D-pad MANUAL/AUTO는 복귀 미션 상태에서 처리된다. 포커스 전환 시 D-pad를 중립으로 놓는다.
- D-pad ↑의 수동 촬영은 자동 키프레임과 별개이며 stop 모드에서는 잠시 주행을 hold한다.

## 실행·확인 (조이스틱이 연결된 노트북)

정상 ROS_DOMAIN_ID/DDS 환경을 적용한 각각 별도 터미널에서 실행한다.

```bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py
```

```bash
ros2 launch manual_joy_control manual_control.launch.py launch_joy:=false
```

`remote_joy` 하나가 /joy를 공급하므로 둘 다 계속 켜둔다. 다른 joy_node는 중복 실행하지 않는다.
주행 노드 출력은 `/motor_speed_cmd_manual`의 `[left_dps, right_dps]`다.
입력 /joy가 0.5초 끊기면 출력은 0이 되며, 이것은 모터 내부 watchdog과 별개다.

```bash
ros2 topic echo /joy
ros2 topic echo /control/active_target --once --qos-durability transient_local
ros2 topic echo /motor_speed_cmd_manual
ros2 topic echo /mission/return/state --once
```

위 확인 명령은 각각 실행하고 Ctrl+C로 끝낸다. 실제 버튼/스틱 조작은 차량을 움직일 수 있다.
