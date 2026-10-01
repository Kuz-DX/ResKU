# Tag 1 측정 경로 자동실행

`ui_tag1_docking.launch.py`는 이제 `tag1_recorded_path_node.py`를 실행한다.
기존 `ui_tag1_docking_node.py` 진입점도 같은 노드로 연결된다.
이 모드는 **고정된 랙에서 검토한 관절 경로 재생**이다. AprilTag 인식·시각 서보로
랙 위치를 보정하는 모드가 아니며, 물리 체결 확인이나 planning scene 부착을 수행하지 않는다.
기존 HOME/BACK/태그 대기 자세 강제 이동, EE 목표 명령, 시간 기반 하강은 사용하지 않는다.

## 현재 상태

연결은 구현됐지만 현재 `config/tag1_recorded_path.yaml`은 실행 차단 상태다.
`executable`, `calibration_verified`, `execution_reviewed`가 false이고
`execution_sequence`가 비어 있다. 임의로 플래그만 true로 바꿔서는 안 된다.

이전 24개 측정점의 접근/복귀 경계 및 별도 체결 측정점과의 연결이 미확정이다.
기록의 bridge 영점과 현재 hardware 영점도 다르다. 변환한 일부 관절값이 제한을 넘는다.
최근의 음수 elbow `dock비슷` 측정 하나를 이전 양수 elbow 경로에 덮어쓰면 안 된다.
변경된 자세는 나머지 5축을 포함해 다시 확인해야 한다.

## 빌드 없는 읽기 전용 검사

작업공간 루트에서 실행한다. ROS 노드나 모터를 시작하지 않는다.

```bash
python3 src/arm/tool_manipulator_bringup/scripts/tag1_recorded_path.py \
  --record src/arm/tool_manipulator_bringup/config/tag1_recorded_path.yaml \
  --hardware src/arm/tool_manipulator_bringup/config/hardware.yaml \
  --tools src/arm/tool_manipulator_bringup/config/tools.yaml
```

현재는 `BLOCKED`와 원인을 출력하고 종료 코드 2를 반환하는 것이 정상이다.
PASS는 정적 관절 검사 통과일 뿐 충돌이나 체결 안전성을 보증하지 않는다.

## 경로 확정 규칙

- `execution_sequence`에서 각 측정점의 **1부터 시작하는 번호**를 명시한다.
  `waypoint_index`는 24개 원본, `docking_sample_index`는 별도의 체결 측정 목록을 참조한다.
- 단계는 `approach → dock → lock → retreat → return` 순서다.
  각 단계는 여러 점을 가질 수 있지만 `lock`은 한 번만 온다.
- 일반 항목 형식은 `{stage: approach, waypoint_index: N}` 또는
  `{stage: dock, docking_sample_index: N}`이다. `N`은 검토한 실제 번호여야 한다.
  잠금 항목은 `{stage: lock}`만 지정한다. **현재는 번호를 임의로 채우지 않았다.**
- 잠금은 직전 dock 자세의 다른 5축을 유지하고 `tools.yaml`의 tag 1 상대 yaw
  `+1.529379 rad`를 더한다. EE는 항상 제외한다.
- source calibration → raw 각도 → 현재 hardware calibration으로 변환한다.
  임의의 ±2π 보정, 리밋 클램핑, 영점 수정은 하지 않는다.
  캡처 이후 raw 다회전 카운터가 재설정됐다면 이 변환을 사용할 수 없다.
- 운영자가 라이브 컨트롤러와 파일 영점 일치, 엔코더 연속성, 단계 연결,
  구간 보간의 충돌 여유를 확인한 후에만 세 승인 플래그를 설정한다.
  이 노드에는 MoveIt 충돌 검사나 동적 장애물 회피가 없다.

## 노드 실행과 UI 연결

아래 명령 자체는 하드웨어를 시작하거나 팔을 움직이지 않는다.
현재 symlink-install에서는 새 스크립트를 소스 경로에서 실행하므로 ROS 빌드가 필요 없다.

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup ui_tag1_docking.launch.py
```

launch를 사용할 수 없으면 소스에서 직접 실행한다.

```bash
python3 src/arm/tool_manipulator_bringup/scripts/tag1_recorded_path_node.py \
  --ros-args --params-file src/arm/tool_manipulator_bringup/config/ui_tag1_docking.yaml
```

실제 재생에는 **동일한 hardware.yaml로 시작한 real_control의 arm_controller**가 필요하다.
읽기 전용 bridge는 포트를 공유하므로 real_control과 동시에 실행하지 않는다.
Servo, 조이스틱, 다른 경로 실행 노드도 중지해야 한다. 명령권 자동 중재는 구현하지 않았다.
팔은 검토된 첫 자세에서 정지해 있어야 한다. 노드는 자동 홈 이동을 하지 않는다.

UI의 `/selected_tool_id`에서 `1`을 받으면 전체 경로를 먼저 검사하고,
검사를 통과한 경우에만 팔 6축 FollowJointTrajectory 액션 **하나**를 보낸다.
최초 1 요청은 수락하며, 다음 실행은 다른 ID(통상 99)를 받은 후 새 1 요청이 필요하다.
`/ui_tag1_docking/status`에 `preflight`, `executing_recorded_path`,
`complete:trajectory_only`, 또는 이유를 포함한 `hold:...`를 발행한다.
`complete`는 관절 경로 완료일 뿐 물리 체결 성공 신호가 아니다.

`/tool_change/cancel`의 Bool true, 하드웨어 오류, 피드백 갱신 중단이나 현재 위치 제한
위반은 액션 취소를 요청한다. 취소는 하드웨어 비상정지를 대체하지 않는다.
HOLD에서 원인을 해결하고 액션 종료가 확인된 뒤 `/ui_tag1_docking/reset` Trigger를 호출한다.
진행 중이거나 수락/종료 여부가 불명확한 액션이 있으면 reset을 거부한다.

## 검증 범위

단위 테스트는 모터·ROS 노드를 생성하지 않고 실제 메시지 타입과 가짜 액션 클라이언트를
사용한다. 소스 문법 및 launch 구성 검사는 ROS 패키지 빌드가 아니다.
실제 controller/장비와의 동작 검증은 별도로 필요하다.
