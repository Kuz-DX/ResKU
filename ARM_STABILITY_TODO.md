# Arm stability rework TODO

기준 브랜치: `rework/aa6614-arm-stability`  
기준 커밋: `aa6614cdee3c31d326314a5d2f6d1a3123be1f51`  
비교 대상 보존 브랜치: `main` (`bd87acb`)

## 현재 검증 범위

- 실제 하드웨어를 사용할 수 없으므로 이 브랜치에서는 실기 동작을 시도하지 않는다.
- 검증 근거는 다음 두 종류로 제한한다.
  1. mock/offline/MoveIt 시뮬레이션에서 새로 재현한 결과
  2. 저장소, Git 이력 또는 사용자가 제공한 기존 실기 오류 로그
- 기존 로그가 없는 물리 현상은 synthetic fault fixture로 가설만 검증하고
  `실기 미검증`으로 남긴다.
- 시뮬레이션 통과는 상태 전이, 오류 분류, 경로/충돌 검사의 통과를 뜻할 뿐
  모터 추종성, 전류 제한, 물리 충돌, 도킹 및 파지 성공을 뜻하지 않는다.
- 기존 로그에서 timestamp, command, feedback 또는 current가 빠진 경우 원인을
  확정하지 않고 `추정`으로 기록한다.

## 원칙

- ROS 패키지는 빌드하지 않는다.
- `aa6614` 이후 커밋은 통째로 cherry-pick하지 않는다. 필요한 변경은 원인과 테스트를
  확인한 뒤 최소 단위로 다시 작성한다.
- 한 단계가 합격하기 전에는 다음 단계의 기능을 추가하지 않는다.
- mock 합격, MoveIt 합격, 실기 합격을 서로 다른 결과로 기록한다.
- 오류 임계값을 완화하기 전에 command, feedback, velocity, current, epoch,
  execution ID를 통해 최초 원인을 확인한다.
- 물리적 도킹과 파지는 software/Arduino 완료 메시지만으로 성공 처리하지 않는다.

## 2026-10-31 채팅 작업 요약

현재 통합 기준은 `main`의 `c8bf074`다. 아래에서 **통합됨**은 이 커밋까지
저장된 변경이고, **작업 트리**는 아직 커밋되지 않은 변경이다.

### 통합됨 (`1a138ab..c8bf074`)

- [x] task ID를 `0=gripper`, `1=drill`, `2=supply_box`, `99=no tool`로 정리하고
  supply-box 작업에는 gripper 형상을 선택한다.
- [x] MoveIt/RSP에 `tool_id`와 `use_mesh`를 동일하게 전달한다. `99`에서 gripper
  finger, rack, spur gear가 생성되지 않는 정적 검사를 추가했다.
- [x] `use_mesh=false`에서도 collision geometry를 유지하고, RViz에는 collision
  STL을 일반 visual로 표시해 빨간 collision 렌더링과 실제 충돌 판정을 구분한다.
- [x] mock 초기 관절값을 HOME으로 바꾸고, 의도된
  `wrist_link_yaw <-> tool_spurgear` 맞물림만 SRDF `ToolMating`으로 제외했다.
- [x] HOME/BACK/TCP_READY named pose만 유지했다. BACK은 supply-box 반대쪽으로
  회전하도록 base 목표를 `-pi`로 정의했다.
- [x] tool ID 1 drill을 `ee_output_link`에 고정해 `ee_joint` 출력과 1:1로
  회전시키고, drill TCP/관성 중심을 로컬 Z축 기준으로 정리했다.
- [x] FK와 Jacobian을 한 체인 순회로 계산하고 TCP stepper의 SVD 중복 계산을
  제거했다. 관련 IK/TCP 회귀 테스트를 유지한다.
- [x] `plan_pose_goal.py`의 `Node.executor` 속성 충돌을 제거하고 MoveIt 오류 이름,
  목표 XYZ, planning time을 상태 메시지에 포함한다.
- [x] pose-goal 접근축을 TCP 로컬 `+Z`로 명시했다. 표면이 world XY에서 35도면
  접근축은 아래 방향 55도라는 정의를 문서와 설정에 반영했다.
- [x] rack 장착 접근과 rack 해체 후 최종 진입을 별도 planning-scene 상태로
  다뤄야 한다는 fail-closed 절차를 단계 7에 추가했다.

### 작업 트리 — 커밋 전

- [x] TCP 조이스틱 XY 평행이동을 `base_joint` yaw로 회전해 base가 +/-90도를
  넘어도 팔 기준 전방/좌우 의미가 유지되게 했다. `tcp_translation_frame:=base`
  로 기존 고정 프레임 동작을 비교할 수 있다.
- [x] L1을 deadman/현재 자세 재검증 전용으로 바꿨다. L1은 고정 TCP_READY
  자세 이동을 시작하지 않는다.
- [x] 현재 자세가 진입 조건을 만족하면 이동 없이 TCP_IK로 들어간다. 실패하면
  HOLD하고 이유를 보고하며, R2만 가장 가까운 검증된 진입 자세를 요청한다.
- [x] TCP entry/deadman/joystick 회귀 테스트 155개 통과, `git diff --check` 통과.
  ROS 패키지 빌드는 실행하지 않았다.

### 시뮬레이션/실측으로 확인된 제약

- [x] HOME 상태는 `/check_state_validity`: `valid=true`, `contacts=[]`로 확인했다.
- [x] tool0 중심 좌표 `[-0.394506, 0, 0.212964]`는 rack 장착 상태에서
  `rack_right <-> circuit_box`와 충돌한다. rack 제거 뒤에도 올바른 TCP +Z 아래
  55도 조건에서는 전 방위각에 bounded IK 해가 없으므로 실행 목표가 아니다.
- [x] tool1 중심 좌표 `[-0.317806, 0.090, 0.163912]`는 도달 가능한 45도 후보와
  +/-18.4 mm 면 오프셋을 검사해도 wrist/camera/EE와 circuit box가 충돌한다.
  중심점은 실행 목표가 아니며 `cartesian_targets.yaml`에서 비활성화했다.
- [x] 실기 캡처에서 shoulder/elbow가 runtime soft limit를 넘는 값이 반복됐다.
  도킹 좌표를 확정하기 전에 joint zero/sign/감속기 기준과 실제 soft limit를
  대조해야 하며, 단순 limit 확장은 금지한다.

### 다음 우선순위

- [ ] 실기에서 관절별 zero/sign과 soft limit를 저속으로 재검증하고, 허용 범위
  밖 캡처가 영점 불일치인지 실제 작업공간 요구인지 구분한다.
- [ ] 실제 도킹 자세에서 `base_actuator -> tcp_link`와 tool-specific TCP TF를
  동시에 기록하고, 물체 중심이 아닌 실제 접촉 TCP pose를 확정한다.
- [ ] 확정된 dock pose에서 30--50 mm 떨어진 pre-dock과 저속 직선 최종 진입을
  분리해 MoveIt/IK/collision 검사를 수행한다.
- [ ] rack 해체 완료 신호와 planning-scene 형상 전이를 구현한다. 해체 전 계획은
  폐기하고 현재 feedback에서 최종 경로를 다시 계획한다.
- [ ] drill 회전 중심/방향과 `ee_gear` 1:1 맞물림을 RViz 및 실물 기준으로 다시
  확인한다. 현재 STL 원점의 X 편심 약 1.47 mm도 구분해 기록한다.
- [ ] 최신 작업 트리 변경을 커밋하기 전 mock에서 base 0, +/-90, +/-180도 부근의
  조이스틱 방향과 L1 무이동 진입/R2 명시 이동을 확인한다.

## 단계 0 — 기준선 고정과 재현 자료

- [x] 현재 `main`을 보존하고 `aa6614` 기준 재작업 브랜치를 생성한다.
- [x] `aa6614`의 `test_arm_mode_core.py`: 43 passed (2026-10-31, 빌드 없음).
- [ ] 저장소와 Git 이력에서 반복 오류의 기존 근거를 수집한다: DXL 정렬,
  RMD direction mismatch, following error, waypoint stall/timeout, HOME 충돌,
  docking deadman/ACK.
- [ ] 원본 로그가 없는 오류는 관련 커밋 diff·문서·테스트를 `간접 근거`로
  분류하고 synthetic fault fixture로 재현한다.
- [ ] 각 로그에 최초 오류와 후속 오류를 구분한다.
- [ ] 공통 재현 레코드 형식을 정의한다: timestamp, stage, epoch, execution ID,
  command/position/velocity/current, controller/hardware status, Joy, MoveIt 결과.

합격 조건: 각 반복 오류에 `기존 원본 로그`, `간접 근거+fixture`, `근거 없음` 중
하나가 표시되고, 근거가 있는 오류에는 최소 한 개의 재현 fixture가 있다.

## 단계 1 — 상태 머신만 안정화

- [ ] HOLD, MANUAL, TCP_IK, TOOL_AUTO, FAULT 전이를 표로 고정한다.
- [ ] manager만 raw motor command를 발행하는지 테스트한다.
- [ ] mode 변경, cancel, focus 변경마다 epoch가 갱신되고 늦은 명령이 거부되는지
  테스트한다.
- [ ] 최초 실패 원인이 후속 stale/inactive 오류로 덮이지 않게 한다.
- [ ] controller 미준비, stale feedback, 잘못된 joint count/NaN을 주입한다.
- [ ] fault reset은 controller healthy + fresh + stopped 조건에서만 허용한다.

합격 조건: 순수 Python 테스트로 모든 전이와 실패 주입이 결정적으로 재현된다.

## 단계 2 — mock plant 추가

- [ ] 명령을 즉시 feedback으로 복사하지 않는 관절 mock을 만든다.
- [ ] 속도/가속도 제한, 1차 지연, encoder quantization, 정지 잔차를 지원한다.
- [ ] 관절별 stall, feedback dropout, 지연, overshoot를 주입할 수 있게 한다.
- [ ] following error, stall, timeout이 서로 구분되는지 확인한다.
- [ ] 목표 반전 중 아직 이전 방향으로 움직이는 정상 지연을 direction fault로
  오인하지 않는 테스트를 작성한다.
- [ ] mock 결과로 임계값을 바꾸지 말고, 측정 가능한 의미와 단위를 먼저 고정한다.

합격 조건: 같은 seed와 fixture에서 동일한 최초 실패 원인이 나온다.

## 단계 3 — 시작 복구와 관절 한계

- [ ] 정상 운전 soft limit과 물리적으로 확인된 startup safe endpoint를 분리한다.
- [ ] 부호, offset, raw 값과 joint rad 변환을 관절별 fixture로 검증한다.
- [ ] soft limit 밖에서 안쪽으로 복구하는 경우만 제한적으로 허용한다.
- [ ] 실측 속도에서 안전 끝점 전 정지가 불가능하면 복구를 시작하지 않는다.
- [ ] 시작 복구 중 producer 명령, mode 진입, cancel 동작을 검증한다.
- [ ] DXL zero 정렬은 명령 도달과 실측 정지 잔차를 별도 조건으로 판정한다.

합격 조건: 정상 범위, 허용 복구 범위, 물리 범위 밖의 세 경우가 테스트에서
명확히 분리된다.

## 단계 4 — 경로 실행기

- [ ] 단일 짧은 관절 경로만 사용해 command-feedback pacing을 검증한다.
- [ ] acceleration, jerk, feedback lead, final settling의 책임을 분리한다.
- [ ] 경로 중간 통과와 최종 waypoint 정지를 구분한다.
- [ ] stall 판정에 목표 방향 진행량과 command lead를 모두 사용한다.
- [ ] cancel/failure 후 현재 실측 위치에서 HOLD하는지 확인한다.
- [ ] nearest suffix 재개는 순수 관절 경로에만 적용하고 작업 상태가 있는
  docking/pick-place에는 자동 적용하지 않는다.

합격 조건: 정상 지연, 실제 stall, feedback 단절이 각기 다른 이유로 종료된다.

## 단계 5 — MoveIt planning scene

- [x] Python task ID를 `0=gripper, 1=drill, 2=supply_box`로 통일하고 supply-box
  작업에는 실제 장착되는 gripper collision 형상을 사용한다.
- [ ] chassis, circuit box, camera/bracket, rack, tool 형상을 scene에 포함한다.
- [ ] SRDF allowed-collision matrix를 최소 권한으로 검토한다.
- [ ] HOME은 고정 joint 직선이 아니라 현재 실측 자세에서 계획한다.
- [ ] 현재 자세→기록 경로 진입, 전체 기록 경로, 귀환 경로를 각각 검사한다.
- [ ] 정상 시작뿐 아니라 경로 중단 자세와 limit 인접 자세에서도 계획한다.
- [ ] sampled state validity 간격과 검증 예산을 정하고 작은 장애물/swept-volume
  한계를 문서화한다.
- [ ] 계획 중 feedback, epoch, ownership이 바뀌면 결과를 폐기한다.

합격 조건: 각 시작 자세와 경로에 plan 결과, collision 결과, 실패 지점이 남는다.

## 단계 6 — TCP joystick과 deadman

- [ ] fresh Joy, release-to-rearm, L1 release, timeout을 독립 테스트한다.
- [ ] collision service 대기 중 적분 catch-up이 발생하지 않게 한다.
- [ ] collision 승인 전 후보 자세를 commit하지 않는다.
- [ ] R2의 의미를 `현재 정지 자세 TCP 진입 검사` 또는 `TCP-ready 이동` 중 하나로
  고정하고 UI/문서/테스트를 일치시킨다.
- [ ] HOME, L1, L2 조합이 동시에 두 workflow를 시작하지 않는지 확인한다.

합격 조건: 오래된 service 응답이나 빠른 재입력으로 motion이 재시작되지 않는다.

## 단계 7 — 도킹 workflow

- [ ] taught path 완료, tag 관측, 보정 이동, Arduino 구동, 물리 확인을 별도
  stage로 만든다.
- [ ] rack gear가 장착된 접근 단계와 해체된 최종 도킹 단계를 서로 다른
  planning-scene 상태로 모델링한다. 해체 확인 전에는 rack collision을 유지한다.
- [ ] rack 해체 완료 신호는 sequence ID와 timestamp를 포함하고, 누락되거나 오래된
  신호이면 다음 계획을 금지한다.
- [ ] 해체 확인 뒤 현재 joint feedback에서 planning scene을 갱신하고 경로를 새로
  계획한다. 해체 전 생성한 최종 도킹 trajectory는 재사용하지 않는다.
- [ ] `rack_right`와 `circuit_box`의 allowed-collision을 전역 허용하지 않는다.
  mock에서는 `attached -> detach confirmed -> removed/world object` 전이와 각 단계의
  collision 결과를 각각 기록한다.
- [ ] L2 deadman과 UI 요청의 정책을 분리한다.
- [ ] Joy heartbeat stale과 실제 L2 release를 다른 오류로 보고한다.
- [ ] Arduino 명령 전 subscriber와 정지 HOLD를 확인한다.
- [ ] 명령 이전의 오래된 ACK/상태로 성공하지 않게 sequence를 검증한다.
- [ ] timeout/cancel 뒤 Arduino가 계속 움직일 수 있음을 상태에 표시한다.
- [ ] release path가 없는 tool은 UI release를 명시적으로 거부한다.

합격 조건: software path 완료, Arduino 완료, 물리 도킹 확인이 혼동되지 않는다.

## 단계 8 — 실기 가능 시 수행할 보류 항목

현재 작업 범위에서는 아래 항목을 실행하거나 완료 처리하지 않는다. 시뮬레이션 결과와
기존 로그로 값을 확정할 수 없는 이유를 남긴 뒤, 하드웨어 사용 가능 시 별도 승인과
현장 안전 절차에 따라 수행한다.

- [ ] 관절별 부호, 영점, soft limit, safe endpoint를 저속으로 확인한다.
- [ ] 한 관절씩 ramp 응답과 command-feedback-current를 기록한다.
- [ ] 무부하 HOME/TCP-ready를 저속으로 실행한다.
- [ ] tool path를 tag/Arduino continuation 없이 실행한다.
- [ ] tag 보정을 preview-only로 검증한다.
- [ ] Arduino 구동을 정지한 팔에서 별도로 검증한다.
- [ ] collision object와 실제 여유 거리 차이를 기록한다.
- [ ] 실측 자료로만 following-error, stall, timeout, speed/acceleration 값을 확정한다.

보류 해제 조건: 실제 하드웨어 사용 가능, 현장 안전 절차 확인, 관절별 저속 시험 승인.

## 현재 확인된 증거 위치

- `src/arm/tool_manipulator_bringup/docs/dxl_trajectory_diagnostics.md`:
  DXL stall 관련 기존 관측과 미확정 가설
- `src/arm/tool_manipulator_bringup/docs/tcp_ik_stage4_benchmark.json`:
  feedback 0.3 응답을 사용한 offline IK benchmark; 실제 collision timing 아님
- `src/arm/tool_manipulator_bringup/docs/tcp_ready_evaluation.json`:
  자세/진입 평가 결과; `collision_free: null` 항목은 미검증
- `src/arm/tool_change_min/docs/changer_ros_mock_results.json`:
  tool changer ROS mock 결과
- `src/arm/tool_manipulator_bringup/test/`:
  timeout, stale feedback, collision rejection, deadman 등을 재현하는 기존 fixture
- `main`의 `hold_recovery_runbook.md` 및 `control_mode_manager.md`:
  후속 실기 오류 문구와 대응 기록. 원본 runtime log가 아니므로 간접 근거로만 사용

현재 저장소 검색에서는 arm runtime의 원본 `.log` 또는 rosbag을 찾지 못했다. 따라서
원본 실기 로그가 별도로 제공되지 않는 한 커밋 diff와 문서에 적힌 수치만으로 물리적
원인을 확정하지 않는다.

### 초기 증거 매트릭스

| 오류군 | 현재 근거 | 분류 | 이 브랜치에서 할 일 |
| --- | --- | --- | --- |
| DXL 정렬 잔차/timeout | 후속 커밋의 허용 오차·진단 문자열 변경, 단위 테스트 | 간접 근거 | quantization·정지 잔차 fixture |
| RMD direction mismatch | `78e4a25` diff와 watchdog 테스트 | 간접 근거 | setpoint 반전+feedback 지연 fixture |
| following error | manager 코드·테스트와 후속 runbook | 간접 근거 | 정상 지연과 실제 stall을 분리하는 mock plant |
| waypoint stall/timeout | trajectory 진단 문서·테스트 | 간접 근거 | 진행량·lead·dropout fault injection |
| HOME circuit-box 충돌 | `13b26bc` diff와 후속 문서 | 간접 근거 | 동일 URDF/SRDF의 MoveIt planning fixture |
| docking L2 release/stale | deadman 코드·테스트와 후속 runbook | 간접 근거 | raw Joy sequence와 heartbeat fixture |
| Arduino ACK/timeout | bridge/coordinator 코드와 changer mock 결과 | mock 근거 | 오래된 ACK, ERROR, no-response sequence fixture |
| 실제 체결·접촉·전류 | 원본 로그 및 센서 자료 없음 | 근거 없음 | 시뮬레이션 합격 대상에서 제외, 실기 미검증 표시 |

## 이후 커밋 참고 지도

아래 커밋은 정답으로 cherry-pick하지 않고 회귀 사례와 아이디어만 참고한다.

- `378ae43`, `9629d86`, `42e4f45`: DXL 정렬과 fault reset
- `78e4a25`: RMD direction mismatch/CAN timestamp
- `1fd1dc2`, `13b26bc`: HOME workflow와 MoveIt 계획
- `554c999`, `46249ba`, `c1646fc`, `c980682`, `e76ce55`: 경로 속도/재표본화
- `c74bfef`, `3adbcf2`, `39f444c`, `0469a3b`: docking/Arduino/deadman
- `710cb7e`, `8c6a9f7`, `d3aa5f0`: startup safe endpoint와 soft limit
- `bd87acb`: 현재 자세 TCP 진입 의미

