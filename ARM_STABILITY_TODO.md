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

- [x] BACK 준비 상태를 과거 완료 latch/epoch가 아니라 신선한 정지 HOLD 피드백과
  현재 BACK 자세 오차로 재검증한다. 움직임 전 tool 목표/계획 실패 뒤에는 BACK을
  반복하지 않고 tool0/1/2를 다시 선택하며, 이미 BACK에서 L2+□를 눌러도 무이동으로
  성공 처리한다.
- [x] TCP 조이스틱 XY 평행이동을 `base_joint` yaw로 회전해 base가 +/-90도를
  넘어도 팔 기준 전방/좌우 의미가 유지되게 했다. `tcp_translation_frame:=base`
  로 기존 고정 프레임 동작을 비교할 수 있다.
- [x] L1을 deadman/현재 자세 재검증 전용으로 바꿨다. L1은 고정 TCP_READY
  자세 이동을 시작하지 않는다.
- [x] 현재 자세가 진입 조건을 만족하면 이동 없이 TCP_IK로 들어간다. 실패하면
  HOLD하고 이유를 보고한다. R2는 현재 정지 자세를 다시 검사할 뿐 이동을 요청하지
  않으며, 자세 이동은 명시적인 `/arm/go_tcp_ready`만 소유한다.
- [x] TCP entry/deadman/joystick 회귀 테스트 155개 통과, `git diff --check` 통과.
  ROS 패키지 빌드는 실행하지 않았다.

### 시뮬레이션/실측으로 확인된 제약

- [x] HOME 상태는 `/check_state_validity`: `valid=true`, `contacts=[]`로 확인했다.
- [x] tool1 장착 모델의 HOME 자세를 유지한 base 직선 스윕을 0.01 rad 이하 간격으로
  비교했다. `+pi`는 316개 중 60개가 invalid이고 circuit/supply box 양쪽과
  충돌했으며, `-pi`도 60개가 invalid이지만 circuit box 접촉만 발생했다. 따라서
  BACK은 `-pi`를 유지하되, 어느 방향도 base 단독 직선 회전 경로로 실행하지 않고
  MoveIt 회피 계획과 전체 경로 collision 검사를 계속 요구한다. tool1 고정 장착면의
  `tool_2_link <-> ee_actuator` 접촉은 SRDF `ToolMating`으로 제외했다.
- [x] gripper 장착 planning scene에서 BACK부터 실측 tool0/tool2 joint goal까지
  MoveIt 계획을 재현했다. 두 목표 모두 goal state 자체가 invalid여서 계획되지 않았다.
  tool0은 `rack_right <-> tool_case_1`, tool2는 `rack_left <-> supply_box` 접촉이다.
  이를 전역 allowed-collision로 숨기지 않고, 각 목표가 접촉 전 pre-dock인지 실제
  mating/grasp 자세인지 정의한 뒤 planning-scene 단계 전이 또는 새 여유 자세로 해결한다.
- [x] 실측 goal의 의미를 확정했다. tool0 goal은 기어가 맞물린 최종 자세이고,
  제거 workflow는 `접촉 전 pre-dock -> EE 회전/rack 제거 -> 제거 상태 확인 -> 현재
  feedback에서 직선 접근 재계획 -> 최종 기어 맞물림` 순서다. tool2 goal은 박스에
  닿은 grasp 자세다. 둘 다 일반 collision-free pre-dock 목표가 아니므로 별도 진입
  목표와 scene 전이가 구현될 때까지 `cartesian_targets.yaml executable: false`로
  fail-closed 처리했다.
- [x] tool0의 접촉 전 pre-dock을 최종점의 TCP local `-Z` 30 mm로 정의했다.
  rack 제거 EE 명령은 아래 실측 명세를 기록했지만 전용 실행/feedback 판정이 아직
  구현되지 않았으므로 실행하지 않는다. tool2 grasp 단계의
  의도된 접촉은 `rack_left <-> supply_box`와 `rack_right <-> supply_box` 양쪽이다.
  이 두 쌍은 grasp 단계에서만 제한적으로 허용하고 전역 SRDF에는 추가하지 않는다.
- [x] 사용자가 tool0 `[-0.394506, 0, 0.212964]`, 표면 35도와 tool1
  `[-0.317806, 0.090, 0.163912]`, 표면 55도를 `arm_world` 실제 pre-dock으로
  확정했다. L2 경로는 YAML 마지막 관절점 대신 이 pose의 bounded axis IK를 거쳐
  MoveIt `현재 -> home -> back -> pre-dock` 세 leg를 계획하도록 변경했다.
- [x] tool1 `[-0.317806, 0.090, 0.163912] m`는 도킹 원 중심과 최종
  `tcp_link`가 일치해야 하는 동일한 작업점으로 정의했다. CAD/STL 배치 계산에서 나온
  `[-0.255506, 0.080600, 0.172229] m`는 작업 의미가 없는 mesh reference로 분리하고,
  시각/collision mesh 위치를 바꾸지 않는 별도 `tool_case_y_plus_dock_point` frame을
  추가했다.
- [x] 기존 axis IK/BACK limit 항목을 현재 모델 기준으로 재검증했다. tool0/tool1의
  물체 기준 좌표는 TCP IK 목표가 아니며 런타임은 실측 joint goal을 사용하므로 seed
  80개 axis IK 실패를 실행 blocker로 사용하지 않는다. SRDF BACK shoulder는
  `1.358215 rad`로 hardware 상한 `1.4622368 rad`보다 약 `0.104 rad` 안쪽이고,
  HOME/BACK/TCP_READY 전체가 0.02 rad planning margin 안임을 정적 테스트로 고정했다.
  tool1 실측 elbow `1.581792 rad`는 planning 상한 `1.557952 rad`를 넘으므로 새 자세를
  측정할 때까지 `cartesian_targets.yaml executable: false`로 fail-closed 처리한다.
- [x] 실기 캡처에서 shoulder/elbow가 runtime soft limit를 넘는 값이 반복됐다.
  도킹 좌표를 확정하기 전에 joint zero/sign/감속기 기준과 실제 soft limit를
  대조해야 하며, 단순 limit 확장은 금지한다.

### 다음 우선순위

- [x] soft limit 밖이지만 5도 recovery envelope 안인 RMD가 부팅 직후 바깥 방향
  속도로 추정돼 일반 limit guard가 복구 전에 fault를 latch하던 순서를 수정했다.
  startup 중에는 envelope/following-error/stall 감시가 담당하고, 완료 후 일반
  outward guard가 즉시 다시 활성화된다.
- [x] 실측 tool0/tool2 끝점을 MoveIt joint goal로 사용하고, BACK 기준 가장 가까운
  base `2π` 동치각을 선택하도록 했다. 저장 경로 직접 재생은 production에 합치지 않았다.
- [ ] tool1 끝점의 elbow `1.581792 rad`를 planning 상한 `1.557952 rad` 이하로
  다시 측정한다. 자동 clamp는 TCP pose를 바꾸므로 금지한다.
- [x] tag/TCP 축 일치와 tag→TCP 오프셋을 기록했다: tool0
  `[-45.7, 0, -44.5] mm`, tool1 `[-45.7, 0, -50.0] mm`.
- [ ] tool1의 기존 camera-normal 30 mm 전진을 X 45.7 mm 오프셋까지 사용하는
  tag-relative coarse/fine/dock 단계로 교체하고 mock collision 회귀를 통과시킨다.

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
- [x] 최신 작업 트리 변경을 커밋하기 전 mock에서 base 0, +/-90, +/-180도 부근의
  조이스틱 방향과 L1 무이동 진입/R2 명시 이동을 확인했다. 방향 회귀는 0,
  +/-90, +/-179, +/-180도를 포함한다. managed mock에서 L1 전후 관절값과
  execution ID가 변하지 않았고, margin 밖 별도 seed에서 R2가 shoulder를
  `1.400000 -> 1.3612368 rad`로 이동해 `last_execution_result=succeeded`를
  확인했다.

## Tool별 완성 계획

현재 예상 완성 순서는 `tool0 -> tool1 -> tool2`다. 세 tool 모두 mock/MoveIt 통과와
실기 완료를 구분한다. 실측 TCP, payload, 전류 또는 물리 체결 근거가 없는 항목은
software 상태나 Arduino Bool만으로 완료 처리하지 않는다.

### Tool0 — gripper 도킹

현재 실측 joint goal은 joint limit 안이지만 pre-dock이 아니라 기어가 맞물린 최종
자세다. tag 0 크기와 tag-to-TCP 오프셋, 체결 뒤 TCP `-Z` 30 mm 후퇴와 BACK 복귀
흐름은 정의돼 있다. 다음 항목을 마쳐 첫 번째 완성 대상으로 삼는다.

- [ ] rack 접촉이 없는 별도 pre-dock 자세를 측정하거나 tag/TCP 관계로 계산하고
  전체 MoveIt 경로 collision을 통과시킨다.
- [ ] pre-dock 정지 뒤 EE를 회전해 rack을 제거하고, 목표 위치·전류 또는 별도 센서로
  제거 완료를 확인한다. EE 명령 완료만으로 제거 성공 처리하지 않는다.
- [x] rack 제거 EE 명령을 양의 `ee_joint`, gripper에서 모터를 바라본 기준 CCW,
  `5 rad/s`로 최대 2초로 확정했다. 기대 변위는 10 rad이고 10% 완료 허용오차를
  적용한 성공 조건은 시작점부터 양의 방향으로 unwrap해 누적한 `ee_joint` 변위
  `>= 9.0 rad`이다. Dynamixel Wizard Velocity Limit는 `16.8 rad/s`로 설정했다.
  2초 경과만으로는 성공 처리하지 않는다.
- [ ] rack 제거 전용 manager-owned 속도 명령과 unwrap 누적 feedback 판정을 구현하고,
  2초 안에 9.0 rad에 도달하지 못하거나 역방향/feedback 정지가 발생하면 정지·실패
  처리한다. 일반 tool-auto 위치 명령의 `0.12 rad/s` 한계는 올리지 않는다.
- [ ] rack 제거 확인 뒤 planning scene과 현재 joint/EE feedback을 갱신하고, 제거 전
  생성한 trajectory를 폐기한다.
- [ ] 갱신된 현재 상태에서 최종 기어 맞물림까지 직선 TCP 접근을 새로 IK/충돌
  검사한다. 최종 맞물림 접촉만 해당 단계에서 제한적으로 모델링한다.
- [ ] `tool_docking.yaml`의 tag-relative `coarse`, `fine`, `dock` TCP pose를 실제
  측정값으로 채운다. null pose나 카메라 중심을 목표로 대체하지 않는다.
- [ ] 현재 camera-normal 30 mm 전진 대신 측정된 X/Z 오프셋
  `[-45.7, 0, -44.5] mm`를 모두 사용하는 단계별 접근을 적용한다.
- [ ] 각 단계의 IK, joint limit, 전체 보간 경로 collision과 최종 정지 오차를
  mock에서 검증한다.
- [ ] rack 장착 접근과 해체 확인 뒤의 최종 도킹을 서로 다른 planning-scene
  상태로 모델링하고, 해체 뒤 현재 feedback에서 다시 계획한다.
- [ ] 체결 명령에는 sequence ID와 timestamp를 추가해 명령 이전 ACK를 거부한다.
- [ ] `/arm/docked`와 별도로 물리 체결을 확인할 센서 또는 명시적인 검증 절차를
  정의한다.
- [ ] tool0 장착 상태의 `HOME -> BACK -> pre-dock -> dock -> retreat -> BACK`에서
  command/feedback/velocity/current와 최초 실패 원인을 기록한다.
- [ ] tag 유실, 취소, collision, Arduino timeout에서 현재 실측 위치 HOLD와
  firmware watchdog 동작을 실기에서 확인한다.

Tool0 완료 조건: 측정된 3단계 TCP 목표, 장착 상태별 scene 전이, 전체 경로 collision
통과, 새로운 체결 ACK와 물리 체결 확인, 전류를 포함한 저속 실기 왕복 기록이 모두 있다.

### Tool1 — drill 도킹 및 구동

tool1 drill 형상과 TCP는 모델에 포함돼 있고 고정 장착면
`tool_2_link <-> ee_actuator`만 `ToolMating`으로 제외했다. `-pi` BACK은 `+pi`보다
supply-box collision이 적으므로 유지한다. 그러나 현재 실측 pre-dock/final capture의
elbow가 실행 한계를 넘으므로 fail-closed 상태를 유지한다.

- [ ] elbow를 planning margin 상한 `1.557952 rad` 이하로 두면서 tag 가시성,
  접근축과 기계 간극을 만족하는 pre-dock/final pose를 다시 측정한다.
- [ ] 후보 `1.581792 rad`와 final capture `1.590693 rad`의 차이를 raw feedback,
  zero/sign 및 캡처 시점과 함께 정리한다. 자동 clamp나 limit 확장은 금지한다.
- [ ] `cartesian_targets.yaml`의 `executable`과 docking blocker가 서로 모순되지
  않도록 하나의 authoritative 실행 가능 상태로 통합한다.
- [ ] camera-normal 30 mm 전진을 측정된 X/Z 오프셋
  `[-45.7, 0, -50.0] mm`를 사용하는 tag-relative coarse/fine/dock 접근으로 바꾼다.
- [ ] drill 실제 회전축, 회전 방향, 감속비와 `ee_joint` 1:1 결합을 실물/RViz에서
  대조하고 STL 원점의 약 1.47 mm X 편심을 TCP 보정과 구분한다.
- [ ] cable, circuit box와 supply box를 포함한 drill swept volume collision을
  접근·회전·후퇴 각각 검사한다.
- [ ] drill 무회전/회전 상태의 전류, 진동과 정지 시간을 측정하고 과전류 및
  회전 timeout 기준을 실측으로 정한다.
- [ ] 도킹 완료, 팔 정지, 축 결합 확인이 모두 참일 때만 drill 회전을 허용하는
  interlock과 firmware watchdog을 추가한다.
- [ ] tool1 실제 질량, 무게중심과 관성을 측정해 임시 URDF 값을 교체한다.

Tool1 완료 조건: limit 안의 재측정 자세와 전체 collision 경로, tag-relative 최종
도킹, 실제 회전축/방향, interlock, 회전 전류·진동 및 watchdog 실기 기록이 모두 있다.

### Tool2 — supply-box pick/place

tool2 작업은 별도 툴 형상이 아니라 tool0과 같은 물리 gripper를 사용한다. 저장 joint
goal은 접촉 전 pre-grasp가 아니라 rack/finger가 박스에 닿은 grasp 자세다. pick/place
상태 머신은 있으나 설정은 `enabled: false`이고 gripper 보정과 payload/사람 안전
조건이 비어 있으므로 개발 모드로 유지한다.

- [ ] supply box와 접촉하지 않는 별도 pre-grasp 자세를 정의하고 MoveIt 진입 목표로
  사용한다.
- [ ] pre-grasp에서 grasp까지의 저속 직선 접근과 finger/rack 접촉만 grasp 단계의
  제한된 collision policy로 검증한다.
- [ ] 실측 `open_rad`, `close_rad`, `ee_speed_rad_s`를 입력하고 무부하/접촉/파지
  전류를 분리해 측정한다.
- [ ] supply box의 치수, 질량, 무게중심, 파지면과 마찰 조건을 기록하고 실제 grasp
  TCP와 접근 자세를 확정한다.
- [ ] 파지 성공을 gripper 위치와 전류로 교차 확인하고, 가능하면 접촉/압력 센서를
  추가한다. 명령 완료만으로 파지 성공 처리하지 않는다.
- [ ] 파지 뒤 supply box를 attached collision object와 payload 관성에 반영하고,
  release 확인 뒤 world object로 전환한다.
- [ ] payload 상태의 joint speed/acceleration/current 제한과 전체 pick-return-place
  경로 collision을 검증한다.
- [ ] 카메라-to-ground TF, 사람의 발 위치 추정, detection timestamp와 base 정지
  조건을 검증한다.
- [ ] 사람이 움직이거나 관측이 stale이면 고정한 배치 목표를 폐기하고 HOLD한 뒤
  새 관측에서 다시 시작한다.
- [ ] 사람과 팔/물체 사이의 최소거리, 접근 속도, 즉시 정지 조건을 별도의 안전
  요구사항으로 정의한다.
- [ ] 사람 대신 고정 더미 목표로 grasp, attached-object 이동, placement, release와
  귀환 전체를 먼저 검증한다.
- [ ] 고정 더미 실기와 사람 주변 안전 검토가 끝난 뒤에만
  `tool2_pick_place.yaml enabled: true`로 변경한다.

Tool2 완료 조건: gripper와 payload 실측 보정, attached/world scene 전이, 파지·해제
물리 확인, 고정 더미 전체 시퀀스, 사람 관측 갱신 및 거리/정지 안전 조건이 모두 통과한다.

### 세 tool 공통 완료 조건

- [ ] 관절별 zero/sign, soft limit와 startup recovery endpoint를 저속 실측한다.
- [ ] tool별 실제 작업 TCP, 질량, 무게중심과 관성을 측정한다.
- [ ] `HOME -> BACK`을 base 단독 직선 회전으로 실행하지 않고 현재 feedback에서
  MoveIt 회피 계획과 전체 sampled collision 검사를 수행한다.
- [ ] chassis, circuit box, rack, 장착 tool과 운반 payload를 planning scene에
  포함하고 장착/해체/파지/해제 상태 전이를 테스트한다.
- [ ] 모든 실행에 timestamp, epoch, execution/sequence ID, command, feedback,
  velocity, current, collision/planning 결과와 최초 실패 원인을 남긴다.
- [ ] Arduino 통신 단절과 프로세스 종료 시 자체 정지하는 firmware watchdog을
  검증한다.
- [ ] 실패와 cancel 뒤 현재 실측 위치 HOLD, scene 갱신과 새 planning 정책을 세
  tool에 동일하게 적용한다.

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

- [x] 명령을 즉시 feedback으로 복사하지 않는 순수 Python 관절 mock을 만들었다.
- [x] 속도/가속도 제한, 1차 지연, encoder quantization, seed 기반 정지 잔차를
  `rad`, `rad/s`, `rad/s^2`, `s` 단위로 지원한다.
- [x] 관절별 stall, feedback dropout, command 지연, 일회성 overshoot를 주입할 수
  있게 했다.
- [x] manager 연동 fixture에서 following error, waypoint stall, waypoint timeout,
  joint feedback timeout이 서로 다른 최초 실패 원인으로 나오는지 확인했다.
- [x] 목표 반전 중 아직 이전 방향으로 움직이는 정상 지연을 direction fault로
  오인하지 않는 테스트를 작성했다.
- [x] mock 결과로 production 임계값을 바꾸지 않았다. 모델 입력과 fault fixture의
  측정 가능한 의미·단위를 `joint_mock_plant.py`에 고정했다.

합격 조건: 같은 seed와 fixture에서 동일한 최초 실패 원인이 나온다. 충족:
`test_joint_mock_plant.py`가 각 fixture를 두 번 실행해 최초 reason 문자열까지 비교한다.

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
- [x] chassis, circuit box, camera/bracket, rack, tool 형상을 robot-description
  planning scene에 포함했고 tool 0/1/2별 collision geometry 존재를 정적 검증한다.
- [x] SRDF allowed-collision matrix를 고정 조립면·직접 인접면·명시된 fixture resting
  contact로 제한했다. rack/supply-box, rack/circuit-box 및 비인접 arm/fixture 접촉이
  전역 허용되지 않는 회귀 테스트를 추가했다. merge에서 들어온
  `rack_left/right <-> tool_case_1` 전역 제외도 제거했으며, `ik_manual`의 rack 제거
  planning-scene 단계 전이가 구현될 때 해당 단계에서만 제한적으로 허용한다.
- [x] HOME/BACK은 고정 joint 직선으로 실행하지 않고 현재 정지 실측 snapshot을
  MoveIt start state로 사용해 각 named pose까지 계획한다.
- [x] 현재 자세→기록 경로 진입, 전체 기록/계획 경로, EE motion과 retreat/귀환을
  각각 sampled state-validity gate로 검사한다.
- [ ] 정상 시작뿐 아니라 경로 중단 자세와 limit 인접 자세에서도 실제 MoveIt plan과
  collision 결과를 수집한다. 로컬 단위 테스트에서는 중단 자세가 그대로 start state가
  되고 limit 밖 목표가 planner 호출 전에 거부되는 것까지 고정했다.
- [x] sampled state validity 간격과 검증 예산을 정하고 작은 장애물/swept-volume
  한계를 문서화한다.
- [x] 계획/검증 중 feedback, epoch, ownership이 바뀌면 결과를 폐기하며, 완료된
  stale future의 result도 소비하지 않는 테스트를 추가했다.

합격 조건: 각 시작 자세와 경로에 plan 결과, collision 결과, 실패 지점이 남는다.
정적 scene/정책과 stale-result 폐기는 완료했다. 실제 MoveIt graph에서 정상·중단·limit
인접 seed별 결과를 저장하는 통합 검증은 남아 있다. 세부 기준은
`docs/moveit_scene_validation.md`에 기록했다.

## 단계 6 — TCP joystick과 deadman

- [x] fresh Joy, release-to-rearm, L1 release, 250 ms timeout을 독립 테스트했다.
- [x] collision service 대기 중에는 새 IK를 적분하지 않고, 승인 뒤 다음 증분도
  20 ms control quantum만 사용하는 catch-up 방지 테스트를 추가했다.
- [x] collision 승인 전 후보 자세를 commit하지 않으며, release로 취소된 오래된
  future가 나중에 성공해도 빠른 재입력의 새 transaction에 반영되지 않는다.
- [x] R2를 `현재 정지 자세 TCP 진입 검사`로 고정했다. fresh stopped HOLD,
  전 관절 `|velocity| <= 0.02 rad/s`, limit margin, Jacobian condition, 현재 full-state
  collision만 검사하고 named pose·nearest pose·mode transition은 요청하지 않는다.
- [x] HOME, L1, L2 조합이 동시에 두 workflow를 시작하지 않는지 확인했다. L2가
  있으면 TCP 준비와 square HOME을 먼저 quiesce하고 tool selector만 소유한다.

합격 조건: 오래된 service 응답이나 빠른 재입력으로 motion이 재시작되지 않는다.
충족: 취소된 future의 지연 성공, collision 대기, release/repress 및 L2+HOME+L1
fixture가 모두 새 motion/commit을 만들지 않는지 검증한다.

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

