# Arm stability rework TODO

기준 브랜치: `rework/aa6614-arm-stability`  
기준 커밋: `aa6614cdee3c31d326314a5d2f6d1a3123be1f51`  
비교 대상 보존 브랜치: `main` (`bd87acb`)

## 원칙

- ROS 패키지는 빌드하지 않는다.
- `aa6614` 이후 커밋은 통째로 cherry-pick하지 않는다. 필요한 변경은 원인과 테스트를
  확인한 뒤 최소 단위로 다시 작성한다.
- 한 단계가 합격하기 전에는 다음 단계의 기능을 추가하지 않는다.
- mock 합격, MoveIt 합격, 실기 합격을 서로 다른 결과로 기록한다.
- 오류 임계값을 완화하기 전에 command, feedback, velocity, current, epoch,
  execution ID를 통해 최초 원인을 확인한다.
- 물리적 도킹과 파지는 software/Arduino 완료 메시지만으로 성공 처리하지 않는다.

## 단계 0 — 기준선 고정과 재현 자료

- [x] 현재 `main`을 보존하고 `aa6614` 기준 재작업 브랜치를 생성한다.
- [x] `aa6614`의 `test_arm_mode_core.py`: 43 passed (2026-10-31, 빌드 없음).
- [ ] 반복 오류별 실제 로그를 수집한다: DXL 정렬, RMD direction mismatch,
  following error, waypoint stall/timeout, HOME 충돌, docking deadman/ACK.
- [ ] 각 로그에 최초 오류와 후속 오류를 구분한다.
- [ ] 공통 재현 레코드 형식을 정의한다: timestamp, stage, epoch, execution ID,
  command/position/velocity/current, controller/hardware status, Joy, MoveIt 결과.

합격 조건: 각 반복 오류에 최소 한 개의 재현 로그 또는 재현 fixture가 있다.

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
- [ ] L2 deadman과 UI 요청의 정책을 분리한다.
- [ ] Joy heartbeat stale과 실제 L2 release를 다른 오류로 보고한다.
- [ ] Arduino 명령 전 subscriber와 정지 HOLD를 확인한다.
- [ ] 명령 이전의 오래된 ACK/상태로 성공하지 않게 sequence를 검증한다.
- [ ] timeout/cancel 뒤 Arduino가 계속 움직일 수 있음을 상태에 표시한다.
- [ ] release path가 없는 tool은 UI release를 명시적으로 거부한다.

합격 조건: software path 완료, Arduino 완료, 물리 도킹 확인이 혼동되지 않는다.

## 단계 8 — 실기 단계 검증

- [ ] 관절별 부호, 영점, soft limit, safe endpoint를 저속으로 확인한다.
- [ ] 한 관절씩 ramp 응답과 command-feedback-current를 기록한다.
- [ ] 무부하 HOME/TCP-ready를 저속으로 실행한다.
- [ ] tool path를 tag/Arduino continuation 없이 실행한다.
- [ ] tag 보정을 preview-only로 검증한다.
- [ ] Arduino 구동을 정지한 팔에서 별도로 검증한다.
- [ ] collision object와 실제 여유 거리 차이를 기록한다.
- [ ] 실측 자료로만 following-error, stall, timeout, speed/acceleration 값을 확정한다.

합격 조건: 각 단계의 로그와 현장 확인 결과가 있고, 이전 단계 실패 시 다음 단계로
진행하지 않는다.

## 이후 커밋 참고 지도

아래 커밋은 정답으로 cherry-pick하지 않고 회귀 사례와 아이디어만 참고한다.

- `378ae43`, `9629d86`, `42e4f45`: DXL 정렬과 fault reset
- `78e4a25`: RMD direction mismatch/CAN timestamp
- `1fd1dc2`, `13b26bc`: HOME workflow와 MoveIt 계획
- `554c999`, `46249ba`, `c1646fc`, `c980682`, `e76ce55`: 경로 속도/재표본화
- `c74bfef`, `3adbcf2`, `39f444c`, `0469a3b`: docking/Arduino/deadman
- `710cb7e`, `8c6a9f7`, `d3aa5f0`: startup safe endpoint와 soft limit
- `bd87acb`: 현재 자세 TCP 진입 의미

