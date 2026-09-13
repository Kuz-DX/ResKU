# PlanD 변경 체크리스트 (`slope_traverse_pland_node`)

planC 기준으로 바뀐 것들. 실측 테스트할 때 이 순서대로 영향 확인.

```bash
ros2 launch robot_bringup autonomous_pland.launch.py
```

- [ ] **트리거 pitch값 변경(6차, 최신)** — `slope_exit_entry_pitch_deg`: -5.0 → 0.0(부호전환) → -3.0 → **-2.0**(고정임계값, pitch<=이값이면 트리거). **1회성 래치(`slope_exit_trigger_used_`)** 유지 — "처음 -2도 찍힌 시점"만 트리거로 인정, 그 뒤 SLOPE_EXIT 중 pitch가 다시 -2 이하가 돼도 재트리거 안 됨. RECOVERY까지 다녀와 새 SLOPE_DRIVE 세션이 시작돼야만(=탈출 3조건 실제 충족 후에만) 래치가 풀림.
  - ⚠️ **부작용**: `slope_exit_end_pitch_deg`(탈출조건 pitch_ok)는 -3.0 그대로라 진입(-2.0)보다 더 낮음(더 가파른 조건) — 진입 시점 pitch(~-2)는 항상 -3보다 크므로 pitch_ok가 SLOPE_EXIT 진입 즉시 거의 항상 참이 됨. 실질 게이트는 roll_ok/path_straight_ok 둘뿐(이전과 동일한 구조).
- [ ] **SLOPE_DRIVE side==0→IDLE 폴백 제거** — side(roll_deg 기반)가 pitch(myAHRS+ 기반)보다 먼저 풀려서 SLOPE_EXIT 트리거를 확인할 기회조차 없이 IDLE로 튕겨나가던 문제 대응. 이제 pitch 트리거 전까지 SLOPE_DRIVE에서 스스로 안 나감.
- [ ] **SLOPE_EXIT side==0→RECOVERY 조기출구 제거** — 같은 이유로 일관성 있게 제거. 이제 SLOPE_EXIT은 탈출 3조건(roll/pitch/path) AND 하나로만 나감.
- [ ] **SLOPE_EXIT 탈출조건 pitch_ok 완화** — `slope_exit_end_pitch_deg`: -6.0 → **-3.0**.
- [ ] **SLOPE_DRIVE climb_timeout_sec_(600초) 정지-래치 제거** — 시간 초과로 스스로 영구정지하던 것 제거. yaw_error(90°) 즉시성 안전 트리거는 유지.
- [ ] **SLOPE_EXIT slope_exit_timeout_sec_(15초) 정지-래치 제거** — 위와 동일 사유. 이제 SLOPE_EXIT은 조건 충족될 때까지 무한정 시도.

## 참고

- 위 항목 전부 `slope_traverse_planc_node.cpp`(PlanC)는 그대로, `slope_traverse_pland_node.cpp`(PlanD)만 적용됨.
- `climb_timeout_sec_`/`slope_exit_timeout_sec_`/`side_exit_dwell_sec` 파라미터는 인터페이스 호환 위해 선언만 유지, 로직 미사용.
- 상세 배경/가설은 `검증.md` 참고.
