> 현재 시험 기본값은 **수동 랜드마크 촬영**입니다. `enable_auto_keyframes=false`로
> 자동 촬영만 잠시 중지했고 기존 자동 로직은 남겨 두었습니다. 이전의 1m/2초
> 자동 간격 설명은 자동 촬영을 다시 켰을 때만 적용됩니다.

# 거리 체크포인트 VPR / 정지 사진 Dashboard

이 문서가 이전 8m watchdog 및 실시간 preview 설명보다 우선한다.

## 기록과 순서

- `MANUAL_RECORDING` 또는 `WAIT_RETURN_COMMAND`에서 조이스틱 D-pad ↑ 또는
  `/path/landmark` 버튼을 한 번 누를 때마다 후방 사진 **한 장**을 저장한다.
  정지 촬영 모드에서는 기존 mux hold와 정지 확인 후 촬영한다. 버튼을 누른 위치에서
  사진이 실제로 찍힌 시점의 odometry 위치를 쓴다.
- 사진·CameraInfo·이미지 시각 TF·20개 이상 SIFT 특징점이 유효해야
  `captured` 이벤트를 내고 Return 참조로 등록한다. 실패하면 hold를 해제하고
  `failed` 이유를 보낸다. TF 검증 실패로 남은 보관 파일은 참조로 활성화하지 않는다.
- 기존 `meta.yaml`의 ID/order, mission_pose, s_along_path, 이미지 시각/TF와
  `features.npz`의 좌표/descriptor를 저장한다. 사진과 특징점은 해당 경로 위치에 연결된다.
  `capture_source=manual`로 저장하고 UI 왼쪽에 촬영 시점의 사진과 SIFT 점을 표시한다.
  기록 위치는 odometry 기반 참조이며 측량된 절대 정답은 아니다.
- Return에서는 **기록 순서의 역순**으로 체크포인트를 처리한다. Return/Return Manual에서
  새로운 outbound 체크포인트를 저장하지 않는다. 프로세스 재시작 시 과거 미션 자동 로딩 없음.

## 도달과 검사

- `/return_path`와 저장 위치를 투영한 `/visual_landmark/checkpoints` 카탈로그를 사용한다.
  반환 JSON은 역순 ID, order, return arc, mission pose, outbound s, 특징점 수를 포함한다.
- 컨트롤러의 corner/floor 제한을 적용한 `/visual_landmark/route_progress`를 사용한다.
  전역 XY 최근접점으로 교차로의 다른 가지를 선택하지 않는다.
- `enable_checkpoint_vpr=true`에서 8m/slip/start/resume 트리거는 사용하지 않는다.
  `enable_adaptive_visual_check=true`는 요청 기반 처리 스위치로 유지한다.
- 도달 허용 구간은 return arc 기준 **0.35m 전부터 0.75m 후까지**이다.
  기본값은 시험용이다. 정확한 점에서 정지시키지 않고 해당 구간에서 검사를 시작한다.
- 요청에는 `checkpoint_id`, `checkpoint_arc`, UUID가 포함된다. 해당 ID가 최종 선택돼야
  보정할 수 있다. 주변 bounded 후보도 검증하여 기존 ambiguity 거부를 유지한다.
  다른 후보가 더 잘 맞으면 해당 요청은 거부하며 그 후보를 임의로 보정에 사용하지 않는다.
- 요청당 최대 5초/4프레임, 독립된 2프레임 일치가 필요하다. 진행도가 해당 구간을 넘으면
  SIFT 전에 거부한다. 성공/실패/timeout 후 다음 체크포인트로 이동한다.
  타임아웃으로 뒤의 지점을 지나쳤으면 `checkpoint_missed`를 기록하며 이전 사진을
  엉뚱한 위치에서 비교하지 않는다. 서로 다른 체크포인트에는 5초 cooldown을 적용하지 않는다.
- Manual에서는 시작하지 않고 진행 중 요청을 취소한다. Auto 복귀 시 같은 지점이 아직
  도달 구간 안이면 재검사하며 이미 지났으면 missed 처리한다. 기존 완료 ID는 유지한다.
- 도달 판정 자체가 odom/경로 진행도 추정이므로 실제 위치 오차가 클 때 촬영 지점과 달라질
  수 있다. 시각 매칭 실패를 실제 위치 도달로 간주하지 않는다. 이 모드는 전역 재탐색기가 아니다.
- 기존 TF/LOCK, 깊이, F/PnP, 공간 분포, prior, 두 프레임 일치, 보정 한계는 유지한다.
  보정 ON 기본값 유지. 모션 명령/제자리 회전/정지 탐색은 추가하지 않는다.

## UI / RViz

`vpr_viewer`는 이제 카메라 실시간 토픽을 구독하지 않는다. UI SIFT도 실행하지 않는다.
기존 `live_features` 파라미터는 호환성 때문에 받지만 snapshot UI에서는 무시한다.

- 왼쪽: Record에서 새로 저장된 정답 참조 사진과 이미 계산된 SIFT 점.
  Return에서는 해당 역순 체크포인트의 참조 사진.
- 오른쪽: 실제 체크 순간의 현재 전방 사진과 실제 계산된 SIFT 점.
- 하단: 지정된 정답과 현재 프레임의 descriptor match / PnP inlier.
- 다음 체크포인트 검사 전까지 마지막 사진을 유지한다. 새 검사가 시작되면 이전
  현재/Matches를 비워 이전 지점의 사진을 새 참조와 혼동하지 않게 한다.
- `visual_debug_matches=true` 필요. 입력/TF 문제로 SIFT 전에 거부되면 현재 사진/매칭은
  대기 상태이고 localization_event의 reason을 확인한다. 표시 자체는 보정 성공이 아니다.
- 각 요청의 two-frame 검증 중에는 최대 4장의 순간 결과로 갱신될 수 있다. 동영상 preview가 아니다.

이미지 토픽은 모두 CompressedImage / reliable transient_local depth 1:
`/visual_landmark/debug/reference/compressed`, `/visual_landmark/debug/current/compressed`,
`/visual_landmark/debug/matches/compressed`.

RViz: 기존 경로 표시와 함께 Add → MarkerArray → `/visual_landmark/checkpoint_markers`.
각 저장 위치와 ID가 mission 좌표로 표시된다. Fixed Frame은 mission 또는 mission으로
TF가 연결된 프레임이어야 한다. 이 마커는 경로 추종용 Path를 변형하지 않는다.

## 적용 명령

로봇 PC와 노트북 각각 수정 코드 반영 후:

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_landmark robot_bringup
source install/setup.bash
```

차체 정지 후 기존 주행 launch를 종료하고 **로봇 PC**에서 재실행한다.
카메라/팔 LOCK TF는 기존 검증된 설정으로 별도로 실행한다.

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  can_interface:=can_drive imu_port:=/dev/ttyACM1 reverse_return:=false \
  enable_auto_keyframes:=false enable_visual_localization:=true \
  enable_adaptive_visual_check:=true enable_checkpoint_vpr:=true \
  visual_debug_matches:=true visual_apply_correction:=true \
  enable_landmark_recovery:=false enable_stationary_vpr_debug:=false
```

노트북에서 기존 정상 DDS/domain 99 환경을 적용한 터미널:

```bash
ros2 run visual_landmark vpr_viewer
```

Q 또는 실행 터미널 Ctrl+C로 UI만 종료, 같은 명령으로 재실행한다.
새 Record → 원하는 위치에서 D-pad ↑로 수동 촬영 → `captured`와 `reference_loaded`
및 `/visual_landmark/checkpoints` 확인 → Return → scheduler_event의 trigger=checkpoint,
checkpoint_id 역순 확인 → localization_event의 localized/correction_applied 확인.

```bash
ros2 topic echo /visual_landmark/scheduler_event
ros2 topic echo /visual_landmark/localization_event
```

`enable_checkpoint_vpr=false`는 기존 8m/slip scheduler로 돌아간다.
실차 인식/지연/보정 정확도는 코드 테스트로 확정되지 않으며 카메라 시험이 필요하다.


Record 표시 보강: `/visual_landmark/debug/captured/compressed`는 촬영 노드가 저장 성공
직후 기존 SIFT 점을 재사용해 직접 발행한다. localization 참조 로딩/표시 옵션과 분리되어
Record 사진을 볼 수 있다. Return의 비교용 reference 토픽은 기존대로 localizer가 발행한다.
`capture_status`는 2초마다 Record 상태와 수동 촬영 준비 여부를
보내며 UI 왼쪽 대기 화면에 표시한다. TF/품질 거부를 건너뛰어 유효 참조로 만들지는 않는다.
수동 촬영 간격과 5초/두 프레임 검사 때문에 속도가 빠르면 다음 체크포인트가 missed일 수 있으므로
scheduler_event를 확인한다. 실차 현재 실패 원인은 해당 event 출력으로 확인해야 한다.

## 지연 영상과 odometry 연결

APC850의 수신 시각 대비 이미지 timestamp 차이 약 0.615초가 관측되었다.
이 값을 무조건 보정하거나 timestamp를 현재 시각으로 바꾸지 않는다.
촬영 노드는 odometry 샘플을 최대 512개/도착 기준 3초 보관하고, 이미지 timestamp에
가장 가까운 과거 샘플의 twist와 경로 거리를 metadata에 저장한다. 영상과 선택 샘플의
차이 제한은 기존 0.5초를 유지한다. 실제 카메라/base pose는 기존 이미지 시각 TF로 조회한다.
최신 odometry callback은 여전히 0.5초 안에 수신되어야 하고 이미지 freshness 1초도 유지한다.
metadata의 `odom_image_skew_s`와 `odom_sample_selection=nearest_buffered`로 선택을 확인한다.
이 변경은 카메라 버퍼 지연/시계 오프셋 자체를 해결한 것이 아니며 timestamp가 실제 촬영
시각을 의미하는지는 카메라 설정/실차 시험으로 별도 확인해야 한다.
