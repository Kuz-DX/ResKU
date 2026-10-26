# Adaptive Visual Correction 구현 보고

작업 기준: 로컬 main. commit/push 없음. 기존 카메라 장착 좌표·optical 기준 정정 작업을 보존했다.

1. **기존 구현 재사용**
   APC850 reference RGB, D435i aligned RGB-D, SIFT cache/mutual ratio, depth→3D,
   PnP RANSAC/LM, inlier·geometry·cheirality·RMSE·최대 보정량·두 프레임 합의·ambiguity·LOCK 검사를 유지했다.
   persistent mission←odom, Manual/Auto 경로 보존, 별도 yaw recovery도 유지했다.

2. **변경 architecture**
   별도 가벼운 visual_check_scheduler_node가 watchdog/slip/cooldown/요청 수명을 담당한다.
   visual_localization_node는 adaptive 기본 모드에서 요청이 없으면 decode/SIFT/matching/PnP 전에 반환한다.
   reduced_odom 및 ReturnStateMachineNode는 변경하지 않았다. 추종 제어식도 변경하지 않았다.

3. **추가/수정 파일**
   추가: visual_landmark/adaptive.py, visual_check_scheduler_node.py,
   benchmark_adaptive.py, test/test_adaptive.py, test_adaptive_nodes.py,
   ADAPTIVE_VISUAL.md, IMPLEMENTATION_REPORT.md.
   수정: visual_landmark_node.py, visual_localization_node.py, localization.py,
   test_cross_camera.py, package.xml, setup.py, README.md, HARDWARE_VALIDATION.md,
   CAMERA_VPR_TODO.md, return_navigation/return_path_follower_node.py,
   robot_bringup/launch/manual_return_bringup.launch.py.
   별도 선행 장착 작업의 측정 YAML·후방 launch·test_rear_launch.py 변경도 작업 트리에 남아 있다.

4. **자동 keyframe**
   첫 valid anchor 후 거리 1.25m OR heading 12° OR 6초+최소 이동 0.5m.
   정지 중 timeout 중복 생성 없음. 자동 촬영은 이동 중이며 hold/motion 명령 없음.
   수동 stop 버튼은 보존. freshness·CameraInfo·TF·최소 특징점, 선택 blur 검사.
   자동 SIFT는 한 번 계산 후 features.npz로 재사용. 실패는 anchor를 갱신하지 않고 1초 retry.
   schema 3 확장, 기존 schema 2 지원. metadata 세부사항은 ADAPTIVE_VISUAL.md 참조.

5. **Hard watchdog**
   Return 진입 시 0에서 시작, raw odometry XY 주행거리 8m에 체크 요청.
   활성 Return 중 Manual 이동도 포함하며 Auto 재개 시 보존. 실제 global drift 측정값이 아니다.
   큼직한 odom 불연속은 제외한다. 정상·작은 보정의 validated 성공만 distance anchor를 reset한다.

6. **wheel/gyro Early Trigger**
   기존 /imu/gyro/z_corrected 및 reduced_odom 진단을 재사용한다.
   abs(wheel_wz-gyro_wz) deadband 초과분을 dt로 적분하고 decay를 뺀다.
   calibration 완료·gyro source·fresh finite wheel/gyro·fresh diagnostics일 때만 유효하다.
   fallback/stale/시간 단절에서는 score를 버리며 early trigger를 중단한다.
   covariance나 누적 회전량을 trigger로 사용하지 않는다.

7. **Candidate 제한**
   추종기가 이미 제한한 코너/남은 구간에서 route progress를 발행한다.
   outbound order/s와 RETURN 경로 방향으로 reference arc를 캐시하고 정상 후보 최대 3개.
   실패 후 같은 요청에서 +2 확장, 절대 상한 7. 기본 전체 brute-force 없음.
   각 bounded 후보의 PnP는 유지해 기존 기하 기반 ambiguity gate를 약화하지 않았다.
   자기 교차·역순 테스트 포함. 경로 단순화 및 큰 outbound drift 영향은 실차 검증 필요.

8. **성공/실패 동작**
   matching UUID의 validated/localized만 anchor reset. 실패/노드 없음은 기존 보정과 일반 복귀 유지.
   요청 하나, 최대 5초/4처리 프레임, 종료 후 5초 cooldown. timeout을 넘은 성공은 거부.
   Manual 진입은 요청 취소, distance 보존. 별도 yaw search를 자동 활성화하지 않는다.

9. **Correction 조건**
   모든 기존 quality gate 및 두 프레임 합의 후, 관찰 모드가 아니고
   translation>=0.03m OR yaw>=0.01rad인 경우 publish. 이보다 작아도 visual success이다.
   별도 legacy yaw recovery는 resume를 위한 correction commit 동작을 보존한다.
   raw odometry/EKF reset 없음.

10. **새 ROS topic**
    /visual_landmark/check_request, /visual_landmark/scheduler_event,
    /visual_landmark/route_progress (모두 std_msgs/String JSON).
    기존 localization_event에는 UUID·terminal 여부·trigger·candidate/timing을 추가했다.
    기존 search_request와 의미를 분리했다.

11. **Launch parameter**
    enable_auto_keyframes 및 auto_keyframe_distance_m/yaw_deg/max_interval_s/
    min_motion_for_timeout_m/retry_s/min_features/blur_min;
    enable_adaptive_visual_check, visual_check_distance_m/retry_cooldown_s/timeout_s/
    max_frames/on_return_start, max_visual_candidates;
    enable_slip_early_trigger, slip_wz_deadband_radps, slip_score_trigger/decay,
    wheel_timeout_s, gyro_timeout_s, diagnostics_timeout_s,
    correction_min_translation_m/yaw_rad. 전체 기본값 표는 ADAPTIVE_VISUAL.md 참조.
    visual_apply_correction=false, enable_landmark_recovery=false 유지.

12. **테스트 결과**
    최종 테스트 수/결과는 아래 검증 기록을 참조한다. 자동 조건·watchdog·cooldown·상태 보존·
    stale/fallback·잘못된 UUID·bounded 후보·교차 경로·연속 합의·요청 기한·deadband·모호성 검증 포함.
    격리 ROS_DOMAIN_ID=229에서 시각 노드 3개 생성·종료 확인(모터 기동 없음).
    launch --show-args로 parameter 노출 확인.

13. **기존 회귀 검증**
    기존 테스트를 삭제하지 않았다. 기존 live geometry fixture에 새 요청/진행도 계약만 추가했다.
    기존 return_navigation 및 drive_cmd_mux 테스트를 함께 실행했다.
    Manual/Auto resume, stop capture, recovery, 기존 기하 rejection 및 observation-only 포함.

14. **실차 튜닝 parameter**
    auto 간격·heading·timeout/min-motion, blur, watchdog distance, slip deadband/score/decay,
    freshness·cooldown·check timeout, candidate window, correction deadband.
    initial engineering defaults; field calibration required.
    슬립·기하 threshold를 실차 검증 없이 완화하지 않는다.

15. **아직 검증하지 않은 assumption 및 성능 한계**
    광학 TF/LOCK·카메라 보정 최종 품질·moving blur·실제 cross-camera 매칭·바닥 slip 임계값·
    교차/평행 경로의 실제 분기 인식·JECS CPU·실차 도착 정확도는 검증되지 않았다.
    Float64 gyro freshness는 도착 시각이며 센서 source stamp를 제공하지 않는다.
    후보 arc는 단순화 경로와 outbound s가 대략 일치한다는 가정을 사용한다.
    출발 기록의 odometry 오차까지 제거하거나 surveyed 절대 위치를 주는 구조가 아니다.
    프로세스 재시작 후 기존 mission 자동 복원은 기존대로 지원하지 않는다.

    합성 벤치마크: OpenCV 4.5.4, 1 thread, 640×480 무작위 텍스처,
    1296 SIFT features, 100 references 중 3개, 10회 반복.
    단일 처리 프레임 평균/최대 292.6/299.1ms, SIFT 평균 48.3ms,
    matching+depth 평균 240.0ms, PnP+refine 평균 4.3ms, CPU time 평균 292.6ms.
    이는 동일 합성 reference와 가상 depth의 microbenchmark이며 실제 인식 성능/전체 체크 지연이 아니다.
    전체 check의 요청→terminal 지연은 scheduler_event.check_latency_s로 실차에서 수집해야 한다.
    원본 성능 JSON은 debug_report/adaptive_visual/synthetic_benchmark.json (Git 제외).

16. **실행 명령**
    ADAPTIVE_VISUAL.md의 phase별 명령 사용. 먼저 auto-only, 다음 adaptive+보정 OFF,
    known-offset, slip 로그 튜닝을 거쳐 마지막에 correction을 켠다.
    일반 bringup은 모터를 기동하므로 정지 기하 시험에는 모터 차단/별도 visual harness가 필요하다.

17. **rosbag 검증 명령**
    ADAPTIVE_VISUAL.md의 전체 rosbag 명령 참조.
    wheel/gyro/진단/odom, scheduler/localization/correction, 경로/주행 명령,
    state/origin, 두 영상·CameraInfo·depth·TF를 동시 기록한다.
    직진·정상 회전·제자리 회전·바닥별 mismatch와 요청 시점, matching 실패 후 복귀 유지,
    실제 known-offset 및 최종 Base 오차를 대조한다.

## 최종 검증 기록 (2026-10-26)

- `/usr/bin/python3`, ROS Humble 환경에서 119 passed (1.34s).
- 실행: `python3 -m pytest -q src/drive/return/visual_landmark/test src/drive/return/return_navigation/test src/drive/return/drive_cmd_mux/test`.
  소스 테스트에는 세 패키지 루트를 PYTHONPATH에 추가하고 기존 ROS PYTHONPATH를 보존했다.
- `colcon build --packages-select visual_landmark return_navigation drive_cmd_mux robot_bringup`: 4 packages finished.
- `ros2 launch robot_bringup manual_return_bringup.launch.py --show-args`: 성공(모터 실행 없음).
- ROS_DOMAIN_ID=229: VisualLandmarkNode, VisualLocalizationNode, VisualCheckScheduler 생성/종료 성공.
- `git diff --check`: 통과. commit/push 미실행.
- 실제 카메라/차체를 구동하는 field validation은 수행하지 않았다.
