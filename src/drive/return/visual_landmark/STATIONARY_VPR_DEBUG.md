# 임시 RETURN_MANUAL 정지 VPR debug

기준: Kuz-DX/ResKU main `74d09e0c1808b8670f434ef3cd0068cb3e7f7b1d`.
시작 전 git status는 clean이었으며 origin/main과 일치했다.
기본 enable_stationary_vpr_debug=false. scheduler/state machine/controller는 수정하지 않는다.

## 빌드·실행

로봇 PC의 기존 카메라/실제 팔 TF는 [VPR_RUN_COMMANDS.md](VPR_RUN_COMMANDS.md)를 따른다.
아래는 기존 주행 launch를 대체한다. 같은 launch를 중복 실행하지 않는다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup visual_landmark vision tool_manipulator_bringup manual_joy_control
source install/setup.bash
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
ros2 launch robot_bringup manual_return_bringup.launch.py \
  enable_visual_localization:=true \
  enable_adaptive_visual_check:=true \
  visual_debug_matches:=true \
  visual_apply_correction:=false \
  enable_landmark_recovery:=false \
  enable_stationary_vpr_debug:=true
```

환경과 CAN/IMU 인자는 실제 로봇 설정에 맞춘다. 위 명령은 기존 주행 스택을 켠다.
새 Record 전부터 localization 노드가 켜져 있어야 live reference를 로드한다.
다른 터미널/노트북에서 동일한 ROS 환경으로:

```bash
ros2 run visual_landmark vpr_viewer
```

```bash
ros2 topic echo /visual_landmark/localization_event
```

## 전체 실행 순서: 팔 LOCK 포함

모든 터미널에서 ROS setup/워크스페이스 setup과 동일한 ROS_DOMAIN_ID를 적용한다.
CAN·장치 설정과 카메라 시리얼은 [VPR_RUN_COMMANDS.md](VPR_RUN_COMMANDS.md)를 따른다.
아래 각각은 별도 터미널에서 실행한다.

1. **로봇 PC: 팔 실기 IK/Manual** (기존 Arm_TF/real_control과 중복 실행 금지)

```bash
ros2 launch tool_manipulator_bringup ik_manual.launch.py real_hardware:=true launch_joy:=false
```

2. **노트북: 팔·주행 공유 조이스틱 발행자**

```bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py
```

현재 main의 파일명은 `remote_joy.launch.py`이며 `remote.launch.py`는 없다.
기존 팔 조작으로 카메라를 LOCK 자세에 고정하고 유지한다.

3. **노트북: 주행 조이스틱 해석** (joy_node는 위 remote_joy 하나만 사용)

```bash
ros2 launch manual_joy_control manual_control.launch.py launch_joy:=false
```

4. **로봇 PC: 전후방 카메라** (실제 D455 시리얼 대입)

```bash
ros2 launch vision vpr_observation.launch.py \
  rear_launch:=rear_vpr_trial.launch.py \
  start_arm:=true arm_serial_no:=_ACTUAL_D455_SERIAL \
  start_visual_nodes:=false start_viewer:=false
```

기존 arm_camera.launch.py를 함께 실행해 같은 전방 장치를 중복 점유하지 않는다.

5. **로봇 PC: 위 빌드·실행 절의 stationary debug 주행 launch**
6. **로봇 PC: 입력 검사**

```bash
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_stationary_preflight.json
```

ready_for_observation=true를 확인한다. 팔 IK launch만으로 차체→팔 장착→카메라 TF가
모두 올바르다고 가정하지 않는다. 특히 차체/팔 base_link 충돌과 optical 연결을 확인한다.

7. **노트북: 위 vpr_viewer와 localization_event echo 실행**
8. **선택: rosbag**. 기록 없이도 시험 가능하다.
9. 아래 Record→Return→Manual→정지 debug trigger 순서를 수행한다.

## 시험 순서

1. 평소처럼 Record, 수동 outbound 주행으로 rear keyframe/reference_loaded를 확보한다.
2. Return을 시작하고 FOLLOW_RETURN_PATH와 route_progress가 한 번 이상 나온 것을 확인한다.
3. 다음 명령 또는 기존 UI로 Manual 전환한다. Auto로 돌아갈 필요가 없다.

```bash
ros2 topic pub --once /mission/return/control_mode std_msgs/msg/String '{data: MANUAL}'
ros2 topic echo /mission/return/state --once
```

4. RETURN_MANUAL에서 조이스틱으로 원하는 자세로 이동한 뒤 조이스틱을 놓고 완전히 정지한다.
5. 검사 한 번 요청:

```bash
ros2 topic pub --once /visual_landmark/debug_check std_msgs/msg/Bool '{data: true}'
```

6. Matches와 event를 확인한다. 두 독립 RGB-D 프레임 일치가 필요하다.
   state는 RETURN_MANUAL로 남는다. 다른 자세로 이동·정지한 뒤 같은 명령을 다시 실행한다.
   중복 요청은 기존 요청을 바꾸거나 취소하지 않고 busy로 거부한다.

이 기능은 이동을 명령하거나 조이스틱을 잠그지 않는다. RETURN_MANUAL에서는 기존 수동
조종이 계속 가능하다. 정지 joystick/기존 mux가 정지를 유지하며, 주행자가 다시 움직이면
검사를 취소한다. odom 기반 gate는 실제 slip·모터 상태의 독립 측정은 아니다.
정지 odom source stamp와 도착 시각이 모두 0.5초 이내이고 유한해야 하며,
|vx|<=0.03m/s, |wz|<=0.03rad/s여야 한다. 이는 state machine의 기본 정지 기준과 같다.
정지 gate는 요청 시, 각 프레임 전 및 commit 전에 재확인하고 0.1초 timer에서도 확인한다.

## production 경로와 차이

- 기본 OFF: debug 구독/odom 구독을 생성하지 않는다. FOLLOW adaptive/legacy 경로 유지.
- ON: 전용 Bool true만 RETURN_MANUAL에서 검사 한 번 시작한다. false는 무시한다.
- UUID/timeout/max frames/후보 수 제한/기하·depth·PnP gate/두 프레임 일치를 재사용한다.
- Manual에서는 controller의 최신 progress를 요구할 수 없으므로 마지막 FOLLOW에서 받은
  arc를 요청 시작 시 고정한다. 새 global XY 후보 탐색은 하지 않는다. 이 구간에서 멀리
  이동하면 nearby reference를 못 찾을 수 있다. 다시 Auto로 돌아가라는 의미는 아니다.
- route/origin/mission 변경 시 frozen progress를 초기화한다. Manual 상태 변경은 검사를 취소한다.
- observation-only 전용: apply_correction=true이면 debug 요청 자체를 거부한다.
  성공해도 controller correction 발행, 내부 correction, production last_reference_arc는 바꾸지 않는다.
- debug 요청은 scheduler pending UUID와 무관하므로 watchdog anchor를 reset하지 않는다.
- no camera/동기화 실패/입력 소실이면 기존 timeout으로 끝난다.
- 별도 알고리즘 없이 기존 on_pair → features → match → 두 프레임 검증을 사용한다.
  debug_frame_result/terminal event의 attempts를 Viewer가 표시한다. 매칭/PnP는 생산 VPR 검사에서만 실행한다.
  별도의 live preview는 뷰어에서 SIFT 검출만 수행하며 아래 설명처럼 검증 결과와 구분한다.
- Matches는 마지막 후보/프레임이며 녹색 inlier만으로 최종 검증 성공을 의미하지 않는다.
  not_run은 해당 단계가 실행되지 않았거나 진단값이 없다는 뜻이다.

## event 예시 (일부 필드)

```json
{"event":"check_started","trigger":"stationary_debug","check_request":"NEW_UUID","route_arc":2.0}
{"event":"debug_frame_result","trigger":"stationary_debug","attempts":[{"id":"L1","reference_features":450,"current_features":520,"mutual_ratio_matches":60,"pnp_points":42,"inliers":35,"inlier_ratio":0.833,"reprojection_rmse":0.8,"reject_reason":null}]}
{"event":"validated","trigger":"stationary_debug","check_complete":true,"correction_applied":false,"planar_correction":{"xyz":[0.1,0.02,0.0],"rpy":[0.0,0.0,0.01]}}
```

수치는 설명용이며 실제 event에는 기존 timing/TF/UUID 등도 들어간다.
주요 실패:

| event/reason | 확인 |
| --- | --- |
| debug_check_rejected / stationary_debug_busy | 기존 검사 종료 후 재요청 |
| stationary_debug_requires_RETURN_MANUAL | 실제 mission state 확인 |
| stationary_debug_requires_observation_only | apply_correction=false로 실행 |
| stationary_debug_robot_moving | joystick 중립, vx/wz 확인 |
| stationary_debug_no_odometry | /odometry/filtered 수신 |
| stationary_debug_stale_or_invalid_odometry | source stamp/시계/도착 freshness/유한값 |
| stationary_debug_missing_origin_references_or_camera_info | 새 mission/reference_loaded/CameraInfo |
| stationary_debug_missing_auto_route_progress | Return 중 FOLLOW 진행 정보가 있었는지 확인 |
| no_match + attempts[].reject_reason | 특징점, mutual ratio, depth, PnP, inlier, 재투영 등 기존 gate |
| rejected + ambiguous landmarks / correction_step_limit | 기존 ambiguity/보정 한계, 임계값 완화하지 않음 |
| rejected + TF/LOCK/stale RGB-D 관련 reason | 실제 팔 TF/LOCK/영상 시간 확인 |
| skipped / check timeout | 카메라·RGB-D sync·timeout 확인 |

검사 중 움직이거나 odom freshness가 사라지면 rejected/check_complete=true로 종료된다.
OFF일 때 전용 trigger에는 응답하지 않는다(기능이 비활성화됨).

## 변경 파일·검토·롤백

- visual_localization_node.py: 선택 구독, 정지 gate, Manual 요청 수명, frozen arc,
  기존 pipeline 재사용, production anchor/보정 출력과 분리, 프레임 진단 event.
- manual_return_bringup.launch.py: default false debug 인자와 localization 전달.
- viewer.py: stationary_debug attempts의 기존 진단 8개 표시, 상태 화면 높이 확장.
- test/test_stationary_debug.py: 정지/이동/신선도/중복/timeout/frame 한도/두 프레임/관찰/
  상태 변경/보정 금지/출력 authority 회귀 검사.
- 이 문서 및 VPR_RUN_COMMANDS.md 링크.

아직 commit하지 않는다. 승인 전 기존 변경을 포함해 자동 commit하지 않는다.
검토:

```bash
git status --short
git diff --stat
git diff
# 새 파일은 git diff에 기본 포함되지 않으므로 직접 확인
cat src/drive/return/visual_landmark/test/test_stationary_debug.py
cat src/drive/return/visual_landmark/STATIONARY_VPR_DEBUG.md
```

이번 작업의 reverse patch는 `/tmp/resku-stationary-vpr-debug.patch`에 저장한다.
해당 패치를 로봇 PC에서 쓰려면 먼저 복사한다. rollback 전 launch를 종료한다.

```bash
git apply --reverse --check /tmp/resku-stationary-vpr-debug.patch
git apply --reverse /tmp/resku-stationary-vpr-debug.patch
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_landmark robot_bringup
source install/setup.bash
```

--check가 실패하면 후속 수정이 겹친 것이므로 강제로 restore하지 않는다.
이 reverse patch는 이번 수정분만 취소하고 생성된 debug 테스트/문서를 제거한다.
승인 후 이번 변경 파일만 별도 commit했다면 `git revert <temporary_vpr_debug_commit>`으로
되돌리고 동일하게 재빌드한다. 기능만 끄려면 default처럼
`enable_stationary_vpr_debug:=false`로 재실행하면 된다.

## 검증 결과

- visual_landmark + return_navigation + drive_cmd_mux: 146 tests passed.
- visual_landmark, robot_bringup: colcon build 성공.
- 실제 ROS Humble localization 노드 초기화: OFF 전용 토픽 구독 없음, ON 구독 있음 확인.
- git diff --check, reverse patch --check 통과.
- 실차 정지/수동 전환/카메라 정확도는 아직 실차에서 시험하지 않았다.


## 라이브 SIFT 특징점 표시 (2026-10-27)

`vpr_viewer`는 전후방 영상에 **실제 SIFT keypoint를 청록색으로 상시 표시**한다.
Record/Return/검사 요청과 무관하며, 검출점은 매칭 성공·PnP inlier·보정 결과가 아니다.
SIFT 원/방향 표시와 개수를 보여준다. 생산 VPR의 원본 해상도 특징과 동일한 집합을
보장하지 않는다(뷰어는 최대 폭 480, 최대 120개, 기본 2Hz로 부하를 제한).

```bash
ros2 run visual_landmark vpr_viewer
```

화면은 **VPR Dashboard 단일 창**이다. 위쪽은 후방/전방 라이브 SIFT,
아래 왼쪽은 Matches, 아래 오른쪽은 상태/진단이다. 창 크기는 조절할 수 있으며 영상 비율을 유지한다.
입력이 없어도 창을 표시하고 WAITING, 마지막 영상이 멈추면 빨간 STALE와 수신 경과 시간을
계속 갱신한다. source age는 ROS 영상 stamp와 현재 ROS 시각의 차이로 PC 시계 불일치도 포함한다.
RX 번호가 증가하지 않으면 새 영상이 수신되지 않는 것이다. 기존 프레임에 점이 보여도
그 영상이 live임을 뜻하지 않으므로 STALE를 확인한다.

```bash
# 뷰어 부하를 더 낮출 때
ros2 run visual_landmark vpr_viewer --ros-args -p preview_rate_hz:=2.0 -p preview_max_features:=200
# SIFT preview를 끄고 라이브 영상과 생산 매칭 그림만 볼 때
ros2 run visual_landmark vpr_viewer --ros-args -p live_features:=false
```

영상·SIFT preview는 표시 진단일 뿐 motion/mission/correction 토픽을 발행하지 않는다.
Matches는 생산 localizer의 debug image를 그대로 표시하며 VPR 검사·입력·참조가 준비되어야 한다.
`automatic_rejected: stale automatic odometry`는 참조 촬영 실패다. /odometry/filtered 수신과
영상-odom source stamp skew, 두 PC 시계를 확인하고 해결한 뒤 새 mission에서 기록한다.
이 거부를 우회하려고 freshness gate를 완화하지 않는다.

생산 품질 gate·단일 UI 실제 metrics·preview check 중 pause: [VPR_QUALITY_UI.md](VPR_QUALITY_UI.md).
