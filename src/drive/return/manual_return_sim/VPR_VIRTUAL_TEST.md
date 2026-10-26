# RViz 가상 험지 누적오차 + production VPR 시험

가상 영상은 320×240이며 depth는 16UC1 mm 단위다.
production scheduler/localizer는 읽기 전용 telemetry subclass로 실행한다.

이 시험은 **합성 카메라 영상**과 가상 차체/odom을 쓴다. 실제 카메라/모터/CAN은
실행하지 않는다. 이 PC에서 실제 전후방 장치와 로봇 카메라 토픽은 확인되지 않았다.
물리 카메라가 정지한 채 가상 차체만 움직이는 영상을 섞으면 잘못된 검증이 되므로,
이 시험의 camera TF와 영상/depth는 같은 가상 장면/ground truth에서 생성한다.

## 시작

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-up-to manual_return_sim visual_landmark
source install/setup.bash
export ROS_DOMAIN_ID=109
export ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
ros2 launch manual_return_sim vpr_rough_terrain.launch.py
```

도메인 109가 아니면 launch가 거부한다. production의 도메인 99와 섞지 않는다.
이 launch는 cmd_vel을 가상 운동학 노드에 연결하고 실제 CAN 드라이버는 포함하지 않는다.
기존 RViz가 도메인 99에 있으면 이 시험 데이터가 안 보인다. launch가 새 RViz를 연다.

같은 도메인에 같은 시험을 중복 실행하지 않는다. Ctrl+C로 종료 후 재실행하면 새 Record부터 시작한다.

시작을 대기하고 화면을 먼저 살펴보려면 `auto_start:=false`로 실행한다.
그 뒤 도메인 109의 별도 터미널에서:

```bash
ros2 topic pub --once /sim/vpr/start std_msgs/msg/Bool '{data: true}'
```

보정 OFF 대조군:

```bash
ros2 launch manual_return_sim vpr_rough_terrain.launch.py apply_after_debug:=false
```

화면 없이 자동 검증:

```bash
ros2 launch manual_return_sim vpr_rough_terrain.launch.py rviz:=false viewer:=false
```

## 자동 시나리오

1. 노드 준비 후 Record, 원점에서 한 장의 자동 reference 기록, 정지 대기 후 4m outbound.
2. production state machine으로 Return/180도 회전/production Pure Pursuit 복귀.
3. 복귀 중 ground truth에만 횡방향 0.08m/s를 추가해 총 0.35m 오차를 누적한다.
   odom에는 그 미끄러짐이 없다. 단순 경로 이탈이 아니라 추정 위치 자체의 오차다.
4. production RETURN_MANUAL 전환, 정지 후 stationary_debug 요청.
   production SIFT/mutual ratio/depth/PnP/두 프레임 검증으로 correction을 계산한다.
   이 단계는 항상 apply_correction=false. 실패를 성공으로 대체하지 않는다.
5. stationary_debug validated가 실제로 확인됐을 때만 apply_correction을 선택값으로
   설정하고 AUTO 재개한다. scheduler의 재개/거리 요청으로 기존 VPR을 실행한다.
6. ON이면 실제 production correction 토픽을 통해 follower/state machine의 좌표에 적용.
   OFF이면 관측만 수행. FINISHED에서 화면과 결과를 유지한다.

키보드/조이스틱 조작은 필요 없다. 실패하면 failed_* 단계에 머물고 로그를 확인한다.
가상 odometry는 단순 차동구동 적분으로 직접 제공한다. wheel/gyro/reduced_odom
추정기나 실제 토질/구동계 동역학을 검증하는 시뮬레이터는 아니다.

## 화면

- 초록/주황: 기록 경로 / production return 경로.
- 빨강 궤적·구: ground truth. 가상 차체의 실제 위치.
- 파랑 궤적·구: odom. 차체가 믿는 위치.
- 노랑 궤적·구: 실제 발행된 VPR correction으로 변환한 odom 위치.
- status: SYNTHETIC 표시, phase/state, 주입 slip, truth-odom / truth-corrected 오차.
- RViz Image: 가상 rear/front RGB. outbound에서는 전방에 벽이 없어 어두울 수 있다.
- VPR viewer Matches: production matching의 descriptor match / PnP inlier.
  해당 검사가 실행될 때만 갱신된다. 녹색 선만으로 최종 성공을 판단하지 않는다.

기존 sim_debug_viz의 actual driven trace는 **odom 기반 trace**다.
독립 실제 위치는 반드시 새 SIM truth(빨강)로 읽는다.
기록/복귀 경로 자체를 옮기지 않고, 추정 좌표 보정 후 실제 궤적이 어떻게 바뀌는지 본다.

## 기록·진단

```bash
# 같은 도메인 109 / localhost 환경의 별도 터미널
ros2 topic echo /sim/vpr/metrics
ros2 topic echo /sim/vpr/watchdog
ros2 topic echo /visual_landmark/localization_event
```

완료/실패 시 JSON 보고서:

- `/tmp/resku_virtual_vpr_on.json`
- `/tmp/resku_virtual_vpr_off.json`

로그에는 reference_loaded / validated / localized와 실제 planar_correction이 표시된다.
단계가 완료돼야 보고서가 생긴다. 오래된 보고서와 혼동하지 말고 파일 수정 시각도 확인한다.

## 검증 범위와 한계

- 실제 production capture/matching/PnP/동기화/TF/두 프레임/요청 수명과 controller 경로를 사용.
- 성공한 VPR 결과를 ground truth로 직접 만들어 넣지 않는다. 영상과 depth에서 계산한다.
- 보정 수학의 부호·평행 오차 인식·Manual 정지 검사·적용 후 궤적을 확인한다.
- 정확한 가상 intrinsic/extrinsic, 단일 정적 평면의 고대비 텍스처 장면이다.
  서로 다른 다수 reference가 동일 벽을 볼 때의 ambiguity를 이 단일-landmark 시험으로 검증하지 않는다.
- 실제 험지 slip 분포·blur·조명·가림·깊이 잡음·실제 TF 오차·다중 경로 recognition은 별도 시험이다.
- 실제 카메라를 켰다는 뜻이 아니다. 실차 인식 성능/최종 도착 정확도 증명으로 해석하지 않는다.

합성 렌더링 회귀 테스트:

```bash
PYTHONPATH="$PWD/src/drive/return/manual_return_sim:$PWD/src/drive/return/visual_landmark:$PYTHONPATH" \
python3 -m pytest src/drive/return/manual_return_sim/test -q
```

통합 실행 중 발견한 production capture 오류도 수정했다:
localization_ready를 numpy.bool_ 대신 Python bool로 반환해 captured JSON event가
실제로 발행되도록 했다. 매칭/acceptance 기준 변경은 없다.

## 이번 환경에서 실제 실행한 비교 결과

| 항목 | 보정 OFF | 보정 ON |
| --- | --- | --- |
| 주입한 횡방향 slip | 0.350m | 0.350m |
| FINISHED 실제 y | +0.34996m | -0.00140m |
| 실제 correction 발행 | 0회 | 1회(auto_resume) |
| 종료 시 truth와 보정 추정 위치의 차이 | 0.350m | 0.00095m |

두 시나리오 모두 FINISHED까지 실행했다. ON의 stationary_debug는 302 inlier,
Auto 적용은 307 inlier였으며 PnP RMSE는 약 0.38px였다.
종방향 x는 둘 다 약 0.3m에서 기존 goal tolerance로 종료했다.
작은 y/추정 오차를 전체 도착 정확도나 실차 정확도로 해석하지 않는다.
이번 측정 스냅샷은 workspace의 debug_report/vpr_virtual/off.json, on.json에 보관한다.
