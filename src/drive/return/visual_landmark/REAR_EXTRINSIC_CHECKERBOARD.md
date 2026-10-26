# 후방캠 체커보드 외부 보정 준비

카메라와 차체를 고정하고 평평한 10×7칸 보드를 후방 영상에 전체가 보이게 놓는다.
내부 코너 9×6, 실측 한 칸 15mm. 기존 intrinsic은 시험 수준이므로 이 도구의 출력도
검증 전에는 후보이다. TF나 모터/보정 명령은 발행하지 않는다.

## 화면 조작과 보드 배치

카메라는 건드리지 말고 보드를 단단한 지지대에 고정한다. 큰 반사·흔들림·가림을 피하고,
코너가 선명히 보이는 거리로 놓는다. 중앙뿐 아니라 다른 위치에서도 독립 측정해 검증한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run visual_landmark rear_extrinsic preview \
 --calibration src/vision/camera/config/calibration/apc850_rear_preview_trial_1280x720.yaml \
 --output-dir debug_report/rear_extrinsic
```

후방 드라이버가 먼저 실행 중이어야 한다. 영상은 ROS에서 구독하므로 카메라 장치를
미리보기 프로세스가 중복 점유하지 않는다. 창은 영상 수신 후 표시된다.

- SPACE: 화면 고정/재개.
- S: 검출된 영상을 저장하고 화면 고정. PNG 원본·라벨 영상·코너/K/D/카메라→보드 후보 JSON 저장.
- Q: 창 종료.

P0는 빨강(내부 코너 0), PX는 파랑(8), PY는 초록(45).
P0→PX는 120mm, P0→PY는 75mm이다. **종이 외곽점이 아니다.**
대칭 체커보드는 물리적 원점 표식이 없으므로 매번 화면에 고정된 라벨과 실제 점을
대응시켜 측정해야 한다. 보드가 뒤집히거나 크게 회전한 뒤 라벨 순서가 같다고 가정하지 않는다.
필요하면 검출 영역 밖 여백에 물리적 표시를 붙인다. 저장 후 보드/차체가 움직이면 그 survey를 재사용하지 않는다.

## 독립적인 차체 기준 survey

세 점의 **차체 base_link 기준** x/y/z(m)를 측정한다. x 전방, y 좌측, z 상방.
카메라에서 보드까지의 거리만으로는 차체 기준 extrinsic이 정해지지 않는다.
base_link가 실제로 어디인지 먼저 확인한다. 차체가 기울어졌다면 축을 바닥 축으로 착각하지 않는다.

`debug_report/rear_extrinsic/board_survey_template.yaml`의 null을 실측값으로 채워 별도 이름으로 저장:

```yaml
p0_base_m: [null, null, null]
px_base_m: [null, null, null]
py_base_m: [null, null, null]
```

이 null은 미측정 표시이며 0으로 대체하면 안 된다. 도구는 3점의 길이와 직교성을 검사한다.
120mm/75mm의 짧은 기준선에서는 수 mm 측정 오차도 큰 각도 오차가 되므로 자/직각자/수평계를
활용한다. 가능하면 더 큰 정확한 보드/측정 지그를 사용한다. 검사를 통과해도 정밀 측정을 증명하지 않는다.

## 후보 계산 및 검증

```bash
ros2 run visual_landmark rear_extrinsic solve \
 --observation debug_report/rear_extrinsic/sample_ACTUAL.json \
 --survey debug_report/rear_extrinsic/board_survey_measured.yaml \
 --output debug_report/rear_extrinsic/rear_optical_candidates.yaml
```

계산: `base_from_camera = base_from_board * inverse(camera_from_board)`.
IPPE의 양의 depth 후보를 모두 저장하고 reprojection RMS 순서로 정렬한다.
평면 포즈에는 복수 해가 있을 수 있다. 가장 작은 재투영 오차만으로 정답을 확정하지 않는다.
다른 측정 보드 자세에서도 같은 camera 위치/회전이 재현되는지, 뒷면 중심 위치
(-0.316627, 0, 0.650674)m 및 정후방/약 3도 하향이라는 실제 장착과 부합하는지 확인한다.
약 3도는 사용자 관찰값이지 보정된 optical RPY 숫자가 아니다. roll은 미확정이다.

재투영·독립 반복·측정 불확실성·실제 VPR known-offset 시험을 확인한 뒤에만 후방 optical TF를 확정한다.
그 전에는 visual_apply_correction=false를 유지한다. 저장 후보를 자동으로 TF에 반영하지 않는다.
