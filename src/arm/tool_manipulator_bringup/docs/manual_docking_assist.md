# manual.launch.py의 수동 도킹 보조

`src/arm/tool_manipulator_bringup/launch/manual.launch.py`가 실행하는 `manual.py`에
툴0·툴1 전용 보조를 연결했다. 사용자가 도킹 근처까지 일반 수동 조작으로 접근하고,
L2+△(툴0) 또는 L2+○(툴1)를 한 번 눌러 보조를 선택한다. 선택 버튼과 L2를 놓고
모든 조작을 중립으로 둔다. 이후에도 기존 manual 조작을 계속 사용한다.
선택 이후 촬영된 유효한 태그 입력 3개를 확인하면 이동 보정을 적용한다.
태그 획득 대기 중에는 보정 없이 일반 수동 조작이 가능하다. L2는 계속 잡지 않는다.

보조 선택으로 저장된 tool0/tool1 관절 경로를 실행하지 않는다. `ik_manual.launch.py`,
`ik_manual_test.launch.py`의 도킹 구현과 툴2 경로는 별도다.

## 보조 활성화 전후 동일한 키 매핑

도킹 보조를 활성화해도 TCP 조작 모드로 전환하지 않는다. 별도의 조이스틱→TCP
속도 변환을 제거했고, 보조 활성화 여부와 관계없이 `Manual.command_velocity()`와
`Manual.run_operator()`에서 같은 키 해석·관절 목표 계산을 사용한다.

| 입력 | 일반 수동 / 보조 활성화 후 공통 동작 |
| --- | --- |
| 오른쪽 스틱 위/아래 (축 4) | 기존 RMD 어깨·팔꿈치·손목 피치 협조 동작 |
| 왼쪽 스틱 왼쪽/오른쪽 (축 0) | 손목 요 직접 회전 |
| 왼쪽 스틱 위/아래 (축 1) | 단독 동작 없음 |
| L1/R1 | 베이스 양/음 직접 회전 |
| D-pad 왼쪽/오른쪽 (축 6) | 손목 롤 직접 회전 |
| D-pad 위/아래 (축 7) | 단독 동작 없음 |
| ○ 유지 + D-pad 왼쪽/오른쪽 | EE 양/음 속도 명령 |
| ○ 유지 + D-pad 위/아래 | 손목 피치 직접 조작 |
| ○ 유지 + 왼쪽 스틱 위/아래 | 현재 말단 피치를 유지하며 기준 좌표계 상하 이동 |
| 네모 | 기존 HOME 동작. 보조를 해제하고 TCP 준비 자세→시작 자세 이동 |
| HOME 중 네모 재입력 | 기존 HOME 취소 |
| Options | 기존 drive/arm 전환. 보조도 해제 |
| L2+△ / L2+○ | 툴0 / 툴1 보조 선택; 같은 조합 재입력은 해제 |
| L2+× | 기존 툴2 경로 요청. 활성 보조는 해제 |

○의 기존 배타 조작 계층도 유지한다. ○를 누르면 EE·피치·상하 이동을 조작하며,
동시에 들어온 기존 베이스·롤·요·전후 입력은 원래 manual 코드대로 억제한다.

## AprilTag가 보조하는 부분

오른쪽 스틱의 기존 RMD 협조 입력을 먼저 관절 속도로 계산하고, 그 결과로 발생하는
실제 말단 속도가 태그의 도킹 축을 향할 때 횡오차·자세 보정을 추가한다.
스틱 부호를 TCP 전진/후퇴로 재해석하지 않는다. 기존 수동 방향에 따라 도킹 쪽으로
접근하는 입력을 사용한다. 후퇴 입력은 기존 수동 명령 그대로 전달한다.

베이스/롤/요 직접 조작 또는 ○ 조합을 사용하면 해당 수동 조작이 우선하며 그 주기의
태그 보정을 생략한다. 기존에 할당되지 않은 스틱·D-pad 방향에는 새 동작을 배정하지
않는다. 입력이 중립이면 보정도 멈춘다. EE도 ○+D-pad로 계속 조작할 수 있다.

보정이 적용되는 접근 동작에는 횡오차 5 mm, 자세오차 0.08 rad의 정렬 조건을 적용한다.
정렬이 부족하면 원래 접근 동작을 보류하고 제한된 보정만 수행한다. 원래 이동의
전진 성분만 제거해서 큰 횡방향 이동이 남지 않도록 전체 접근 속도를 함께 제한한다.
30 mm 이내에서 접근 속도를 줄이고 기준면 1 mm 앞에서 전진을 제한한다.
이 정렬·감속·전진 제한은 태그 보정이 적용되는 RMD 접근 동작에 대한 것이며,
우선권을 가진 직접 관절/○ 조작을 자동 경로로 바꾸지 않는다.
`reached`는 설정한 목표에 대한 제어 상태다. 체결 펄스·자동 후퇴·BACK 복귀는 수행하지 않는다.

태그의 마지막 유효 입력이 0.25초를 넘거나, 조이스틱·관절 피드백이 끊기거나,
focus가 바뀌거나, fault/컨트롤러 오류가 발생하면 보조를 해제하고 위치를 유지한다.
중립 확인 후 L2 조합으로 다시 선택해야 한다. 태그 재등장만으로 보조가 재개되지 않는다.
태그를 5초 안에 획득하지 못하면 선택을 취소한다.

기존 `manual.launch.py`의 충돌 검사 비활성 상태는 동일하다. 가상 테스트는 실제
접촉·기구 공차·카메라 인식·물리 충돌 회피를 검증하지 않는다.

## 입력과 구현

- 기본 실제 입력: `/arm/apriltag/centers`의 JSON `std_msgs/String`,
  `/arm/camera/color/camera_info`의 `CameraInfo`.
- 광학 프레임: `arm_camera_color_optical_frame`. 이미지 시각의
  `base_actuator <- optical_frame` TF와 기존 코너 기반 pose solver를 사용한다.
  카메라 내부 파라미터나 누락된 optical TF를 임의로 생성하지 않는다.
- 카메라와 검출기는 이 launch에서 시작하지 않는다. 기존 카메라/검출기 입력을 구독한다.
- `manual_docking_core.py`: 태그 입력 수명, 획득 상태, 기존 수동 속도에 대한 보정, 6축 differential IK.
- `manual_docking.py`: ROS 입력, 버튼 선택, 중립 확인, 기존 위치 controller 연결.
- 6축 전체의 속도·관절 한계·목표 lead 제한을 함께 적용한다. IK에 축방향 속도 제약을
  넣고 실제 FK 증분도 검사하여 횡/자세 보정이 임의 삽입으로 바뀌는 것을 제한한다.
- `/manual/diagnostics`의 `docking_assist`에 상태, 툴, 준비 여부, 입력 거부 이유,
  횡/축/자세 오차와 가상 입력 여부를 기록한다. `input_mapping`은 항상 `manual`이다.
- 서비스 해제: `ros2 service call /manual/docking/cancel std_srvs/srv/Trigger '{}'`.

launch 인자 `docking_assist_config`, `docking_camera_frame`,
`docking_camera_info_topic`, `docking_centers_topic`으로 설정과 입력을 지정한다.

## 임의값 및 미검증 적용값

설정 파일은 `config/manual_docking_assist.yaml`이다. 아래 값은 코드/설정에
`ASSUMED / 임의값`으로 표시했다. 실기에서 계측한 새 값은 없다.

| 항목 | 임의 적용값 |
| --- | --- |
| 툴0 태그→목표 TCP 위치 | `[-0.0457, 0, -0.05157]` m |
| 툴1 태그→목표 TCP 위치 | `[-0.0457, 0, -0.0500]` m |
| 두 툴 태그→목표 TCP quaternion | `[0, 0, 0, 1]` (xyzw) |
| 보정이 최대 강도가 되는 기존 수동 접근 속도 | 0.010 m/s (`assist_full_speed_m_s`); 조이스틱 속도 설정이 아님 |
| 횡보정 / 자세보정 gain | 각각 0.8 /s |
| 횡보정 / 자세보정 속도 상한 | 0.003 m/s / 0.030 rad/s |
| 보조 관절 속도 상한 | 0.060 rad/s (기존 관절별 상한도 적용) |
| DLS damping / 회전 가중 길이 | 0.020 / 0.20 m |
| 보조 허용 목표 거리 / 자세오차 | 0.15 m / 0.60 rad |
| 전진 허용 횡 / 자세오차 | 0.005 m / 0.08 rad |
| 기준면 앞 정지 거리 / 감속 거리 | 0.001 m / 0.030 m |
| 태그 유효 시간 / 획득 제한 시간 | 0.25 s / 5.0 s |
| 연속 안정 관측 수 | 3 |
| 태그 위치 / 회전 점프 제한 | 0.010 m / 0.10 rad |
| vision decision margin / reprojection 상한 | 40.0 / 1.5 px |
| vision 최소 변 길이 / PnP solution gap | 30.0 px / 0.05 px |
| vision 중복해 위치 / 회전 허용오차 | 0.002 m / 0.02 rad |

태그→TCP 오프셋은 기존 `tool_docking.yaml`의 참고값을 가져왔으나 새 모드에서
검증하지 않았으므로 임의 적용값으로 취급했다. 특히 회전 identity는 **raw PnP 태그축과
원하는 TCP축이 일치한다는 가정**이다. 인쇄 태그 방향과 실제 좌표 변환을 확인해야 한다.
태그 ID 0/1, family `tag36h11`, 크기 20 mm는 저장소의 기존 설정을 사용했다.
하드웨어 zero/soft limit와 URDF 기구 길이는 변경하지 않았다.

## 가상 검증 재현

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
./build.bash --packages-select tool_manipulator_bringup --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/resku_docking_ros_logs \
  python3 src/arm/tool_manipulator_bringup/test/manual_docking_mock_smoke.py
```

스크립트는 정확히 `manual.launch.py real_hardware:=false launch_joy:=false
docking_mock_input:=true`를 실행하고 종료 시 정리한다. 다른 팔 launch가 없는
격리된 ROS 도메인에서 실행한다. 실제 하드웨어와 가상 태그의 조합은 launch와 노드
두 곳에서 거부한다. 정상 `manual.launch.py`는 가상 태그 입력을 구독하지 않는다.

가상 입력은 `/manual/docking/mock_tag_pose`의 JSON String이다. 이미지 인식을
우회하고 `{synthetic: true, stamp: {sec, nanosec}, frame_id: "base_actuator",
id, family, xyz_m, xyzw}` 형식의 포즈를 사용한다. 반복 입력도 타임스탬프가 증가해야 한다.

가상 초기 관절값은 `[0, 0.4, 0.8, -0.7, 0, 0]` rad, EE 0 rad다.
툴0 목표는 선택 시 TCP 기준 `[0.008, 0, 0.040]` m,
툴1 목표는 `[0.006, 0, 0.030]` m를 더한 자세다. 이 테스트 전용 값도 임의값으로
표시했다. 태그 포즈는 각 목표와 설정된 태그→TCP 변환에서 역산한다.
카메라 내부 파라미터와 카메라 외부 변환의 가상 측정값은 사용하지 않는다.

단위 테스트는 `test/test_manual_docking.py`이며 실제 URDF의 6축 Jacobian,
좌표계 변환, 기존 키 매핑·EE 명령 동일성, 접근 제한, 중립 유지, 태그 입력 유효기간, HOME·재선택 및 모의 폐루프를
검증한다. 실제 카메라 인식 성공 여부/인식률은 판정하지 않는다.

통합 결과는 `/tmp/resku_manual_docking_mock_report.json`, launch 로그는
`/tmp/resku_manual_docking_mock_194.log`에 저장된다.

2026-11-03 검증에서 모의 controller 연결과 툴0·툴1 보조, 잘못된 ID/오래된 입력 거부,
중립 유지, 기존 수동 접근에 대한 보정, 후퇴, 기존 HOME 동작, 태그 및 조이스틱 유실을
포함한 통합 검사 9개를 통과했다. 단위·회귀 검사에서는 201개가 통과했다.
기존 `test_manual.py` 7개 실패는 변경 전 HEAD 코드에서도 동일하게 재현됐다.
기존 저장 자세/각도 기대값이 현재 hardware.yaml 제한값과 일치하지 않는 항목이며,
이번 도킹 변경으로 하드웨어 제한을 수정하지 않았다.
