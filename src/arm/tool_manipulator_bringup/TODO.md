# Tool-manipulator control TODO

## 실행 순서

1. **[구현 완료 · 실측값 대기] cancel/fault의 실제 trajectory 취소와 정지 확인**
   - executor는 활성 `FollowJointTrajectory`를 `MoveIt2.cancel_execution()`으로 취소한다.
   - 취소 action의 종료와 최신 6축 velocity가 설정 threshold 아래임을 확인할 때까지
     `hold:..._unconfirmed` 상태를 유지한다.
   - `docking.yaml`의 `cancel_stop_timeout_sec`, `stopped_velocity_rad_s`는 실측 후 입력한다.
2. **[다음] tool 2 collision/TCP 일치**: `ee_output_link` 기준 tool 2 TCP의 +Y 60 mm offset을
   attached collision에 반영하고 RViz/실물 치수로 검증한다.
3. **[다음] active tool TCP를 MoveIt/Servo의 실제 EE frame으로 전환**: `/active_tool_tcp` 단순
   상태 토픽이 아니라 planning/Servo가 사용하는 TCP가 되게 한다.
4. **[다음] fixed 10초 대기 제거**: controller, MoveGroup, Servo, scene service, camera, 최신
   joint state를 모두 확인하는 readiness gate를 만든다.
5. **[다음] arm command ownership**: 도킹 중 teleop·recorded replay·다른 trajectory goal을 거부한다.
6. **[다음] FK/TF 기반 tag 목표점과 실측 vision gate를 설정한다.**
7. **[다음] planning 속도/가속도 제한을 hardware.yaml과 일치시키고 phase timeout을 실측한다.**

## 현재 구현됨

- `hardware.yaml` + `tools.yaml` + `docking.yaml` 통합 fail-closed 사전검사. `null`, placeholder, 비활성 selected tool, 빈 fixture, 누락 tag/pose/collision/yaw/descent/retreat/mission pose, tag/depth/servo gate를 모두 launch 전에 보고한다.
- 통과 전에는 `tool_change.launch.py`가 `controller_manager`, U2D2, CAN 포트를 시작하지 않는다.
- attach action은 접근 pose → filter/depth gate → XY servo → tool별 straight descent → MoveIt yaw delta → one-shot `/wrist_yaw_rotation_complete` → configured straight retreat → one-shot `/docking_complete` 순서로 실행한다.
- `/docking_complete`만 `tool_scene_manager`의 attach trigger다. attach 뒤 collision/TCP/active tool 상태를 갱신한다.
- failure/cancel/fault/filter failure/회전 또는 후퇴 실패는 hold하고 완료 토픽·scene attach를 막는다. 자동 retreat fallback은 없다.
- 기존 `vision.apriltag.py`와 `camera/launch/cameras.launch.py`는 수정하지 않고 설정된 실제 토픽을 구독한다.
- physical detach는 명시적 미지원: `Dock.mode=1` 및 scene detach service가 거부된다.

## 실측 후 설정에 입력할 항목

- Dynamixel: `current_limit_raw`, `velocity_limit_raw`, `profile_acceleration_raw`, `profile_velocity_raw` 및 필요 시 영점/soft limit 재확인.
- RMD: sign, ROS zero offset, soft limits, 속도·전류·보호 조건. 값의 단위·방법은 `howtorun.md` 표를 따른다.
- 각 tool: tag ID, 6축 coarse pose, fixture/tool primitive collision, yaw delta, descent/retreat frame·direction·distance·speed·timeout.
- vision/servo: tag side, expected camera frame, age/margin/depth gate, 목표 XY와 수렴값.
- 공통 `docking_wait`, mission wait 6축 pose.

## 반드시 할 bench 검증

1. 모든 관절의 저속 단독 방향·영점·soft limit 및 U2D2 TTL/RS-485 feedback 안정성.
2. 각 tool의 approach, XY servo 부호, descent/retreat frame 방향, yaw 체결 각도.
3. 실제 체결 성공 여부. 현재 yaw action 성공은 센서 검증이 아니므로 물리 retention을 별도로 관찰·기록한다.
4. 통신 단절, stale state, controller fault, cancel에서 torque/hold 동작과 사람이 안전하게 복구할 절차.
5. 물리 detach/unlock 센서·절차를 설계/검증한 뒤에만 detach 상태기를 추가한다.
