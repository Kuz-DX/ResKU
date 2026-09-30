# Tool Manipulator Bringup

실기 팔 제어와 **attach-only** AprilTag tool docking 패키지다. 실제 실행 순서와 Jetson/로컬 토픽 표는 저장소 루트의 [`howtorun.md`](../../../../howtorun.md) `Tool Manipulator — 실기 제어·툴 교체 실행 절차`를 따른다.

- URDF `world`는 차량 IMU 중심이며, `arm_world`는 IMU/world 기준 `(0.1921, 0, 0.235)` m offset이며 지면 기준 높이는 `0.350` m이다. 차체, 회로박스, 대칭 tool case는 fixed visual/collision links로 표시된다.
- `config/hardware.yaml`: U2D2 TTL/RS-485와 RMD CAN, 모든 관절 영점·리밋·register 설정의 단일 원본
- `config/tools.yaml`: tool/fixture collision, TCP, tag, 접근·yaw·직선 motion 설정의 단일 원본
- `config/docking.yaml`: existing vision AprilTag 입력, tag/depth gate, XY servo, 공통 pose의 단일 원본
- `config_validator.py`: 세 원본의 모든 필수 필드를 검사한다. 실패하면 `real_tool_change.launch.py`는 hardware를 시작하지 않는다.
- `real_control.launch.py`: 검사 완료한 `hardware.yaml`으로 real `ros2_control`, controller, MoveIt을 기동한다.
- `real_tool_change.launch.py`: real control + filter + XY servo + dock action + planning scene stack을 기동한다.
- `arm_tag_docking_vision.launch.py`: 수정하지 않은 `vision.apriltag`를 `docking.yaml`의 단일 입력 설정으로 실행한다.

도킹은 `yaw trajectory 성공 → /wrist_yaw_rotation_complete → configured retreat 성공 → /docking_complete → scene attach` 순서다. yaw 성공은 물리 체결이나 scene 부착 검증이 아니다. 별도 체결 센서는 현재 사용하지 않는다.

물리 detach/unlock 절차는 아직 구현되지 않았다. `Dock.mode=1`과 scene detach service는 명시적으로 거부되며 성공으로 보고하지 않는다. 재시작·도킹 실패 뒤 tool 상태가 불명확하면 scene manager는 `UNKNOWN`으로 시작/전환하며, 실물이 비어 있음을 확인한 운영자만 `~/operator_confirm_empty`로 복구할 수 있다.
