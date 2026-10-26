# MG996R 도킹 서보

MG996R 신호선은 Arduino D9에 연결합니다. 서보 전원은 전류를 감당하는 외부 5–6 V 전원으로 공급하고 Arduino와 GND를 공통으로 연결합니다. 일반 MG996R은 각도형 서보이므로 `analogWrite(9, 250)` 대신 Arduino `Servo` 라이브러리를 사용합니다.

`DockingMotor.ino`를 업로드하면 시작 상태 `docked=false`에서 180°를 명령합니다. `/docking`에 `true`를 한 번 보내면 90°(체결), 다시 한 번 보내면 180°(해지)를 명령합니다. 각 명령 3초 뒤 `/docked`에 각각 `true`, `false`를 발행합니다. 3초는 대기 시간이며 실제 도착을 측정하지 않습니다. `/docking`의 `false`는 동작하지 않습니다. 기존 Arm 코디네이터의 `/arm/docking`, `/arm/docked`도 지원합니다.

노드 실행(저장소 루트에서):

```bash
source /opt/ros/humble/setup.bash
python3 src/arm/tool_manipulator_bringup/scripts/arduino_docking_bridge.py --ros-args -p port:=/dev/ttyACM0
```

패키지를 이미 빌드한 환경이라면 `ros2 run tool_manipulator_bringup arduino_docking_bridge.py --ros-args -p port:=/dev/ttyACM0`도 사용할 수 있습니다.

다른 터미널에서 상태 확인:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /docked
```

별도 터미널에서 체결 명령 1회(90°, 약 3초 후 `data: true`):

```bash
ros2 topic pub --once /docking std_msgs/msg/Bool '{data: true}'
```

위 명령이 끝나고 3초 이상 지난 뒤 같은 명령을 다시 실행하면 해지(180°, `data: false`)합니다. 테스트할 때도 한 번에 하나씩 보내세요. 아래 명령은 동작하지 않는 `false` 입력 확인용입니다.

```bash
ros2 topic pub --once /docking std_msgs/msg/Bool '{data: false}'
```

보정용 `MG996R_Calibration.ino`가 아니라 **`DockingMotor.ino`를 다시 업로드**해야 ROS 명령이 동작합니다. 시리얼 모니터와 ROS 브리지는 같은 포트를 동시에 열 수 없습니다. Arduino 재시작 시 180°를 명령하고 상태를 `false`로 초기화합니다. `docked`는 위치 센서 판정이 아니라 명령 후 3초 경과에 따른 추정값입니다.
