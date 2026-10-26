# 도킹 모터

`DockingMotor.ino`를 Arduino에 업로드합니다. 모터 드라이버의 PWM/enable 입력을 D9, 방향 입력을 D8에 연결합니다. 모터 전원은 드라이버에 별도로 공급하고 Arduino와 GND를 공통으로 연결합니다. 모터를 Arduino 핀에 직접 연결하지 마세요. 방향이 반대면 D8의 HIGH/LOW를 서로 바꾸세요.

ROS에서는 `colcon build --packages-select tool_manipulator_bringup` 후 다음 노드를 실행합니다.

```bash
ros2 run tool_manipulator_bringup arduino_docking_bridge.py --ros-args -p port:=/dev/ttyACM0
```

`/docking` 또는 기존 Arm 코디네이터의 `/arm/docking`에 `std_msgs/msg/Bool` `true` 상승 에지를 보내면 현재 도킹 상태에 따라 D9 PWM 250으로 3초 구동합니다. 완료 후 `/docked`와 `/arm/docked`에 결과를 발행합니다. 중복 명령은 구동 중 무시됩니다. 전원 투입 또는 보드 재시작 시 상태는 `false`로 초기화되므로 실제 기구 상태를 확인한 뒤 시작해야 합니다. 이 상태는 센서 판정이 아니라 3초 구동 완료에 따른 추정값입니다.
