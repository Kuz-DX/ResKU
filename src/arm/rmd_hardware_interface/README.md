# rmd_hardware_interface

`rmd_sdk`를 ROS 2 `ros2_control` actuator interface로 연결합니다. `robot_arm_description`의 각 RMD 관절은 아래 pluginlib 식별자를 사용합니다.

```xml
<plugin>rmd_hardware_interface/MyActuatorRmdHardwareInterface</plugin>
```

관절별 CAN interface, actuator ID, 속도 제한과 timeout은 URDF/Xacro의 hardware parameter로 전달됩니다. 일반적인 관절 추가는 이 패키지가 아니라 `robot_arm_description`과 `robot_arm_bringup`에서 수행합니다.

실기 position interface는 상위 명령원이 수동인지 AUTO인지와 무관하게 마지막
하드웨어 안전계층을 통과합니다. 유효한 최초 feedback 전에는 제어를 시작하지 않고,
NaN/Inf와 관절 범위 밖 명령을 송신 직전에 차단하며, `feedback_timeout_ms` 동안
정상 feedback이 갱신되지 않으면 ros2_control에 오류를 반환합니다. 리밋을 조금
넘은 명령은 경계로 clamp하고, `position_limit_rejection_margin`보다 크게 벗어난
명령만 오류로 거부하여 정상적인 경계 운용의 불필요한 정지를 피합니다.

## Upstream documentation

### MyActuator RMD X-series Hardware

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview
This package holds the [**`ros2_control` integration**](https://control.ros.org/humble/index.html) for the [**MyActuator RMD-X actuator series**](https://www.myactuator.com/rmd-x) in the form of a [hardware component](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html). The hardware interface is based on the [C++ driver that I have written for these actuators](https://github.com/2b-t/myactuator_rmd).

For using it add the following lines to your URDF refering to the joint of interest `joint_name`:

```xml
<ros2_control name="${some_name}" type="actuator">
  <hardware>
    <plugin>rmd_hardware_interface/MyActuatorRmdHardwareInterface</plugin>
    <param name="ifname">${ifname}</param>
    <param name="actuator_id">${actuator_id}</param>
    <param name="torque_constant">${torque_constant}</param>
    <!-- Optional: Low-pass filters for velocity and effort (0 < alpha <= 1); defaults to no filter -->
    <param name="velocity_alpha">0.1</param>
    <param name="effort_alpha">0.1</param>
    <!-- Optional: Cycle time of the asynchronous thread; defaults to 1ms (1000Hz) -->
    <param name="cycle_time">1</param>
  </hardware>
  <joint name="${joint_name}">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

The `ifname` has to correspond to the name of the CAN interface as shown by `$ ifconfig` (e.g. `can0`) and the `actuator_id` to the ID of the actuator (e.g. `1`). The `torque_constant` is required for controlling the actuator over its effort interface and depends on the actuator type. Furthermore optional [low-pass filters](https://en.wikipedia.org/wiki/Low-pass_filter) (by means of the filter coefficient `alpha`) for the read velocity and effort can be activated. The correlation between the ratio of [sample](https://en.wikipedia.org/wiki/Sampling_(signal_processing)) (in our case the update rate of the hardware interface) and [cut-off frequency](https://en.wikipedia.org/wiki/Cutoff_frequency) is given by `sample_frequency/cutoff_frequency = (1-alpha)*2*pi/alpha`. For `alpha = 0.07` this ratio corresponds to `125`, meaning if the hardware interface is running at 1000 Hz any oscillations with a higher frequency than around 8 Hz will be filtered out.

| Without low pass filter (corresponds to `alpha = 1.0`)       | With low pass filter (`alpha = 0.07`)                        |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![Effort and velocity without low-pass filter](./media/without_low_pass_filter.png) | ![Effort and velocity with low-pass filter](./media/with_low_pass_filter.png) |

Similarly the cycle-time for the asynchronous thread interfacing the actuator through CAN can be specified. For examples refer to the `rmd_sdk_description` package.
