# **Dynamixel Hardware Interface User Guide**

> **이 워크스페이스의 사용 상태:** 실제 arm 구성은 이 로컬 플러그인으로
> `base_joint`와 `gripper_joint`를 제어합니다. 각 position command에는 Xacro에서
> 관절별 min/max가 전달됩니다. 플러그인은 송신 직전에 NaN/Inf와 범위 초과를
> 검사하고, 작은 경계 초과는 clamp하며 설정 margin보다 큰 초과는 거부합니다.
> 시작 시에는 유효한 실제 위치를 command로 동기화하고, 지속 송신 실패·하드웨어
> 오류·예기치 않은 torque-off는 ros2_control 오류로 반환합니다. 이 최종 검사는
> 수동 및 MoveIt/AUTO 명령원에 동일하게 적용됩니다.

아래는 원본 패키지 문서입니다.

## **1. Introduction**

ROS 2 package providing a hardware interface for controlling [Dynamixel](https://www.dynamixel.com/) motors via the [ros2_control framework](https://github.com/ros-controls/ros2_control). This repository includes the **dynamixel_hardware_interface plugin** for seamless integration with ROS 2 control, along with the [dynamixel_interfaces](https://github.com/ROBOTIS-GIT/dynamixel_interfaces) package containing custom message definitions used by the interface


## 2. **Prerequisites**

This package currently supports ROS 2 Humble, Jazzy, Rolling. Ensure that ROS 2 is properly installed.

- Hardware Requirements:

  - Dynamixel servos
  - USB2 Dynamixel or U2D2 adapter
  - Proper power supply for Dynamixel motors


## **3. Installation**

1. Clone the repository into your ROS workspace:

   ```bash
   cd ~/${WORKSPACE}/src
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
   ```

2. Build the package:

   ```bash
   cd ~/${WORKSPACE}
   colcon build
   ```

3. Source your workspace:

   ```bash
   source ~/${WORKSPACE}/install/setup.bash
   ```


## 4. Currently Used Packages

This project integrates with the following ROS 2 packages to provide extended functionality:

- **[open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)**
  A ROS-based open-source software package designed for the **OpenManipulator-X and OMY**. It provides essential features like motion planning, kinematics, and control utilities for seamless integration with ROS 2 environments.

## 5. Configuration

To effectively use the **Dynamixel Hardware Interface** in a ROS 2 control system, you need to configure specific parameters in your `ros2_control` hardware description file. Below is a concise explanation of the key parameters, illustrated with examples from the **OpenManipulator-X ROS 2 control.xacro** file.

1. **Port Settings**: Define serial port and baud rate for communication.
2. **Hardware Setup**: Configure joints and transmissions.
3. **Joints**: Control and monitor robot joints.
4. **GPIO**: Define and control Dynamixel motors.
yaw
#### **1. Port and Communication Settings**

These parameters define how the interface communicates with the Dynamixel motors:

- **`port_name`**: Serial port for communication.

- **`baud_rate`**: Communication baud rate.

- **`error_timeout_ms`**: Timeout for communication errors (in milliseconds).

#### **2. Hardware Configuration**

These parameters define the hardware setup:

- **`number_of_joints`**: Total number of joints.

- **`number_of_transmissions`**: Number of transmissions.

- **Transmission Matrices**: Define joint-to-transmission mappings.

#### **3. Joint Configuration**

Joints define the control and state interfaces for robot movement:

##### **Key Attributes**

- **`name`**: Unique joint name.
   Example: `${prefix}joint1`

##### **Sub-Elements**

1. **`<command_interface>`**: Sends commands to joints.

   ```xml
   <command_interface name="position">
   ```

2. **`<state_interface>`**: Monitors joint state data.

   ```xml
   <state_interface name="position"/>
   <state_interface name="velocity"/>
   <state_interface name="effort"/>
   ```


#### **4. GPIO Configuration**

The GPIO tag is used to define the configuration of Dynamixel motors in a robotics system. It serves as a declarative structure to set up motor-specific parameters, command interfaces, and state monitoring capabilities. This allows seamless integration of Dynamixel hardware with software frameworks.


##### **Key Attributes**

- **`name`**: A unique identifier for the motor configuration (e.g., `dxl1`).
- **`ID`**: The unique ID assigned to the motor in the Dynamixel network (e.g., `11`).


##### **Sub-Elements**

1. **`<param>`**: Specifies motor-specific settings. These parameters correspond to the properties of the Dynamixel motor, such as its type, control mode, or PID gain values.

   ```
   <param name="type">dxl</param>
   ```

2. **`<command_interface>`**: Defines the control commands that can be sent to the motor. For example, setting the desired goal position.

   ```
   <command_interface name="Goal Position"/>
   ```

3. **`<state_interface>`**: Specifies the state feedback interfaces to monitor real-time motor data, such as position, velocity, and current.

   ```
   <state_interface name="Present Position"/>
   <state_interface name="Present Velocity"/>
   <state_interface name="Present Current"/>
   ```

##### **Example GPIO Configuration**

Below is an example of a fully defined GPIO configuration for a Dynamixel motor. This example demonstrates how to configure a motor with ID `11`, define command interfaces, monitor state data, and set additional parameters such as PID gains and drive mode.

```
<gpio name="dxl1">
  <param name="type">dxl</param>
  <param name="ID">11</param>
  <command_interface name="Goal Position"/>
  <state_interface name="Present Position"/>
  <state_interface name="Present Velocity"/>
  <state_interface name="Present Current"/>
  <param name="Position P Gain">800</param> <!-- Proportional gain for position control -->
  <param name="Position I Gain">100</param> <!-- Integral gain for position control -->
  <param name="Position D Gain">100</param> <!-- Derivative gain for position control -->
  <param name="Drive Mode">0</param> <!-- 0: Clockwise, 1: Counterclockwise -->
</gpio>
```

##### **Dynamixel Control Table Reference**

The Dynamixel hardware interface uses control tables, defined in model-specific files such as `xm430_w350.model`, to configure and interact with the motor's internal settings. These control tables map hardware parameters to specific memory addresses and data types, enabling fine-grained control and monitoring.

Example from `xm430_w350.model`:

```
[Control Table]
Address   Size    Data Name
0         2       Model Number
2         4       Model Information
6         1       Firmware Version
7         1       ID
...
```

##### **Usage**

- The control table specifies the internal memory layout of the Dynamixel motor.
- For instance, you can set the motor ID at address `7`, or configure firmware-specific options at address `6`.

These settings can be defined within the GPIO configuration or dynamically updated through commands based on the control table schema.

This professional explanation highlights the flexibility and precision of the Dynamixel hardware interface, empowering developers to fully utilize their motor's capabilities within a structured framework. For further details, refer to the [official Dynamixel e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-eeprom-area).


## **6. Usage**

Ensure the parameters are configured correctly in your `ros2_control` YAML file or XML launch file.

- Example Parameter Configuration

  ```xml
  <ros2_control>
      <param name="dynamixel_state_pub_msg_name">dynamixel_hardware_interface/dxl_state</param>
      <param name="get_dynamixel_data_srv_name">dynamixel_hardware_interface/get_dxl_data</param>
      <param name="set_dynamixel_data_srv_name">dynamixel_hardware_interface/set_dxl_data</param>
      <param name="reboot_dxl_srv_name">dynamixel_hardware_interface/reboot_dxl</param>
      <param name="set_dxl_torque_srv_name">dynamixel_hardware_interface/set_dxl_torque</param>
  </ros2_control>
  ```

#### Topic and Service Descriptions

##### 1. **dynamixel_state_pub_msg_name**

- **Description**: Defines the topic name for publishing **the Dynamixel state.**

- **Default Value**: `dynamixel_hardware_interface/dxl_state`


##### 2. **get_dynamixel_data_srv_name**

- **Description**: Specifies the service name for retrieving Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/get_dxl_data`

##### 3. **set_dynamixel_data_srv_name**

- **Description**: Specifies the service name for setting Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_data`

##### 4. **reboot_dxl_srv_name**

- **Description**: Specifies the service name for rebooting Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/reboot_dxl`

##### 5. **set_dxl_torque_srv_name**

- **Description**: Specifies the service name for enabling or disabling torque on Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_torque`


## **7. Contributing**

We welcome contributions! Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) to submit issues or pull requests.


## **8. License**

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
