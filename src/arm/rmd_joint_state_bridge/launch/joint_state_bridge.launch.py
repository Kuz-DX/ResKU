"""RViz(display.launch.py)와 같이 띄워서 실기 팔의 실측 각도/엔코더를
읽기 전용으로 미러링한다 - 커맨드를 안 보내므로 팔을 손으로 자유롭게
움직이며 자세를 캡처할 때 안전하다.

기본 파라미터는 army_manipulator_ros2_control.xacro의 기본 인자(can_ifname=
can_arm, actuator id 4/5/6, dxl_port_name=/dev/ttyUSB0 등)와 같다 - 실기
캘리브레이션 값(sign/q_offset, position_direction/position_zero_offset)이
그 xacro와 다르면 여기서도 launch argument로 동일하게 넘겨야 한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("can_ifname", default_value="can_arm"),
        DeclareLaunchArgument("shoulder_actuator_id", default_value="4"),
        DeclareLaunchArgument("elbow_actuator_id", default_value="5"),
        DeclareLaunchArgument("wrist_actuator_id", default_value="6"),
        DeclareLaunchArgument("shoulder_sign", default_value="1.0"),
        DeclareLaunchArgument("elbow_sign", default_value="-1.0"),
        DeclareLaunchArgument("wrist_sign", default_value="-1.0"),
        DeclareLaunchArgument("shoulder_q_offset", default_value="0.0"),
        DeclareLaunchArgument("elbow_q_offset", default_value="0.0"),
        DeclareLaunchArgument("wrist_q_offset", default_value="0.0"),
        DeclareLaunchArgument("dxl_port_name", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("dxl_baud_rate", default_value="1000000"),
        DeclareLaunchArgument("base_dxl_id", default_value="0"),
        DeclareLaunchArgument("gripper_dxl_id", default_value="4"),
        DeclareLaunchArgument("base_zero_offset", default_value="-9.314331344042"),
        DeclareLaunchArgument("base_direction", default_value="1.0"),
        DeclareLaunchArgument("base_wraparound", default_value="true"),
        DeclareLaunchArgument("gripper_zero_offset", default_value="-1.375980766733627"),
        DeclareLaunchArgument("gripper_direction", default_value="1.0"),
        DeclareLaunchArgument("gripper_wraparound", default_value="true"),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
    ]

    node = Node(
        package="rmd_joint_state_bridge",
        executable="joint_state_bridge_node",
        output="screen",
        parameters=[{
            "can_ifname": LaunchConfiguration("can_ifname"),
            "shoulder_actuator_id": LaunchConfiguration("shoulder_actuator_id"),
            "elbow_actuator_id": LaunchConfiguration("elbow_actuator_id"),
            "wrist_actuator_id": LaunchConfiguration("wrist_actuator_id"),
            "shoulder_sign": LaunchConfiguration("shoulder_sign"),
            "elbow_sign": LaunchConfiguration("elbow_sign"),
            "wrist_sign": LaunchConfiguration("wrist_sign"),
            "shoulder_q_offset": LaunchConfiguration("shoulder_q_offset"),
            "elbow_q_offset": LaunchConfiguration("elbow_q_offset"),
            "wrist_q_offset": LaunchConfiguration("wrist_q_offset"),
            "dxl_port_name": LaunchConfiguration("dxl_port_name"),
            "dxl_baud_rate": LaunchConfiguration("dxl_baud_rate"),
            "base_dxl_id": LaunchConfiguration("base_dxl_id"),
            "gripper_dxl_id": LaunchConfiguration("gripper_dxl_id"),
            "base_zero_offset": LaunchConfiguration("base_zero_offset"),
            "base_direction": LaunchConfiguration("base_direction"),
            "base_wraparound": LaunchConfiguration("base_wraparound"),
            "gripper_zero_offset": LaunchConfiguration("gripper_zero_offset"),
            "gripper_direction": LaunchConfiguration("gripper_direction"),
            "gripper_wraparound": LaunchConfiguration("gripper_wraparound"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
        }],
    )

    return LaunchDescription(declared_arguments + [node])
