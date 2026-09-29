"""Mock-hardware TCP joystick teleoperation through MoveIt Servo."""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_joy = LaunchConfiguration("launch_joy")
    config = (
        MoveItConfigsBuilder("tool_manipulator", package_name="tool_manipulator_moveit_config")
        .robot_description(mappings={"use_mock_hardware": "true", "use_mesh": "false"})
        .to_moveit_configs()
    )
    controller_config = PathJoinSubstitution([
        FindPackageShare("tool_manipulator_moveit_config"), "config", "ros2_controllers.yaml"])
    from ament_index_python.packages import get_package_share_directory
    servo_path = get_package_share_directory("tool_manipulator_bringup") + "/config/servo.yaml"
    with open(servo_path, encoding="utf-8") as stream:
        servo_config = {"moveit_servo": yaml.safe_load(stream)["/**"]["ros__parameters"]}

    state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen")
    arm_controller = RegisterEventHandler(OnProcessExit(
        target_action=state_broadcaster,
        on_exit=[Node(
            package="controller_manager", executable="spawner",
            arguments=["arm_controller", "-c", "/controller_manager"], output="screen")]))

    return LaunchDescription([
        DeclareLaunchArgument("launch_joy", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[config.robot_description, controller_config], output="screen"),
        state_broadcaster,
        arm_controller,
        Node(package="moveit_servo", executable="servo_node_main", name="servo_node",
             parameters=[servo_config, config.robot_description,
                         config.robot_description_semantic,
                         config.robot_description_kinematics], output="screen"),
        Node(package="joy", executable="joy_node", name="joy_node",
             parameters=[{"dev": LaunchConfiguration("joy_dev"), "deadzone": 0.05,
                          "autorepeat_rate": 50.0}],
             condition=IfCondition(launch_joy), output="screen"),
        Node(package="tool_manipulator_bringup", executable="tcp_joy_teleop.py",
             output="screen"),
    ])
