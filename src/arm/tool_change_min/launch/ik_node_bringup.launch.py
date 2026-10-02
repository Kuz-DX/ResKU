"""Bring up the current tool manipulator and one-shot supply grasp IK flow."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock_hardware")
    hardware = LaunchConfiguration("hardware_yaml")
    tool_id = LaunchConfiguration("tool_id")
    use_mesh = LaunchConfiguration("use_mesh")

    mock_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("tool_manipulator_bringup"), "launch", "mock.launch.py",
        ])),
        launch_arguments={"tool_id": tool_id, "use_mesh": use_mesh}.items(),
        condition=IfCondition(use_mock),
    )
    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("tool_manipulator_bringup"), "launch", "real_control.launch.py",
        ])),
        launch_arguments={
            "hardware_config": hardware,
            "tool_id": tool_id,
            "use_mesh": use_mesh,
            # This node owns the startup grasp-wait motion.
            "move_home_on_start": "false",
        }.items(),
        condition=UnlessCondition(use_mock),
    )

    grasp_node = Node(
        package="tool_change_min",
        executable="supply_grasp_ik_node.py",
        name="supply_grasp_ik_node",
        output="screen",
        parameters=[{
            "hardware_yaml": hardware,
            "target_topic": ParameterValue(
                LaunchConfiguration("target_topic"), value_type=str),
            "planning_frame": ParameterValue(
                LaunchConfiguration("planning_frame"), value_type=str),
            "arm_duration_sec": ParameterValue(
                LaunchConfiguration("arm_duration_sec"), value_type=float),
            "ee_duration_sec": ParameterValue(
                LaunchConfiguration("ee_duration_sec"), value_type=float),
            "use_fixed_target_z": ParameterValue(
                LaunchConfiguration("use_fixed_target_z"), value_type=bool),
            "base_height_m": ParameterValue(
                LaunchConfiguration("base_height_m"), value_type=float),
            "box_height_m": ParameterValue(
                LaunchConfiguration("box_height_m"), value_type=float),
            "box_grasp_height_ratio": ParameterValue(
                LaunchConfiguration("box_grasp_height_ratio"), value_type=float),
            "supplybox_tcp_offset_z": ParameterValue(
                LaunchConfiguration("supplybox_tcp_offset_z"), value_type=float),
            "ik_request_timeout_sec": ParameterValue(
                LaunchConfiguration("ik_request_timeout_sec"), value_type=float),
            "ik_avoid_collisions": ParameterValue(
                LaunchConfiguration("ik_avoid_collisions"), value_type=bool),
            "settle_before_close_sec": ParameterValue(
                LaunchConfiguration("settle_before_close_sec"), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="false: validated real Dynamixel/RMD hardware; true: MoveIt mock hardware."),
        DeclareLaunchArgument(
            "hardware_yaml", default_value=PathJoinSubstitution([
                FindPackageShare("tool_manipulator_bringup"), "config", "hardware.yaml",
            ])),
        DeclareLaunchArgument(
            "tool_id", default_value="1",
            description="Tool 1 is the ee_joint-driven rack gripper used by this sequence."),
        DeclareLaunchArgument("use_mesh", default_value="true"),
        DeclareLaunchArgument("target_topic", default_value="/arm/target_point"),
        DeclareLaunchArgument("planning_frame", default_value="base_actuator"),
        DeclareLaunchArgument("arm_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("ee_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("use_fixed_target_z", default_value="true"),
        DeclareLaunchArgument("base_height_m", default_value="0.350"),
        DeclareLaunchArgument("box_height_m", default_value="0.095"),
        DeclareLaunchArgument("box_grasp_height_ratio", default_value="0.5"),
        DeclareLaunchArgument("supplybox_tcp_offset_z", default_value="-0.0055"),
        DeclareLaunchArgument("ik_request_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("ik_avoid_collisions", default_value="true"),
        DeclareLaunchArgument("settle_before_close_sec", default_value="2.0"),
        mock_bringup,
        real_bringup,
        grasp_node,
    ])
