"""Real-arm controller + MoveIt launch. Refuses incomplete hardware.yaml."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


ARM_DXL = ('base_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint')
ARM_RMD = ('shoulder_joint', 'elbow_joint', 'wrist_pitch_joint')


def _missing(value):
    return value is None or (isinstance(value, list) and any(_missing(item) for item in value))


def _validate(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    errors = []
    u2d2 = config.get("buses", {}).get("dynamixel_u2d2", {})
    if u2d2.get("device") != "/dev/ttyUSB0":
        errors.append("buses.dynamixel_u2d2.device")
    if u2d2.get("baud_rate") != 1000000 or u2d2.get("protocol") != 2.0:
        errors.append("buses.dynamixel_u2d2.baud_rate/protocol")
    if (u2d2.get("mixed_ttl_rs485") is not True or
            u2d2.get("feedback_mode") != "individual_read" or
            u2d2.get("command_mode") != "sync_write"):
        errors.append("buses.dynamixel_u2d2.mixed_transport_mode")
    joints = config.get("joints", {})
    for name in ARM_DXL:
        spec = joints.get(name, {})
        if spec.get("bus") != "dynamixel_u2d2" or spec.get("transport_channel") not in ("ttl", "rs485"):
            errors.append(f"{name}.bus/transport_channel")
        for key in ("zero_raw", "soft_limit_raw", "soft_limit_rad", "velocity_limit_rad_s",
                    "current_limit_ma", "current_limit_raw", "velocity_limit_raw",
                    "profile_acceleration_raw", "profile_velocity_raw"):
            if _missing(spec.get(key)):
                errors.append(f"{name}.{key}")
    for name in ARM_RMD:
        spec = joints.get(name, {})
        for key in ("sign", "q_offset_rad", "soft_limit_rad", "velocity_limit_rad_s",
                    "current_limit_a", "current_release_threshold_a",
                    "current_retreat_velocity_rad_s", "current_max_retreat_distance_rad",
                    "current_limit_duration_ms"):
            if _missing(spec.get(key)):
                errors.append(f"{name}.{key}")
    if errors:
        raise RuntimeError("real control blocked: fill hardware.yaml fields: " + ", ".join(errors))


def _start_real(context):
    hardware_file = LaunchConfiguration('hardware_config').perform(context)
    _validate(hardware_file)
    controllers = str(Path(get_package_share_directory('tool_manipulator_moveit_config')) / 'config' / 'ros2_controllers.yaml')
    moveit_config = (
        MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
        .robot_description(mappings={
            'use_mock_hardware': 'false',
            'hardware_config_file': hardware_file,
            'tool_id': '0',
            'use_mesh': 'true',
        })
        .to_moveit_configs()
    )
    control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[moveit_config.robot_description, controllers],
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[moveit_config.robot_description],
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen',
    )
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'], output='screen',
    )
    ee_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['ee_controller', '--controller-manager', '/controller_manager'], output='screen',
    )
    actions = [
        state_publisher,
        control_node,
        TimerAction(period=3.0, actions=[broadcaster]),
        TimerAction(period=5.0, actions=[arm_controller]),
        TimerAction(period=6.0, actions=[ee_controller]),
        TimerAction(period=7.0, actions=generate_move_group_launch(moveit_config).entities),
    ]

    launch_servo = LaunchConfiguration('launch_servo').perform(context).lower() in ('1', 'true', 'yes')
    launch_teleop = (
        LaunchConfiguration('launch_tcp_joy_teleop').perform(context).lower()
        in ('1', 'true', 'yes')
    )
    launch_joy = LaunchConfiguration('launch_joy').perform(context).lower() in ('1', 'true', 'yes')
    if launch_teleop and not launch_servo:
        raise RuntimeError('real control blocked: TCP joystick teleop requires launch_servo:=true')

    delayed_actions = []
    if launch_servo:
        servo_file = LaunchConfiguration('servo_config').perform(context)
        with open(servo_file, encoding='utf-8') as stream:
            servo_config = {'moveit_servo': yaml.safe_load(stream) or {}}
        delayed_actions.append(Node(
            package='moveit_servo', executable='servo_node_main', name='servo_node',
            parameters=[servo_config, moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics],
            output='screen',
        ))
    if launch_joy:
        delayed_actions.append(Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.05,
                'autorepeat_rate': 50.0,
            }],
            output='screen',
        ))
    if launch_teleop:
        delayed_actions.append(Node(
            package='tool_manipulator_bringup', executable='tcp_joy_teleop.py',
            output='screen',
        ))
    if delayed_actions:
        actions.append(TimerAction(period=8.0, actions=delayed_actions))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    default_config = str(share / 'config' / 'hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=default_config),
        DeclareLaunchArgument('servo_config', default_value=str(share / 'config' / 'servo.yaml')),
        DeclareLaunchArgument('launch_servo', default_value='false'),
        DeclareLaunchArgument('launch_tcp_joy_teleop', default_value='false'),
        DeclareLaunchArgument('launch_joy', default_value='false'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        OpaqueFunction(function=_start_real),
    ])
