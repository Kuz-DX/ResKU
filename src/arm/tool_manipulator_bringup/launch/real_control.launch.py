"""Real-arm controller + MoveIt launch. Refuses incomplete hardware.yaml."""
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _home_after_controller(event, context, home_node):
    if event.returncode != 0:
        return [LogInfo(msg='ERROR: Startup recovery skipped: arm_controller failed to activate.')]
    return [home_node]


def _validate(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    helper = (Path(get_package_prefix('tool_manipulator_bringup')) /
              'lib/tool_manipulator_bringup/hardware_preflight.py')
    errors = runpy.run_path(str(helper))['validate'](config)
    if errors:
        raise RuntimeError("real control blocked: " + "; ".join(errors))


def _start_real(context):
    hardware_file = LaunchConfiguration('hardware_config').perform(context)
    # Resolve against the launch caller's cwd before xacro resolves YAML paths
    # relative to the installed robot-description file.
    hardware_file = str(Path(hardware_file).expanduser().resolve(strict=True))
    _validate(hardware_file)
    controllers = str(Path(get_package_share_directory('tool_manipulator_moveit_config')) / 'config' / 'ros2_controllers.yaml')
    manual_controllers = str(
        Path(get_package_share_directory('tool_manipulator_bringup')) /
        'config/manual_controllers.yaml')
    moveit_config = (
        MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
        .robot_description(mappings={
            'use_mock_hardware': 'false',
            'hardware_config_file': hardware_file,
            'tool_id': LaunchConfiguration('tool_id').perform(context),
            'use_mesh': LaunchConfiguration('use_mesh').perform(context),
        })
        .to_moveit_configs()
    )
    control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[moveit_config.robot_description, controllers, manual_controllers],
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
        arguments=['ee_velocity_controller', '--controller-manager', '/controller_manager'], output='screen',
    )
    actions = [
        state_publisher,
        control_node,
        TimerAction(period=3.0, actions=[broadcaster]),
        TimerAction(period=5.0, actions=[arm_controller]),
        TimerAction(period=6.0, actions=[ee_controller]),
        TimerAction(period=7.0, actions=generate_move_group_launch(moveit_config).entities),
    ]
    move_home = LaunchConfiguration('move_home_on_start').perform(context).lower() in ('1', 'true', 'yes')
    startup_node = Node(
        package='tool_manipulator_bringup', executable='move_to_named_pose.py',
        arguments=[
            *(['home'] if move_home else ['--recover-only']),
            '--hardware', hardware_file,
            '--servo-config', LaunchConfiguration('servo_config'),
            '--duration', LaunchConfiguration('home_duration'),
            '--wait-for-server', '30.0', '--joint-state-timeout', '10.0',
        ], output='screen',
    )
    actions.insert(0, RegisterEventHandler(OnProcessExit(
        target_action=arm_controller,
        on_exit=lambda event, context: _home_after_controller(event, context, startup_node),
    )))

    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    default_config = str(share / 'config' / 'hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=default_config),
        DeclareLaunchArgument(
            'tool_id', default_value='0',
            description='Static tool model: 0=gripper, 1=drill, 2=Phillips bit, 99=no tool.'),
        DeclareLaunchArgument('use_mesh', default_value='true'),
        DeclareLaunchArgument('move_home_on_start', default_value='true',
                              description='Move arm to SRDF home once after controller activation.'),
        DeclareLaunchArgument('home_duration', default_value='8.0',
                              description='Startup home trajectory duration in seconds.'),
        DeclareLaunchArgument('servo_config', default_value=str(share / 'config' / 'servo.yaml')),
        OpaqueFunction(function=_start_real),
    ])
