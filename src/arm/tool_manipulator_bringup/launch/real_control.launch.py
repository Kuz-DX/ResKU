"""Real-arm controller + MoveIt launch. Refuses incomplete hardware.yaml."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_move_group_launch


def _load_common():
    path = Path(__file__).with_name('arm_bringup_common.py')
    spec = importlib.util.spec_from_file_location('tool_manipulator_arm_bringup_common', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = _load_common()


def _home_after_controller(event, context, home_node):
    if event.returncode != 0:
        return [LogInfo(msg='ERROR: Startup recovery skipped: arm_controller failed to activate.')]
    return [home_node]


def _start_real(context):
    base = common.build_arm_base(
        real_hardware=True,
        hardware_file=LaunchConfiguration('hardware_config').perform(context),
        tool_id=LaunchConfiguration('tool_id').perform(context),
        use_mesh=LaunchConfiguration('use_mesh').perform(context),
        error_prefix='real control blocked',
    )
    hardware_file = base.hardware_file
    moveit_config = base.moveit_config
    control_node = base.control_node
    state_publisher = base.state_publisher
    broadcaster = base.spawner('joint_state_broadcaster')
    arm_controller = base.spawner('arm_controller')
    ee_controller = base.spawner('ee_velocity_controller')
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
    default_config = str(common.default_hardware_config())
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=default_config),
        DeclareLaunchArgument(
            'tool_id', default_value='0',
            description='Task/tool ID: 0=gripper, 1=drill, 2=supply_box, 99=no mounted tool.'),
        DeclareLaunchArgument('use_mesh', default_value='true'),
        DeclareLaunchArgument('move_home_on_start', default_value='true',
                              description='Move arm to SRDF home once after controller activation.'),
        DeclareLaunchArgument('home_duration', default_value='8.0',
                              description='Startup home trajectory duration in seconds.'),
        DeclareLaunchArgument('servo_config', default_value=str(share / 'config' / 'servo.yaml')),
        OpaqueFunction(function=_start_real),
    ])
