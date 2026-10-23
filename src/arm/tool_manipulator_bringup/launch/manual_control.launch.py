"""Manual coordination through Servo; separate-joint mode uses position control."""
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _after(process, successors):
    def completed(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Manual bringup failed; manual input remains disabled.'),
                    EmitEvent(event=Shutdown(reason='manual bringup process failed'))]
        return successors
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=completed))


def _start(context):
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    real = LaunchConfiguration('real_hardware').perform(context).lower() == 'true'
    hardware = str(Path(LaunchConfiguration('hardware_config').perform(context)).expanduser().resolve(strict=True))
    if real:
        helper = Path(get_package_prefix('tool_manipulator_bringup'))/'lib/tool_manipulator_bringup/hardware_preflight.py'
        errors = runpy.run_path(str(helper))['validate'](yaml.safe_load(Path(hardware).read_text()))
        if errors:
            raise RuntimeError('Manual real control blocked: ' + '; '.join(errors))
    model = (MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
             .robot_description(mappings={'use_mock_hardware': 'false' if real else 'true',
                                         'hardware_config_file': hardware, 'use_mesh': 'true'})
             .to_moveit_configs())
    original = Path(get_package_share_directory('tool_manipulator_moveit_config'))/'config/ros2_controllers.yaml'
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[model.robot_description, str(original), str(share/'config/manual_controllers.yaml')],
                   output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[model.robot_description], output='screen')
    def spawner(name, inactive=False):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', '/controller_manager', '--controller-manager-timeout', '60',
                               *(['--inactive'] if inactive else [])], output='screen')
    jsb = spawner('joint_state_broadcaster')
    arm, ee = spawner('arm_controller'), spawner('ee_controller')
    recovery = Node(package='tool_manipulator_bringup', executable='move_to_named_pose.py',
                    arguments=['--recover-only', '--hardware', hardware,
                               '--servo-config', str(share/'config/servo.yaml'),
                               '--duration', '8.0', '--wait-for-server', '30.0',
                               '--joint-state-timeout', '10.0'], output='screen')
    manual_spawner = spawner('manual_position_controller', inactive=True)
    switch = ExecuteProcess(cmd=['ros2', 'control', 'switch_controllers', '-c', '/controller_manager',
                                  '--deactivate', 'arm_controller', '--activate',
                                  'manual_position_controller', '--strict'], output='screen')
    mode = LaunchConfiguration('control_mode').perform(context)
    coordinated = mode == 'coordinated'
    executable = 'manual_controller.sep.py' if mode == 'separate' else 'manual_controller.py'
    manual = Node(package='tool_manipulator_bringup', executable=executable,
                  parameters=[model.robot_description, {'hardware_config': hardware,
                                                       'servo_mode': coordinated}], output='screen')
    with open(share/'config/servo.yaml', encoding='utf-8') as stream:
        servo_config = {'moveit_servo': yaml.safe_load(stream)}
    if coordinated and not servo_config['moveit_servo'].get('check_collisions'):
        raise RuntimeError('Coordinated manual control requires Servo collision checking')
    servo = Node(package='moveit_servo', executable='servo_node_main', name='servo_node',
                 parameters=[servo_config, model.robot_description,
                             model.robot_description_semantic,
                             model.robot_description_kinematics], output='screen')
    start_servo = ExecuteProcess(cmd=['ros2', 'service', 'call', '/servo_node/start_servo',
                                      'std_srvs/srv/Trigger', '{}'], output='screen')
    joy = Node(package='joy', executable='joy_node',
               parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 50.0}], output='screen')
    ready = [manual, joy]
    if coordinated:
        ready.extend([servo, start_servo])
    if LaunchConfiguration('launch_rviz').perform(context).lower() == 'true':
        ready.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share/'config/joint_rviz.rviz')], output='screen'))
    next_step = ready if coordinated else [manual_spawner]
    handlers = [_after(jsb, [arm]), _after(arm, [ee]),
                _after(ee, [recovery] if real else next_step),
                _after(recovery, next_step)]
    if not coordinated:
        handlers.extend([_after(manual_spawner, [switch]), _after(switch, ready)])
    return [*handlers, state, control, jsb]


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('real_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('control_mode', default_value='coordinated', choices=['coordinated', 'separate']),
        DeclareLaunchArgument('hardware_config', default_value=str(share/'config/hardware.yaml')),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_start),
    ])
