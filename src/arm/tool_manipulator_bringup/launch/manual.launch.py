"""Main Servo-free manual control; mock hardware is the default."""
import fcntl
import os
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_launch_lock = None


class JoystickConnectionLog:
    """Forward only the driver's actual device-open event to the console."""
    def __init__(self):
        self.pending = {}

    def consume(self, stream, event):
        lines = (self.pending.get(stream, b'') + event.text).split(b'\n')
        self.pending[stream] = lines.pop()[-8192:]
        return [LogInfo(msg='조이스틱 연결 완료') for line in lines if b'Opened joystick:' in line]

    def stdout(self, event):
        return self.consume('stdout', event)

    def stderr(self, event):
        return self.consume('stderr', event)


def _after(process, successors):
    def completed(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Servo-free bringup failed'))]
        return successors
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=completed))


def _shutdown_on_exit(process):
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
        EmitEvent(event=Shutdown(reason='Servo-free critical process exited'))]))


def _bringup(context):
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    real = LaunchConfiguration('real_hardware').perform(context) == 'true'
    hardware = str(Path(LaunchConfiguration('hardware_config').perform(context)).expanduser().resolve(strict=True))
    mock_initial = LaunchConfiguration('mock_initial_positions_file').perform(context)
    mappings = {'use_mock_hardware': 'false' if real else 'true',
                'hardware_config_file': hardware, 'use_mesh': 'true'}
    if mock_initial:
        if real:
            raise RuntimeError('mock_initial_positions_file is only supported with mock hardware')
        mappings['initial_positions_file'] = str(Path(mock_initial).expanduser().resolve(strict=True))
    if real:
        helper = Path(get_package_prefix('tool_manipulator_bringup'))/'lib/tool_manipulator_bringup/hardware_preflight.py'
        errors = runpy.run_path(str(helper))['validate'](yaml.safe_load(Path(hardware).read_text()))
        if errors:
            raise RuntimeError('Servo-free real control blocked: ' + '; '.join(errors))
    model = (MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
             .robot_description(mappings=mappings)
             .to_moveit_configs())
    description = dict(model.robot_description)
    if not real:
        # This launch alone gives the mock EE the real velocity-only interface.
        root = ET.fromstring(description['robot_description'])
        ee = root.find("ros2_control/joint[@name='ee_joint']/command_interface")
        if ee is None:
            raise RuntimeError('Mock EE command interface missing')
        ee.set('name', 'velocity')
        description['robot_description'] = ET.tostring(root, encoding='unicode')
    original = Path(get_package_share_directory('tool_manipulator_moveit_config'))/'config/ros2_controllers.yaml'
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[description, str(original), str(share/'config/manual_controllers.yaml')],
                   output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[description], output='screen')
    def spawner(name, inactive=False):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', '/controller_manager', '--controller-manager-timeout', '30',
                               *(['--inactive'] if inactive else [])], output='screen')
    jsb = spawner('joint_state_broadcaster')
    ee = spawner('ee_velocity_controller')
    manual_spawner = spawner('manual_position_controller', inactive=True)
    manual = Node(package='tool_manipulator_bringup', executable='manual.py',
                  parameters=[description, {'hardware_config': hardware, 'servo_mode': False}],
                  output='screen', sigterm_timeout='3', sigkill_timeout='3')
    joy = Node(package='joy', executable='joy_node',
               parameters=[{'device_id': 0, 'deadzone': .05, 'autorepeat_rate': 50.}],
               output='log', respawn=True, respawn_delay=2.)
    joy_log = JoystickConnectionLog()
    joy_events = RegisterEventHandler(OnProcessIO(
        target_action=joy, on_stdout=joy_log.stdout, on_stderr=joy_log.stderr))
    ready = [manual]
    if LaunchConfiguration('launch_joy').perform(context) == 'true':
        ready.append(joy)
    if LaunchConfiguration('launch_rviz').perform(context) == 'true':
        ready.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share/'config/joint_rviz.rviz')], output='screen'))
    # No JTC/Servo/recover-only process; the new node owns its startup ramp.
    return [_after(jsb, [ee]), _after(ee, [manual_spawner]), _after(manual_spawner, ready),
            _shutdown_on_exit(control), _shutdown_on_exit(state), _shutdown_on_exit(manual),
            # Joystick is optional: its absence or driver restart must never
            # shut down the hardware/controller processes or startup recovery.
            joy_events,
            LogInfo(msg='Servo-free manual control: startup RMD recovery to 2-degree inset; no collision checking.'),
            state, control, jsb]


def _preflight(context):
    global _launch_lock
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    _launch_lock = open(f'/tmp/resku_arm_manual_{domain}.lock', 'a')
    try:
        fcntl.flock(_launch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('Another Resku_arm launch is running') from exc
    check = Node(package='tool_manipulator_bringup', executable='manual.py',
                 arguments=['--check-exclusive'], output='screen')
    return [_after(check, [OpaqueFunction(function=_bringup)]), check]


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('real_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(share/'config/hardware.yaml')),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_joy', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mock_initial_positions_file', default_value=''),
        OpaqueFunction(function=_preflight),
    ])
