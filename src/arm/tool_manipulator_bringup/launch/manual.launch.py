"""Main Servo-free manual control; collision gate temporarily disabled."""
import fcntl
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_common():
    path = Path(__file__).with_name('arm_bringup_common.py')
    spec = importlib.util.spec_from_file_location('tool_manipulator_arm_bringup_common', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = _load_common()

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
    real = LaunchConfiguration('real_hardware').perform(context) == 'true'
    base = common.build_arm_base(
        real_hardware=real,
        hardware_file=LaunchConfiguration('hardware_config').perform(context),
        mock_initial_positions_file=(
            LaunchConfiguration('mock_initial_positions_file').perform(context)),
        mock_ee_velocity=True,
        error_prefix='Servo-free real control blocked',
    )
    share = base.bringup_share
    hardware = base.hardware_file
    description = base.description
    control = base.control_node
    state = base.state_publisher
    jsb = base.spawner('joint_state_broadcaster')
    ee = base.spawner('ee_velocity_controller')
    manual_spawner = base.spawner('manual_position_controller', inactive=True)
    # Temporary manual-only bypass: keep manual_collision.py available, but do
    # not start its collision gate or the MoveIt state-validity scene.
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
            _shutdown_on_exit(control), _shutdown_on_exit(state),
            _shutdown_on_exit(manual),
            # Joystick is optional: its absence or driver restart must never
            # shut down the hardware/controller processes or startup recovery.
            joy_events,
            LogInfo(msg='Servo-free manual control: collision checking temporarily disabled.'),
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
        DeclareLaunchArgument('real_hardware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(common.default_hardware_config())),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_joy', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mock_initial_positions_file', default_value=''),
        OpaqueFunction(function=_preflight),
    ])
