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
    # Manual jogging retains its existing behavior. Requested automatic angular
    # alignment validates its complete path with the existing MoveIt scene service.
    manual = Node(package='tool_manipulator_bringup', executable='standalone_managed_manual.py',
                  parameters=[description, {'hardware_config': hardware, 'servo_mode': False,
                      'startup_recovery_enabled': False,  # Single recovery owner: manager.
                      'invert_forward': False,  # Reverse standalone manual right-stick RMD motion.
                      'r2_pitch_translation': True,
                      'docking_lock_buttons': True,
                      'terminal_status_log': True,
                      'docking_assist_config': LaunchConfiguration('docking_assist_config').perform(context),
                      'docking_camera_frame': LaunchConfiguration('docking_camera_frame').perform(context),
                      'docking_camera_info_topic': LaunchConfiguration('docking_camera_info_topic').perform(context),
                      'docking_centers_topic': LaunchConfiguration('docking_centers_topic').perform(context)}],
                  output='screen', sigterm_timeout='3', sigkill_timeout='3')
    joy = Node(package='joy', executable='joy_node',
               parameters=[{'device_id': 0, 'deadzone': .05, 'autorepeat_rate': 50.}],
               output='screen', respawn=True, respawn_delay=2.)
    joy_log = JoystickConnectionLog()
    joy_events = RegisterEventHandler(OnProcessIO(
        target_action=joy, on_stdout=joy_log.stdout, on_stderr=joy_log.stderr))
    scene = Node(package='moveit_ros_move_group', executable='move_group',
                 parameters=[base.moveit_config.to_dict(), common.alignment_collision_description(description),
                             {'allow_trajectory_execution': False,
                              'publish_robot_description_semantic': True}], output='screen')
    manager = Node(package='tool_manipulator_bringup', executable='arm_control_mode_manager.py',
                   parameters=[{'hardware_config': hardware, 'focus_arm_mode': 'manual'}],
                   output='screen')
    # Keep the manual alignment scene's finger exception local to alignment.
    # Automatic routes use the full robot geometry in their own planning scene.
    auto_scene = Node(package='moveit_ros_move_group', executable='move_group',
                      namespace='manual_auto',
                      parameters=[base.moveit_config.to_dict(), description,
                                  {'allow_trajectory_execution': False,
                                   'publish_robot_description_semantic': True}],
                      remappings=[('joint_states', '/joint_states')], output='screen')
    tool_sequence = Node(package='tool_manipulator_bringup', executable='manual_tool_sequence.py',
                         parameters=[description, base.moveit_config.robot_description_semantic,
                                     {'hardware_config': hardware,
                                      'trajectory_dir': str(share/'config/resampled'),
                                      'cartesian_targets_file': str(share/'config/cartesian_targets.yaml'),
                                      'shoulder_isolation_enabled': True,
                                      'shoulder_isolation_half_width_rad': .08,
                                      'shoulder_isolation_speed_rad_s': .015,
                                      'docking_config': LaunchConfiguration('docking_config').perform(context),
                                      'docking_motion_enabled': True,
                                      'docking_camera_frame': LaunchConfiguration('docking_camera_frame').perform(context),
                                      'docking_camera_info_topic': LaunchConfiguration('docking_camera_info_topic').perform(context),
                                      'docking_centers_topic': LaunchConfiguration('docking_centers_topic').perform(context)}],
                         remappings=[('/check_state_validity', '/manual_auto/check_state_validity'),
                                     ('/plan_kinematic_path', '/manual_auto/plan_kinematic_path')],
                         output='screen')
    ready = [scene, auto_scene, manager, manual, tool_sequence]
    if real and LaunchConfiguration('launch_docking_bridge').perform(context) == 'true':
        ready.append(Node(package='tool_manipulator_bringup', executable='arduino_docking_bridge.py',
                          parameters=[{'port': LaunchConfiguration('docking_serial_port').perform(context),
                                       'baud': int(LaunchConfiguration('docking_serial_baud').perform(context))}],
                          output='screen'))
    if LaunchConfiguration('launch_joy').perform(context) == 'true':
        ready.append(joy)
    if LaunchConfiguration('launch_rviz').perform(context) == 'true':
        ready.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share/'config/joint_rviz.rviz')], output='screen'))
    # The manager owns startup recovery and all controller command output.
    return [_after(jsb, [ee]), _after(ee, [manual_spawner]), _after(manual_spawner, ready),
            _shutdown_on_exit(control), _shutdown_on_exit(state),
            _shutdown_on_exit(manual),
            _shutdown_on_exit(manager), _shutdown_on_exit(tool_sequence),
            _shutdown_on_exit(scene), _shutdown_on_exit(auto_scene),
            # Joystick is optional: its absence or driver restart must never
            # shut down the hardware/controller processes or startup recovery.
            joy_events,
            LogInfo(msg='Manual jogging: collision checking disabled; requested angular alignment: path checked, gripper fingers excluded.'),
            LogInfo(msg='/selected_tool_id: HOME -> BACK -> tool docking; successful completion restores manual control.'),
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
        DeclareLaunchArgument('docking_assist_config', default_value=str(share/'config/manual_docking_assist.yaml')),
        DeclareLaunchArgument('docking_config', default_value=str(share/'config/tool_docking.yaml')),
        DeclareLaunchArgument('launch_docking_bridge', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('docking_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('docking_serial_baud', default_value='115200'),
        DeclareLaunchArgument('docking_camera_frame', default_value='arm_camera_color_optical_frame'),
        DeclareLaunchArgument('docking_camera_info_topic', default_value='/arm/camera/color/camera_info'),
        DeclareLaunchArgument('docking_centers_topic', default_value='/arm/apriltag/centers'),
        OpaqueFunction(function=_preflight),
    ])
