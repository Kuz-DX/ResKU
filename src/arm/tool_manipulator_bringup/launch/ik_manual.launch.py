"""Mode-managed IK/manual arm stack. Defaults to real hardware and external Joy."""
import importlib.util
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def bringup(context):
    spec = importlib.util.spec_from_file_location('managed_common', Path(__file__).with_name('arm_bringup_common.py'))
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)
    source_config = Path(__file__).resolve().parents[1]/'config/manual_controllers.yaml'
    ready_config = source_config.parent/'tcp_ready.yaml'
    trajectory_dir = source_config.parent/'resampled'
    if not trajectory_dir.is_dir():
        trajectory_dir = Path(get_package_share_directory('tool_manipulator_bringup'))/'config/resampled'
    real = LaunchConfiguration('real_hardware').perform(context) == 'true'
    tcp_enabled = LaunchConfiguration('launch_tcp_joy').perform(context) == 'true'
    initial = LaunchConfiguration('mock_initial_positions_file').perform(context)
    if tcp_enabled and not real and not initial:
        config_dir = source_config.parent if source_config.is_file() else Path(
            get_package_share_directory('tool_manipulator_bringup'))/'config'
        initial = str(config_dir/'tcp_mock_initial_positions.yaml')
    base = common.build_arm_base(
        real_hardware=real,
        hardware_file=LaunchConfiguration('hardware_config').perform(context), mock_ee_velocity=True,
        # Use the same mounted-tool geometry for display and collision checks.
        tool_id=LaunchConfiguration('tool_id').perform(context),
        mock_initial_positions_file=initial,
        manual_controller_file=str(source_config) if source_config.is_file() else '')
    scripts = Path(__file__).resolve().parents[1]/'scripts'
    if not scripts.is_dir():
        scripts = Path(get_package_prefix('tool_manipulator_bringup'))/'lib/tool_manipulator_bringup'
    manager = Node(executable=sys.executable, arguments=[str(scripts/'arm_control_mode_manager.py')],
                   parameters=[{'hardware_config': base.hardware_file,
                                # TcpJoy owns entry after alignment and a fresh collision proof.
                                'focus_arm_mode': ''}],
                   output='screen')
    manual = Node(executable=sys.executable, arguments=[str(scripts/'managed_manual.py')],
                  parameters=[base.description, {'hardware_config': base.hardware_file,
                                                 'startup_recovery_enabled': False}], output='screen')
    jsb = base.spawner('joint_state_broadcaster')
    ee = base.spawner('ee_velocity_controller')
    pos = base.spawner('manual_position_controller', inactive=True)
    def after(process, actions):
        return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=lambda event, ctx:
            actions if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='managed startup failed'))]))
    ready = [manager, manual]
    if real and LaunchConfiguration('launch_docking_bridge').perform(context) == 'true':
        ready.append(Node(executable=sys.executable,
                          arguments=[str(scripts/'arduino_docking_bridge.py')],
                          parameters=[{
                              'port': LaunchConfiguration('docking_serial_port').perform(context),
                              'baud': int(LaunchConfiguration('docking_serial_baud').perform(context)),
                          }], output='screen'))
    tcp_nodes = []
    if LaunchConfiguration('launch_tcp_joy').perform(context) == 'true':
        tcp = Node(executable=sys.executable, arguments=[str(scripts/'tcp_joy.py')],
                   parameters=[base.description, {'hardware_config': base.hardware_file,
                                                  'tcp_ready_yaml': str(ready_config)}], output='screen')
        docking = LaunchConfiguration('enable_tool_docking').perform(context) == 'true'
        pick_place = LaunchConfiguration('enable_tool2_pick_place').perform(context) == 'true'
        if pick_place and not docking:
            raise ValueError('tool2 pick/place requires the managed docking coordinator')
        docking_parameters = ({
            'docking_config': LaunchConfiguration('docking_config').perform(context),
            'docking_camera_frame': LaunchConfiguration('docking_camera_frame').perform(context),
            'docking_motion_enabled': (
                LaunchConfiguration('enable_docking_motion').perform(context) == 'true'),
        } if docking else {})
        if pick_place:
            docking_parameters.update(
                pick_place_config=LaunchConfiguration('tool2_pick_place_config').perform(context),
                pick_place_ready_yaml=str(ready_config))
        tool_script = ('tool2_pick_place_coordinator.py' if pick_place else
                       'tool_docking_coordinator.py' if docking else 'tool_trajectory_joy.py')
        tool_paths = Node(executable=sys.executable,
                          arguments=[str(scripts/tool_script)],
                          parameters=[base.description,
                                      base.moveit_config.robot_description_semantic,
                                      {'hardware_config': base.hardware_file,
                                                        'trajectory_dir': str(trajectory_dir),
                                                        'shoulder_isolation_enabled': True,
                                                        'shoulder_isolation_half_width_rad': 0.08,
                                                        'shoulder_isolation_speed_rad_s': 0.015,
                                                        **docking_parameters}],
                          output='screen')
        # Planning scene service only: no execution ownership or controller actions.
        scene = Node(package='moveit_ros_move_group', executable='move_group',
                     parameters=[base.moveit_config.to_dict(), base.description,
                                 {'allow_trajectory_execution': False,
                                  'publish_robot_description_semantic': True}], output='screen')
        tcp_nodes = [tcp, tool_paths, scene]
        if docking:
            # Losing the docking watchdog must stop the managed stack as well.
            ready.append(RegisterEventHandler(OnProcessExit(target_action=tool_paths, on_exit=[
                EmitEvent(event=Shutdown(reason='docking coordinator exited'))])))
        ready.extend(tcp_nodes)
    if LaunchConfiguration('launch_joy').perform(context) == 'true':
        ready.append(Node(package='joy', executable='joy_node', parameters=[{'autorepeat_rate': 50.}]))
    preflight = Node(executable=sys.executable, arguments=[str(scripts/'manual.py'), '--check-exclusive'],
                     output='screen')
    return [after(preflight, [base.state_publisher, base.control_node, jsb]),
            after(jsb, [ee]), after(ee, [pos]), after(pos, ready),
            *[RegisterEventHandler(OnProcessExit(target_action=p, on_exit=[
                EmitEvent(event=Shutdown(reason='managed control process exited'))]))
              for p in (manager, manual, base.control_node, base.state_publisher)],
            preflight]


def generate_launch_description():
    source_config = Path(__file__).resolve().parents[1]/'config/hardware.yaml'
    hardware_default = (source_config if source_config.is_file() else
                        Path(get_package_share_directory('tool_manipulator_bringup'))/'config/hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('real_hardware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(hardware_default)),
        DeclareLaunchArgument('launch_joy', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mock_initial_positions_file', default_value=''),
        DeclareLaunchArgument('tool_id', default_value='99', choices=['0', '1', '2', '99'],
                              description='Mounted tool: 0=gripper, 1/2=other tools, 99=empty wrist'),
        DeclareLaunchArgument('launch_tcp_joy', default_value='true', choices=['true', 'false']),
        # One L2 + circle request owns the taught approach and its docking tail.
        # Enabling the producer does not submit a path at launch time.
        DeclareLaunchArgument('enable_tool_docking', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_tool2_pick_place', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('tool2_pick_place_config', default_value=str(hardware_default.parent/'tool2_pick_place.yaml')),
        DeclareLaunchArgument('enable_docking_motion', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('docking_config', default_value=str(hardware_default.parent/'tool_docking.yaml')),
        DeclareLaunchArgument('docking_camera_frame', default_value='arm_camera_color_optical_frame'),
        DeclareLaunchArgument('launch_docking_bridge', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('docking_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('docking_serial_baud', default_value='115200'),
        OpaqueFunction(function=bringup),
    ])
