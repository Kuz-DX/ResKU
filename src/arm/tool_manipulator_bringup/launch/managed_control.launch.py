"""Mode-managed arm stack. Defaults to mock hardware and HOLD."""
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
    base = common.build_arm_base(
        real_hardware=LaunchConfiguration('real_hardware').perform(context) == 'true',
        hardware_file=LaunchConfiguration('hardware_config').perform(context), mock_ee_velocity=True,
        manual_controller_file=str(source_config) if source_config.is_file() else '')
    scripts = Path(__file__).resolve().parents[1]/'scripts'
    if not scripts.is_dir():
        scripts = Path(get_package_prefix('tool_manipulator_bringup'))/'lib/tool_manipulator_bringup'
    manager = Node(executable=sys.executable, arguments=[str(scripts/'arm_control_mode_manager.py')],
                   parameters=[{'hardware_config': base.hardware_file}], output='screen')
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
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('real_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(share/'config/hardware.yaml')),
        DeclareLaunchArgument('launch_joy', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=bringup),
    ])
