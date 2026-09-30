"""Real arm + attachment-only tool docking launch with a single fail-closed preflight."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _load_yaml(path: Path) -> dict:
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def _validator(share: Path):
    script = share.parent.parent / 'lib' / 'tool_manipulator_bringup' / 'config_validator.py'
    if not script.is_file():
        raise RuntimeError(f'real tool change blocked: installed preflight missing: {script}')
    sys.path.insert(0, str(script.parent))
    spec = importlib.util.spec_from_file_location('tool_manipulator_preflight', script)
    if spec is None or spec.loader is None:
        raise RuntimeError('real tool change blocked: cannot load preflight')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _validate_all(context):
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    hardware = Path(LaunchConfiguration('hardware_config').perform(context))
    tools = Path(LaunchConfiguration('tools_config').perform(context))
    docking = Path(LaunchConfiguration('docking_config').perform(context))
    try:
        errors = _validator(share).validate_all(_load_yaml(hardware), _load_yaml(tools), _load_yaml(docking))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'real tool change blocked: cannot read configuration: {exc}') from exc
    if errors:
        raise RuntimeError('real tool change blocked; fill these settings before any hardware starts:\n- ' + '\n- '.join(errors))
    return []


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    hardware = LaunchConfiguration('hardware_config')
    tools = LaunchConfiguration('tools_config')
    docking = LaunchConfiguration('docking_config')
    real_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'real_control.launch.py')),
        launch_arguments={'hardware_config': hardware}.items(),
    )
    docking_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'apriltag_tool_docking.launch.py')),
        launch_arguments={'tools_config': tools, 'docking_params': docking}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config' / 'hardware.yaml')),
        DeclareLaunchArgument('tools_config', default_value=str(share / 'config' / 'tools.yaml')),
        DeclareLaunchArgument('docking_config', default_value=str(share / 'config' / 'docking.yaml')),
        OpaqueFunction(function=_validate_all),
        real_control,
        TimerAction(period=10.0, actions=[docking_stack]),
    ])
