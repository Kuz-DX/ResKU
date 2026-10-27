"""Existing arm control with model TF isolated from vehicle VPR TF.

This DOES start arm hardware by default. It replaces (never accompanies) the
normal ik_manual launch. Camera driver and LOCK sensor TF remain separate.
No arm description, controller, or joint geometry is changed.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    arm_share = Path(get_package_share_directory('tool_manipulator_bringup'))
    defaults = dict(real_hardware='true', launch_joy='false', launch_tcp_joy='true',
                    hardware_config=str(arm_share/'config/hardware.yaml'))
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        # Keep remaps for arm nodes started later by OnProcessExit handlers too.
        # This launch process contains only the arm stack.
        GroupAction(scoped=False, actions=[
            SetRemap(src='/tf', dst='/arm_model/tf'),
            SetRemap(src='/tf_static', dst='/arm_model/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(arm_share/'launch/ik_manual.launch.py')),
                launch_arguments={key: LaunchConfiguration(key) for key in defaults}.items()),
        ]),
    ])
