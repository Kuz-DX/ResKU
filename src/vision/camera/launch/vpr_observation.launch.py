"""Sensor/VPR-only setup: never starts motors, odometry, arm controllers or motion.

Existing measured TF/origin/mission publishers must come from the robot system.
No camera housing measurement is substituted for an optical extrinsic.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('vision'))
    arg = LaunchConfiguration
    defaults = dict(
        device='/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0',
        start_rear='true', start_arm='false', start_visual_nodes='true',
        camera_info_url='package://vision/config/camera/calibration/apc850_rear_preview_trial_1280x720.yaml',
        arm_config=str(share/'config/camera/vpr_front_d455.yaml'),
        arm_serial_no='',
        expected_lock_pose_file='')
    declarations = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    def validate_front(context):
        if arg('start_arm').perform(context).lower() == 'true' and not arg('arm_serial_no').perform(context).strip():
            raise ValueError('Actual front D455 arm_serial_no required when start_arm=true')
        return []

    nodes = [OpaqueFunction(function=validate_front),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/apc850_rear.launch.py')),
            condition=IfCondition(arg('start_rear')),
            launch_arguments=dict(device=arg('device'), camera_info_url=arg('camera_info_url'),
                                  width='1280', height='720', fps='15.0',
                                  publish_extrinsic='false').items()),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             namespace='arm', name='camera', condition=IfCondition(arg('start_arm')),
             parameters=[arg('arm_config'), {'serial_no': ParameterValue(arg('arm_serial_no'), value_type=str)}], output='screen'),
        Node(package='visual_landmark', executable='visual_landmark_node',
             condition=IfCondition(arg('start_visual_nodes')),
             parameters=[dict(enable_auto_keyframes=True, capture_mode='stop')], output='screen'),
        Node(package='visual_landmark', executable='visual_localization_node',
             condition=IfCondition(arg('start_visual_nodes')),
             parameters=[dict(enable_adaptive_visual_check=True, apply_correction=False,
                              lock_pose_guard=True, expected_lock_pose_file=arg('expected_lock_pose_file'))],
             output='screen'),
        Node(package='visual_landmark', executable='visual_check_scheduler_node',
             condition=IfCondition(arg('start_visual_nodes')),
             parameters=[dict(enable_adaptive_visual_check=True, enable_slip_early_trigger=False)],
             output='screen'),
    ]
    return LaunchDescription(declarations+nodes)
