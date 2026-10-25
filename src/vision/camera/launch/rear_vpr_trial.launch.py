"""Rear-only VPR trial input. Approximate extrinsic, no motion/control nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Optical axes: right/down/forward. Rear-facing, 3 deg down, assumed zero
    # housing roll. Translation approximates the lens surface as optical origin.
    defaults = dict(
        device='/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0',
        parent_frame='base_link',
        camera_info_url='package://vision/config/camera/calibration/apc850_rear_preview_trial_1280x720.yaml',
        x='-0.343627', y='0.0', z='0.650674',
        roll='-1.6231562043547265', pitch='0.0', yaw='1.5707963267948966')
    args = {key: LaunchConfiguration(key) for key in defaults}
    args.update(width='1280', height='720', fps='15.0', publish_extrinsic='true')
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        LogInfo(msg='TRIAL rear optical TF: approximate 27mm rearward offset, 3deg down, zero housing roll. Keep visual_apply_correction=false. Do not run another rear driver/TF publisher.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('vision'))/'launch/apc850_rear.launch.py')),
            launch_arguments=args.items()),
    ])
