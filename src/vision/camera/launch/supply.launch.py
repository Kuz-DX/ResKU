"""Run supply-box perception using the arm camera and aligned depth.

Camera drivers and the arm TF/controller must be started separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = {
        'model_path': ('', str),
        'target_class': ('supplybox', str),
        'conf_threshold': ('0.5', float),
        'infer_size': ('320', int),
        'depth_roi_radius': ('5', int),
        'max_depth_m': ('1.0', float),
        'planning_frame': ('base_actuator', str),
        'target_depth_topic': ('/arm/target_depth_m', str),
        'stop_base_x_m': ('0.32', float),
        'target_confirm_frames': ('3', int),
        'color_topic': ('/arm/camera/color/image_raw/compressed', str),
        'depth_topic': (
            '/arm/camera/aligned_depth_to_color/image_raw/compressedDepth', str),
        'camera_info_topic': ('/arm/camera/color/camera_info', str),
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, (default, _) in parameters.items()
    ]
    return LaunchDescription(arguments + [
        Node(
            package='vision',
            executable='supply',
            name='supply_node',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (_, value_type) in parameters.items()
            }],
        ),
    ])
