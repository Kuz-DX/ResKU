from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    config = (MoveItConfigsBuilder("tool_manipulator", package_name="tool_manipulator_moveit_config")
              .robot_description(mappings={"tool_id": LaunchConfiguration("tool_id"),
                                           "use_mesh": LaunchConfiguration("use_mesh")})
              .to_moveit_configs())
    return LaunchDescription([
        DeclareLaunchArgument("tool_id", default_value="0"),
        DeclareLaunchArgument("use_mesh", default_value="true"),
        *generate_move_group_launch(config).entities,
    ])
