"""Mock hardware, controllers, MoveIt and RViz; no task scripts."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    tool_id = LaunchConfiguration("tool_id")
    use_mesh = LaunchConfiguration("use_mesh")
    config = (
        MoveItConfigsBuilder("tool_manipulator", package_name="tool_manipulator_moveit_config")
        .robot_description(mappings={
            "use_mock_hardware": "true",
            "tool_id": tool_id,
            "use_mesh": use_mesh,
        })
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "tool_id", default_value="0",
            description=("Task/tool ID: 0=gripper, 1=drill, "
                         "2=supply_box handled by gripper, 99=no mounted tool.")),
        DeclareLaunchArgument(
            "use_mesh", default_value="false",
            description="Show visual meshes; collision meshes remain enabled."),
        *generate_demo_launch(config).entities,
    ])
