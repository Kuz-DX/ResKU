from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("tool_manipulator", package_name="tool_manipulator_design_moveit_config")
        .robot_description(mappings={"use_mock_hardware": "true", "use_mesh": "false"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(config)
