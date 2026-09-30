"""Manipulator URDF visualization: robot_state_publisher + JSP GUI + RViz.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mesh = LaunchConfiguration("use_mesh")
    tool_id = LaunchConfiguration("tool_id")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("tool_manipulator_description"),
            "urdf", "tool_manipulator.urdf.xacro",
        ]),
        " use_mesh:=", use_mesh,
        " tool_id:=", tool_id,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mesh", default_value="true",
            description="true: CAD STL visual meshes; false: kinematic frames only.",
        ),
        DeclareLaunchArgument(
            "tool_id", default_value="0",
            description="Recognized tool tag ID: 0=no tool, 1=gripper, 2/3=other tools.",
        ),
        DeclareLaunchArgument(
            "publish_joint_states", default_value="true",
            description="Start joint_state_publisher_gui. Set false when a hardware bridge publishes /joint_states.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(publish_joint_states),
            parameters=[{
                "zeros": {
                    "base_joint": 0.0,
                    "shoulder_joint": 0.0,
                    "elbow_joint": 0.0,
                    "wrist_pitch_joint": 0.0,
                    "wrist_roll_joint": 0.0,
                    "wrist_yaw_joint": 0.0,
                    "ee_joint": 0.0,
                }
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("tool_manipulator_description"), "rviz", "display.rviz"
            ])],
        ),
    ])
