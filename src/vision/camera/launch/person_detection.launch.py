"""Run one shared OpenVINO person detector for drive/left/right cameras."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_model = str(
        Path(get_package_share_directory("vision"))
        / "models"
        / "mando-dummy-v1.xml"
    )

    arguments = [
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="RF-DETR OpenVINO IR XML model path",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/drive/camera/color/image_raw/compressed",
            description="CompressedImage camera input",
        ),
        DeclareLaunchArgument(
            "detections_topic",
            default_value="/person_detection/detections",
            description="JSON detection result topic",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/drive/person/detecion",
            description="Drive CompressedImage output with person bounding boxes",
        ),
        DeclareLaunchArgument(
            "left_input_topic",
            default_value="/side/left/image_raw/compressed",
            description="Left camera CompressedImage input",
        ),
        DeclareLaunchArgument(
            "left_detections_topic",
            default_value="/left/person/detections",
            description="Left camera JSON detection result topic",
        ),
        DeclareLaunchArgument(
            "left_output_topic",
            default_value="/left/person/detection",
            description="Left camera CompressedImage output with person bounding boxes",
        ),
        DeclareLaunchArgument(
            "right_input_topic",
            default_value="/side/right/image_raw/compressed",
            description="Right camera CompressedImage input",
        ),
        DeclareLaunchArgument(
            "right_detections_topic",
            default_value="/right/person/detections",
            description="Right camera JSON detection result topic",
        ),
        DeclareLaunchArgument(
            "right_output_topic",
            default_value="/right/person/detection",
            description="Right camera CompressedImage output with person bounding boxes",
        ),
        DeclareLaunchArgument("enable_side_cameras", default_value="true"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5"),
        DeclareLaunchArgument("device", default_value="CPU"),
        DeclareLaunchArgument("jpeg_quality", default_value="90"),
        DeclareLaunchArgument("publish_visualization", default_value="true"),
        DeclareLaunchArgument("cache_dir", default_value=""),
    ]

    person_detection = Node(
        package="vision",
        executable="person_detection",
        name="person_detection_openvino",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": ParameterValue(
                    LaunchConfiguration("model_path"), value_type=str
                ),
                "input_topic": ParameterValue(
                    LaunchConfiguration("input_topic"), value_type=str
                ),
                "detections_topic": ParameterValue(
                    LaunchConfiguration("detections_topic"), value_type=str
                ),
                "output_topic": ParameterValue(
                    LaunchConfiguration("output_topic"), value_type=str
                ),
                "left_input_topic": ParameterValue(
                    LaunchConfiguration("left_input_topic"), value_type=str
                ),
                "left_detections_topic": ParameterValue(
                    LaunchConfiguration("left_detections_topic"), value_type=str
                ),
                "left_output_topic": ParameterValue(
                    LaunchConfiguration("left_output_topic"), value_type=str
                ),
                "right_input_topic": ParameterValue(
                    LaunchConfiguration("right_input_topic"), value_type=str
                ),
                "right_detections_topic": ParameterValue(
                    LaunchConfiguration("right_detections_topic"), value_type=str
                ),
                "right_output_topic": ParameterValue(
                    LaunchConfiguration("right_output_topic"), value_type=str
                ),
                "enable_side_cameras": ParameterValue(
                    LaunchConfiguration("enable_side_cameras"), value_type=bool
                ),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"), value_type=float
                ),
                "device": ParameterValue(
                    LaunchConfiguration("device"), value_type=str
                ),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
                "publish_visualization": ParameterValue(
                    LaunchConfiguration("publish_visualization"), value_type=bool
                ),
                "cache_dir": ParameterValue(
                    LaunchConfiguration("cache_dir"), value_type=str
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [person_detection])
