"""Run one shared YOLO person detector for drive/left/right/arm cameras."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    installed_model = (
        Path(get_package_share_directory("vision")) / "models" / "person.pt"
    )
    source_model = Path(__file__).resolve().parents[2] / "models" / "person.pt"
    default_model = str(
        installed_model if installed_model.is_file() else source_model
    )

    arguments = [
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="Ultralytics YOLO person.pt model path",
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
        DeclareLaunchArgument(
            "arm_input_topic",
            default_value="/arm/camera/color/image_raw/compressed",
            description="Arm camera CompressedImage input",
        ),
        DeclareLaunchArgument(
            "arm_detections_topic",
            default_value="/arm/person/detections",
            description="Arm camera JSON detection result topic",
        ),
        DeclareLaunchArgument(
            "arm_output_topic",
            default_value="/arm/person/detection",
            description="Arm camera CompressedImage output with person bounding boxes",
        ),
        DeclareLaunchArgument("enable_arm_camera", default_value="true"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5"),
        DeclareLaunchArgument("infer_size", default_value="640"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("jpeg_quality", default_value="90"),
        DeclareLaunchArgument("publish_visualization", default_value="true"),
    ]

    person_detection = Node(
        package="vision",
        executable="person_detection",
        name="person_detection_yolo",
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
                "arm_input_topic": ParameterValue(
                    LaunchConfiguration("arm_input_topic"), value_type=str
                ),
                "arm_detections_topic": ParameterValue(
                    LaunchConfiguration("arm_detections_topic"), value_type=str
                ),
                "arm_output_topic": ParameterValue(
                    LaunchConfiguration("arm_output_topic"), value_type=str
                ),
                "enable_arm_camera": ParameterValue(
                    LaunchConfiguration("enable_arm_camera"), value_type=bool
                ),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"), value_type=float
                ),
                "infer_size": ParameterValue(
                    LaunchConfiguration("infer_size"), value_type=int
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
            }
        ],
    )

    return LaunchDescription(arguments + [person_detection])
