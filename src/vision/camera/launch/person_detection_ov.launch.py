# TEST-ONLY: isolated person-detection CPU/OpenVINO benchmark launch file.
"""Launch the isolated person-detection backend/scheduler benchmark node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _argument(name: str, default: str, description: str = '') -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description() -> LaunchDescription:
    """Declare every test-node parameter and launch one isolated node."""
    installed = Path(get_package_share_directory('vision')) / 'models' / 'person.pt'
    source = Path(__file__).resolve().parents[2] / 'models' / 'person.pt'
    default_model = str(installed if installed.is_file() else source)

    defaults = {
        'model_path': default_model,
        'backend': 'ultralytics',
        'perf_hint': 'LATENCY',
        'num_requests': '1',
        'scheduler': 'fifo',
        'priority_pattern': "['drive','arm','drive','left','right']",
        'max_wait_ms': '300',
        'stale_input_ms': '1000',
        'async_infer': 'false',
        'log_period_s': '5.0',
        'csv_dir': '~/perf_logs',
        'run_tag': '',
        'latency_mode': 'receive',
        'topic_suffix': '_ov',
        'qos_depth': '0',
        'print_table': 'true',
        'input_topic': '/drive/camera/color/image_raw/compressed',
        'detections_topic': '/person_detection/detections',
        'output_topic': '/drive/person/detecion',
        'left_input_topic': '/side/left/image_raw/compressed',
        'left_detections_topic': '/left/person/detections',
        'left_output_topic': '/left/person/detection',
        'right_input_topic': '/side/right/image_raw/compressed',
        'right_detections_topic': '/right/person/detections',
        'right_output_topic': '/right/person/detection',
        'arm_input_topic': '/arm/camera/color/image_raw/compressed',
        'arm_detections_topic': '/arm/person/detections',
        'arm_output_topic': '/arm/person/detection',
        'enable_arm_camera': 'true',
        'enable_side_cameras': 'true',
        'confidence_threshold': '0.5',
        'infer_size': '640',
        'device': 'cpu',
        'jpeg_quality': '90',
        'publish_visualization': 'true',
    }
    descriptions = {
        'model_path': 'PT path, OpenVINO model folder, or OpenVINO XML path',
        'topic_suffix': 'Suffix appended only to detection and visualization outputs',
        'qos_depth': '0 uses sensor_data; positive values create KEEP_LAST depth N',
    }
    arguments = [
        _argument(name, value, descriptions.get(name, ''))
        for name, value in defaults.items()
    ]
    types = {
        'num_requests': int,
        'priority_pattern': list[str],
        'max_wait_ms': int,
        'stale_input_ms': int,
        'async_infer': bool,
        'log_period_s': float,
        'qos_depth': int,
        'print_table': bool,
        'enable_arm_camera': bool,
        'enable_side_cameras': bool,
        'confidence_threshold': float,
        'infer_size': int,
        'jpeg_quality': int,
        'publish_visualization': bool,
    }
    parameters = {
        name: ParameterValue(
            LaunchConfiguration(name), value_type=types.get(name, str)
        )
        for name in defaults
    }
    node = Node(
        package='vision',
        executable='person_detection_ov',
        name='person_detection_ov',
        output='screen',
        emulate_tty=True,
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [node])
