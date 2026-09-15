"""여름 미션 — side_cameras + summer_traffic + summer_supply.

summer_supply는 사이드캠이 아니라 로봇팔 카메라(/arm/camera/...)를 쓰므로
side_cameras.launch.py와는 무관하다 — side_cameras는 summer_traffic(좌측
사이드캠 단일)용으로만 쓰인다.

"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    side_cameras = os.path.join(
        get_package_share_directory('dolbotz'),
        'launch', 'side_cameras.launch.py')
    # config/models/trafficlightv1.pt — 실행 환경(colcon install)에서의 기본
    # 위치. 소스에 절대경로로 리터럴 박지 않고 launch 인자로 노출하는 건
    # force_mode 하드코딩 사고 이후의 프로젝트 관례(mission_spring.launch.py의
    # led_port/ifof_model_path 주석 참고) — 머신마다 모델 파일을 다른 경로에
    # 둬야 하면 이 인자로 오버라이드하면 된다.
    default_traffic_model_path = os.path.join(
        get_package_share_directory('dolbotz'),
        'config', 'models', 'trafficlightv1.pt')
    traffic_model_path = LaunchConfiguration('traffic_model_path')
    supply_stop_base_x_m = LaunchConfiguration('supply_stop_base_x_m')
    return LaunchDescription([
        DeclareLaunchArgument(
            'traffic_model_path',
            default_value=default_traffic_model_path,
            description=(
                "summer_traffic_node의 신호등 YOLO 모델 경로. 기본값은 "
                "config/models/trafficlightv1.pt(summer_traffic.py의 "
                "TRAFFIC_MODEL_RELATIVE_PATH와 동일한 파일) — 다른 경로의 "
                "모델을 쓰려면 이 인자로 오버라이드할 것."
            ),
        ),
        DeclareLaunchArgument(
            'supply_stop_base_x_m',
            default_value='0.32',
            description=(
                '로봇팔 카메라 supplybox 좌표의 base_actuator 기준 |X|가 '
                '이 값(m) 이하로 연속 확인되면 /arm/target_point(IK 목표)를 전달.'
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(side_cameras)),
        Node(
            package='dolbotz',
            executable='summer_traffic',
            name='summer_traffic_node',
            output='screen',
            parameters=[{
                'model_path': ParameterValue(traffic_model_path, value_type=str),
            }],
        ),
        Node(
            package='dolbotz',
            executable='summer_supply',
            name='summer_supply_node',
            output='screen',
            parameters=[{
                'stop_base_x_m': ParameterValue(
                    supply_stop_base_x_m, value_type=float),
            }],
        ),
        # 박스는 처음부터 가동범위 안에 배치한다. 차체 접근 없이 파지한다.
    ])
