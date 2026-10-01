"""Publish the arm-camera mounting calibration once for all perception nodes.

Start this launch alongside the arm controller and RealSense driver.  Do not
include it from supply.launch.py or arm_tag_docking_vision.launch.py: those
nodes only consume the TF tree and may run independently.
"""
# 이 launch 파일은 인식 노드들이 공유할 팔-카메라 장착 보정 TF 발행기를 실행합니다.
# 팔 제어기와 RealSense 드라이버와 함께 실행하며, 소비 노드 launch에 중복 포함하지 않습니다.

# 파일 경로를 운영체제 경로 규칙에 맞게 조합하고 문자열로 변환합니다.
from pathlib import Path

# 설치된 패키지의 공유 디렉터리를 찾는 ROS 패키지 인덱스 함수입니다.
from ament_index_python.packages import get_package_share_directory
# 반환할 ROS 2 launch 설명 객체 형식입니다.
from launch import LaunchDescription
# launch 실행 시 설정 파일 경로를 바꿀 수 있도록 인자를 선언합니다.
from launch.actions import DeclareLaunchArgument
# 실행 시점에 launch 인자 값을 읽기 위한 치환 객체입니다.
from launch.substitutions import LaunchConfiguration
# ROS 2 노드를 launch 설명에 추가하는 액션입니다.
from launch_ros.actions import Node


# ROS 2 launch 시스템이 실행할 전체 설명을 구성해 반환합니다.
def generate_launch_description():
    # bringup 패키지의 설치 경로 아래 기본 보정 YAML 경로를 계산합니다.
    default_config = Path(
        # 패키지의 share 디렉터리를 찾아 설치 위치와 무관한 경로를 얻습니다.
        get_package_share_directory('tool_manipulator_bringup')
    # 기본 설정 파일이 있는 config 하위 경로와 파일 이름을 덧붙입니다.
    ) / 'config' / 'arm_camera_extrinsics.yaml'
    # launch 인자로 전달되는 보정 YAML 경로를 실행 시점에 참조합니다.
    config = LaunchConfiguration('extrinsics_config')
    # 인자 선언과 TF 발행기 노드 실행을 하나의 launch 설명에 담습니다.
    return LaunchDescription([
        # 보정 설정 파일 경로를 덮어쓸 수 있는 launch 인자를 등록합니다.
        DeclareLaunchArgument(
            # 명령행 또는 상위 launch에서 사용할 인자 이름입니다.
            'extrinsics_config',
            # 별도 지정이 없으면 패키지 안의 기본 보정 파일을 사용합니다.
            default_value=str(default_config),
            # 카메라 링크에서 팔 카메라 링크로의 측정 외부 파라미터 파일임을 설명합니다.
            description='Measured cam_link <- arm_camera_link extrinsics YAML.',
        ),
        # 보정 YAML을 읽어 팔-카메라 정적 변환을 발행하는 ROS 노드를 시작합니다.
        Node(
            # 실행 파일이 설치된 ROS 패키지 이름입니다.
            package='tool_manipulator_bringup',
            # 실제 TF 발행기 노드의 실행 파일 이름입니다.
            executable='arm_camera_extrinsics_broadcaster.py',
            # ROS 그래프에서 노드를 식별할 이름입니다.
            name='arm_camera_extrinsics',
            # 노드의 표준 출력과 오류 출력을 현재 화면에 표시합니다.
            output='screen',
            # 위 launch 인자에서 가져온 YAML을 노드 파라미터로 전달합니다.
            parameters=[config],
        ),
    ])
