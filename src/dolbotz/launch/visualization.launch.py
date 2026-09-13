"""
visualization.launch.py

"경로 주행" 스택 통합 RViz + 그 전제조건 릴레이 노드들.

기존에 slope_decision의 OpenCV 창, path_visualizer.py(노트북 단독 실행),
arm_visualizer.py처럼 디버그 뷰어가 따로따로 흩어져 있어서 실행 중인
로봇이 지금 뭘 보고 어디로 가려는지 한눈에 볼 방법이 없었음 -- 이 launch
하나로 그걸 RViz에 통합한다(이번 스코프는 경로 주행 관련 스택만 --
사이드캠/암캠 이미지, 미션 판정 결과는 포함 안 함).

이 launch가 띄우는 것:
  1. terrain_viz_relay (신규, utils/terrain_viz_relay.py) -- /terrain/elevation_map,
     /terrain/slope_deg, /terrain/gradient_magnitude(전부 32FC1)를 NaN-안전
     컬러맵 8bit(bgr8)로 재발행. RViz Image 디스플레이가 그대로 못 띄우는
     인코딩이라 필요 (자세한 이유는 terrain_viz_relay.py 및
     rviz/dolbotz_visualization.rviz 상단 주석 참고).
  2. slope_visualizer (기존 노드, utils/slope_visualizer.py) -- /debug/slope_markers
     발행. 새로 만든 게 아니라 이미 있던 걸 여기서 같이 띄우는 것뿐.
  3. path_camera_overlay_relay (신규, utils/path_camera_overlay_relay.py) --
     /path를 카메라 원본(원근) 시점 화면에 빨간 선으로 덧씌워 재발행.
     실기에서 BEV(/bev/centerline_overlay, 균일 축척)만 보면 짧은 거리도
     "끊긴 것처럼" 보여서 직관적으로 헷갈린다는 피드백으로 추가 -- 카메라
     시점은 사람이 실제로 보는 느낌과 비슷해서 더 잘 읽힘(대신 원근 때문에
     가까운 거리가 과장돼 보이는 착시가 있다는 점은 감안).
  4. RViz2 (start_rviz:=true일 때만) -- dolbotz_visualization.rviz 설정으로.

start_rviz 인자로 GUI만 끌 수 있게 한 이유: 위 1~3번 릴레이 노드는 가볍고
"누가 지금 보고 있냐"와 무관하게 계속 떠 있어도 무해하지만(다른 노드가
_viz/slope_markers/path_camera_overlay 토픽을 구독할 수도 있음), RViz2 GUI
프로세스 자체는 헤드리스(SSH, 로봇 본체 단독 실행 등)에서 X 디스플레이가
없으면 죽거나 아예 못 뜬다 -- enable_visualizer(mission_winter.launch.py의
slope_decision OpenCV 창 토글)와 같은 스타일의 안전장치.

주의(요구사항 5 관련, dolbotz_visualization.rviz 상단 주석에 더 자세히):
/path -> odom TF 변환이 실제로 되는지는 이 launch 자체가 검증하지 않는다
-- path_relay_node.cpp 자신의 주석에도 "실기 확인 필요"라고 돼 있던 항목.
RViz에서 transformed_global_plan Path가 로봇 근처에 말이 되게 뜨는지 눈으로
확인하는 게 지금 가장 쉬운 검증 방법이다.

전제조건: perception 스택(mission_winter/spring/summer/fall.launch.py 중 하나)과,
MPPI 궤적/costmap/odom을 보려면 autonomous.launch.py(또는 최소
reduced_odom_bringup.launch.py+nav2.launch.py)도 같이 떠 있어야 한다 -- 이 launch 파일은 그
전제조건들을 스스로 띄우지 않는다(다른 mission_*.launch.py들과 나란히,
독립적으로 켜고 끄는 것이 이 프로젝트의 기존 컨벤션이라 그대로 따름 --
howtorun.md의 "여러 터미널로 따로 띄우기" 패턴 참고).

사용법:
    ros2 launch dolbotz visualization.launch.py                       # RViz까지 켬
    ros2 launch dolbotz visualization.launch.py start_rviz:=false     # 릴레이 노드만(헤드리스)
    ros2 launch dolbotz visualization.launch.py colormap:=turbo       # 컬러맵 변경
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('dolbotz'),
        'rviz', 'dolbotz_visualization.rviz')

    start_rviz = LaunchConfiguration('start_rviz')
    colormap = LaunchConfiguration('colormap')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description=(
                "RViz2 GUI 프로세스를 띄울지 여부. X 디스플레이 없는 헤드리스 "
                "환경(SSH, 로봇 본체 단독 실행 등)에서는 'false'로 꺼야 한다 -- "
                "꺼도 terrain_viz_relay/slope_visualizer 릴레이 노드는 그대로 뜬다."
            ),
        ),
        DeclareLaunchArgument(
            'colormap',
            default_value='jet',
            description="terrain_viz_relay의 컬러맵 (jet/turbo/viridis).",
        ),

        Node(
            package='dolbotz',
            executable='terrain_viz_relay',
            name='terrain_viz_relay',
            parameters=[{'colormap': colormap}],
            output='screen',
        ),
        Node(
            package='dolbotz',
            executable='slope_visualizer',
            name='slope_visualizer',
            output='screen',
        ),
        Node(
            package='dolbotz',
            executable='path_camera_overlay_relay',
            name='path_camera_overlay_relay',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(start_rviz),
        ),
    ])
