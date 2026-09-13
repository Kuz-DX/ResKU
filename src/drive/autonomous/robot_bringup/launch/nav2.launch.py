"""
nav2.launch.py

Nav2 MPPI controller_server bringup. Step 3 of the doldrive_ws pipeline.

Verified via dry-run (vcan0 + fake IMU/wheel-odom + path_publisher_test
sending a FollowPath goal directly, bypassing bt_navigator): controller_server
activates cleanly, SkidCritic/SlopeCritic load and score correctly, and
/cmd_vel_auto comes out sane (linear.x capped at vx_max, angular.z tracking
the path curvature) -- including through a simulated uphill/downhill slope
(see path_publisher_test.cpp).

Found and fixed in this pass: SlopeCritic used to subscribe to /imu directly
inside its own initialize(), which reproducibly crashed controller_server
with SIGSEGV (isolated via apport coredump + gdb backtrace + A/B testing
with/without each custom critic) -- root cause was mixing an independent
executor-thread subscription callback with the critic being scored from the
optimizer's own action-execution thread. Fixed by reading pitch from
data.state.pose.pose.orientation (already passed into every score() call,
already reflects the EKF-fused IMU orientation via odom->base_link TF)
instead of a separate subscription. our_mppi_critics no longer depends on
sensor_msgs.

[하림 수정] 2026-08-22: planner_server/global_costmap/bt_navigator를
제거함 -- 실제 주행 경로(perception -> /path -> path_relay_node ->
FollowPath -> controller_server)는 이 셋을 전혀 거치지 않고(path_relay_
node.cpp 자체 문서화된 설계), RViz "2D Nav Goal"(NavigateToPose 액션 ->
bt_navigator -> ComputePathToPose -> planner_server -> FollowPath) 방식을
실제로 써본 적이 없는 것으로 확인됨(우리가 지금까지 검증에 쓴 건 완성된
경로를 FollowPath에 직접 찔러넣는 synthetic_quarter_arc_path.py류이지,
점 하나 찍고 Nav2가 전역경로를 계산하게 하는 방식이 아니었음).
유지하는 대가로 nav2_lifecycle_manager가 node_names 안의 모든 노드가
CONFIGURE를 통과해야만 ACTIVATE로 넘어가는 구조(lifecycle_manager.cpp:
279-292, changeStateForAllNodes)라, planner_server/bt_navigator의 설정
문제가 실제로 쓰이는 controller_server(MPPI)의 activate까지 막을 수
있는 불필요한 결합 리스크만 남아있었음 -- 안 쓰는 기능 때문에 쓰는
기능이 죽을 수 있는 구조라 아예 제거. 나중에 point-to-point 내비게이션이
실제로 필요해지면(예: map_server/SLAM 붙는 시점) 그때 다시 추가할 것
-- our_mppi_params.yaml git 이력에 이전 설정이 남아있음.

Run reduced_odom_bringup.launch.py first/alongside this - controller_server가 subscribe하는
'odom'/odom_topic이 /odometry/filtered로 remap되며, 이 토픽은 이름 호환성을
유지한 production reduced_odom_node가 발행한다.

critics: SkidCritic/SlopeCritic come from our_mppi_critics, plugged into
the stock (apt-installed) nav2_mppi_controller - NOT the full fork in
mppi_ws/our_mppi_controller (deliberately not migrated, see project
decision).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mppi_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'our_mppi_params.yaml')

    return LaunchDescription([

        # Nav2 MPPI 컨트롤러 서버 (출력 토픽을 MUX 노드로 들어가도록 리맵핑)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                mppi_params,
                {'use_sim_time': False},
            ],
            remappings=[
                ('odom', '/odometry/filtered'),
                ('/cmd_vel', '/cmd_vel_auto'),  # 자율주행 명령을 MUX 노드로 우회
            ],
            output='screen',
        ),

        # controller_server(및 내부 local_costmap)를 configure->activate까지
        # 자동 전환. 없으면 unconfigured 상태로 멈춰 FollowPath 액션서버
        # 자체가 생성되지 않음. planner_server/bt_navigator 제거 후에는
        # controller_server 하나만 관리하면 되므로, 더 이상 무관한 노드의
        # configure 실패가 여기 발목을 잡을 일이 없음.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),
    ])
