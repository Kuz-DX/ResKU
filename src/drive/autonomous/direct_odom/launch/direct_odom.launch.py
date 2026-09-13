"""
direct_odom.launch.py

direct_odom_node 단독 실행. reduced_odom_bringup.launch.py(robot_bringup)와 동시에 띄워서
같은 /wheel/odom, /imu 입력으로 /odometry/filtered(EKF) vs
/direct_odom(dead-reckoning) A/B 비교하기 위한 것 -- 센서 드라이버/EKF는
포함하지 않는다(reduced_odom_bringup.launch.py가 이미 띄운다는 전제).

사용 예:
    ros2 launch robot_bringup reduced_odom_bringup.launch.py &
    ros2 launch direct_odom direct_odom.launch.py

    # 또는 rosbag 재생과 함께:
    ros2 launch direct_odom direct_odom.launch.py
    ros2 bag play <bag> --clock

TF는 publish하지 않는다(odom->base_link TF는 EKF가 이미 발행 중이라
충돌 방지).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('output_topic', default_value='/direct_odom'),

        Node(
            package='direct_odom',
            executable='direct_odom_node',
            name='direct_odom_node',
            parameters=[{
                'wheel_odom_topic': wheel_odom_topic,
                'imu_topic': imu_topic,
                'output_topic': output_topic,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'imu_timeout_sec': 0.3,
                'max_dt_sec': 1.0,
            }],
            output='screen',
        ),
    ])
