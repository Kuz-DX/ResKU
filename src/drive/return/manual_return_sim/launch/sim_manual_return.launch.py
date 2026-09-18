"""
sim_manual_return.launch.py

Pre-flight virtual test rig for the manual+return mission -- runs the REAL
mission stack (rmd_x8_driver_node, reduced_odom_node, drive_cmd_mux_node,
manual_path_recorder_node, return_state_machine_node,
return_path_follower_node) completely unmodified against a simulated CAN
motor pair + simulated IMU (sim_rmd_x8_hardware) on vcan0, instead of real
hardware. Opens RViz2 to watch the recorded/return path live.

One-time setup (needs root, run yourself -- this launch does not do it):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

Usage:
    ros2 launch manual_return_sim sim_manual_return.launch.py

There is no real joystick in this rig -- drive it by hand with
`ros2 topic pub` on /motor_speed_cmd_manual (std_msgs/Float32MultiArray,
[left_dps, right_dps]), e.g.:
    ros2 topic pub -r 20 /motor_speed_cmd_manual std_msgs/msg/Float32MultiArray \\
        "{data: [150.0, 150.0]}"   # drive forward
    ros2 topic pub -r 20 /motor_speed_cmd_manual std_msgs/msg/Float32MultiArray \\
        "{data: [-100.0, 100.0]}"  # turn in place
Ctrl+C the pub to stop, then trigger RETURN:
    ros2 topic pub -1 /mission/return/trigger std_msgs/msg/Bool "{data: true}"

Watch /recorded_path grow (green) then /return_path (orange) get followed
back toward the origin in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')

    reduced_odom_params = os.path.join(
        get_package_share_directory('reduced_odom'), 'config', 'reduced_odom.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('manual_return_sim'), 'config', 'sim_manual_return.rviz')

    motor_params = {
        'can_interface': can_interface,
        'left_motor_can_id': 1,
        'right_motor_can_id': 2,
        'left_direction_sign': 1.0,
        'right_direction_sign': -1.0,
        'wheel_radius_m': 0.1125,
        'external_gear_ratio': 1.0,
        'effective_track_width_m': 0.4904,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value='vcan0',
            description='Virtual CAN interface (create it first, see this file\'s docstring)'),

        # base_link -> imu_link static TF -- reduced_odom_node needs this to
        # rotate the fake IMU's orientation into base_link (imuToBase()).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sim_base_to_imu_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),

        # Real low-level CAN driver, pointed at the virtual bus.
        Node(
            package='rmd_x8_driver',
            executable='rmd_x8_driver_node',
            name='rmd_x8_driver',
            parameters=[motor_params],
            output='screen',
        ),

        # Fake motors + fake IMU on the same virtual bus.
        Node(
            package='manual_return_sim',
            executable='sim_rmd_x8_hardware',
            name='sim_rmd_x8_hardware',
            parameters=[motor_params],
            output='screen',
        ),

        # Real EKF fusion, unmodified.
        Node(
            package='reduced_odom',
            executable='reduced_odom_node',
            name='reduced_odom_node',
            parameters=[reduced_odom_params],
            output='screen',
        ),

        # Real manual+return mission nodes, unmodified.
        Node(
            package='drive_cmd_mux',
            executable='drive_cmd_mux_node',
            name='drive_cmd_mux_node',
            output='screen',
        ),
        Node(
            package='return_navigation',
            executable='manual_path_recorder_node',
            name='manual_path_recorder_node',
            output='screen',
        ),
        Node(
            package='return_navigation',
            executable='return_state_machine_node',
            name='return_state_machine_node',
            output='screen',
        ),
        Node(
            package='return_navigation',
            executable='return_path_follower_node',
            name='return_path_follower_node',
            output='screen',
        ),

        # Debug-only: odom->mission TF (so RViz can render mission-frame
        # paths) + cross-cycle path history markers.
        Node(
            package='manual_return_sim',
            executable='sim_debug_viz',
            name='sim_debug_viz',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
