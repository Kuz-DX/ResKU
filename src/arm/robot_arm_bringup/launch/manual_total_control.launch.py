"""Manual-only hardware bringup with MANUAL_EE and MANUAL_100 modes."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('use_mesh', default_value='true'),
        DeclareLaunchArgument('launch_joy', default_value='true'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('control_toggle_button', default_value='9'),
        # AUTO is intentionally disabled in this manual mission launch.
        # Keep the old argument here as a reference for the future AUTO launch:
        # DeclareLaunchArgument('auto_mode_button', default_value='8'),
        DeclareLaunchArgument('manual_100_mode_button', default_value='8'),
        DeclareLaunchArgument('manual_ee_pause_button', default_value='10'),
    ]
    use_mock = LaunchConfiguration('use_mock_hardware')
    use_mesh = LaunchConfiguration('use_mesh')
    xacro_file = PathJoinSubstitution([
        FindPackageShare('army_manipulator_description'), 'urdf',
        'army_manipulator.urdf.xacro'])
    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_mock_hardware:=', use_mock, ' use_mesh:=', use_mesh]), value_type=str)}
    controllers = PathJoinSubstitution([
        FindPackageShare('army_manipulator_bringup'), 'config',
        'ros2_controllers.yaml'])
    mux_config = PathJoinSubstitution([
        FindPackageShare('robot_arm_bringup'), 'config', 'joint_command_mux.yaml'])

    state_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    controller_spawners = RegisterEventHandler(OnProcessExit(
        target_action=state_spawner,
        on_exit=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['position_controller', '-c', '/controller_manager']),
            # AUTO mission only. Do not load these trajectory controllers in
            # the manual launch; preserve the definitions for later reuse.
            # Node(package='controller_manager', executable='spawner',
            #      arguments=['arm_controller', '-c', '/controller_manager', '--inactive']),
            # Node(package='controller_manager', executable='spawner',
            #      arguments=['gripper_controller', '-c', '/controller_manager', '--inactive']),
        ]))
    # AUTO mission only. The future AUTO launch should own MoveIt and maru IK.
    # move_group = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     PathJoinSubstitution([FindPackageShare('army_manipulator_moveit_config'),
    #                           'launch', 'move_group.launch.py'])))
    nodes = [
        SetEnvironmentVariable('ARMY_MANIPULATOR_USE_MESH', use_mesh),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        # DolbotZ-Center가 MANUAL_EE에서도 AUTO와 동일한 실제 TF 기반 관절
        # 좌표를 사용하도록 UI 전용 PoseArray를 계속 발행한다.
        Node(package='army_manipulator_bringup',
             executable='arm_pose_array_publisher.py',
             name='arm_pose_array_publisher', output='screen',
             parameters=[{'publish_hz': 20.0}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[robot_description, controllers], output='screen'),
        state_spawner, controller_spawners,
        # AUTO mission only:
        # move_group,
        # Node(package='army_manipulator_bringup', executable='maru_ik_node.py',
        #      output='screen'),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'dev': LaunchConfiguration('joy_dev'), 'deadzone': 0.08}],
             condition=IfCondition(LaunchConfiguration('launch_joy'))),
        Node(package='robot_arm_bringup', executable='safety_manager.py',
             output='screen', parameters=[{
                 'control_toggle_button': LaunchConfiguration('control_toggle_button'),
                 # A negative button index can never produce a rising edge,
                 # so AUTO cannot be selected from this manual-only launch.
                 'auto_mode_button': -1,
                 'manual_100_mode_button': LaunchConfiguration(
                     'manual_100_mode_button'),
                 'manual_ee_pause_button': LaunchConfiguration(
                     'manual_ee_pause_button')}]),
        Node(package='robot_arm_bringup', executable='gamepad_position_controller.py',
             output='screen', parameters=[{
                 'shoulder_axis': 4, 'elbow_axis': 1, 'wrist_axis': 7,
                 'base_yaw_positive_button': 4, 'base_yaw_negative_button': 5,
                 'command_topic': '/manual_joint_commands'}]),
        # TODO: 실물 구동 전 joystick 축과 wrist_link의 실제 이동 방향을 반드시
        # 저속으로 검증한다. 현재 MANUAL_EE는 독립 differential IK 경로이며,
        # AUTO 상위제어 공용화는 상위제어 수정이 필요하므로 보류되어 있다.
        Node(package='robot_arm_bringup', executable='ee_manual_controller.py',
             output='screen', parameters=[robot_description]),
        Node(package='robot_arm_bringup', executable='manual_gripper_controller.py',
             output='screen'),
        Node(package='robot_arm_bringup', executable='gripper_hold_fin.py',
             name='gripper_hold_fin', output='screen', parameters=[{
                 'position_threshold': 1.8294,
                 'effort_threshold': 130.0,
                 'output_topic': '/gripper_hold_fin'}]),
        Node(package='robot_arm_bringup', executable='joint_command_mux.py',
             output='screen', parameters=[mux_config]),
    ]
    return LaunchDescription(args + nodes)
