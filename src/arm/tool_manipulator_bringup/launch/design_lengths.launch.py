"""Independent mock MoveIt model with adjustable link lengths (metres)."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import tempfile
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def build(context):
    # 모든 노드에 같은 생성 모델을 전달한다.
    mappings = {"use_mock_hardware": "true", "use_mesh": LaunchConfiguration("use_mesh").perform(context), "design_tool_changer_length": "0.110"}
    # L3는 wrist→extra 장착 길이이며, 말단 extra 설계에서는 짧아질 수 있다.
    for name, lower, upper in (("L1", .160, .200), ("L2", .180, .260), ("L3", .020, .170)):
        value = float(LaunchConfiguration(name).perform(context))  # 길이 [m]。
        if not lower <= value <= upper:
            raise ValueError(f"{name} must be between {lower} and {upper} m")
        mappings["design_" + name] = str(value)
    mount_x = float(LaunchConfiguration("chassis_mount_x").perform(context))
    # 차체 중앙(world x=0)에서 전방(+x)으로의 base 장착 위치.
    if not .22 <= mount_x <= .39:
        raise ValueError("chassis_mount_x must be between 0.22 and 0.39 m")
    mappings["chassis_mount_x"] = str(mount_x)
    path = Path(get_package_share_directory("tool_manipulator_description")) / "urdf/tool_manipulator_design.urdf.xacro"
    config = (MoveItConfigsBuilder("tool_manipulator", package_name="tool_manipulator_design_moveit_config")
              .robot_description(file_path=str(path), mappings=mappings)
              .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    # 0은 추가축 미장착. 양수는 extra MX-106T에서 EE까지의 구조 길이이며
    # wrist→extra(L3)와 합친 구조 길이는 최소 140 mm다.
    extra = float(LaunchConfiguration("extra_pitch").perform(context))
    if extra != 0 and not .11 <= extra <= .20:
        raise ValueError("extra_pitch must be 0 or 0.11..0.20 m")
    l3 = float(LaunchConfiguration("L3").perform(context))
    if extra > 0 and l3 + extra < .14:
        raise ValueError("L3 (wrist-to-extra) + extra_pitch (extra-to-EE) must be at least 0.14 m")
    controllers = yaml.safe_load((config.package_path / "config/ros2_controllers.yaml").read_text())
    robot = ET.fromstring(config.robot_description["robot_description"])
    semantic = ET.fromstring(config.robot_description_semantic["robot_description_semantic"])
    # 기존 설정의 차체 충돌 무시 항목은 설계 검토에 부적합하여 복제본에서 복구한다.
    for item in list(semantic.findall("disable_collisions")):
        if "chassis_link" in (item.get("link1"), item.get("link2")) and item.get("reason") != "Adjacent":
            semantic.remove(item)
    if extra > 0:
        # pinion 출력(wrist 링크 뒤)에 extra MX-106T 피치축을 둔다.
        # L3는 wrist→extra 거리이며 extra는 EE만 지지한다.
        tool_joint = robot.find("joint[@name='tool_changer_joint']")
        tool_joint.find("child").set("link", "extra_pitch_mount")
        tool_joint.find("origin").set("xyz", "0 0 0")
        mount = ET.SubElement(robot, "link", name="extra_pitch_mount")
        inertial = ET.SubElement(mount, "inertial")
        ET.SubElement(inertial, "mass", value="0.153")  # MX-106T 단품 질량 [kg].
        ET.SubElement(inertial, "inertia", ixx="0.0001", iyy="0.0001", izz="0.0001", ixy="0", ixz="0", iyz="0")
        for kind in ("visual", "collision"):
            shape = ET.SubElement(mount, kind)
            ET.SubElement(shape, "origin", xyz="0 0 0")
            ET.SubElement(ET.SubElement(shape, "geometry"), "cylinder", radius="0.030", length="0.050")
        joint = ET.SubElement(robot, "joint", name="extra_pitch_joint", type="revolute")
        ET.SubElement(joint, "parent", link="extra_pitch_mount")
        ET.SubElement(joint, "child", link="tool_changer_link")
        ET.SubElement(joint, "origin", xyz="0 0 0")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        # 설계용 가정 제한. effort는 연속 정격이 아니며 mock 인터페이스용이다.
        ET.SubElement(joint, "limit", lower="-1.5708", upper="1.5708", effort="8.4", velocity="0.5")
        control = ET.SubElement(robot.find("ros2_control"), "joint", name="extra_pitch_joint")
        ET.SubElement(control, "command_interface", name="position")
        state = ET.SubElement(control, "state_interface", name="position")
        ET.SubElement(state, "param", name="initial_value").text = "0"
        ET.SubElement(control, "state_interface", name="velocity")
        # EE 링크는 MX-106T 동력전달부(153 g)로 보고, extra 축 기준 +Z 방향으로
        # 배치한다. 이 길이가 static-screening의 extra→EE 길이와 같다.
        ee_link = robot.find("link[@name='tool_changer_link']")
        for shape in list(ee_link.findall("visual")) + list(ee_link.findall("collision")):
            origin = shape.find("origin")
            if origin is None:
                origin = ET.SubElement(shape, "origin")
            origin.set("xyz", f"0 0 {extra / 2:.6f}")
            cylinder = shape.find("geometry/cylinder")
            if cylinder is not None:
                cylinder.set("length", f"{extra:.6f}")
        inertial = ee_link.find("inertial")
        inertial.find("mass").set("value", "0.153")
        inertial.find("origin").set("xyz", f"0 0 {extra / 2:.6f}")
        robot.find("joint[@name='tcp_joint']/origin").set("xyz", f"0 0 {extra:.6f}")
        # arm을 TCP까지의 단일 체인으로 확장한다. extra가 pinion/gripper 뒤에
        # 있으므로 joint를 별도 추가하면 KDL group이 비체인이 된다.
        semantic.find("group[@name='arm']/chain").set("tip_link", "tool_changer_link")
        for state in semantic.findall("group_state[@group='arm']"):
            ET.SubElement(state, "joint", name="extra_pitch_joint", value="0")
            ET.SubElement(state, "joint", name="gripper_joint", value="0")
        for state in semantic.findall("group_state[@group='gripper']"):
            ET.SubElement(state, "joint", name="extra_pitch_joint", value="0")
        ET.SubElement(semantic, "disable_collisions", link1="extra_pitch_mount", link2="tool_changer_link", reason="Adjacent")
        controllers["arm_controller"]["ros__parameters"]["joints"].append("extra_pitch_joint")
        config.trajectory_execution["moveit_simple_controller_manager"]["arm_controller"]["joints"].append("extra_pitch_joint")
    config.robot_description["robot_description"] = ET.tostring(robot, encoding="unicode")
    config.robot_description_semantic["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")
    # 각도 슬라이더 모드는 별도 JSP GUI를 쓰며 mock controller와 동시 발행하지 않는다.
    if LaunchConfiguration("angle_sliders").perform(context).lower() == "true":
        return [SetLaunchConfiguration("allow_trajectory_execution", "false"), Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[config.robot_description]),
                Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", parameters=[config.robot_description]),
                *generate_moveit_rviz_launch(config).entities,
                *generate_move_group_launch(config).entities]
    # controller_manager는 컨트롤러별 ROS parameter 파일을 필요로 한다.
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as handle:
        yaml.safe_dump(controllers, handle)
        controller_file = handle.name
    def cleanup(context):
        if os.path.exists(controller_file):
            os.unlink(controller_file)
        return []
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[config.robot_description]),
             Node(package="controller_manager", executable="ros2_control_node", parameters=[config.robot_description, controller_file])]
    for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller"):
        nodes.append(Node(package="controller_manager", executable="spawner", arguments=[name]))
    nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    nodes.extend(generate_move_group_launch(config).entities)
    nodes.extend(generate_moveit_rviz_launch(config).entities)
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("L1", default_value="0.180"),
                              DeclareLaunchArgument("L2", default_value="0.220"),
                              DeclareLaunchArgument("L3", default_value="0.150"),
                              DeclareLaunchArgument("chassis_mount_x", default_value="0.278"),
                              DeclareLaunchArgument("extra_pitch", default_value="0.110"),
                              DeclareLaunchArgument("angle_sliders", default_value="false"),
                              DeclareLaunchArgument("use_mesh", default_value="true"),
                              OpaqueFunction(function=build)])
