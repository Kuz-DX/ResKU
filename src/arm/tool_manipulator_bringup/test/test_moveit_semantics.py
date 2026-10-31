"""Static MoveIt semantics and tool-selection regressions; no ROS graph or hardware."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
SRDF = ROOT / "tool_manipulator_moveit_config/config/tool_manipulator.srdf"
XACRO = ROOT / "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro"
JOINTS = {
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
}
HOME = {
    "base_joint": 0.0,
    "shoulder_joint": 1.4830934149601134,
    "elbow_joint": 1.507093414960,
    "wrist_pitch_joint": -1.553343034275,
    "wrist_roll_joint": 0.0,
    "wrist_yaw_joint": 0.0,
}
TCP_READY = {
    "base_joint": 0.0,
    "shoulder_joint": 1.3056807708063805,
    "elbow_joint": 1.3669418648356366,
    "wrist_pitch_joint": -1.2812461346468489,
    "wrist_roll_joint": 0.0,
    "wrist_yaw_joint": 0.0,
}


def _arm_poses():
    root = ET.parse(SRDF).getroot()
    return {
        state.attrib["name"]: {
            joint.attrib["name"]: float(joint.attrib["value"])
            for joint in state.findall("joint")
        }
        for state in root.findall("group_state[@group='arm']")
    }


def test_only_requested_named_arm_poses_exist():
    poses = _arm_poses()
    assert set(poses) == {"home", "back", "tcp_ready"}
    assert set(poses["home"]) == JOINTS
    assert poses["home"] == HOME
    assert set(poses["back"]) == JOINTS
    for name in JOINTS - {"base_joint"}:
        assert poses["back"][name] == HOME[name]
    assert poses["back"]["base_joint"] == pytest.approx(-math.pi, abs=1e-12)
    assert poses["tcp_ready"] == TCP_READY


def test_rmd_urdf_axes_use_the_named_pose_sign_convention():
    result = subprocess.run(
        ["xacro", str(XACRO), "tool_id:=99",
         "use_mock_hardware:=true", "use_mesh:=false"],
        check=True, capture_output=True, text=True,
    )
    root = ET.fromstring(result.stdout)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    for name in ("shoulder_joint", "elbow_joint", "wrist_pitch_joint"):
        assert joints[name].find("axis").attrib["xyz"] == "0 0 1"


def test_tool_case_centres_match_measured_arm_world_coordinates():
    result = subprocess.run(
        ["xacro", str(XACRO), "tool_id:=99",
         "use_mock_hardware:=true", "use_mesh:=false"],
        check=True, capture_output=True, text=True,
    )
    root = ET.fromstring(result.stdout)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    def xyz(name):
        return np.asarray([
            float(value) for value in joints[name].find("origin").attrib["xyz"].split()
        ])

    np.testing.assert_allclose(
        xyz("arm_world_to_tool_case_1"),
        [-0.394506, 0.0, 0.212964], atol=1e-12)
    np.testing.assert_allclose(
        xyz("arm_world_to_tool_case_y_plus"),
        [-0.317806, 0.090, 0.163912], atol=1e-12)
    np.testing.assert_allclose(
        xyz("arm_world_to_supplybox_pickup_area"),
        [-0.236197, -0.120, 0.182], atol=1e-12)


@pytest.mark.parametrize(
    ("tool_id", "expected"),
    [("0", {"tool_1_link"}),
     ("1", {"tool_2_link"}),
     ("2", {"tool_1_link"}),
     ("99", set())],
)
def test_tool_id_expands_only_the_selected_tool(tool_id, expected):
    result = subprocess.run(
        ["xacro", str(XACRO), f"tool_id:={tool_id}",
         "use_mock_hardware:=true", "use_mesh:=false"],
        check=True, capture_output=True, text=True,
    )
    links = {
        link.attrib["name"]
        for link in ET.fromstring(result.stdout).findall("link")
        if link.attrib["name"] in {"tool_1_link", "tool_2_link", "tool_3_link"}
    }
    assert links == expected


def test_drill_geometry_and_tcp_are_aligned_to_ee_output_z_axis():
    result = subprocess.run(
        ["xacro", str(XACRO), "tool_id:=1",
         "use_mock_hardware:=true", "use_mesh:=false"],
        check=True, capture_output=True, text=True,
    )
    root = ET.fromstring(result.stdout)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    mount = joints["tool_2_mount_joint"]
    assert mount.find("parent").attrib["link"] == "ee_output_link"
    assert mount.find("origin").attrib["rpy"] == "0 0 0"
    assert joints["tool_2_tcp_joint"].find("origin").attrib["xyz"] == "0 0 0.060"

    link = root.find("link[@name='tool_2_link']")
    # The STL bounding box is longest on Z, so no mesh rotation is allowed.
    assert link.find("visual/origin") is None
    assert link.find("collision/origin") is None
    assert link.find("inertial/origin").attrib["xyz"] == "0 0 0.030"
