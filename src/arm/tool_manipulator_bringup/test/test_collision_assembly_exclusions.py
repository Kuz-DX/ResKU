"""Regression checks for false proximity at mechanical assembly interfaces."""
from pathlib import Path
import xml.etree.ElementTree as ET

SRDF = Path(__file__).resolve().parents[2] / "tool_manipulator_moveit_config/config/tool_manipulator.srdf"

def pairs():
    return {frozenset((e.attrib["link1"], e.attrib["link2"]))
            for e in ET.parse(SRDF).getroot().findall("disable_collisions")}

def test_rigid_base_and_chassis_are_excluded():
    assert frozenset(("base_actuator", "chassis")) in pairs()

def test_mating_pitch_and_wrist_interfaces_are_excluded():
    for a, b in (("elbow_link", "wrist_link_pitch"),
                 ("elbow_link", "wrist_actuator_roll"),
                 ("wrist_link_pitch", "wrist_link_roll"),
                 ("wrist_link_roll", "wrist_link_yaw")):
        assert frozenset((a, b)) in pairs()

def test_nonadjacent_and_fixture_checks_remain_enabled():
    for a, b in (("base_link", "wrist_link_yaw"),
                 ("elbow_link", "wrist_link_roll"),
                 ("wrist_link_yaw", "circuit_box"),
                 ("wrist_link_yaw", "tool_case_1"),
                 ("base_link", "chassis")):
        assert frozenset((a, b)) not in pairs()
