"""The complete toolbox2 assembly shifts; its face angle and height stay fixed."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'tool_change_min'))
from tool_change_min.kinematics import _origin


@pytest.mark.parametrize('tool',['0','1','2','99'])
def test_fixture_mesh_collision_and_both_markers_move_together_without_tilting(tool):
    urdf=ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
    robot=ET.fromstring(xacro.process_file(str(urdf),mappings={'tool_id':tool}).toxml())
    joints={j.find('child').get('link'):j for j in robot.findall('joint')}
    def world(link):
        if link=='arm_world':return np.eye(4)
        joint=joints[link]
        assert joint.get('type')=='fixed'
        return world(joint.find('parent').get('link'))@_origin(joint.find('origin'))
    case=world('tool_case_y_plus')
    np.testing.assert_allclose(case[:3,3],[-.277431292172065,.101709250843959,.172228826],atol=1e-12,rtol=0.)
    np.testing.assert_allclose(world('tool_case_y_plus_center'),case,atol=1e-12,rtol=0.)
    dock=world('tool_case_y_plus_docking')
    np.testing.assert_allclose(dock[:3,3],[-.23290167202993463,.0803325032798424,.2068154938579454],atol=1e-12,rtol=0.)
    link=robot.find("link[@name='tool_case_y_plus']")
    assert link.find('visual/origin').attrib==link.find('collision/origin').attrib
    mesh=case@_origin(link.find('visual/origin'))
    assert np.degrees(np.arccos(abs(mesh[2,2])))==pytest.approx(55.,abs=1e-10)
