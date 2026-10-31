"""Calibration consistency across copied installs and real control xacro."""
from pathlib import Path
import math
import runpy
import xml.etree.ElementTree as ET
import importlib.util
from unittest.mock import Mock, patch

import pytest
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
HELPER = runpy.run_path(str(ROOT / 'launch/hardware_config.py'))


@pytest.mark.parametrize('isolated', [True, False])
def test_copied_install_prefers_calibration_in_its_own_workspace(tmp_path, isolated):
    prefix = tmp_path / 'install'
    if isolated:
        prefix /= 'tool_manipulator_bringup'
    share = prefix / 'share/tool_manipulator_bringup'
    installed = share / 'config/hardware.yaml'
    installed.parent.mkdir(parents=True)
    installed.write_text('stale install')
    source = tmp_path / 'src/arm/tool_manipulator_bringup/config/hardware.yaml'
    source.parent.mkdir(parents=True)
    source.write_text('updated calibration')
    assert HELPER['default_hardware_config'](share) == source
    source.unlink()
    assert HELPER['default_hardware_config'](share) == installed


@pytest.mark.parametrize('zero', [1561, 1703, -2535])
def test_real_control_uses_updated_zero_and_rmd_offsets(tmp_path, zero):
    config = yaml.safe_load((ROOT / 'config/hardware.yaml').read_text())
    config['joints']['base_joint']['zero_raw'] = zero
    config['joints']['shoulder_joint']['q_offset_rad'] = -1.596278117895
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump(config))
    xml = xacro.process_file(str(ROOT.parent / 'tool_manipulator_moveit_config/config/tool_manipulator.urdf.xacro'),
        mappings={'use_mock_hardware': 'false', 'hardware_config_file': str(path),
                  'use_mesh': 'false'}).toxml()
    robot = ET.fromstring(xml)
    base = robot.find("ros2_control/joint[@name='base_joint']")
    offset = float(base.find("param[@name='position_zero_offset']").text)
    assert offset == pytest.approx(zero * 2 * math.pi / 4096 - math.pi)
    assert zero * 2 * math.pi / 4096 - math.pi - offset == pytest.approx(0., abs=1e-12)
    assert (offset + math.pi) * 4096 / (2 * math.pi) == pytest.approx(zero)
    shoulder = robot.find("ros2_control[@name='rmd_shoulder_joint']/hardware/param[@name='q_offset']")
    assert float(shoulder.text) == pytest.approx(-1.596278117895)


@pytest.mark.parametrize('filename', ['manual.launch.py', 'ik_manual.launch.py', 'ik_manual_test.launch.py'])
def test_requested_launches_select_same_editable_calibration(filename):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('calibration_launch', ROOT / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch('launch.logging.get_logger', return_value=Mock()):
        description = module.generate_launch_description()
        context = LaunchContext()
    arg = next(item for item in description.entities
               if isinstance(item, DeclareLaunchArgument) and item.name == 'hardware_config')
    value = ''.join(item.perform(context) for item in arg.default_value)
    assert Path(value).resolve() == ROOT / 'config/hardware.yaml'


@pytest.mark.parametrize('mode', ['manual', 'managed'])
def test_options_at_calibrated_base_zero_does_not_rotate_base(mode):
    from test_manual import harness, JOINTS
    from arm_mode_core import ModeCore
    config = yaml.safe_load((ROOT / 'config/hardware.yaml').read_text())
    raw = config['joints']['base_joint']['zero_raw']
    xml = xacro.process_file(str(ROOT.parent / 'tool_manipulator_moveit_config/config/tool_manipulator.urdf.xacro'),
        mappings={'use_mock_hardware': 'false', 'hardware_config_file': str(ROOT / 'config/hardware.yaml'),
                  'use_mesh': 'false'}).toxml()
    offset = float(ET.fromstring(xml).find(
        "ros2_control/joint[@name='base_joint']/param[@name='position_zero_offset']").text)
    measured = [raw * 2 * math.pi / 4096 - math.pi - offset, .4, -.3, -.5, -.04, .02]
    if mode == 'managed':
        c = ModeCore([(-2., 2.)]*6, [.12]*6)
        c.entry_alignment_enabled = True
        c.entry_alignment_complete = False
        c.ready, c.focus = True, 'arm'
        c.feedback(measured, [0.]*6, 1.)
        for step in range(1, 130):
            now = 1.+step*.02
            c.feedback(measured, [0.]*6, now)
            measured = list(c.tick(now, .02))
            assert measured[0] == pytest.approx(0., abs=1e-12)
            assert (offset + measured[0] + math.pi)*4096/(2*math.pi) == pytest.approx(raw)
        assert c.entry_alignment_complete
    else:
        node = harness()
        node.entry_alignment_complete = False
        node.positions.update(zip(JOINTS, measured))
        node.target = node.measured()
        node.focus, node.joy_time = 'arm', None
        for step in range(1, 130):
            now = 1.+step*.02
            node.now_seconds = lambda: now
            node.state_times = dict.fromkeys(JOINTS, now)
            node.last_time = now-.02
            node.on_timer()
            assert node.target[0] == pytest.approx(0., abs=1e-12)
            assert (offset + node.target[0] + math.pi)*4096/(2*math.pi) == pytest.approx(raw)
            node.positions = dict(zip(JOINTS, node.target))
            if node.entry_alignment_complete:
                break
        assert node.entry_alignment_complete
