"""The temporary gripper exclusion applies only to the test tool2 producer."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_trajectory_joy_test import (TEST_TOOL2_GRIPPER_LINKS, ToolTrajectoryJoy,
                                      collision_result_accepted, load_path)


def response(*pairs):
    return SimpleNamespace(valid=False, contacts=[SimpleNamespace(
        contact_body_1=a, contact_body_2=b) for a, b in pairs])


@pytest.mark.parametrize('link', sorted(TEST_TOOL2_GRIPPER_LINKS))
def test_gripper_contacts_are_ignored_only_with_the_explicit_exclusion(link):
    result = response((link, 'supply_box'))
    assert not collision_result_accepted(result)
    assert collision_result_accepted(result, ignored_links=TEST_TOOL2_GRIPPER_LINKS)
    for pair in [('cam_link', 'circuit_box'), ('wrist_actuator_yaw', 'supply_box'),
                 ('elbow_link', 'chassis')]:
        assert not collision_result_accepted(
            response((link, 'supply_box'), pair), ignored_links=TEST_TOOL2_GRIPPER_LINKS)


def test_exclusions_do_not_accept_missing_results_or_invalid_states_without_contacts():
    for result in [None, response()]:
        assert not collision_result_accepted(result, ignored_links=TEST_TOOL2_GRIPPER_LINKS)


@pytest.mark.parametrize('sampled', [False, True])
def test_exclusion_is_explicit_in_both_test_tool2_configs(sampled):
    directory = ROOT/'config/test'
    if sampled:
        directory /= 'resampled'
    model = Mock()
    model.within_limits.return_value = True
    loaded = load_path(directory/'tool2_trajectory.yaml', model)
    assert loaded['temporary_ignored_collision_links'] == TEST_TOOL2_GRIPPER_LINKS
    for tool in (0, 1):
        assert not load_path(directory/f'tool{tool}_trajectory.yaml', model).get(
            'temporary_ignored_collision_links')


def test_exclusion_cannot_be_extended_or_enabled_on_another_tool(tmp_path):
    source = yaml.safe_load((ROOT/'config/test/tool2_trajectory.yaml').read_text())
    model = Mock()
    model.within_limits.return_value = True
    for name in ('tool0_trajectory.yaml', 'tool1_trajectory.yaml'):
        path = tmp_path/name
        path.write_text(yaml.safe_dump(source))
        with pytest.raises(ValueError, match='tool2 temporary gripper'):
            load_path(path, model)
    path = tmp_path/'tool2_trajectory.yaml'
    for invalid in ['rack_right', ['rack_right'], [*TEST_TOOL2_GRIPPER_LINKS, 'cam_link']]:
        source['temporary_ignored_collision_links'] = invalid
        path.write_text(yaml.safe_dump(source))
        with pytest.raises(ValueError, match='tool2 temporary gripper'):
            load_path(path, model)
    del source['temporary_ignored_collision_links']
    path.write_text(yaml.safe_dump(source))
    assert not load_path(path, model).get('temporary_ignored_collision_links')


@pytest.mark.parametrize('tool,enabled,extra,accepted', [
    (2, True, (), True), (0, True, (), False), (1, True, (), False),
    (2, False, (), False), (2, True, (('cam_link', 'circuit_box'),), False),
])
def test_runtime_validation_keeps_the_exclusion_scoped_to_tool2(tool, enabled, extra, accepted):
    policy = {'temporary_ignored_collision_links': TEST_TOOL2_GRIPPER_LINKS} if enabled else {}
    result = response(('rack_right', 'supply_box'), *extra)
    node = SimpleNamespace(
        stage='checking', path_id=tool, paths={2: policy}, original_epoch=7,
        healthy_hold=Mock(return_value={'epoch': 7}), unchanged=Mock(return_value=True),
        collision_deadline=float('inf'), future=Mock(), request_index=0,
        request_contact_pairs=[frozenset()], requests=[object()], mode=Mock(), fail=Mock())
    node.future.done.return_value = True
    node.future.result.return_value = result
    ToolTrajectoryJoy.tick(node)
    if accepted:
        assert node.stage == 'request_mode'
        node.mode.call_async.assert_called_once()
        node.fail.assert_not_called()
    else:
        node.mode.call_async.assert_not_called()
        node.fail.assert_called_once()
