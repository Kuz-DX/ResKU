"""Tool2 opens with stopped-arm validation before any vertical approach."""
from pathlib import Path
import sys
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_trajectory_joy_test import ARM_JOINTS, ToolTrajectoryJoy
from tool2_pick_place_coordinator_test import Tool2PickPlaceCoordinator

POLICY = dict(ee_open_rad=-.076699, ee_open_tolerance_rad=.01)


def node(ee):
    instance = SimpleNamespace(paths={2: {'vertical_descent': POLICY}},
        full_state=dict(zip(ARM_JOINTS, [0.]*6), ee_joint=ee),
        tool2_open_ready=False, tool2_opening=False,
        healthy_hold=Mock(return_value={}), report=Mock(), begin=Mock())
    instance.tool2_open_verified = MethodType(ToolTrajectoryJoy.tool2_open_verified, instance)
    instance.require_tool2_open = MethodType(ToolTrajectoryJoy.require_tool2_open, instance)
    return instance


def test_closed_fingers_trigger_validated_ee_only_opening_before_arm_motion():
    instance = node(0.)
    ToolTrajectoryJoy.begin(instance, 2)
    instance.begin.assert_called_once()
    path_id, opening = instance.begin.call_args.args
    assert path_id == 2 and instance.tool2_opening
    assert opening['ee_target_rad'] == POLICY['ee_open_rad']
    np.testing.assert_array_equal(opening['start'], opening['waypoints'][0])
    assert opening['ee_speed_rad_s'] == .08


@pytest.mark.parametrize('ee,allowed', [(-.076699, True), (-.067, True),
                                       (0., False), (-.09, False)])
def test_arm_continuation_requires_confirmed_opening(ee, allowed):
    instance = node(ee)
    instance.tool2_opening = True
    if allowed:
        assert ToolTrajectoryJoy.finish_tool2_opening(instance)
        instance.begin.assert_called_once_with(2)
        assert instance.tool2_open_ready and not instance.tool2_opening
    else:
        with pytest.raises(ValueError, match='confirmed open fingers'):
            ToolTrajectoryJoy.finish_tool2_opening(instance)
        instance.begin.assert_not_called()


def test_closing_fingers_during_arm_execution_cancels_the_tool2_path():
    instance = node(0.)
    instance.stage, instance.path_id, instance.path = 'executing', 2, {}
    instance.fail = Mock()
    ToolTrajectoryJoy.tick(instance)
    instance.fail.assert_called_once()
    assert 'confirmed open fingers' in instance.fail.call_args.args[0]


def test_fingers_closed_after_validation_prevent_any_arm_command_publication():
    instance = node(0.)
    instance.stage, instance.path_id, instance.path = 'wait_mode', 2, {}
    instance.status = {'mode': 'tool_auto', 'transitioning': False, 'epoch': 'owner'}
    instance.status_time = instance.state_time = time.monotonic()
    instance.unchanged, instance.fail, instance.publisher = Mock(return_value=True), Mock(), Mock()
    ToolTrajectoryJoy.tick(instance)
    instance.fail.assert_called_once()
    instance.publisher.publish.assert_not_called()


def test_pick_place_does_not_advance_when_only_the_opening_has_completed():
    instance = SimpleNamespace(finish_tool2_opening=Mock(return_value=True),
                               pick_sequence=Mock())
    Tool2PickPlaceCoordinator.path_completed(instance)
    instance.pick_sequence.complete.assert_not_called()
