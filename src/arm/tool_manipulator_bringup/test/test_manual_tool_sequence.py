"""Ownership transitions without ROS discovery, devices, or motor output."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from std_msgs.msg import Int32

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_tool_sequence import ManualToolSequence
from tool_docking_coordinator import ToolDockingCoordinator


def completed(success=True):
    future = Future()
    future.set_result(SimpleNamespace(success=success))
    return future


@pytest.fixture
def node():
    n = ManualToolSequence.__new__(ManualToolSequence)
    n.pending_selection = n.handoff_future = None
    n.return_to_manual = n.resume_pending = False
    n.stage = 'idle'
    n.paths = {0: {}, 1: {}, 2: {}}
    n.pre_dock_targets = {}
    n.status = dict(mode='manual', requested='manual', ready=True, startup_complete=True,
                    entry_alignment_complete=True, focus='arm', transitioning=False, epoch='a')
    n.status_time = n.state_time = 1.
    n.full_state = {}
    n.cancel = Mock()
    n.cancel.call_async.return_value = completed()
    n.manual_mode = Mock()
    n.manual_mode.call_async.return_value = completed()
    with patch('manual_tool_sequence.time.monotonic', return_value=1.), \
            patch.object(ToolDockingCoordinator, 'report'), \
            patch.object(ToolDockingCoordinator, 'tick'), \
            patch.object(ToolDockingCoordinator, 'fail',
                         side_effect=lambda reason: n.report('failed', reason)):
        yield n


@pytest.mark.parametrize('tool', [0, 1, 2])
def test_selection_stops_manual_then_dispatches_original_sequence_once(node, tool):
    message = Int32(data=tool)
    node.on_selected_tool(message)
    node.cancel.call_async.assert_called_once()
    assert node.return_to_manual
    with patch.object(ToolDockingCoordinator, 'on_selected_tool') as start:
        node.tick()  # Still manual: do not run a path before HOLD confirmation.
        start.assert_not_called()
        node.on_selected_tool(Int32(data=(tool+1) % 3))
        assert node.cancel.call_async.call_count == 1
        node.status.update(mode='hold', requested='hold', epoch='b')
        node.tick()
        start.assert_called_once_with(message)
        assert node.pending_selection is None


@pytest.mark.parametrize('changes', [dict(focus='drive'), dict(ready=False),
    dict(startup_complete=False), dict(entry_alignment_complete=False),
    dict(mode='fault'), dict(transitioning=True)])
def test_unready_selection_never_requests_ownership(node, changes):
    node.status.update(changes)
    node.on_selected_tool(Int32(data=0))
    node.cancel.call_async.assert_not_called()
    assert node.pending_selection is None


def test_invalid_and_stale_requests_are_not_queued(node):
    node.on_selected_tool(Int32(data=99))
    node.state_time = 0.
    node.on_selected_tool(Int32(data=0))
    node.cancel.call_async.assert_not_called()
    assert node.pending_selection is None


def test_completion_waits_for_confirmed_manual_mode(node):
    node.return_to_manual = True
    node.status.update(mode='hold', requested='hold')
    node.stage = 'executing'
    node.report('completed', 'full sequence finished')
    node.tick()
    node.manual_mode.call_async.assert_not_called()
    node.stage = 'idle'
    node.tick()
    node.manual_mode.call_async.assert_called_once()
    node.tick()
    assert node.resume_pending  # Service acceptance alone is not completion.
    node.status.update(mode='manual', requested='manual', epoch='new', transitioning=False)
    node.tick()
    assert not node.resume_pending and not node.return_to_manual


@pytest.mark.parametrize('state', ['failed', 'blocked'])
def test_failure_discards_resume_and_pending_selection(node, state):
    node.return_to_manual = node.resume_pending = True
    node.pending_selection = Int32(data=1)
    node.report(state, 'stopped')
    node.tick()
    node.manual_mode.call_async.assert_not_called()
    assert not node.return_to_manual and node.pending_selection is None


def test_external_cancel_epoch_prevents_automatic_resume(node):
    node.return_to_manual = True
    node.status.update(mode='hold', requested='hold')
    node.report('completed', 'finished')
    node.status['epoch'] = 'cancelled'
    node.tick()
    node.manual_mode.call_async.assert_not_called()
    assert not node.resume_pending


@pytest.mark.parametrize('during_resume', [False, True])
def test_explicit_cancel_during_handoff_never_resumes(node, during_resume):
    node.return_to_manual = True
    if during_resume:
        node.resume_pending = True
    else:
        node.pending_selection = Int32(data=1)
    response = node.on_cancel(None, SimpleNamespace())
    assert response.success
    assert not node.resume_pending and node.pending_selection is None
    node.manual_mode.call_async.assert_not_called()
    node.cancel.call_async.assert_called_once()


def test_hold_rejection_aborts_without_dispatch(node):
    node.cancel.call_async.return_value = completed(False)
    node.on_selected_tool(Int32(data=0))
    with patch.object(ToolDockingCoordinator, 'on_selected_tool') as start:
        node.tick()
        start.assert_not_called()
    assert not node.return_to_manual


def test_auto_producer_does_not_map_joystick_buttons(node):
    with patch.object(ToolDockingCoordinator, 'on_joy') as mapped:
        node.on_joy(Mock())
        mapped.assert_not_called()


def test_managed_standalone_loses_local_alignment_on_ownership_change():
    from standalone_managed_manual import StandaloneManagedManual
    from test_manual import harness
    n = StandaloneManagedManual.__new__(StandaloneManagedManual)
    n.__dict__.update(harness().__dict__)
    n.manager_status = dict(mode='tool_auto', ready=True)
    n.manager_time = 1.
    n.manager_epoch = 'old'
    n.docking = Mock(selected=True)
    n.path_points = [n.target.copy()]
    StandaloneManagedManual.poll_controller(n, 1.)
    n.docking.cancel.assert_called_once_with('manual ownership changed')
    assert n.path_points is None and not n.controller_ready and n.neutral_required


def test_standalone_buttons_stay_suspended_during_automation():
    from standalone_managed_manual import StandaloneManagedManual
    from manual_control_common import ManualController
    n = StandaloneManagedManual.__new__(StandaloneManagedManual)
    n.manager_status = dict(mode='tool_auto')
    n.controller_ready = False
    with patch.object(ManualController, 'on_joy') as focus:
        message = Mock()
        n.on_joy(message)
        focus.assert_called_once_with(n, message)
    assert n.path_combos == [False]*3 and n.previous_stop_pose
