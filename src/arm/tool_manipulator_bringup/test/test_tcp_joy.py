"""Offline Cartesian increment and command gating regressions."""
from pathlib import Path
import sys
from unittest.mock import Mock, patch
import numpy as np
import pytest

ARM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ARM/'tool_change_min'))
sys.path.insert(0, str(ARM/'tool_manipulator_bringup/scripts'))
from tcp_joy_core import TcpStepper, joy_twist, LimitBlocked, TcpMetrics
from tcp_joy import TcpJoy
from tool_change_min.kinematics import ArmKinematics
from manual_control_common import ArmChain
import xacro
import yaml


@pytest.fixture
def model():
    return ArmKinematics(ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                         ARM/'tool_manipulator_bringup/config/hardware.yaml')


def test_deadman_modifier_and_vector_speed_bound():
    buttons = [0]*13
    assert not joy_twist([1.]*8, buttons)[0]
    buttons[4] = 1
    held, twist = joy_twist([1.]*8, buttons)
    assert held and np.linalg.norm(twist[:3]) == pytest.approx(1.)
    np.testing.assert_array_equal(twist[3:], 0.)
    buttons[5] = 1
    _, twist = joy_twist([1.]*8, buttons)
    np.testing.assert_array_equal(twist[:3], 0.)
    assert np.linalg.norm(twist[3:]) == pytest.approx(1.)
    assert not joy_twist([float('nan')]*8, buttons)[0]


def test_actual_urdf_fk_matches_manual_chain(model):
    xml = xacro.process_file(str(ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro')).toxml()
    manual = ArmChain(xml)
    for q in (np.zeros(6), np.array([.1, -.4, .7, -.2, .3, -.2])):
        fk, jac = manual.pose_and_jacobian(dict(zip(model.joint_names, q)))
        np.testing.assert_allclose(model.fk(q), fk, atol=1e-12)
        np.testing.assert_allclose(model.jacobian(q)[:3, 1:4], jac, atol=1e-7)


def test_geometric_jacobian_matches_independent_central_differences(model):
    from tool_change_min.kinematics import _rotvec
    rng = np.random.default_rng(42)
    for _ in range(30):
        q = np.array([rng.uniform(lo+.03, hi-.03)
                      for lo, hi in model.limits.values()])
        columns = []
        for i in range(6):
            delta = np.zeros(6)
            delta[i] = 1e-5
            plus, minus = model.fk(q+delta), model.fk(q-delta)
            columns.append(np.r_[(plus[:3, 3]-minus[:3, 3])/2e-5,
                                 _rotvec(plus[:3, :3] @ minus[:3, :3].T)/2e-5])
        np.testing.assert_allclose(model.jacobian(q), np.column_stack(columns), atol=1e-7)


def test_completed_solver_result_after_deadline_is_not_accepted():
    s = fake_stepper()
    with patch('time.monotonic', side_effect=[10., 10.001, 10.1]):
        with pytest.raises(ValueError, match='time budget'):
            s.candidate(np.zeros(6), np.array([1., 0, 0, 0, 0, 0]), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))


def fake_stepper():
    model = Mock()
    model.within_limits.return_value = True
    model.fk.return_value = np.eye(4)
    model.jacobian.return_value = np.diag([.2]*3+[1.]*3)
    model.solve.return_value = np.ones(6)*.001
    model.pose_error.return_value = np.zeros(6)
    step = TcpStepper(model, [.12]*6)
    step.reset(np.zeros(6))
    return step


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1., 1.])
def test_increment_frames_at_nonidentity_target(axis, sign):
    step = fake_stepper()
    # Local X -> base Y, local Y -> base Z, local Z -> base X.
    step.pose[:3, :3] = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    step.pose[:3, 3] = [.2, -.1, .3]
    before = step.pose.copy()
    twist = np.zeros(6)
    twist[axis] = sign
    with patch('time.monotonic', return_value=10.):
        _, target = step.candidate(np.zeros(6), twist, .02)
    if axis < 3:
        expected = before[:3, 3].copy()
        expected[axis] += sign*step.linear*.02*step.scale*step.velocity_scale
        np.testing.assert_allclose(target[:3, 3], expected)
        np.testing.assert_array_equal(target[:3, :3], before[:3, :3])
    else:
        # Independently construct the elementary local-axis rotation.
        angle = sign*step.angular*.02*step.scale*step.velocity_scale
        local = np.eye(3)
        i, j = ((1, 2), (2, 0), (0, 1))[axis-3]
        local[i, i] = local[j, j] = np.cos(angle)
        local[i, j], local[j, i] = -np.sin(angle), np.sin(angle)
        np.testing.assert_allclose(before[:3, :3].T @ target[:3, :3], local, atol=1e-14)
        np.testing.assert_array_equal(target[:3, 3], before[:3, 3])
    np.testing.assert_array_equal(step.pose, before)


def test_local_rotation_speed_prediction_uses_target_frame():
    step = fake_stepper()
    step.pose[:3, :3] = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    # Local X rotation demands the slower base-Y joint, not base X.
    step.speeds[4] = .03
    step.model.solve.return_value = np.zeros(6)
    with patch('time.monotonic', return_value=10.):
        step.candidate(np.zeros(6), np.array([0., 0., 0., 1., 0., 0.]), .02)
    assert step.velocity_scale == pytest.approx(.9*.03/step.angular)


def test_combined_translation_and_local_rotation_preserve_frames():
    step = fake_stepper()
    step.pose[:3, :3] = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    before = step.pose.copy()
    twist = np.array([.4, -.3, .2, .2, -.4, .3])
    with patch('time.monotonic', return_value=10.):
        _, target = step.candidate(np.zeros(6), twist, .02)
    from tool_change_min.kinematics import _rotvec
    factor = .02*step.scale*step.velocity_scale
    np.testing.assert_allclose(target[:3, 3]-before[:3, 3], twist[:3]*step.linear*factor)
    np.testing.assert_allclose(_rotvec(before[:3, :3].T @ target[:3, :3]),
                               twist[3:]*step.angular*factor, atol=1e-12)


def test_limit_block_preserves_target_and_reverse_can_succeed():
    s = fake_stepper()
    s.model.solve.side_effect = ValueError('IK iteration would violate soft limits')
    for _ in range(6):
        with pytest.raises(LimitBlocked):
            s.candidate(np.zeros(6), np.array([1., 0, 0, 0, 0, 0]), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))
    np.testing.assert_array_equal(s.pose, np.eye(4))
    s.model.solve.side_effect = None
    q, pose = s.candidate(np.zeros(6), np.array([-1., 0, 0, 0, 0, 0]), .02)
    assert pose[0, 3] < 0


def test_mixed_ik_errors_are_not_classified_as_limit_block():
    s = fake_stepper()
    s.model.solve.side_effect = [ValueError('IK did not converge')] + [
        ValueError('IK iteration would violate soft limits')]*3
    with pytest.raises(ValueError) as exc:
        s.candidate(np.zeros(6), np.array([1., 0, 0, 0, 0, 0]), .02)
    assert not isinstance(exc.value, LimitBlocked)


def test_real_model_can_reverse_from_blocked_tcp_direction(model):
    measured = np.array([-.00767, 1.4830934, 1.5070934, -1.5570934,
                         .1549321, .0567573])
    s = TcpStepper(model, [.12]*6)
    s.reset(measured)
    twist = np.array([1., 0, 0, 0, 0, 0])
    with patch('time.monotonic', return_value=10.):
        for _ in range(100):
            try:
                q, pose = s.candidate(measured, twist, .02)
            except LimitBlocked:
                break
            s.commit(q, pose)
            measured = q.copy()
        else:
            pytest.fail('expected outward direction to reach joint margin')
        before = s.pose.copy()
        q, pose = s.candidate(measured, -twist, .02)
    assert pose[0, 3] < before[0, 3]
    assert model.within_limits(q, s.margin)
    assert np.max(np.abs(q-s.q)) <= .12*.02+1e-9


def test_increment_only_commits_after_collision_acceptance():
    s = fake_stepper()
    q, target = s.candidate(np.zeros(6), np.array([1.,0,0,0,0,0]), .02)
    assert target[0,3] == pytest.approx(.0004)
    assert s.pose[0,3] == 0.
    s.commit(q,target)
    _, target2 = s.candidate(q, np.array([1.,0,0,0,0,0]), .02)
    assert target2[0,3] == pytest.approx(.0008)
    # The accumulated target pose and its matching previous command solution
    # form the next IK seed; live feedback is enforced separately by lead checks.
    np.testing.assert_array_equal(s.model.solve.call_args.args[1], s.q)


@pytest.mark.parametrize('kind', ['speed', 'lead', 'singularity', 'ik'])
def test_failed_candidate_preserves_last_target(kind):
    s = fake_stepper()
    measured = np.zeros(6)
    if kind == 'speed':
        s.model.solve.return_value = np.ones(6)*.1
    if kind == 'lead':
        measured[:] = -.09
    if kind == 'singularity':
        s.model.jacobian.return_value = np.zeros((6,6))
    if kind == 'ik':
        s.model.solve.side_effect = ValueError('no solution')
    with pytest.raises(ValueError):
        s.candidate(measured, np.ones(6), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))
    np.testing.assert_array_equal(s.pose, np.eye(4))


def test_ik_time_and_iteration_budgets(model):
    q = np.zeros(6)
    target = model.fk(q)
    target[0,3] += .01
    with pytest.raises(ValueError, match='time budget'):
        model.solve(target, q, max_time_s=1e-12)
    with pytest.raises(ValueError, match='converge'):
        model.solve(target, q, max_iterations=0)


def test_repeated_increments_with_lagging_feedback(model):
    measured = np.array([-.00767, 1.4830934, 1.5070934, -1.5570934,
                         .0567573, .0582913])
    step = TcpStepper(model, [.12]*6)
    step.reset(measured)
    # Deterministic convergence test; deadline exhaustion is tested separately.
    with patch('time.monotonic', return_value=10.):
        for _ in range(100):
            before = step.q.copy()
            q, pose = step.candidate(measured, np.array([0., 1, 0, 0, 0, 0]), .02)
            assert np.max(np.abs(q-before)) <= .12*.02+1e-9
            assert np.max(np.abs(q-measured)) <= step.lead
            assert model.within_limits(q, step.margin)
            error = model.pose_error(q, pose)
            assert np.linalg.norm(error[:3]) < 1e-5
            assert np.linalg.norm(error[3:]) < 1e-4
            step.commit(q, pose)
            measured += .3*(q-measured)


def test_deadman_release_discards_pending_checks():
    node = Mock(deadman=False, deadman_engaged=False, rearmed=False, blocked=False)
    node.on_joy = TcpJoy.on_joy.__get__(node)
    node.on_joy(Mock(axes=[0.]*8, buttons=[0]*13))
    node.discard.assert_called_once()
    assert not node.deadman and node.rearmed
    np.testing.assert_array_equal(node.twist, np.zeros(6))


def test_repeated_failure_requests_hold_and_stops_heartbeat():
    node = Mock(failures=0, blocked=False, last_failure_reason='',
                last_failure_time=-float('inf'))
    node.metrics = TcpMetrics()
    for _ in range(5):
        TcpJoy.fail(node, 'collision service failed')
    assert node.blocked
    assert node.last_failure_reason == 'collision service failed'
    assert np.isfinite(node.last_failure_time)
    node.cancel.call_async.assert_called_once()


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1., 1.])
def test_real_feedback_all_tcp_directions_obey_joint_speed(model, axis, sign):
    measured = np.array([-.0046019424, 1.4837043012, 1.5070917825,
                         -1.5568336395, .0582912699, .0582912699])
    step = TcpStepper(model, [.12]*6)
    step.reset(measured)
    twist = np.zeros(6)
    twist[axis] = sign
    # Freeze wall clock here to test numerical convergence deterministically;
    # the separate deadline regression checks time exhaustion.
    with patch('time.monotonic', return_value=10.):
        q, pose = step.candidate(measured, twist, .02)
    assert np.max(np.abs(q-measured)) > 1e-6
    assert np.all(np.abs(q-measured) <= .12*.02+1e-9)
    assert model.within_limits(q, .02)
    error = model.pose_error(q, pose)
    assert np.linalg.norm(error[:3]) < 1e-5
    assert np.linalg.norm(error[3:]) < 1e-4
    np.testing.assert_array_equal(step.q, measured)


def test_wait_for_collision_server_keeps_hold_heartbeat_without_failures():
    node = Mock()
    node.metrics = TcpMetrics()
    node.status = dict(mode='tcp_ik', focus='arm', ready=True,
                       transitioning=False, epoch='test')
    node.status_time = node.state_time = node.last_time = node.joy_time = 10.
    node.epoch = 'test'
    node.measured = np.zeros(6)
    node.stepper = fake_stepper()
    node.blocked = False
    node.failures = 0
    node.last_failure_reason = ''
    node.last_failure_time = -float('inf')
    node.last_ik_duration_ms = None
    node.pending = None
    node.deadman = node.rearmed = True
    node.check.service_is_ready.return_value = False
    with patch('time.monotonic', return_value=10.02):
        TcpJoy.tick(node)
    assert not node.rearmed
    node.fail.assert_not_called()
    node.publish.assert_called_once()
    assert 'waiting for MoveIt' in node.reason


def test_tcp_mock_initial_pose_is_inside_margin_and_nonsingular(model):
    values = yaml.safe_load((ARM/'tool_manipulator_bringup/config/tcp_mock_initial_positions.yaml').read_text())
    q = np.array([values['initial_positions'][j] for j in model.joint_names])
    assert model.within_limits(q, .02)
    jac = model.jacobian(q)
    jac[:3] /= .2
    assert np.linalg.cond(jac) < 20.


def test_candidate_total_deadline_preserves_previous_goal():
    step = fake_stepper()
    with patch('time.monotonic', side_effect=[10., 10.04]):
        with pytest.raises(ValueError, match='time budget'):
            step.candidate(np.zeros(6), np.array([1., 0, 0, 0, 0, 0]), .02)
    step.model.solve.assert_not_called()
    np.testing.assert_array_equal(step.q, np.zeros(6))


def test_gripper_zero_and_internal_allowed_collisions_stay_consistent():
    hardware = yaml.safe_load((ARM/'tool_manipulator_bringup/config/hardware.yaml').read_text())
    assert hardware['joints']['ee_joint']['zero_raw'] == 1992
    text = (ARM/'tool_manipulator_moveit_config/config/tool_manipulator.srdf').read_text()
    expected = {
        ('rack_left', 'ee_actuator'), ('tool_spurgear', 'ee_actuator'),
        ('rack_right', 'wrist_link_yaw'), ('rack_left', 'tool_spurgear'),
        ('rack_right', 'tool_spurgear'), ('rack_left', 'wrist_link_yaw'),
    }
    import xml.etree.ElementTree as ET
    root = ET.fromstring(text)
    allowed = {tuple(sorted((x.get('link1'), x.get('link2'))))
               for x in root.findall('disable_collisions')}
    assert {tuple(sorted(pair)) for pair in expected} <= allowed
