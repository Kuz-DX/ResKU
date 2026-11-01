"""Deterministic joint-plant behavior and ModeCore fault classification."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore
from joint_mock_plant import JointMockPlant, JointPlantConfig


DT = .02
CONFIG = JointPlantConfig(velocity_limit_rad_s=.4,
                          acceleration_limit_rad_s2=.8,
                          time_constant_s=.1,
                          encoder_quantization_rad=.001,
                          stop_residual_rad=.0005)


def ready_tool_core():
    core = ModeCore([(-2., 2.)]*2, [.12]*2, joint_names=['a', 'b'])
    core.focus, core.ready = 'arm', True
    core.feedback([0., 0.], [0., 0.], 0.)
    core.request('tool_auto', 0.)
    for step in range(1, 6):
        now = step*DT
        core.feedback([0., 0.], [0., 0.], now)
        core.tick(now, DT)
    assert core.mode == 'tool_auto'
    return core


def run_fixture(kind, seed=17):
    core = ready_tool_core()
    plant = JointMockPlant([CONFIG, CONFIG], seed=seed)
    assert core.path(core.epoch, kind, [[.5, 0.]], .1)
    if kind == 'stall':
        plant.inject(0, stalled=True)
    elif kind == 'timeout':
        plant.inject(0, stalled=True)
        core.path_stall_timeout = 200.
    elif kind == 'dropout':
        plant.inject(0, feedback_dropout=True)
    elif kind == 'following':
        plant.inject(0, overshoot_rad=.2)
    for step in range(1, 6200):
        now = .1+step*DT
        feedback = plant.feedback()
        if feedback.complete:
            core.feedback(feedback.positions, feedback.velocities, now)
        command = core.tick(now, DT)
        if command is not None:
            plant.step(command, DT)
        if core.last_execution_result == 'aborted' or core.mode == 'fault':
            return core.reason
    raise AssertionError('fixture did not terminate')


def test_dynamics_are_bounded_quantized_and_not_command_copies():
    plant = JointMockPlant([CONFIG], seed=3)
    feedback = plant.step([1.], DT)
    assert feedback.positions[0] != 1.
    assert abs(feedback.velocities[0]) <= CONFIG.acceleration_limit_rad_s2*DT
    previous_velocity = feedback.velocities[0]
    for _ in range(100):
        feedback = plant.step([1.], DT)
        assert abs(feedback.velocities[0]) <= CONFIG.velocity_limit_rad_s+1e-12
        assert abs(feedback.velocities[0]-previous_velocity) <= (
            CONFIG.acceleration_limit_rad_s2*DT+1e-12)
        assert feedback.positions[0]/CONFIG.encoder_quantization_rad == pytest.approx(
            round(feedback.positions[0]/CONFIG.encoder_quantization_rad))
        previous_velocity = feedback.velocities[0]


def test_seeded_stop_residual_is_repeatable_and_bounded():
    traces = []
    for _ in range(2):
        plant = JointMockPlant([CONFIG], seed=91)
        traces.append([plant.step([0.], DT).positions[0] for _ in range(200)])
    assert traces[0] == traces[1]
    assert abs(traces[0][-1]) <= CONFIG.stop_residual_rad+CONFIG.encoder_quantization_rad/2


def test_per_joint_delay_stall_dropout_and_overshoot_injection():
    plant = JointMockPlant([CONFIG, CONFIG], seed=5)
    plant.inject(0, command_delay_s=.2)
    for _ in range(9):
        plant.step([.3, .3], DT)
    # Joint 0 has only its seeded sub-milliradian stop residual response while
    # joint 1 has already accepted the position step.
    assert abs(plant.position[0]) < .001 and plant.position[1] > .005
    plant.inject(1, stalled=True)
    stopped_at = plant.position[1]
    plant.step([.3, .3], DT)
    assert plant.position[1] == stopped_at
    plant.inject(0, feedback_dropout=True)
    sampled = plant.feedback().positions
    assert sampled[0] is None
    assert abs(sampled[1]-plant.position[1]) <= CONFIG.encoder_quantization_rad/2
    plant.inject(1, overshoot_rad=.25)
    assert plant.position[1] == pytest.approx(stopped_at+.25)


@pytest.mark.parametrize('kind,expected', [
    ('following', 'following error: a'),
    ('stall', 'waypoint stalled: a'),
    ('timeout', 'waypoint timeout'),
    ('dropout', 'joint feedback timeout'),
])
def test_manager_reports_distinct_first_failure(kind, expected):
    first = run_fixture(kind)
    assert expected in first
    assert run_fixture(kind) == first


def test_normal_reversal_lag_is_not_a_direction_fault():
    plant = JointMockPlant([JointPlantConfig(.5, 1., .15)], seed=2)
    for _ in range(20):
        plant.step([.3], DT)
    assert plant.velocity[0] > 0.
    feedback = plant.step([-.3], DT)
    # Acceleration-limited hardware legitimately continues in the old direction
    # immediately after command reversal; this observation alone is not a fault.
    assert feedback.velocities[0] > 0.

    core = ModeCore([(-2., 2.)], [.5], joint_names=['reversing_joint'])
    core.focus, core.ready = 'arm', True
    core.feedback(feedback.positions, feedback.velocities, plant.time)
    core.target = list(feedback.positions)
    core.goal = [-.3]
    core.mode = 'hold'
    for _ in range(5):
        feedback = plant.step([-.3], DT)
        core.feedback(feedback.positions, feedback.velocities, plant.time)
        core.tick(plant.time, DT)
    assert core.mode != 'fault'
