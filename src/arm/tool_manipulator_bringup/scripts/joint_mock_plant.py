"""Deterministic, ROS-independent joint plant for controller fault tests.

All public values use SI units: radians, seconds, rad/s and rad/s^2.  The
plant models actuator motion separately from encoder feedback; a command is
therefore never copied directly into a feedback sample.
"""
from collections import deque
from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class JointPlantConfig:
    velocity_limit_rad_s: float = 0.5
    acceleration_limit_rad_s2: float = 1.0
    time_constant_s: float = 0.12
    encoder_quantization_rad: float = 0.0
    stop_residual_rad: float = 0.0

    def __post_init__(self):
        values = (self.velocity_limit_rad_s, self.acceleration_limit_rad_s2,
                  self.time_constant_s)
        if not all(math.isfinite(value) and value > 0. for value in values):
            raise ValueError('plant motion limits must be finite and positive')
        if (not math.isfinite(self.encoder_quantization_rad)
                or self.encoder_quantization_rad < 0.
                or not math.isfinite(self.stop_residual_rad)
                or self.stop_residual_rad < 0.):
            raise ValueError('feedback resolution and residual must be finite and non-negative')


@dataclass
class JointFault:
    stalled: bool = False
    feedback_dropout: bool = False
    command_delay_s: float = 0.0
    overshoot_rad: float = 0.0


@dataclass(frozen=True)
class PlantFeedback:
    positions: tuple
    velocities: tuple

    @property
    def complete(self):
        return all(value is not None for value in self.positions+self.velocities)


class JointMockPlant:
    """Small deterministic plant with independently injectable joint faults."""

    def __init__(self, configs, positions=None, seed=0):
        self.configs = tuple(configs)
        if not self.configs:
            raise ValueError('plant requires at least one joint')
        count = len(self.configs)
        self.position = list(positions if positions is not None else [0.]*count)
        if len(self.position) != count or not all(map(math.isfinite, self.position)):
            raise ValueError('initial positions must match the joint count and be finite')
        self.velocity = [0.]*count
        self.faults = [JointFault() for _ in range(count)]
        self._commands = [deque([(0., self.position[index])]) for index in range(count)]
        generator = random.Random(seed)
        self._residual = [generator.uniform(-config.stop_residual_rad,
                                             config.stop_residual_rad)
                          for config in self.configs]
        self.time = 0.

    def inject(self, joint, *, stalled=None, feedback_dropout=None,
               command_delay_s=None, overshoot_rad=0.0):
        """Change one joint's fault state; overshoot is a one-shot displacement."""
        if not 0 <= joint < len(self.configs):
            raise IndexError('joint index out of range')
        fault = self.faults[joint]
        if stalled is not None:
            fault.stalled = bool(stalled)
        if feedback_dropout is not None:
            fault.feedback_dropout = bool(feedback_dropout)
        if command_delay_s is not None:
            if not math.isfinite(command_delay_s) or command_delay_s < 0.:
                raise ValueError('command delay must be finite and non-negative')
            fault.command_delay_s = float(command_delay_s)
        if not math.isfinite(overshoot_rad):
            raise ValueError('overshoot must be finite')
        self.position[joint] += float(overshoot_rad)

    def step(self, commands, dt):
        if (len(commands) != len(self.configs) or not all(map(math.isfinite, commands))
                or not math.isfinite(dt) or dt <= 0.):
            raise ValueError('step requires finite joint commands and positive dt')
        self.time += dt
        for index, (command, config, fault) in enumerate(zip(
                commands, self.configs, self.faults)):
            history = self._commands[index]
            if command != history[-1][1]:
                history.append((self.time, float(command)))
            cutoff = self.time-fault.command_delay_s
            while len(history) > 1 and history[1][0] <= cutoff:
                history.popleft()
            delayed_command = history[0][1]
            if fault.stalled:
                self.velocity[index] = 0.
                continue
            desired_velocity = max(-config.velocity_limit_rad_s,
                                   min(config.velocity_limit_rad_s,
                                       (delayed_command+self._residual[index]
                                        -self.position[index])/config.time_constant_s))
            maximum_change = config.acceleration_limit_rad_s2*dt
            self.velocity[index] += max(-maximum_change, min(
                maximum_change, desired_velocity-self.velocity[index]))
            self.position[index] += self.velocity[index]*dt
        return self.feedback()

    def feedback(self):
        positions, velocities = [], []
        for position, velocity, config, fault in zip(
                self.position, self.velocity, self.configs, self.faults):
            if fault.feedback_dropout:
                positions.append(None)
                velocities.append(None)
                continue
            quantum = config.encoder_quantization_rad
            positions.append(position if quantum == 0. else round(position/quantum)*quantum)
            velocities.append(velocity)
        return PlantFeedback(tuple(positions), tuple(velocities))
