from rclpy.duration import Duration
from rclpy.time import Time
import pytest
import tf2_ros

from dolbotz.missions.summer_supply import (
    is_within_base_x_stop_distance,
    lookup_transform_with_latest_fallback,
)


class FakeBuffer:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = []

    def lookup_transform(self, target, source, stamp, *, timeout):
        self.calls.append((target, source, stamp, timeout))
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def test_exact_transform_is_preferred():
    exact = object()
    buffer = FakeBuffer([exact])

    result, used_latest, error = lookup_transform_with_latest_fallback(
        buffer, 'base_actuator', 'camera_color_optical_frame',
        Time(seconds=123), Duration(seconds=0.5))

    assert result is exact
    assert not used_latest
    assert error is None
    assert len(buffer.calls) == 1


def test_future_extrapolation_retries_with_latest_transform():
    latest = object()
    future_error = tf2_ros.ExtrapolationException(
        'Lookup would require extrapolation into the future')
    buffer = FakeBuffer([future_error, latest])

    result, used_latest, error = lookup_transform_with_latest_fallback(
        buffer, 'base_actuator', 'camera_color_optical_frame',
        Time(seconds=123), Duration(seconds=0.5))

    assert result is latest
    assert used_latest
    assert error is future_error
    assert len(buffer.calls) == 2
    assert buffer.calls[1][2].nanoseconds == 0


def test_past_extrapolation_remains_a_failure():
    buffer = FakeBuffer([
        tf2_ros.ExtrapolationException(
            'Lookup would require extrapolation into the past')
    ])

    with pytest.raises(tf2_ros.ExtrapolationException):
        lookup_transform_with_latest_fallback(
            buffer, 'base_actuator', 'camera_color_optical_frame',
            Time(seconds=123), Duration(seconds=0.5))

    assert len(buffer.calls) == 1


@pytest.mark.parametrize('base_x', [-0.030, 0.0, 0.030])
def test_base_x_stop_distance_includes_signed_boundary(base_x):
    assert is_within_base_x_stop_distance(base_x, 0.030)


@pytest.mark.parametrize('base_x', [-0.031, 0.031, float('nan')])
def test_base_x_stop_distance_rejects_outside_or_invalid(base_x):
    assert not is_within_base_x_stop_distance(base_x, 0.030)


@pytest.mark.parametrize('base_x', [-0.32, 0.31, 0.32])
def test_stationary_pickup_accepts_existing_range(base_x):
    assert is_within_base_x_stop_distance(base_x, 0.32)


@pytest.mark.parametrize('base_x', [-0.3201, 0.3201, float('inf')])
def test_stationary_pickup_withholds_outside_range(base_x):
    assert not is_within_base_x_stop_distance(base_x, 0.32)
