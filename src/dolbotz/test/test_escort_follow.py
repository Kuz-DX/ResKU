"""
Unit tests for escort_follow's pure functions: predict_position(),
within_reacquire_radius(), update_velocity_ema(), stamp_to_sec().

Run:
    pytest test/test_escort_follow.py -v
or via colcon:
    colcon test --packages-select dolbotz
"""

import math
from types import SimpleNamespace

import pytest

from dolbotz.missions.escort_follow import (  # type: ignore[import]
    predict_position,
    within_reacquire_radius,
    update_velocity_ema,
    stamp_to_sec,
)


# ---------------------------------------------------------------------------
# predict_position
# ---------------------------------------------------------------------------

class TestPredictPosition:
    def test_moves_along_constant_velocity(self):
        last_point = (1.0, 2.0, 3.0)
        velocity = (1.0, 0.0, -1.0)
        result = predict_position(last_point, velocity, elapsed_sec=2.0)
        assert result == pytest.approx((3.0, 2.0, 1.0))

    def test_zero_velocity_stays_put(self):
        last_point = (0.5, -1.0, 4.0)
        result = predict_position(last_point, (0.0, 0.0, 0.0), elapsed_sec=5.0)
        assert result == pytest.approx(last_point)

    def test_zero_elapsed_stays_put(self):
        last_point = (0.5, -1.0, 4.0)
        velocity = (2.0, -3.0, 1.0)
        result = predict_position(last_point, velocity, elapsed_sec=0.0)
        assert result == pytest.approx(last_point)


# ---------------------------------------------------------------------------
# within_reacquire_radius
# ---------------------------------------------------------------------------

class TestWithinReacquireRadius:
    def test_exact_match_is_within(self):
        p = (1.0, 1.0, 1.0)
        assert within_reacquire_radius(
            p, p, elapsed_sec=0.0, base_radius_m=0.5, growth_m_per_sec=0.3)

    def test_candidate_outside_base_radius_at_zero_elapsed(self):
        predicted = (0.0, 0.0, 0.0)
        candidate = (1.0, 0.0, 0.0)  # 1.0m 떨어짐
        assert not within_reacquire_radius(
            predicted, candidate, elapsed_sec=0.0,
            base_radius_m=0.5, growth_m_per_sec=0.3)

    def test_radius_grows_with_elapsed_time(self):
        predicted = (0.0, 0.0, 0.0)
        candidate = (1.0, 0.0, 0.0)  # 1.0m 떨어짐
        # elapsed=0일 땐 반경 0.5m라 실패해야 함
        assert not within_reacquire_radius(
            predicted, candidate, elapsed_sec=0.0,
            base_radius_m=0.5, growth_m_per_sec=0.3)
        # elapsed=2.0이면 반경 0.5+0.3*2.0=1.1m라 통과해야 함
        assert within_reacquire_radius(
            predicted, candidate, elapsed_sec=2.0,
            base_radius_m=0.5, growth_m_per_sec=0.3)

    def test_boundary_is_inclusive(self):
        predicted = (0.0, 0.0, 0.0)
        candidate = (0.5, 0.0, 0.0)  # 정확히 base_radius_m만큼 떨어짐
        assert within_reacquire_radius(
            predicted, candidate, elapsed_sec=0.0,
            base_radius_m=0.5, growth_m_per_sec=0.0)

    def test_3d_distance_used_not_just_xy(self):
        predicted = (0.0, 0.0, 0.0)
        candidate = (0.0, 0.0, 3.0)  # z축으로만 3m
        assert not within_reacquire_radius(
            predicted, candidate, elapsed_sec=0.0,
            base_radius_m=0.5, growth_m_per_sec=0.0)
        assert math.dist(predicted, candidate) == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# update_velocity_ema
# ---------------------------------------------------------------------------

class TestUpdateVelocityEma:
    def test_bootstrap_with_no_prev_velocity_returns_raw(self):
        prev_point = (0.0, 0.0, 0.0)
        new_point = (2.0, 0.0, 0.0)
        result = update_velocity_ema(
            prev_velocity=None, prev_point=prev_point, new_point=new_point,
            dt=1.0, alpha=0.5)
        assert result == pytest.approx((2.0, 0.0, 0.0))

    def test_alpha_one_ignores_previous_velocity(self):
        result = update_velocity_ema(
            prev_velocity=(10.0, 10.0, 10.0),
            prev_point=(0.0, 0.0, 0.0), new_point=(1.0, 0.0, 0.0),
            dt=1.0, alpha=1.0)
        assert result == pytest.approx((1.0, 0.0, 0.0))

    def test_alpha_zero_keeps_previous_velocity_unchanged(self):
        prev_velocity = (3.0, -1.0, 0.5)
        result = update_velocity_ema(
            prev_velocity=prev_velocity,
            prev_point=(0.0, 0.0, 0.0), new_point=(100.0, 100.0, 100.0),
            dt=1.0, alpha=0.0)
        assert result == pytest.approx(prev_velocity)

    def test_blend_matches_known_formula(self):
        # raw = (2-0)/1.0 = 2.0 (x축만), prev=1.0, alpha=0.5
        # -> 0.5*2.0 + 0.5*1.0 = 1.5
        result = update_velocity_ema(
            prev_velocity=(1.0, 0.0, 0.0),
            prev_point=(0.0, 0.0, 0.0), new_point=(2.0, 0.0, 0.0),
            dt=1.0, alpha=0.5)
        assert result == pytest.approx((1.5, 0.0, 0.0))

    def test_nonzero_dt_scales_raw_velocity(self):
        # 2m 이동하는데 0.5초 걸림 -> raw velocity = 4 m/s
        result = update_velocity_ema(
            prev_velocity=None,
            prev_point=(0.0, 0.0, 0.0), new_point=(2.0, 0.0, 0.0),
            dt=0.5, alpha=1.0)
        assert result == pytest.approx((4.0, 0.0, 0.0))

    def test_non_positive_dt_returns_prev_velocity_unchanged(self):
        prev_velocity = (1.0, 2.0, 3.0)
        result = update_velocity_ema(
            prev_velocity=prev_velocity,
            prev_point=(0.0, 0.0, 0.0), new_point=(5.0, 5.0, 5.0),
            dt=0.0, alpha=0.5)
        assert result == prev_velocity

        result_negative = update_velocity_ema(
            prev_velocity=prev_velocity,
            prev_point=(0.0, 0.0, 0.0), new_point=(5.0, 5.0, 5.0),
            dt=-0.1, alpha=0.5)
        assert result_negative == prev_velocity

    def test_non_positive_dt_with_no_prev_velocity_returns_zero(self):
        result = update_velocity_ema(
            prev_velocity=None,
            prev_point=(0.0, 0.0, 0.0), new_point=(5.0, 5.0, 5.0),
            dt=0.0, alpha=0.5)
        assert result == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# stamp_to_sec
# ---------------------------------------------------------------------------

def test_stamp_to_sec_combines_sec_and_nanosec():
    stamp = SimpleNamespace(sec=10, nanosec=500_000_000)
    assert stamp_to_sec(stamp) == pytest.approx(10.5)
