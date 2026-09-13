"""
Unit tests for slope_decision.forward_slope_exceeds(), resolve_force_mode(),
pixel_span_width_m(), track_width_plausible().

Run:
    pytest test/test_slope_decision.py -v
or via colcon:
    colcon test --packages-select dolbotz
"""

import numpy as np
import pytest

from dolbotz.drive_area.slope_decision import (  # type: ignore[import]
    forward_slope_exceeds,
    FORWARD_SLOPE_MIN_PIXELS,
    ALWAYS_RELAY_FLAT_DEFAULT,
    resolve_force_mode,
    pixel_span_width_m,
    track_width_plausible,
    edge_sample_strip_bounds,
)

THRESHOLD = 10.0


def test_all_segments_default_to_flat_path():
    assert ALWAYS_RELAY_FLAT_DEFAULT is True


def flat_field(h: int = 20, w: int = 20, val: float = 0.0) -> np.ndarray:
    return np.full((h, w), val, dtype=np.float32)


def test_flat_field_never_exceeds():
    field = flat_field(val=2.0)  # 완만 — 임계각 훨씬 아래
    assert not forward_slope_exceeds(field, THRESHOLD)


def test_uniformly_steep_field_exceeds():
    field = flat_field(val=25.0)  # 전체 다 임계각 넘음
    assert forward_slope_exceeds(field, THRESHOLD)


def test_single_noisy_pixel_does_not_trigger():
    """단일(또는 min_pixels 미만) 픽셀만 튀는 건 노이즈로 보고 무시해야 한다."""
    field = flat_field(val=0.0)
    field[0, 0] = 90.0  # 픽셀 1개만 극단값
    assert not forward_slope_exceeds(field, THRESHOLD)


def test_enough_steep_pixels_triggers():
    field = flat_field(val=0.0)
    field.flat[:FORWARD_SLOPE_MIN_PIXELS] = 45.0  # 정확히 임계 픽셀 수만큼 채움
    assert forward_slope_exceeds(field, THRESHOLD)


def test_nan_pixels_ignored_not_counted_as_steep():
    field = flat_field(val=np.nan)
    assert not forward_slope_exceeds(field, THRESHOLD)


def test_custom_min_pixels_override():
    field = flat_field(val=0.0)
    field[0, :5] = 45.0  # 5픽셀만 steep
    assert not forward_slope_exceeds(field, THRESHOLD, min_pixels=10)
    assert forward_slope_exceeds(field, THRESHOLD, min_pixels=5)


# ---------------------------------------------------------------------------
# resolve_force_mode
# ---------------------------------------------------------------------------

def test_resolve_force_mode_valid_values_pass_through():
    assert resolve_force_mode('auto') == 'auto'
    assert resolve_force_mode('flat') == 'flat'
    assert resolve_force_mode('slope') == 'slope'


def test_resolve_force_mode_unknown_value_falls_back_to_auto():
    assert resolve_force_mode('bogus') == 'auto'
    assert resolve_force_mode('') == 'auto'
    assert resolve_force_mode('FLAT') == 'auto'  # 대소문자 구분함, 폴백


# ---------------------------------------------------------------------------
# pixel_span_width_m
# ---------------------------------------------------------------------------

def test_pixel_span_width_m_known_pinhole_case():
    # fx=500px, z=2m, 100px 폭 -> 100 * 2 / 500 = 0.4m
    assert pixel_span_width_m(0, 99, z_m=2.0, fx=500.0) == 200.0 / 500.0


def test_pixel_span_width_m_single_column_is_one_pixel_wide():
    assert pixel_span_width_m(10, 10, z_m=1.0, fx=100.0) == 1.0 / 100.0


def test_pixel_span_width_m_scales_with_depth():
    """같은 픽셀 폭이라도 더 멀리 있으면(z가 크면) 실측 폭도 비례해서 커진다."""
    near = pixel_span_width_m(0, 49, z_m=1.0, fx=500.0)
    far = pixel_span_width_m(0, 49, z_m=2.0, fx=500.0)
    assert far == pytest.approx(near * 2.0)


# ---------------------------------------------------------------------------
# track_width_plausible
# ---------------------------------------------------------------------------

def test_track_width_plausible_within_tolerance():
    assert track_width_plausible(1.0, expected_width_m=0.9144, tolerance_factor=1.5)


def test_track_width_plausible_beyond_tolerance():
    assert not track_width_plausible(5.0, expected_width_m=0.9144, tolerance_factor=1.5)


def test_track_width_plausible_narrower_than_expected_still_plausible():
    """더 좁은 건(가려짐 등) 정상일 수 있으니 하한은 안 본다."""
    assert track_width_plausible(0.1, expected_width_m=0.9144, tolerance_factor=1.5)


def test_track_width_plausible_exact_boundary():
    assert track_width_plausible(1.0, expected_width_m=1.0, tolerance_factor=1.0)


# ---------------------------------------------------------------------------
# edge_sample_strip_bounds
# ---------------------------------------------------------------------------

def test_edge_sample_strip_bounds_normal_case_no_clamp():
    """트랙이 충분히 넓으면 center_col ± half_span_px 지점을 그대로 겨냥하고,
    achieved baseline == 2*half_span_px (clamp 없음)."""
    result = edge_sample_strip_bounds(
        track_col_first=0, track_col_last=999, center_col=500,
        half_span_px=200.0, strip_half_width_px=10)
    assert result is not None
    left_x0, left_x1, right_x0, right_x1, achieved_baseline_px = result
    assert (left_x0, left_x1) == (290, 311)
    assert (right_x0, right_x1) == (690, 711)
    assert achieved_baseline_px == pytest.approx(400.0)


def test_edge_sample_strip_bounds_strip_width_matches_param():
    result = edge_sample_strip_bounds(
        track_col_first=0, track_col_last=999, center_col=500,
        half_span_px=200.0, strip_half_width_px=10)
    left_x0, left_x1, right_x0, right_x1, _ = result
    assert left_x1 - left_x0 == 2 * 10 + 1
    assert right_x1 - right_x0 == 2 * 10 + 1


def test_edge_sample_strip_bounds_clamped_when_target_outside_track():
    """트랙이 로봇 track_width_m보다 좁으면 목표 지점이 트랙 밖으로 나가고,
    achieved baseline이 요청한 2*half_span_px보다 작게 clamp된다 — 호출부는
    이 값으로 atan2 분모를 다시 계산해야 분자·분모가 일치한다."""
    result = edge_sample_strip_bounds(
        track_col_first=400, track_col_last=600, center_col=500,
        half_span_px=200.0, strip_half_width_px=5)
    assert result is not None
    left_x0, left_x1, right_x0, right_x1, achieved_baseline_px = result
    assert left_x0 == 400  # track 경계에 눌림
    assert right_x1 == 601  # track_col_last + 1
    assert achieved_baseline_px == pytest.approx(200.0)  # 400 < 요청한 400 아님, 실제론 600-400
    assert achieved_baseline_px < 400.0  # 요청한(2*half_span_px=400) baseline보다 작음


def test_edge_sample_strip_bounds_none_when_track_too_narrow():
    """트랙이 너무 좁아 두 스트립이 겹치면(강하게 clamp됨) None — 호출부는
    이 경우 절반-band 방식으로 폴백해야 한다."""
    result = edge_sample_strip_bounds(
        track_col_first=490, track_col_last=510, center_col=500,
        half_span_px=200.0, strip_half_width_px=15)
    assert result is None


def test_edge_sample_strip_bounds_none_when_half_span_too_small_for_strip_width():
    """half_span_px이 strip_half_width_px보다 작으면(스트립끼리 겹침) None."""
    result = edge_sample_strip_bounds(
        track_col_first=0, track_col_last=999, center_col=500,
        half_span_px=0.0, strip_half_width_px=5)
    assert result is None


def test_edge_sample_strip_bounds_bounds_stay_within_track():
    """clamp된 경우에도 반환된 열 범위가 [track_col_first, track_col_last]
    안에 있어야 한다."""
    result = edge_sample_strip_bounds(
        track_col_first=400, track_col_last=600, center_col=500,
        half_span_px=200.0, strip_half_width_px=5)
    left_x0, left_x1, right_x0, right_x1, _ = result
    assert left_x0 >= 400
    assert right_x1 - 1 <= 600
