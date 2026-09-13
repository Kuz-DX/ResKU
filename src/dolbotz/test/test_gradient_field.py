"""
Unit + benchmark tests for compute_gradient_field().

Run:
    pytest test/test_gradient_field.py -v
or via colcon:
    colcon test --packages-select dolbotz

All synthetic elevation maps are constructed so the expected analytical
gradient is known — no numerical mystery.
"""

import time

import numpy as np
import pytest
import rclpy
from std_msgs.msg import Header

from dolbotz.drive_area.gradient_map import (  # type: ignore[import]
    compute_gradient_field,
    plan_path_on_slope_field,
    apply_lateral_clearance,
    smooth_elevation,
    centerline_row_targets,
)

RES = 0.15  # metres — default resolution used throughout


# ---------------------------------------------------------------------------
# Synthetic map builders
# ---------------------------------------------------------------------------

def flat_map(h: int = 40, w: int = 40, val: float = 0.0) -> np.ndarray:
    return np.full((h, w), val, dtype=np.float32)


def ramp_x(h: int = 40, w: int = 40, slope: float = 0.2, res: float = RES) -> np.ndarray:
    """Slope in +x direction: elevation[r, c] = slope * c * res  →  gx = slope, gy = 0."""
    cols = (np.arange(w, dtype=np.float32) * res * slope)
    return np.tile(cols, (h, 1))


def ramp_y(h: int = 40, w: int = 40, slope: float = 0.2, res: float = RES) -> np.ndarray:
    """Slope in +y direction.

    Convention: row↑ ≡ y↓, so elevation[r, c] = -slope * r * res  →  gy = slope.
    Derivation:
      grad_row[r] ≈ d(elev)/d(row_coord) = -slope * res / res = -slope
      gy = -grad_row = slope  ✓
    """
    rows = (-slope * np.arange(h, dtype=np.float32) * res)
    return np.tile(rows.reshape(-1, 1), (1, w))


def diagonal_ramp(slope_x: float = 0.1, slope_y: float = 0.1,
                  h: int = 40, w: int = 40, res: float = RES) -> np.ndarray:
    """Combined ramp: elevation[r, c] = slope_x*c*res - slope_y*r*res."""
    cols = slope_x * np.arange(w, dtype=np.float32) * res
    rows = -slope_y * np.arange(h, dtype=np.float32) * res
    return rows.reshape(-1, 1) + cols.reshape(1, -1)


# ---------------------------------------------------------------------------
# Correctness tests
# ---------------------------------------------------------------------------

class TestSmoothElevation:
    def test_sigma_zero_returns_unchanged_copy(self):
        elev = flat_map(val=0.5)
        out = smooth_elevation(elev, sigma_m=0.0, resolution=RES)
        assert np.array_equal(out, elev)
        out[0, 0] = 999.0
        assert elev[0, 0] == 0.5  # 원본 안 건드림(복사본)

    def test_flat_map_stays_flat(self):
        elev = flat_map(val=0.5)
        out = smooth_elevation(elev, sigma_m=0.3, resolution=RES)
        np.testing.assert_allclose(out, 0.5)

    def test_isolated_spike_is_reduced(self):
        elev = flat_map(h=11, w=11, val=0.0)
        elev[5, 5] = 1.0
        out = smooth_elevation(elev, sigma_m=0.3, resolution=RES)
        assert 0.0 < out[5, 5] < 1.0

    def test_larger_sigma_smooths_more(self):
        elev = flat_map(h=11, w=11, val=0.0)
        elev[5, 5] = 1.0
        small = smooth_elevation(elev, sigma_m=RES, resolution=RES)
        big = smooth_elevation(elev, sigma_m=4 * RES, resolution=RES)
        assert big[5, 5] < small[5, 5]

    def test_all_nan_stays_nan(self):
        elev = np.full((5, 5), np.nan, dtype=np.float32)
        out = smooth_elevation(elev, sigma_m=0.3, resolution=RES)
        assert np.all(np.isnan(out))

    def test_nan_does_not_leak_into_distant_valid_cells(self):
        elev = flat_map(h=21, w=21, val=0.2)
        elev[10, 10] = np.nan
        out = smooth_elevation(elev, sigma_m=RES, resolution=RES)
        assert out[0, 0] == pytest.approx(0.2)

    def test_originally_nan_cells_stay_nan_not_inpainted(self):
        """희소한 맵(NaN 비율 높음)에서 빈 공간까지 이웃값으로 채우면 그
        경계에 새 가짜 경사가 생긴다(실기 확인) — 원래 NaN이던 셀은 값이
        있는 이웃이 있어도 절대 채우지 않아야 한다."""
        elev = np.full((11, 11), np.nan, dtype=np.float32)
        elev[5, 4] = 0.1
        elev[5, 6] = 0.1
        out = smooth_elevation(elev, sigma_m=RES, resolution=RES)
        assert np.isnan(out[5, 5])  # 양옆에 값이 있어도 원래 NaN이면 그대로 NaN


class TestFlatMap:
    def test_gradient_is_zero(self):
        gx, gy, mag, direction, slope_deg = compute_gradient_field(flat_map(), RES)
        np.testing.assert_allclose(gx, 0.0, atol=1e-6)
        np.testing.assert_allclose(gy, 0.0, atol=1e-6)
        np.testing.assert_allclose(mag, 0.0, atol=1e-6)
        np.testing.assert_allclose(slope_deg, 0.0, atol=1e-4)

    def test_output_shape_preserved(self):
        h, w = 13, 27
        elev = flat_map(h, w)
        for arr in compute_gradient_field(elev, RES):
            assert arr.shape == (h, w), f'Shape mismatch: {arr.shape}'


class TestRampX:
    slope = 0.25

    def test_gx_equals_slope_interior(self):
        """Interior cells should have gx ≈ slope (central diff is exact for linear)."""
        elev = ramp_x(slope=self.slope)
        gx, gy, mag, direction, slope_deg = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_gy_near_zero_interior(self):
        elev = ramp_x(slope=self.slope)
        _, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gy[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_magnitude_equals_slope(self):
        elev = ramp_x(slope=self.slope)
        _, _, mag, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(mag[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_direction_points_forward(self):
        """Uphill in +x → direction ≈ 0 radians."""
        elev = ramp_x(slope=self.slope)
        _, _, _, direction, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(direction[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_slope_deg_arctan(self):
        elev = ramp_x(slope=self.slope)
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)
        expected = np.degrees(np.arctan(self.slope))
        np.testing.assert_allclose(slope_deg[1:-1, 1:-1], expected, atol=1e-3)


class TestRampY:
    slope = 0.18

    def test_gy_equals_slope_interior(self):
        elev = ramp_y(slope=self.slope)
        _, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gy[1:-1, 1:-1], self.slope, atol=1e-5)

    def test_gx_near_zero_interior(self):
        elev = ramp_y(slope=self.slope)
        gx, _, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], 0.0, atol=1e-5)

    def test_direction_points_left(self):
        """Uphill in +y → direction ≈ pi/2 radians."""
        elev = ramp_y(slope=self.slope)
        _, _, _, direction, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(direction[1:-1, 1:-1], np.pi / 2, atol=1e-4)


class TestDiagonalRamp:
    sx, sy = 0.1, 0.15

    def test_combined_gx_gy(self):
        elev = diagonal_ramp(slope_x=self.sx, slope_y=self.sy)
        gx, gy, _, _, _ = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(gx[1:-1, 1:-1], self.sx, atol=1e-5)
        np.testing.assert_allclose(gy[1:-1, 1:-1], self.sy, atol=1e-5)

    def test_magnitude(self):
        elev = diagonal_ramp(slope_x=self.sx, slope_y=self.sy)
        _, _, mag, _, _ = compute_gradient_field(elev, RES)
        expected = np.hypot(self.sx, self.sy)
        np.testing.assert_allclose(mag[1:-1, 1:-1], expected, atol=1e-5)


class TestSlopeDeg45:
    """45° slope: tan(45°) = 1.0 → slope_deg = 45."""

    def test_45deg(self):
        # elevation[r, c] = 1.0 * c * RES  →  gx = 1.0  →  slope = arctan(1) = 45°
        elev = ramp_x(slope=1.0)
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)
        np.testing.assert_allclose(slope_deg[1:-1, 1:-1], 45.0, atol=0.01)


# ---------------------------------------------------------------------------
# centerline_row_targets
# ---------------------------------------------------------------------------

class TestCenterlineRowTargets:
    def test_empty_list_is_all_nan(self):
        t = centerline_row_targets([], grid_h=10, grid_w=10, resolution_m=0.15)
        assert np.all(np.isnan(t))

    def test_single_point_is_all_nan(self):
        t = centerline_row_targets([(0.0, 0.0)], grid_h=10, grid_w=10, resolution_m=0.15)
        assert np.all(np.isnan(t))

    def test_straight_centerline_at_zero_maps_to_center_row(self):
        t = centerline_row_targets([(0.0, 0.0), (1.5, 0.0)], grid_h=10, grid_w=10, resolution_m=0.15)
        np.testing.assert_allclose(t, 5.0)

    def test_out_of_range_columns_are_nan(self):
        t = centerline_row_targets([(0.0, 0.0), (0.6, -0.3)], grid_h=10, grid_w=10, resolution_m=0.15)
        assert not np.any(np.isnan(t[:5]))
        assert np.all(np.isnan(t[5:]))

    def test_drift_direction_matches_y_left_convention(self):
        """y_left이 음수(오른쪽)로 갈수록 row는 커져야 한다(row 0=왼쪽 규약)."""
        t = centerline_row_targets([(0.0, 0.0), (1.5, -0.3)], grid_h=10, grid_w=10, resolution_m=0.15)
        assert np.all(np.diff(t) > 0)


# ---------------------------------------------------------------------------
# plan_path_on_slope_field
# ---------------------------------------------------------------------------

class TestPlanPathOnSlopeField:
    def test_flat_field_goes_straight(self):
        """No obstacles → shortest path is the straight row toward target_col."""
        slope = np.zeros((10, 10), dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=9)
        assert path is not None
        assert path[0] == (5, 0)
        assert path[-1][1] == 9
        assert all(r == 5 for r, _ in path)

    def test_routes_around_localized_steep_patch(self):
        """A steep patch blocking only the middle rows should be detoured around."""
        h, w = 10, 10
        elev = np.zeros((h, w), dtype=np.float32)
        elev[3:7, 4:7] = 50.0  # steep wall, but only in rows 3-6
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)

        path = plan_path_on_slope_field(slope_deg, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is not None
        assert path[0] == (5, 0)
        assert path[-1][1] == 9
        # every visited cell must respect the slope limit
        for r, c in path:
            assert slope_deg[r, c] <= 30.0

    def test_returns_partial_path_when_wall_blocks_the_rest(self):
        """A steep wall spanning every row can't be crossed, but the reachable
        cells short of it should still come back as a best-effort partial
        path rather than nothing at all. See plan_path_on_slope_field()."""
        h, w = 10, 10
        elev = np.zeros((h, w), dtype=np.float32)
        elev[:, 6:] = 100.0  # every row blocked from column 6 onward
        _, _, _, _, slope_deg = compute_gradient_field(elev, RES)

        path = plan_path_on_slope_field(slope_deg, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is not None
        assert path[0] == (5, 0)
        assert path[-1][1] < 9  # never reached target_col — wall stopped it short
        # every visited cell must still respect the slope limit
        for r, c in path:
            assert slope_deg[r, c] <= 30.0

    def test_returns_none_when_start_impassable(self):
        slope = np.full((10, 10), 45.0, dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=9,
                                         max_slope_deg=30.0)
        assert path is None

    def test_slope_cost_weight_prefers_flatter_detour_over_shortest_path(self):
        """max_slope_deg 밑이기만 하면 완전 평지든
        threshold 바로 밑의 급경사든 이동 비용이 같아서, 바로 옆에 평평한
        길이 있어도 최단거리라는 이유만으로 급경사를 그대로 뚫고 지나가는
        경로가 나왔다. slope_cost_weight=0(기존 동작)에서는 직진, >0에서는
        같은 만큼 전진하면서 이 완만한 우회로를 선택해야 한다."""
        h, w = 10, 10
        slope = np.zeros((h, w), dtype=np.float32)
        # rows 4-5만 통과 가능한 정도로 가파름(20° < max_slope_deg=30°) —
        # 막힌 건 아니고 우회 가능한 정도. rows 0-3/6-9는 계속 평평하게
        # 전방으로 이어짐(막다른 길이 아님).
        slope[4:6, 4:6] = 20.0

        straight = plan_path_on_slope_field(
            slope, start=(5, 0), target_col=9, max_slope_deg=30.0,
            slope_cost_weight=0.0)
        assert any(r in (4, 5) and c in (4, 5) for r, c in straight), (
            'weight=0이면 그냥 최단거리(직진)라 steep patch를 지나가야 함')

        detoured = plan_path_on_slope_field(
            slope, start=(5, 0), target_col=9, max_slope_deg=30.0,
            slope_cost_weight=0.3)
        assert detoured is not None
        assert detoured[-1][1] == 9  # 그래도 target_col까지는 도달
        assert not any(r in (4, 5) and c in (4, 5) for r, c in detoured), (
            'weight>0이면 완만한 우회로가 있으니 steep patch를 피해야 함')

    def test_centerline_bias_pulls_path_toward_rgb_centerline(self):
        """지형에 경사 차이가 전혀 없으면(완전 평지) 원래는 직진이 최단거리지만,
        centerline_row_by_col로 다른 row를 지정하면 그쪽으로 붙어야 한다."""
        slope = np.zeros((10, 10), dtype=np.float32)
        target_row = np.full(10, 8.0)

        unbiased = plan_path_on_slope_field(slope, start=(5, 0), target_col=9)
        assert all(r == 5 for r, _ in unbiased)

        biased = plan_path_on_slope_field(
            slope, start=(5, 0), target_col=9,
            centerline_row_by_col=target_row, centerline_cost_weight=0.5)
        assert biased[-1] == (8, 9)

    def test_centerline_bias_disabled_by_zero_weight(self):
        slope = np.zeros((10, 10), dtype=np.float32)
        target_row = np.full(10, 8.0)
        path = plan_path_on_slope_field(
            slope, start=(5, 0), target_col=9,
            centerline_row_by_col=target_row, centerline_cost_weight=0.0)
        assert all(r == 5 for r, _ in path)

    def test_centerline_bias_ignores_nan_columns(self):
        """centerline_row_by_col의 특정 열이 NaN이면(관측 범위 밖) 그 열에서는
        유도 비용이 안 붙어야 한다 — 그 구간만 평범한 최단거리로 통과."""
        slope = np.zeros((10, 10), dtype=np.float32)
        target_row = np.full(10, np.nan)
        path = plan_path_on_slope_field(
            slope, start=(5, 0), target_col=9,
            centerline_row_by_col=target_row, centerline_cost_weight=0.5)
        assert all(r == 5 for r, _ in path)

    def test_raises_on_start_out_of_bounds(self):
        slope = np.zeros((10, 10), dtype=np.float32)
        with pytest.raises(ValueError):
            plan_path_on_slope_field(slope, start=(20, 0), target_col=9)

    def test_target_col_beyond_grid_is_clamped(self):
        slope = np.zeros((10, 10), dtype=np.float32)
        path = plan_path_on_slope_field(slope, start=(5, 0), target_col=999)
        assert path is not None
        assert path[-1][1] == 9  # clamped to last column


class TestApplyLateralClearance:
    """로봇 폭만큼 좌우 여유가 없으면(row 축 = 좌우) 그 셀도 impassable로
    처리하는지 검증. 실기 테스트에서 발견된, 셀 1~2개 폭짜리 틈으로
    경로가 새는 문제를 재현/방지하는 테스트."""

    def test_zero_margin_returns_unchanged_copy(self):
        slope = np.array([[10.0, 50.0], [50.0, 10.0]], dtype=np.float32)
        out = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=0)
        assert np.array_equal(out, slope, equal_nan=True)
        out[0, 0] = 999.0
        assert slope[0, 0] == 10.0  # 원본 안 건드림(복사본)

    def test_narrow_gap_between_walls_is_closed(self):
        """행 0-3, 6-9는 막혀있고 4-5만(2칸=0.30m) 뚫려있음 — margin_cells=2
        (편도 여유 2칸=0.30m 필요)면 이 좁은 틈은 로봇이 못 지나가는 걸로
        처리돼야 한다(전부 NaN)."""
        h, w = 10, 5
        slope = np.zeros((h, w), dtype=np.float32)
        slope[0:4, :] = 50.0
        slope[6:10, :] = 50.0
        # 행 4,5만 0.0(통과 가능) — 폭 2칸짜리 좁은 틈

        out = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=2)
        assert np.all(np.isnan(out[4:6, :]))

    def test_wide_enough_gap_stays_open(self):
        """행 0-2, 8-9만 막혀있고 3-7(5칸=0.75m)이 뚫려있으면, margin_cells=2
        여유로도 중앙(행 5) 쪽은 그대로 통과 가능해야 한다."""
        h, w = 10, 5
        slope = np.zeros((h, w), dtype=np.float32)
        slope[0:3, :] = 50.0
        slope[8:10, :] = 50.0

        out = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=2)
        assert not np.any(np.isnan(out[5, :]))
        assert np.all(out[5, :] <= 30.0)

    def test_grid_edge_without_enough_margin_is_blocked(self):
        """맨 위/아래 가장자리는 여유를 확보할 수 있는지 알 수 없으니
        안전하게 통과 불가로 처리돼야 한다."""
        h, w = 10, 5
        slope = np.zeros((h, w), dtype=np.float32)  # 전부 평지
        out = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=2)
        assert np.all(np.isnan(out[0:2, :]))   # 위쪽 가장자리
        assert np.all(np.isnan(out[8:10, :]))  # 아래쪽 가장자리
        assert not np.any(np.isnan(out[2:8, :]))  # 중간은 여유 확보되니 그대로

    def test_nan_cell_blocks_neighbors_within_margin(self):
        """관측 안 된(NaN) 셀도 impassable 취급되어 margin 안의 이웃까지 막아야 한다."""
        h, w = 10, 5
        slope = np.zeros((h, w), dtype=np.float32)
        slope[5, :] = np.nan
        out = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=2)
        # 5행 기준 위아래 2칸(3,4,5,6,7)까지 전부 막혀야 함
        assert np.all(np.isnan(out[3:8, :]))

    def test_integration_forces_route_around_narrow_gap(self):
        """apply_lateral_clearance를 먼저 적용하면, 원래(전처리 없이는)
        셀 1~2개 폭 틈으로 새던 경로가 그 틈을 피해서 계획돼야 한다."""
        h, w = 10, 10
        slope = np.zeros((h, w), dtype=np.float32)
        # 열 4~5에 벽을 세우되, 행 4-5(2칸 폭)만 뚫어둠(좁은 틈).
        slope[0:4, 4:6] = 50.0
        slope[6:10, 4:6] = 50.0

        # 전처리 없이 원래 방식대로 하면 그 좁은 틈으로 경로가 지나갈 수 있다.
        raw_path = plan_path_on_slope_field(slope, start=(5, 0), target_col=9,
                                             max_slope_deg=30.0)
        assert raw_path is not None
        assert any(4 <= c <= 5 and r in (4, 5) for r, c in raw_path)

        # 폭 여유(margin_cells=2)를 적용하면 그 틈이 막혀서, 완전히 막힌
        # 벽이 된다(양옆 행이 다 막혀있어 틈 자체가 NaN) — target_col까지는
        # 못 가지만, 벽 앞(열 3)까지의 부분 경로는 여전히 최선책으로 반환된다
        # (plan_path_on_slope_field() 참고).
        cleared = apply_lateral_clearance(slope, max_slope_deg=30.0, margin_cells=2)
        blocked_path = plan_path_on_slope_field(cleared, start=(5, 0), target_col=9,
                                                  max_slope_deg=30.0)
        assert blocked_path is not None
        assert blocked_path[-1] == (5, 3)  # 벽(열 4) 바로 앞에서 멈춤
        assert blocked_path[-1][1] < 9


# ---------------------------------------------------------------------------
# Benchmark tests
# ---------------------------------------------------------------------------

class TestBenchmark:
    """Timing gates — fail if gradient computation is unreasonably slow.

    Target pipeline: 5–10 Hz on onboard i7 (no GPU).
    Budget per gradient call: well under 10 ms even for large maps.
    """

    @staticmethod
    def _time_ms(h: int, w: int, n: int = 200) -> float:
        elev = np.random.rand(h, w).astype(np.float32)
        compute_gradient_field(elev, RES)          # warm-up
        t0 = time.perf_counter()
        for _ in range(n):
            compute_gradient_field(elev, RES)
        return (time.perf_counter() - t0) / n * 1e3

    def test_bench_nominal_27x27(self):
        """4 m × 4 m @ 0.15 m → 27×27 cells.  Must finish < 2 ms."""
        elapsed = self._time_ms(27, 27)
        print(f'\n  gradient_field 27×27: {elapsed:.3f} ms/call')
        assert elapsed < 2.0, f'{elapsed:.2f} ms > 2 ms threshold'

    def test_bench_medium_50x50(self):
        """7.5 m × 7.5 m @ 0.15 m → 50×50 cells.  Must finish < 5 ms."""
        elapsed = self._time_ms(50, 50)
        print(f'\n  gradient_field 50×50: {elapsed:.3f} ms/call')
        assert elapsed < 5.0, f'{elapsed:.2f} ms > 5 ms threshold'

    def test_bench_large_100x100(self):
        """15 m × 15 m @ 0.15 m → 100×100 cells.  Must finish < 15 ms."""
        elapsed = self._time_ms(100, 100)
        print(f'\n  gradient_field 100×100: {elapsed:.3f} ms/call')
        assert elapsed < 15.0, f'{elapsed:.2f} ms > 15 ms threshold'


# ---------------------------------------------------------------------------
# _cells_to_path_msg — header/frame_id 회귀 테스트
#
# 배경: 발행되는 /terrain/planned_path 좌표는 body 규약(x=전방, y=좌측)인데,
# 이 메서드가 예전에는 입력 elevation map의 Header(depth 이미지에서 그대로
# 물려받음, 보통 frame_id='camera_*_optical_frame')를 그대로 재사용해서
# 좌표값과 frame_id가 서로 다른 규약으로 나갔었다. 여기서는 새 Header
# (frame_id=path_frame_id 파라미터, stamp만 입력에서 유지)를 만들어 쓰도록
# 고쳤다 — 이 테스트가 그 동작을 고정한다. flat_drive.py의
# TestPathToMsgFrameId와 동일한 패턴.
# ---------------------------------------------------------------------------

class TestCellsToPathMsgFrameId:
    @staticmethod
    def _make_node():
        from dolbotz.drive_area.gradient_map import GradientMapNode
        return GradientMapNode()

    def test_optical_frame_input_becomes_camera_link_output(self):
        rclpy.init()
        try:
            node = self._make_node()
            try:
                source_header = Header()
                source_header.frame_id = 'camera_depth_optical_frame'
                source_header.stamp.sec = 123
                source_header.stamp.nanosec = 456

                cells = [(5, 0), (4, 1), (3, 2)]  # (row, col)
                msg = node._cells_to_path_msg(cells, grid_h=10, source_header=source_header)

                assert msg.header.frame_id == 'camera_link'
                assert len(msg.poses) == len(cells)
                for pose in msg.poses:
                    assert pose.header.frame_id == msg.header.frame_id
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_stamp_preserved_and_x_y_unchanged(self):
        rclpy.init()
        try:
            node = self._make_node()
            try:
                source_header = Header()
                source_header.frame_id = 'camera_depth_optical_frame'
                source_header.stamp.sec = 100
                source_header.stamp.nanosec = 250

                grid_h = 10
                center_row = grid_h // 2
                cells = [(5, 0), (4, 1)]  # (row, col)
                msg = node._cells_to_path_msg(cells, grid_h=grid_h, source_header=source_header)

                assert msg.header.stamp.sec == source_header.stamp.sec
                assert msg.header.stamp.nanosec == source_header.stamp.nanosec
                for pose, (row, col) in zip(msg.poses, cells):
                    assert pose.header.stamp.sec == source_header.stamp.sec
                    assert pose.header.stamp.nanosec == source_header.stamp.nanosec
                    assert pose.pose.position.x == pytest.approx(col * node._res)
                    assert pose.pose.position.y == pytest.approx((center_row - row) * node._res)
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_source_header_object_is_not_mutated(self):
        rclpy.init()
        try:
            node = self._make_node()
            try:
                source_header = Header()
                source_header.frame_id = 'camera_depth_optical_frame'
                source_header.stamp.sec = 1
                source_header.stamp.nanosec = 2

                node._cells_to_path_msg([(1, 0)], grid_h=10, source_header=source_header)

                assert source_header.frame_id == 'camera_depth_optical_frame'
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_path_frame_id_parameter_defaults_to_camera_link(self):
        rclpy.init()
        try:
            node = self._make_node()
            try:
                assert node.get_parameter('path_frame_id').value == 'camera_link'
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()
