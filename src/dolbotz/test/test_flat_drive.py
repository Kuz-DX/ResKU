"""
Unit tests for the ROS-free pure functions in dolbotz/flat_drive.py.

Run (requires ROS env sourced, since flat_drive.py imports rclpy at module level
— same requirement as test/test_gradient_field.py, see howtorun.md):
    source /opt/ros/humble/setup.bash
    python3 -m pytest test/test_flat_drive.py -v

Independent-physics style, mirroring TestCameraBodyToLevelMatrix in
test_elevation_map.py: synthetic ground truth is built from a from-scratch
R_total rotation (mount composed with chassis dynamic tilt), never by feeding
the function-under-test's own output back into itself. This is deliberate —
that kind of circularity is exactly what let the mount-offset-double-removal
bug in elevation_map.py's camera_body_to_level_matrix() slip through undetected.
"""

import time

import cv2
import numpy as np
import pytest
import rclpy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

from dolbotz.drive_area.flat_drive import (
    bev_ground_projection_matrix,
    bev_mask_to_centerline_path,
    bev_pixel_to_meters,
    bridge_interior_gaps,
    extend_path_to_robot,
    ground_to_image_homography,
    image_to_bev_homography,
    mask_row_width_plausible,
    mask_to_bev,
)
from dolbotz.utils.attitude import R_BODY_TO_OPTICAL, roll_pitch_from_accel_body

G = 9.81
CAMERA_HEIGHT = 0.5
FX = FY = 500.0
CX, CY = 320.0, 240.0
CAMERA_MATRIX = np.array([
    [FX, 0.0, CX],
    [0.0, FY, CY],
    [0.0, 0.0, 1.0],
])


def _r_total(mount_roll_deg, mount_pitch_deg, chassis_roll_deg, chassis_pitch_deg) -> Rotation:
    """The one true physical rotation (world-level -> camera's current frame):
    fixed mount tilt composed with the chassis's current dynamic lean. Built
    completely independently of anything in flat_drive.py or elevation_map.py."""
    mount = Rotation.from_euler(
        'xyz', [np.radians(mount_roll_deg), np.radians(mount_pitch_deg), 0])
    chassis = Rotation.from_euler(
        'xyz', [np.radians(chassis_roll_deg), np.radians(chassis_pitch_deg), 0])
    return mount * chassis


def _independent_raw_pixel(r_total: Rotation, x_forward: float, y_left: float) -> tuple[float, float]:
    """Ground truth: project a known world-level ground point straight through
    pinhole optics, using only R_total (independently built) + the already-
    validated R_BODY_TO_OPTICAL constant + the definitional camera_matrix
    projection. Never calls ground_to_image_homography()."""
    p_level = np.array([x_forward, y_left, -CAMERA_HEIGHT])
    p_body_current = r_total.inv().apply(p_level)
    p_optical = R_BODY_TO_OPTICAL @ p_body_current
    pixel_h = CAMERA_MATRIX @ p_optical
    return float(pixel_h[0] / pixel_h[2]), float(pixel_h[1] / pixel_h[2])


MOUNT_PITCH_DEGS = [0.0, 10.0, 20.0]
CHASSIS_PITCH_DEGS = [0.0, 7.0, -7.0]
GROUND_POINTS = [(1.0, 0.0), (2.0, 0.8), (2.5, -1.2), (0.8, 0.3)]


class TestGroundToImageHomography:
    @pytest.mark.parametrize("mount_pitch_deg", MOUNT_PITCH_DEGS)
    @pytest.mark.parametrize("chassis_pitch_deg", CHASSIS_PITCH_DEGS)
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_known_ground_point_projects_to_correct_raw_pixel(
        self, mount_pitch_deg, chassis_pitch_deg, x_forward, y_left,
    ):
        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        u_expected, v_expected = _independent_raw_pixel(r_total, x_forward, y_left)

        h_g2i = ground_to_image_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg), CAMERA_HEIGHT)
        pixel_h = h_g2i @ np.array([x_forward, y_left, 1.0])
        u_actual, v_actual = pixel_h[0] / pixel_h[2], pixel_h[1] / pixel_h[2]

        assert u_actual == pytest.approx(u_expected, abs=1e-6)
        assert v_actual == pytest.approx(v_expected, abs=1e-6)

    def test_mount_offset_parameters_do_not_affect_result(self):
        """Passing different roll_offset/pitch_offset values must not change the
        output at all — they are structurally unused (see docstring), exactly
        mirroring camera_body_to_level_matrix()'s fixed behaviour."""
        roll_meas, pitch_meas = np.radians(3.0), np.radians(12.0)
        h1 = ground_to_image_homography(CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, 0.0, CAMERA_HEIGHT)
        h2 = ground_to_image_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, np.radians(99.0), np.radians(-42.0), CAMERA_HEIGHT)
        np.testing.assert_allclose(h1, h2)


class TestBevGroundProjectionMatrix:
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_meters_to_pixel_and_back_round_trips(self, x_forward, y_left):
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        pixel_h = m @ np.array([x_forward, y_left, 1.0])
        col, row = pixel_h[0] / pixel_h[2], pixel_h[1] / pixel_h[2]

        x_rec, y_rec = bev_pixel_to_meters(row, col, 400, 400, 0.02)
        assert x_rec == pytest.approx(x_forward, abs=1e-9)
        assert y_rec == pytest.approx(y_left, abs=1e-9)

    def test_forward_distance_increases_toward_top_of_image(self):
        """Larger x_forward (farther ahead) must land at a smaller row (higher up
        the BEV image) — the conventional 'road recedes upward' BEV layout."""
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        near = m @ np.array([0.5, 0.0, 1.0])
        far = m @ np.array([3.0, 0.0, 1.0])
        assert far[1] / far[2] < near[1] / near[2]

    def test_leftward_offset_moves_toward_smaller_column(self):
        """Positive y_left (robot's left) must land at a smaller column (left
        side of the BEV image), matching an intuitive, non-mirrored top-down view."""
        m = bev_ground_projection_matrix(bev_width_px=400, bev_height_px=400, bev_meters_per_pixel=0.02)
        center = m @ np.array([1.0, 0.0, 1.0])
        left = m @ np.array([1.0, 0.8, 1.0])
        assert left[0] / left[2] < center[0] / center[2]


class TestImageToBevHomographyEndToEnd:
    @pytest.mark.parametrize("mount_pitch_deg", MOUNT_PITCH_DEGS)
    @pytest.mark.parametrize("chassis_pitch_deg", CHASSIS_PITCH_DEGS)
    @pytest.mark.parametrize("x_forward,y_left", GROUND_POINTS)
    def test_known_ground_point_raw_pixel_warps_to_correct_bev_pixel(
        self, mount_pitch_deg, chassis_pitch_deg, x_forward, y_left,
    ):
        """Full chain: an independently-computed raw camera pixel for a known
        ground point, run through image_to_bev_homography(), must land at the
        BEV pixel that bev_ground_projection_matrix() independently predicts for
        that same ground point. This is the check that actually matters for the
        real pipeline (cv2.warpPerspective consumes exactly this homography)."""
        bev_w, bev_h, mpp = 400, 400, 0.02

        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        u_raw, v_raw = _independent_raw_pixel(r_total, x_forward, y_left)

        h_full = image_to_bev_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg),
            CAMERA_HEIGHT, bev_w, bev_h, mpp)
        bev_h_pixel = h_full @ np.array([u_raw, v_raw, 1.0])
        col_actual, row_actual = bev_h_pixel[0] / bev_h_pixel[2], bev_h_pixel[1] / bev_h_pixel[2]

        m = bev_ground_projection_matrix(bev_w, bev_h, mpp)
        expected_h = m @ np.array([x_forward, y_left, 1.0])
        col_expected, row_expected = expected_h[0] / expected_h[2], expected_h[1] / expected_h[2]

        assert col_actual == pytest.approx(col_expected, abs=1e-4)
        assert row_actual == pytest.approx(row_expected, abs=1e-4)


class TestMaskToBev:
    def test_identity_homography_is_a_passthrough(self):
        mask = np.zeros((50, 50), dtype=np.uint8)
        mask[10:20, 15:25] = 255
        bev = mask_to_bev(mask, np.eye(3), bev_width_px=50, bev_height_px=50)
        np.testing.assert_array_equal(bev, mask)

    def test_uses_nearest_neighbor_no_gray_values(self):
        """A binary mask must warp to another binary mask (0/255 only) — no
        antialiased/gray edge pixels from linear interpolation."""
        mask = np.zeros((60, 60), dtype=np.uint8)
        mask[20:40, 20:40] = 255
        # A homography that isn't axis-aligned forces resampling at non-integer
        # source coordinates, which is where interpolation artifacts would show.
        rot = np.array([
            [np.cos(0.3), -np.sin(0.3), 5.0],
            [np.sin(0.3), np.cos(0.3), 3.0],
            [0.0, 0.0, 1.0],
        ])
        bev = mask_to_bev(mask, rot, bev_width_px=60, bev_height_px=60)
        assert set(np.unique(bev).tolist()) <= {0, 255}


class TestBevMaskToCenterlinePath:
    BEV_W, BEV_H, MPP = 100, 100, 0.05

    def _corridor_mask(self, col_center_fn, half_width_px=10):
        """Build a synthetic BEV mask of a corridor whose column center varies
        per row according to col_center_fn(row)."""
        mask = np.zeros((self.BEV_H, self.BEV_W), dtype=np.uint8)
        for row in range(self.BEV_H):
            c = col_center_fn(row)
            lo, hi = int(round(c - half_width_px)), int(round(c + half_width_px))
            mask[row, max(0, lo):min(self.BEV_W, hi + 1)] = 255
        return mask

    def test_straight_centered_corridor_reads_zero_lateral_offset(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert len(path) == self.BEV_H
        y_lefts = np.array([y for _, y in path])
        np.testing.assert_allclose(y_lefts, 0.0, atol=self.MPP)  # within half a pixel

    def test_straight_corridor_forward_distance_increases_from_robot_outward(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        x_forwards = np.array([x for x, _ in path])
        assert np.all(np.diff(x_forwards) > 0)  # near (robot) -> far, strictly increasing
        assert x_forwards[0] == pytest.approx(self.MPP, abs=1e-9)  # row=H-1, nearest to robot
        assert x_forwards[-1] == pytest.approx(self.BEV_H * self.MPP, abs=1e-9)  # row=0, farthest

    def test_curved_corridor_lateral_offset_tracks_known_curve(self):
        """Corridor that drifts linearly leftward as it recedes (row decreases)
        — the extracted centerline's y_left must track the known drift."""
        slope_px_per_row = 0.3

        def col_center(row):
            return self.BEV_W / 2.0 + slope_px_per_row * (self.BEV_H - 1 - row)

        mask = self._corridor_mask(col_center)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        for x_forward, y_left in path:
            row = self.BEV_H - x_forward / self.MPP
            expected_col = col_center(row)
            expected_y_left = (self.BEV_W / 2.0 - expected_col) * self.MPP
            assert y_left == pytest.approx(expected_y_left, abs=self.MPP)

    def test_rows_below_min_pixel_threshold_are_skipped_as_holes(self):
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0)  # full-width rows elsewhere
        gap_rows = [40, 41, 42]
        for row in gap_rows:
            mask[row] = 0
            mask[row, 48:50] = 255  # only 2 lit pixels — below min_row_pixels=5

        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert len(path) == self.BEV_H - len(gap_rows)

        gap_x_forwards = {(self.BEV_H - r) * self.MPP for r in gap_rows}
        present_x_forwards = {x for x, _ in path}
        assert gap_x_forwards.isdisjoint(present_x_forwards)

    def test_empty_mask_returns_empty_path(self):
        mask = np.zeros((self.BEV_H, self.BEV_W), dtype=np.uint8)
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert path == []

    def test_width_check_disabled_by_default_keeps_old_behavior(self):
        """expected_track_width_m을 안 주면(기본 None) 아무리 넓은 row라도
        예전처럼 그대로 통과해야 한다 — 하위호환 확인."""
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0, half_width_px=45)  # 4.5m 폭
        path = bev_mask_to_centerline_path(mask, min_row_pixels=5,
                                            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
                                            bev_meters_per_pixel=self.MPP)
        assert len(path) == self.BEV_H

    def test_overly_wide_row_is_skipped_when_width_check_enabled(self):
        """폭 4.5m짜리 통로는 예상 트랙 폭(0.9144m) * tolerance(1.5)=1.37m를
        훨씬 넘으니, 폭 검사를 켜면 그 row들이 전부 걸러져야 한다."""
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0, half_width_px=45)  # 4.5m 폭
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            expected_track_width_m=0.9144, track_width_tolerance_factor=1.5)
        assert path == []

    def test_plausible_width_row_unaffected_by_width_check(self):
        """실제 트랙 폭(0.9144m)에 가까운 통로는 폭 검사를 켜도 그대로 통과해야 한다."""
        half_width_px = int(round((0.9144 / 2.0) / self.MPP))  # ~= 9px
        mask = self._corridor_mask(lambda row: self.BEV_W / 2.0, half_width_px=half_width_px)
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            expected_track_width_m=0.9144, track_width_tolerance_factor=1.5)
        assert len(path) == self.BEV_H

    def test_left_truncated_row_offsets_inward_from_right_edge(self):
        mask = self._corridor_mask(lambda row: 10.0, half_width_px=20)  # cols 0..30
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            robot_half_width_m=0.25)
        expected_x_forwards = [(self.BEV_H - row) * self.MPP for row in range(self.BEV_H - 1, -1, -1)]
        expected_y_left = (self.BEV_W / 2.0 - 30) * self.MPP + 0.25
        np.testing.assert_allclose([x for x, _ in path], expected_x_forwards)
        np.testing.assert_allclose([y for _, y in path], expected_y_left)

    def test_right_truncated_row_offsets_inward_from_left_edge(self):
        mask = self._corridor_mask(lambda row: self.BEV_W - 10.0, half_width_px=20)  # cols 70..99
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            robot_half_width_m=0.25)
        expected_y_left = (self.BEV_W / 2.0 - 70) * self.MPP - 0.25
        np.testing.assert_allclose([y for _, y in path], expected_y_left)

    def test_truncated_row_uses_half_regulation_track_width(self):
        mask = self._corridor_mask(lambda row: 10.0, half_width_px=20)  # cols 0..30
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H,
            bev_meters_per_pixel=self.MPP,
            expected_track_width_m=0.9144,
            track_width_tolerance_factor=10.0,
            robot_half_width_m=0.25)

        expected_y_left = (self.BEV_W / 2.0 - 30) * self.MPP + 0.4572
        np.testing.assert_allclose([y for _, y in path], expected_y_left)

    def test_both_edges_truncated_falls_back_to_centroid(self):
        mask = np.full((self.BEV_H, self.BEV_W), 255, dtype=np.uint8)  # full-width every row
        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            robot_half_width_m=0.25)
        y_lefts = np.array([y for _, y in path])
        np.testing.assert_allclose(y_lefts, 0.0, atol=self.MPP)

    def test_truncated_edge_offset_is_normal_not_purely_lateral_on_a_curve(self):
        """접선(같은 쪽 경계의 직전 row 대비)이 기울어져 있으면, 오프셋이
        column뿐 아니라 row(전방거리)에도 성분을 가져야 한다 — 법선벡터
        오프셋이지 단순 수평 이동이 아님을 확인."""
        slope_px_per_row = 0.1

        def col_center(row):
            return 10.0 + slope_px_per_row * (self.BEV_H - 1 - row)

        mask = self._corridor_mask(col_center, half_width_px=30)  # left side touches col 0
        for row in range(self.BEV_H):
            assert mask[row, 0] == 255  # 전제 확인: 왼쪽은 항상 잘려있음

        path = bev_mask_to_centerline_path(
            mask, min_row_pixels=5,
            bev_width_px=self.BEV_W, bev_height_px=self.BEV_H, bev_meters_per_pixel=self.MPP,
            robot_half_width_m=0.25)

        unshifted_x_forwards = {(self.BEV_H - r) * self.MPP for r in range(self.BEV_H)}
        drifted = [x for x, _ in path[2:] if x not in unshifted_x_forwards]
        assert len(drifted) > 0


class TestMaskRowWidthPlausible:
    def test_within_tolerance_is_plausible(self):
        cols = np.arange(0, 20)  # 20px * 0.05m/px = 1.0m
        assert mask_row_width_plausible(
            cols, meters_per_pixel=0.05, expected_width_m=0.9144, tolerance_factor=1.5)

    def test_beyond_tolerance_is_not_plausible(self):
        cols = np.arange(0, 100)  # 100px * 0.05m/px = 5.0m — 훨씬 넓음
        assert not mask_row_width_plausible(
            cols, meters_per_pixel=0.05, expected_width_m=0.9144, tolerance_factor=1.5)

    def test_narrow_span_is_plausible(self):
        """더 좁은 건(가려짐 등으로) 정상일 수 있으니 하한은 안 본다."""
        cols = np.arange(0, 3)  # 3px * 0.05m/px = 0.15m — 훨씬 좁음
        assert mask_row_width_plausible(
            cols, meters_per_pixel=0.05, expected_width_m=0.9144, tolerance_factor=1.5)

    def test_empty_cols_is_not_plausible(self):
        assert not mask_row_width_plausible(
            np.array([], dtype=np.int64), meters_per_pixel=0.05,
            expected_width_m=0.9144, tolerance_factor=1.5)

    def test_exact_boundary_is_plausible(self):
        # expected_width_m * tolerance_factor = 1.0m 정확히 맞춘 폭(20px @ 0.05m/px)
        cols = np.arange(0, 20)
        assert mask_row_width_plausible(
            cols, meters_per_pixel=0.05, expected_width_m=1.0, tolerance_factor=1.0)


class TestSegmentationToPathEndToEnd:
    """Ties the re-derived homography together with mask_to_bev +
    bev_mask_to_centerline_path: a known straight corridor in world-level
    ground coordinates is projected into a synthetic *raw camera image* mask
    (independent of the homography under test — via the same from-scratch
    R_total + pinhole projection helper used in TestGroundToImageHomography),
    then run through the real mask_to_bev -> bev_mask_to_centerline_path
    pipeline, and the recovered centerline must match the known corridor
    centerline."""

    def test_known_world_corridor_recovered_through_full_pipeline(self):
        bev_w, bev_h, mpp = 200, 200, 0.02
        mount_pitch_deg = 10.0
        chassis_pitch_deg = 3.0
        y_center = 0.3  # corridor centered 0.3 m left of the robot's forward axis
        half_width = 0.4  # 0.8 m wide corridor

        r_total = _r_total(0.0, mount_pitch_deg, 0.0, chassis_pitch_deg)
        accel_body = r_total.inv().apply([0, 0, G])
        roll_meas, pitch_meas = roll_pitch_from_accel_body(accel_body)

        img_w, img_h = 640, 480
        raw_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        corners_world = []
        for x_forward in np.linspace(0.8, 3.5, 60):
            for y_left in (y_center - half_width, y_center + half_width):
                corners_world.append((x_forward, y_left))
        pixels = np.array([
            _independent_raw_pixel(r_total, xf, yl) for xf, yl in corners_world
        ], dtype=np.float32)
        hull = cv2.convexHull(pixels)
        cv2.fillConvexPoly(raw_mask, hull.astype(np.int32), 255)

        h_full = image_to_bev_homography(
            CAMERA_MATRIX, roll_meas, pitch_meas, 0.0, np.radians(mount_pitch_deg),
            CAMERA_HEIGHT, bev_w, bev_h, mpp)
        bev_mask = mask_to_bev(raw_mask, h_full, bev_w, bev_h)

        path = bev_mask_to_centerline_path(bev_mask, min_row_pixels=5,
                                            bev_width_px=bev_w, bev_height_px=bev_h,
                                            bev_meters_per_pixel=mpp)

        recovered = [(x, y) for x, y in path if 1.2 <= x <= 3.0]
        assert len(recovered) > 20
        y_lefts = np.array([y for _, y in recovered])
        np.testing.assert_allclose(y_lefts, y_center, atol=0.05)


# ---------------------------------------------------------------------------
# _path_to_msg — header/frame_id 회귀 테스트
#
# 배경: 발행되는 Path 좌표는 body 규약(x=전방, y=좌측)인데, 이 메서드가
# 예전에는 입력 컬러 이미지의 Header(보통 frame_id='camera_*_optical_frame',
# optical 좌표계: x=오른쪽/y=아래/z=전방)를 그대로 재사용해서 좌표값과
# frame_id가 서로 다른 규약으로 나갔었다 (path_relay_node가 이 frame_id를
# 믿고 TF 변환하므로 실제 주행에서 조향이 잘못될 수 있었음). 여기서는 새
# Header(frame_id=path_frame_id 파라미터, stamp만 입력에서 유지)를 만들어
# 쓰도록 고쳤다 — 이 테스트가 그 동작을 고정한다.
# ---------------------------------------------------------------------------

class TestPathToMsgFrameId:
    @staticmethod
    def _make_node():
        from dolbotz.drive_area.flat_drive import FlatDriveNode
        return FlatDriveNode()

    def test_optical_frame_input_becomes_camera_link_output(self):
        rclpy.init()
        try:
            node = self._make_node()
            try:
                source_header = Header()
                source_header.frame_id = 'camera_color_optical_frame'
                source_header.stamp.sec = 123
                source_header.stamp.nanosec = 456

                path_points = [(1.0, 0.2), (1.5, -0.1)]
                msg = node._path_to_msg(path_points, source_header)

                assert msg.header.frame_id == 'camera_link'
                assert len(msg.poses) == len(path_points)
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
                source_header.frame_id = 'camera_color_optical_frame'
                source_header.stamp.sec = 100
                source_header.stamp.nanosec = 250

                path_points = [(0.5, 0.3), (2.7, -0.4)]
                msg = node._path_to_msg(path_points, source_header)

                assert msg.header.stamp.sec == source_header.stamp.sec
                assert msg.header.stamp.nanosec == source_header.stamp.nanosec
                for pose, (x_forward, y_left) in zip(msg.poses, path_points):
                    assert pose.header.stamp.sec == source_header.stamp.sec
                    assert pose.header.stamp.nanosec == source_header.stamp.nanosec
                    assert pose.pose.position.x == pytest.approx(x_forward)
                    assert pose.pose.position.y == pytest.approx(y_left)
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
                source_header.frame_id = 'camera_color_optical_frame'
                source_header.stamp.sec = 1
                source_header.stamp.nanosec = 2

                node._path_to_msg([(1.0, 0.0)], source_header)

                assert source_header.frame_id == 'camera_color_optical_frame'
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


# ---------------------------------------------------------------------------
# extend_path_to_robot(): 여름 미션에서 자갈 구간처럼 BEV 하단(로봇 근처)에
# segmentation 유효 row가 없어 path_points가 로봇 바로 앞부터 시작하지 못할
# 때, 로봇(BEV 하단 중앙, 0,0)에서 자갈 너머 첫 신뢰 가능 경로 구간까지
# 직선으로 연결하는 폴백(2026-09-05, 사용자 요청). 방향/목표점은 앞쪽(로봇
#에서 먼 쪽) 여러 점을 최소제곱 직선 피팅해서 추정 — 노이즈 한 점에 흔들려
# 반대 방향으로 조향하는 사고를 막기 위함.
# 반환값은 (extension 점들, 로봇에서 먼 순) + path_points(원본, 그대로) 순서.
# ---------------------------------------------------------------------------

class TestExtendPathToRobot:
    def test_noop_when_gap_within_threshold(self):
        path_points = [(0.1, 0.0), (0.22, 0.0)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)
        assert out == path_points

    def test_noop_when_empty(self):
        assert extend_path_to_robot([], 0.12, min_gap_m=0.3) == []

    def test_first_point_is_exactly_robot_origin(self):
        """항목1: 하단 마스크가 없는 상태에서 생성 경로의 첫 점이 정확히
        (0,0)인지 확인 — 상단 경로 접선을 로봇 위치까지 연장한 값(0이 아닐
        수 있음)이 아니라, 로봇이 실제로 서 있는 화면 하단 중앙에서
        시작해야 한다(이전 구현의 버그, 2026-09-05 사용자 지적)."""
        path_points = [(1.0, 0.5), (1.12, 0.55), (1.24, 0.6), (1.36, 0.65), (1.48, 0.7)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)
        assert out[0] == (pytest.approx(0.0, abs=1e-9), pytest.approx(0.0, abs=1e-9))

    def test_steers_same_side_as_upstream_path_when_left(self):
        """다음 경로가 왼쪽(y_left > 0)이면 연결 구간도 왼쪽으로 나가야 한다."""
        path_points = [(1.0, 0.5), (1.12, 0.55), (1.24, 0.6), (1.36, 0.65), (1.48, 0.7)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)
        extension = out[:-len(path_points)]
        assert len(extension) > 0
        # 로봇 원점 이후 점들은(원점 자체는 제외) 전부 같은 부호(왼쪽)여야 함.
        for x, y in extension[1:]:
            assert y > 0.0

    def test_steers_same_side_as_upstream_path_when_right(self):
        """다음 경로가 오른쪽(y_left < 0)이면 연결 구간도 오른쪽으로 나가야 한다."""
        path_points = [(1.0, -0.5), (1.12, -0.55), (1.24, -0.6), (1.36, -0.65), (1.48, -0.7)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)
        extension = out[:-len(path_points)]
        assert len(extension) > 0
        for x, y in extension[1:]:
            assert y < 0.0

    def test_single_noisy_point_does_not_flip_steering_direction(self):
        """항목3: 첫 두 점에 노이즈가 있어도 반대 방향 연장이 발생하지 않는지
        확인. 상단 경로 전체는 뚜렷하게 왼쪽(y_left ~= 0.6)인데, 로봇에서
        가장 먼(피팅에 안 쓰이는) 점 하나에 노이즈로 반대 부호가 섞여 있는
        상황 — fit_points=5로 앞쪽 5개만 쓰므로 이 노이즈 점은 피팅에
        영향을 주면 안 된다."""
        path_points = [
            (1.0, 0.58), (1.12, 0.60), (1.24, 0.61), (1.36, 0.59), (1.48, 0.62),
            (1.60, -0.9),  # 노이즈 — 훨씬 먼 점, fit_points=5 밖
        ]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3, fit_points=5)
        extension = out[:-len(path_points)]
        assert len(extension) > 0
        for x, y in extension[1:]:
            assert y > 0.0  # 노이즈에 흔들려 음수(반대쪽)로 튀지 않아야 함

    def test_extension_end_uses_fitted_line_not_raw_first_point(self):
        # 여러 점(y_left=0.5,0.7,0.9,1.1,1.3 -> slope=1.0)에 대한 최소제곱
        # 직선 피팅으로 목표점(x_end에서의 y)을 구하고, 그 목표점과 로봇
        # 원점(0,0)을 잇는 직선으로 연장하는지 확인. 원본 path_points[0]의
        # y_left를 그대로 쓰는 게 아니라 피팅값을 쓰므로(노이즈 완화),
        # extension이 (0,0) -> (x_end, fitted_y_end) 직선 위에 있어야 한다.
        path_points = [(1.0, 0.5), (1.2, 0.7), (1.4, 0.9), (1.6, 1.1), (1.8, 1.3)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)

        xs = np.array([p[0] for p in path_points])
        ys = np.array([p[1] for p in path_points])
        slope, intercept = np.polyfit(xs, ys, 1)
        x_end = path_points[0][0]
        y_end = slope * x_end + intercept

        extension = out[:-len(path_points)]
        assert len(extension) > 0
        for x, y in extension:
            # (0,0) -> (x_end, y_end) 직선 위: y/x == y_end/x_end (x=0 지점 제외).
            if x > 1e-9:
                assert y / x == pytest.approx(y_end / x_end, abs=1e-6)

    def test_single_point_extends_straight_ahead_keeping_y(self):
        # 방향을 구할 점이 하나뿐이면 y_left를 유지한 채 로봇까지 직진.
        path_points = [(1.0, 0.4)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)

        assert out[-1] == path_points[0]
        assert out[0] == (pytest.approx(0.0, abs=1e-9), pytest.approx(0.0, abs=1e-9))

    def test_points_stay_monotonically_ordered_by_x_forward(self):
        path_points = [(1.0, 0.5), (1.12, 0.5)]
        out = extend_path_to_robot(path_points, 0.12, min_gap_m=0.3)
        xs = [p[0] for p in out]
        assert xs == sorted(xs)


# ---------------------------------------------------------------------------
# bridge_interior_gaps(): 여름 미션에서 트랙 중간의 큰 segmentation 공백
# (자갈 구간 등)을 truncate_at_large_gap()처럼 그 지점에서 끊지 않고, 앞뒤
# 구간을 각각 최소제곱 직선으로 피팅해 이어붙이는 폴백(2026-09-05, 사용자
# 요청 — 항목4/5).
# ---------------------------------------------------------------------------

class TestBridgeInteriorGaps:
    MPP = 0.05

    def test_noop_when_no_large_gap(self):
        path_points = [(0.05 + i * self.MPP, 0.0) for i in range(10)]
        out = bridge_interior_gaps(path_points, self.MPP, max_gap_rows=5)
        np.testing.assert_allclose(
            [p[0] for p in out], [p[0] for p in path_points])
        np.testing.assert_allclose(
            [p[1] for p in out], [p[1] for p in path_points])

    def test_noop_when_empty(self):
        assert bridge_interior_gaps([], self.MPP, max_gap_rows=5) == []

    def test_connects_before_and_after_a_large_interior_gap(self):
        """항목5: 중간 마스크 공백(자갈 구간) 앞뒤 경로가 끊기지 않고
        이어지는지 확인 — 앞쪽 구간과 뒤쪽 구간이 결과에 모두 남아있고,
        그 사이(gap 구간)에 x_forward가 채워진 점들이 생겨야 한다."""
        # 로봇 앞(0.05~0.5m) 정상 트랙, 0.5~1.5m 자갈(공백), 1.5~2.0m
        # 다시 트랙(왼쪽으로 살짝 틀어짐).
        before = [(round(0.05 + i * self.MPP, 6), 0.0) for i in range(10)]  # 0.05..0.50
        after = [(round(1.5 + i * self.MPP, 6), 0.3) for i in range(10)]    # 1.50..1.95
        path_points = before + after

        out = bridge_interior_gaps(path_points, self.MPP, max_gap_rows=5)

        # 원래 앞/뒤 구간 점들이 모두 살아남아야 한다(끊기지 않음).
        out_xs = {round(x, 6) for x, _ in out}
        for x, _ in before + after:
            assert x in out_xs
        # gap 구간(0.5~1.5m 사이) 안에도 새로 채워진 점이 있어야 한다.
        bridged = [(x, y) for x, y in out if 0.5 < x < 1.5]
        assert len(bridged) > 0
        # 이어붙인 구간의 y_left는 0.0(before)과 0.3(after) 사이여야 한다
        # (양쪽 밖으로 벗어나는 극단적 오버슈트가 없어야 함).
        for x, y in bridged:
            assert -1e-6 <= y <= 0.3 + 1e-6

    def test_gap_right_after_robot_falls_back_to_truncation(self):
        """로봇 바로 앞(피팅에 쓸 앞쪽 점이 1개 이하)에서 큰 gap이 나면
        bridge_interior_gaps()는 못 잇고 그 지점에서 끊는다 — 이 영역은
        extend_path_to_robot()의 몫이므로 중복 처리하지 않는다."""
        path_points = [(0.05, 0.0)] + [(round(1.5 + i * self.MPP, 6), 0.0) for i in range(10)]
        out = bridge_interior_gaps(path_points, self.MPP, max_gap_rows=5)
        assert out == [(0.05, 0.0)]


# ---------------------------------------------------------------------------
# FlatDriveNode의 hold_last_path_sec — 화면 전체에서 segmentation이 완전히
# 사라졌을 때(_latest_mask 자체가 없거나 path_points가 완전히 빈 상태),
# odom/IMU 연동 없이 마지막 정상 경로를 시간 상한 안에서만 재사용하는
# 안전장치(2026-09-05, 사용자 요청 — 항목6/7/8).
# ---------------------------------------------------------------------------

class TestHoldLastPath:
    @staticmethod
    def _make_node():
        from dolbotz.drive_area.flat_drive import FlatDriveNode
        return FlatDriveNode()

    def test_holds_last_path_within_timeout(self):
        """항목6/항목: 완전 미검출 시 직전 경로가 제한 시간 동안 유지되는지 확인."""
        rclpy.init()
        try:
            node = self._make_node()
            try:
                node._hold_last_path_sec = 1.0
                node._last_good_path_points = [(1.0, 0.2), (1.2, 0.25)]
                node._last_good_path_time = time.monotonic()

                held = node._get_held_path_points()
                assert held == [(1.0, 0.2), (1.2, 0.25)]
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_stops_after_timeout_elapsed(self):
        """항목7: 제한 시간이 지나면 안전 정지(빈 경로)하는지 확인."""
        rclpy.init()
        try:
            node = self._make_node()
            try:
                node._hold_last_path_sec = 1.0
                node._last_good_path_points = [(1.0, 0.2), (1.2, 0.25)]
                # 타임아웃(1.0초)보다 확실히 이전 시각으로 세팅.
                node._last_good_path_time = time.monotonic() - 5.0

                held = node._get_held_path_points()
                assert held == []
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_publish_held_or_empty_path_emits_empty_when_disabled(self):
        """hold_last_path_sec=0.0(기본, 다른 미션)이면
        _publish_held_or_empty_path()가 마지막 경로를 재사용하지 않고 빈
        Path를 발행해야 한다 — 여름 미션 전용 기능이 다른 미션 기본 동작을
        바꾸지 않아야 함(항목9). 빈 Path라도 반드시 뭔가는 발행해서
        path_relay_node가 매 프레임 콜백을 받게 하는 게 핵심(발행 자체가
        아예 끊기면 소비 측이 옛 목표를 계속 쫓아가는 문제, 사용자 지적)."""
        rclpy.init()
        try:
            node = self._make_node()
            try:
                assert node._hold_last_path_sec == 0.0  # 기본값(다른 미션) 확인
                node._last_good_path_points = [(1.0, 0.2), (1.2, 0.25)]
                node._last_good_path_time = time.monotonic()

                published = []
                node._path_pub.publish = lambda msg: published.append(msg)

                header = Header()
                header.frame_id = 'camera_color_optical_frame'
                node._publish_held_or_empty_path(header)

                assert len(published) == 1
                assert published[0].poses == []
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_publish_held_or_empty_path_reuses_last_path_when_enabled(self):
        """hold_last_path_sec>0(여름 미션)이고 타임아웃 이내면
        _publish_held_or_empty_path()가 빈 Path 대신 마지막 정상 경로를
        재발행해야 한다."""
        rclpy.init()
        try:
            node = self._make_node()
            try:
                node._hold_last_path_sec = 1.0
                node._last_good_path_points = [(1.0, 0.2), (1.2, 0.25)]
                node._last_good_path_time = time.monotonic()

                published = []
                node._path_pub.publish = lambda msg: published.append(msg)

                header = Header()
                header.frame_id = 'camera_color_optical_frame'
                node._publish_held_or_empty_path(header)

                assert len(published) == 1
                assert len(published[0].poses) > 0
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_default_hold_last_path_sec_is_zero(self):
        """다른 미션(파라미터 오버라이드 없음)의 기본값이 0.0(비활성)인지
        확인 — 여름 미션에서만 launch로 1.0을 켠다(항목9)."""
        rclpy.init()
        try:
            node = self._make_node()
            try:
                assert node.get_parameter('hold_last_path_sec').value == 0.0
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()
