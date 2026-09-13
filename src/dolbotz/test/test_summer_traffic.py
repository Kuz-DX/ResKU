"""summer_traffic의 ROS 비의존 판정 로직 단위 테스트."""

import pytest

from dolbotz.missions.summer_traffic import (  # type: ignore[import]
    passes_spatial_filters,
    pick_best_state,
)
from dolbotz.utils.state_debouncer import ConsecutiveStateDebouncer


LABEL_TO_STATE = {'red': 'stop', 'green': 'go'}


class TestPickBestState:
    def test_selects_highest_confidence_known_label(self):
        detections = [('red', 0.81), ('green', 0.92), ('other', 0.99)]
        assert pick_best_state(detections, 0.8, LABEL_TO_STATE) == 'go'

    def test_returns_unknown_below_threshold(self):
        assert pick_best_state([('red', 0.79)], 0.8, LABEL_TO_STATE) == 'unknown'


class TestSpatialFilters:
    def test_accepts_box_on_roi_and_area_boundaries(self):
        # center_y=60, area=100: 두 조건 모두 경계값은 허용한다.
        assert passes_spatial_filters((0.0, 55.0, 10.0, 65.0), 100, 0.6, 100)

    def test_rejects_box_below_roi(self):
        assert not passes_spatial_filters((0.0, 56.0, 10.0, 66.0), 100, 0.6, 100)

    def test_rejects_small_box(self):
        assert not passes_spatial_filters((0.0, 50.0, 9.0, 60.0), 100, 0.6, 100)

    @pytest.mark.parametrize('label', ['red', 'green'])
    def test_filter_is_class_agnostic(self, label):
        bbox = (0.0, 70.0, 20.0, 90.0)
        assert label in LABEL_TO_STATE
        assert not passes_spatial_filters(bbox, 100, 0.6, 100)


class TestConsecutiveStateDebouncer:
    def test_confirms_after_required_consecutive_frames(self):
        debouncer = ConsecutiveStateDebouncer(3)
        assert debouncer.update('stop') is None
        assert debouncer.update('stop') is None
        assert debouncer.update('stop') == 'stop'

    def test_unknown_resets_pending_history(self):
        debouncer = ConsecutiveStateDebouncer(3)
        assert debouncer.update('go') is None
        assert debouncer.update('go') is None
        assert debouncer.update('unknown') is None
        assert debouncer.update('go') is None
        assert debouncer.update('go') is None
        assert debouncer.update('go') == 'go'

    def test_state_change_requires_a_new_consecutive_run(self):
        debouncer = ConsecutiveStateDebouncer(3)
        assert debouncer.update('stop') is None
        assert debouncer.update('stop') is None
        assert debouncer.update('go') is None
        assert debouncer.update('go') is None
        assert debouncer.update('go') == 'go'

    def test_rejects_non_positive_confirm_frames(self):
        with pytest.raises(ValueError):
            ConsecutiveStateDebouncer(0)
