"""Regression tests for the manual-EE gripper completion thresholds."""

import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'gripper_hold_fin.py'
SPEC = importlib.util.spec_from_file_location('gripper_hold_fin', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_effort_must_strictly_exceed_threshold():
    assert not MODULE.effort_exceeds_threshold(130.0, 130.0)
    assert MODULE.effort_exceeds_threshold(130.01, 130.0)


def test_negative_effort_uses_current_magnitude():
    assert MODULE.effort_exceeds_threshold(-131.0, 130.0)


def test_non_finite_effort_is_rejected():
    assert not MODULE.effort_exceeds_threshold(math.nan, 130.0)
    assert not MODULE.effort_exceeds_threshold(math.inf, 130.0)


def test_position_reaches_threshold_inclusively():
    assert not MODULE.position_reaches_threshold(1.82939, 1.8294)
    assert MODULE.position_reaches_threshold(1.8294, 1.8294)
    assert MODULE.position_reaches_threshold(1.9, 1.8294)


def test_non_finite_position_is_rejected():
    assert not MODULE.position_reaches_threshold(math.nan, 1.8294)
    assert not MODULE.position_reaches_threshold(math.inf, 1.8294)


def test_position_is_primary_and_effort_is_fallback():
    assert MODULE.grasp_is_complete(1.8294, 1.8294, 0.0, 130.0)
    assert MODULE.grasp_is_complete(1.0, 1.8294, 131.0, 130.0)
    assert not MODULE.grasp_is_complete(1.0, 1.8294, 130.0, 130.0)
