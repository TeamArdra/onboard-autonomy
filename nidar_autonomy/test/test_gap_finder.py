"""Tests for gap_finder.py -- the reactive "which direction has open
space right now" logic behind the test-mission's hover/move/stop loop.

Pure logic, no ROS required (this module never imports rclpy).
"""
import math

import pytest

from nidar_autonomy.gap_finder import GapResult, find_open_direction

_N = 360
_INC = 2 * math.pi / _N
_ANGLE_MIN = -math.pi
_MIN_CLEARANCE = 0.3
_MAX_STEP = 0.5
_MAX_RANGE = 8.0


def _uniform_scan(value: float, n: int = _N) -> list[float]:
    return [value] * n


def _index_to_angle(index: int) -> float:
    return _ANGLE_MIN + index * _INC


def _find(ranges, **overrides):
    kwargs = dict(
        angle_min=_ANGLE_MIN,
        angle_increment=_INC,
        min_clearance_m=_MIN_CLEARANCE,
        max_step_m=_MAX_STEP,
        max_valid_range_m=_MAX_RANGE,
    )
    kwargs.update(overrides)
    return find_open_direction(ranges, **kwargs)


# -- open corridor: every ray clear -------------------------------------------


def test_open_corridor_picks_front_direction_and_full_step():
    result = _find(_uniform_scan(5.0))
    assert result is not None
    # Whole circle is one arc -- its "middle" wraps to ~0 (front), not any
    # particular arbitrary index.
    assert abs(result.direction_rad) < _INC
    assert result.step_m == pytest.approx(_MAX_STEP)
    assert result.arc_width_rad == pytest.approx(2 * math.pi, rel=1e-3)


# -- dead end: nothing clears the threshold -----------------------------------


def test_dead_end_returns_none():
    # Every ray reports 0.5m -- below max_step_m + min_clearance_m (0.8m).
    result = _find(_uniform_scan(0.5))
    assert result is None


def test_all_out_of_range_returns_none():
    result = _find(_uniform_scan(math.inf))
    assert result is None


# -- off-axis gap: only a narrow arc is open, not centered on index 0 --------


def test_off_axis_gap_is_found_at_its_own_bearing():
    ranges = _uniform_scan(0.5)
    gap_start, gap_len = 90, 10
    for i in range(gap_start, gap_start + gap_len):
        ranges[i] = 5.0

    result = _find(ranges)
    assert result is not None

    expected_mid_angle = _index_to_angle(gap_start + (gap_len - 1) / 2.0)
    assert result.direction_rad == pytest.approx(expected_mid_angle, abs=1e-6)
    assert result.step_m == pytest.approx(_MAX_STEP)
    assert result.arc_width_rad == pytest.approx(gap_len * _INC)


def test_widest_arc_wins_over_a_narrower_closer_one():
    ranges = _uniform_scan(0.5)
    # A narrow-but-usable gap at 0-4...
    for i in range(0, 4):
        ranges[i] = 6.0
    # ...and a wider one elsewhere must win.
    for i in range(200, 220):
        ranges[i] = 6.0

    result = _find(ranges)
    assert result is not None
    expected_mid_angle = _index_to_angle(200 + (20 - 1) / 2.0)
    assert result.direction_rad == pytest.approx(expected_mid_angle, abs=1e-6)


# -- wraparound: an open arc spanning the 0/360 seam is one arc, not two -----


def test_wraparound_arc_is_merged_not_split():
    ranges = _uniform_scan(0.5)
    # Open near the end and near the start -- physically contiguous on a
    # 360-degree sweep.
    for i in list(range(350, 360)) + list(range(0, 10)):
        ranges[i] = 5.0

    result = _find(ranges)
    assert result is not None
    assert result.arc_width_rad == pytest.approx(20 * _INC)


# -- step distance is always the full max_step_m for any usable ray --------
#
# A ray only counts as usable if it already clears max_step_m +
# min_clearance_m (see the module docstring), so moving the full
# max_step_m always leaves at least min_clearance_m of buffer -- step_m
# is never anything other than max_step_m.


def test_step_is_max_step_even_for_a_ray_just_above_threshold():
    ranges = _uniform_scan(0.5)
    for i in range(90, 100):
        ranges[i] = _MAX_STEP + _MIN_CLEARANCE + 0.01  # just barely usable
    result = _find(ranges)
    assert result is not None
    assert result.step_m == pytest.approx(_MAX_STEP)


# -- stale / malformed input --------------------------------------------------


def test_empty_scan_returns_none():
    assert _find([]) is None


def test_nan_readings_are_never_treated_as_open():
    ranges = [math.nan] * _N
    assert _find(ranges) is None


def test_negative_and_zero_readings_are_never_treated_as_open():
    ranges = _uniform_scan(0.0)
    ranges[10] = -1.0
    assert _find(ranges) is None


def test_reading_beyond_max_valid_range_is_not_treated_as_open():
    result = _find(_uniform_scan(_MAX_RANGE + 1.0))
    assert result is None


# -- return type -------------------------------------------------------------


def test_result_is_the_documented_dataclass():
    result = _find(_uniform_scan(5.0))
    assert isinstance(result, GapResult)
