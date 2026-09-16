"""Tests for test_mission_policy.py -- the dev-only, repeating
hover/find-open-space/move/stop loop -- run against
`MockFlightController`, exactly the way `mission_simulator.py`'s tests
run the real exploration stack against it.

Two independent clocks are advanced explicitly and separately in these
tests, matching how the real system will actually be composed: the
*controller's* own simulated motion (`controller.tick(dt)`, unchanged
from `test_mock_flight_controller.py`'s existing contract) and the
*policy's* decision-making (`policy.tick(dt, scan)`). A real
`MAVROSFlightController` republishes its own setpoint stream on an
internal ROS timer instead of a manual `tick()` -- only the mock needs
this second call.
"""
import math

import pytest

from nidar_autonomy.mock_flight_controller import MockFlightController
from nidar_autonomy.test_mission_policy import (
    ScanSample,
    TestMissionPolicy,
    TestMissionState,
)

_DT = 0.1
_N = 360
_INC = 2 * math.pi / _N
_ANGLE_MIN = -math.pi
_MAX_ITERS = 2000


def _open_scan(value: float = 5.0) -> ScanSample:
    return ScanSample(ranges=tuple([value] * _N), angle_min=_ANGLE_MIN, angle_increment=_INC)


def _blocked_scan() -> ScanSample:
    return _open_scan(0.1)


def _make_policy(**overrides):
    controller = MockFlightController()
    kwargs = dict(
        takeoff_height_m=0.5,
        hover_duration_s=1.0,
        max_step_m=0.5,
        min_clearance_m=0.3,
        max_valid_range_m=8.0,
    )
    kwargs.update(overrides)
    return TestMissionPolicy(controller, **kwargs), controller


def _advance(policy, controller, dt=_DT, scan=None, n=1):
    for _ in range(n):
        controller.tick(dt)
        policy.tick(dt, scan)


def _run_until(policy, controller, target_state, *, dt=_DT, scan=None, max_iters=_MAX_ITERS):
    for _ in range(max_iters):
        if policy.state == target_state:
            return
        _advance(policy, controller, dt=dt, scan=scan)
    raise AssertionError(f"never reached state {target_state!r}; stuck at {policy.state!r}")


# -- startup -------------------------------------------------------------


def test_start_arms_and_begins_takeoff():
    policy, controller = _make_policy()
    policy.start()
    assert controller.armed
    assert policy.state == TestMissionState.TAKING_OFF


def test_double_start_is_a_noop():
    policy, controller = _make_policy()
    policy.start()
    policy.start()  # must not raise (e.g. re-arm an already-armed mock)
    assert policy.state == TestMissionState.TAKING_OFF


def test_reaches_hovering_after_takeoff():
    policy, controller = _make_policy()
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    assert controller.position[2] == pytest.approx(0.5, abs=0.05)


# -- hovering / gap-finding ------------------------------------------------


def test_stays_hovering_without_a_scan_even_past_the_dwell():
    policy, controller = _make_policy(hover_duration_s=0.2)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    start_pos = controller.position
    _advance(policy, controller, scan=None, n=50)  # far past the 0.2s dwell
    assert policy.state == TestMissionState.HOVERING
    assert controller.position == start_pos


def test_stays_in_place_on_a_dead_end_scan():
    policy, controller = _make_policy(hover_duration_s=0.2)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    start_pos = controller.position
    _advance(policy, controller, scan=_blocked_scan(), n=50)
    assert policy.state == TestMissionState.HOVERING
    assert controller.position == start_pos


def test_moves_toward_open_space_after_dwell():
    policy, controller = _make_policy(hover_duration_s=0.2, max_step_m=0.5)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    start_pos = controller.position

    _advance(policy, controller, scan=_open_scan(), n=3)  # cross the 0.2s dwell
    assert policy.state == TestMissionState.MOVING

    _run_until(policy, controller, TestMissionState.HOVERING, scan=_open_scan())
    moved = math.dist(start_pos[:2], controller.position[:2])
    assert moved == pytest.approx(0.5, abs=0.02)
    # Height must not drift during a horizontal move.
    assert controller.position[2] == pytest.approx(start_pos[2], abs=0.02)


def test_repeats_the_cycle_multiple_times():
    policy, controller = _make_policy(hover_duration_s=0.1, max_step_m=0.3)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)

    positions = [controller.position[:2]]
    for _ in range(3):
        _run_until(policy, controller, TestMissionState.MOVING, scan=_open_scan())
        _run_until(policy, controller, TestMissionState.HOVERING, scan=_open_scan())
        positions.append(controller.position[:2])

    # Every cycle actually went somewhere new (not stuck oscillating on
    # the same spot).
    for a, b in zip(positions, positions[1:]):
        assert math.dist(a, b) == pytest.approx(0.3, abs=0.02)


# -- graceful stop ----------------------------------------------------------


def test_stop_before_start_is_a_noop():
    policy, controller = _make_policy()
    policy.stop()
    assert policy.state == TestMissionState.IDLE
    assert not controller.armed


def test_stop_while_hovering_lands_and_disarms():
    policy, controller = _make_policy()
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)

    policy.stop()
    assert policy.state == TestMissionState.LANDING

    _run_until(policy, controller, TestMissionState.STOPPED)
    assert not controller.armed


def test_stop_while_moving_lands_from_current_position():
    policy, controller = _make_policy(hover_duration_s=0.1)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    _advance(policy, controller, scan=_open_scan(), n=3)  # cross the 0.1s dwell
    assert policy.state == TestMissionState.MOVING

    policy.stop()
    assert policy.state == TestMissionState.LANDING
    _run_until(policy, controller, TestMissionState.STOPPED)
    assert not controller.armed


def test_stop_is_idempotent_once_landing():
    policy, controller = _make_policy()
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    policy.stop()
    policy.stop()  # must not raise or issue a second land()
    assert policy.state == TestMissionState.LANDING


# -- abort --------------------------------------------------------------------


def test_abort_while_hovering_preempts_immediately():
    policy, controller = _make_policy()
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)

    policy.abort()
    assert policy.state == TestMissionState.ABORTED
    assert controller.velocity == (0.0, 0.0, 0.0)


def test_abort_while_moving_cancels_the_in_progress_move():
    policy, controller = _make_policy(hover_duration_s=0.1)
    policy.start()
    _run_until(policy, controller, TestMissionState.HOVERING)
    _advance(policy, controller, scan=_open_scan(), n=3)  # cross the 0.1s dwell
    assert policy.state == TestMissionState.MOVING

    policy.abort()
    assert policy.state == TestMissionState.ABORTED

    pos_at_abort = controller.position
    _advance(policy, controller, scan=_open_scan(), n=20)
    # Nothing further is ever issued again -- position is unchanged and
    # state never leaves ABORTED, even with a wide-open scan available.
    assert controller.position == pos_at_abort
    assert policy.state == TestMissionState.ABORTED


def test_abort_is_reachable_from_taking_off():
    policy, controller = _make_policy()
    policy.start()
    assert policy.state == TestMissionState.TAKING_OFF
    policy.abort()
    assert policy.state == TestMissionState.ABORTED


def test_abort_after_abort_is_a_noop():
    policy, controller = _make_policy()
    policy.start()
    policy.abort()
    policy.abort()
    assert policy.state == TestMissionState.ABORTED


# -- construction validation --------------------------------------------------


def test_rejects_non_positive_takeoff_height():
    controller = MockFlightController()
    with pytest.raises(ValueError):
        TestMissionPolicy(controller, takeoff_height_m=0.0)


def test_rejects_non_positive_hover_duration():
    controller = MockFlightController()
    with pytest.raises(ValueError):
        TestMissionPolicy(controller, hover_duration_s=-1.0)
