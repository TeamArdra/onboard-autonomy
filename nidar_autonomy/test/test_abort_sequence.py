"""Tests for abort_sequence.py -- the pure decision logic behind
in-flight ABORT (hold -> controlled descend -> disarm, never an instant
mid-air disarm). No ROS required.
"""
import pytest

from nidar_autonomy.abort_sequence import AbortAction, AbortSequenceConfig, next_action

_CFG = AbortSequenceConfig(hold_duration_s=1.0, descend_speed_mps=0.3, ground_height_m=0.05)


def test_holds_immediately_after_abort_while_airborne():
    assert next_action(0.0, current_height_m=1.0, config=_CFG) == AbortAction.HOLD


def test_still_holds_just_before_hold_duration_elapses():
    assert next_action(0.99, current_height_m=1.0, config=_CFG) == AbortAction.HOLD


def test_descends_once_hold_duration_has_elapsed():
    assert next_action(1.0, current_height_m=1.0, config=_CFG) == AbortAction.DESCEND
    assert next_action(5.0, current_height_m=0.5, config=_CFG) == AbortAction.DESCEND


def test_disarms_once_at_ground_height_regardless_of_elapsed_time():
    assert next_action(0.0, current_height_m=0.05, config=_CFG) == AbortAction.DISARM
    assert next_action(0.0, current_height_m=0.0, config=_CFG) == AbortAction.DISARM


def test_disarms_immediately_even_during_what_would_be_the_hold_phase():
    # Abort fired while already very low (e.g. mid-landing) -- must not
    # be forced through a pointless hold-in-air phase first.
    assert next_action(0.0, current_height_m=0.02, config=_CFG) == AbortAction.DISARM


def test_never_gets_stuck_if_height_never_quite_reaches_ground():
    # A height reading that plateaus just above ground_height_m keeps
    # producing DESCEND forever, not some terminal "stuck" state.
    for elapsed in (2.0, 10.0, 100.0):
        assert next_action(elapsed, current_height_m=0.06, config=_CFG) == AbortAction.DESCEND


def test_default_config_is_usable():
    # Exercises the dataclass default without an explicit config -- also
    # documents the shipped defaults are sane (positive, non-trivial).
    action = next_action(0.0, current_height_m=1.0)
    assert action == AbortAction.HOLD


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(hold_duration_s=-0.1),
        dict(descend_speed_mps=0.0),
        dict(descend_speed_mps=-0.1),
        dict(ground_height_m=0.0),
        dict(ground_height_m=-0.1),
    ],
)
def test_rejects_invalid_config(kwargs):
    with pytest.raises(ValueError):
        AbortSequenceConfig(**kwargs)
