import math

import pytest

from nidar_autonomy.flight_command_interface import InvalidCommandError, NotArmedError
from nidar_autonomy.mock_flight_controller import MockFlightController


# -- arm/disarm ----------------------------------------------------------


def test_starts_disarmed_at_origin():
    c = MockFlightController()
    assert c.armed is False
    assert c.position == (0.0, 0.0, 0.0)
    assert c.velocity == (0.0, 0.0, 0.0)


def test_arm_transitions_to_armed():
    c = MockFlightController()
    c.arm()
    assert c.armed is True


def test_double_arm_raises():
    # Deliberately mirrors the real vehicle's arming_guard refusal to
    # re-arm -- see MockFlightController's class docstring.
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.arm()
    assert c.armed is True  # rejected attempt must not change state


def test_disarm_from_armed():
    c = MockFlightController()
    c.arm()
    c.disarm()
    assert c.armed is False


def test_disarm_when_already_disarmed_is_safe_noop():
    c = MockFlightController()
    c.disarm()  # must not raise
    assert c.armed is False


def test_disarm_zeroes_velocity_and_clears_target():
    c = MockFlightController()
    c.arm()
    c.set_velocity(1.0, 0.0, 0.0)
    c.tick(0.1)
    c.disarm()
    assert c.velocity == (0.0, 0.0, 0.0)
    # re-arming and ticking must not resume the old target/velocity
    c.arm()
    c.tick(1.0)
    assert c.velocity == (0.0, 0.0, 0.0)


# -- "requires armed" for every motion command ----------------------------


@pytest.mark.parametrize(
    "call",
    [
        lambda c: c.set_position(1.0, 0.0, 0.0),
        lambda c: c.set_velocity(0.5, 0.0, 0.0),
        lambda c: c.set_yaw(1.0),
        lambda c: c.takeoff(1.0),
        lambda c: c.hold(),
        lambda c: c.land(),
    ],
)
def test_motion_commands_require_armed(call):
    c = MockFlightController()
    with pytest.raises(NotArmedError):
        call(c)


# -- set_position + tick convergence --------------------------------------


def test_set_position_converges_within_tolerance_and_respects_max_speed():
    max_speed = 2.0
    c = MockFlightController(max_speed_mps=max_speed)
    c.arm()
    c.set_position(5.0, -3.0, 1.0)

    dt = 0.05
    for _ in range(2000):
        c.tick(dt)
        speed = math.sqrt(sum(v * v for v in c.velocity))
        assert speed <= max_speed + 1e-6
        if c.position == (5.0, -3.0, 1.0):
            break

    assert c.position == (5.0, -3.0, 1.0)
    assert c.velocity == (0.0, 0.0, 0.0)


def test_set_position_sets_yaw_on_arrival():
    c = MockFlightController()
    c.arm()
    c.set_position(0.1, 0.0, 0.0, yaw=1.5)
    for _ in range(50):
        c.tick(0.05)
    assert c.yaw == 1.5


def test_repeated_identical_set_position_is_idempotent():
    c = MockFlightController()
    c.arm()
    c.set_position(1.0, 1.0, 1.0)
    c.set_position(1.0, 1.0, 1.0)  # must not raise or misbehave
    for _ in range(200):
        c.tick(0.05)
    assert c.position == (1.0, 1.0, 1.0)


# -- set_velocity ----------------------------------------------------------


def test_set_velocity_moves_position_by_velocity_times_dt():
    c = MockFlightController(max_speed_mps=3.0)
    c.arm()
    c.set_velocity(1.0, 2.0, -0.5)
    c.tick(0.2)
    assert c.position == pytest.approx((0.2, 0.4, -0.1))
    c.tick(0.3)
    assert c.position == pytest.approx((0.5, 1.0, -0.25))


def test_set_velocity_out_of_bounds_is_rejected_not_clamped():
    c = MockFlightController(max_speed_mps=2.0)
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.set_velocity(3.0, 0.0, 0.0)
    # rejected -- velocity must remain unchanged (zero), not clamped down
    assert c.velocity == (0.0, 0.0, 0.0)


def test_set_velocity_at_exactly_the_bound_is_accepted():
    c = MockFlightController(max_speed_mps=2.0)
    c.arm()
    c.set_velocity(2.0, 0.0, 0.0)  # must not raise
    assert c.velocity == (2.0, 0.0, 0.0)


# -- hold -------------------------------------------------------------------


def test_hold_cancels_in_progress_position_seek():
    c = MockFlightController()
    c.arm()
    c.set_position(10.0, 0.0, 0.0)
    c.tick(0.1)
    moved_position = c.position
    assert moved_position != (0.0, 0.0, 0.0)

    c.hold()
    assert c.velocity == (0.0, 0.0, 0.0)
    c.tick(1.0)
    c.tick(1.0)
    assert c.position == moved_position  # unchanged after hold()


def test_hold_cancels_in_progress_velocity_command():
    c = MockFlightController()
    c.arm()
    c.set_velocity(1.0, 0.0, 0.0)
    c.tick(0.1)
    moved_position = c.position

    c.hold()
    c.tick(1.0)
    assert c.position == moved_position


# -- abort --------------------------------------------------------------


def test_abort_zeroes_velocity_synchronously_without_tick():
    c = MockFlightController()
    c.arm()
    c.set_velocity(1.0, 1.0, 0.0)
    c.abort()
    assert c.velocity == (0.0, 0.0, 0.0)  # true before any tick() call


def test_abort_cancels_position_target_synchronously():
    c = MockFlightController()
    c.arm()
    c.set_position(5.0, 0.0, 0.0)
    c.abort()
    c.tick(1.0)
    assert c.position == (0.0, 0.0, 0.0)  # target was cleared, no motion


def test_abort_does_not_disarm():
    # Per INTEGRATION_CHECKPOINTS.md Checkpoint 7: in-flight abort must
    # not simply mean "disarm".
    c = MockFlightController()
    c.arm()
    c.abort()
    assert c.armed is True


def test_abort_is_safe_when_disarmed():
    c = MockFlightController()
    c.abort()  # must not raise even though never armed
    assert c.armed is False


# -- takeoff -----------------------------------------------------------------


def test_takeoff_sets_target_above_current_position():
    c = MockFlightController(max_altitude_m=3.0)
    c.arm()
    c.takeoff(2.0)
    for _ in range(200):
        c.tick(0.05)
    assert c.position == pytest.approx((0.0, 0.0, 2.0))


@pytest.mark.parametrize("height", [0.0, -1.0])
def test_takeoff_rejects_nonpositive_height(height):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.takeoff(height)


def test_takeoff_rejects_excessive_height():
    c = MockFlightController(max_altitude_m=3.0)
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.takeoff(10.0)


def test_takeoff_at_exactly_max_altitude_is_accepted():
    c = MockFlightController(max_altitude_m=3.0)
    c.arm()
    c.takeoff(3.0)  # must not raise


# -- land ---------------------------------------------------------------


def test_land_converges_to_ground_and_auto_disarms():
    c = MockFlightController()
    c.arm()
    c.takeoff(2.0)
    for _ in range(200):
        c.tick(0.05)
    assert c.position == pytest.approx((0.0, 0.0, 2.0))

    c.land()
    for _ in range(200):
        c.tick(0.05)
        if not c.armed:
            break

    assert c.position == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert c.armed is False


def test_tick_after_land_auto_disarm_is_safe_noop():
    c = MockFlightController()
    c.arm()
    c.takeoff(1.0)
    for _ in range(100):
        c.tick(0.05)
    c.land()
    for _ in range(100):
        c.tick(0.05)
    assert c.armed is False

    position_after_land = c.position
    c.tick(1.0)  # must not raise, must not move
    assert c.position == position_after_land
    assert c.velocity == (0.0, 0.0, 0.0)


def test_set_position_to_z_zero_does_not_auto_disarm():
    # Only land()'s target-seek auto-disarms on arrival -- an ordinary
    # set_position that happens to target z=0 must not.
    c = MockFlightController()
    c.arm()
    c.set_position(0.0, 0.0, 0.0)
    c.tick(0.05)
    assert c.armed is True


# -- input validation: NaN/Inf ---------------------------------------------


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_set_position_rejects_nan_and_inf(bad):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.set_position(bad, 0.0, 0.0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_set_velocity_rejects_nan_and_inf(bad):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.set_velocity(bad, 0.0, 0.0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_set_yaw_rejects_nan_and_inf(bad):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.set_yaw(bad)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_takeoff_rejects_nan_and_inf(bad):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.takeoff(bad)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_tick_rejects_nan_and_inf_dt(bad):
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.tick(bad)


def test_tick_rejects_nonpositive_dt():
    c = MockFlightController()
    c.arm()
    with pytest.raises(InvalidCommandError):
        c.tick(0.0)
    with pytest.raises(InvalidCommandError):
        c.tick(-0.1)


def test_tick_when_disarmed_is_a_safe_noop():
    c = MockFlightController()
    c.tick(0.1)  # must not raise even though never armed
    assert c.position == (0.0, 0.0, 0.0)


# -- constructor validation --------------------------------------------


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_speed_mps": 0.0},
        {"max_speed_mps": -1.0},
        {"max_altitude_m": 0.0},
        {"position_tolerance_m": -0.01},
    ],
)
def test_constructor_rejects_nonpositive_bounds(kwargs):
    with pytest.raises(InvalidCommandError):
        MockFlightController(**kwargs)
