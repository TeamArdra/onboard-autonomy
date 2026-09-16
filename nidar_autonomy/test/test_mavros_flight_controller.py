"""Exercises MAVROSFlightController against small in-process fakes in
place of the real flight_command.FlightCommandClient and
setpoint_client.SetpointClient -- deliberately does not import rclpy or
mavros_msgs, so this suite runs without a ROS environment, same as the
rest of this package's tests. See setpoint_client.py's own module
docstring for why the REAL ROS-facing implementation has no test file of
its own in this repo (no rclpy/mavros installed in this environment) --
this file is the coverage that IS possible here: that
MAVROSFlightController correctly orchestrates between the two client
interfaces (armed-state gating, validation, delegation), independent of
whichever concrete client implementations are behind them.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import pytest

from nidar_autonomy.flight_command_interface import InvalidCommandError, NotArmedError
from nidar_autonomy.mavros_flight_controller import (
    ArmCommandFailed,
    MAVROSFlightController,
    TelemetryUnavailableError,
)


@dataclass
class FakeArmingResult:
    requested: bool
    success: bool
    state_confirmed: bool
    message: str = "fake result"


class FakeFlightCommandClient:
    """Stands in for flight_command.FlightCommandClient's public surface
    that MAVROSFlightController actually uses."""

    def __init__(self, is_armed: Optional[bool] = None) -> None:
        self.is_armed = is_armed
        self.calls: List[str] = []
        self.next_arm_result: Optional[FakeArmingResult] = None
        self.next_disarm_result: Optional[FakeArmingResult] = None

    def arm(self) -> FakeArmingResult:
        self.calls.append("arm")
        result = self.next_arm_result or FakeArmingResult(
            requested=True, success=True, state_confirmed=True
        )
        if result.success and result.state_confirmed:
            self.is_armed = True
        return result

    def disarm(self) -> FakeArmingResult:
        self.calls.append("disarm")
        result = self.next_disarm_result or FakeArmingResult(
            requested=False, success=True, state_confirmed=True
        )
        if result.success and result.state_confirmed:
            self.is_armed = False
        return result


class FakeSetpointClient:
    """Stands in for setpoint_client.SetpointClient's public surface --
    raises the SAME TelemetryUnavailableError type the real implementation
    does (imported from mavros_flight_controller.py, not redefined here),
    since that's the actual contract MAVROSFlightController relies on."""

    def __init__(
        self,
        position: Optional[Tuple[float, float, float]] = None,
        velocity: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        self._position = position
        self._velocity = velocity
        self.calls: List[tuple] = []

    @property
    def position(self) -> Tuple[float, float, float]:
        if self._position is None:
            raise TelemetryUnavailableError("fake: no position yet")
        return self._position

    @property
    def velocity(self) -> Tuple[float, float, float]:
        if self._velocity is None:
            raise TelemetryUnavailableError("fake: no velocity yet")
        return self._velocity

    def set_position_target(self, x, y, z, yaw=None) -> None:
        self.calls.append(("set_position_target", x, y, z, yaw))

    def set_velocity_target(self, vx, vy, vz, yaw_rate=None) -> None:
        self.calls.append(("set_velocity_target", vx, vy, vz, yaw_rate))

    def hold(self) -> None:
        if self._position is None:
            raise TelemetryUnavailableError("fake: no position yet")
        self.calls.append(("hold",))

    def set_yaw(self, yaw) -> None:
        if self._position is None:
            raise TelemetryUnavailableError("fake: no position yet")
        self.calls.append(("set_yaw", yaw))

    def request_abort(self) -> None:
        self.calls.append(("request_abort",))


def _controller(is_armed=True, position=None, velocity=None, **kwargs):
    client = FakeFlightCommandClient(is_armed=is_armed)
    setpoints = FakeSetpointClient(position=position, velocity=velocity)
    return MAVROSFlightController(client, setpoints, **kwargs), client, setpoints


# -- construction -------------------------------------------------------------


@pytest.mark.parametrize("kwargs", [dict(max_speed_mps=0.0), dict(max_altitude_m=0.0), dict(max_speed_mps=-1.0)])
def test_rejects_non_positive_bounds(kwargs):
    client = FakeFlightCommandClient(is_armed=False)
    setpoints = FakeSetpointClient()
    with pytest.raises(InvalidCommandError):
        MAVROSFlightController(client, setpoints, **kwargs)


# -- arm/disarm delegate to the real client, nothing new -------------------


def test_arm_delegates_to_client_and_succeeds():
    controller, client, _ = _controller(is_armed=False)
    controller.arm()
    assert client.calls == ["arm"]
    assert controller.armed is True


def test_disarm_delegates_to_client_and_succeeds():
    controller, client, _ = _controller(is_armed=True)
    controller.disarm()
    assert client.calls == ["disarm"]
    assert controller.armed is False


def test_arm_raises_when_client_reports_unconfirmed():
    controller, client, _ = _controller(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True, success=True, state_confirmed=False, message="mismatch"
    )
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_client_reports_rejected():
    controller, client, _ = _controller(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False, success=False, state_confirmed=False, message="rejected"
    )
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


# -- full (success, state_confirmed) matrix, both directions ---------------
#
# flight_command.ArmingResult can legitimately land on any of the four
# (success, state_confirmed) combinations (see _set_arming's "NOTE:
# /mavros/state nonetheless shows the requested state -- treat with
# caution" branch for how success=False, state_confirmed=True arises for
# real) -- MAVROSFlightController._raise_if_not_confirmed requires BOTH
# to be True, so every other combination, for both arm() and disarm(),
# must raise ArmCommandFailed. Only (True, True) -- already covered above
# -- must not.


def test_arm_raises_when_client_reports_outright_rejected():
    # success=False, state_confirmed=False: the FCU rejected the request
    # and the state agrees it never armed.
    controller, client, _ = _controller(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True, success=False, state_confirmed=False, message="rejected"
    )
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_client_reports_unconfirmed():
    # success=True, state_confirmed=False: FCU accepted the disarm but
    # /mavros/state never caught up within the confirm timeout.
    controller, client, _ = _controller(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False, success=True, state_confirmed=False, message="mismatch"
    )
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


def test_arm_raises_when_rejected_but_state_confirms_anyway():
    # success=False, state_confirmed=True: the FCU reported rejection but
    # /mavros/state nonetheless shows the requested state. The wrapper
    # must still treat this as a failure -- it must not trust the state
    # confirmation over an explicit FCU rejection.
    controller, client, _ = _controller(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True,
        success=False,
        state_confirmed=True,
        message="rejected but state confirms anyway",
    )
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_rejected_but_state_confirms_anyway():
    # Same success=False/state_confirmed=True mismatch, for disarm().
    controller, client, _ = _controller(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False,
        success=False,
        state_confirmed=True,
        message="rejected but state confirms anyway",
    )
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


def test_arm_precondition_rejection_propagates_unchanged():
    # FlightCommandClient.arm() can raise arming_guard.ArmRejected before
    # ever calling mavros -- that must propagate through this class
    # unchanged, not be swallowed or wrapped.
    class RejectingClient(FakeFlightCommandClient):
        def arm(self):
            raise RuntimeError("stands in for arming_guard.ArmRejected")

    controller = MAVROSFlightController(RejectingClient(is_armed=False), FakeSetpointClient())
    with pytest.raises(RuntimeError):
        controller.arm()


# -- read-only state ---------------------------------------------------


def test_armed_raises_when_client_state_unknown():
    controller, _, _ = _controller(is_armed=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.armed  # noqa: B018


def test_position_raises_telemetry_unavailable_when_unset():
    controller, _, _ = _controller(position=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.position  # noqa: B018


def test_position_returns_the_setpoint_clients_value():
    controller, _, _ = _controller(position=(1.0, 2.0, 3.0))
    assert controller.position == (1.0, 2.0, 3.0)


def test_velocity_raises_telemetry_unavailable_when_unset():
    controller, _, _ = _controller(velocity=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.velocity  # noqa: B018


def test_velocity_returns_the_setpoint_clients_value():
    controller, _, _ = _controller(velocity=(0.1, 0.2, 0.3))
    assert controller.velocity == (0.1, 0.2, 0.3)


# -- abort: callable regardless of armed state, always delegates -----------


@pytest.mark.parametrize("is_armed", [True, False, None])
def test_abort_always_delegates_regardless_of_armed_state(is_armed):
    controller, _, setpoints = _controller(is_armed=is_armed)
    controller.abort()  # must not raise even when armed state is False/unknown
    assert setpoints.calls == [("request_abort",)]


# -- takeoff -------------------------------------------------------------------


def test_takeoff_requires_armed():
    controller, _, _ = _controller(is_armed=False, position=(0.0, 0.0, 0.0))
    with pytest.raises(NotArmedError):
        controller.takeoff(1.0)


def test_takeoff_requires_a_known_position():
    controller, _, _ = _controller(is_armed=True, position=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.takeoff(1.0)


@pytest.mark.parametrize("height", [0.0, -1.0, 3.01])
def test_takeoff_rejects_out_of_bounds_height(height):
    controller, _, _ = _controller(is_armed=True, position=(0.0, 0.0, 0.0), max_altitude_m=3.0)
    with pytest.raises(InvalidCommandError):
        controller.takeoff(height)


def test_takeoff_commands_a_position_target_above_current_xy():
    controller, _, setpoints = _controller(is_armed=True, position=(1.0, 2.0, 0.0))
    controller.takeoff(0.5)
    assert setpoints.calls == [("set_position_target", 1.0, 2.0, 0.5, None)]


# -- land -----------------------------------------------------------------------


def test_land_requires_armed():
    controller, _, _ = _controller(is_armed=False, position=(0.0, 0.0, 1.0))
    with pytest.raises(NotArmedError):
        controller.land()


def test_land_commands_descent_to_current_xy_at_ground():
    controller, _, setpoints = _controller(is_armed=True, position=(1.0, 2.0, 0.5))
    controller.land()
    assert setpoints.calls == [("set_position_target", 1.0, 2.0, 0.0, None)]


# -- hold -----------------------------------------------------------------------


def test_hold_requires_armed():
    controller, _, _ = _controller(is_armed=False, position=(0.0, 0.0, 1.0))
    with pytest.raises(NotArmedError):
        controller.hold()


def test_hold_delegates_to_setpoint_client():
    controller, _, setpoints = _controller(is_armed=True, position=(1.0, 2.0, 0.5))
    controller.hold()
    assert setpoints.calls == [("hold",)]


def test_hold_propagates_telemetry_unavailable():
    controller, _, _ = _controller(is_armed=True, position=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.hold()


# -- set_position ---------------------------------------------------------------


def test_set_position_requires_armed():
    controller, _, _ = _controller(is_armed=False)
    with pytest.raises(NotArmedError):
        controller.set_position(1.0, 2.0, 3.0)


def test_set_position_rejects_non_finite_values():
    controller, _, _ = _controller(is_armed=True)
    with pytest.raises(InvalidCommandError):
        controller.set_position(float("nan"), 0.0, 0.0)


def test_set_position_delegates_with_yaw():
    controller, _, setpoints = _controller(is_armed=True)
    controller.set_position(1.0, 2.0, 3.0, yaw=0.5)
    assert setpoints.calls == [("set_position_target", 1.0, 2.0, 3.0, 0.5)]


# -- set_velocity -----------------------------------------------------------------


def test_set_velocity_requires_armed():
    controller, _, _ = _controller(is_armed=False)
    with pytest.raises(NotArmedError):
        controller.set_velocity(1.0, 0.0, 0.0)


def test_set_velocity_rejects_non_finite_values():
    controller, _, _ = _controller(is_armed=True)
    with pytest.raises(InvalidCommandError):
        controller.set_velocity(float("inf"), 0.0, 0.0)


def test_set_velocity_rejects_speed_over_bound():
    controller, _, _ = _controller(is_armed=True, max_speed_mps=2.0)
    with pytest.raises(InvalidCommandError):
        controller.set_velocity(3.0, 0.0, 0.0)


def test_set_velocity_accepts_speed_at_exactly_the_bound():
    controller, _, setpoints = _controller(is_armed=True, max_speed_mps=2.0)
    controller.set_velocity(2.0, 0.0, 0.0)
    assert setpoints.calls == [("set_velocity_target", 2.0, 0.0, 0.0, None)]


def test_set_velocity_delegates_with_yaw_rate():
    controller, _, setpoints = _controller(is_armed=True)
    controller.set_velocity(0.5, 0.0, 0.0, yaw_rate=0.1)
    assert setpoints.calls == [("set_velocity_target", 0.5, 0.0, 0.0, 0.1)]


# -- set_yaw ----------------------------------------------------------------------


def test_set_yaw_requires_armed():
    controller, _, _ = _controller(is_armed=False, position=(0.0, 0.0, 0.0))
    with pytest.raises(NotArmedError):
        controller.set_yaw(1.0)


def test_set_yaw_rejects_non_finite():
    controller, _, _ = _controller(is_armed=True, position=(0.0, 0.0, 0.0))
    with pytest.raises(InvalidCommandError):
        controller.set_yaw(float("nan"))


def test_set_yaw_delegates_to_setpoint_client():
    controller, _, setpoints = _controller(is_armed=True, position=(0.0, 0.0, 0.0))
    controller.set_yaw(1.2)
    assert setpoints.calls == [("set_yaw", 1.2)]


def test_set_yaw_propagates_telemetry_unavailable():
    controller, _, _ = _controller(is_armed=True, position=None)
    with pytest.raises(TelemetryUnavailableError):
        controller.set_yaw(1.0)
