"""`FlightCommandInterface` implementation backed by mavros -- arming via
the existing, already-reviewed `flight_command.FlightCommandClient`,
motion via `setpoint_client.SetpointClient` (see that module's docstring
for the full control-mode/failsafe/abort design).

This class only orchestrates between the two -- it owns no mavros
subscriptions/publishers/services itself, matching the existing pattern
where `mission_state_node.py`/`test_mission_node.py` own construction of
both clients and inject them here. `arm()`/`disarm()` still delegate
purely to `FlightCommandClient`, unchanged from before this file
implemented real setpoints -- this remains the ONLY code in the repo
permitted to call mavros's arming service (Hard Safety Rule 1), reached
either directly (`arm`/`disarm`) or indirectly (`abort()`'s
`on_ground_after_abort` callback, once `SetpointClient`'s own abort
sequence reports it is safe to do so -- see that module's docstring).

This module does not import `flight_command.py`/`setpoint_client.py` at
runtime (only under `typing.TYPE_CHECKING`), so it stays importable and
unit-testable (with stubs in place of the real clients) without a ROS
environment available -- same posture as every other non-ROS-required
module in this package. In practice, though, this specific class has no
dedicated unit tests in this repo (see `setpoint_client.py`'s docstring
for why): it is thin orchestration over two already-tested/reviewed
pieces (`FlightCommandClient`'s arming logic, `SetpointClient`'s
publish/abort logic), and the actual proof this works is the bench/SITL
validation this repo's Hard Safety Rule 3 requires before real flight,
not a unit test against a ROS environment this sandbox doesn't have.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple

from .flight_command_interface import (
    FlightCommandError,
    FlightCommandInterface,
    InvalidCommandError,
    NotArmedError,
    validate_finite,
)

if TYPE_CHECKING:
    from .flight_command import ArmingResult, FlightCommandClient
    from .setpoint_client import SetpointClient

# Same defaults as MockFlightController, for the same reasons documented
# there (arena is <=15m x 15m indoor; 3m is a sane bench/arena ceiling).
_DEFAULT_MAX_SPEED_MPS = 2.0
_DEFAULT_MAX_ALTITUDE_M = 3.0
_BOUND_EPSILON = 1e-9


class ArmCommandFailed(FlightCommandError):
    """Raised when the underlying `FlightCommandClient` reports an
    arm/disarm attempt that did not both succeed (FCU accepted it) and
    get confirmed (`/mavros/state` actually reflected it) -- see
    `flight_command.ArmingResult`. Note `FlightCommandClient.arm()` can
    also raise `arming_guard.ArmRejected` directly (a local precondition
    refusal, before ever reaching mavros) -- that propagates unchanged
    through this class rather than being wrapped, since it is already an
    established, reviewed exception type in this repo."""


class TelemetryUnavailableError(FlightCommandError):
    """Raised by a read-only state property, or a command that needs a
    current position internally (e.g. `takeoff`, `hold`), when the
    underlying `SetpointClient` cannot answer with confidence -- no
    `/mavros/local_position/pose` message has been received yet. This is
    a real, momentary startup gap (or a genuine telemetry loss), not a
    stub -- callers must handle it rather than receiving a fabricated
    zero/placeholder value, same rule this class already applied to
    `armed` before any setpoint code existed here."""


class MAVROSFlightController(FlightCommandInterface):
    def __init__(
        self,
        client: "FlightCommandClient",
        setpoints: "SetpointClient",
        *,
        max_speed_mps: float = _DEFAULT_MAX_SPEED_MPS,
        max_altitude_m: float = _DEFAULT_MAX_ALTITUDE_M,
    ) -> None:
        if max_speed_mps <= 0 or max_altitude_m <= 0:
            raise InvalidCommandError("max_speed_mps and max_altitude_m must both be positive")
        self._client = client
        self._setpoints = setpoints
        self._max_speed_mps = max_speed_mps
        self._max_altitude_m = max_altitude_m

    # -- FlightCommandInterface: read-only state -----------------------

    @property
    def armed(self) -> bool:
        armed = self._client.is_armed
        if armed is None:
            raise TelemetryUnavailableError(
                "armed state unknown: no /mavros/state message has been "
                "received yet by the underlying FlightCommandClient."
            )
        return armed

    @property
    def position(self) -> Tuple[float, float, float]:
        return self._setpoints.position

    @property
    def velocity(self) -> Tuple[float, float, float]:
        # setpoint_client.py raises this exact TelemetryUnavailableError
        # type directly (imported from this module) -- nothing to
        # catch/rewrap here.
        return self._setpoints.velocity

    # -- FlightCommandInterface: arming, unchanged from before real setpoints --

    def arm(self) -> None:
        self._raise_if_not_confirmed(self._client.arm())

    def disarm(self) -> None:
        self._raise_if_not_confirmed(self._client.disarm())

    def _raise_if_not_confirmed(self, result: "ArmingResult") -> None:
        if not (result.success and result.state_confirmed):
            raise ArmCommandFailed(result.message)

    # -- FlightCommandInterface: motion ------------------------------------

    def abort(self) -> None:
        """Cancel whatever setpoint is active and preempt with the
        hold -> controlled-descent -> ready-to-disarm sequence
        `setpoint_client.py`/`abort_sequence.py` implement (see their
        docstrings and `INTEGRATION_CHECKPOINTS.md` Checkpoint 7). This
        call itself only flags the request and returns immediately --
        the interface's "must not require a further tick() call to take
        effect" requirement refers to PREEMPTION being immediate (any
        prior command is abandoned as of this call), not that a physical
        landing can complete synchronously; it plays out over the
        `SetpointClient`'s own publish timer. Callable regardless of
        current armed state, per the interface contract."""
        self._setpoints.request_abort()

    def takeoff(self, height: float) -> None:
        self._require_armed()
        validate_finite(height=height)
        if height <= 0 or height > self._max_altitude_m + _BOUND_EPSILON:
            raise InvalidCommandError(
                f"takeoff height {height}m must be in (0, {self._max_altitude_m}]m"
            )
        x, y, _z = self._setpoints.position
        self._setpoints.set_position_target(x, y, height)

    def land(self) -> None:
        """Command a descent to the ground at the current (x, y).
        Deliberately does NOT itself disarm on reaching z~=0 -- a
        setpoint merely arriving at 0 is not proof of an actual
        touchdown. Real touchdown/disarm is handled by PX4's own landing
        detector + ground-idle auto-disarm, which
        `state_machine.handle_fcu_disarmed()` already watches for -- same
        reasoning this class already applied to `abort()`'s DISARM step,
        which instead waits for a real height reading near the ground."""
        self._require_armed()
        x, y, _z = self._setpoints.position
        self._setpoints.set_position_target(x, y, 0.0)

    def hold(self) -> None:
        self._require_armed()
        self._setpoints.hold()  # raises TelemetryUnavailableError itself if position unknown

    def set_position(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        self._require_armed()
        validate_finite(x=x, y=y, z=z, yaw=yaw)
        self._setpoints.set_position_target(x, y, z, yaw)

    def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: Optional[float] = None,
    ) -> None:
        self._require_armed()
        validate_finite(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        speed = (vx * vx + vy * vy + vz * vz) ** 0.5
        if speed > self._max_speed_mps + _BOUND_EPSILON:
            raise InvalidCommandError(
                f"requested speed {speed:.3f} m/s exceeds max_speed_mps="
                f"{self._max_speed_mps} -- rejected, not clamped"
            )
        self._setpoints.set_velocity_target(vx, vy, vz, yaw_rate)

    def set_yaw(self, yaw: float) -> None:
        self._require_armed()
        validate_finite(yaw=yaw)
        self._setpoints.set_yaw(yaw)  # raises TelemetryUnavailableError itself if position unknown

    # -- internal -----------------------------------------------------------

    def _require_armed(self) -> None:
        # Reuses this class's own `armed` property (which already raises
        # TelemetryUnavailableError on unknown state) rather than
        # duplicating that check -- an unknown armed state must refuse a
        # motion command exactly as confidently as a known-disarmed one.
        if not self.armed:
            raise NotArmedError("Cannot issue a motion command: vehicle is not armed.")
