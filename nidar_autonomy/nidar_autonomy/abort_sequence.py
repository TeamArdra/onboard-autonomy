"""Pure decision logic for what an in-flight ABORT should do, moment to
moment. No rclpy -- unit-testable without a ROS environment, same split
as `arming_guard.py` vs `flight_command.py` and `state_machine.py` vs
`mission_state_node.py`.

Exists because `FlightCommandInterface.abort()`'s own docstring and
`CHECKPOINT/INTEGRATION_CHECKPOINTS.md` Checkpoint 7 are explicit that
in-flight ABORT must NEVER simply mean an instant mid-air disarm --
that's a crash, not an abort. This module answers, given how long ago
abort was requested and the vehicle's current height, what the setpoint
publisher should be doing RIGHT NOW: keep holding in place to let motion
settle, begin a slow controlled descent, or -- only once genuinely at or
essentially on the ground -- actually disarm.

A pure function of (elapsed time since abort, current height), not a
stateful object -- the caller (`setpoint_client.py`) owns tracking when
abort was requested and re-evaluates this every publish tick, which also
means a height reading that never quite settles below `ground_height_m`
simply keeps producing DESCEND forever rather than the sequence getting
"stuck" in some internal state.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class AbortAction(Enum):
    HOLD = "hold"
    DESCEND = "descend"
    DISARM = "disarm"


@dataclass(frozen=True)
class AbortSequenceConfig:
    # Stabilize in place for this long before starting to descend -- an
    # abort requested mid-move should not turn into an instant descent
    # while there is still significant horizontal velocity to bleed off.
    hold_duration_s: float = 1.0
    # Slow and controlled, not a rushed drop -- this is a safety response,
    # not a race to the ground.
    descend_speed_mps: float = 0.3
    # "Close enough to ground to disarm" -- must be small enough that
    # disarming here can't itself cause a meaningful drop, but not zero
    # (a real rangefinder/EKF height reading essentially never reports
    # exactly 0.0).
    ground_height_m: float = 0.05

    def __post_init__(self) -> None:
        if self.hold_duration_s < 0:
            raise ValueError("hold_duration_s must be >= 0")
        if self.descend_speed_mps <= 0:
            raise ValueError("descend_speed_mps must be positive")
        if self.ground_height_m <= 0:
            raise ValueError("ground_height_m must be positive")


def next_action(
    elapsed_s: float,
    current_height_m: float,
    config: AbortSequenceConfig = AbortSequenceConfig(),
) -> AbortAction:
    """What should be published/done right now, this tick, given `elapsed_s`
    since abort was first requested and the vehicle's `current_height_m`.
    Height is checked before elapsed time on every call (not just after
    the hold phase) so a vehicle that was already very low/landing when
    abort fired disarms immediately, rather than being forced through an
    unnecessary hold phase first."""
    if current_height_m <= config.ground_height_m:
        return AbortAction.DISARM
    if elapsed_s < config.hold_duration_s:
        return AbortAction.HOLD
    return AbortAction.DESCEND
