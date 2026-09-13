"""Pure Python state machine for the mock-only Hover Test scenario, with no
rclpy dependency -- kept separate from hover_test_node.py so it's testable
without a running ROS context, same split as state_machine.py vs.
mission_state_node.py.

This is a much simpler machine than MissionStateMachine: there is no real
vehicle here, no ARM/DISARM confirmation from a real FCU, and no
Checkpoint-gated flight capability -- every transition below is driven
entirely by MockFlightController's deterministic, tick()-based arithmetic
(see hover_test_node.py). The states mirror the obvious phases of a single
hover-and-land scenario: idle -> arming -> taking_off -> hovering ->
landing -> complete, with "aborted" reachable from any in-progress state.
"""

from __future__ import annotations

HOVER_TEST_STATES = (
    "idle",
    "arming",
    "taking_off",
    "hovering",
    "landing",
    "complete",
    "aborted",
)

_RESTARTABLE_STATES = ("idle", "complete", "aborted")


class HoverTestStateMachine:
    def __init__(self) -> None:
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    def handle_start(self) -> str:
        """Begin a new hover scenario: idle/complete/aborted -> arming.
        No-op from any other state (already mid-scenario) -- mirrors
        MissionStateMachine.handle_command("start")'s "only accept start
        from the right state" pattern, so a repeated/duplicate "start"
        can never re-trigger an in-progress run.

        DELIBERATE SIMPLIFICATION vs. mission_state_node.py: restarting
        is accepted from "complete" AND "aborted" here, not just from a
        confirmed-ground-reset-equivalent state. MissionStateMachine
        requires handle_ground_reset_confirmed() -- real FCU telemetry
        confirming a real vehicle actually disarmed -- before "aborted"
        can become restartable again, because guessing wrong there could
        mean re-arming a live aircraft that never actually became safe.
        This state machine has no such hazard to guard against: it only
        ever drives MockFlightController, an in-memory simulation with no
        relationship to any real vehicle, so there is nothing unsafe about
        letting a fresh "start" immediately re-arm the mock from
        "aborted" or "complete". This simplification is specific to this
        being mock-only execution -- it must NOT be copied into any future
        real-hardware hover-test state machine without re-deriving
        whether it's still safe there."""
        if self._state in _RESTARTABLE_STATES:
            self._state = "arming"
        return self._state

    def handle_armed(self) -> str:
        """The mock controller confirmed arm() succeeded: arming ->
        taking_off. No-op elsewhere -- an arm confirmation arriving
        outside "arming" (e.g. late/duplicate) must not fabricate a
        transition."""
        if self._state == "arming":
            self._state = "taking_off"
        return self._state

    def handle_arm_failed(self) -> str:
        """The mock controller's arm() attempt failed (e.g.
        InvalidCommandError from double-arming): arming -> aborted.
        No-op elsewhere, same defensive shape as every other transition
        in this class."""
        if self._state == "arming":
            self._state = "aborted"
        return self._state

    def handle_airborne(self) -> str:
        """The mock vehicle has actually arrived at the target altitude
        (zero velocity, z within tolerance of target_altitude_m -- see
        hover_test_node.py's tick loop): taking_off -> hovering. No-op
        elsewhere -- in particular, must not be called synchronously
        right after issuing takeoff(), since takeoff() only sets a target
        that tick() converges toward over multiple ticks."""
        if self._state == "taking_off":
            self._state = "hovering"
        return self._state

    def handle_hover_duration_elapsed(self) -> str:
        """The configured hover duration has elapsed: hovering ->
        landing. No-op elsewhere -- a stray/duplicate elapsed signal
        outside "hovering" must not fabricate a transition."""
        if self._state == "hovering":
            self._state = "landing"
        return self._state

    def handle_landed(self) -> str:
        """The mock controller's land() target-seek has arrived and
        auto-disarmed (see MockFlightController._arrive_at_target's
        landing handling): landing -> complete. No-op elsewhere."""
        if self._state == "landing":
            self._state = "complete"
        return self._state

    def handle_abort(self) -> str:
        """Universal abort -- mirrors state_machine.py's "abort always
        wins" principle (this repo's CLAUDE.md Hard Safety Rule 2). Any
        state except idle/complete/aborted (i.e. any state where a
        scenario is actually in progress) unconditionally moves to
        "aborted", regardless of which in-progress state it was. No-op
        from idle/complete/aborted -- there is nothing in progress to
        abort, and re-entering "aborted" from "aborted"/"complete" would
        be a meaningless transition, not a real preemption."""
        if self._state not in _RESTARTABLE_STATES:
            self._state = "aborted"
        return self._state
