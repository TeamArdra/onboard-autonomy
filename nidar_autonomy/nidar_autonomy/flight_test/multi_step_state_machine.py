"""Pure Python state machine for the mock-only, multi-step Flight Test
scenarios (Test 1, Test 2, ...) -- no rclpy dependency, same split as
hover_test_state_machine.py vs. hover_test_node.py.

Generalizes hover_test_state_machine.py's single-scenario shape
(idle -> arming -> taking_off -> hovering -> landing -> complete) into an
arbitrary-length ordered sequence: idle -> arming -> executing -> landing
-> complete, where "executing" covers however many steps the selected
scenario has (multi_step_test_node.py tracks which step index it's on;
this class only tracks the coarse phase, same division of responsibility
hover_test_state_machine.py already uses).

DELIBERATE DIFFERENCE from hover_test_state_machine.py: this machine has
a distinct "failed" state, separate from "aborted". "aborted" stays
reserved for an operator-issued ABORT (this repo's CLAUDE.md Hard Safety
Rule 2 -- "abort always wins", universal, no-questions-asked). "failed"
is for the scenario's own execution going wrong on its own (e.g. an
individual step raising an unexpected FlightCommandError) -- the NIDAR
Air Mouse master-task spec this scenario family was built for is explicit
that "Failures must be represented explicitly. Do not silently continue
after a failed action" -- collapsing that into "aborted" would make it
indistinguishable from an operator action after the fact (e.g. in a log
or the GCS UI), which is exactly the ambiguity a distinct state avoids.
Both "failed" and "aborted" are terminal and restartable (a fresh "start"
from either returns to "arming"), same restartability posture as
hover_test_state_machine.py's "complete"/"aborted" -- this state machine
only ever drives MockFlightController, so (per that module's own
docstring) there is no real-vehicle hazard in allowing an immediate
restart.
"""

from __future__ import annotations

MULTI_STEP_STATES = (
    "idle",
    "arming",
    "executing",
    "landing",
    "complete",
    "aborted",
    "failed",
)

_RESTARTABLE_STATES = ("idle", "complete", "aborted", "failed")
_IN_PROGRESS_STATES = ("arming", "executing", "landing")


class MultiStepTestStateMachine:
    def __init__(self) -> None:
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    def handle_start(self) -> str:
        """Begin a new scenario: idle/complete/aborted/failed -> arming.
        No-op from any in-progress state, so a repeated/duplicate "start"
        can never re-trigger a run already underway."""
        if self._state in _RESTARTABLE_STATES:
            self._state = "arming"
        return self._state

    def handle_armed(self) -> str:
        """The mock controller confirmed arm() (and the node has issued
        the implicit takeoff-to-baseline-altitude that always precedes
        step 0): arming -> executing. No-op elsewhere."""
        if self._state == "arming":
            self._state = "executing"
        return self._state

    def handle_arm_failed(self) -> str:
        """arm() itself raised (e.g. double-arm): arming -> failed. Not
        "aborted" -- see module docstring: this is the scenario failing
        on its own, not an operator action."""
        if self._state == "arming":
            self._state = "failed"
        return self._state

    def handle_all_steps_done(self) -> str:
        """Every step in the selected scenario's ordered list has
        arrived/completed: executing -> landing. No-op elsewhere."""
        if self._state == "executing":
            self._state = "landing"
        return self._state

    def handle_step_failed(self) -> str:
        """The action handler for the current step raised
        (FlightCommandError or an unrecognized action name): executing ->
        failed. No-op elsewhere -- must not fabricate a transition from a
        stray/duplicate failure signal outside "executing"."""
        if self._state == "executing":
            self._state = "failed"
        return self._state

    def handle_landed(self) -> str:
        """The mock controller's land() target-seek has arrived and
        auto-disarmed: landing -> complete. No-op elsewhere."""
        if self._state == "landing":
            self._state = "complete"
        return self._state

    def handle_abort(self) -> str:
        """Universal abort -- any in-progress state unconditionally moves
        to "aborted", regardless of which step or phase it was in. No-op
        from idle/complete/aborted/failed -- nothing in progress to
        abort."""
        if self._state in _IN_PROGRESS_STATES:
            self._state = "aborted"
        return self._state
