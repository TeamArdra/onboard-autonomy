"""Pure mission state machine logic, with no ROS/rclpy dependency -- kept
separate from mission_state_node.py so it's testable without a running
ROS context, mirroring the pattern custom-gcs/sim/rosbridge_sim/mission.py
already uses for the same reason.

See mission_state_node.py's module docstring for the current limitation:
only idle/entering/aborted are reachable right now, because nothing
upstream exists yet to report the searching->exiting->complete
transitions for real.
"""

from __future__ import annotations

from .topics import VALID_COMMANDS


class InvalidCommandError(ValueError):
    pass


class MissionStateMachine:
    def __init__(self) -> None:
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    def handle_command(self, command: str) -> str:
        """Apply a command (must already be validated as "start" or
        "abort" -- see command_node.py). Returns the resulting state.
        Raises InvalidCommandError for anything else, as a defense-in-
        depth check, not the primary validation point."""
        if command not in VALID_COMMANDS:
            raise InvalidCommandError(f"{command!r} is not a valid command")

        if command == "abort":
            self._state = "aborted"
            return self._state

        # command == "start"
        if self._state != "idle":
            # No-op: a mission can only be started from idle. Caller decides
            # whether/how to log this; we just refuse the transition.
            return self._state

        self._state = "entering"
        return self._state
