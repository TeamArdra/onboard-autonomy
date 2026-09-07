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

    def handle_fcu_disarmed(self) -> str:
        """Called when /mavros/state reports the FCU has disarmed on its
        own (e.g. ArduCopter's ground-idle auto-disarm after arming with
        no throttle/setpoint stream ever sent) -- NOT as a result of our
        own commanded DISARM, which already moves this state machine via
        handle_command("abort") before the disarm is even requested (see
        mission_state_node.py). Only "entering" implies an arm attempt
        this node itself made and is still relying on being armed, so
        only "entering" transitions here (to "aborted"); every other
        state is a no-op. Without this, an unsolicited FCU disarm would
        leave the reported state stuck on "entering" (i.e. an
        active/armed mission) indefinitely, which is exactly the kind of
        state machine lying about drone reality this module's docstring
        already says not to do.

        Deliberately does NOT go any further than "aborted" -- see
        handle_ground_reset_confirmed() for why an *unsolicited* disarm
        must never, by itself, make the mission restartable again."""
        return self._to_aborted_if_entering()

    def handle_arm_failed(self) -> str:
        """Called when an ARM attempt this node itself made did not
        cleanly succeed -- refused before ever reaching the FCU
        (arming_guard.ArmRejected), rejected by the FCU, or accepted but
        not confirmed via /mavros/state (see flight_command.ArmingResult).
        Same reasoning as handle_fcu_disarmed(): "entering" must not keep
        claiming an active/armed mission when the vehicle never actually
        armed, so this moves "entering" -> "aborted" and is a no-op
        everywhere else. mission_state_node.py always follows this with a
        forced DISARM attempt, so the vehicle ends up in a confirmed,
        known-safe (disarmed) state either way."""
        return self._to_aborted_if_entering()

    def _to_aborted_if_entering(self) -> str:
        if self._state == "entering":
            self._state = "aborted"
        return self._state

    def handle_ground_reset_confirmed(self) -> str:
        """Called when a DISARM *this node itself commanded* (as a direct
        consequence of "abort", or of handle_arm_failed()'s forced
        disarm) is confirmed via the real /mavros/state.armed value --
        never from a timer, a guess, or the unsolicited-disarm path
        (handle_fcu_disarmed() never calls this). This is the ONLY way
        "aborted" ever becomes "idle" again -- a bare "start" command
        can't do it (see test_start_while_aborted_is_a_noop), and an
        unconfirmed/failed commanded disarm leaves the state at "aborted"
        indefinitely, forcing a human/physical intervention rather than
        guessing the vehicle is safe. No-op in every state but "aborted",
        so a stray or duplicate disarm confirmation while already
        idle/entering can never fabricate a transition.

        Scope note: today, "aborted" is only ever reached while the
        vehicle is disarmed-or-being-disarmed on the bench (no takeoff/
        setpoint code exists yet -- see this repo's CLAUDE.md "Current
        Project Phase"), so a confirmed disarm here is also, in practice,
        a confirmed *ground* state. Once real flight exists (Phase 8 /
        Checkpoint 7+), this assumption must be revisited: an in-flight
        abort must not simply mean "immediately disarm" (see
        flight_command_interface.FlightCommandInterface.abort()'s
        docstring), and this reset logic will need a real
        airborne/grounded distinction, not just "is it disarmed"."""
        if self._state == "aborted":
            self._state = "idle"
        return self._state
