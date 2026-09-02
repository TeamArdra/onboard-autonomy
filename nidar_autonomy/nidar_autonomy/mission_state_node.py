"""Owns and publishes /mission/state. Listens ONLY to the sanitized
internal topic from command_node (never the raw /gcs/command) -- see
that node's docstring and this repo's CLAUDE.md Hard Safety Rule 5.
The actual state-transition logic lives in state_machine.py (kept
ROS-free so it's unit-testable); this node is just the ROS plumbing
around it.

IMPORTANT LIMITATION (2026-08-27, Phase 0): "start" moves the state
directly to "entering" and nothing further ever moves it to "searching"
or "exiting" -- those transitions are supposed to be driven by real
events from the exploration/path-planning subsystem (e.g. "we've left
the entry cell" -> searching, "time/coverage budget hit, heading back"
-> exiting), which doesn't exist yet. Deliberately NOT faking those
transitions with a timer here (the way custom-gcs/sim/ does, for testing
purposes) -- that would make this node lie about what the real drone is
actually doing, which defeats the point of it existing. Extend this once
the exploration subsystem exists and can report its own phase
transitions.

CHECKPOINT 3/4 WIRING (2026-09-02): this node now owns the
FlightCommandClient and is the one place that turns a validated GCS
command into a real ARM/DISARM attempt -- see arm_trigger.py for the
(pure, tested) decision logic and CHECKPOINT/INTEGRATION_CHECKPOINTS.md
Checkpoints 3/4. "start" only ever attempts ARM as a direct consequence
of the state machine actually accepting it (idle -> entering); "abort"
always attempts DISARM, regardless of current state, per this repo's
CLAUDE.md Hard Safety Rules 1/2. Neither attempt is retried here --
arm()/disarm() already own their own bounded retry policy (see
flight_command.py); this node just logs the outcome and never crashes on
a rejected/failed attempt.

Threading note: _attempt_arm()/_attempt_disarm() are dispatched onto a
background thread (never called inline from the subscription callback).
flight_command.py's arm()/disarm() block for up to several seconds
waiting on the FCU; running that inline here would stall this node's
single-threaded executor (blocking the 2Hz /mission/state publisher and
all other callbacks) for that whole window. The FlightCommandClient is
constructed with already_spinning=True for the same reason: it must not
call rclpy.spin_once/spin_until_future_complete itself, since this node
is already being spun by an executor elsewhere -- see flight_command.py's
already_spinning docstring for the deadlock/timeout hazard that avoids.
A lock serializes arm/disarm attempts so a start immediately followed by
an abort can't race two concurrent FCU requests.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .arm_trigger import should_attempt_arm, should_attempt_disarm
from .arming_guard import ArmRejected
from .flight_command import FlightCommandClient
from .state_machine import MissionStateMachine
from .topics import MISSION_STATE_TOPIC, VALIDATED_COMMAND_TOPIC

_PUBLISH_RATE_HZ = 2.0


class MissionStateNode(Node):
    def __init__(self, flight_client: FlightCommandClient | None = None) -> None:
        super().__init__("mission_state_node")
        self._machine = MissionStateMachine()
        self._publisher = self.create_publisher(String, MISSION_STATE_TOPIC, 10)
        self._subscription = self.create_subscription(
            String, VALIDATED_COMMAND_TOPIC, self._on_validated_command, 10
        )
        self._timer = self.create_timer(1.0 / _PUBLISH_RATE_HZ, self._publish_state)
        self._flight = (
            flight_client
            if flight_client is not None
            else FlightCommandClient(self, already_spinning=True)
        )
        self._flight_lock = threading.Lock()
        self.get_logger().info(f"Starting in state: {self._machine.state!r}")

    def _on_validated_command(self, msg: String) -> None:
        previous = self._machine.state
        command = msg.data
        new_state = self._machine.handle_command(command)

        if new_state == previous and command == "start":
            self.get_logger().warning(
                f"Got 'start' while already in state {previous!r}; ignoring "
                "(a real mission can only be started from 'idle')."
            )
        elif new_state != previous:
            self.get_logger().info(f"Mission state: {previous!r} -> {new_state!r}")

        if should_attempt_arm(command, previous, new_state):
            threading.Thread(target=self._attempt_arm, daemon=True).start()
        if should_attempt_disarm(command):
            threading.Thread(target=self._attempt_disarm, daemon=True).start()

    def _attempt_arm(self) -> None:
        with self._flight_lock:
            try:
                result = self._flight.arm()
            except ArmRejected as exc:
                self.get_logger().error(
                    f"[mission_state_node] Checkpoint 3: ARM refused before "
                    f"reaching the FCU: {exc}"
                )
                return
        self.get_logger().info(f"[mission_state_node] Checkpoint 3: ARM attempt: {result}")

    def _attempt_disarm(self) -> None:
        with self._flight_lock:
            result = self._flight.disarm()
        self.get_logger().info(f"[mission_state_node] Checkpoint 4: DISARM attempt: {result}")

    def _publish_state(self) -> None:
        self._publisher.publish(String(data=self._machine.state))


def main() -> None:
    rclpy.init()
    node = MissionStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
