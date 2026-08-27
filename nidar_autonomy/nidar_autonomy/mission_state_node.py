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
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .state_machine import MissionStateMachine
from .topics import MISSION_STATE_TOPIC, VALIDATED_COMMAND_TOPIC

_PUBLISH_RATE_HZ = 2.0


class MissionStateNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_state_node")
        self._machine = MissionStateMachine()
        self._publisher = self.create_publisher(String, MISSION_STATE_TOPIC, 10)
        self._subscription = self.create_subscription(
            String, VALIDATED_COMMAND_TOPIC, self._on_validated_command, 10
        )
        self._timer = self.create_timer(1.0 / _PUBLISH_RATE_HZ, self._publish_state)
        self.get_logger().info(f"Starting in state: {self._machine.state!r}")

    def _on_validated_command(self, msg: String) -> None:
        previous = self._machine.state
        new_state = self._machine.handle_command(msg.data)

        if new_state == previous and msg.data == "start":
            self.get_logger().warning(
                f"Got 'start' while already in state {previous!r}; ignoring "
                "(a real mission can only be started from 'idle')."
            )
        elif new_state != previous:
            self.get_logger().info(f"Mission state: {previous!r} -> {new_state!r}")

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
