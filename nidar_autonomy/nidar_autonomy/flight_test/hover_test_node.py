"""READ-ONLY-WITH-RESPECT-TO-REAL-HARDWARE PROOF-OF-CONCEPT.

MOCK-ONLY EXECUTION: this node constructs and drives only
`MockFlightController` -- it never imports `mavros_msgs`, never
constructs `flight_command.FlightCommandClient` or
`mavros_flight_controller.MAVROSFlightController`, and never contacts a
real Pixhawk. Real hardware execution for takeoff/hover/land does not
exist in this repo and is deliberately out of scope here -- see
`mavros_flight_controller.MAVROSFlightController`'s `NotImplementedError`s
and `CHECKPOINT/INTEGRATION_CHECKPOINTS.md` Checkpoint 5 (autonomous
takeoff: NOT STARTED, requires a separate safety review). This node only
ever ARMs/DISARMS a simulated, in-memory vehicle with no relationship to
the real aircraft.

Same "pure logic + thin node" split as state_machine.py /
mission_state_node.py: the actual scenario transitions live in
hover_test_state_machine.py (no rclpy dependency, fully unit-testable);
this file is only the ROS plumbing and the MockFlightController-driven
timing loop around it.

Command routing: this node listens on the same VALIDATED_COMMAND_TOPIC
and MISSION_SELECT_TOPIC as mission_state_node.py, but only ever acts on
a "start" when the latest mission selection is exactly
`{"mission_id": "flight_test", "scenario_id": "hover"}` -- any other
selection (including "main_nidar" or none received yet) is logged and
ignored, mirroring command_node.py's reject-and-log style and
mission_state_node.py's new mission-select guard. "abort" is
unconditional with respect to mission selection (universal abort, same
principle as this repo's CLAUDE.md Hard Safety Rule 2) -- it always acts
as long as this node's own scenario isn't already idle/complete/aborted.
"""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..flight_command_interface import FlightCommandError
from ..mock_flight_controller import MockFlightController
from ..topics import FLIGHT_TEST_STATUS_TOPIC, MISSION_SELECT_TOPIC, VALIDATED_COMMAND_TOPIC
from .hover_test_state_machine import HoverTestStateMachine

SCHEMA_VERSION = 1

_HOVER_MISSION_SELECTION = {"mission_id": "flight_test", "scenario_id": "hover"}

# How close z must be to target_altitude_m, and velocity to zero, to count
# as "arrived" -- generous relative to MockFlightController's own default
# position_tolerance_m (0.02m) so the tick loop's own timer-rate rounding
# can never make arrival flicker undetected.
_ARRIVAL_ALTITUDE_TOLERANCE_M = 0.05


class HoverTestNode(Node):
    def __init__(self) -> None:
        super().__init__("hover_test_node")
        self.declare_parameter("target_altitude_m", 1.0)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("tick_rate_hz", 10.0)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self._target_altitude_m = get("target_altitude_m")
        self._duration_s = get("duration_s")
        self._tick_rate_hz = get("tick_rate_hz")

        self._controller = MockFlightController()
        self._machine = HoverTestStateMachine()
        self._latest_mission_select: Optional[dict] = None
        self._hover_started_at: Optional[float] = None
        self._ever_armed = False

        self.create_subscription(
            String, VALIDATED_COMMAND_TOPIC, self._on_validated_command, 10
        )
        self.create_subscription(
            String, MISSION_SELECT_TOPIC, self._on_mission_select, 10
        )

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._status_pub = self.create_publisher(String, FLIGHT_TEST_STATUS_TOPIC, status_qos)

        self.create_timer(1.0 / self._tick_rate_hz, self._on_tick)
        self.create_timer(1.0, self._on_status_tick)

        self.get_logger().info(
            "hover_test_node: mock-only proof-of-concept, target_altitude_m="
            f"{self._target_altitude_m} duration_s={self._duration_s} "
            f"tick_rate_hz={self._tick_rate_hz} -- never touches mavros/Pixhawk."
        )

    # -- inputs ------------------------------------------------------------

    def _on_mission_select(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Malformed message on {MISSION_SELECT_TOPIC}: {msg.data!r} "
                f"({exc}). Ignoring -- keeping previous selection."
            )
            return
        if not isinstance(parsed, dict):
            self.get_logger().warning(
                f"Malformed message on {MISSION_SELECT_TOPIC}: {msg.data!r} "
                "did not decode to a JSON object. Ignoring -- keeping "
                "previous selection."
            )
            return
        self._latest_mission_select = parsed

    def _on_validated_command(self, msg: String) -> None:
        command = msg.data
        if command == "start":
            self._on_start()
        elif command == "abort":
            self._on_abort()
        else:
            # VALIDATED_COMMAND_TOPIC is only ever populated with "start"/
            # "abort" by command_node.py, but this node never trusts that
            # blindly -- see this repo's CLAUDE.md Hard Safety Rule 5.
            self.get_logger().warning(
                f"hover_test_node: rejected {command!r} on "
                f"{VALIDATED_COMMAND_TOPIC} -- not a valid command. Ignoring."
            )

    def _on_start(self) -> None:
        if self._latest_mission_select != _HOVER_MISSION_SELECTION:
            self.get_logger().info(
                "hover_test_node: rejected 'start' -- current mission "
                f"selection is {self._latest_mission_select!r}, not "
                f"{_HOVER_MISSION_SELECTION!r}. Ignoring."
            )
            return

        previous = self._machine.state
        new_state = self._machine.handle_start()
        if new_state == previous:
            self.get_logger().info(
                f"hover_test_node: got 'start' while already in state "
                f"{previous!r}; ignoring."
            )
            return
        self.get_logger().info(f"hover_test_node: {previous!r} -> {new_state!r}")

        try:
            self._controller.arm()
        except FlightCommandError as exc:
            self.get_logger().error(f"hover_test_node: mock ARM failed: {exc}")
            prev2 = self._machine.state
            new2 = self._machine.handle_arm_failed()
            self.get_logger().warning(f"hover_test_node: {prev2!r} -> {new2!r}")
            return

        self._ever_armed = True
        prev2 = self._machine.state
        new2 = self._machine.handle_armed()
        self.get_logger().info(f"hover_test_node: {prev2!r} -> {new2!r}")
        # takeoff() only sets a target position -- the mock converges
        # toward it over subsequent tick() calls (see _on_tick); it does
        # NOT arrive synchronously, so the state machine deliberately does
        # NOT move to "hovering" here.
        self._controller.takeoff(self._target_altitude_m)
        self._hover_started_at = None

    def _on_abort(self) -> None:
        """Unconditional with respect to mission selection -- abort always
        acts regardless of what MISSION_SELECT_TOPIC currently says, as
        long as this node's own scenario isn't already idle/complete/
        aborted (mirrors mission_state_node.py's "abort always leads to
        disarm" pattern)."""
        if self._machine.state in ("idle", "complete", "aborted"):
            return
        self._controller.abort()
        self._controller.disarm()
        previous = self._machine.state
        new_state = self._machine.handle_abort()
        self.get_logger().warning(f"hover_test_node: abort -- {previous!r} -> {new_state!r}")

    # -- timing loop ---------------------------------------------------------

    def _on_tick(self) -> None:
        if self._controller.armed:
            self._controller.tick(1.0 / self._tick_rate_hz)

        state = self._machine.state
        if state == "taking_off":
            self._tick_taking_off()
        elif state == "hovering":
            self._tick_hovering()
        elif state == "landing":
            self._tick_landing()

    def _tick_taking_off(self) -> None:
        vx, vy, vz = self._controller.velocity
        _x, _y, z = self._controller.position
        arrived_stopped = (vx, vy, vz) == (0.0, 0.0, 0.0)
        arrived_altitude = abs(z - self._target_altitude_m) < _ARRIVAL_ALTITUDE_TOLERANCE_M
        if not (arrived_stopped and arrived_altitude):
            return
        previous = self._machine.state
        new_state = self._machine.handle_airborne()
        self._hover_started_at = time.monotonic()
        self.get_logger().info(f"hover_test_node: {previous!r} -> {new_state!r}")

    def _tick_hovering(self) -> None:
        if self._hover_started_at is None:
            return
        if time.monotonic() - self._hover_started_at < self._duration_s:
            return
        previous = self._machine.state
        new_state = self._machine.handle_hover_duration_elapsed()
        self.get_logger().info(f"hover_test_node: {previous!r} -> {new_state!r}")
        self._controller.land()

    def _tick_landing(self) -> None:
        if self._controller.armed is not False:
            return
        previous = self._machine.state
        new_state = self._machine.handle_landed()
        self.get_logger().info(f"hover_test_node: {previous!r} -> {new_state!r}")

    # -- status publication --------------------------------------------------

    def _on_status_tick(self) -> None:
        state = self._machine.state
        elapsed_hover_s = 0.0
        if state == "hovering" and self._hover_started_at is not None:
            elapsed_hover_s = time.monotonic() - self._hover_started_at

        if self._ever_armed:
            x, y, z = self._controller.position
            current_position = [x, y, z]
            current_altitude_m = z
        else:
            current_position = None
            current_altitude_m = None

        payload = {
            "schema_version": SCHEMA_VERSION,
            "scenario": "hover",
            "state": state,
            "target_altitude_m": self._target_altitude_m,
            "current_altitude_m": current_altitude_m,
            "current_position": current_position,
            "duration_s": self._duration_s,
            "elapsed_hover_s": elapsed_hover_s,
            "armed": self._controller.armed,
            "execution_mode": "mock",
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = HoverTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
