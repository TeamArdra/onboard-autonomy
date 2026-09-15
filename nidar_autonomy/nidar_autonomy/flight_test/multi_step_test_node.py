"""READ-ONLY-WITH-RESPECT-TO-REAL-HARDWARE PROOF-OF-CONCEPT.

MOCK-ONLY EXECUTION: this node constructs and drives only
`MockFlightController` -- it never imports `mavros_msgs`, never
constructs `flight_command.FlightCommandClient` or
`mavros_flight_controller.MAVROSFlightController`, and never contacts a
real Pixhawk. Exactly the same safety posture as `hover_test_node.py` --
see that module's docstring and `flight_test/README.md` -- this file only
generalizes it from one fixed scenario to an arbitrary ordered list of
steps (`flight_test/scenarios.py`'s `MULTI_STEP_SCENARIOS`), so future
scenarios ("Test 3", "Test 4", ...) are a registry addition, not a new
node.

Command routing: same VALIDATED_COMMAND_TOPIC / MISSION_SELECT_TOPIC
pattern as hover_test_node.py, generalized from a single exact match to
membership in `scenarios.scenario_ids()`. "start" is only acted on when
the latest mission selection is `{"mission_id": "flight_test",
"scenario_id": <a known multi-step scenario id>}` -- anything else
(including "hover", which stays hover_test_node.py's exclusively, or
"main_nidar", or no selection yet) is logged and ignored. "abort" is
unconditional with respect to mission selection, same as every other
node in this repo (Hard Safety Rule 2).

Step execution model: every scenario implicitly begins with arm ->
takeoff to `_BASELINE_ALTITUDE_M` and implicitly ends with land ->
disarm -- these are never named steps in scenarios.py, they are this
node's own fixed bookends, same way hover_test_node.py always does
arm -> takeoff -> hover -> land for its one scenario. Only the actions
*between* those bookends come from the selected scenario's step list.
Movement actions (forward/backward/left/right/sideways) are each a fixed
`_STEP_DISTANCE_M` relative offset from wherever the vehicle is when that
step begins (so "forward" then "backward" returns to the position before
those two steps, matching the intuitive reading of Test 1). "hover"
holds position for `_HOVER_DURATION_S`. "yaw" rotates by a fixed
`_YAW_DELTA_RAD` -- applied instantly (MockFlightController's own
set_yaw() is synchronous, see its docstring), so a yaw step is
considered arrived on the very next tick after being issued.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..flight_command_interface import FlightCommandError
from ..mock_flight_controller import MockFlightController
from ..topics import MISSION_SELECT_TOPIC, MULTI_STEP_TEST_STATUS_TOPIC, VALIDATED_COMMAND_TOPIC
from .multi_step_state_machine import MultiStepTestStateMachine
from .scenarios import MOVE_OFFSETS, MultiStepScenario, ScenarioNotFoundError, get_scenario, scenario_ids

SCHEMA_VERSION = 1

_MISSION_ID = "flight_test"

_BASELINE_ALTITUDE_M = 1.0
_HOVER_DURATION_S = 3.0
_YAW_DELTA_RAD = math.pi / 2

# Re-exported for readability at call sites below -- the actual data
# lives in scenarios.py so it stays importable without rclpy (see that
# module's comment).
_MOVE_OFFSETS = MOVE_OFFSETS

# How close velocity must be to zero to count as "arrived" after a
# set_position seek -- MockFlightController zeroes velocity exactly on
# arrival (see its _arrive_at_target()), so this is a tolerance against
# tick-rate rounding, not a real physical margin.
_ARRIVAL_VELOCITY_TOLERANCE = 1e-6
_ARRIVAL_ALTITUDE_TOLERANCE_M = 0.05


class MultiStepTestNode(Node):
    def __init__(self) -> None:
        super().__init__("multi_step_test_node")
        self.declare_parameter("tick_rate_hz", 10.0)
        self._tick_rate_hz = self.get_parameter("tick_rate_hz").value

        self._controller = MockFlightController()
        self._machine = MultiStepTestStateMachine()
        self._latest_mission_select: Optional[dict] = None

        self._scenario: Optional[MultiStepScenario] = None
        self._phase: str = "idle"  # "idle" | "taking_off" | "step" | "landing"
        self._step_index: int = 0
        self._step_started_at: Optional[float] = None
        self._current_yaw: float = 0.0
        self._ever_armed = False

        self.create_subscription(String, VALIDATED_COMMAND_TOPIC, self._on_validated_command, 10)
        self.create_subscription(String, MISSION_SELECT_TOPIC, self._on_mission_select, 10)

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._status_pub = self.create_publisher(String, MULTI_STEP_TEST_STATUS_TOPIC, status_qos)

        self.create_timer(1.0 / self._tick_rate_hz, self._on_tick)
        self.create_timer(1.0, self._on_status_tick)

        self.get_logger().info(
            "multi_step_test_node: mock-only proof-of-concept, scenarios="
            f"{scenario_ids()!r} -- never touches mavros/Pixhawk."
        )

    # -- inputs --------------------------------------------------------------

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
            self.get_logger().warning(
                f"multi_step_test_node: rejected {command!r} on "
                f"{VALIDATED_COMMAND_TOPIC} -- not a valid command. Ignoring."
            )

    def _selected_scenario_id(self) -> Optional[str]:
        sel = self._latest_mission_select
        if not isinstance(sel, dict) or sel.get("mission_id") != _MISSION_ID:
            return None
        scenario_id = sel.get("scenario_id")
        return scenario_id if isinstance(scenario_id, str) and scenario_id in scenario_ids() else None

    def _on_start(self) -> None:
        scenario_id = self._selected_scenario_id()
        if scenario_id is None:
            self.get_logger().info(
                "multi_step_test_node: rejected 'start' -- current mission "
                f"selection is {self._latest_mission_select!r}, not a "
                f"recognized {_MISSION_ID!r} scenario ({scenario_ids()!r}). "
                "Ignoring."
            )
            return

        previous = self._machine.state
        new_state = self._machine.handle_start()
        if new_state == previous:
            self.get_logger().info(
                f"multi_step_test_node: got 'start' while already in state "
                f"{previous!r}; ignoring."
            )
            return
        self.get_logger().info(f"multi_step_test_node: {previous!r} -> {new_state!r}")

        self._scenario = get_scenario(scenario_id)
        self._step_index = 0
        self._current_yaw = 0.0

        try:
            self._controller.arm()
        except FlightCommandError as exc:
            self.get_logger().error(f"multi_step_test_node: mock ARM failed: {exc}")
            prev2 = self._machine.state
            new2 = self._machine.handle_arm_failed()
            self.get_logger().warning(f"multi_step_test_node: {prev2!r} -> {new2!r}")
            return

        self._ever_armed = True
        prev2 = self._machine.state
        new2 = self._machine.handle_armed()
        self.get_logger().info(f"multi_step_test_node: {prev2!r} -> {new2!r}")
        self._phase = "taking_off"
        self._controller.takeoff(_BASELINE_ALTITUDE_M)

    def _on_abort(self) -> None:
        if self._machine.state in ("idle", "complete", "aborted", "failed"):
            return
        self._controller.abort()
        self._controller.disarm()
        self._phase = "idle"
        previous = self._machine.state
        new_state = self._machine.handle_abort()
        self.get_logger().warning(f"multi_step_test_node: abort -- {previous!r} -> {new_state!r}")

    # -- timing loop -----------------------------------------------------------

    def _on_tick(self) -> None:
        if self._controller.armed:
            self._controller.tick(1.0 / self._tick_rate_hz)

        if self._machine.state != "executing" and self._machine.state != "landing":
            return

        if self._phase == "taking_off":
            self._tick_taking_off()
        elif self._phase == "step":
            self._tick_step()
        elif self._phase == "landing":
            self._tick_landing()

    def _tick_taking_off(self) -> None:
        vx, vy, vz = self._controller.velocity
        _x, _y, z = self._controller.position
        arrived_stopped = (vx, vy, vz) == (0.0, 0.0, 0.0)
        arrived_altitude = abs(z - _BASELINE_ALTITUDE_M) < _ARRIVAL_ALTITUDE_TOLERANCE_M
        if not (arrived_stopped and arrived_altitude):
            return
        self._begin_step(0)

    def _begin_step(self, index: int) -> None:
        assert self._scenario is not None
        if index >= len(self._scenario.steps):
            self._controller.land()
            previous = self._machine.state
            new_state = self._machine.handle_all_steps_done()
            self.get_logger().info(f"multi_step_test_node: {previous!r} -> {new_state!r}")
            self._phase = "landing"
            return

        self._step_index = index
        action = self._scenario.steps[index].action
        self._phase = "step"
        self.get_logger().info(
            f"multi_step_test_node: starting step {index + 1}/{len(self._scenario.steps)}: {action!r}"
        )

        try:
            if action == "hover":
                self._controller.hold()
                self._step_started_at = time.monotonic()
            elif action in _MOVE_OFFSETS:
                dx, dy, dz = _MOVE_OFFSETS[action]
                x, y, z = self._controller.position
                self._controller.set_position(x + dx, y + dy, z + dz)
                self._step_started_at = None
            elif action == "yaw":
                self._current_yaw += _YAW_DELTA_RAD
                self._controller.set_yaw(self._current_yaw)
                self._step_started_at = time.monotonic()
            else:
                raise FlightCommandError(f"unrecognized action {action!r}")
        except FlightCommandError as exc:
            self.get_logger().error(
                f"multi_step_test_node: step {index + 1} ({action!r}) failed to "
                f"start: {exc}"
            )
            self._fail_and_land()

    def _tick_step(self) -> None:
        assert self._scenario is not None
        action = self._scenario.steps[self._step_index].action
        arrived: bool
        if action == "hover":
            assert self._step_started_at is not None
            arrived = time.monotonic() - self._step_started_at >= _HOVER_DURATION_S
        elif action == "yaw":
            # set_yaw() is synchronous in MockFlightController -- arrived
            # on the very next tick after being issued.
            arrived = True
        elif action in _MOVE_OFFSETS:
            arrived = self._controller.velocity == (0.0, 0.0, 0.0)
        else:
            # Should be unreachable -- _begin_step() already rejects an
            # unrecognized action before _phase is ever set to "step".
            arrived = False
        if arrived:
            self._begin_step(self._step_index + 1)

    def _tick_landing(self) -> None:
        if self._controller.armed is not False:
            return
        self._phase = "idle"
        previous = self._machine.state
        new_state = self._machine.handle_landed()
        self.get_logger().info(f"multi_step_test_node: {previous!r} -> {new_state!r}")

    def _fail_and_land(self) -> None:
        """A step's action handler raised -- land immediately (if armed)
        and move the state machine to "failed", not "aborted" (see
        multi_step_state_machine.py's module docstring for why they're
        kept distinct)."""
        previous = self._machine.state
        new_state = self._machine.handle_step_failed()
        self.get_logger().warning(f"multi_step_test_node: step failed -- {previous!r} -> {new_state!r}")
        if self._controller.armed:
            try:
                self._controller.land()
            except FlightCommandError:
                self._controller.disarm()
        self._phase = "idle"

    # -- status publication ----------------------------------------------------

    def _on_status_tick(self) -> None:
        state = self._machine.state
        total_steps = len(self._scenario.steps) if self._scenario is not None else 0
        current_action = (
            self._scenario.steps[self._step_index].action
            if self._scenario is not None and self._phase == "step" and self._step_index < total_steps
            else None
        )

        if self._ever_armed:
            x, y, z = self._controller.position
            current_position = [x, y, z]
        else:
            current_position = None

        payload = {
            "schema_version": SCHEMA_VERSION,
            "mission_id": _MISSION_ID,
            "scenario_id": self._scenario.id if self._scenario is not None else None,
            "state": state,
            "phase": self._phase,
            "current_step_index": self._step_index if self._scenario is not None else None,
            "current_step_action": current_action,
            "total_steps": total_steps,
            "current_position": current_position,
            "armed": self._controller.armed,
            "execution_mode": "mock",
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MultiStepTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
