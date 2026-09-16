"""ROS node wrapper for the dev-only "test mission" (hover / find-open-
space / move / stop, repeating) -- see `test_mission_policy.py` for the
decision logic this only wires up to real topics and clients.

REAL FLIGHT, unlike `simulation_node.py`: this node legitimately imports
`mavros_flight_controller`/`flight_command`/`setpoint_client`, since
driving real flight is exactly its job -- the one deliberate structural
difference from `simulation_node.py`'s "never touches mavros" isolation
(see that module's own docstring). Do not "fix" this import by removing
it thinking it's a mistake.

Isolation from the real competition mission instead comes from topic
naming, per `topics.py`'s "Test-mission topics" section: this node
subscribes ONLY `/test_mission/command` (never the real `/gcs/command`)
and publishes ONLY `/test_mission/status` (never `/mission/state`).
`mission_state_node.py` never imports or references this node, and this
node never imports `command_node.py`, `mission_state_node.py`, or
`arm_trigger.py`.

Threading: mirrors `mission_state_node.py`'s own established pattern --
the blocking calls (`arm()`/`disarm()`, which wait on real FCU
confirmation for up to several seconds) are dispatched onto daemon
threads, serialized by `self._flight_lock`, never called inline from a
subscription/timer callback (that would stall this node's single-
threaded executor -- the ~10Hz mission tick and the setpoint publisher's
own timer -- for the whole wait). The periodic mission tick itself
touches `self._policy`'s internal state without that same lock, same
tradeoff `mission_state_node.py` already accepts for `self._machine`
(simple attribute mutation, not a compound multi-step transaction, so
the GIL's per-assignment atomicity is treated as sufficient here too --
this is an intentional, precedented choice, not an oversight).
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .flight_command import FlightCommandClient
from .flight_command_interface import FlightCommandError, FlightCommandInterface
from .mavros_flight_controller import MAVROSFlightController
from .setpoint_client import SetpointClient
from .test_mission_policy import ScanSample, TestMissionPolicy
from .topics import (
    GEOFENCE_BREACH_TOPIC,
    SCAN_TOPIC,
    TEST_MISSION_COMMAND_TOPIC,
    TEST_MISSION_STATUS_TOPIC,
    VALID_TEST_MISSION_COMMANDS,
)

# A /scan reading older than this is never trusted -- the policy holds in
# place rather than moving on stale data (see gap_finder.py's and
# test_mission_policy.py's "never guess" convention).
_SCAN_STALE_SEC = 1.0


class TestMissionNode(Node):
    __test__ = False  # not a pytest test class -- name collides with "Test*" convention

    def __init__(self, controller: FlightCommandInterface | None = None) -> None:
        """`controller`, when given, replaces the real
        `MAVROSFlightController` (real `FlightCommandClient` +
        `SetpointClient`, both requiring real mavros topics/services) --
        e.g. a `mock_flight_controller.MockFlightController` in tests, so
        this node's own ROS-glue (command validation, scan staleness,
        geofence handling, topic isolation) can be exercised without a
        real Pixhawk/mavros, same injection pattern
        `mission_state_node.MissionStateNode(flight_client=...)` and
        `simulation_node.SimulationNode(simulator=...)` already use."""
        super().__init__("test_mission_node")

        self.declare_parameter("takeoff_height_m", 0.5)
        self.declare_parameter("hover_duration_s", 3.0)
        self.declare_parameter("max_step_m", 0.5)
        self.declare_parameter("min_clearance_m", 0.5)
        self.declare_parameter("max_valid_range_m", 8.0)
        self.declare_parameter("tick_hz", 10.0)
        get = lambda name: self.get_parameter(name).value  # noqa: E731

        self._flight_lock = threading.Lock()

        if controller is not None:
            self._controller = controller
        else:
            client = FlightCommandClient(self, already_spinning=True)
            setpoints = SetpointClient(self, on_ground_after_abort=self._on_ground_after_abort)
            self._controller = MAVROSFlightController(client, setpoints)
        self._policy = TestMissionPolicy(
            self._controller,
            takeoff_height_m=get("takeoff_height_m"),
            hover_duration_s=get("hover_duration_s"),
            max_step_m=get("max_step_m"),
            min_clearance_m=get("min_clearance_m"),
            max_valid_range_m=get("max_valid_range_m"),
        )

        self._latest_scan: Optional[ScanSample] = None
        self._latest_scan_monotonic: Optional[float] = None

        self.create_subscription(String, TEST_MISSION_COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(Bool, GEOFENCE_BREACH_TOPIC, self._on_geofence, 10)
        self._status_pub = self.create_publisher(String, TEST_MISSION_STATUS_TOPIC, 10)

        tick_hz = get("tick_hz")
        self._dt = 1.0 / tick_hz
        self.create_timer(self._dt, self._tick)

        self.get_logger().warning(
            f"test_mission_node: REAL FLIGHT test mission ready on "
            f"{TEST_MISSION_COMMAND_TOPIC} ('start'/'stop' only) -- this "
            "node commands the real Pixhawk, unlike simulation_node.py"
        )

    # -- inputs -------------------------------------------------------------

    def _on_command(self, msg: String) -> None:
        command = msg.data
        if command not in VALID_TEST_MISSION_COMMANDS:
            self.get_logger().warning(
                f"ignoring invalid test-mission command {command!r} -- "
                f"expected one of {VALID_TEST_MISSION_COMMANDS}"
            )
            return
        if command == "start":
            self.get_logger().warning("TEST MISSION: start requested (REAL FLIGHT)")
            threading.Thread(target=self._start_locked, daemon=True).start()
        else:
            self.get_logger().warning("TEST MISSION: stop requested -- landing in place")
            # stop()/land() never blocks on the FCU (only a setpoint
            # target is set; touchdown/disarm happens later via PX4's own
            # landing detection -- see mavros_flight_controller.land()'s
            # docstring), so this can run inline under the lock rather
            # than needing its own thread.
            with self._flight_lock:
                self._policy.stop()

    def _start_locked(self) -> None:
        with self._flight_lock:
            try:
                self._policy.start()
            except FlightCommandError as exc:
                self.get_logger().error(f"TEST MISSION: start failed: {exc!r}")

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = ScanSample(
            ranges=tuple(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
        )
        self._latest_scan_monotonic = time.monotonic()

    def _on_geofence(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.get_logger().error("TEST MISSION: geofence breach reported -- aborting")
        self._policy.abort()  # non-blocking: only flags the setpoint client's abort sequence

    # -- tick -----------------------------------------------------------------

    def _tick(self) -> None:
        self._policy.tick(self._dt, self._current_scan_if_fresh())
        self._status_pub.publish(String(data=self._policy.state))

    def _current_scan_if_fresh(self) -> Optional[ScanSample]:
        if self._latest_scan is None or self._latest_scan_monotonic is None:
            return None
        if time.monotonic() - self._latest_scan_monotonic > _SCAN_STALE_SEC:
            return None
        return self._latest_scan

    # -- abort -> disarm handoff --------------------------------------------

    def _on_ground_after_abort(self) -> None:
        """`SetpointClient`'s own abort sequence has reached ground
        height and reports it is now safe to disarm -- see
        `setpoint_client.py`'s docstring. This is the ONLY path by which
        this node calls `disarm()` outside a normal `stop()`, and it
        still goes exclusively through the already-approved
        `FlightCommandClient` (Hard Safety Rule 1). Dispatched onto a
        thread (same reasoning as `_start_locked`) since `disarm()`
        blocks waiting on FCU confirmation, and this callback itself runs
        from `SetpointClient`'s own timer tick on the executor thread."""
        threading.Thread(target=self._disarm_after_abort_locked, daemon=True).start()

    def _disarm_after_abort_locked(self) -> None:
        with self._flight_lock:
            try:
                self._controller.disarm()
            except FlightCommandError as exc:
                self.get_logger().error(f"TEST MISSION: post-abort disarm failed: {exc!r}")


def main() -> None:
    rclpy.init()
    node = TestMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
