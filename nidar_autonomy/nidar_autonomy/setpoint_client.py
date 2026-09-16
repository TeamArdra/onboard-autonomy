"""ROS-facing setpoint publisher + OFFBOARD engagement -- the ONE place
in this repo allowed to publish a mavros setpoint topic or request
OFFBOARD mode, mirroring `flight_command.py`'s own scoping ("the only
place in this repo allowed to touch mavros's arming interface") for the
motion side of the boundary instead of the arming side. Backs
`mavros_flight_controller.MAVROSFlightController`'s real
takeoff/land/hold/set_position/set_velocity/abort.

Design (per the safety-reviewed design pass this file was built from --
see the repo's plan record for "First Real Test-Flight Mission"):

  - **Control mode**: PX4 Offboard via `/mavros/setpoint_position/local`
    (position-seek) and `/mavros/setpoint_velocity/cmd_vel` (velocity).
    A dedicated timer republishes the active setpoint continuously --
    PX4 requires a steady stream or it drops out of Offboard.
  - **Position source**: `/mavros/local_position/pose` +
    `/mavros/local_position/velocity_local` -- PX4's own EKF2 output
    (fed by the MTF-01 for this stage), never fabricated if no message
    has arrived yet (`TelemetryUnavailableError`, same "raise, don't
    fabricate" rule as `flight_command_interface.py`).
  - **Offboard entry**: PX4 requires an existing setpoint stream before
    it will accept a mode switch -- this class always streams a
    hold-at-current-position setpoint from construction, and only
    requests OFFBOARD once that stream has run for
    `_MIN_STREAM_TICKS_BEFORE_OFFBOARD` ticks.
  - **Abort-preemption**: `request_abort()` immediately stops honoring
    whatever command was active; the publish timer then drives
    `abort_sequence.next_action()` every tick (hold -> controlled
    descent -> "at ground, safe to disarm") until told to disarm. This
    class NEVER calls the mavros arming service itself -- Hard Safety
    Rule 1 reserves that exclusively to `flight_command.FlightCommandClient`
    -- it only reports readiness via `on_ground_after_abort`, and the
    caller (`MAVROSFlightController`, which already holds the approved
    `FlightCommandClient`) is the one that actually disarms.

Deliberately NOT covered by any test in this environment -- there is no
rclpy/mavros installed here to run against, so this file cannot be
imported, let alone unit-tested, in this sandbox. The pure decision logic
it depends on (`abort_sequence.py`) IS fully unit-tested. This module is
ROS glue around that logic and MUST be validated against SITL, then a
props-off bench test, before being trusted -- per this repo's Hard Safety
Rule 3 and the plan that authorized writing it. Do not treat "it imports
without a syntax error" as validation.
"""
from __future__ import annotations

import time
from typing import Callable, Optional, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.node import Node

from .abort_sequence import AbortAction, AbortSequenceConfig
from .abort_sequence import next_action as next_abort_action
from .geometry import quaternion_from_yaw

# Reused directly (not re-defined) so `MAVROSFlightController` never needs
# to catch-and-rewrap this module's telemetry-not-ready errors -- a safe
# one-directional import: `mavros_flight_controller.py` has no
# unconditional rclpy/mavros_msgs import (only under `TYPE_CHECKING`), so
# importing from it here does not pull ROS into anything that imports
# THIS module transitively, and there is no import cycle since that
# module never imports this one at runtime.
from .mavros_flight_controller import TelemetryUnavailableError

SETPOINT_POSITION_TOPIC = "/mavros/setpoint_position/local"
SETPOINT_VELOCITY_TOPIC = "/mavros/setpoint_velocity/cmd_vel"
LOCAL_POSITION_TOPIC = "/mavros/local_position/pose"
LOCAL_VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
SET_MODE_SERVICE = "/mavros/set_mode"
STATE_TOPIC = "/mavros/state"

OFFBOARD_MODE = "OFFBOARD"

# ~1s of warm-up at the default 20Hz rate -- see module docstring.
_MIN_STREAM_TICKS_BEFORE_OFFBOARD = 20


class SetpointClient:
    def __init__(
        self,
        node: Node,
        *,
        rate_hz: float = 20.0,
        abort_config: AbortSequenceConfig = AbortSequenceConfig(),
        on_ground_after_abort: Optional[Callable[[], None]] = None,
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._abort_config = abort_config
        self._on_ground_after_abort = on_ground_after_abort

        self._position_pub = node.create_publisher(PoseStamped, SETPOINT_POSITION_TOPIC, 10)
        self._velocity_pub = node.create_publisher(TwistStamped, SETPOINT_VELOCITY_TOPIC, 10)
        self._set_mode_client = node.create_client(SetMode, SET_MODE_SERVICE)

        self._latest_position: Optional[Tuple[float, float, float]] = None
        self._latest_velocity: Optional[Tuple[float, float, float]] = None
        self._latest_mode: Optional[str] = None
        node.create_subscription(PoseStamped, LOCAL_POSITION_TOPIC, self._on_position, 10)
        node.create_subscription(TwistStamped, LOCAL_VELOCITY_TOPIC, self._on_velocity, 10)
        node.create_subscription(State, STATE_TOPIC, self._on_state, 10)

        # What the publish loop is currently expressing -- distinct from
        # PX4's own reported flight mode (self._latest_mode).
        self._active = "POSITION"  # "POSITION" | "VELOCITY"
        self._target_position: Optional[Tuple[float, float, float, Optional[float]]] = None
        self._target_velocity: Optional[Tuple[float, float, float, Optional[float]]] = None

        self._abort_requested = False
        self._abort_started_monotonic: Optional[float] = None
        self._abort_hold_position: Optional[Tuple[float, float, float]] = None
        self._landed_after_abort = False

        self._stream_ticks = 0
        self._offboard_requested = False

        node.create_timer(1.0 / rate_hz, self._tick)
        self._log.info(
            f"setpoint_client: streaming on {SETPOINT_POSITION_TOPIC} / "
            f"{SETPOINT_VELOCITY_TOPIC} @ {rate_hz}Hz once "
            f"{LOCAL_POSITION_TOPIC} is available"
        )

    # -- telemetry ----------------------------------------------------------

    @property
    def position(self) -> Tuple[float, float, float]:
        if self._latest_position is None:
            raise TelemetryUnavailableError(f"no {LOCAL_POSITION_TOPIC} message received yet")
        return self._latest_position

    @property
    def velocity(self) -> Tuple[float, float, float]:
        if self._latest_velocity is None:
            raise TelemetryUnavailableError(f"no {LOCAL_VELOCITY_TOPIC} message received yet")
        return self._latest_velocity

    @property
    def offboard_engaged(self) -> bool:
        return self._latest_mode == OFFBOARD_MODE

    def _on_position(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._latest_position = (p.x, p.y, p.z)

    def _on_velocity(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._latest_velocity = (v.x, v.y, v.z)

    def _on_state(self, msg: State) -> None:
        self._latest_mode = msg.mode

    # -- commands -------------------------------------------------------------

    def set_position_target(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        self._active = "POSITION"
        self._target_position = (x, y, z, yaw)

    def set_velocity_target(self, vx: float, vy: float, vz: float, yaw_rate: Optional[float] = None) -> None:
        self._active = "VELOCITY"
        self._target_velocity = (vx, vy, vz, yaw_rate)

    def hold(self) -> None:
        """Freeze in place at the current known position. Requires a real
        position reading -- raises rather than holding at a fabricated
        location if none exists yet."""
        x, y, z = self.position
        self.set_position_target(x, y, z)

    def set_yaw(self, yaw: float) -> None:
        """Retarget the yaw of whatever position target is active (or the
        current known position, if none is). Velocity-mode yaw control
        (turning while translating via `set_velocity`) is out of scope
        for this stage -- nothing in `test_mission_policy.py` issues yaw
        while in velocity mode, so this is not implemented for that
        combination rather than silently doing something unreviewed."""
        if self._target_position is not None:
            x, y, z, _old_yaw = self._target_position
            self._target_position = (x, y, z, yaw)
            return
        x, y, z = self.position
        self._active = "POSITION"
        self._target_position = (x, y, z, yaw)

    def request_abort(self) -> None:
        """Idempotent: a second call while already aborting does not
        reset the sequence's clock or re-capture the hold position."""
        if self._abort_requested:
            return
        self._abort_requested = True
        self._abort_started_monotonic = time.monotonic()
        self._abort_hold_position = self._latest_position

    # -- tick -------------------------------------------------------------------

    def _tick(self) -> None:
        if self._abort_requested:
            self._tick_abort()
            return

        if self._latest_position is None:
            return  # nothing safe to publish yet -- see module docstring

        if self._stream_ticks < _MIN_STREAM_TICKS_BEFORE_OFFBOARD:
            self._publish_hold_at_current()
            self._stream_ticks += 1
            return

        if not self._offboard_requested:
            self._request_offboard()

        if self._active == "VELOCITY" and self._target_velocity is not None:
            self._publish_velocity(*self._target_velocity)
        elif self._target_position is not None:
            self._publish_position(*self._target_position)
        else:
            self._publish_hold_at_current()

    def _tick_abort(self) -> None:
        if self._abort_hold_position is None:
            self._abort_hold_position = self._latest_position
            if self._abort_hold_position is None:
                return  # still nothing safe to publish

        elapsed = time.monotonic() - self._abort_started_monotonic
        height = (
            self._latest_position[2]
            if self._latest_position is not None
            else self._abort_hold_position[2]
        )
        action = next_abort_action(elapsed, height, self._abort_config)

        if action == AbortAction.HOLD:
            x, y, z = self._abort_hold_position
            self._publish_position(x, y, z, None)
        elif action == AbortAction.DESCEND:
            self._publish_velocity(0.0, 0.0, -self._abort_config.descend_speed_mps, None)
        else:  # DISARM
            self._publish_velocity(0.0, 0.0, 0.0, None)
            if not self._landed_after_abort:
                self._landed_after_abort = True
                self._log.warning(
                    "[setpoint_client] abort sequence reached ground height -- "
                    "signaling ready to disarm"
                )
                if self._on_ground_after_abort is not None:
                    self._on_ground_after_abort()

    def _request_offboard(self) -> None:
        if not self._set_mode_client.service_is_ready():
            return  # retry next tick
        self._offboard_requested = True
        request = SetMode.Request()
        request.custom_mode = OFFBOARD_MODE
        future = self._set_mode_client.call_async(request)
        future.add_done_callback(self._on_set_mode_response)

    def _on_set_mode_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- service/network failure, not a bug
            self._log.error(f"[setpoint_client] SetMode(OFFBOARD) call failed: {exc!r}")
            self._offboard_requested = False  # allow a retry next tick
            return
        if response.mode_sent:
            self._log.info("[setpoint_client] OFFBOARD requested (mode_sent=True)")
        else:
            self._log.error("[setpoint_client] SetMode(OFFBOARD) rejected (mode_sent=False) -- will retry")
            self._offboard_requested = False

    # -- publishing -------------------------------------------------------------

    def _publish_position(self, x: float, y: float, z: float, yaw: Optional[float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(yaw if yaw is not None else 0.0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._position_pub.publish(msg)

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: Optional[float]) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        if yaw_rate is not None:
            msg.twist.angular.z = yaw_rate
        self._velocity_pub.publish(msg)

    def _publish_hold_at_current(self) -> None:
        assert self._latest_position is not None
        self._publish_position(*self._latest_position, None)
