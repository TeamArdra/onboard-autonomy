"""Dev-only "test mission" -- hover / find open space / move / stop,
repeating until told to stop. Pure logic, no rclpy, driven by an explicit
`tick(dt, scan)` call, same "no background clock" posture as
`mock_flight_controller.MockFlightController.tick()` -- required for
reproducible tests, and because this is meant to run against either
`MockFlightController` (tests, and a first bench dry-run) or a real
`MAVROSFlightController` unchanged, same "same interface, same calling
code, no rewrite" principle `mission_simulator.py` already established
for the RUN SIMULATION path.

UNLIKE `mission_simulator.py`, this module is explicitly allowed to
drive a REAL `FlightCommandInterface` -- it has no "HARD SAFETY BOUNDARY:
never imports rclpy/mavros" guarantee, because commanding real flight is
exactly its job. Its own safety posture instead comes from:

  - only ever moving `max_step_m` at a time, and only in a direction
    `gap_finder.find_open_direction()` has just certified clears
    `min_clearance_m` beyond that step (see that module's docstring);
  - never guessing a direction when the scan is missing/stale -- staying
    in HOVERING and simply trying again next tick;
  - `abort()` being callable from every state and always winning: it
    hands off to the controller's own `abort()` (cancel-in-place, never
    an instant mid-air disarm -- see `flight_command_interface.py`) and
    this policy then refuses to issue anything else, ever, for its
    remaining lifetime.

Frame note: `gap_finder`'s direction is a bearing in the sensor's own
frame. This module applies it directly as a local-frame (x, y) offset
from the controller's current position -- i.e. it assumes the lidar's
zero-bearing is aligned with the vehicle's local +X axis and the vehicle
is not yawing during a move. That is correct for this stage (no yaw
commands are ever issued here -- `set_yaw` is never called) and should be
revisited if a future stage adds yaw control.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .flight_command_interface import FlightCommandInterface
from .gap_finder import find_open_direction

_ARRIVAL_TOLERANCE_M = 0.05


@dataclass(frozen=True)
class ScanSample:
    """A single `/scan` reading, already unpacked to plain values by the
    ROS boundary (`test_mission_node.py`) -- see `gap_finder.py`'s own
    docstring for why this module never touches `sensor_msgs/LaserScan`
    directly."""

    ranges: tuple[float, ...]
    angle_min: float
    angle_increment: float


class TestMissionState:
    __test__ = False  # not a pytest test class -- name collides with "Test*" convention
    IDLE = "idle"
    TAKING_OFF = "taking_off"
    HOVERING = "hovering"
    MOVING = "moving"
    LANDING = "landing"
    STOPPED = "stopped"
    ABORTED = "aborted"


class TestMissionPolicy:
    """Owns one run of the repeating hover/move/stop loop against a given
    `FlightCommandInterface`. Call `start()` once, then `tick()`
    repeatedly (each call is one fixed-`dt` step) until `state` is
    `STOPPED` or `ABORTED`. Call `stop()` at any time for a graceful
    landing, or `abort()` for an immediate safety-triggered preempt (see
    module docstring)."""

    __test__ = False  # not a pytest test class -- name collides with "Test*" convention

    def __init__(
        self,
        controller: FlightCommandInterface,
        *,
        takeoff_height_m: float = 0.5,
        hover_duration_s: float = 3.0,
        max_step_m: float = 0.5,
        min_clearance_m: float = 0.5,
        max_valid_range_m: float = 8.0,
    ) -> None:
        if takeoff_height_m <= 0:
            raise ValueError("takeoff_height_m must be positive")
        if hover_duration_s <= 0:
            raise ValueError("hover_duration_s must be positive")

        self._controller = controller
        self._takeoff_height_m = takeoff_height_m
        self._hover_duration_s = hover_duration_s
        self._max_step_m = max_step_m
        self._min_clearance_m = min_clearance_m
        self._max_valid_range_m = max_valid_range_m

        self._state = TestMissionState.IDLE
        self._hover_elapsed_s = 0.0
        self._target: Optional[tuple[float, float, float]] = None

    @property
    def state(self) -> str:
        return self._state

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        """Arm and take off. Only valid from IDLE -- a duplicate `start`
        while already running is a no-op, not a second takeoff attempt
        (mirrors `state_machine.MissionStateMachine`'s "only the state
        that implies this transition makes sense reacts" convention)."""
        if self._state != TestMissionState.IDLE:
            return
        self._controller.arm()
        self._controller.takeoff(self._takeoff_height_m)
        self._state = TestMissionState.TAKING_OFF

    def stop(self) -> None:
        """Graceful stop: land in place from wherever the mission
        currently is, then disarm on touchdown (the controller's own
        `land()` contract). No-op once already landing/stopped/aborted."""
        if self._state in (
            TestMissionState.LANDING,
            TestMissionState.STOPPED,
            TestMissionState.ABORTED,
            TestMissionState.IDLE,
        ):
            return
        self._controller.land()
        self._state = TestMissionState.LANDING

    def abort(self) -> None:
        """Immediate safety-triggered preempt (e.g. a geofence breach) --
        callable from ANY state, always wins, and this policy issues
        nothing further afterward. Delegates the actual "what does a safe
        in-flight stop mean" decision entirely to the controller's own
        `abort()` (hold-then-land, never an instant disarm mid-air -- see
        `flight_command_interface.FlightCommandInterface.abort()`)."""
        if self._state == TestMissionState.ABORTED:
            return
        self._controller.abort()
        self._state = TestMissionState.ABORTED

    # -- per-tick advance ---------------------------------------------------

    def tick(self, dt: float, scan: Optional[ScanSample]) -> None:
        """Advance by one `dt`-second step. `scan` is the latest `/scan`
        reading, or `None` if none has arrived recently enough to trust
        (staleness is the caller/node's call, not this module's -- see
        `test_mission_node.py`). A no-op once STOPPED or ABORTED."""
        if self._state in (TestMissionState.STOPPED, TestMissionState.ABORTED, TestMissionState.IDLE):
            return

        if self._state == TestMissionState.TAKING_OFF:
            self._tick_taking_off()
        elif self._state == TestMissionState.HOVERING:
            self._tick_hovering(dt, scan)
        elif self._state == TestMissionState.MOVING:
            self._tick_moving()
        elif self._state == TestMissionState.LANDING:
            self._tick_landing()

    def _tick_taking_off(self) -> None:
        _x, _y, z = self._controller.position
        if z >= self._takeoff_height_m - _ARRIVAL_TOLERANCE_M:
            self._enter_hovering()

    def _tick_hovering(self, dt: float, scan: Optional[ScanSample]) -> None:
        self._hover_elapsed_s += dt
        if self._hover_elapsed_s < self._hover_duration_s:
            return

        if scan is None or not scan.ranges:
            # No trustworthy scan yet -- keep hovering and try again next
            # tick, rather than guessing a direction. Do not reset
            # _hover_elapsed_s: once the hover dwell has elapsed, every
            # subsequent tick re-attempts immediately as soon as a scan
            # does arrive, instead of waiting out a second full dwell.
            return

        gap = find_open_direction(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            min_clearance_m=self._min_clearance_m,
            max_step_m=self._max_step_m,
            max_valid_range_m=self._max_valid_range_m,
        )
        if gap is None:
            # Nowhere safe to go right now -- stay put and re-check next
            # tick (e.g. a momentarily-noisy scan, or a genuine dead end
            # someone needs to come move the vehicle out of).
            return

        x, y, z = self._controller.position
        target = (
            x + gap.step_m * math.cos(gap.direction_rad),
            y + gap.step_m * math.sin(gap.direction_rad),
            z,
        )
        self._controller.set_position(*target)
        self._target = target
        self._state = TestMissionState.MOVING

    def _tick_moving(self) -> None:
        assert self._target is not None
        x, y, z = self._controller.position
        tx, ty, tz = self._target
        if math.dist((x, y, z), (tx, ty, tz)) <= _ARRIVAL_TOLERANCE_M:
            self._controller.hold()
            self._target = None
            self._enter_hovering()

    def _tick_landing(self) -> None:
        if not self._controller.armed:
            self._state = TestMissionState.STOPPED

    def _enter_hovering(self) -> None:
        self._state = TestMissionState.HOVERING
        self._hover_elapsed_s = 0.0
