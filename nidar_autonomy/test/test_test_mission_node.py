"""Integration tests for test_mission_node.py -- requires a real rclpy
context, gated with `pytest.importorskip("rclpy")` (same pattern
test_simulation_node.py/test_mission_state_node.py already use). Run
with ROS sourced (`source /opt/ros/humble/setup.bash`) to actually
exercise this file instead of skipping it.

Uses `MockFlightController` (already fully tested, ROS-free) via this
node's `controller=` injection point instead of the real
`MAVROSFlightController` -- this file is coverage of the ROS GLUE this
node adds (command validation, scan staleness handling, geofence
wiring, topic isolation), not a re-test of `TestMissionPolicy`'s
decision logic (see test_test_mission_policy.py for that) or of the real
mavros-facing clients (see setpoint_client.py's own docstring for why
those have no test file in this sandbox).
"""
import math
import time

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from nidar_autonomy.mock_flight_controller import MockFlightController  # noqa: E402
from nidar_autonomy.test_mission_policy import ScanSample, TestMissionState  # noqa: E402
from nidar_autonomy.topics import COMMAND_TOPIC, TEST_MISSION_COMMAND_TOPIC  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.test_mission_node import TestMissionNode

    node = TestMissionNode(controller=MockFlightController())
    return node, node.destroy_node


class TestTopicIsolation:
    def test_command_topic_is_the_test_mission_specific_one_not_real(self):
        assert TEST_MISSION_COMMAND_TOPIC != COMMAND_TOPIC
        assert TEST_MISSION_COMMAND_TOPIC.startswith("/test_mission/")

    def test_invalid_command_is_ignored_not_crashing(self):
        node, cleanup = _make_node()
        try:
            node._on_command(String(data="run"))  # simulation vocabulary -- must be rejected here
            assert node._policy.state == TestMissionState.IDLE
            node._on_command(String(data="anything else"))
            assert node._policy.state == TestMissionState.IDLE
        finally:
            cleanup()


class TestCommandHandling:
    def test_start_command_arms_and_begins_takeoff(self):
        node, cleanup = _make_node()
        try:
            node._start_locked()  # synchronous stand-in for the threaded dispatch -- deterministic for tests
            assert node._controller.armed
            assert node._policy.state == TestMissionState.TAKING_OFF
        finally:
            cleanup()

    def test_stop_command_before_start_is_a_noop(self):
        node, cleanup = _make_node()
        try:
            node._on_command(String(data="stop"))
            assert node._policy.state == TestMissionState.IDLE
        finally:
            cleanup()

    def test_stop_command_lands(self):
        node, cleanup = _make_node()
        try:
            node._start_locked()
            node._on_command(String(data="stop"))
            assert node._policy.state == TestMissionState.LANDING
        finally:
            cleanup()


class TestScanStaleness:
    def test_scan_older_than_threshold_is_not_used(self):
        node, cleanup = _make_node()
        try:
            node._latest_scan = ScanSample(ranges=(5.0,) * 360, angle_min=-math.pi, angle_increment=2 * math.pi / 360)
            node._latest_scan_monotonic = 0.0  # arbitrarily far in the past
            assert node._current_scan_if_fresh() is None
        finally:
            cleanup()

    def test_fresh_scan_is_used(self):
        node, cleanup = _make_node()
        try:
            node._latest_scan = ScanSample(ranges=(5.0,) * 360, angle_min=-math.pi, angle_increment=2 * math.pi / 360)
            node._latest_scan_monotonic = time.monotonic()
            assert node._current_scan_if_fresh() is not None
        finally:
            cleanup()

    def test_no_scan_received_yet_is_none(self):
        node, cleanup = _make_node()
        try:
            assert node._current_scan_if_fresh() is None
        finally:
            cleanup()


class TestGeofence:
    def test_breach_true_triggers_abort(self):
        node, cleanup = _make_node()
        try:
            node._start_locked()
            node._on_geofence(Bool(data=True))
            assert node._policy.state == TestMissionState.ABORTED
        finally:
            cleanup()

    def test_breach_false_does_not_abort(self):
        node, cleanup = _make_node()
        try:
            node._start_locked()
            node._on_geofence(Bool(data=False))
            assert node._policy.state == TestMissionState.TAKING_OFF
        finally:
            cleanup()
