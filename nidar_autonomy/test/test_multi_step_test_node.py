"""Integration tests for multi_step_test_node.py -- requires a real rclpy
context, gated with `pytest.importorskip("rclpy")`, same pattern as this
repo's existing test_frontier_explorer_node.py / test_mission_state_node.py.

Drives the node's timing loop by calling `_on_tick()`/`_on_status_tick()`
directly (never a real timer/executor spin) for fast, deterministic
tests -- same rationale as test_frontier_explorer_node.py's white-box
approach: the pub/sub wiring itself is a thin, already-reviewed pattern
shared with hover_test_node.py, so what actually needs test coverage is
the step-sequencing logic.
"""
import json

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.flight_test.multi_step_test_node import MultiStepTestNode

    node = MultiStepTestNode()
    return node, node.destroy_node


def _select(node, mission_id, scenario_id):
    node._on_mission_select(String(data=json.dumps({"mission_id": mission_id, "scenario_id": scenario_id})))


def _validated(node, command):
    node._on_validated_command(String(data=command))


def _run_until_complete(node, max_ticks=2000):
    """Ticks the node until it reaches a terminal state. "hover" steps
    (and takeoff/land target-seeks) are wall-clock/arrival-based, not
    tick-count-based, so a tight loop with no real sleep would never
    satisfy a hover step's duration -- fast-forward it by rewinding
    _step_started_at into the past before each tick instead of actually
    sleeping for real seconds."""
    import time

    for _ in range(max_ticks):
        if node._machine.state in ("complete", "aborted", "failed"):
            return
        if (
            node._phase == "step"
            and node._scenario is not None
            and node._scenario.steps[node._step_index].action == "hover"
            and node._step_started_at is not None
        ):
            node._step_started_at = time.monotonic() - 3600.0
        node._on_tick()
    raise AssertionError(f"did not reach a terminal state within {max_ticks} ticks (state={node._machine.state!r})")


class TestMissionSelectGuard:
    def test_start_ignored_without_a_selection(self):
        node, cleanup = _make_node()
        try:
            _validated(node, "start")
            assert node._machine.state == "idle"
        finally:
            cleanup()

    def test_start_ignored_for_wrong_mission_id(self):
        node, cleanup = _make_node()
        try:
            _select(node, "main_nidar", "forward_backward_hover")
            _validated(node, "start")
            assert node._machine.state == "idle"
        finally:
            cleanup()

    def test_start_ignored_for_unknown_scenario_id(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "hover")  # hover_test_node's scenario, not this node's
            _validated(node, "start")
            assert node._machine.state == "idle"
        finally:
            cleanup()

    def test_start_accepted_for_recognized_scenario(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "forward_backward_hover")
            _validated(node, "start")
            assert node._machine.state == "executing"
        finally:
            cleanup()


class TestFullRunSelectsTheRightScenario:
    def test_test1_forward_backward_hover_completes(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "forward_backward_hover")
            _validated(node, "start")
            _run_until_complete(node)
            assert node._machine.state == "complete"
            assert node._controller.armed is False
        finally:
            cleanup()

    def test_test2_sideways_hover_sideways_hover_completes(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "sideways_hover_sideways_hover")
            _validated(node, "start")
            _run_until_complete(node)
            assert node._machine.state == "complete"
        finally:
            cleanup()

    def test_selecting_test1_does_not_run_test2s_steps(self):
        """The core assertion the master-task spec calls out: selecting
        a different scenario actually changes which steps execute, not
        just which label is shown."""
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "forward_backward_hover")
            _validated(node, "start")
            assert node._scenario is not None
            assert [s.action for s in node._scenario.steps] == ["forward", "backward", "hover"]
        finally:
            cleanup()


class TestAbort:
    def test_abort_mid_scenario_goes_to_aborted_and_disarms(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "forward_backward_hover")
            _validated(node, "start")
            for _ in range(5):
                node._on_tick()
            assert node._machine.state == "executing"
            _validated(node, "abort")
            assert node._machine.state == "aborted"
            assert node._controller.armed is False
        finally:
            cleanup()

    def test_abort_is_a_noop_when_idle(self):
        node, cleanup = _make_node()
        try:
            _validated(node, "abort")
            assert node._machine.state == "idle"
        finally:
            cleanup()


class TestStatusPublication:
    def test_status_reports_scenario_and_step_progress(self):
        node, cleanup = _make_node()
        try:
            _select(node, "flight_test", "forward_backward_hover")
            _validated(node, "start")

            captured = {}
            node._status_pub.publish = lambda msg: captured.setdefault("status", msg)
            node._on_status_tick()
            payload = json.loads(captured["status"].data)

            assert payload["mission_id"] == "flight_test"
            assert payload["scenario_id"] == "forward_backward_hover"
            assert payload["total_steps"] == 3
            assert payload["execution_mode"] == "mock"
        finally:
            cleanup()

    def test_status_before_any_start_reports_idle_with_no_scenario(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node._status_pub.publish = lambda msg: captured.setdefault("status", msg)
            node._on_status_tick()
            payload = json.loads(captured["status"].data)

            assert payload["state"] == "idle"
            assert payload["scenario_id"] is None
            assert payload["current_position"] is None
        finally:
            cleanup()
