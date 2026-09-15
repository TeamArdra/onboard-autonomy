"""Pure-logic tests for the multi-step Flight Test scenario registry --
no rclpy dependency."""

import pytest

from nidar_autonomy.flight_test.scenarios import (
    KNOWN_ACTIONS,
    MULTI_STEP_SCENARIOS,
    ScenarioNotFoundError,
    get_scenario,
    scenario_ids,
)


class TestRegistryIntegrity:
    def test_registry_is_non_empty(self):
        assert len(MULTI_STEP_SCENARIOS) > 0

    def test_every_scenario_has_a_unique_id(self):
        ids = [s.id for s in MULTI_STEP_SCENARIOS]
        assert len(ids) == len(set(ids))

    def test_every_scenario_has_at_least_one_step(self):
        for scenario in MULTI_STEP_SCENARIOS:
            assert len(scenario.steps) > 0, scenario.id

    def test_every_step_action_is_a_known_action(self):
        """Guards against multi_step_test_node.py and scenarios.py
        silently drifting apart -- an action named here that the node
        doesn't recognize would be accepted at selection time and only
        fail once the mission is actually running."""
        for scenario in MULTI_STEP_SCENARIOS:
            for step in scenario.steps:
                assert step.action in KNOWN_ACTIONS, (scenario.id, step.action)


class TestGetScenario:
    def test_get_scenario_returns_the_matching_scenario(self):
        scenario = get_scenario("forward_backward_hover")
        assert scenario.id == "forward_backward_hover"
        assert [s.action for s in scenario.steps] == ["forward", "backward", "hover"]

    def test_get_scenario_unknown_id_raises(self):
        with pytest.raises(ScenarioNotFoundError):
            get_scenario("does_not_exist")


class TestScenarioIds:
    def test_scenario_ids_matches_registry(self):
        assert set(scenario_ids()) == {s.id for s in MULTI_STEP_SCENARIOS}


class TestSpecificScenarios:
    def test_test1_forward_backward_hover(self):
        scenario = get_scenario("forward_backward_hover")
        assert [s.action for s in scenario.steps] == ["forward", "backward", "hover"]

    def test_test2_sideways_hover_sideways_hover(self):
        scenario = get_scenario("sideways_hover_sideways_hover")
        assert [s.action for s in scenario.steps] == ["sideways", "hover", "sideways", "hover"]
