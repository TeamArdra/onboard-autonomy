"""Static registry of multi-step Flight Test scenarios -- pure data, no
rclpy dependency, same "importable without a ROS environment" rule as
flight_command_interface.py and hover_test_state_machine.py.

Each scenario is an ordered tuple of `Step`s. A `Step` names one of the
actions `multi_step_test_node.py` knows how to execute against
`FlightCommandInterface` (see that module's `_ACTION_HANDLERS`) -- this
module only says WHAT sequence a scenario is, never HOW an action is
carried out.

This is the onboard-autonomy-side half of the mission/scenario registry.
The GCS-side registry (`custom-gcs/gcs/backend/app/missions.py`) is a
separate, hand-synced copy for display/selection purposes -- same
convention already used for `topics.py` vs.
`custom-gcs/docs/DATA_MODELS.md` (no shared codegen between the two
repos). Keep `scenario_id`s identical in both places; a mismatch means
the GCS could offer a scenario this node will silently refuse to start
(see `multi_step_test_node.py`'s mission-select guard, which logs and
ignores an unrecognized `scenario_id` rather than fabricating one).

MOCK-ONLY, BY CONSTRUCTION -- same posture as the rest of
`nidar_autonomy/flight_test/`: nothing here ever drives
`MAVROSFlightController`. See `flight_test/__init__.py`.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Step:
    action: str
    # Optional per-step override (e.g. a different hover duration_s for
    # one specific step). Empty by default -- most steps use
    # multi_step_test_node.py's own fixed defaults per action.
    params: dict = field(default_factory=dict)


@dataclass(frozen=True)
class MultiStepScenario:
    id: str
    name: str
    steps: tuple[Step, ...]


# Movement actions: (dx, dy, dz) relative offset applied to the vehicle's
# position at the moment the step begins -- see multi_step_test_node.py's
# module docstring for the full execution model (these are the only part
# of it that's pure data, so it lives here rather than in the node
# module, keeping this file importable without rclpy for testing, same
# "no rclpy dependency" rule as flight_command_interface.py). "sideways"
# and "left" share a direction deliberately -- there is no other
# convention yet for what "sideways" means on its own.
MOVE_OFFSETS = {
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "sideways": (0.0, 1.0, 0.0),
}

# Every action name that isn't a positional move (those are the keys of
# MOVE_OFFSETS above) that multi_step_test_node.py knows how to execute.
NON_MOVE_ACTIONS = ("hover", "yaw")

KNOWN_ACTIONS = frozenset(MOVE_OFFSETS) | frozenset(NON_MOVE_ACTIONS)


# Every action name used below must be in KNOWN_ACTIONS --
# test_scenarios.py checks this so a scenario definition can't silently
# drift out of sync with what the node actually knows how to execute.
MULTI_STEP_SCENARIOS: tuple[MultiStepScenario, ...] = (
    MultiStepScenario(
        id="forward_backward_hover",
        name="Test 1: Forward / Backward / Hover",
        steps=(
            Step(action="forward"),
            Step(action="backward"),
            Step(action="hover"),
        ),
    ),
    MultiStepScenario(
        id="sideways_hover_sideways_hover",
        name="Test 2: Sideways / Hover / Sideways / Hover",
        steps=(
            Step(action="sideways"),
            Step(action="hover"),
            Step(action="sideways"),
            Step(action="hover"),
        ),
    ),
)


class ScenarioNotFoundError(KeyError):
    """No MultiStepScenario in MULTI_STEP_SCENARIOS has this id."""


def get_scenario(scenario_id: str) -> MultiStepScenario:
    for scenario in MULTI_STEP_SCENARIOS:
        if scenario.id == scenario_id:
            return scenario
    raise ScenarioNotFoundError(scenario_id)


def scenario_ids() -> tuple[str, ...]:
    return tuple(s.id for s in MULTI_STEP_SCENARIOS)
