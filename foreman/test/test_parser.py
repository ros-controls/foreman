from pathlib import Path

import pytest

from foreman.parser import parse_yaml_file
from foreman.parser import ParsedScenario
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import LifecycleState
from foreman.types import SystemGoal


@pytest.fixture
def scenario_path():
    """Path to the scenario.yaml file."""
    return Path(__file__).parent.parent / "config" / "scenario.yaml"


@pytest.fixture
def parsed_scenario(scenario_path):
    """Parse the scenario.yaml file."""
    return parse_yaml_file(scenario_path)


class TestParsedScenario:
    """Tests for ParsedScenario structure."""

    def test_hardware_list(self, parsed_scenario):
        assert parsed_scenario.hardware == ["FrankaHardwareInterface", "kassow"]

    def test_lifecycle_nodes_list(self, parsed_scenario):
        assert parsed_scenario.lifecycle_nodes == ["dummy_lifecycle_node"]

    def test_metadata_empty(self, parsed_scenario):
        assert parsed_scenario.metadata == {}

    def test_dependency_rules_not_parsed(self, parsed_scenario):
        """Dependencies are inferred at runtime, never parsed from YAML."""
        assert parsed_scenario.dependency_rules == []


class TestGoalStates:
    """Tests for all three goal states."""

    def test_all_goal_states_present(self, parsed_scenario):
        assert set(parsed_scenario.goals.keys()) == {"idle", "broadcast_only", "running"}

    def test_idle_goal(self, parsed_scenario):
        goal = parsed_scenario.goals["idle"]
        assert goal.name == "idle"

        assert len(goal.hardware_goals) == 2
        hw_names = {c.name for c in goal.hardware_goals}
        assert hw_names == {"FrankaHardwareInterface", "kassow"}
        for hw in goal.hardware_goals:
            assert hw.lifecycle_state == LifecycleState.INACTIVE

        assert len(goal.controller_goals) == 3
        ctrl_names = {c.name for c in goal.controller_goals}
        assert ctrl_names == {"joint_state_broadcaster",
                              "kassow_joint_trajectory_controller", "franka_joint_trajectory_controller"}
        for ctrl in goal.controller_goals:
            assert ctrl.lifecycle_state == LifecycleState.INACTIVE

        assert len(goal.lifecycle_node_goals) == 1
        assert goal.lifecycle_node_goals[0].name == "dummy_lifecycle_node"
        assert goal.lifecycle_node_goals[0].lifecycle_state == LifecycleState.INACTIVE

    def test_broadcast_only_goal(self, parsed_scenario):
        goal = parsed_scenario.goals["broadcast_only"]
        assert goal.name == "broadcast_only"

        assert len(goal.hardware_goals) == 2
        for hw in goal.hardware_goals:
            assert hw.lifecycle_state == LifecycleState.ACTIVE

        ctrl_by_name = {c.name: c for c in goal.controller_goals}
        assert ctrl_by_name["joint_state_broadcaster"].lifecycle_state == LifecycleState.ACTIVE
        assert ctrl_by_name["kassow_joint_trajectory_controller"].lifecycle_state == LifecycleState.INACTIVE
        assert ctrl_by_name["franka_joint_trajectory_controller"].lifecycle_state == LifecycleState.INACTIVE

    def test_running_goal(self, parsed_scenario):
        goal = parsed_scenario.goals["running"]
        assert goal.name == "running"

        assert len(goal.hardware_goals) == 2
        for hw in goal.hardware_goals:
            assert hw.lifecycle_state == LifecycleState.ACTIVE

        assert len(goal.controller_goals) == 3
        for ctrl in goal.controller_goals:
            assert ctrl.lifecycle_state == LifecycleState.ACTIVE

    def test_goal_component_types(self, parsed_scenario):
        idle = parsed_scenario.goals["idle"]
        for hw in idle.hardware_goals:
            assert hw.component_type == ComponentType.HARDWARE
        for ctrl in idle.controller_goals:
            assert ctrl.component_type == ComponentType.CONTROLLER
        for lc in idle.lifecycle_node_goals:
            assert lc.component_type == ComponentType.LIFECYCLE_NODE
