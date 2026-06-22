from pathlib import Path

import pytest

from foreman.parser import parse_yaml_file
from foreman.parser import ParsedScenario
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ControllerDependencyRule
from foreman.types import HardwareRequirement
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


class TestDependencyRules:
    """Tests for parsed dependency rules."""

    def test_rules_count(self, parsed_scenario):
        assert len(parsed_scenario.dependency_rules) == 3

    def test_joint_state_broadcaster_rule(self, parsed_scenario):
        rule = next(r for r in parsed_scenario.dependency_rules if r.controller_name ==
                    "joint_state_broadcaster")
        assert rule.controller_name == "joint_state_broadcaster"
        assert len(rule.required_hardware) == 3
        reqs_by_name = {req.name: req for req in rule.required_hardware}
        assert set(reqs_by_name.keys()) == {
            "FrankaHardwareInterface", "kassow", "dummy_lifecycle_node"}
        assert reqs_by_name["kassow"].state == LifecycleState.INACTIVE
        assert reqs_by_name["FrankaHardwareInterface"].state == LifecycleState.INACTIVE
        assert reqs_by_name["dummy_lifecycle_node"].state == LifecycleState.ACTIVE

    def test_kassow_jtc_rule(self, parsed_scenario):
        rule = next(r for r in parsed_scenario.dependency_rules if r.controller_name ==
                    "kassow_joint_trajectory_controller")
        assert rule.controller_name == "kassow_joint_trajectory_controller"
        assert len(rule.required_hardware) == 1
        assert rule.required_hardware[0].name == "kassow"
        assert rule.required_hardware[0].state == LifecycleState.ACTIVE

    def test_franka_jtc_rule(self, parsed_scenario):
        rule = next(r for r in parsed_scenario.dependency_rules if r.controller_name ==
                    "franka_joint_trajectory_controller")
        assert rule.controller_name == "franka_joint_trajectory_controller"
        assert len(rule.required_hardware) == 1
        assert rule.required_hardware[0].name == "FrankaHardwareInterface"
        assert rule.required_hardware[0].state == LifecycleState.ACTIVE


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
