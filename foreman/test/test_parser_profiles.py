import pytest
from pathlib import Path

from foreman.parser import parse_yaml_file
from foreman.types import (
    ComponentType,
    LifecycleState,
)


@pytest.fixture
def profiles_path():
    """Path to the profiles example config used by these tests."""
    return Path(__file__).parent / "test_parser_profiles_config.yaml"


@pytest.fixture
def parsed_profiles(profiles_path):
    """Parse the profiles example config."""
    return parse_yaml_file(profiles_path)


def _by_name(components):
    """Map component name -> Component for easy lookup in assertions."""
    return {c.name: c for c in components}


class TestProfileScenario:
    """Tests for the overall ParsedScenario built from profiles."""

    def test_profiles_become_goals(self, parsed_profiles):
        assert set(parsed_profiles.goals.keys()) == {"base", "running"}

    def test_no_dependency_rules(self, parsed_profiles):
        # profiles list hardware explicitly, so ordering is by planner priority
        assert parsed_profiles.dependency_rules == []

    def test_metadata_empty(self, parsed_profiles):
        # profile keys must not leak into metadata
        assert parsed_profiles.metadata == {}

    def test_tracked_components_cover_the_universe(self, parsed_profiles):
        expected = {
            "joint_state_broadcaster",
            "forward_position_controller",
            "RRBot",
            "robot_manager",
        }
        assert expected <= parsed_profiles.tracked_components


class TestProfileExpansion:
    """Tests for how each profile expands into a complete SystemGoal."""

    def test_listed_components_are_active(self, parsed_profiles):
        goal = parsed_profiles.goals["running"]
        ctrl = _by_name(goal.controller_goals)
        hw = _by_name(goal.hardware_goals)
        lc = _by_name(goal.lifecycle_node_goals)
        assert ctrl["joint_state_broadcaster"].lifecycle_state == LifecycleState.ACTIVE
        assert ctrl["forward_position_controller"].lifecycle_state == LifecycleState.ACTIVE
        assert hw["RRBot"].lifecycle_state == LifecycleState.ACTIVE
        assert lc["robot_manager"].lifecycle_state == LifecycleState.ACTIVE

    def test_unlisted_components_are_deactivated(self, parsed_profiles):
        goal = parsed_profiles.goals["base"]
        ctrl = _by_name(goal.controller_goals)
        # forward_position_controller is in the universe (via running) but not in base
        assert ctrl["forward_position_controller"].lifecycle_state == LifecycleState.INACTIVE

    def test_goals_are_complete_target_states(self, parsed_profiles):
        goal = parsed_profiles.goals["base"]
        ctrl_names = {c.name for c in goal.controller_goals}
        assert ctrl_names == {"joint_state_broadcaster", "forward_position_controller"}

    def test_lifecycle_nodes_supported(self, parsed_profiles):
        goal = parsed_profiles.goals["running"]
        lc = _by_name(goal.lifecycle_node_goals)
        assert lc["robot_manager"].lifecycle_state == LifecycleState.ACTIVE

    def test_goal_component_types(self, parsed_profiles):
        goal = parsed_profiles.goals["running"]
        for hw in goal.hardware_goals:
            assert hw.component_type == ComponentType.HARDWARE
        for ctrl in goal.controller_goals:
            assert ctrl.component_type == ComponentType.CONTROLLER
        for lc in goal.lifecycle_node_goals:
            assert lc.component_type == ComponentType.LIFECYCLE_NODE
