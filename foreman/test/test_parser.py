from pathlib import Path

import pytest

from foreman.parser import parse_yaml_file
from foreman.types import ComponentType, LifecycleState


@pytest.fixture
def scenario_path():
    """Path to the scenario.yaml file."""
    return Path(__file__).parent.parent / "config" / "scenario.yaml"


@pytest.fixture
def parsed_scenario(scenario_path):
    """Parse the scenario.yaml file."""
    return parse_yaml_file(scenario_path)


@pytest.fixture
def autostart_scenario_path():
    """Path to the scenario_autostart.yaml file."""
    return Path(__file__).parent.parent / "config" / "scenario_autostart.yaml"


@pytest.fixture
def parsed_autostart_scenario(autostart_scenario_path):
    """Parse the scenario_autostart.yaml file."""
    return parse_yaml_file(autostart_scenario_path)


@pytest.fixture
def profile_switching_scenario_path():
    """Path to the scenario_profile_switching.yaml file."""
    return Path(__file__).parent.parent / "config" / "scenario_profile_switching.yaml"


@pytest.fixture
def parsed_profile_switching_scenario(profile_switching_scenario_path):
    """Parse the scenario_profile_switching.yaml file."""
    return parse_yaml_file(profile_switching_scenario_path)


class TestParsedScenario:
    """Tests for ParsedScenario structure."""

    def test_hardware_list(self, parsed_scenario):
        assert parsed_scenario.hardware == ["FrankaHardwareInterface", "kassow"]

    def test_lifecycle_nodes_list(self, parsed_scenario):
        assert parsed_scenario.lifecycle_nodes == ["dummy_lifecycle_node"]

    def test_metadata_empty(self, parsed_scenario):
        assert parsed_scenario.metadata == {}

    def test_autostart_profile_defaults_empty(self, parsed_scenario):
        assert parsed_scenario.autostart_profile == ""


class TestAutostartScenario:
    """Tests for a scenario configured to autostart into a profile."""

    def test_autostart_profile_is_running(self, parsed_autostart_scenario):
        assert parsed_autostart_scenario.autostart_profile == "running"

    def test_autostart_profile_is_a_declared_profile(self, parsed_autostart_scenario):
        assert parsed_autostart_scenario.autostart_profile in parsed_autostart_scenario.profiles


class TestProfileSwitchingScenario:
    """Tests for a scenario restricting which profile may follow which."""

    def test_allowed_transitions_are_parsed_per_profile(self, parsed_profile_switching_scenario):
        profiles = parsed_profile_switching_scenario.profiles
        assert profiles["idle"].allowed_transitions == ["broadcast_only"]
        assert profiles["broadcast_only"].allowed_transitions == ["running"]
        assert profiles["running"].allowed_transitions == ["broadcast_only", "running"]

    def test_profile_without_allowed_transitions_defaults_empty(self, parsed_scenario):
        for profile in parsed_scenario.profiles.values():
            assert profile.allowed_transitions == []

    def test_allowed_transitions_to_unknown_profile_raises(self, tmp_path):
        scenario = """
        hardware: [hw1]
        profiles:
          idle:
            allowed_transitions: [not_a_real_profile]
            hardware:
              hw1: inactive
        """
        scenario_path = tmp_path / "scenario_bad_allowed_transitions.yaml"
        scenario_path.write_text(scenario)

        with pytest.raises(ValueError, match="not_a_real_profile"):
            parse_yaml_file(scenario_path)


class TestDependencyRules:
    """Tests for parsed dependency rules."""

    def test_rules_count(self, parsed_scenario):
        assert len(parsed_scenario.dependency_rules) == 3

    def test_joint_state_broadcaster_rule(self, parsed_scenario):
        rule = next(
            r
            for r in parsed_scenario.dependency_rules
            if r.controller_name == "joint_state_broadcaster"
        )
        assert rule.controller_name == "joint_state_broadcaster"
        assert len(rule.required_hardware) == 3
        reqs_by_name = {req.name: req for req in rule.required_hardware}
        assert set(reqs_by_name.keys()) == {
            "FrankaHardwareInterface",
            "kassow",
            "dummy_lifecycle_node",
        }
        assert reqs_by_name["kassow"].state == LifecycleState.INACTIVE
        assert reqs_by_name["FrankaHardwareInterface"].state == LifecycleState.INACTIVE
        assert reqs_by_name["dummy_lifecycle_node"].state == LifecycleState.ACTIVE

    def test_kassow_jtc_rule(self, parsed_scenario):
        rule = next(
            r
            for r in parsed_scenario.dependency_rules
            if r.controller_name == "kassow_joint_trajectory_controller"
        )
        assert rule.controller_name == "kassow_joint_trajectory_controller"
        assert len(rule.required_hardware) == 1
        assert rule.required_hardware[0].name == "kassow"
        assert rule.required_hardware[0].state == LifecycleState.ACTIVE

    def test_franka_jtc_rule(self, parsed_scenario):
        rule = next(
            r
            for r in parsed_scenario.dependency_rules
            if r.controller_name == "franka_joint_trajectory_controller"
        )
        assert rule.controller_name == "franka_joint_trajectory_controller"
        assert len(rule.required_hardware) == 1
        assert rule.required_hardware[0].name == "FrankaHardwareInterface"
        assert rule.required_hardware[0].state == LifecycleState.ACTIVE


class TestProfiles:
    """Tests for all three profiles."""

    def test_all_profiles_present(self, parsed_scenario):
        assert set(parsed_scenario.profiles.keys()) == {"idle", "broadcast_only", "running"}

    def test_idle_profile(self, parsed_scenario):
        profile = parsed_scenario.profiles["idle"]
        assert profile.name == "idle"

        assert len(profile.hardware_targets) == 2
        hw_names = {c.name for c in profile.hardware_targets}
        assert hw_names == {"FrankaHardwareInterface", "kassow"}
        for hw in profile.hardware_targets:
            assert hw.lifecycle_state == LifecycleState.INACTIVE

        assert len(profile.controller_targets) == 3
        ctrl_names = {c.name for c in profile.controller_targets}
        assert ctrl_names == {
            "joint_state_broadcaster",
            "kassow_joint_trajectory_controller",
            "franka_joint_trajectory_controller",
        }
        for ctrl in profile.controller_targets:
            assert ctrl.lifecycle_state == LifecycleState.INACTIVE

        assert len(profile.lifecycle_node_targets) == 1
        assert profile.lifecycle_node_targets[0].name == "dummy_lifecycle_node"
        assert profile.lifecycle_node_targets[0].lifecycle_state == LifecycleState.INACTIVE

    def test_broadcast_only_profile(self, parsed_scenario):
        profile = parsed_scenario.profiles["broadcast_only"]
        assert profile.name == "broadcast_only"

        assert len(profile.hardware_targets) == 2
        for hw in profile.hardware_targets:
            assert hw.lifecycle_state == LifecycleState.ACTIVE

        ctrl_by_name = {c.name: c for c in profile.controller_targets}
        assert ctrl_by_name["joint_state_broadcaster"].lifecycle_state == LifecycleState.ACTIVE
        assert (
            ctrl_by_name["kassow_joint_trajectory_controller"].lifecycle_state
            == LifecycleState.INACTIVE
        )
        assert (
            ctrl_by_name["franka_joint_trajectory_controller"].lifecycle_state
            == LifecycleState.INACTIVE
        )

    def test_running_profile(self, parsed_scenario):
        profile = parsed_scenario.profiles["running"]
        assert profile.name == "running"

        assert len(profile.hardware_targets) == 2
        for hw in profile.hardware_targets:
            assert hw.lifecycle_state == LifecycleState.ACTIVE

        assert len(profile.controller_targets) == 3
        for ctrl in profile.controller_targets:
            assert ctrl.lifecycle_state == LifecycleState.ACTIVE

    def test_profile_component_types(self, parsed_scenario):
        idle = parsed_scenario.profiles["idle"]
        for hw in idle.hardware_targets:
            assert hw.component_type == ComponentType.HARDWARE
        for ctrl in idle.controller_targets:
            assert ctrl.component_type == ComponentType.CONTROLLER
        for lc in idle.lifecycle_node_targets:
            assert lc.component_type == ComponentType.LIFECYCLE_NODE
