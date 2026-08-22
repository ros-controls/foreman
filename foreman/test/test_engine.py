import threading

import pytest

from foreman.engine import ForemanEngine
from foreman.parser import ParsedScenario
from foreman.types import (
    Component,
    ComponentType,
    ForemanError,
    ForemanErrorCategory,
    LifecycleState,
    SystemProfile,
)


def _prepare_engine(config: ParsedScenario) -> ForemanEngine:
    """
    Engine ready for request_profile(), with every tracked component UNCONFIGURED.

    request_profile() needs at least one observation before it accepts anything --
    this establishes that baseline so a test can request a profile first, then
    simulate the state changes that follow.
    """
    engine = ForemanEngine(config, threading.Lock())

    controllers = config.tracked_components - set(config.hardware) - set(config.lifecycle_nodes)
    components = (
        [
            Component(name, ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)
            for name in config.hardware
        ]
        + [
            Component(name, ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED)
            for name in config.lifecycle_nodes
        ]
        + [
            Component(name, ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
            for name in controllers
        ]
    )
    engine.set_system_state(components)
    return engine


@pytest.fixture
def minimal_foreman_config():
    profile = SystemProfile(
        "active_profile",
        hardware_targets=[Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE)],
    )
    return ParsedScenario(
        hardware=["hw1"],
        dependency_rules=[],
        profiles={"active_profile": profile},
        tracked_components={"hw1"},
    )


def test_engine_error_and_abort(minimal_foreman_config):
    engine = _prepare_engine(minimal_foreman_config)

    ERROR_MSG = "Hardware 'hw1' rejected configuration!"

    # profile to activate comes
    response = engine.request_profile("active_profile")
    assert response.success is True
    assert engine.is_at_profile is False

    # planner wants to transition
    next_transition_command = engine.get_next_transition()
    assert next_transition_command is not None
    assert next_transition_command.goal_state == LifecycleState.INACTIVE

    # some failure happens, and we abort profile
    error = ForemanError(ForemanErrorCategory.EXECUTION, ERROR_MSG, ["hw1"])
    engine.abort_profile(error)

    # system dropped the profile due to abort
    assert engine.is_at_profile is False

    # planner outputs nothing
    assert engine.get_next_transition() is None

    # frontend will see the error and no active profile
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.message == ERROR_MSG
    assert snapshot.profile == "None"


def test_set_system_state_expected_transition(minimal_foreman_config):
    engine = _prepare_engine(minimal_foreman_config)
    engine.request_profile("active_profile")

    # verify planner issues command
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "hw1"
    assert cmd.goal_state == LifecycleState.INACTIVE

    # simulate successful expected state change via state monitor
    comp1_new = Component("hw1", ComponentType.HARDWARE, LifecycleState.INACTIVE)
    response = engine.set_system_state([comp1_new])

    # Verify the new ForemanResponse contract
    assert response.success is True
    assert response.error is None

    # verify no errors were triggered in snapshot
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False


def test_set_system_state_unexpected_downgrade(minimal_foreman_config):
    lock = threading.Lock()
    engine = ForemanEngine(minimal_foreman_config, lock)

    # start in active state
    comp1 = Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE)
    engine.set_system_state([comp1])
    engine.request_profile("active_profile")

    # verify we are at profile and no commands are active
    assert engine.is_at_profile is True
    assert engine.get_next_transition() is None

    # simulate unprompted hardware crash
    comp1_crashed = Component("hw1", ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)
    response = engine.set_system_state([comp1_crashed])

    # Verify the new ForemanResponse contract caught the error
    assert response.success is False
    assert response.error is not None
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "hw1" in response.error.component_names

    # verify error was generated correctly in snapshot
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert "hw1" in snapshot.error.components
    assert snapshot.profile == "None"

    # verify planner halts
    assert engine.get_next_transition() is None


# --- Lifecycle Node Engine Tests ---


@pytest.fixture
def lifecycle_foreman_config():
    profile = SystemProfile(
        "active_profile",
        lifecycle_node_targets=[
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
        ],
    )
    return ParsedScenario(
        hardware=[],
        dependency_rules=[],
        profiles={"active_profile": profile},
        lifecycle_nodes=["robot_manager"],
        tracked_components={"robot_manager"},
    )


def test_profile_rejects_missing_lifecycle_node(lifecycle_foreman_config):
    """Engine rejects profile if lifecycle node is not in observed state."""
    lock = threading.Lock()
    engine = ForemanEngine(lifecycle_foreman_config, lock)

    # Only report hardware, no lifecycle node in state
    engine.set_system_state([Component("some_hw", ComponentType.HARDWARE, LifecycleState.ACTIVE)])

    response = engine.request_profile("active_profile")
    assert response.success is False
    assert "robot_manager" in response.message


def test_lifecycle_node_expected_transition(lifecycle_foreman_config):
    """Engine accepts expected lifecycle node state change without error."""
    engine = _prepare_engine(lifecycle_foreman_config)
    engine.request_profile("active_profile")

    # Planner issues a command
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "robot_manager"
    assert cmd.goal_state == LifecycleState.INACTIVE

    # Simulate expected state change
    updated = Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE)
    response = engine.set_system_state([updated])
    assert response.success is True
    assert response.error is None


def test_unexpected_lifecycle_node_state_change(lifecycle_foreman_config):
    """Engine detects unexpected lifecycle node state drop."""
    lock = threading.Lock()
    engine = ForemanEngine(lifecycle_foreman_config, lock)

    # Start at profile
    active = Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
    engine.set_system_state([active])
    engine.request_profile("active_profile")
    assert engine.is_at_profile is True

    # Simulate unprompted lifecycle node crash
    crashed = Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED)
    response = engine.set_system_state([crashed])

    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "robot_manager" in response.error.component_names

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.profile == "None"


# --- Unsatisfiable Dependency Tests ---


@pytest.fixture
def dependency_config():
    """Config where controller depends on a lifecycle node being ACTIVE."""
    from foreman.types import ControllerDependencyRule
    from foreman.types import HardwareRequirement

    rules = [
        ControllerDependencyRule(
            controller_name="gripper",
            required_hardware=[HardwareRequirement("robot_manager", LifecycleState.ACTIVE)],
        )
    ]

    # Profile that requests controller active but doesn't include lifecycle node
    profile_missing_dep = SystemProfile(
        "active",
        controller_targets=[Component("gripper", ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
    )

    # Profile that properly includes the lifecycle node
    profile_with_dep = SystemProfile(
        "active_full",
        controller_targets=[Component("gripper", ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
        lifecycle_node_targets=[
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
        ],
    )

    return ParsedScenario(
        hardware=[],
        dependency_rules=rules,
        profiles={"active": profile_missing_dep, "active_full": profile_with_dep},
        lifecycle_nodes=["robot_manager"],
        tracked_components={"gripper", "robot_manager"},
    )


def test_profile_rejected_unsatisfiable_dependency(dependency_config):
    """Profile is rejected when controller dependency is not met and not in profile."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
        ]
    )

    response = engine.request_profile("active")
    assert response.success is False
    assert "gripper" in response.message
    assert "robot_manager" in response.message


def test_profile_accepted_when_dependency_in_profile(dependency_config):
    """Profile is accepted when dependency is included in profile targets."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
        ]
    )

    response = engine.request_profile("active_full")
    assert response.success is True


def test_profile_accepted_when_dependency_already_satisfied(dependency_config):
    """Profile is accepted when dependency is already at required state."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE),
        ]
    )

    response = engine.request_profile("active")
    assert response.success is True


# --- Snapshot Profile Availability Tests ---
def test_snapshot_available_profiles_empty_before_ready(minimal_foreman_config):
    """Snapshot reports no available profiles before the first observed state."""
    lock = threading.Lock()
    engine = ForemanEngine(minimal_foreman_config, lock)

    snapshot = engine.get_engine_snapshot()
    assert snapshot.all_profiles == ["active_profile"]
    assert snapshot.available_profiles == []


def test_snapshot_available_profiles_reflects_dependency_satisfaction(dependency_config):
    """Available profiles narrow to what's achievable, e.g. after a tool change."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    # robot_manager not yet active: "active" is unsatisfiable, "active_full" is not.
    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
        ]
    )
    snapshot = engine.get_engine_snapshot()
    assert set(snapshot.all_profiles) == {"active", "active_full"}
    assert snapshot.available_profiles == ["active_full"]

    # robot_manager now active (e.g. after a tool change): "active" becomes achievable too.
    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE),
        ]
    )
    snapshot = engine.get_engine_snapshot()
    assert set(snapshot.available_profiles) == {"active", "active_full"}
