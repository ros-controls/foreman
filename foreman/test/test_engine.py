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

    # a rejected command shouldn't be retried until explicitly re-requested
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

    # still driving toward "active_profile" -- one more step needed (INACTIVE -> ACTIVE)
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.profile == "active_profile"
    assert snapshot.at_profile is False

    # last step: reaches the target
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.goal_state == LifecycleState.ACTIVE
    comp1_active = Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE)
    engine.set_system_state([comp1_active])

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.profile == "active_profile"
    assert snapshot.at_profile is True


def test_when_hardware_error_and_controller_can_not_transition_mid_transition_expect_error_state_and_none_state(
    hardware_and_controller_config,
):
    """An unexpected change to a component not being driven is still an error."""
    engine = _prepare_engine(hardware_and_controller_config)

    # hw1 already active and settled; ctrl1 configured, one step from active
    engine.set_system_state(
        [
            Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE),
            Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        ]
    )
    engine.request_profile("running")

    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "ctrl1"
    assert cmd.goal_state == LifecycleState.ACTIVE

    # hw1 crashes -- unrelated to what's being driven. ctrl1 can't activate
    # without it and stays at INACTIVE, short of its commanded goal.
    response = engine.set_system_state(
        [
            Component("hw1", ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
            Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        ]
    )

    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "hw1" in response.error.component_names
    assert "ctrl1" in response.error.component_names
    assert engine._current_profile.name == "running"
    assert engine.get_next_transition() is not None
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    components_by_name = {c.name: c for c in snapshot.components}
    assert components_by_name["hw1"].lifecycle_state == LifecycleState.UNCONFIGURED
    assert components_by_name["ctrl1"].lifecycle_state == LifecycleState.INACTIVE


def test_set_system_state_unexpected_downgrade(minimal_foreman_config):
    """Once the target is reached, a later crash is flagged but not auto-recovered."""
    engine = _prepare_engine(minimal_foreman_config)

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

    # at_profile was already reached once -- Foreman doesn't drive back
    # on its own; only a fresh request would resume driving
    assert engine.get_next_transition() is None

    # hw1 comes back up on its own -- matches its profile target, error clears
    engine.set_system_state([comp1])
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.profile == "active_profile"
    assert snapshot.at_profile is True


@pytest.fixture
def hardware_and_controller_config():
    running = SystemProfile(
        "running",
        hardware_targets=[Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE)],
        controller_targets=[Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
    )
    all_inactive = SystemProfile(
        "all_inactive",
        hardware_targets=[Component("hw1", ComponentType.HARDWARE, LifecycleState.INACTIVE)],
        controller_targets=[Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.INACTIVE)],
    )
    return ParsedScenario(
        hardware=["hw1"],
        dependency_rules=[],
        profiles={"running": running, "all_inactive": all_inactive},
        tracked_components={"hw1", "ctrl1"},
    )


def test_when_hardware_and_controller_recover_separately_expect_error_and_known_state_when_valid_and_error_and_none_when_invalid_all_ok_when_both_reach_target_profile(
    hardware_and_controller_config,
):
    """
    Profile returns only once every tracked component matches again.

    Deactivating hw1 takes ctrl1 down with it, as controller_manager would.
    The resulting state happens to match a different configured profile
    ("all_inactive"), but it's still flagged as an error, since nobody
    requested it. Reactivating hw1 alone is not enough: the profile stays
    'None' until ctrl1 is reactivated too, at which point the error also
    clears on its own -- no request_profile() call after the initial one,
    every change here comes from outside Foreman. Re-requesting the same
    target profile doesn't clear the error either, since it's still
    unsatisfied -- only the components actually matching it does.
    """
    engine = _prepare_engine(hardware_and_controller_config)
    engine.request_profile("running")

    # both reach "running" directly -- matching the target is expected,
    # regardless of the exact commanded step
    hw1_active = Component("hw1", ComponentType.HARDWARE, LifecycleState.ACTIVE)
    ctrl1_active = Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.ACTIVE)
    response = engine.set_system_state([hw1_active, ctrl1_active])
    snapshot = engine.get_engine_snapshot()
    assert snapshot.profile == "running"
    assert snapshot.error.is_error is False
    assert response.success is True

    # hw1 deactivated directly, taking ctrl1 down with it -- lands on a known
    # profile ("all_inactive"), but it's still unexpected: nobody requested it
    hw1_inactive = Component("hw1", ComponentType.HARDWARE, LifecycleState.INACTIVE)
    ctrl1_inactive = Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    response = engine.set_system_state([hw1_inactive, ctrl1_inactive])
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.profile == "all_inactive"
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    # "all_inactive" is a complete, valid profile -- Foreman doesn't fight
    # a deliberate manual switch by driving back toward "running"
    assert engine.get_next_transition() is None

    # hw1 reactivated alone -- ctrl1 is still inactive, profile stays "None"
    engine.set_system_state([hw1_active, ctrl1_inactive])
    snapshot = engine.get_engine_snapshot()
    assert snapshot.profile == "None"
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert snapshot.error.components == ["ctrl1"]

    # ctrl1 reactivated too -- both match "running" again, profile and error recover
    engine.set_system_state([hw1_active, ctrl1_active])
    snapshot = engine.get_engine_snapshot()
    assert snapshot.profile == "running"
    assert snapshot.error.is_error is False
    assert snapshot.error.components == []

    # both drop again, unexpectedly
    response = engine.set_system_state([hw1_inactive, ctrl1_inactive])
    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.profile == "all_inactive"
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    # explicit re-request doesn't clear an UNEXPECTED_STATE error either --
    # only the live state actually matching the target does
    response = engine.request_profile("running")
    assert response.success is True
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}
    assert snapshot.profile == "all_inactive"

    # the request itself is what resumes driving, since it hasn't reached
    # "running" again yet -- it doesn't just sit there re-flagging the error
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "hw1"

    # both reach "running" directly -- matching the target is expected
    engine.set_system_state([hw1_active, ctrl1_active])
    snapshot = engine.get_engine_snapshot()
    assert snapshot.profile == "running"
    assert snapshot.at_profile is True


def test_when_requesting_profile_while_parked_at_a_different_valid_profile_expect_driving_starts(
    hardware_and_controller_config,
):
    """A profile request drives toward its target, even starting from a different valid one."""
    engine = _prepare_engine(hardware_and_controller_config)

    # parked at "all_inactive" -- nobody has requested anything yet
    engine.set_system_state(
        [
            Component("hw1", ComponentType.HARDWARE, LifecycleState.INACTIVE),
            Component("ctrl1", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        ]
    )
    assert engine.get_engine_snapshot().profile == "all_inactive"

    # requesting "running" drives toward it, not blocked by starting at
    # a different, valid, complete profile
    engine.request_profile("running")
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "hw1"
    assert cmd.goal_state == LifecycleState.ACTIVE


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

    # still driving toward "active_profile" -- one more step needed (INACTIVE -> ACTIVE)
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.profile == "active_profile"
    assert snapshot.at_profile is False

    # last step: reaches the target
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.goal_state == LifecycleState.ACTIVE
    active = Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
    engine.set_system_state([active])

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.profile == "active_profile"
    assert snapshot.at_profile is True


def test_unexpected_lifecycle_node_state_change(lifecycle_foreman_config):
    """Engine detects unexpected lifecycle node state drop."""
    engine = _prepare_engine(lifecycle_foreman_config)

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
