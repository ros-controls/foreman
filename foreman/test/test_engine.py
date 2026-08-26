import threading

import pytest

from foreman.engine import ForemanEngine
from foreman.parser import ParsedScenario
from foreman.types import (
    Component,
    ComponentType,
    ControllerDependencyRule,
    ForemanError,
    ForemanErrorCategory,
    HardwareRequirement,
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


def _state(
    hw1=LifecycleState.UNCONFIGURED,
    ctrl1=LifecycleState.UNCONFIGURED,
    lc1=LifecycleState.UNCONFIGURED,
):
    """
    Build the component list set_system_state() expects.

    set_system_state() replaces the observed state wholesale, not merges it --
    this always states hw1/ctrl1/lc1 explicitly so a test can't accidentally
    drop one. Pass lc1=None to omit it from observation entirely (e.g.
    simulating it hasn't reported in yet).
    """
    components = [
        Component("hw1", ComponentType.HARDWARE, hw1),
        Component("ctrl1", ComponentType.CONTROLLER, ctrl1),
    ]
    if lc1 is not None:
        components.append(Component("lc1", ComponentType.LIFECYCLE_NODE, lc1))
    return components


@pytest.fixture
def foreman_config():
    """
    Standard test scenario: hw1, ctrl1 (requires hw1 active), lc1.

    Mirrors config/scenario_integration_test.yaml's shape, so engine and
    integration tests describe the same system. Profiles:
    - active / all_inactive / idle: every component at one state.
    - ros2_control_active / ros2_control_inactive: hw1 + ctrl1 only, lc1
      untargeted -- each is also a subset of the matching active/all_inactive
      profile, giving an overlapping-profile pair for free.
    - ctrl1_active_only: ctrl1 active without targeting hw1 -- deliberately
      unsatisfiable unless hw1 is already active externally.
    """

    def profile(name, hw=None, ctrl=None, lc=None):
        return SystemProfile(
            name,
            hardware_targets=(
                [Component("hw1", ComponentType.HARDWARE, hw)] if hw is not None else []
            ),
            controller_targets=(
                [Component("ctrl1", ComponentType.CONTROLLER, ctrl)] if ctrl is not None else []
            ),
            lifecycle_node_targets=(
                [Component("lc1", ComponentType.LIFECYCLE_NODE, lc)] if lc is not None else []
            ),
        )

    # ros2_control_active/_inactive listed before their supersets, deliberately --
    # needed so the overlap regression test below can actually detect a revert.
    profiles = {
        "idle": profile(
            "idle",
            LifecycleState.UNCONFIGURED,
            LifecycleState.UNCONFIGURED,
            LifecycleState.UNCONFIGURED,
        ),
        "ros2_control_active": profile(
            "ros2_control_active", LifecycleState.ACTIVE, LifecycleState.ACTIVE
        ),
        "ros2_control_inactive": profile(
            "ros2_control_inactive", LifecycleState.INACTIVE, LifecycleState.INACTIVE
        ),
        "active": profile(
            "active", LifecycleState.ACTIVE, LifecycleState.ACTIVE, LifecycleState.ACTIVE
        ),
        "all_inactive": profile(
            "all_inactive",
            LifecycleState.INACTIVE,
            LifecycleState.INACTIVE,
            LifecycleState.INACTIVE,
        ),
        "ctrl1_active_only": profile("ctrl1_active_only", ctrl=LifecycleState.ACTIVE),
    }

    return ParsedScenario(
        hardware=["hw1"],
        lifecycle_nodes=["lc1"],
        dependency_rules=[
            ControllerDependencyRule(
                controller_name="ctrl1",
                required_hardware=[HardwareRequirement("hw1", LifecycleState.ACTIVE)],
            )
        ],
        profiles=profiles,
        tracked_components={"hw1", "ctrl1", "lc1"},
    )


def test_when_execution_error_aborts_profile_expect_target_cleared_and_error_reported(
    foreman_config,
):
    engine = _prepare_engine(foreman_config)
    ERROR_MSG = "Hardware 'hw1' rejected configuration!"

    response = engine.request_profile("active")
    assert response.success is True
    assert engine.is_at_profile is False

    next_transition_command = engine.get_next_transition()
    assert next_transition_command is not None
    assert next_transition_command.component.name == "hw1"
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
    assert snapshot.target_profile == "None"
    assert snapshot.current_profile == "idle"


def test_when_state_matches_commanded_transitions_expect_no_error_until_target_reached(
    foreman_config,
):
    """Expected, planner-driven transitions never trigger an anomaly, all the way to the target."""
    engine = _prepare_engine(foreman_config)
    engine.request_profile("active")

    state = {
        "hw1": LifecycleState.UNCONFIGURED,
        "ctrl1": LifecycleState.UNCONFIGURED,
        "lc1": LifecycleState.UNCONFIGURED,
    }
    while not engine.is_at_profile:
        cmd = engine.get_next_transition()
        assert cmd is not None
        state[cmd.component.name] = cmd.goal_state
        response = engine.set_system_state(_state(**state))
        assert response.success is True
        assert response.error is None

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.target_profile == "active"
    assert snapshot.current_profile == "active"


def test_when_hardware_error_and_controller_can_not_transition_mid_transition_expect_error_state_and_none_state(
    foreman_config,
):
    """An unexpected change to a component not being driven is still an error."""
    engine = _prepare_engine(foreman_config)

    # hw1 already active and settled; ctrl1 configured, one step from active
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.INACTIVE))
    engine.request_profile("ros2_control_active")

    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "ctrl1"
    assert cmd.goal_state == LifecycleState.ACTIVE

    # hw1 crashes -- unrelated to what's being driven. ctrl1 can't activate
    # without it and stays at INACTIVE, short of its commanded goal.
    response = engine.set_system_state(
        _state(hw1=LifecycleState.UNCONFIGURED, ctrl1=LifecycleState.INACTIVE)
    )

    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "hw1" in response.error.component_names
    assert "ctrl1" in response.error.component_names
    assert engine._target_profile.name == "ros2_control_active"
    assert engine.get_next_transition() is not None
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "None"

    components_by_name = {c.name: c for c in snapshot.components}
    assert components_by_name["hw1"].lifecycle_state == LifecycleState.UNCONFIGURED
    assert components_by_name["ctrl1"].lifecycle_state == LifecycleState.INACTIVE

    # both externally settle at "ros2_control_inactive" -- a known, valid
    # profile, but still not "ros2_control_active": the error stays, just recomputed
    engine.set_system_state(_state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_inactive"
    assert snapshot.error.is_error is True
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    # both externally reach "ros2_control_active" -- the targeted profile -- error clears
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_active"
    assert snapshot.error.is_error is False
    assert engine.is_at_profile is True


def test_when_target_already_reached_expect_later_unprompted_crash_flagged_but_not_auto_recovered(
    foreman_config,
):
    """Once the target is reached, a later crash is flagged but not auto-recovered."""
    engine = _prepare_engine(foreman_config)

    # start already at the ros2_control_active target
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    engine.request_profile("ros2_control_active")

    assert engine.is_at_profile is True
    assert engine.get_next_transition() is None

    # simulate unprompted hardware crash
    response = engine.set_system_state(
        _state(hw1=LifecycleState.UNCONFIGURED, ctrl1=LifecycleState.ACTIVE)
    )

    assert response.success is False
    assert response.error is not None
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "hw1" in response.error.component_names

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert "hw1" in snapshot.error.components
    assert snapshot.target_profile == "ros2_control_active"
    # "ros2_control_active" no longer matches (hw1 crashed), but ctrl1 alone
    # still satisfies "ctrl1_active_only", which doesn't target hw1 at all
    assert snapshot.current_profile == "ctrl1_active_only"

    # the target was already reached once -- Foreman doesn't drive back
    # on its own; only a fresh request would resume driving
    assert engine.get_next_transition() is None

    # hw1 comes back up on its own -- matches its profile target, error clears
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_active"
    assert engine.is_at_profile is True


def test_when_hardware_and_controller_recover_separately_expect_error_and_known_state_when_valid_and_error_and_none_when_invalid_all_ok_when_both_reach_target_profile(
    foreman_config,
):
    """
    Profile returns only once every tracked component matches again.

    Deactivating hw1 takes ctrl1 down with it, as controller_manager would.
    The resulting state happens to match a different configured profile
    ("ros2_control_inactive"), but it's still flagged as an error, since
    nobody requested it. Reactivating hw1 alone is not enough: the profile
    stays 'None' until ctrl1 is reactivated too, at which point the error
    also clears on its own -- no request_profile() call after the initial
    one, every change here comes from outside Foreman. Re-requesting the
    same target profile doesn't clear the error either, since it's still
    unsatisfied -- only the components actually matching it does.
    """
    engine = _prepare_engine(foreman_config)
    engine.request_profile("ros2_control_active")

    # both reach "ros2_control_active" directly -- matching the target is
    # expected, regardless of the exact commanded step
    response = engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE)
    )
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_active"
    assert snapshot.error.is_error is False
    assert response.success is True

    # hw1 deactivated directly, taking ctrl1 down with it -- lands on a known
    # profile ("ros2_control_inactive"), but it's still unexpected: nobody requested it
    response = engine.set_system_state(
        _state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE)
    )
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_inactive"
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    # "ros2_control_inactive" is a complete, valid profile -- Foreman doesn't
    # fight a deliberate manual switch by driving back toward "ros2_control_active"
    assert engine.get_next_transition() is None

    # hw1 reactivated alone -- ctrl1 is still inactive, current_profile stays "None"
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.INACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "None"
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert snapshot.error.components == ["ctrl1"]

    # ctrl1 reactivated too -- both match "ros2_control_active" again, profile and error recover
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_active"
    assert snapshot.error.is_error is False
    assert snapshot.error.components == []

    # both drop again, unexpectedly
    response = engine.set_system_state(
        _state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE)
    )
    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_inactive"
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}

    # explicit re-request doesn't clear an UNEXPECTED_STATE error either --
    # only the live state actually matching the target does
    response = engine.request_profile("ros2_control_active")
    assert response.success is True
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert set(snapshot.error.components) == {"hw1", "ctrl1"}
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_inactive"

    # the request itself is what resumes driving, since it hasn't reached
    # "ros2_control_active" again yet -- it doesn't just sit there re-flagging the error
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "hw1"

    # both reach "ros2_control_active" directly -- matching the target is expected
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "ros2_control_active"
    assert snapshot.current_profile == "ros2_control_active"
    assert engine.is_at_profile is True


def test_when_requesting_profile_while_parked_at_a_different_valid_profile_expect_driving_starts(
    foreman_config,
):
    """A profile request drives toward its target, even starting from a different valid one."""
    engine = _prepare_engine(foreman_config)

    # parked at "ros2_control_inactive" -- nobody has requested anything yet
    engine.set_system_state(_state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "None"
    assert snapshot.current_profile == "ros2_control_inactive"

    # requesting "ros2_control_active" drives toward it, not blocked by
    # starting at a different, valid, complete profile -- hw1 must activate
    # first, since ctrl1 can't activate until hw1 already is
    engine.request_profile("ros2_control_active")
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "hw1"
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_when_profile_omits_a_tracked_component_expect_its_state_ignored_for_matching(
    foreman_config,
):
    """A tracked component not listed in a profile's targets doesn't affect matching it."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.UNCONFIGURED))
    engine.request_profile("ros2_control_active")

    # ctrl1 isn't active yet -- not at profile, and lc1 isn't targeted at all
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "ctrl1"

    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert snapshot.current_profile == "ros2_control_active"
    assert engine.is_at_profile is True

    # lc1 isn't part of "ros2_control_active"'s targets -- its state doesn't
    # affect whether the profile matches, but an unprompted change is still flagged
    response = engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.ACTIVE)
    )
    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    snapshot = engine.get_engine_snapshot()
    assert snapshot.current_profile == "ros2_control_active"
    assert engine.is_at_profile is True


def test_when_targeted_component_becomes_finalized_expect_current_profile_falls_back_to_a_matching_untargeted_profile(
    foreman_config,
):
    """A component reporting FINALIZED doesn't count toward matching a profile that targets it."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.ACTIVE)
    )
    engine.request_profile("active")
    assert engine.get_engine_snapshot().current_profile == "active"

    # lc1 disconnects -- reported as FINALIZED, the same way component_state_monitor
    # does when its transition_event publisher disappears
    engine.set_system_state(
        _state(
            hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.FINALIZED
        )
    )
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "active"
    # "active" no longer matches (lc1 isn't ACTIVE), but hw1+ctrl1 alone
    # still satisfy "ros2_control_active", which doesn't target lc1 at all
    assert snapshot.current_profile == "ros2_control_active"


def test_when_target_profile_is_a_superset_of_an_earlier_configured_profile_expect_it_still_matches(
    foreman_config,
):
    """A matching target profile must win over a narrower, earlier-configured one."""
    engine = _prepare_engine(foreman_config)

    # "ros2_control_active" (narrower, configured first) also matches this state --
    # "active" (the requested superset, configured later) must still win
    engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.ACTIVE)
    )

    response = engine.request_profile("active")
    assert response.success is True
    snapshot = engine.get_engine_snapshot()
    assert snapshot.target_profile == "active"
    assert snapshot.current_profile == "active"
    assert engine.is_at_profile is True


def test_when_lifecycle_node_missing_from_observed_state_expect_profile_rejected(foreman_config):
    """Engine rejects a profile if a component it targets is not in observed state."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=None)
    )

    response = engine.request_profile("active")
    assert response.success is False
    assert "lc1" in response.message


def test_when_lifecycle_node_state_change_matches_commanded_transition_expect_no_error(
    foreman_config,
):
    """Engine accepts an expected lifecycle node state change without error."""
    engine = _prepare_engine(foreman_config)
    # hw1/ctrl1 already settled at their target -- this test's focus is lc1's own transition
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE))
    engine.request_profile("active")

    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == "lc1"
    assert cmd.goal_state == LifecycleState.INACTIVE

    response = engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.INACTIVE)
    )
    assert response.success is True
    assert response.error is None
    assert engine.is_at_profile is False

    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.goal_state == LifecycleState.ACTIVE
    engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.ACTIVE)
    )

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is False
    assert snapshot.target_profile == "active"
    assert snapshot.current_profile == "active"
    assert engine.is_at_profile is True


def test_when_lifecycle_node_state_drops_unprompted_expect_error(foreman_config):
    """Engine detects an unexpected lifecycle node state drop."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(
        _state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.ACTIVE)
    )
    engine.request_profile("active")
    assert engine.is_at_profile is True

    # simulate unprompted lifecycle node crash
    response = engine.set_system_state(
        _state(
            hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.ACTIVE, lc1=LifecycleState.UNCONFIGURED
        )
    )

    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert "lc1" in response.error.component_names

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.target_profile == "active"
    # "active" no longer matches (lc1 crashed), but hw1+ctrl1 alone still
    # satisfy "ros2_control_active", which doesn't target lc1 at all
    assert snapshot.current_profile == "ros2_control_active"


def test_when_dependency_not_met_and_not_in_profile_expect_profile_rejected(foreman_config):
    """Profile is rejected when a controller dependency isn't met and isn't in the profile."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(_state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE))

    response = engine.request_profile("ctrl1_active_only")
    assert response.success is False
    assert "ctrl1" in response.message
    assert "hw1" in response.message


def test_when_dependency_included_in_profile_expect_profile_accepted(foreman_config):
    """Profile is accepted when its dependency is included in the profile's own targets."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(_state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE))

    response = engine.request_profile("active")
    assert response.success is True


def test_when_dependency_already_satisfied_externally_expect_profile_accepted(foreman_config):
    """Profile is accepted when its dependency is already met in the live state."""
    engine = _prepare_engine(foreman_config)
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.INACTIVE))

    response = engine.request_profile("ctrl1_active_only")
    assert response.success is True


def test_when_controller_depends_on_lifecycle_node_expect_dependency_enforced():
    """
    _check_unsatisfiable_dependencies() treats a lifecycle-node dependency like a
    hardware one -- exercised with its own scenario, since foreman_config's own
    ctrl1 -> hw1 dependency never reaches the lifecycle-node branch.
    """
    profile_missing_dep = SystemProfile(
        "active",
        controller_targets=[Component("gripper", ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
    )
    profile_with_dep = SystemProfile(
        "active_full",
        controller_targets=[Component("gripper", ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
        lifecycle_node_targets=[
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
        ],
    )
    config = ParsedScenario(
        hardware=[],
        dependency_rules=[
            ControllerDependencyRule(
                controller_name="gripper",
                required_hardware=[HardwareRequirement("robot_manager", LifecycleState.ACTIVE)],
            )
        ],
        profiles={"active": profile_missing_dep, "active_full": profile_with_dep},
        lifecycle_nodes=["robot_manager"],
        tracked_components={"gripper", "robot_manager"},
    )
    engine = _prepare_engine(config)
    engine.set_system_state(
        [
            Component("gripper", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            Component("robot_manager", ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
        ]
    )

    # rejected: robot_manager isn't active, and "active" doesn't target it
    response = engine.request_profile("active")
    assert response.success is False
    assert "gripper" in response.message
    assert "robot_manager" in response.message

    # accepted: "active_full" targets robot_manager itself
    response = engine.request_profile("active_full")
    assert response.success is True


def test_when_not_ready_expect_snapshot_reports_no_available_profiles(foreman_config):
    """Snapshot reports no available profiles before the first observed state."""
    lock = threading.Lock()
    engine = ForemanEngine(foreman_config, lock)

    snapshot = engine.get_engine_snapshot()
    assert set(snapshot.all_profiles) == {
        "active",
        "all_inactive",
        "idle",
        "ros2_control_active",
        "ros2_control_inactive",
        "ctrl1_active_only",
    }
    assert snapshot.available_profiles == []


def test_when_dependency_satisfaction_changes_expect_available_profiles_updates(foreman_config):
    """Available profiles narrow to what's achievable, e.g. after a tool change."""
    engine = _prepare_engine(foreman_config)

    # hw1 not yet active: "ctrl1_active_only" is unsatisfiable, "active" is not
    engine.set_system_state(_state(hw1=LifecycleState.INACTIVE, ctrl1=LifecycleState.INACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert "ctrl1_active_only" not in snapshot.available_profiles
    assert "active" in snapshot.available_profiles

    # hw1 now active (e.g. after a tool change): "ctrl1_active_only" becomes achievable too
    engine.set_system_state(_state(hw1=LifecycleState.ACTIVE, ctrl1=LifecycleState.INACTIVE))
    snapshot = engine.get_engine_snapshot()
    assert "ctrl1_active_only" in snapshot.available_profiles


def test_when_state_observed_expect_response_reports_missing_configured_components(
    foreman_config,
):
    """Every set_system_state() call reports configured components absent from observed state."""
    engine = ForemanEngine(foreman_config, threading.Lock())

    response = engine.set_system_state([])
    assert response.missing_components == ["ctrl1", "hw1", "lc1"]

    # a component absent from observed state, e.g. a scenario.yaml name typo
    response = engine.set_system_state(_state(lc1=None))
    assert response.missing_components == ["lc1"]

    response = engine.set_system_state(_state())
    assert response.missing_components == []
