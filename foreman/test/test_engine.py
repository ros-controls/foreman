import pytest
import threading
from foreman.engine import ForemanEngine
from foreman.types import Component, ComponentType, LifecycleState, SystemGoal, ForemanError, ForemanErrorCategory
from foreman.parser import ParsedScenario

@pytest.fixture
def minimal_foreman_config():
    goal = SystemGoal('active_goal', 
                      hardware_goals=[Component('hw1', ComponentType.HARDWARE, LifecycleState.ACTIVE)])
    return ParsedScenario(
        controller_manager="test_cm",
        transition_pause=0.0,
        hardware=["hw1"],
        dependency_rules=[],
        goals={'active_goal': goal}
    )

def test_engine_error_and_abort(minimal_foreman_config):
    lock = threading.Lock()
    engine = ForemanEngine(minimal_foreman_config, lock)
    
    ERROR_MSG = "Hardware 'hw1' rejected configuration!"

    # initialize
    initial_components = [Component('hw1', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)]
    response = engine.set_system_state(initial_components)
    assert response.success is True
    
    # goal to activate comes
    response = engine.request_goal('active_goal')
    assert response.success is True
    assert engine.is_at_goal is False
    
    # planner wants to transition
    next_transition_command = engine.get_next_transition()
    assert next_transition_command is not None
    assert next_transition_command.goal_state == LifecycleState.INACTIVE
    
    # some failure happens, and we abort goal
    error = ForemanError(
        ForemanErrorCategory.EXECUTION, 
        ERROR_MSG, 
        ['hw1']
    )
    engine.abort_goal(error)
    
    # system dropped the goal due to abort
    assert engine.is_at_goal is False 
    
    # planner outputs nothing
    assert engine.get_next_transition() is None
    
    # frontend will see the error and no active goal
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.message == ERROR_MSG
    assert snapshot.goal == 'None'

def test_set_system_state_expected_transition(minimal_foreman_config):
    lock = threading.Lock()
    engine = ForemanEngine(minimal_foreman_config, lock)
    
    # initialize unconfigured
    comp1 = Component('hw1', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)
    engine.set_system_state([comp1])
    engine.request_goal('active_goal')
    
    # verify planner issues command
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == 'hw1'
    assert cmd.goal_state == LifecycleState.INACTIVE
    
    # simulate successful expected state change via state monitor
    comp1_new = Component('hw1', ComponentType.HARDWARE, LifecycleState.INACTIVE)
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
    comp1 = Component('hw1', ComponentType.HARDWARE, LifecycleState.ACTIVE)
    engine.set_system_state([comp1])
    engine.request_goal('active_goal')
    
    # verify we are at goal and no commands are active
    assert engine.is_at_goal is True
    assert engine.get_next_transition() is None
    
    # simulate unprompted hardware crash
    comp1_crashed = Component('hw1', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)
    response = engine.set_system_state([comp1_crashed])
    
    # Verify the new ForemanResponse contract caught the error
    assert response.success is False
    assert response.error is not None
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert 'hw1' in response.error.component_names
    
    # verify error was generated correctly in snapshot
    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.error.category == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert 'hw1' in snapshot.error.components
    assert snapshot.goal == 'None'
    
    # verify planner halts
    assert engine.get_next_transition() is None


# --- Lifecycle Node Engine Tests ---

@pytest.fixture
def lifecycle_foreman_config():
    goal = SystemGoal('active_goal',
                      lifecycle_node_goals=[Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)])
    return ParsedScenario(
        controller_manager="test_cm",
        transition_pause=0.0,
        hardware=[],
        dependency_rules=[],
        goals={'active_goal': goal},
        lifecycle_nodes=["robot_manager"]
    )


def test_goal_rejects_missing_lifecycle_node(lifecycle_foreman_config):
    """Engine rejects goal if lifecycle node is not in observed state."""
    lock = threading.Lock()
    engine = ForemanEngine(lifecycle_foreman_config, lock)

    # Only report hardware, no lifecycle node in state
    engine.set_system_state([Component('some_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)])

    response = engine.request_goal('active_goal')
    assert response.success is False
    assert 'robot_manager' in response.message


def test_lifecycle_node_expected_transition(lifecycle_foreman_config):
    """Engine accepts expected lifecycle node state change without error."""
    lock = threading.Lock()
    engine = ForemanEngine(lifecycle_foreman_config, lock)

    initial = Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED)
    engine.set_system_state([initial])
    engine.request_goal('active_goal')

    # Planner issues a command
    cmd = engine.get_next_transition()
    assert cmd is not None
    assert cmd.component.name == 'robot_manager'
    assert cmd.goal_state == LifecycleState.INACTIVE

    # Simulate expected state change
    updated = Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE)
    response = engine.set_system_state([updated])
    assert response.success is True
    assert response.error is None


def test_unexpected_lifecycle_node_state_change(lifecycle_foreman_config):
    """Engine detects unexpected lifecycle node state drop."""
    lock = threading.Lock()
    engine = ForemanEngine(lifecycle_foreman_config, lock)

    # Start at goal
    active = Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)
    engine.set_system_state([active])
    engine.request_goal('active_goal')
    assert engine.is_at_goal is True

    # Simulate unprompted lifecycle node crash
    crashed = Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED)
    response = engine.set_system_state([crashed])

    assert response.success is False
    assert response.error.category == ForemanErrorCategory.UNEXPECTED_STATE
    assert 'robot_manager' in response.error.component_names

    snapshot = engine.get_engine_snapshot()
    assert snapshot.error.is_error is True
    assert snapshot.goal == 'None'


# --- Unsatisfiable Dependency Tests ---

@pytest.fixture
def dependency_config():
    """Config where controller depends on a lifecycle node being ACTIVE."""
    from foreman.types import ControllerDependencyRule, HardwareRequirement

    rules = [
        ControllerDependencyRule(
            controller_name='gripper',
            required_hardware=[HardwareRequirement('robot_manager', LifecycleState.ACTIVE)]
        )
    ]

    # Goal that requests controller active but doesn't include lifecycle node
    goal_missing_dep = SystemGoal('active',
        controller_goals=[Component('gripper', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    # Goal that properly includes the lifecycle node
    goal_with_dep = SystemGoal('active_full',
        controller_goals=[Component('gripper', ComponentType.CONTROLLER, LifecycleState.ACTIVE)],
        lifecycle_node_goals=[Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)])

    return ParsedScenario(
        controller_manager="test_cm",
        transition_pause=0.0,
        hardware=[],
        dependency_rules=rules,
        goals={'active': goal_missing_dep, 'active_full': goal_with_dep},
        lifecycle_nodes=["robot_manager"]
    )


def test_goal_rejected_unsatisfiable_dependency(dependency_config):
    """Goal is rejected when controller dependency is not met and not in goal."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state([
        Component('gripper', ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
    ])

    response = engine.request_goal('active')
    assert response.success is False
    assert 'gripper' in response.message
    assert 'robot_manager' in response.message


def test_goal_accepted_when_dependency_in_goal(dependency_config):
    """Goal is accepted when dependency is included in goal targets."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state([
        Component('gripper', ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
    ])

    response = engine.request_goal('active_full')
    assert response.success is True


def test_goal_accepted_when_dependency_already_satisfied(dependency_config):
    """Goal is accepted when dependency is already at required state."""
    lock = threading.Lock()
    engine = ForemanEngine(dependency_config, lock)

    engine.set_system_state([
        Component('gripper', ComponentType.CONTROLLER, LifecycleState.INACTIVE),
        Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE),
    ])

    response = engine.request_goal('active')
    assert response.success is True