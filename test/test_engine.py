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
    ABORT_GOAL_NAME = "aborted"

    # initialize
    initial_components = [Component('hw1', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)]
    engine.set_system_state(initial_components)
    
    # goal to activate comes
    success, msg = engine.request_goal('active_goal')
    assert success is True
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
    
    # system is now AT GOAL
    assert engine.is_at_goal is True 
    
    # planner outputs nothing
    assert not engine.get_next_transition()
    
    # frontend will see the error
    snapshot = engine.get_engine_snapshot()
    assert snapshot['error']['is_error'] is True
    assert snapshot['error']['message'] == ERROR_MSG
    assert snapshot['goal'] == ABORT_GOAL_NAME

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
    engine.set_system_state([comp1_new])
    
    # verify no errors were triggered
    snapshot = engine.get_engine_snapshot()
    assert snapshot['error']['is_error'] is False

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
    engine.set_system_state([comp1_crashed])
    
    # verify error was generated correctly
    snapshot = engine.get_engine_snapshot()
    assert snapshot['error']['is_error'] is True
    assert snapshot['error']['category'] == ForemanErrorCategory.UNEXPECTED_STATE.value
    assert 'hw1' in snapshot['error']['components']
    assert snapshot['goal'] == 'aborted'
    
    # verify planner halts
    assert engine.get_next_transition() is None