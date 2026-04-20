import pytest

from foreman.planner import Planner
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ControllerDependencyRule
from foreman.types import LifecycleState
from foreman.types import SystemGoal
from foreman.types import SystemState
from foreman.types import SystemTransitionCommand
from foreman.types import HardwareRequirement


@pytest.fixture
def basic_planner():
    rules = [
        ControllerDependencyRule(
            controller_name='franka_jtc',
            required_hardware=[HardwareRequirement('franka_hw', LifecycleState.ACTIVE)]
        )
    ]
    return Planner(dependency_rules=rules)


@pytest.fixture
def broadcaster_planner():
    rules = [
        ControllerDependencyRule(
            controller_name='joint_state_broadcaster',
            required_hardware=[HardwareRequirement('franka_hw', LifecycleState.INACTIVE)]
        )
    ]
    return Planner(dependency_rules=rules)


@pytest.fixture
def asymmetric_planner():
    rules = [
        ControllerDependencyRule(
            controller_name='dual_arm_jtc',
            required_hardware=[
                HardwareRequirement('hw_a', LifecycleState.ACTIVE),
                HardwareRequirement('hw_b', LifecycleState.ACTIVE)
            ]
        )
    ]
    return Planner(dependency_rules=rules)


def apply_command(state: SystemState, cmd: SystemTransitionCommand):
    """Helper to mutate state with a planner command, simulating a successful transition."""
    if cmd:
        state.components[cmd.component.name] = Component(
            cmd.component.name, cmd.component.component_type, cmd.goal_state
        )


def test_scenario_1_standard_bring_up(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('active',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.ACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_2_standard_teardown(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
    })
    goal = SystemGoal('unc',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)])

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.UNCONFIGURED
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.UNCONFIGURED


def test_scenario_3_broadcaster_bringup(broadcaster_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
        'joint_state_broadcaster': Component('joint_state_broadcaster', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('active',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('joint_state_broadcaster', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    cmd = broadcaster_planner.get_next_transition(state, goal)
    apply_command(state, cmd)

    cmd = broadcaster_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.ACTIVE
    apply_command(state, cmd)


def test_scenario_4_partial_pause(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
    })
    goal = SystemGoal('idle',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.INACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)])

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.INACTIVE


def test_scenario_5_controller_swap(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'ctrl_A': Component('ctrl_A', ComponentType.CONTROLLER, LifecycleState.ACTIVE),
        'ctrl_B': Component('ctrl_B', ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    })
    goal = SystemGoal('swap',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[
                          Component('ctrl_A', ComponentType.CONTROLLER, LifecycleState.INACTIVE),
                          Component('ctrl_B', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
                      ])

    basic_planner.rules['ctrl_A'] = ControllerDependencyRule('ctrl_A', [HardwareRequirement('franka_hw', LifecycleState.ACTIVE)])
    basic_planner.rules['ctrl_B'] = ControllerDependencyRule('ctrl_B', [HardwareRequirement('franka_hw', LifecycleState.ACTIVE)])

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'ctrl_A'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'ctrl_B'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_6_hardware_failure(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.INACTIVE),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    })
    goal = SystemGoal('active',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    cmd = basic_planner.get_next_transition(state, goal)
    # Command applied, but failed. We re-issue the command.
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.ACTIVE

def test_scenario_7_controller_teardown_failure(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
    })
    goal = SystemGoal('unc',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)])

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    
    # Command applied, but failed. We re-issue the command.
    cmd_retry = basic_planner.get_next_transition(state, goal)
    assert cmd_retry.component.name == 'franka_jtc'
    assert cmd_retry.goal_state == LifecycleState.INACTIVE


def test_scenario_8_controller_activation_failure(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    })
    goal = SystemGoal('active',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    cmd = basic_planner.get_next_transition(state, goal)
    # Command applied, but failed. We re-issue the command.
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_9_asymmetric_hardware_failure(asymmetric_planner):
    state = SystemState(components={
        'hw_a': Component('hw_a', ComponentType.HARDWARE, LifecycleState.ACTIVE),
        'hw_b': Component('hw_b', ComponentType.HARDWARE, LifecycleState.INACTIVE),
        'dual_arm_jtc': Component('dual_arm_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    })
    goal = SystemGoal('active',
                      hardware_goals=[
                          Component('hw_a', ComponentType.HARDWARE, LifecycleState.ACTIVE),
                          Component('hw_b', ComponentType.HARDWARE, LifecycleState.ACTIVE)
                      ],
                      controller_goals=[Component('dual_arm_jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    # Command applied on hw_b, but failed. We re-issue the command.
    cmd = asymmetric_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'hw_b'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_10_controller_configure_blocked_by_hardware(basic_planner):
    """Test that a controller cannot configure if its required hardware is unconfigured."""
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('idle',
                      hardware_goals=[Component('franka_hw', ComponentType.HARDWARE, LifecycleState.INACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)])

    # First, we configure hardware before controller
    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.INACTIVE

    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.INACTIVE