import pytest

from foreman.planner import Planner
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ControllerDependencyRule
from foreman.types import HardwareRequirement
from foreman.types import LifecycleState
from foreman.types import SystemGoal
from foreman.types import SystemState
from foreman.types import SystemTransitionCommand


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


def test_planner_pulls_rules_from_provider(basic_planner):
    """Verify the planner uses rules returned by its provider."""
    class _FakeProvider:
        def get_dependency_rules(self):
            return [ControllerDependencyRule(
                controller_name='new_ctrl',
                required_hardware=[HardwareRequirement('new_hw', LifecycleState.ACTIVE)])]

    basic_planner.set_dependency_provider(_FakeProvider())
    rules = basic_planner.get_current_rules()

    # Rules now come from the provider instead of the initial configuration.
    assert 'franka_jtc' not in rules
    assert rules['new_ctrl'].required_hardware[0].name == 'new_hw'


def test_scenario_1_standard_bring_up(basic_planner):
    state = SystemState(components={
        'franka_hw': Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
        'franka_jtc': Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('active',
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.INACTIVE)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      controller_goals=[
                          Component('ctrl_A', ComponentType.CONTROLLER, LifecycleState.INACTIVE),
                          Component('ctrl_B', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
                      ])

    basic_planner.rules['ctrl_A'] = ControllerDependencyRule(
        'ctrl_A', [HardwareRequirement('franka_hw', LifecycleState.ACTIVE)])
    basic_planner.rules['ctrl_B'] = ControllerDependencyRule(
        'ctrl_B', [HardwareRequirement('franka_hw', LifecycleState.ACTIVE)])

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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
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
                      hardware_goals=[
                          Component('franka_hw', ComponentType.HARDWARE, LifecycleState.INACTIVE)],
                      controller_goals=[Component('franka_jtc', ComponentType.CONTROLLER, LifecycleState.INACTIVE)])

    # First, we configure hardware before controller
    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_hw'
    assert cmd.goal_state == LifecycleState.INACTIVE

    apply_command(state, cmd)

    cmd = basic_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'franka_jtc'
    assert cmd.goal_state == LifecycleState.INACTIVE


# --- Lifecycle Node Tests ---

@pytest.fixture
def lifecycle_planner():
    """Planner where a controller depends on a lifecycle node."""
    rules = [
        ControllerDependencyRule(
            controller_name='gripper',
            required_hardware=[HardwareRequirement('robot_manager', LifecycleState.ACTIVE)]
        )
    ]
    return Planner(dependency_rules=rules)


def test_scenario_11_lifecycle_node_bringup(lifecycle_planner):
    """Lifecycle node transitions in the same phase as hardware (step up)."""
    state = SystemState(components={
        'robot_manager': Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED),
        'gripper': Component('gripper', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('active',
                      lifecycle_node_goals=[
                          Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('gripper', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'robot_manager'
    assert cmd.component.component_type == ComponentType.LIFECYCLE_NODE
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'robot_manager'
    assert cmd.goal_state == LifecycleState.ACTIVE
    apply_command(state, cmd)

    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'gripper'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'gripper'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_12_controller_blocked_by_lifecycle_node(lifecycle_planner):
    """Controller cannot activate until lifecycle node meets required state."""
    state = SystemState(components={
        'robot_manager': Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE),
        'gripper': Component('gripper', ComponentType.CONTROLLER, LifecycleState.INACTIVE)
    })
    goal = SystemGoal('active',
                      lifecycle_node_goals=[
                          Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('gripper', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    # Lifecycle node must activate before the controller can
    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'robot_manager'
    assert cmd.goal_state == LifecycleState.ACTIVE
    apply_command(state, cmd)

    # Now controller can activate
    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'gripper'
    assert cmd.goal_state == LifecycleState.ACTIVE


def test_scenario_13_lifecycle_node_stepdown_blocked_by_controller(lifecycle_planner):
    """Lifecycle node cannot step down while a dependent controller is active."""
    state = SystemState(components={
        'robot_manager': Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE),
        'gripper': Component('gripper', ComponentType.CONTROLLER, LifecycleState.ACTIVE)
    })
    goal = SystemGoal('idle',
                      lifecycle_node_goals=[
                          Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.INACTIVE)],
                      controller_goals=[Component('gripper', ComponentType.CONTROLLER, LifecycleState.INACTIVE)])

    # Controller must deactivate first (priority: ctrl deactivate > infra down)
    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'gripper'
    assert cmd.goal_state == LifecycleState.INACTIVE
    apply_command(state, cmd)

    # Now lifecycle node can step down
    cmd = lifecycle_planner.get_next_transition(state, goal)
    assert cmd.component.name == 'robot_manager'
    assert cmd.goal_state == LifecycleState.INACTIVE


def test_scenario_14_mixed_hw_lifecycle_and_controllers():
    """Full scenario: HW + lifecycle node + controller, bring-up sequence."""
    rules = [
        ControllerDependencyRule(
            controller_name='jtc',
            required_hardware=[
                HardwareRequirement('hw_arm', LifecycleState.ACTIVE),
                HardwareRequirement('robot_manager', LifecycleState.ACTIVE)
            ]
        )
    ]
    planner = Planner(dependency_rules=rules)

    state = SystemState(components={
        'hw_arm': Component('hw_arm', ComponentType.HARDWARE, LifecycleState.UNCONFIGURED),
        'robot_manager': Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.UNCONFIGURED),
        'jtc': Component('jtc', ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED)
    })
    goal = SystemGoal('active',
                      hardware_goals=[
                          Component('hw_arm', ComponentType.HARDWARE, LifecycleState.ACTIVE)],
                      lifecycle_node_goals=[
                          Component('robot_manager', ComponentType.LIFECYCLE_NODE, LifecycleState.ACTIVE)],
                      controller_goals=[Component('jtc', ComponentType.CONTROLLER, LifecycleState.ACTIVE)])

    # Infrastructure steps up first (hw and lifecycle node interleaved)
    transitions = []
    for _ in range(10):
        cmd = planner.get_next_transition(state, goal)
        if cmd is None:
            break
        transitions.append((cmd.component.name, cmd.goal_state))
        apply_command(state, cmd)

    # Verify controller comes last (after all infrastructure is ACTIVE)
    ctrl_indices = [i for i, (name, _) in enumerate(transitions) if name == 'jtc']
    infra_indices = [i for i, (name, _) in enumerate(transitions)
                     if name in ('hw_arm', 'robot_manager')]
    assert all(c > max(infra_indices) for c in ctrl_indices)

    # Verify we reached the goal
    assert planner.get_next_transition(state, goal) is None
