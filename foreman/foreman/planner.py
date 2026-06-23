from typing import List, Optional

from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ControllerDependencyRule
from foreman.types import LifecycleState
from foreman.types import SystemGoal
from foreman.types import SystemState
from foreman.types import SystemTransitionCommand


class Planner:
    """Plan the next single step towards the lifecycle state goal of the system."""

    def __init__(self, dependency_rules: List[ControllerDependencyRule]):
        self.rules = {rule.controller_name: rule for rule in dependency_rules}

    def replace_dependency_rules(self, dependency_rules: List[ControllerDependencyRule]):
        """Swap the whole rule set at runtime with freshly inferred rules."""
        self.rules = {rule.controller_name: rule for rule in dependency_rules}

    def get_next_transition(
        self, current_state: SystemState, goal: SystemGoal
    ) -> Optional[SystemTransitionCommand]:
        """
        Output the next transition based on named goal state and these priorities.

        Priority: C deactivate > HW Down > HW Up > C cleanup > C config > C activate.
        """
        cmds_hw_step_up = []
        cmds_hw_step_down = []
        cmds_ctrl_config = []
        cmds_ctrl_cleanup = []
        cmds_ctrl_activate = []
        cmds_ctrl_deactivate = []

        # 1. "Infrastructure" Transitions (Hardware + Lifecycle Nodes) - nodes controllers depend upon
        # This is an unfortunate name, but we're rolling with it for now.
        # Later, when we implement a more complex ordering of components, we can lose
        # this ordering of "hardware+lifecycle nodes first, controllers second".
        # we'll probably build a DAG, so any component can depend on any other.
        # For simplicity, this is it for now, bear with me that in "HardwareRequirements" for example
        # we have hardware_interfaces AND lifecycle nodes.
        infrastructure_goals = goal.hardware_goals + goal.lifecycle_node_goals
        for infra_goal in infrastructure_goals:
            infra_component = current_state.components.get(infra_goal.name)

            if not infra_component:
                infra_component = Component(
                    infra_goal.name,
                    infra_goal.component_type,
                    LifecycleState.UNCONFIGURED
                )

            next_state = infra_component.lifecycle_state.step_towards(infra_goal.lifecycle_state)
            if next_state:
                if next_state < infra_component.lifecycle_state:
                    if not self._can_hardware_step_down(infra_component.name, next_state, current_state):
                        cmds_hw_step_down.append(
                            SystemTransitionCommand(infra_component, next_state))

                cmds_hw_step_up.append(SystemTransitionCommand(infra_component, next_state))

        # 2. Controller Transitions
        for controller_goal in goal.controller_goals:
            controller_component = current_state.components.get(controller_goal.name)

            if not controller_component:
                controller_component = Component(
                    controller_goal.name,
                    ComponentType.CONTROLLER,
                    LifecycleState.UNCONFIGURED
                )

            next_state = controller_component.lifecycle_state.step_towards(
                controller_goal.lifecycle_state)

            if next_state:
                current = controller_component.lifecycle_state

                if next_state > current:
                    # Guard activation AND configuration against hardware states
                    if not self._can_controller_step_up(controller_goal.name, next_state, current_state):
                        continue

                # Categorize into the appropriate phase
                if current == LifecycleState.ACTIVE and next_state == LifecycleState.INACTIVE:
                    cmds_ctrl_deactivate.append(
                        SystemTransitionCommand(controller_component, next_state))
                elif current == LifecycleState.INACTIVE and next_state == LifecycleState.ACTIVE:
                    cmds_ctrl_activate.append(SystemTransitionCommand(
                        controller_component, next_state))
                elif current == LifecycleState.UNCONFIGURED and next_state == LifecycleState.INACTIVE:
                    cmds_ctrl_config.append(SystemTransitionCommand(
                        controller_component, next_state))
                elif current == LifecycleState.INACTIVE and next_state == LifecycleState.UNCONFIGURED:
                    cmds_ctrl_cleanup.append(SystemTransitionCommand(
                        controller_component, next_state))

        # Strict order for issuing next command
        # C deactivate > HW Down > HW Up > C cleanup > C config > C activate
        if cmds_ctrl_deactivate:
            return cmds_ctrl_deactivate[0]
        if cmds_hw_step_down:
            return cmds_hw_step_down[0]
        if cmds_hw_step_up:
            return cmds_hw_step_up[0]
        if cmds_ctrl_cleanup:
            return cmds_ctrl_cleanup[0]
        if cmds_ctrl_config:
            return cmds_ctrl_config[0]
        if cmds_ctrl_activate:
            return cmds_ctrl_activate[0]

    def _can_controller_step_up(self, ctrl_name: str, next_ctrl_state: LifecycleState, current_state: SystemState) -> bool:
        """Check if hardware dependencies are met for configuring or activating a controller."""
        rule = self.rules.get(ctrl_name)
        if not rule:
            return True

        for req in rule.required_hardware:
            hw_component = current_state.components.get(req.name)

            if not hw_component:
                return False

            if next_ctrl_state == LifecycleState.ACTIVE:
                # To activate, HW must meet the explicitly defined target state (e.g., ACTIVE)
                if hw_component.lifecycle_state < req.state:
                    return False
            elif next_ctrl_state == LifecycleState.INACTIVE:
                # To configure, HW must at least be INACTIVE to have exported its interfaces
                if hw_component.lifecycle_state < LifecycleState.INACTIVE:
                    return False

        return True

    def _can_hardware_step_down(
        self, hw_name: str, next_hw_state: LifecycleState, current_state: SystemState
    ) -> bool:
        """Check if hardware can safely step down without violating controller dependencies."""
        for comp in current_state.components.values():
            if comp.component_type != ComponentType.CONTROLLER:
                continue

            rule = self.rules.get(comp.name)
            if not rule:
                continue

            for req in rule.required_hardware:
                if req.name == hw_name:
                    # If controller is ACTIVE, hardware cannot drop below explicitly required state
                    if comp.lifecycle_state == LifecycleState.ACTIVE:
                        if next_hw_state < req.state:
                            return False

                    # If controller is INACTIVE, hardware cannot be UNCONFIGURED
                    elif comp.lifecycle_state == LifecycleState.INACTIVE:
                        if next_hw_state < LifecycleState.INACTIVE:
                            return False

        return True
