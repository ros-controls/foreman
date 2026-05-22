from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Set

import yaml

from foreman.types import (
    Component,
    ComponentType,
    ControllerDependencyRule,
    HardwareRequirement,
    LifecycleState,
    SystemGoal,
)

# TODO: Once we settle on a config model
# TODO: Bulletproof this config parsing once we settle on one
# TODO: reconsider naming, for example ParsedScenario
# TODO: Rethink parsing output structure dataclass

@dataclass
class ParsedScenario:
    """Complete parsed scenario configuration."""

    controller_manager: str
    transition_pause: float
    hardware: List[str]
    dependency_rules: List[ControllerDependencyRule]
    goals: Dict[str, SystemGoal]
    lifecycle_nodes: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    tracked_components: Set[str] = field(default_factory=set)


def parse_state_string(state_str: str) -> LifecycleState:
    """Convert YAML state string to LifecycleState enum."""
    state_mapping = {
        'unconfigured': LifecycleState.UNCONFIGURED,
        'inactive': LifecycleState.INACTIVE,
        'active': LifecycleState.ACTIVE,
        'finalized': LifecycleState.FINALIZED,
    }
    normalized = state_str.lower()
    if normalized not in state_mapping:
        raise ValueError(f"Unknown state: {state_str}")
    return state_mapping[normalized]


def parse_requires(
    requires: List[str], hardware: List[str], lifecycle_nodes: List[str] = None
) -> List[HardwareRequirement]:
    """Parse the 'requires' field into list of HardwareRequirement.

    Supports:
    - [all, inactive] -> all hardware + lifecycle nodes must be at that state
    - [component_name, active] -> specific hardware or lifecycle node must be at that state
    """
    if lifecycle_nodes is None:
        lifecycle_nodes = []

    if not requires:
        return []

    # we can get either a single [component, state] entry
    # or a list of [component, state] entries.
    # here we normalize so we work with a list of [component, state]
    if len(requires) == 2 and isinstance(requires[0], str):
        requires_normalized = [requires]
    else:
        requires_normalized = requires

    reqs = []

    for req in requires_normalized:
        if not isinstance(req, list) or len(req) != 2:
            raise ValueError(f"Invalid requirement format: {req}. Expected [target, state].")
            
        target = req[0]
        state = parse_state_string(req[1])

        if target == 'all':
            reqs.extend([
                HardwareRequirement(name=name, state=state)
                for name in hardware + lifecycle_nodes
            ])
        else:
            reqs.append(HardwareRequirement(name=target, state=state))

    return reqs


def parse_yaml_file(file_path: Path) -> ParsedScenario:
    """Parse a scenario YAML file into a ParsedScenario object."""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError("Empty YAML file")

    controller_manager = data.get('controller_manager', '')
    transition_pause = data.get('transition_pause', 0.0)
    hardware = data.get('hardware', [])
    lifecycle_nodes = data.get('lifecycle_nodes', [])

    dependency_rules = []
    controllers = data.get('controllers', {})
    for ctrl_name, ctrl_config in controllers.items():
        requires = ctrl_config.get('requires', [])
        reqs = parse_requires(requires, hardware, lifecycle_nodes)

        dependency_rules.append(ControllerDependencyRule(
            controller_name=ctrl_name,
            required_hardware=reqs
        ))

    goals = {}
    goal_states = data.get('goal_states', {})
    for goal_name, goal_config in goal_states.items():
        hw_goals = []
        ctrl_goals = []
        lc_goals = []

        for hw_name, state_str in goal_config.get('hardware', {}).items():
            hw_goals.append(Component(
                name=hw_name,
                component_type=ComponentType.HARDWARE,
                lifecycle_state=parse_state_string(state_str)
            ))

        for ctrl_name, state_str in goal_config.get('controllers', {}).items():
            ctrl_goals.append(Component(
                name=ctrl_name,
                component_type=ComponentType.CONTROLLER,
                lifecycle_state=parse_state_string(state_str)
            ))

        for lc_name, state_str in goal_config.get('lifecycle_nodes', {}).items():
            lc_goals.append(Component(
                name=lc_name,
                component_type=ComponentType.LIFECYCLE_NODE,
                lifecycle_state=parse_state_string(state_str)
            ))

        goals[goal_name] = SystemGoal(
            name=goal_name,
            hardware_goals=hw_goals,
            controller_goals=ctrl_goals,
            lifecycle_node_goals=lc_goals
        )

    metadata = {}
    known_keys = {'controller_manager', 'transition_pause', 'hardware', 'lifecycle_nodes', 'controllers', 'goal_states'}
    for key, value in data.items():
        if key not in known_keys:
            metadata[key] = value
    
    tracked_components = set(hardware + lifecycle_nodes)
    for rule in dependency_rules:
        tracked_components.add(rule.controller_name)
    for goal in goals.values():
        tracked_components.update(c.name for c in goal.hardware_goals)
        tracked_components.update(c.name for c in goal.controller_goals)
        tracked_components.update(c.name for c in goal.lifecycle_node_goals)

    return ParsedScenario(
        controller_manager=controller_manager,
        transition_pause=transition_pause,
        hardware=hardware,
        lifecycle_nodes=lifecycle_nodes,
        dependency_rules=dependency_rules,
        goals=goals,
        metadata=metadata,
        tracked_components=tracked_components
    )
