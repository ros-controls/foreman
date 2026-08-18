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
    OperatingMode,
    StopState,
    SystemProfile,
)

# TODO: Once we settle on a config model
# TODO: Bulletproof this config parsing once we settle on one
# TODO: reconsider naming, for example ParsedScenario
# TODO: Rethink parsing output structure dataclass


@dataclass
class ParsedScenario:
    """Complete parsed scenario configuration."""

    hardware: List[str]
    dependency_rules: List[ControllerDependencyRule]
    profiles: Dict[str, SystemProfile]
    lifecycle_nodes: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    tracked_components: Set[str] = field(default_factory=set)
    autostart_profile: str = ""


def parse_state_string(state_str: str) -> LifecycleState:
    """Convert YAML state string to LifecycleState enum."""
    state_mapping = {
        "unconfigured": LifecycleState.UNCONFIGURED,
        "inactive": LifecycleState.INACTIVE,
        "active": LifecycleState.ACTIVE,
        "finalized": LifecycleState.FINALIZED,
    }
    normalized = state_str.lower()
    if normalized not in state_mapping:
        raise ValueError(f"Unknown state: {state_str}")
    return state_mapping[normalized]


def parse_operating_mode_string(mode_str: str) -> OperatingMode:
    """Convert YAML operating mode string to OperatingMode enum."""
    mode_mapping = {
        "automatic": OperatingMode.AUTOMATIC,
        "manual_reduced": OperatingMode.MANUAL_REDUCED,
        "manual_high": OperatingMode.MANUAL_HIGH,
    }
    normalized = mode_str.lower()
    if normalized not in mode_mapping:
        raise ValueError(f"Unknown operating mode: {mode_str}")
    return mode_mapping[normalized]


def parse_stop_state_string(stop_state_str: str) -> StopState:
    """Convert YAML stop state string to StopState enum."""
    stop_state_mapping = {
        "running": StopState.RUNNING,
        "normal_stop": StopState.NORMAL_STOP,
        "operational_stop": StopState.OPERATIONAL_STOP,
        "emergency_stop": StopState.EMERGENCY_STOP,
    }
    normalized = stop_state_str.lower()
    if normalized not in stop_state_mapping:
        raise ValueError(f"Unknown stop state: {stop_state_str}")
    return stop_state_mapping[normalized]


def parse_requires(
    requires: List[str], hardware: List[str], lifecycle_nodes: List[str] = None
) -> List[HardwareRequirement]:
    """
    Parse the 'requires' field into list of HardwareRequirement.

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

        if target == "all":
            reqs.extend(
                [
                    HardwareRequirement(name=name, state=state)
                    for name in hardware + lifecycle_nodes
                ]
            )
        else:
            reqs.append(HardwareRequirement(name=target, state=state))

    return reqs


def parse_yaml_file(file_path: Path) -> ParsedScenario:
    """Parse a scenario YAML file into a ParsedScenario object."""
    with open(file_path) as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError("Empty YAML file")

    autostart_profile = data.get("autostart_profile", "")
    hardware = data.get("hardware", [])
    lifecycle_nodes = data.get("lifecycle_nodes", [])

    dependency_rules = []
    controllers = data.get("controllers", {})
    for ctrl_name, ctrl_config in controllers.items():
        requires = ctrl_config.get("requires", [])
        reqs = parse_requires(requires, hardware, lifecycle_nodes)

        dependency_rules.append(
            ControllerDependencyRule(controller_name=ctrl_name, required_hardware=reqs)
        )

    profiles = {}
    profile_configs = data.get("profiles", {})
    for profile_name, profile_config in profile_configs.items():
        hw_targets = []
        ctrl_targets = []
        lc_targets = []

        for hw_name, state_str in profile_config.get("hardware", {}).items():
            hw_targets.append(
                Component(
                    name=hw_name,
                    component_type=ComponentType.HARDWARE,
                    lifecycle_state=parse_state_string(state_str),
                )
            )

        for ctrl_name, state_str in profile_config.get("controllers", {}).items():
            ctrl_targets.append(
                Component(
                    name=ctrl_name,
                    component_type=ComponentType.CONTROLLER,
                    lifecycle_state=parse_state_string(state_str),
                )
            )

        for lc_name, state_str in profile_config.get("lifecycle_nodes", {}).items():
            lc_targets.append(
                Component(
                    name=lc_name,
                    component_type=ComponentType.LIFECYCLE_NODE,
                    lifecycle_state=parse_state_string(state_str),
                )
            )

        operating_mode = profile_config.get("operating_mode")
        stop_state = profile_config.get("stop_state")

        profiles[profile_name] = SystemProfile(
            name=profile_name,
            hardware_targets=hw_targets,
            controller_targets=ctrl_targets,
            lifecycle_node_targets=lc_targets,
            operating_mode=(
                parse_operating_mode_string(operating_mode)
                if operating_mode
                else OperatingMode.AUTOMATIC
            ),
            stop_state=(parse_stop_state_string(stop_state) if stop_state else StopState.RUNNING),
        )

    metadata = {}
    known_keys = {
        "autostart_profile",
        "hardware",
        "lifecycle_nodes",
        "controllers",
        "profiles",
    }
    for key, value in data.items():
        if key not in known_keys:
            metadata[key] = value

    tracked_components = set(hardware + lifecycle_nodes)
    for rule in dependency_rules:
        tracked_components.add(rule.controller_name)
    for profile in profiles.values():
        tracked_components.update(c.name for c in profile.hardware_targets)
        tracked_components.update(c.name for c in profile.controller_targets)
        tracked_components.update(c.name for c in profile.lifecycle_node_targets)

    if autostart_profile and autostart_profile not in profiles:
        raise ValueError(
            f"autostart_profile '{autostart_profile}' not found in profiles. "
            f"Available: {list(profiles.keys())}"
        )

    return ParsedScenario(
        autostart_profile=autostart_profile,
        hardware=hardware,
        lifecycle_nodes=lifecycle_nodes,
        dependency_rules=dependency_rules,
        profiles=profiles,
        metadata=metadata,
        tracked_components=tracked_components,
    )
