import threading
from typing import Dict, List, Optional

from foreman.parser import ParsedScenario
from foreman.planner import Planner
from foreman.types import (
    Component,
    ErrorSnapshot,
    ForemanError,
    ForemanErrorCategory,
    ForemanResponse,
    ForemanSnapshot,
    LifecycleState,
    SystemProfile,
    SystemState,
    SystemTransitionCommand,
)


class ForemanEngine:
    """
    Foreman domain facade.

    All business logic is here, no ROS, just python.
    """

    def __init__(self, config: ParsedScenario, state_lock: threading.Lock):
        self._config = config
        self._planner = Planner(config.dependency_rules)
        self._state = SystemState()
        self._state_lock = state_lock

        self._current_profile = None
        self._at_profile = False
        self._is_ready = False  # when we get first /activity reading
        self._error_state: Optional[ForemanError] = None

    @property
    def is_at_profile(self) -> bool:
        """Checks if there are any remaining transitions to reach the profile."""
        with self._state_lock:
            return self._at_profile

    def request_profile(self, profile_name: str) -> ForemanResponse:
        """
        Request a new profile for the system.

        Returns: (success, message)
        """
        profile = self._config.profiles.get(profile_name)
        if not profile:
            return ForemanResponse(False, f"Profile '{profile_name}' not found in configuration.")

        with self._state_lock:
            if not self._is_ready:
                return ForemanResponse(
                    False, "Foreman not ready. Is /activity topic being published?"
                )

            missing_components = self._locked_missing_profile_components(profile)
            if missing_components:
                return ForemanResponse(
                    False,
                    f"Cannot accept profile '{profile_name}'. Missing components in observed state: {missing_components}",
                )

            unsatisfiable = self._locked_check_unsatisfiable_dependencies(profile)
            if unsatisfiable:
                return ForemanResponse(
                    False,
                    f"Cannot accept profile '{profile_name}'. Unsatisfiable dependencies:\n"
                    + "\n".join(f"  - {msg}" for msg in unsatisfiable),
                )

            error_cleared_msg = "Error cleared on new profile. " if self._error_state else ""
            self._error_state = None  # new profile received, clear error and try again.

            # TODO: minor. On first profile, if we're already at profile, we don't catch this, as self._current_profile == Null.
            # Fix this so we log "Already at profile"
            if self._current_profile == profile:
                if self._at_profile:
                    return ForemanResponse(True, f"Already at profile '{profile_name}'.")
                return ForemanResponse(True, f"Already transitioning to '{profile_name}'.")

            self._current_profile = profile
            self._at_profile = self._locked_is_at_profile()

        return ForemanResponse(True, f"{error_cleared_msg}Profile '{profile_name}' accepted.")

    def abort_profile(self, error: ForemanError):
        """Aborts the current profile by stopping transitions."""
        with self._state_lock:
            self._error_state = error
            self._locked_abort_transition()

    def get_next_transition(self) -> Optional[SystemTransitionCommand]:
        """Calculate the next step toward the profile."""
        if not self._current_profile:
            return None

        with self._state_lock:
            if not self._is_ready or self._error_state:
                return None

            return self._planner.get_next_transition(self._state, self._current_profile)

    def set_system_state(self, components: List[Component]) -> ForemanResponse:
        """
        Set internal system state to that which is observed.

        Called for every update from the /activity topic and from a lifecycle
        node's /transition_event. Checks the resulting profile and updates the
        error accordingly.
        """
        tracked_components = [c for c in components if c.name in self._config.tracked_components]

        with self._state_lock:
            previous_state = self._state.components
            was_at_profile = self._at_profile
            self._state.components = {comp.name: comp for comp in tracked_components}

            was_ready = self._is_ready
            self._is_ready = True

            if not was_ready:
                return ForemanResponse(True, "System state observed.")

            response = self._locked_check_profile(previous_state, was_at_profile)
            self._at_profile = self._locked_is_at_profile()
            return response

    def _locked_check_profile(
        self, previous_state: Dict[str, Component], was_at_profile: bool
    ) -> ForemanResponse:
        """
        Check the live state against the configured profiles and update the error.

        Clears a stale error once the live state matches a configured profile
        again. Raises a new error only if we were already at the requested
        profile and a component then drifts away from it in the background --
        not while still driving toward it.

        MUST be called while holding self._state_lock!
        """
        if self._locked_matching_profile_name() != "None":
            self._error_state = None

        if self._error_state or not self._current_profile or not was_at_profile:
            return ForemanResponse(True, "System state observed.")

        unexpected_changes = []
        for incoming in self._state.components.values():
            existing = previous_state.get(incoming.name)
            if existing and incoming.lifecycle_state != existing.lifecycle_state:
                unexpected_changes.append(
                    (incoming.name, existing.lifecycle_state.name, incoming.lifecycle_state.name)
                )

        missing_components = self._locked_missing_profile_components(self._current_profile)

        if not unexpected_changes and not missing_components:
            return ForemanResponse(True, "System state observed with no anomalies.")

        error_msgs = []
        error_components = []

        if missing_components:
            error_msgs.append(f"Required components vanished from /activity: {missing_components}")
            error_components.extend(missing_components)

        if unexpected_changes:
            msgs = [f"{name} ({old}->{new})" for name, old, new in unexpected_changes]
            error_msgs.append(f"Unexpected state changes: {', '.join(msgs)}")
            error_components.extend([change[0] for change in unexpected_changes])

        self._error_state = ForemanError(
            category=ForemanErrorCategory.UNEXPECTED_STATE,
            message="Aborting transition:\n  - " + "\n  - ".join(error_msgs),
            component_names=list(set(error_components)),
        )
        self._locked_abort_transition()

        return ForemanResponse(
            success=False, message="Unexpected system state.", error=self._error_state
        )

    @property
    def is_ready(self) -> bool:
        """Return True if the system is observed and ready to plan."""
        return self._is_ready

    def get_engine_snapshot(self) -> ForemanSnapshot:
        """Return a simplified snapshot of the system state."""
        with self._state_lock:
            return ForemanSnapshot(
                profile=self._locked_current_profile_name(),
                ready=self._is_ready,
                at_profile=self._at_profile,
                error=ErrorSnapshot(
                    is_error=self._error_state is not None,
                    category=(
                        self._error_state.category.value
                        if self._error_state
                        else ForemanErrorCategory.NONE.value
                    ),
                    message=self._error_state.message if self._error_state else "",
                    components=self._error_state.component_names if self._error_state else [],
                ),
                components=list(self._state.components.values()),
                all_profiles=list(self._config.profiles.keys()),
                available_profiles=(
                    [
                        name
                        for name, profile in self._config.profiles.items()
                        if self._locked_is_profile_available(profile)
                    ]
                    if self._is_ready
                    else []
                ),
            )

    def _locked_is_at_profile(self) -> bool:
        """
        Check if the current profile is reached.

        MUST be called while holding self._state_lock!
        """
        if not self._is_ready or not self._current_profile:
            return False

        # If planner returns nothing, we have reached the target profile
        return self._planner.get_next_transition(self._state, self._current_profile) is None

    def _locked_current_profile_name(self) -> str:
        """
        Name of the profile Foreman currently reports.

        While driving toward a requested profile, returns that profile's name,
        even mid-transition. Otherwise, returns the profile that matches the
        live observed state, or "None" if none matches.

        MUST be called while holding self._state_lock!
        """
        if self._current_profile:
            return self._current_profile.name
        return self._locked_matching_profile_name()

    def _locked_matching_profile_name(self) -> str:
        """
        Find the configured profile that the live observed state matches.

        A profile matches when every one of its declared targets is present in
        the observed state at exactly its declared lifecycle state. Returns
        "None" if no profile matches.

        MUST be called while holding self._state_lock!
        """
        for name, profile in self._config.profiles.items():
            targets = (
                profile.hardware_targets
                + profile.controller_targets
                + profile.lifecycle_node_targets
            )
            matches = True
            for target in targets:
                observed = self._state.components.get(target.name)
                if observed is None or observed.lifecycle_state != target.lifecycle_state:
                    matches = False
                    break
            if matches:
                return name
        return "None"

    def _locked_missing_profile_components(self, target_profile: SystemProfile) -> List[str]:
        """
        Check if all components in the target_profile are present in current state.

        Returns a list of missing components.
        MUST be called while holding self._state_lock!
        """
        missing = []
        all_component_targets = (
            target_profile.hardware_targets
            + target_profile.controller_targets
            + target_profile.lifecycle_node_targets
        )

        for component_target in all_component_targets:
            if component_target.name not in self._state.components:
                missing.append(component_target.name)
        return missing

    def _locked_is_profile_available(self, profile: SystemProfile) -> bool:
        """
        Check if a profile is currently achievable given observed component state.

        MUST be called while holding self._state_lock!
        """
        return not self._locked_missing_profile_components(
            profile
        ) and not self._locked_check_unsatisfiable_dependencies(profile)

    def _locked_check_unsatisfiable_dependencies(self, profile: SystemProfile) -> List[str]:
        """
        Validate that all controller dependencies in the profile can be satisfied.

        A dependency is satisfiable if:
        - It is already at or above the required state in current observed state, OR
        - It is included in the profile's infrastructure targets at or above the required state.
        Returns a list of error strings. Empty = all satisfiable.
        MUST be called while holding self._state_lock!
        """
        # TODO: refactor naming. Unfortunately, we treat lifecycle nodes same as hardware, so
        # in places, like rule.required_hardware, we are thinking about lifecycle nodes as well.
        # Lets use "infrastructure" for now to mean both of those
        profile_infrastructure_states = {}
        for comp in profile.hardware_targets + profile.lifecycle_node_targets:
            profile_infrastructure_states[comp.name] = comp.lifecycle_state

        errors = []
        for ctrl_target in profile.controller_targets:
            rule = self._planner.rules.get(ctrl_target.name)
            if not rule:
                continue

            # if stepping down, we don't care.
            if ctrl_target.lifecycle_state == LifecycleState.UNCONFIGURED:
                continue

            for req in rule.required_hardware:
                if ctrl_target.lifecycle_state == LifecycleState.ACTIVE:
                    required_state = req.state
                else:
                    # for configure, we need at least inactive.
                    required_state = LifecycleState.INACTIVE

                dependency_profile_state = profile_infrastructure_states.get(req.name)
                dependency_current = self._state.components.get(req.name)
                dependency_current_state = (
                    dependency_current.lifecycle_state if dependency_current else None
                )

                satisfied_by_profile = (
                    dependency_profile_state is not None
                    and dependency_profile_state >= required_state
                )
                satisfied_by_current = (
                    dependency_current_state is not None
                    and dependency_current_state >= required_state
                )

                if not satisfied_by_profile and not satisfied_by_current:
                    state_str = (
                        dependency_current_state.name if dependency_current_state else "UNKNOWN"
                    )
                    errors.append(
                        f"'{ctrl_target.name}' requires '{req.name}' at {required_state.name}, "
                        f"but it is {state_str} and not targeted in this profile"
                    )
        return errors

    def _locked_abort_transition(self):
        """
        Abort any ongoing transitions.

        MUST be called while holding self._state_lock!
        """
        if not self._is_ready:
            return

        self._current_profile = None
