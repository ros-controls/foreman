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

        self._target_profile = None
        self._current_profile = "None"  # name of the profile the live state matches
        self._is_ready = False  # when we get first /activity reading
        self._error_state: Optional[ForemanError] = None
        self._last_issued_command: Optional[SystemTransitionCommand] = None
        self._reached_target = False  # latches True once at_profile, until the next request

    @property
    def is_at_profile(self) -> bool:
        """Checks if there are any remaining transitions to reach the profile."""
        with self._state_lock:
            return self._is_at_profile()

    def request_profile(self, profile_name: str) -> ForemanResponse:
        """
        Request a new profile for the system.

        Clears a blocked-category error outright. Recomputes an
        UNEXPECTED_STATE error against the new target instead of
        clearing it blindly.

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

            missing_components = self._missing_profile_components(profile)
            if missing_components:
                return ForemanResponse(
                    False,
                    f"Cannot accept profile '{profile_name}'. Missing components in observed state: {missing_components}",
                )

            unsatisfiable = self._check_unsatisfiable_dependencies(profile)
            if unsatisfiable:
                return ForemanResponse(
                    False,
                    f"Cannot accept profile '{profile_name}'. Unsatisfiable dependencies:\n"
                    + "\n".join(f"  - {msg}" for msg in unsatisfiable),
                )

            had_error = self._error_state is not None
            if (
                self._error_state
                and self._error_state.category != ForemanErrorCategory.UNEXPECTED_STATE
            ):
                self._error_state = None  # blocked category: only an explicit request can clear it
            self._last_issued_command = None

            # TODO: minor. On first profile, if we're already at profile, we don't catch this, as self._target_profile == None.
            # Fix this so we log "Already at profile"
            if self._target_profile == profile and not self._error_state:
                if self._is_at_profile():
                    return ForemanResponse(True, f"Already at profile '{profile_name}'.")
                return ForemanResponse(True, f"Already transitioning to '{profile_name}'.")

            self._target_profile = profile
            self._current_profile = self._matching_profile_name()
            self._reached_target = self._is_at_profile()

            if self._error_state:
                mismatches = self._profile_mismatches(profile)
                self._error_state = (
                    None
                    if not mismatches
                    else ForemanError(
                        category=self._error_state.category,
                        message=self._error_state.message,
                        component_names=mismatches,
                    )
                )

            error_cleared_msg = (
                "Error cleared on new profile. " if had_error and not self._error_state else ""
            )

        return ForemanResponse(True, f"{error_cleared_msg}Profile '{profile_name}' accepted.")

    def abort_profile(self, error: ForemanError):
        """
        Aborts the target profile by stopping transitions.

        Also gives up the target profile, unless the error is
        UNEXPECTED_STATE.
        """
        with self._state_lock:
            self._error_state = error
            self._last_issued_command = None
            if error.category != ForemanErrorCategory.UNEXPECTED_STATE:
                self._target_profile = None  # give up on the goal; only a new request retries

    def get_next_transition(self) -> Optional[SystemTransitionCommand]:
        """
        Calculate the next step toward the profile.

        Returns None once the target has already been reached, even if
        the live state later drifts away from it. Driving toward a
        target resumes only once it's requested again.
        """
        if not self._target_profile:
            return None

        with self._state_lock:
            if not self._is_ready or not self._target_profile:
                return None
            if self._reached_target:
                return None

            cmd = self._planner.get_next_transition(self._state, self._target_profile)
            self._last_issued_command = cmd
            return cmd

    def set_system_state(self, components: List[Component]) -> ForemanResponse:
        """
        Set internal system state to that which is observed, and update the error.

        Called for every update from the /activity topic and from a lifecycle
        node's /transition_event.
        """
        tracked_components = [c for c in components if c.name in self._config.tracked_components]

        with self._state_lock:
            previous_state = self._state.components
            self._state.components = {comp.name: comp for comp in tracked_components}
            self._current_profile = self._matching_profile_name()

            was_ready = self._is_ready
            self._is_ready = True

            if not was_ready:
                return ForemanResponse(True, "System state observed.")

            response = self.check_profile(previous_state)
            if self._is_at_profile():
                self._reached_target = True
            return response

    def check_profile(self, previous_state: Dict[str, Component]) -> ForemanResponse:
        """
        Check the live state against the configured profiles and update the error.

        Refreshes an active error's component list on every call. Clears
        it once every targeted component matches its profile target again.

        Raises a new error if a component changes to something other than
        what Foreman commanded, or its own profile target. This applies
        whenever a profile is targeted, not just mid-transition. Also
        raises an error if a required component vanishes.

        MUST be called while holding self._state_lock!
        """
        if self._error_state:
            return self._recheck_error()

        if not self._target_profile:
            return ForemanResponse(True, "System state observed.")

        unexpected_changes = []
        for incoming in self._state.components.values():
            existing = previous_state.get(incoming.name)
            if existing and incoming.lifecycle_state != existing.lifecycle_state:
                target = self._profile_target_state(self._target_profile, incoming.name)
                expected = (
                    self._last_issued_command
                    and self._last_issued_command.component.name == incoming.name
                    and self._last_issued_command.goal_state == incoming.lifecycle_state
                ) or (target is not None and incoming.lifecycle_state == target)
                if not expected:
                    unexpected_changes.append(
                        (
                            incoming.name,
                            existing.lifecycle_state.name,
                            incoming.lifecycle_state.name,
                        )
                    )

        missing_components = self._missing_profile_components(self._target_profile)

        if not unexpected_changes and not missing_components:
            return ForemanResponse(True, "System state observed with no anomalies.")

        error_msgs = []

        if missing_components:
            error_msgs.append(f"Required components vanished from /activity: {missing_components}")

        if unexpected_changes:
            msgs = [f"{name} ({old}->{new})" for name, old, new in unexpected_changes]
            error_msgs.append(f"Unexpected state changes: {', '.join(msgs)}")

        self._error_state = ForemanError(
            category=ForemanErrorCategory.UNEXPECTED_STATE,
            message="Unexpected system state:\n  - " + "\n  - ".join(error_msgs),
            component_names=self._profile_mismatches(self._target_profile),
        )
        self._last_issued_command = None

        return ForemanResponse(
            success=False, message="Unexpected system state.", error=self._error_state
        )

    def _recheck_error(self) -> ForemanResponse:
        """
        Refresh the active error against the target profile's live mismatches.

        Clears the error once every targeted component matches again.
        MUST be called while holding self._state_lock!
        """
        if not self._target_profile:
            return ForemanResponse(False, "Unexpected system state.", error=self._error_state)

        mismatches = self._profile_mismatches(self._target_profile)
        if not mismatches:
            self._error_state = None
            return ForemanResponse(True, "System state observed.")

        self._error_state = ForemanError(
            category=self._error_state.category,
            message=self._error_state.message,
            component_names=mismatches,
        )
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
                target_profile=(self._target_profile.name if self._target_profile else "None"),
                current_profile=self._current_profile,
                ready=self._is_ready,
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
                        if self._is_profile_available(profile)
                    ]
                    if self._is_ready
                    else []
                ),
            )

    def _is_at_profile(self) -> bool:
        """
        Check if the live state matches the target profile.

        MUST be called while holding self._state_lock!
        """
        return (
            self._target_profile is not None and self._current_profile == self._target_profile.name
        )

    def _profile_matches_state(self, profile: SystemProfile) -> bool:
        """
        Check if the live observed state satisfies every target of the given profile.

        MUST be called while holding self._state_lock!
        """
        targets = (
            profile.hardware_targets + profile.controller_targets + profile.lifecycle_node_targets
        )
        for target in targets:
            observed = self._state.components.get(target.name)
            if observed is None or observed.lifecycle_state != target.lifecycle_state:
                return False
        return True

    def _matching_profile_name(self) -> str:
        """
        Find the configured profile that the live observed state matches, or "None".

        Prefers the target profile over another, narrower one that happens to
        match the same state -- e.g. "active" (gripper only) and "active_full"
        (gripper + robot_manager) both match once both are active; without this
        preference, "active_full" being requested and reached would never show
        as current, since "active" comes first and matches too.

        MUST be called while holding self._state_lock!
        """
        if self._target_profile and self._profile_matches_state(self._target_profile):
            return self._target_profile.name

        for name, profile in self._config.profiles.items():
            if self._profile_matches_state(profile):
                return name
        return "None"

    def _missing_profile_components(self, target_profile: SystemProfile) -> List[str]:
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

    def _profile_target_state(self, profile: SystemProfile, name: str) -> Optional[LifecycleState]:
        """
        Return a component's target state in the profile, or None if it's not targeted.

        MUST be called while holding self._state_lock!
        """
        targets = (
            profile.hardware_targets + profile.controller_targets + profile.lifecycle_node_targets
        )
        for target in targets:
            if target.name == name:
                return target.lifecycle_state
        return None

    def _profile_mismatches(self, profile: SystemProfile) -> List[str]:
        """
        List components not at the profile's target state, including missing ones.

        MUST be called while holding self._state_lock!
        """
        targets = (
            profile.hardware_targets + profile.controller_targets + profile.lifecycle_node_targets
        )
        mismatches = []
        for target in targets:
            observed = self._state.components.get(target.name)
            if observed is None or observed.lifecycle_state != target.lifecycle_state:
                mismatches.append(target.name)
        return mismatches

    def _is_profile_available(self, profile: SystemProfile) -> bool:
        """
        Check if a profile is currently achievable given observed component state.

        MUST be called while holding self._state_lock!
        """
        return not self._missing_profile_components(
            profile
        ) and not self._check_unsatisfiable_dependencies(profile)

    def _check_unsatisfiable_dependencies(self, profile: SystemProfile) -> List[str]:
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
