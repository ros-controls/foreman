from typing import List, Optional, Tuple
import threading

from foreman.planner import Planner
from foreman.parser import ParsedScenario
from foreman.types import (
    Component,
    ComponentType,
    LifecycleState,
    SystemState,
    SystemTransitionCommand,
    SystemGoal,
    ForemanError,
    ForemanResponse,
    ForemanErrorCategory,
    ErrorSnapshot,
    ForemanSnapshot
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

        self._current_goal = None
        self._is_ready = False # when we get first /activity reading
        self._error_state: Optional[ForemanError] = None
        self._last_issued_command: Optional[SystemTransitionCommand] = None

    @property
    def is_at_goal(self) -> bool:
        """Checks if there are any remaining transitions to reach the goal."""
        with self._state_lock:
            return self._locked_is_at_goal()

    def request_goal(self, goal_name: str) -> ForemanResponse:
        """
        Request a new goal for the system
        Returns: (success, message)
        """
        goal = self._config.goals.get(goal_name)
        if not goal:
            return ForemanResponse(False, f"Goal '{goal_name}' not found in configuration.")
        
        with self._state_lock:
            if not self._is_ready:
                return ForemanResponse(False, "Foreman not ready. Is /activity topic being published?")

            missing_components = self._locked_missing_goal_components(goal)
            if missing_components:
                return ForemanResponse(
                    False, 
                    f"Cannot accept goal '{goal_name}'. Missing components in observed state: {missing_components}"
                )

            unsatisfiable = self._locked_check_unsatisfiable_dependencies(goal)
            if unsatisfiable:
                return ForemanResponse(
                    False,
                    f"Cannot accept goal '{goal_name}'. Unsatisfiable dependencies:\n"
                    + "\n".join(f"  - {msg}" for msg in unsatisfiable)
                )

            error_cleared_msg = "Error cleared on new goal. " if self._error_state else ""
            self._error_state = None # new goal received, clear error and try again.
            self._last_issued_command = None
            
            # TODO: minor. On first goal, if we're already at goal, we don't catch this, as self._current_goal == Null.
            # Fix this so we log "Already at goal"
            if self._current_goal == goal:
                if self._locked_is_at_goal():
                    return ForemanResponse(True, f"Already at goal '{goal_name}'.")
                return ForemanResponse(True, f"Already transitioning to '{goal_name}'.")
                
            self._current_goal = goal
        
        return ForemanResponse(True, f"{error_cleared_msg}Goal '{goal_name}' accepted.")
    
    def abort_goal(self, error: ForemanError):
        """Aborts the current goal by stopping transitions."""
        with self._state_lock:
            self._error_state = error
            self._last_issued_command = None
            self._locked_abort_transition()

    def get_next_transition(self) -> Optional[SystemTransitionCommand]:
        """
        Calculate the next step toward the goal.
        """
        if not self._current_goal:
            return None

        with self._state_lock:
            if not self._is_ready or self._error_state:
                return None
            
            cmd = self._planner.get_next_transition(self._state, self._current_goal)
            self._last_issued_command = cmd
            return cmd
            
    def set_system_state(self, components: List[Component]) -> ForemanResponse:
        """
        Set internal system state to that which is observed.
        Monitors for unexpected changes in component state.
        """
        with self._state_lock:
            # overwrite existing state
            previous_state = self._state.components
            self._state.components = {comp.name: comp for comp in components}
            
            was_ready = self._is_ready
            self._is_ready = True

            # In these cases, we just observe state
            if (self._error_state or 
                not was_ready or
                not self._current_goal):
                return ForemanResponse(True, "System state observed.")

            # otherwise, check for anomalies
            unexpected_changes = []
            missing_components = []

            # unexpected state drops
            for incoming in components:
                existing = previous_state.get(incoming.name)
                if existing and incoming.lifecycle_state != existing.lifecycle_state:
                    expected = (
                        self._last_issued_command and 
                        self._last_issued_command.component.name == incoming.name and 
                        self._last_issued_command.goal_state == incoming.lifecycle_state
                    )
                    if not expected:
                        unexpected_changes.append(
                            (incoming.name, existing.lifecycle_state.name, incoming.lifecycle_state.name)
                        )

            # unexpected missing components
            missing_components = self._locked_missing_goal_components(self._current_goal)

            # if any anomalies, emit error
            if unexpected_changes or missing_components:
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
                    component_names=list(set(error_components))
                )
                
                self._last_issued_command = None
                self._locked_abort_transition()

                return ForemanResponse(
                    success=False, 
                    message="Unexpected system state.", 
                    error=self._error_state
                )

            return ForemanResponse(True, "System state observed with no anomalies.")

    @property
    def current_goal_name(self) -> str:
        return self._current_goal.name if self._current_goal else "None"

    @property
    def is_ready(self) -> bool:
        """Is the system observed and ready to plan?"""
        return self._is_ready

    def get_engine_snapshot(self) -> ForemanSnapshot:
        """
        Returns a simplified snapshot of the system state.
        """
        with self._state_lock:
             return ForemanSnapshot(
                goal=self.current_goal_name,
                ready=self._is_ready,
                at_goal=self._locked_is_at_goal(),
                error=ErrorSnapshot(
                    is_error=self._error_state is not None,
                    category=self._error_state.category.value if self._error_state else ForemanErrorCategory.NONE.value,
                    message=self._error_state.message if self._error_state else "",
                    components=self._error_state.component_names if self._error_state else []
                ),
                components=list(self._state.components.values())
            )

    def _locked_is_at_goal(self) -> bool:
        """
        Checks if the current goal is reached.
        MUST be called while holding self._state_lock!
        """
        if not self._is_ready or not self._current_goal:
            return False
        
        # If planner returns nothing, we have reached the goal state
        return self._planner.get_next_transition(self._state, self._current_goal) is None

    def _locked_missing_goal_components(self, target_goal: SystemGoal) -> List[str]:
        """Checks if all components in the target_goal are present in current state.
        Returns a list of missing components.
        MUST be called while holding self._state_lock!
        """
        missing = []
        all_component_goals = (
            target_goal.hardware_goals + target_goal.controller_goals + target_goal.lifecycle_node_goals
        )
        
        for component_goal in all_component_goals:
            if component_goal.name not in self._state.components:
                missing.append(component_goal.name)
        return missing

    def _locked_check_unsatisfiable_dependencies(self, goal: SystemGoal) -> List[str]:
        """
        Validates that all controller dependencies in the goal can be satisfied.
        A dependency is satisfiable if:
        - It is already at or above the required state in current observed state, OR
        - It is included in the goal's infrastructure targets at or above the required state.
        Returns a list of error strings. Empty = all satisfiable.
        MUST be called while holding self._state_lock!
        """
        # TODO: refactor naming. Unfortunately, we treat lifecycle nodes same as hardware, so 
        # in places, like rule.required_hardware, we are thinking about lifecycle nodes as well. 
        # Lets use "infrastructure" for now to mean both of those
        goal_infrastructure_states = {}
        for comp in goal.hardware_goals + goal.lifecycle_node_goals:
            goal_infrastructure_states[comp.name] = comp.lifecycle_state
            
        errors = []
        for ctrl_goal in goal.controller_goals:
            rule = self._planner.rules.get(ctrl_goal.name)
            if not rule:
                continue

            # if stepping down, we don't care.
            if ctrl_goal.lifecycle_state == LifecycleState.UNCONFIGURED:
                continue

            for req in rule.required_hardware:
                if ctrl_goal.lifecycle_state == LifecycleState.ACTIVE:
                    required_state = req.state
                else:
                    # for configure, we need at least inactive.
                    required_state = LifecycleState.INACTIVE

                dependency_goal_state = goal_infrastructure_states.get(req.name)
                dependency_current = self._state.components.get(req.name)
                dependency_current_state = dependency_current.lifecycle_state if dependency_current else None

                satisfied_by_goal = dependency_goal_state is not None and dependency_goal_state >= required_state
                satisfied_by_current = dependency_current_state is not None and dependency_current_state >= required_state

                if not satisfied_by_goal and not satisfied_by_current:
                    state_str = dependency_current_state.name if dependency_current_state else "UNKNOWN"
                    errors.append(
                        f"'{ctrl_goal.name}' requires '{req.name}' at {required_state.name}, "
                        f"but it is {state_str} and not targeted in this goal"
                    )
        return errors

    def _locked_abort_transition(self):
        """
        Aborts any ongoing transitions.
        MUST be called while holding self._state_lock!
        """
        if not self._is_ready:
            return

        self._current_goal = None