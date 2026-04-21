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
    ForemanErrorCategory
)
from controller_manager_msgs.msg import ControllerManagerActivity

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

        self._current_goal = config.goals.get('broadcast_only')
        self._is_ready = False # when we get first /activity reading
        self._error_state: Optional[ForemanError] = None
        self._last_issued_command: Optional[SystemTransitionCommand] = None

    @property
    def is_at_goal(self) -> bool:
        """Checks if there are any remaining transitions to reach the goal."""
        with self._state_lock:
            return self._locked_is_at_goal()

    def request_goal(self, goal_name: str) -> Tuple[bool, str]:
        """
        Request a new goal for the system
        Returns: (success, message)
        """
        goal = self._config.goals.get(goal_name)
        if not goal:
            return False, f"Goal '{goal_name}' not found in configuration."
        
        with self._state_lock:
            self._error_state = None # new goal received, clear error and try again.
            self._last_issued_command = None
            
            if self._current_goal == goal:
                if self._locked_is_at_goal():
                    return True, f"Already at goal '{goal_name}'."
                return True, f"Already transitioning to '{goal_name}'."
                
            self._current_goal = goal
            self._is_ready = not self._locked_any_goal_components_missing()
        
        return True, f"Goal '{goal_name}' requested."
    
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
            
    def set_system_state(self, components: List[Component]):
        """
        Set internal system state to that which is observed.
        Monitors for unexpected changes in component state.
        """

        #TODO: should we guard here against partial component updates? for example
        # if one component does not appear for some time in the set_system_state, is that
        # an error? Something to note.
        with self._state_lock:
            # detect unexpected changes
            unexpected_changes = []
            
            if self._is_ready and not self._error_state:
                for incoming in components:
                    existing = self._state.components.get(incoming.name)
                    
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

            # still update state
            self._state.components = {comp.name: comp for comp in components}
            self._is_ready = not self._locked_any_goal_components_missing()

            # if something unexpected happened, error and 
            if unexpected_changes:
                names = [change[0] for change in unexpected_changes]
                msgs = [f"{name} ({old}->{new})" for name, old, new in unexpected_changes]
                
                self._error_state = ForemanError(
                    category=ForemanErrorCategory.UNEXPECTED_STATE,
                    message=f"Unexpected state changes:\n{'\n'.join(msgs)}",
                    component_names=names
                )
                self._last_issued_command = None
                self._locked_abort_transition()

    @property
    def current_goal_name(self) -> str:
        return self._current_goal.name if self._current_goal else "None"

    @property
    def is_ready(self) -> bool:
        """Is the system observed and ready to plan?"""
        return self._is_ready

    def get_engine_snapshot(self) -> dict:
        """
        Returns a simplified snapshot of the system.
        """
        with self._state_lock:
            return {
                "goal": self.current_goal_name,
                "ready": self._is_ready,
                "at_goal": self._locked_is_at_goal(),
                "error": {
                    "is_error": self._error_state is not None,
                    "category": self._error_state.category.value if self._error_state else ForemanErrorCategory.NONE.value,
                    "message": self._error_state.message if self._error_state else "",
                    "components": self._error_state.component_names if self._error_state else []
                },
                "components": {
                    name: comp.lifecycle_state.name 
                    for name, comp in self._state.components.items()
                }
            }

    def _locked_is_at_goal(self) -> bool:
        """
        Checks if the current goal is reached.
        MUST be called while holding self._state_lock!
        """
        if not self._is_ready or not self._current_goal:
            return False
        
        # If planner returns nothing, we have reached the goal state
        return self._planner.get_next_transition(self._state, self._current_goal) is None

    def _locked_any_goal_components_missing(self) -> bool:
        """Checks if all components in the goal are present in current state.
        MUST be called while holding self._state_lock!
        """
        if not self._current_goal:
            return True
            
        all_component_goals = (
            self._current_goal.hardware_goals + 
            self._current_goal.controller_goals
        )
        
        for component_goal in all_component_goals:
            if component_goal.name not in self._state.components:
                return True
        return False

    def _locked_abort_transition(self):
        """
        Aborts any ongoing transitions by setting the current goal to exactly match the current state.
        MUST be called while holding self._state_lock!
        """
        if not self._is_ready:
            return

        hw_goals = []
        ctrl_goals = []
        
        for component in self._state.components.values():
            component_goal = Component(
                name=component.name, 
                component_type=component.component_type, 
                lifecycle_state=component.lifecycle_state
            )
            if component.component_type == ComponentType.HARDWARE:
                hw_goals.append(component_goal)
            else:
                ctrl_goals.append(component_goal)

        self._current_goal = SystemGoal(
            name="aborted",
            hardware_goals=hw_goals,
            controller_goals=ctrl_goals
        )