from typing import List, Optional, Tuple
import threading

from foreman.planner import Planner
from foreman.parser import ParsedScenario
from foreman.types import (
    Component,
    ComponentType,
    LifecycleState,
    SystemState,
    SystemTransitionCommand
)
from controller_manager_msgs.msg import ControllerManagerActivity

class ForemanEngine:
    """
    Foreman domain facade.
    All business logic is here, no ROS, jsut python.
    """
    # TODO: rework names. Write an engine_api.py using Typing.Protocol when API solidifies.

    def __init__(self, config: ParsedScenario, state_lock: threading.Lock):
        self._config = config
        self._planner = Planner(config.dependency_rules)
        self._state = SystemState()
        self._state_lock = state_lock

        # TODO: somehow set this up automatically. Current goal will be currently read state.
        self._current_goal = config.goals.get('broadcast_only')
        self._is_ready = False # when we get first /activity reading

    def request_goal(self, goal_name: str) -> Tuple[bool, str]:
        """
        Request a new goal for the system
        Returns: (success, message)
        """
        goal = self._config.goals.get(goal_name)
        if not goal:
            return False, f"Goal '{goal_name}' not found in configuration."
        
        with self._state_lock:
            self._current_goal = goal
        
        # TODO: ok for now, but do we return more informative error structs for frontends?
        return True, f"Goal '{goal_name}' requested."

    def get_next_transition(self) -> List[SystemTransitionCommand]:
        """
        Calculate the next step toward the goal.
        """
        if not self._current_goal:
            return []

        with self._state_lock:
            if not self._is_ready:
                return []
            
            return self._planner.calculate_transitions(self._state, self._current_goal)

    def set_system_state(self, components: List[Component]):
        """
        Set internal system state to that which is observed.
        The internal state should exactly match that state.
        """
        with self._state_lock:
            self._state.components = {comp.name: comp for comp in components}
            self._is_ready = not self._any_goal_components_missing()

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
                "components": {
                    name: comp.lifecycle_state.name 
                    for name, comp in self._state.components.items()
                }
            }

    def _any_goal_components_missing(self) -> bool:
        """Checks if all components in the goal are present in current state."""
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