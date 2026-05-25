from rclpy.node import Node

from foreman.types import LifecycleState
from foreman.engine import ForemanEngine 

class AutostartAdapter:
    """Adapter to transition automatically to a desired state after all desired components are loaded."""
    
    STABLE_TICKS_REQUIRED = 50  # consecutive ticks with no state change before requesting transition

    def __init__(self, node: Node, engine: ForemanEngine, goal_name: str, autostart: bool = False):
        self._node = node
        self._autostart = autostart
        self.engine = engine
        self.goal_name = goal_name
        self.transition_success = False
        self._last_observed_states = None
        self._stable_ticks = 0

    def autostart(self):
        if not self._autostart:
            self._node.get_logger().info(f"Autostart parameter is set to false.")
            return

        if not self.all_components_ready():
            not_ready = self._get_not_ready_components()
            self._node.get_logger().info(f"Some components are not ready yet: {not_ready}. Waiting before requesting autostart...")
            self._stable_ticks = 0
            return

        if not self._is_state_stable():
            return

        self._node.get_logger().info(f"All components stable and ready. Requesting goal transition.")
        self.transition_success = self.send_goal_request()

    @property
    def is_done(self) -> bool:
        """True once the goal request was accepted successfully."""
        return self.transition_success

    def send_goal_request(self) -> bool:
        """Send goal request to the engine. Returns True if accepted."""
        response = self.engine.request_goal(self.goal_name)
        if response.success:
            self._node.get_logger().info(f"Autostart: {response.message}")
        else:
            self._node.get_logger().warn(f"Autostart failed: {response.message}")
        return response.success

    def _is_state_stable(self) -> bool:
        """Return True once the system state has been unchanged for STABLE_TICKS_REQUIRED consecutive ticks."""
        current_states = {c.name: c.lifecycle_state for c in self.engine.get_engine_snapshot().components}
        if current_states != self._last_observed_states:
            self._last_observed_states = current_states
            self._stable_ticks = 0
            return False
        self._stable_ticks += 1
        return self._stable_ticks >= self.STABLE_TICKS_REQUIRED

    def _get_not_ready_components(self, desired_state=LifecycleState.UNCONFIGURED):
        """Return names of tracked components that are missing or below the desired state."""
        snapshot = self.engine.get_engine_snapshot()
        observed = {c.name: c for c in snapshot.components}
        missing = [n for n in self.engine._config.tracked_components if n not in observed]
        wrong_state = [
            c.name for c in observed.values()
            if c.lifecycle_state < desired_state
        ]
        return missing + wrong_state

    def all_components_ready(self, desired_state=LifecycleState.UNCONFIGURED):
        """Check if all the tracked components are in the desired state."""
        snapshot = self.engine.get_engine_snapshot()
        observed = {c.name: c for c in snapshot.components}

        all_unconfigured = (
            all(name in observed for name in self.engine._config.tracked_components)
            and all(c.lifecycle_state >= desired_state for c in observed.values())
        )

        # self._node.get_logger().info(f"Here are all the components observed: {observed}.")
        # self._node.get_logger().info(f"Here are all the components tracked: {self.engine._config.tracked_components}.")
        
        return all_unconfigured 