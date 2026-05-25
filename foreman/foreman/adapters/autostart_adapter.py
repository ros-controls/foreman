from rclpy.node import Node

from foreman.types import LifecycleState
from foreman.engine import ForemanEngine 

class AutostartAdapter:
    """Adapter to transition automatically to a desired state after all desired components are loaded."""
    def __init__(self, node: Node, engine: ForemanEngine, goal_name:str):
        self._node = node
        self._autostart = True
        self.engine = engine 
        self.goal_name = goal_name  
        self.goal_sent = False 

    def autostart(self): 
        if not self._autostart: 
            self._node.get_logger().info(f"Autostart parameter is set to false.")
            return 

        if not self.all_components_ready():
            not_ready = self._get_not_ready_components()
            self._node.get_logger().info(f"Some components are not ready yet: {not_ready}. Waiting before requesting autostart...")
            return
               
        if self.all_components_ready(): 
            if self.goal_sent: 
                return 
            # Transition to the goal state 
            #TODO: give info about the desired state
            self._node.get_logger().info(f"All components are ready. Requesting goal transition.")
            self.send_goal_request()
            self.goal_sent = True
            # return True 

    def send_goal_request(self): 
        """Send goal request to the Foreman Service."""
        response = self.engine.request_goal(self.goal_name)
        if response.success:
            self._node.get_logger().info(f"Autostart: {response.message}")
        else:
            self._node.get_logger().warn(f"Autostart failed: {response.message}")       


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
        return all_unconfigured



    