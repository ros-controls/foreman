from typing import Dict, List

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from rclpy.task import Future

from foreman.types import LifecycleState
from foreman.types import SystemTransitionCommand

# maps (current_state, goal_state) to the lifecycle transition ID
_TRANSITION_MAP = {
    (LifecycleState.UNCONFIGURED, LifecycleState.INACTIVE): Transition.TRANSITION_CONFIGURE,
    (LifecycleState.INACTIVE, LifecycleState.ACTIVE): Transition.TRANSITION_ACTIVATE,
    (LifecycleState.ACTIVE, LifecycleState.INACTIVE): Transition.TRANSITION_DEACTIVATE,
    (LifecycleState.INACTIVE, LifecycleState.UNCONFIGURED): Transition.TRANSITION_CLEANUP,
}


class LifecycleNodeServiceCaller:
    """Adapter for transitioning lifecycle nodes. Uses /<node>/change_state service."""

    def __init__(self, node: Node, lifecycle_nodes: List[str]):
        self._node = node
        self._clients: Dict[str, object] = {}
        self.logger_prefix = "Adapters.LifecycleNodeServiceCaller:"

        # TODO: minor: pass this explicitly? in controller_manager_service_caller.py as well.
        group = self._node.callback_group_services

        for lc_name in lifecycle_nodes:
            client = self._node.create_client(
                ChangeState,
                f'/{lc_name}/change_state',
                callback_group=group
            )
            self._clients[lc_name] = client

        if lifecycle_nodes:
            self._node.get_logger().info(
                f"{self.logger_prefix} Created clients for {lifecycle_nodes}"
            )

    def execute_transition(self, cmd: SystemTransitionCommand) -> Future:
        """Execute a lifecycle transition and return the Future."""
        name = cmd.component.name
        current = cmd.component.lifecycle_state
        goal = cmd.goal_state

        client = self._clients.get(name)
        if client is None:
            raise ValueError(f"No lifecycle client for node '{name}'")

        if not client.service_is_ready():
            raise RuntimeError(
                f"Service /{name}/change_state not ready. Is '{name}' running?"
            )

        transition_id = _TRANSITION_MAP.get((current, goal))
        if transition_id is None:
            raise ValueError(
                f"No valid lifecycle transition from {current.name} to {goal.name} for '{name}'"
            )

        self._node.get_logger().info(
            f"{self.logger_prefix} {name} -> {goal.name}"
        )

        req = ChangeState.Request()
        req.transition.id = transition_id
        return client.call_async(req)
