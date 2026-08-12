import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman_msgs.srv import SetGoal


class RosSetGoalServer:
    """ROS 2 service to set a named goal for Foreman Engine."""

    def __init__(self, node: Node, engine: ForemanEngine):
        self._node = node
        self._engine = engine
        self._poll_period = 0.05
        self.logger_prefix = "Adapters.RosSetGoalServer:"
        # Using MutuallyExclusiveCallbackGroup
        # If a service is processing, we reject new service requests.
        self._callback_group = MutuallyExclusiveCallbackGroup()

        self._srv = self._node.create_service(
            SetGoal,
            'foreman/set_goal',
            self._handle_set_goal,
            callback_group=self._callback_group
        )

        print()

        self._node.get_logger().info(f"{self.logger_prefix} Service /foreman/set_goal is ready.")

    def _handle_set_goal(self, request, response):
        """Set the target system state."""
        goal_name = request.goal
        # TODO: demote some of these to DEBUG logs.
        self._node.get_logger().info(
            f"{self.logger_prefix} Received request for goal '{goal_name}'")

        engine_response = self._engine.request_goal(goal_name)
        if not engine_response.success:
            self._node.get_logger().warning(f"{engine_response.message}")
            response.success = False
            response.message = engine_response.message
            return response

        self._node.get_logger().info(f"{engine_response.message}")

        while True:
            snapshot = self._engine.get_engine_snapshot()

            if snapshot.error.is_error:
                response.success = False
                response.message = (
                    f"[{snapshot.error.category}] {snapshot.error.message}"
                )
                self._node.get_logger().error(
                    f"{self.logger_prefix} Goal '{goal_name}' aborted: "
                    f"{response.message}")
                return response

            if snapshot.goal != goal_name:
                response.success = False
                response.message = (
                    f"Goal '{goal_name}' was preempted by goal '{snapshot.goal}'."
                )
                self._node.get_logger().warning(
                    f"{self.logger_prefix} {response.message}")
                return response

            if snapshot.at_goal:
                response.success = True
                response.message = f"Goal '{goal_name}' reached."
                self._node.get_logger().info(
                    f"{self.logger_prefix} {response.message}")
                return response

            time.sleep(self._poll_period)
