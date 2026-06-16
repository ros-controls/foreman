from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman_msgs.srv import SetGoal


class RosSetGoalServer:
    """
    ROS 2 service to set a named goal for Foreman Engine
    """

    def __init__(self, node: Node, engine: ForemanEngine):
        self._node = node
        self._engine = engine
        self.logger_prefix = "Adapters.RosSetGoalServer:"
        # Using MutuallyExclusiveCallbackGroup
        # If a service is processing, we reject new service requests.
        self._srv = self._node.create_service(
            SetGoal,
            'foreman/set_goal',
            self._handle_set_goal,
            callback_group=self._node.callback_group_services
        )

        print()

        self._node.get_logger().info(f"{self.logger_prefix} Service /foreman/set_goal is ready.")

    def _handle_set_goal(self, request, response):
        """Sets the target system state."""
        goal_name = request.goal
        # TODO: demote some of these to DEBUG logs.
        self._node.get_logger().info(
            f"{self.logger_prefix} Received request for goal '{goal_name}'")

        engine_response = self._engine.request_goal(goal_name)

        response.success = engine_response.success
        response.message = engine_response.message

        if not engine_response.success:
            self._node.get_logger().warn(f"{engine_response.message}")
        else:
            self._node.get_logger().info(f"{engine_response.message}")

        return response
