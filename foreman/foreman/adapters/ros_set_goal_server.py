import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman_msgs.srv import SetGoal


class RosSetGoalServer:
    """ROS 2 service to set a named goal for Foreman Engine."""

    def __init__(self, node: Node, engine: ForemanEngine, *, execution_lock):
        self._node = node
        self._engine = engine
        self._poll_period = 0.05
        self._execution_lock = execution_lock
        self._shutting_down = False
        self.logger_prefix = "Adapters.RosSetGoalServer:"
        # Let concurrent callers reach the execution lock and get rejected
        self._callback_group = ReentrantCallbackGroup()

        self._srv = self._node.create_service(
            SetGoal, "~/set_goal", self._handle_set_goal, callback_group=self._callback_group
        )

        print()

        self._node.get_logger().info(f"{self.logger_prefix} Service set_goal is ready.")

    def request_shutdown(self):
        """Stop waiting for a goal, so a blocking call does not outlive the node."""
        self._shutting_down = True

    def _handle_set_goal(self, request, response):
        """Set the target system state."""
        goal_name = request.goal
        # TODO: demote some of these to DEBUG logs.
        self._node.get_logger().info(
            f"{self.logger_prefix} Received request for goal '{goal_name}'"
        )

        if not self._execution_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another set_goal request is already active."
            self._node.get_logger().warning(f"{self.logger_prefix} {response.message}")
            return response

        try:
            engine_response = self._engine.request_goal(goal_name)
            if not engine_response.success:
                self._node.get_logger().warning(f"{engine_response.message}")
                response.success = False
                response.message = engine_response.message
                return response

            self._node.get_logger().info(f"{engine_response.message}")

            while True:
                if self._shutting_down:
                    response.success = False
                    response.message = f"Stopped waiting for goal '{goal_name}'."
                    return response

                snapshot = self._engine.get_engine_snapshot()

                if snapshot.error.is_error:
                    response.success = False
                    response.message = f"[{snapshot.error.category}] {snapshot.error.message}"
                    self._node.get_logger().error(
                        f"{self.logger_prefix} Goal '{goal_name}' aborted: " f"{response.message}"
                    )
                    return response

                if snapshot.goal != goal_name:
                    response.success = False
                    response.message = (
                        f"Goal '{goal_name}' was preempted by goal '{snapshot.goal}'."
                    )
                    self._node.get_logger().warning(f"{self.logger_prefix} {response.message}")
                    return response

                if snapshot.at_goal:
                    response.success = True
                    response.message = f"Goal '{goal_name}' reached."
                    self._node.get_logger().info(f"{self.logger_prefix} {response.message}")
                    return response

                time.sleep(self._poll_period)
        finally:
            self._execution_lock.release()
