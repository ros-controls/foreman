import time

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman.types import ErrorSnapshot
from foreman_msgs.action import SetGoal
from foreman_msgs.msg import ForemanErrorState


def _to_error_msg(snapshot: ErrorSnapshot) -> ForemanErrorState:
    """Convert the engine's error snapshot into its ROS representation."""
    msg = ForemanErrorState()
    msg.is_error = snapshot.is_error
    msg.category = snapshot.category
    msg.message = snapshot.message
    msg.components = list(snapshot.components or [])
    return msg


class RosSetGoalActionServer:
    """Drive the system to a named goal with a ROS 2 action."""

    def __init__(
        self,
        node: Node,
        engine: ForemanEngine,
        poll_period: float = 0.05,
        *,
        execution_lock
    ):
        self._node = node
        self._engine = engine
        self._poll_period = poll_period
        self._execution_lock = execution_lock
        self.logger_prefix = "Adapters.RosSetGoalActionServer:"

        self._action_server = ActionServer(
            node,
            SetGoal,
            'foreman/set_goal',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=node.callback_group_subscriber
        )

        self._node.get_logger().info(f"{self.logger_prefix} Action /foreman/set_goal is ready.")

    def _on_goal_request(self, goal_request) -> GoalResponse:
        self._node.get_logger().info(
            f"{self.logger_prefix} Received request for goal '{goal_request.goal}'")
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        """Cancel waiting without rolling the system back."""
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal_name = goal_handle.request.goal
        result = SetGoal.Result()

        if not self._execution_lock.acquire(blocking=False):
            result.success = False
            result.message = "Another set_goal request is already active."
            result.error = _to_error_msg(self._engine.get_engine_snapshot().error)
            self._node.get_logger().warning(f"{self.logger_prefix} {result.message}")
            goal_handle.abort()
            return result

        try:
            engine_response = self._engine.request_goal(goal_name)
            if not engine_response.success:
                self._node.get_logger().warning(f"{engine_response.message}")
                result.success = False
                result.message = engine_response.message
                result.error = _to_error_msg(self._engine.get_engine_snapshot().error)
                goal_handle.abort()
                return result

            self._node.get_logger().info(f"{engine_response.message}")

            feedback = SetGoal.Feedback()
            while True:
                if not goal_handle.is_active:
                    result.success = False
                    result.message = f"Goal '{goal_name}' was preempted."
                    return result

                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = f"Stopped waiting for goal '{goal_name}'."
                    result.error = _to_error_msg(self._engine.get_engine_snapshot().error)
                    goal_handle.canceled()
                    return result

                snapshot = self._engine.get_engine_snapshot()
                error_msg = _to_error_msg(snapshot.error)

                if snapshot.error.is_error:
                    result.success = False
                    result.message = f"[{snapshot.error.category}] {snapshot.error.message}"
                    result.error = error_msg
                    self._node.get_logger().error(
                        f"{self.logger_prefix} Goal '{goal_name}' aborted: {result.message}")
                    goal_handle.abort()
                    return result

                if snapshot.at_goal:
                    result.success = True
                    result.message = f"Goal '{goal_name}' reached."
                    result.error = error_msg
                    self._node.get_logger().info(f"{self.logger_prefix} {result.message}")
                    goal_handle.succeed()
                    return result

                feedback.at_goal = False
                feedback.error = error_msg
                goal_handle.publish_feedback(feedback)

                time.sleep(self._poll_period)
        finally:
            self._execution_lock.release()
