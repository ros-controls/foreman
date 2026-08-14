import time

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman.types import ForemanSnapshot
from foreman_msgs.action import SetGoal
from foreman_msgs.msg import ComponentState
from foreman_msgs.msg import ForemanErrorState


def _to_error_msg(snapshot: ForemanSnapshot) -> ForemanErrorState:
    """
    Convert the engine's error into its ROS representation.

    The error names the blamed components; their observed states come from the
    same snapshot, so a client sees what state each one was in. A component that
    is no longer observed is still named, with its state left empty.
    """
    observed = {component.name: component for component in snapshot.components}

    msg = ForemanErrorState()
    msg.is_error = snapshot.error.is_error
    msg.category = snapshot.error.category
    msg.message = snapshot.error.message

    for name in snapshot.error.components or []:
        component_msg = ComponentState()
        component_msg.name = name
        component = observed.get(name)
        if component:
            component_msg.component_type = component.component_type.value
            component_msg.lifecycle_state = component.lifecycle_state.name
        msg.components.append(component_msg)

    return msg


class RosSetGoalActionServer:
    """ROS 2 action interface to set the Foreman goal."""

    def __init__(
        self,
        node: Node,
        engine: ForemanEngine,
        poll_period: float = 0.05,
        *,
        execution_lock
    ):
        self._engine = engine
        self._poll_period = poll_period
        self._execution_lock = execution_lock
        self._logger = node.get_logger().get_child('action')

        self._action_server = ActionServer(
            node,
            SetGoal,
            'foreman/set_goal',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=node.callback_group_subscriber
        )

        self._logger.info("Action /foreman/set_goal is ready.")

    def _on_goal_request(self, goal_request) -> GoalResponse:
        self._logger.debug(f"Received request for goal '{goal_request.goal}'")
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
            self._logger.warning(result.message)
            goal_handle.abort()
            return result

        try:
            engine_response = self._engine.request_goal(goal_name)
            if not engine_response.success:
                self._logger.warning(engine_response.message)
                result.success = False
                result.message = engine_response.message
                goal_handle.abort()
                return result

            self._logger.debug(engine_response.message)

            feedback = SetGoal.Feedback()
            while True:
                if not goal_handle.is_active:
                    # Reachable when the action server is destroyed on shutdown:
                    result.success = False
                    result.message = f"Goal '{goal_name}' is no longer active."
                    return result

                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = f"Stopped waiting for goal '{goal_name}'."
                    result.error = _to_error_msg(self._engine.get_engine_snapshot())
                    goal_handle.canceled()
                    return result

                snapshot = self._engine.get_engine_snapshot()
                error_msg = _to_error_msg(snapshot)

                if snapshot.error.is_error:
                    result.success = False
                    result.message = f"[{snapshot.error.category}] {snapshot.error.message}"
                    result.error = error_msg
                    self._logger.error(f"Goal '{goal_name}' aborted: {result.message}")
                    goal_handle.abort()
                    return result

                if snapshot.at_goal:
                    result.success = True
                    result.message = f"Goal '{goal_name}' reached."
                    result.error = error_msg
                    self._logger.info(result.message)
                    goal_handle.succeed()
                    return result

                feedback.at_goal = False
                feedback.error = error_msg
                goal_handle.publish_feedback(feedback)

                time.sleep(self._poll_period)
        finally:
            self._execution_lock.release()
