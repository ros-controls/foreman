import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman_msgs.srv import SetProfile


class RosSetProfileServer:
    """ROS 2 service to set a named profile for Foreman Engine."""

    def __init__(self, node: Node, engine: ForemanEngine, *, execution_lock):
        self._node = node
        self._engine = engine
        self._poll_period = 0.05
        self._execution_lock = execution_lock
        self._shutting_down = False
        self.logger_prefix = "Adapters.RosSetProfileServer:"
        # Let concurrent callers reach the execution lock and get rejected
        self._callback_group = ReentrantCallbackGroup()

        self._srv = self._node.create_service(
            SetProfile,
            "~/set_profile",
            self._handle_set_profile,
            callback_group=self._callback_group,
        )

        self._node.get_logger().info(f"{self.logger_prefix} Service set_profile is ready.")

    def request_shutdown(self):
        """Stop waiting for a profile, so a blocking call does not outlive the node."""
        self._shutting_down = True

    def _handle_set_profile(self, request, response):
        """Set the target system state."""
        profile_name = request.profile
        # TODO: demote some of these to DEBUG logs.
        self._node.get_logger().info(
            f"{self.logger_prefix} Received request for profile '{profile_name}'"
        )

        if not self._execution_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another set_profile request is already active."
            self._node.get_logger().warning(f"{self.logger_prefix} {response.message}")
            return response

        try:
            engine_response = self._engine.request_profile(profile_name)
            if not engine_response.success:
                self._node.get_logger().warning(f"{engine_response.message}")
                response.success = False
                response.message = engine_response.message
                return response

            self._node.get_logger().info(f"{engine_response.message}")

            while True:
                if self._shutting_down:
                    response.success = False
                    response.message = f"Stopped waiting for profile '{profile_name}'."
                    return response

                snapshot = self._engine.get_engine_snapshot()

                if snapshot.error.is_error:
                    response.success = False
                    response.message = f"[{snapshot.error.category}] {snapshot.error.message}"
                    self._node.get_logger().error(
                        f"{self.logger_prefix} Profile '{profile_name}' aborted: "
                        f"{response.message}"
                    )
                    return response

                if snapshot.target_profile != profile_name:
                    response.success = False
                    response.message = (
                        f"Profile '{profile_name}' was preempted by profile "
                        f"'{snapshot.target_profile}'."
                    )
                    self._node.get_logger().warning(f"{self.logger_prefix} {response.message}")
                    return response

                if snapshot.at_profile:
                    response.success = True
                    response.message = f"Profile '{profile_name}' reached."
                    self._node.get_logger().info(f"{self.logger_prefix} {response.message}")
                    return response

                time.sleep(self._poll_period)
        finally:
            self._execution_lock.release()
