from rclpy.node import Node

from foreman.engine import ForemanEngine
from foreman.types import ErrorSnapshot
from foreman.types import ForemanSnapshot
from foreman_msgs.msg import ForemanErrorState
from foreman_msgs.msg import ForemanStatus


def _to_error_msg(snapshot: ErrorSnapshot) -> ForemanErrorState:
    msg = ForemanErrorState()
    msg.is_error = snapshot.is_error
    msg.category = snapshot.category
    msg.message = snapshot.message
    msg.components = list(snapshot.components or [])
    return msg


def _to_status_msg(snapshot: ForemanSnapshot) -> ForemanStatus:
    msg = ForemanStatus()
    msg.goal = snapshot.goal
    msg.ready = snapshot.ready
    msg.at_goal = snapshot.at_goal
    msg.error = _to_error_msg(snapshot.error)
    return msg


class RosStatusPublisher:
    """Publish the current Foreman engine status as a ROS 2 topic."""

    def __init__(
        self,
        node: Node,
        engine: ForemanEngine,
        publish_period: float = 0.1,
    ):
        self._node = node
        self._engine = engine
        self.logger_prefix = "Adapters.RosStatusPublisher:"

        self._publisher = self._node.create_publisher(
            ForemanStatus,
            'foreman/status',
            10
        )
        self._timer = self._node.create_timer(
            publish_period,
            self.publish_status,
            callback_group=self._node.callback_group_timer
        )

        self._node.get_logger().info(
            f"{self.logger_prefix} Topic /foreman/status is ready.")

    def publish_status(self):
        """Publish one snapshot of the current Foreman status."""
        self._publisher.publish(
            _to_status_msg(self._engine.get_engine_snapshot())
        )
