from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from foreman.types import ForemanSnapshot
from foreman_msgs.msg import ComponentState, ForemanStatus


class RosStatusPublisher:
    """Publish the current Foreman engine status as a ROS 2 topic."""

    def __init__(self, node: Node):
        self.logger_prefix = "Adapters.RosStatusPublisher:"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = node.create_publisher(ForemanStatus, "~/status", qos_profile)
        self._last_published = None

        node.get_logger().info(
            f"{self.logger_prefix} Topic {self._publisher.topic_name} is ready."
        )

    def publish_status(self, snapshot: ForemanSnapshot):
        """Publish the given Foreman status snapshot if it changed."""
        observed = {component.name: component for component in snapshot.components}

        msg = ForemanStatus()
        msg.profile = snapshot.profile
        msg.ready = snapshot.ready
        msg.at_profile = snapshot.at_profile
        msg.error.is_error = snapshot.error.is_error
        msg.error.category = snapshot.error.category
        msg.error.message = snapshot.error.message

        # The error names the blamed components.
        # A component that is no longer observed is still
        # named, with its state left empty.
        for name in snapshot.error.components or []:
            component_msg = ComponentState()
            component_msg.name = name
            component = observed.get(name)
            if component:
                component_msg.component_type = component.component_type.value
                component_msg.lifecycle_state = component.lifecycle_state.name
            msg.error.components.append(component_msg)

        msg.all_profiles = list(snapshot.all_profiles)
        msg.available_profiles = list(snapshot.available_profiles)
        msg.operating_mode = snapshot.operating_mode
        msg.stop_state = snapshot.stop_state

        if msg == self._last_published:
            return

        self._last_published = msg
        self._publisher.publish(msg)
