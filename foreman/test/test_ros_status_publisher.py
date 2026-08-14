import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy

from foreman.adapters.ros_status_publisher import RosStatusPublisher
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanSnapshot
from foreman.types import LifecycleState


def _component(name="joint_trajectory_controller", state=LifecycleState.INACTIVE):
    return Component(
        name=name,
        component_type=ComponentType.CONTROLLER,
        lifecycle_state=state
    )


def _snapshot(components=None):
    return ForemanSnapshot(
        goal="running",
        ready=True,
        at_goal=False,
        error=ErrorSnapshot(
            is_error=True,
            category=ForemanErrorCategory.EXECUTION.value,
            message="boom",
            components=["joint_trajectory_controller"],
        ),
        components=components if components is not None else [_component()]
    )


class TestRosStatusPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_status_publisher")
        self.addCleanup(self.node.destroy_node)

    def test_status_topic_is_advertised_in_node_namespace(self):
        RosStatusPublisher(self.node)
        expected = f"/{self.node.get_name()}/status"
        advertised = dict(self.node.get_topic_names_and_types())
        self.assertIn(expected, advertised)
        self.assertEqual(advertised[expected], ["foreman_msgs/msg/ForemanStatus"])

    def test_publish_status_maps_every_snapshot_field(self):
        publisher = RosStatusPublisher(self.node)
        publisher._publisher = MagicMock()

        publisher.publish_status(_snapshot())

        published = publisher._publisher.publish.call_args[0][0]
        self.assertEqual(published.goal, "running")
        self.assertTrue(published.ready)
        self.assertFalse(published.at_goal)
        self.assertTrue(published.error.is_error)
        self.assertEqual(published.error.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(published.error.message, "boom")
        self.assertEqual(
            [c.name for c in published.error.components], ["joint_trajectory_controller"])
        self.assertEqual(published.error.components[0].component_type,
                         ComponentType.CONTROLLER.value)
        self.assertEqual(published.error.components[0].lifecycle_state, "INACTIVE")

    def test_status_topic_is_transient_local(self):
        publisher = RosStatusPublisher(self.node)

        qos = publisher._publisher.qos_profile
        self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(qos.depth, 1)

    def test_unchanged_status_is_not_republished(self):
        publisher = RosStatusPublisher(self.node)
        publisher._publisher = MagicMock()

        publisher.publish_status(_snapshot())
        publisher.publish_status(_snapshot())
        publisher.publish_status(_snapshot())

        self.assertEqual(publisher._publisher.publish.call_count, 1)

    def test_changed_status_is_republished(self):
        publisher = RosStatusPublisher(self.node)
        publisher._publisher = MagicMock()

        publisher.publish_status(_snapshot())

        changed = _snapshot()
        changed.at_goal = True
        publisher.publish_status(changed)

        self.assertEqual(publisher._publisher.publish.call_count, 2)
        self.assertTrue(publisher._publisher.publish.call_args[0][0].at_goal)

    def test_publish_status_handles_missing_error_components(self):
        publisher = RosStatusPublisher(self.node)
        publisher._publisher = MagicMock()

        snapshot = _snapshot()
        snapshot.error.components = None

        publisher.publish_status(snapshot)

        published = publisher._publisher.publish.call_args[0][0]
        self.assertEqual(list(published.error.components), [])

    def test_vanished_component_is_named_without_a_state(self):
        # A component that dropped out of /activity is no longer observed, so
        # only its name is known.
        publisher = RosStatusPublisher(self.node)
        publisher._publisher = MagicMock()

        publisher.publish_status(_snapshot(components=[_component("something_else")]))

        published = publisher._publisher.publish.call_args[0][0]
        self.assertEqual(len(published.error.components), 1)
        self.assertEqual(published.error.components[0].name, "joint_trajectory_controller")
        self.assertEqual(published.error.components[0].component_type, "")
        self.assertEqual(published.error.components[0].lifecycle_state, "")


if __name__ == "__main__":
    unittest.main()
