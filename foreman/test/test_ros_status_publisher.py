import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from foreman.adapters.ros_status_publisher import RosStatusPublisher
from foreman.types import (
    Component,
    ComponentType,
    ErrorSnapshot,
    ForemanErrorCategory,
    ForemanSnapshot,
    LifecycleState,
    OperatingMode,
    StopState,
)


def _component(name="joint_trajectory_controller", state=LifecycleState.INACTIVE):
    return Component(name=name, component_type=ComponentType.CONTROLLER, lifecycle_state=state)


def _snapshot(components=None, all_profiles=None, available_profiles=None):
    return ForemanSnapshot(
        profile="running",
        ready=True,
        at_profile=False,
        error=ErrorSnapshot(
            is_error=True,
            category=ForemanErrorCategory.EXECUTION.value,
            message="boom",
            components=["joint_trajectory_controller"],
        ),
        components=components if components is not None else [_component()],
        all_profiles=all_profiles if all_profiles is not None else ["idle", "running"],
        available_profiles=available_profiles if available_profiles is not None else ["idle"],
        operating_mode=OperatingMode.AUTOMATIC.value,
        stop_state=StopState.RUNNING.value,
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
        self.assertEqual(published.profile, "running")
        self.assertTrue(published.ready)
        self.assertFalse(published.at_profile)
        self.assertTrue(published.error.is_error)
        self.assertEqual(published.error.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(published.error.message, "boom")
        self.assertEqual(
            [c.name for c in published.error.components], ["joint_trajectory_controller"]
        )
        self.assertEqual(
            published.error.components[0].component_type, ComponentType.CONTROLLER.value
        )
        self.assertEqual(published.error.components[0].lifecycle_state, "INACTIVE")
        self.assertEqual(list(published.all_profiles), ["idle", "running"])
        self.assertEqual(list(published.available_profiles), ["idle"])
        self.assertEqual(published.operating_mode, OperatingMode.AUTOMATIC.value)
        self.assertEqual(published.stop_state, StopState.RUNNING.value)

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
        changed.at_profile = True
        publisher.publish_status(changed)

        self.assertEqual(publisher._publisher.publish.call_count, 2)
        self.assertTrue(publisher._publisher.publish.call_args[0][0].at_profile)

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
