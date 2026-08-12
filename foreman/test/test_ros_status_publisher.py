import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from foreman.adapters.ros_status_publisher import _to_status_msg
from foreman.adapters.ros_status_publisher import RosStatusPublisher
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanSnapshot


def _snapshot():
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
        components=[]
    )


class TestToStatusMsg(unittest.TestCase):
    def test_maps_snapshot_fields(self):
        msg = _to_status_msg(_snapshot())

        self.assertEqual(msg.goal, "running")
        self.assertTrue(msg.ready)
        self.assertFalse(msg.at_goal)
        self.assertTrue(msg.error.is_error)
        self.assertEqual(msg.error.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(msg.error.message, "boom")
        self.assertEqual(list(msg.error.components), ["joint_trajectory_controller"])


class TestRosStatusPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_status_publisher")
        self.node.callback_group_timer = MutuallyExclusiveCallbackGroup()
        self.addCleanup(self.node.destroy_node)
        self.engine = MagicMock()

    def _publisher(self):
        return RosStatusPublisher(
            self.node,
            self.engine,
            publish_period=3600.0
        )

    def test_status_topic_is_advertised(self):
        self._publisher()
        advertised = dict(self.node.get_topic_names_and_types())
        self.assertIn("/foreman/status", advertised)
        self.assertEqual(advertised["/foreman/status"], ["foreman_msgs/msg/ForemanStatus"])

    def test_publish_status_publishes_engine_snapshot(self):
        self.engine.get_engine_snapshot.return_value = _snapshot()
        publisher = self._publisher()
        publisher._publisher = MagicMock()

        publisher.publish_status()

        published = publisher._publisher.publish.call_args[0][0]
        self.assertEqual(published.goal, "running")
        self.assertEqual(published.error.message, "boom")


if __name__ == "__main__":
    unittest.main()
