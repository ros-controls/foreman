import unittest
from unittest.mock import MagicMock

import rclpy

from foreman.adapters.ros_set_goal_server import RosSetGoalServer
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanResponse
from foreman.types import ForemanSnapshot
from foreman_msgs.srv import SetGoal


def _snapshot(goal="force_ctrl", ready=True, at_goal=False, error=None):
    """Build a ForemanSnapshot with a no-error default."""
    if error is None:
        error = ErrorSnapshot(
            is_error=False,
            category=ForemanErrorCategory.NONE.value,
            message="",
            components=[]
        )
    return ForemanSnapshot(goal=goal, ready=ready, at_goal=at_goal, error=error, components=[])


def _error_snapshot(message="boom"):
    return ErrorSnapshot(
        is_error=True,
        category=ForemanErrorCategory.EXECUTION.value,
        message=message,
        components=["ctrl_a"]
    )


class TestRosSetGoalServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_set_goal_service")
        self.addCleanup(self.node.destroy_node)
        self.engine = MagicMock()

    def _server(self):
        server = RosSetGoalServer(self.node, self.engine)
        server._poll_period = 0.0
        return server

    def test_service_is_advertised_as_foreman_set_goal(self):
        self._server()
        advertised = dict(self.node.get_service_names_and_types())
        self.assertIn("/foreman/set_goal", advertised)
        self.assertEqual(advertised["/foreman/set_goal"], ["foreman_msgs/srv/SetGoal"])

    def test_rejected_goal_returns_without_waiting(self):
        self.engine.request_goal.return_value = ForemanResponse(
            False, "Goal 'nope' not found in configuration.")
        request = SetGoal.Request(goal="nope")
        response = SetGoal.Response()

        response = self._server()._handle_set_goal(request, response)

        self.assertFalse(response.success)
        self.assertIn("not found", response.message)
        self.engine.get_engine_snapshot.assert_not_called()

    def test_succeeds_only_once_engine_reports_at_goal(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_goal=False),
            _snapshot(at_goal=False),
            _snapshot(at_goal=True),
        ]
        request = SetGoal.Request(goal="force_ctrl")
        response = SetGoal.Response()

        response = self._server()._handle_set_goal(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.message, "Goal 'force_ctrl' reached.")
        self.assertEqual(self.engine.get_engine_snapshot.call_count, 3)

    def test_engine_error_returns_failure(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_goal=False),
            _snapshot(error=_error_snapshot(message="Service rejected the transition.")),
        ]
        request = SetGoal.Request(goal="force_ctrl")
        response = SetGoal.Response()

        response = self._server()._handle_set_goal(request, response)

        self.assertFalse(response.success)
        self.assertIn("Service rejected the transition.", response.message)

    def test_preempted_goal_returns_failure(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(goal="other_goal")
        request = SetGoal.Request(goal="force_ctrl")
        response = SetGoal.Response()

        response = self._server()._handle_set_goal(request, response)

        self.assertFalse(response.success)
        self.assertIn("preempted", response.message)


if __name__ == "__main__":
    unittest.main()
