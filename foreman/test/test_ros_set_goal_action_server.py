import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.callback_groups import ReentrantCallbackGroup

from foreman.adapters.ros_set_goal_action_server import _to_error_msg
from foreman.adapters.ros_set_goal_action_server import RosSetGoalActionServer
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanResponse
from foreman.types import ForemanSnapshot


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


def _error_snapshot(category=ForemanErrorCategory.EXECUTION, message="boom", components=None):
    return ErrorSnapshot(
        is_error=True,
        category=category.value,
        message=message,
        components=components if components is not None else ["ctrl_a"]
    )


def _goal_handle(goal_name="force_ctrl"):
    """Fake action goal handle: active, not cancelled, records terminal calls."""
    handle = MagicMock()
    handle.request.goal = goal_name
    handle.is_active = True
    handle.is_cancel_requested = False
    return handle


class TestToErrorMsg(unittest.TestCase):
    def test_maps_every_field(self):
        msg = _to_error_msg(_error_snapshot(components=["a", "b"]))
        self.assertTrue(msg.is_error)
        self.assertEqual(msg.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(msg.message, "boom")
        self.assertEqual(list(msg.components), ["a", "b"])

    def test_none_components_become_empty_list(self):
        msg = _to_error_msg(ErrorSnapshot(
            is_error=False, category="None", message="", components=None))
        self.assertEqual(list(msg.components), [])


class TestRosSetGoalActionServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_set_goal_action")
        # the adapter reads this; the real node sets it, a bare node doesn't
        self.node.callback_group_subscriber = ReentrantCallbackGroup()
        self.addCleanup(self.node.destroy_node)
        self.engine = MagicMock()

    def _server(self):
        # poll_period=0 so the wait loop does not slow the tests down
        return RosSetGoalActionServer(self.node, self.engine, poll_period=0.0)

    def test_action_is_advertised_as_foreman_set_goal(self):
        self._server()
        advertised = dict(get_action_names_and_types(node=self.node))
        self.assertIn("/foreman/set_goal", advertised)
        self.assertEqual(advertised["/foreman/set_goal"], ["foreman_msgs/action/SetGoal"])

    def test_rejected_goal_aborts_without_waiting(self):
        self.engine.request_goal.return_value = ForemanResponse(
            False, "Goal 'nope' not found in configuration.")
        self.engine.get_engine_snapshot.return_value = _snapshot()
        handle = _goal_handle("nope")

        result = self._server()._execute(handle)

        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("not found", result.message)

    def test_succeeds_only_once_engine_reports_at_goal(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        # two polls in transition, then arrived
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_goal=False),
            _snapshot(at_goal=False),
            _snapshot(at_goal=True),
        ]
        handle = _goal_handle()

        result = self._server()._execute(handle)

        handle.succeed.assert_called_once()
        handle.abort.assert_not_called()
        self.assertTrue(result.success)
        # feedback published for each poll that was still in transition
        self.assertEqual(handle.publish_feedback.call_count, 2)

    def test_feedback_carries_current_error_state(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_goal=False),
            _snapshot(at_goal=True),
        ]
        handle = _goal_handle()

        self._server()._execute(handle)

        feedback = handle.publish_feedback.call_args[0][0]
        self.assertFalse(feedback.at_goal)
        self.assertFalse(feedback.error.is_error)

    def test_engine_error_aborts_and_reports_it(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_goal=False),
            _snapshot(error=_error_snapshot(message="Service rejected the transition.")),
        ]
        handle = _goal_handle()

        result = self._server()._execute(handle)

        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("Service rejected the transition.", result.message)
        self.assertTrue(result.error.is_error)
        self.assertEqual(result.error.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(list(result.error.components), ["ctrl_a"])

    def test_cancel_stops_waiting(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=False)
        handle = _goal_handle()
        handle.is_cancel_requested = True

        result = self._server()._execute(handle)

        handle.canceled.assert_called_once()
        handle.succeed.assert_not_called()
        handle.abort.assert_not_called()
        self.assertFalse(result.success)

    def test_preempted_goal_returns_without_terminal_call(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=False)
        handle = _goal_handle()
        handle.is_active = False

        result = self._server()._execute(handle)

        handle.succeed.assert_not_called()
        handle.abort.assert_not_called()
        handle.canceled.assert_not_called()
        self.assertFalse(result.success)

    def test_already_at_goal_succeeds_without_feedback(self):
        self.engine.request_goal.return_value = ForemanResponse(
            True, "Already at goal 'force_ctrl'.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=True)
        handle = _goal_handle()

        result = self._server()._execute(handle)

        handle.succeed.assert_called_once()
        handle.publish_feedback.assert_not_called()
        self.assertTrue(result.success)


if __name__ == "__main__":
    unittest.main()
