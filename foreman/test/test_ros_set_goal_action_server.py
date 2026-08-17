import threading
import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.callback_groups import ReentrantCallbackGroup

from foreman.adapters.ros_set_goal_action_server import _to_error_msg
from foreman.adapters.ros_set_goal_action_server import RosSetGoalActionServer
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanResponse
from foreman.types import ForemanSnapshot
from foreman.types import LifecycleState


def _component(name="ctrl_a", state=LifecycleState.INACTIVE):
    return Component(name=name, component_type=ComponentType.CONTROLLER, lifecycle_state=state)


def _snapshot(goal="force_ctrl", ready=True, at_goal=False, error=None, components=None):
    """Build a ForemanSnapshot with a no-error default."""
    if error is None:
        error = ErrorSnapshot(
            is_error=False, category=ForemanErrorCategory.NONE.value, message="", components=[]
        )
    return ForemanSnapshot(
        goal=goal,
        ready=ready,
        at_goal=at_goal,
        error=error,
        components=components if components is not None else [],
    )


def _error_snapshot(category=ForemanErrorCategory.EXECUTION, message="boom", components=None):
    return ErrorSnapshot(
        is_error=True,
        category=category.value,
        message=message,
        components=components if components is not None else ["ctrl_a"],
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
        msg = _to_error_msg(_snapshot(error=_error_snapshot(components=["a", "b"])))
        self.assertTrue(msg.is_error)
        self.assertEqual(msg.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual(msg.message, "boom")
        self.assertEqual([c.name for c in msg.components], ["a", "b"])

    def test_none_components_become_empty_list(self):
        msg = _to_error_msg(
            _snapshot(
                error=ErrorSnapshot(is_error=False, category="None", message="", components=None)
            )
        )
        self.assertEqual(list(msg.components), [])

    def test_blamed_components_carry_their_observed_state(self):
        msg = _to_error_msg(
            _snapshot(
                error=_error_snapshot(components=["ctrl_a"]),
                components=[_component("ctrl_a", LifecycleState.ACTIVE)],
            )
        )

        self.assertEqual(len(msg.components), 1)
        self.assertEqual(msg.components[0].name, "ctrl_a")
        self.assertEqual(msg.components[0].component_type, ComponentType.CONTROLLER.value)
        self.assertEqual(msg.components[0].lifecycle_state, "ACTIVE")

    def test_vanished_component_is_named_without_a_state(self):
        # "Required components vanished from /activity" drops the component from
        # the observed state, so only its name is known.
        msg = _to_error_msg(
            _snapshot(
                error=_error_snapshot(components=["gone"]), components=[_component("still_here")]
            )
        )

        self.assertEqual(len(msg.components), 1)
        self.assertEqual(msg.components[0].name, "gone")
        self.assertEqual(msg.components[0].component_type, "")
        self.assertEqual(msg.components[0].lifecycle_state, "")


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

    def _server(self, execution_lock=None):
        # poll_period=0 so the wait loop does not slow the tests down
        if execution_lock is None:
            execution_lock = threading.Lock()
        return RosSetGoalActionServer(
            self.node, self.engine, poll_period=0.0, execution_lock=execution_lock
        )

    def test_action_is_advertised_in_node_namespace(self):
        self._server()
        expected = f"/{self.node.get_name()}/set_goal"
        advertised = dict(get_action_names_and_types(node=self.node))
        self.assertIn(expected, advertised)
        self.assertEqual(advertised[expected], ["foreman_msgs/action/SetGoal"])

    def test_shutdown_stops_waiting_for_the_goal(self):
        # is_active stays True on shutdown, so without _shutting_down the wait
        # loop never ends and the process cannot exit.
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=False)
        server = self._server()
        server.request_shutdown()

        result = server._execute(_goal_handle())

        self.assertFalse(result.success)
        self.assertIn("Stopped waiting", result.message)

    def test_rejected_goal_aborts_without_waiting(self):
        self.engine.request_goal.return_value = ForemanResponse(
            False, "Goal 'nope' not found in configuration."
        )
        self.engine.get_engine_snapshot.return_value = _snapshot()
        handle = _goal_handle("nope")

        result = self._server()._execute(handle)

        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("not found", result.message)

    def test_rejected_goal_does_not_report_a_leftover_engine_error(self):
        self.engine.request_goal.return_value = ForemanResponse(
            False, "Goal 'nope' not found in configuration."
        )
        self.engine.get_engine_snapshot.return_value = _snapshot(
            error=_error_snapshot(message="leftover failure")
        )

        result = self._server()._execute(_goal_handle("nope"))

        self.assertFalse(result.success)
        self.assertIn("not found", result.message)
        self.assertFalse(result.error.is_error)
        self.assertEqual(result.error.message, "")

    def test_busy_set_goal_execution_aborts_without_requesting_goal(self):
        self.engine.get_engine_snapshot.return_value = _snapshot()
        execution_lock = threading.Lock()
        execution_lock.acquire()
        handle = _goal_handle()

        result = self._server(execution_lock=execution_lock)._execute(handle)

        handle.abort.assert_called_once()
        self.engine.request_goal.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("already active", result.message)
        execution_lock.release()

    def test_busy_set_goal_execution_does_not_report_an_engine_error(self):
        self.engine.get_engine_snapshot.return_value = _snapshot(
            error=_error_snapshot(message="unrelated failure")
        )
        execution_lock = threading.Lock()
        execution_lock.acquire()

        result = self._server(execution_lock=execution_lock)._execute(_goal_handle())

        self.assertFalse(result.success)
        self.assertIn("already active", result.message)
        self.assertFalse(result.error.is_error)
        self.assertEqual(result.error.message, "")
        execution_lock.release()

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

    def test_successful_goal_releases_execution_lock(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=True)
        execution_lock = threading.Lock()
        handle = _goal_handle()

        result = self._server(execution_lock=execution_lock)._execute(handle)

        self.assertTrue(result.success)
        self.assertTrue(execution_lock.acquire(blocking=False))
        execution_lock.release()

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
            _snapshot(
                error=_error_snapshot(message="Service rejected the transition."),
                components=[_component("ctrl_a", LifecycleState.INACTIVE)],
            ),
        ]
        handle = _goal_handle()

        result = self._server()._execute(handle)

        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("Service rejected the transition.", result.message)
        self.assertTrue(result.error.is_error)
        self.assertEqual(result.error.category, ForemanErrorCategory.EXECUTION.value)
        self.assertEqual([c.name for c in result.error.components], ["ctrl_a"])
        self.assertEqual(result.error.components[0].lifecycle_state, "INACTIVE")

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

    def test_inactive_goal_returns_without_terminal_call(self):
        self.engine.request_goal.return_value = ForemanResponse(True, "Goal accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=False)
        handle = _goal_handle()
        handle.is_active = False

        result = self._server()._execute(handle)

        handle.succeed.assert_not_called()
        handle.abort.assert_not_called()
        handle.canceled.assert_not_called()
        self.assertFalse(result.success)
        self.assertIn("Stopped waiting", result.message)

    def test_already_at_goal_succeeds_without_feedback(self):
        self.engine.request_goal.return_value = ForemanResponse(
            True, "Already at goal 'force_ctrl'."
        )
        self.engine.get_engine_snapshot.return_value = _snapshot(at_goal=True)
        handle = _goal_handle()

        result = self._server()._execute(handle)

        handle.succeed.assert_called_once()
        handle.publish_feedback.assert_not_called()
        self.assertTrue(result.success)


if __name__ == "__main__":
    unittest.main()
