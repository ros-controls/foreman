import unittest
from unittest.mock import MagicMock, PropertyMock

import rclpy
from controller_manager_msgs.msg import ControllerManagerActivity, NamedLifecycleState
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from foreman.adapters.component_state_monitor import ComponentStateMonitor
from foreman.types import ComponentType, ForemanErrorCategory, ForemanResponse, LifecycleState


def _named_state(name, state_id):
    return NamedLifecycleState(name=name, state=State(id=state_id))


def _matched_info(current_count):
    """Stand-in for QoSSubscriptionMatchedInfo -- the adapter only reads .current_count."""
    return MagicMock(current_count=current_count)


class TestComponentStateMonitor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_component_state_monitor")
        # the adapter reads these; the real node sets them, a bare node doesn't
        self.node.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.node.callback_group_subscriber = ReentrantCallbackGroup()
        self.addCleanup(self.node.destroy_node)

        self.engine = MagicMock()
        self.engine.is_ready = False
        self.engine.set_system_state.return_value = ForemanResponse(True, "ok")

        self.monitor = ComponentStateMonitor(
            node=self.node,
            engine=self.engine,
            controller_manager_name="controller_manager",
            lifecycle_nodes=["lc1"],
        )

    def test_when_activity_received_expect_hardware_and_controllers_pushed_to_engine(self):
        msg = ControllerManagerActivity(
            hardware_components=[_named_state("hw1", LifecycleState.ACTIVE.value)],
            controllers=[_named_state("ctrl1", LifecycleState.INACTIVE.value)],
        )

        self.monitor._activity_callback(msg)

        pushed = {c.name: c for c in self.engine.set_system_state.call_args[0][0]}
        self.assertEqual(pushed["hw1"].component_type, ComponentType.HARDWARE)
        self.assertEqual(pushed["hw1"].lifecycle_state, LifecycleState.ACTIVE)
        self.assertEqual(pushed["ctrl1"].component_type, ComponentType.CONTROLLER)
        self.assertEqual(pushed["ctrl1"].lifecycle_state, LifecycleState.INACTIVE)

    def test_when_activity_has_unknown_state_id_expect_component_skipped(self):
        msg = ControllerManagerActivity(
            hardware_components=[_named_state("hw1", 99)],
            controllers=[_named_state("ctrl1", 99)],
        )

        self.monitor._activity_callback(msg)

        pushed = self.engine.set_system_state.call_args[0][0]
        self.assertEqual(pushed, [])

    def test_when_lifecycle_publisher_appears_expect_get_state_requested(self):
        client = MagicMock()
        self.monitor._lc_node_get_state_clients["lc1"] = client

        self.monitor._on_lifecycle_publisher_matched("lc1", _matched_info(1))

        self.assertTrue(self.monitor._lc_nodes_alive["lc1"])
        client.call_async.assert_called_once()

    def test_when_lifecycle_publisher_matched_again_while_alive_expect_no_duplicate_call(self):
        client = MagicMock()
        self.monitor._lc_node_get_state_clients["lc1"] = client
        self.monitor._on_lifecycle_publisher_matched("lc1", _matched_info(1))

        self.monitor._on_lifecycle_publisher_matched("lc1", _matched_info(1))

        client.call_async.assert_called_once()

    def test_when_lifecycle_publisher_disappears_expect_component_marked_finalized(self):
        self.monitor._lc_nodes_alive["lc1"] = True

        self.monitor._on_lifecycle_publisher_matched("lc1", _matched_info(0))

        self.assertFalse(self.monitor._lc_nodes_alive["lc1"])
        pushed = {c.name: c for c in self.engine.set_system_state.call_args[0][0]}
        self.assertEqual(pushed["lc1"].lifecycle_state, LifecycleState.FINALIZED)

    def test_when_publisher_disappears_but_was_never_alive_expect_no_change(self):
        self.monitor._on_lifecycle_publisher_matched("lc1", _matched_info(0))

        self.engine.set_system_state.assert_not_called()

    def test_when_get_state_response_succeeds_expect_component_state_pushed(self):
        future = MagicMock()
        future.result.return_value = GetState.Response(
            current_state=State(id=LifecycleState.ACTIVE.value)
        )

        self.monitor._on_lifecycle_get_state_response("lc1", future)

        pushed = {c.name: c for c in self.engine.set_system_state.call_args[0][0]}
        self.assertEqual(pushed["lc1"].lifecycle_state, LifecycleState.ACTIVE)

    def test_when_get_state_response_raises_expect_no_state_pushed(self):
        future = MagicMock()
        future.result.side_effect = RuntimeError("service call failed")

        self.monitor._on_lifecycle_get_state_response("lc1", future)

        self.engine.set_system_state.assert_not_called()

    def test_when_transition_event_received_expect_component_state_pushed(self):
        msg = TransitionEvent(goal_state=State(id=LifecycleState.INACTIVE.value))

        self.monitor._lifecycle_transition_event_callback("lc1", msg)

        pushed = {c.name: c for c in self.engine.set_system_state.call_args[0][0]}
        self.assertEqual(pushed["lc1"].lifecycle_state, LifecycleState.INACTIVE)

    def test_when_transition_event_has_unknown_goal_state_expect_ignored(self):
        msg = TransitionEvent(goal_state=State(id=99))

        self.monitor._lifecycle_transition_event_callback("lc1", msg)

        self.engine.set_system_state.assert_not_called()

    def test_when_engine_becomes_ready_expect_ready_logged(self):
        type(self.engine).is_ready = PropertyMock(side_effect=[False, True])
        logger = MagicMock()
        self.node.get_logger = MagicMock(return_value=logger)

        self.monitor._activity_callback(ControllerManagerActivity())

        self.assertTrue(any("READY" in call.args[0] for call in logger.info.call_args_list))

    def test_when_engine_reports_unexpected_state_error_expect_warning_logged(self):
        self.engine.set_system_state.return_value = ForemanResponse(
            False,
            "state mismatch",
            error=MagicMock(category=ForemanErrorCategory.UNEXPECTED_STATE),
        )
        logger = MagicMock()
        self.node.get_logger = MagicMock(return_value=logger)

        self.monitor._activity_callback(ControllerManagerActivity())

        logger.warning.assert_called_once()
        logger.error.assert_not_called()

    def test_when_engine_reports_other_error_expect_error_logged(self):
        self.engine.set_system_state.return_value = ForemanResponse(
            False, "rejected", error=MagicMock(category=ForemanErrorCategory.EXECUTION)
        )
        logger = MagicMock()
        self.node.get_logger = MagicMock(return_value=logger)

        self.monitor._activity_callback(ControllerManagerActivity())

        logger.error.assert_called_once()
        logger.warning.assert_not_called()

    def test_when_response_reports_missing_components_expect_warning_logged_every_time(self):
        self.engine.set_system_state.return_value = ForemanResponse(
            True, "ok", missing_components=["hw1"]
        )
        logger = MagicMock()
        self.node.get_logger = MagicMock(return_value=logger)

        self.monitor._activity_callback(ControllerManagerActivity())
        self.monitor._activity_callback(ControllerManagerActivity())

        missing_warnings = [
            call for call in logger.warning.call_args_list if "hw1" in call.args[0]
        ]
        self.assertEqual(len(missing_warnings), 2)


if __name__ == "__main__":
    unittest.main()
