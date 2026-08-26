import types
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node

import foreman.node as node_module
from foreman.node import ForemanNode
from foreman.types import (
    Component,
    ComponentType,
    ForemanErrorCategory,
    LifecycleState,
    SystemTransitionCommand,
)


def _future(done=True, result=None, exception=None):
    future = MagicMock()
    future.done.return_value = done
    if exception is not None:
        future.result.side_effect = exception
    else:
        future.result.return_value = result
    return future


def _command(name="hw1", component_type=ComponentType.HARDWARE):
    return SystemTransitionCommand(
        Component(name, component_type, LifecycleState.INACTIVE), LifecycleState.ACTIVE
    )


class TestForemanNodeMainLoop(unittest.TestCase):
    """
    Exercises callback_main_loop()/_log_and_abort_profile() directly, bound onto a
    bare rclpy node -- ForemanNode's real __init__ needs a config_path ROS
    parameter pointing at a real scenario file, which these tests don't need.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_foreman_node_main_loop")
        self.addCleanup(self.node.destroy_node)

        self.node.foreman_config = MagicMock(autostart_profile="")
        self.node.autostart_adapter = MagicMock(is_done=False)
        self.node.foreman_engine = MagicMock()
        self.node.foreman_engine.get_next_transition.return_value = None
        self.node.ros_status_publisher = MagicMock()
        self.node.controller_manager_service_caller = MagicMock()
        self.node.lifecycle_node_service_caller = MagicMock()
        self.node._service_call_active_future = None
        self.node._active_transition = None
        self.node.callback_main_loop = types.MethodType(ForemanNode.callback_main_loop, self.node)
        self.node._log_and_abort_profile = types.MethodType(
            ForemanNode._log_and_abort_profile, self.node
        )

    def test_when_autostart_configured_and_not_done_expect_autostart_called(self):
        self.node.foreman_config.autostart_profile = "active"
        self.node.autostart_adapter.is_done = False

        self.node.callback_main_loop()

        self.node.autostart_adapter.autostart.assert_called_once()

    def test_when_autostart_done_expect_autostart_not_called_again(self):
        self.node.foreman_config.autostart_profile = "active"
        self.node.autostart_adapter.is_done = True

        self.node.callback_main_loop()

        self.node.autostart_adapter.autostart.assert_not_called()

    def test_when_service_call_still_pending_expect_no_new_command_issued(self):
        self.node._service_call_active_future = _future(done=False)

        self.node.callback_main_loop()

        self.node.foreman_engine.get_next_transition.assert_not_called()

    def test_when_controller_manager_service_rejects_expect_profile_aborted(self):
        self.node._active_transition = _command("ctrl1", ComponentType.CONTROLLER)
        self.node._service_call_active_future = _future(done=True, result=MagicMock(ok=False))

        self.node.callback_main_loop()

        fault = self.node.foreman_engine.abort_profile.call_args[0][0]
        self.assertEqual(fault.category, ForemanErrorCategory.EXECUTION)
        self.assertEqual(fault.component_names, ["ctrl1"])
        self.assertIsNone(self.node._service_call_active_future)
        self.assertIsNone(self.node._active_transition)

    def test_when_lifecycle_node_service_rejects_expect_profile_aborted(self):
        self.node._active_transition = _command("lc1", ComponentType.LIFECYCLE_NODE)
        self.node._service_call_active_future = _future(done=True, result=MagicMock(success=False))

        self.node.callback_main_loop()

        fault = self.node.foreman_engine.abort_profile.call_args[0][0]
        self.assertEqual(fault.component_names, ["lc1"])

    def test_when_service_call_raises_expect_transport_error_and_profile_aborted(self):
        self.node._active_transition = _command()
        self.node._service_call_active_future = _future(done=True, exception=RuntimeError("boom"))

        self.node.callback_main_loop()

        fault = self.node.foreman_engine.abort_profile.call_args[0][0]
        self.assertEqual(fault.category, ForemanErrorCategory.TRANSPORT)
        self.assertIn("boom", fault.message)
        self.assertIsNone(self.node._active_transition)

    def test_when_no_next_command_expect_nothing_issued(self):
        self.node.callback_main_loop()

        self.node.controller_manager_service_caller.execute_transition.assert_not_called()
        self.node.lifecycle_node_service_caller.execute_transition.assert_not_called()

    def test_when_next_command_targets_controller_expect_controller_manager_called(self):
        cmd = _command("ctrl1", ComponentType.CONTROLLER)
        self.node.foreman_engine.get_next_transition.return_value = cmd
        self.node.controller_manager_service_caller.execute_transition.return_value = _future(
            done=False
        )

        self.node.callback_main_loop()

        self.node.controller_manager_service_caller.execute_transition.assert_called_once_with(cmd)
        self.assertEqual(self.node._active_transition, cmd)

    def test_when_next_command_targets_lifecycle_node_expect_lifecycle_caller_called(self):
        cmd = _command("lc1", ComponentType.LIFECYCLE_NODE)
        self.node.foreman_engine.get_next_transition.return_value = cmd
        self.node.lifecycle_node_service_caller.execute_transition.return_value = _future(
            done=False
        )

        self.node.callback_main_loop()

        self.node.lifecycle_node_service_caller.execute_transition.assert_called_once_with(cmd)

    def test_when_issuing_command_raises_expect_execution_error_and_profile_aborted(self):
        cmd = _command("ctrl1", ComponentType.CONTROLLER)
        self.node.foreman_engine.get_next_transition.return_value = cmd
        self.node.controller_manager_service_caller.execute_transition.side_effect = RuntimeError(
            "not ready"
        )

        self.node.callback_main_loop()

        fault = self.node.foreman_engine.abort_profile.call_args[0][0]
        self.assertEqual(fault.category, ForemanErrorCategory.EXECUTION)
        self.assertIn("not ready", fault.message)
        self.assertIsNone(self.node._active_transition)


class TestForemanNodeDestroy(unittest.TestCase):
    """
    destroy_node() calls super().destroy_node(), which needs a real ForemanNode
    instance (not a bound-method trick on a bare Node) -- built via __new__ to
    skip the heavy real __init__.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_when_node_is_destroyed_expect_adapters_shut_down(self):
        node = ForemanNode.__new__(ForemanNode)
        Node.__init__(node, "test_foreman_node_destroy")
        node.ros_set_profile_action_server = MagicMock()
        node.ros_set_profile_server = MagicMock()

        node.destroy_node()

        node.ros_set_profile_action_server.request_shutdown.assert_called_once()
        node.ros_set_profile_server.request_shutdown.assert_called_once()


class TestForemanNodeMain(unittest.TestCase):
    """main()'s own error handling, fully mocked -- no real rclpy/node needed."""

    def test_when_node_construction_fails_expect_fatal_exit(self):
        with patch.object(node_module, "rclpy") as mock_rclpy, patch.object(
            node_module, "ForemanNode", side_effect=RuntimeError("bad config")
        ):
            with self.assertRaises(SystemExit) as ctx:
                node_module.main()

        self.assertEqual(ctx.exception.code, 1)
        mock_rclpy.shutdown.assert_called_once()

    def test_when_spin_is_interrupted_expect_clean_shutdown(self):
        with patch.object(node_module, "rclpy") as mock_rclpy, patch.object(
            node_module, "ForemanNode"
        ) as MockNode, patch.object(node_module, "MultiThreadedExecutor") as MockExecutor:
            mock_rclpy.ok.return_value = True
            MockExecutor.return_value.spin.side_effect = KeyboardInterrupt()

            node_module.main()

        MockNode.return_value.destroy_node.assert_called_once()
        mock_rclpy.shutdown.assert_called_once()
