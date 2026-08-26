import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from foreman.adapters.controller_manager_service_caller import ControllerManagerServiceCaller
from foreman.types import Component, ComponentType, LifecycleState, SystemTransitionCommand


class TestControllerManagerServiceCaller(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_cm_caller")
        # the adapter reads this; the real node sets it, a bare node doesn't
        self.node.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.addCleanup(self.node.destroy_node)

    def test_service_names_built_from_controller_manager_name(self):
        caller = ControllerManagerServiceCaller(self.node, "rrbot/controller_manager")
        resolved = self.node.resolve_service_name(caller._client_switch_controller.srv_name)
        self.assertEqual(resolved, "/rrbot/controller_manager/switch_controller")

    def test_activate_sends_activate_controllers(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_switch_controller = MagicMock()
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            LifecycleState.ACTIVE,
        )
        caller.execute_transition(cmd)
        sent = caller._client_switch_controller.call_async.call_args[0][0]
        self.assertEqual(list(sent.activate_controllers), ["ctrl_a"])
        self.assertEqual(list(sent.deactivate_controllers), [])

    def test_deactivate_sends_deactivate_controllers(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_switch_controller = MagicMock()
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.ACTIVE),
            LifecycleState.INACTIVE,
        )
        caller.execute_transition(cmd)
        sent = caller._client_switch_controller.call_async.call_args[0][0]
        self.assertEqual(list(sent.deactivate_controllers), ["ctrl_a"])

    def test_hardware_sends_target_state_id(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_set_hardware_component_state = MagicMock()
        cmd = SystemTransitionCommand(
            Component("RRBot", ComponentType.HARDWARE, LifecycleState.INACTIVE),
            LifecycleState.ACTIVE,
        )
        caller.execute_transition(cmd)
        sent = caller._client_set_hardware_component_state.call_async.call_args[0][0]
        self.assertEqual(sent.name, "RRBot")
        self.assertEqual(sent.target_state.id, LifecycleState.ACTIVE.value)

    def test_configure_sends_configure_request(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_configure_controller = MagicMock()
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.UNCONFIGURED),
            LifecycleState.INACTIVE,
        )
        caller.execute_transition(cmd)
        sent = caller._client_configure_controller.call_async.call_args[0][0]
        self.assertEqual(sent.name, "ctrl_a")

    def test_cleanup_sends_cleanup_request(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_cleanup_controller = MagicMock()
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.INACTIVE),
            LifecycleState.UNCONFIGURED,
        )
        caller.execute_transition(cmd)
        sent = caller._client_cleanup_controller.call_async.call_args[0][0]
        self.assertEqual(sent.name, "ctrl_a")

    def test_when_transition_not_processable_expect_value_error(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.FINALIZED),
            LifecycleState.INACTIVE,
        )
        with self.assertRaises(ValueError):
            caller.execute_transition(cmd)

    def test_when_service_not_ready_expect_runtime_error(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_set_hardware_component_state = MagicMock(
            service_is_ready=MagicMock(return_value=False)
        )
        cmd = SystemTransitionCommand(
            Component("RRBot", ComponentType.HARDWARE, LifecycleState.INACTIVE),
            LifecycleState.ACTIVE,
        )
        with self.assertRaises(RuntimeError):
            caller.execute_transition(cmd)


if __name__ == "__main__":
    unittest.main()
