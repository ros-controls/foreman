import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from foreman.adapters.controller_manager_service_caller import ControllerManagerServiceCaller
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import LifecycleState
from foreman.types import SystemTransitionCommand


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
            LifecycleState.ACTIVE)
        caller.execute_transition(cmd)
        sent = caller._client_switch_controller.call_async.call_args[0][0]
        self.assertEqual(list(sent.activate_controllers), ["ctrl_a"])
        self.assertEqual(list(sent.deactivate_controllers), [])

    def test_deactivate_sends_deactivate_controllers(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_switch_controller = MagicMock()
        cmd = SystemTransitionCommand(
            Component("ctrl_a", ComponentType.CONTROLLER, LifecycleState.ACTIVE),
            LifecycleState.INACTIVE)
        caller.execute_transition(cmd)
        sent = caller._client_switch_controller.call_async.call_args[0][0]
        self.assertEqual(list(sent.deactivate_controllers), ["ctrl_a"])

    def test_hardware_sends_target_state_id(self):
        caller = ControllerManagerServiceCaller(self.node, "controller_manager")
        caller._client_set_hardware_component_state = MagicMock()
        cmd = SystemTransitionCommand(
            Component("RRBot", ComponentType.HARDWARE, LifecycleState.INACTIVE),
            LifecycleState.ACTIVE)
        caller.execute_transition(cmd)
        sent = caller._client_set_hardware_component_state.call_async.call_args[0][0]
        self.assertEqual(sent.name, "RRBot")
        self.assertEqual(sent.target_state.id, LifecycleState.ACTIVE.value)


if __name__ == '__main__':
    unittest.main()
