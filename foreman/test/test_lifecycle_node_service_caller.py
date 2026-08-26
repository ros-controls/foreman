import unittest
from unittest.mock import MagicMock

import rclpy
from lifecycle_msgs.msg import Transition
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from foreman.adapters.lifecycle_node_service_caller import LifecycleNodeServiceCaller
from foreman.types import Component, ComponentType, LifecycleState, SystemTransitionCommand


def _command(current, goal, name="lc1"):
    return SystemTransitionCommand(Component(name, ComponentType.LIFECYCLE_NODE, current), goal)


class TestLifecycleNodeServiceCaller(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_lc_caller")
        # the adapter reads this; the real node sets it, a bare node doesn't
        self.node.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.addCleanup(self.node.destroy_node)

    def test_service_names_built_from_lifecycle_node_names(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        resolved = self.node.resolve_service_name(caller._clients["lc1"].srv_name)
        self.assertEqual(resolved, "/lc1/change_state")

    def test_when_no_client_for_node_expect_value_error(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        cmd = _command(LifecycleState.INACTIVE, LifecycleState.ACTIVE, name="unknown")

        with self.assertRaises(ValueError):
            caller.execute_transition(cmd)

    def test_when_service_not_ready_expect_runtime_error(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        caller._clients["lc1"] = MagicMock(service_is_ready=MagicMock(return_value=False))
        cmd = _command(LifecycleState.INACTIVE, LifecycleState.ACTIVE)

        with self.assertRaises(RuntimeError):
            caller.execute_transition(cmd)

    def test_when_no_valid_transition_expect_value_error(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        caller._clients["lc1"] = MagicMock(service_is_ready=MagicMock(return_value=True))
        cmd = _command(LifecycleState.UNCONFIGURED, LifecycleState.ACTIVE)

        with self.assertRaises(ValueError):
            caller.execute_transition(cmd)

    def test_when_configuring_expect_configure_transition_sent(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        client = MagicMock(service_is_ready=MagicMock(return_value=True))
        caller._clients["lc1"] = client
        cmd = _command(LifecycleState.UNCONFIGURED, LifecycleState.INACTIVE)

        caller.execute_transition(cmd)

        sent = client.call_async.call_args[0][0]
        self.assertEqual(sent.transition.id, Transition.TRANSITION_CONFIGURE)

    def test_when_activating_expect_activate_transition_sent(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        client = MagicMock(service_is_ready=MagicMock(return_value=True))
        caller._clients["lc1"] = client
        cmd = _command(LifecycleState.INACTIVE, LifecycleState.ACTIVE)

        caller.execute_transition(cmd)

        sent = client.call_async.call_args[0][0]
        self.assertEqual(sent.transition.id, Transition.TRANSITION_ACTIVATE)

    def test_when_deactivating_expect_deactivate_transition_sent(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        client = MagicMock(service_is_ready=MagicMock(return_value=True))
        caller._clients["lc1"] = client
        cmd = _command(LifecycleState.ACTIVE, LifecycleState.INACTIVE)

        caller.execute_transition(cmd)

        sent = client.call_async.call_args[0][0]
        self.assertEqual(sent.transition.id, Transition.TRANSITION_DEACTIVATE)

    def test_when_cleaning_up_expect_cleanup_transition_sent(self):
        caller = LifecycleNodeServiceCaller(self.node, ["lc1"])
        client = MagicMock(service_is_ready=MagicMock(return_value=True))
        caller._clients["lc1"] = client
        cmd = _command(LifecycleState.INACTIVE, LifecycleState.UNCONFIGURED)

        caller.execute_transition(cmd)

        sent = client.call_async.call_args[0][0]
        self.assertEqual(sent.transition.id, Transition.TRANSITION_CLEANUP)


if __name__ == "__main__":
    unittest.main()
