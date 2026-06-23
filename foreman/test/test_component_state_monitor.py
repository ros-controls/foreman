import unittest
from unittest import mock
from unittest.mock import Mock

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.msg import HardwareComponentState
from controller_manager_msgs.msg import HardwareInterface
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from foreman.adapters.component_state_monitor import ComponentStateMonitor
from foreman.types import LifecycleState


def _controller_state(name, required_command=None, required_state=None):
    """Build a real ControllerState message, like list_controllers returns."""
    return ControllerState(
        name=name,
        required_command_interfaces=required_command or [],
        required_state_interfaces=required_state or [],
    )


def _hardware_component(name, command=None, state=None):
    """Build a real HardwareComponentState message, like list_hardware_components returns."""
    return HardwareComponentState(
        name=name,
        command_interfaces=[HardwareInterface(name=n) for n in (command or [])],
        state_interfaces=[HardwareInterface(name=n) for n in (state or [])],
    )


class TestInferDependencyRules(unittest.TestCase):
    """Pure inference logic: controller required-interfaces to the owning hardware."""

    def test_maps_controller_to_owning_hardware(self):
        controllers = [_controller_state("ctrl_a", required_command=["joint1/position"])]
        hardware = [_hardware_component("RRBot", command=["joint1/position"])]
        rules = {r.controller_name: r for r in
                 ComponentStateMonitor.infer_dependency_rules(controllers, hardware)}
        deps = [(h.name, h.state) for h in rules["ctrl_a"].required_hardware]
        self.assertEqual(deps, [("RRBot", LifecycleState.ACTIVE)])

    def test_resolves_owner_by_interface_not_by_prefix(self):
        # here e.g. 'rrbot_joint1/position' is owned by RRBotSystemPositionOnly, not by 'rrbot_joint1'.
        controllers = [_controller_state("pos_ctrl", required_command=["rrbot_joint1/position"])]
        hardware = [_hardware_component("RRBotSystemPositionOnly",
                                        command=["rrbot_joint1/position"])]
        rules = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)
        self.assertEqual([h.name for h in rules[0].required_hardware], ["RRBotSystemPositionOnly"])

    def test_controller_with_no_owning_hardware_has_no_dependency(self):
        controllers = [_controller_state("lonely", required_command=["ghost/iface"])]
        hardware = [_hardware_component("RRBot", command=["joint1/position"])]
        rules = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)
        self.assertEqual(rules[0].required_hardware, [])


class TestDependencyInferenceWiring(unittest.TestCase):
    """CM service round-trip wiring, with a real node and a mocked engine."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_component_state_monitor")
        # the adapter reads these; the real ForemanNode sets them, a bare node may not
        self.node.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.node.callback_group_subscriber = ReentrantCallbackGroup()
        self.addCleanup(self.node.destroy_node)
        self.engine = Mock()
        self.monitor = ComponentStateMonitor(self.node, self.engine, "test_controller_manager", [])

    def test_infer_skips_when_services_not_ready(self):
        # No CM running, clients not ready then we must not call the services
        with mock.patch.object(self.monitor._client_list_controllers, "call_async") as ctrl_call, \
                mock.patch.object(self.monitor._client_list_hardware_components, "call_async") as hw_call:
            self.monitor.infer_dependencies()
        ctrl_call.assert_not_called()
        hw_call.assert_not_called()
        self.assertFalse(self.monitor._dependencies_inferred)

    def test_infer_runs_only_once(self):
        self.monitor._dependencies_inferred = True
        with mock.patch.object(self.monitor._client_list_controllers, "call_async") as ctrl_call:
            self.monitor.infer_dependencies()
        ctrl_call.assert_not_called()

    def test_rules_not_pushed_until_both_responses_arrive(self):
        self.monitor._controller_states = []      # controllers arrived
        self.monitor._hardware_components = None   # hardware not yet received
        self.monitor._build_and_push_dependency_rules()
        self.engine.update_dependency_rules.assert_not_called()

    def test_inferred_rules_are_pushed_to_engine(self):
        self.monitor._controller_states = [
            _controller_state("ctrl_a", required_command=["joint1/position"])]
        self.monitor._hardware_components = [
            _hardware_component("RRBot", command=["joint1/position"])]
        self.monitor._build_and_push_dependency_rules()
        self.engine.update_dependency_rules.assert_called_once()
        rules = self.engine.update_dependency_rules.call_args[0][0]
        self.assertEqual(rules[0].controller_name, "ctrl_a")
        self.assertEqual(rules[0].required_hardware[0].name, "RRBot")
        self.assertEqual(rules[0].required_hardware[0].state, LifecycleState.ACTIVE)

    def test_full_flow_queries_cm_and_pushes_rules(self):
        controllers_future = Future()
        controllers_resp = ListControllers.Response()
        controllers_resp.controller = [
            _controller_state("ctrl_a", required_command=["joint1/position"])]
        controllers_future.set_result(controllers_resp)

        hardware_future = Future()
        hardware_resp = ListHardwareComponents.Response()
        hardware_resp.component = [_hardware_component("RRBot", command=["joint1/position"])]
        hardware_future.set_result(hardware_resp)

        with mock.patch.object(self.monitor._client_list_controllers,
                               "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_controllers,
                              "call_async", return_value=controllers_future), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "call_async", return_value=hardware_future):
            self.monitor.infer_dependencies()

        self.assertTrue(self.monitor._dependencies_inferred)
        self.engine.update_dependency_rules.assert_called_once()
        rules = self.engine.update_dependency_rules.call_args[0][0]
        self.assertEqual(rules[0].controller_name, "ctrl_a")
        self.assertEqual(rules[0].required_hardware[0].name, "RRBot")


if __name__ == '__main__':
    unittest.main()
