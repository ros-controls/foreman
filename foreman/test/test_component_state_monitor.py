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

from foreman.adapters.component_state_monitor import ComponentStateMonitor
from foreman.types import LifecycleState


def _controller_state(name, required_command=None, required_state=None):
    """Build a ControllerState message like list_controllers returns."""
    return ControllerState(
        name=name,
        required_command_interfaces=required_command or [],
        required_state_interfaces=required_state or [],
    )


def _hardware_component(name, command=None, state=None):
    """Build a HardwareComponentState message like list_hardware_components returns."""
    return HardwareComponentState(
        name=name,
        command_interfaces=[HardwareInterface(name=n) for n in (command or [])],
        state_interfaces=[HardwareInterface(name=n) for n in (state or [])],
    )


class TestInferDependencyRules(unittest.TestCase):
    """Test the inference of dependency rules based on controller and hardware states."""

    def test_command_interface_requires_hardware_active(self):
        # A controller that writes a command interface needs its hardware ACTIVE.
        controllers = [_controller_state(
            "forward_position_controller", required_command=["joint1/position"])]
        hardware = [_hardware_component("RRBot", command=["joint1/position"])]
        rule = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)[0]
        self.assertEqual(rule.required_hardware[0].name, "RRBot")
        self.assertEqual(rule.required_hardware[0].state, LifecycleState.ACTIVE)

    def test_state_interface_requires_hardware_inactive(self):
        # A broadcaster only reads a state interface, so the hardware need only be INACTIVE.
        controllers = [_controller_state("joint_state_broadcaster",
                                         required_state=["joint1/position"])]
        hardware = [_hardware_component("RRBot", state=["joint1/position"])]
        rule = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)[0]
        self.assertEqual(rule.required_hardware[0].name, "RRBot")
        self.assertEqual(rule.required_hardware[0].state, LifecycleState.INACTIVE)

    def test_needing_command_and_state_requires_active(self):
        # If a controller needs both kinds from one hardware, the strict ACTIVE requirement wins.
        controllers = [_controller_state("mixed",
                                         required_command=["joint1/position"],
                                         required_state=["joint1/velocity"])]
        hardware = [_hardware_component("RRBot",
                                        command=["joint1/position"],
                                        state=["joint1/velocity"])]
        rule = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)[0]
        self.assertEqual(rule.required_hardware[0].state, LifecycleState.ACTIVE)

    def test_owner_resolved_by_interface_name_not_prefix(self):
        # rrbot_joint1/position is owned by whichever hardware exports it, not by rrbot_joint1.
        controllers = [_controller_state("pos_ctrl", required_command=["rrbot_joint1/position"])]
        hardware = [_hardware_component("RRBotSystemPositionOnly",
                                        command=["rrbot_joint1/position"])]
        rule = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)[0]
        self.assertEqual([h.name for h in rule.required_hardware], ["RRBotSystemPositionOnly"])

    def test_interface_with_no_owning_hardware_gives_no_dependency(self):
        # An interface no hardware exports (e.g. hardware not up yet) yields no dependency.
        controllers = [_controller_state("lonely", required_command=["ghost/iface"])]
        hardware = [_hardware_component("RRBot", command=["joint1/position"])]
        rule = ComponentStateMonitor.infer_dependency_rules(controllers, hardware)[0]
        self.assertEqual(rule.required_hardware, [])


class TestDependencyRulesQuery(unittest.TestCase):
    """Tests querying dependency rules from the controller_manager."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_component_state_monitor")
        # ComponentStateMonitor expects these callback groups on its node; a real ForemanNode
        # creates them, so we add them to this plain test node.
        self.node.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.node.callback_group_subscriber = ReentrantCallbackGroup()
        self.addCleanup(self.node.destroy_node)
        self.engine = Mock()
        self.monitor = ComponentStateMonitor(self.node, self.engine, "test_controller_manager", [])

    def test_returns_empty_when_services_not_ready(self):
        # When controller_manager services are unavailable, do not attempt any queries.
        with mock.patch.object(self.monitor._client_list_controllers, "call") as controllers_call:
            self.assertEqual(self.monitor.get_dependency_rules(), [])
        controllers_call.assert_not_called()

    def test_queries_cm_and_builds_rules(self):
        controllers_response = ListControllers.Response(controller=[
            _controller_state("forward_position_controller", required_command=["joint1/position"])])
        hardware_response = ListHardwareComponents.Response(component=[
            _hardware_component("RRBot", command=["joint1/position"])])

        with mock.patch.object(self.monitor._client_list_controllers,
                               "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_controllers,
                              "call", return_value=controllers_response), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "call", return_value=hardware_response):
            rules = self.monitor.get_dependency_rules()

        self.assertEqual(len(rules), 1)
        self.assertEqual(rules[0].controller_name, "forward_position_controller")
        self.assertEqual(rules[0].required_hardware[0].name, "RRBot")
        self.assertEqual(rules[0].required_hardware[0].state, LifecycleState.ACTIVE)


if __name__ == '__main__':
    unittest.main()
