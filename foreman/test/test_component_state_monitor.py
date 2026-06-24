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


class TestDependencyInferenceWiring(unittest.TestCase):
    """The service round-trip: read the CM on each activity update and set the rules on the engine."""

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

    def _infer_with(self, controllers, hardware):
        # Mock the controller manager services to return the given controllers and hardware, then refresh rules.
        controllers_future = Future()
        controllers_future.set_result(ListControllers.Response(controller=controllers))
        hardware_future = Future()
        hardware_future.set_result(ListHardwareComponents.Response(component=hardware))

        with mock.patch.object(self.monitor._client_list_controllers,
                               "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "service_is_ready", return_value=True), \
            mock.patch.object(self.monitor._client_list_controllers,
                              "call_async", return_value=controllers_future), \
            mock.patch.object(self.monitor._client_list_hardware_components,
                              "call_async", return_value=hardware_future):
            self.monitor._refresh_dependency_rules()

    def test_refresh_skips_when_services_not_ready(self):
        # No controller_manager is up, so its clients are not ready, we do nothing.
        with mock.patch.object(self.monitor._client_list_controllers, "call_async") as list_call:
            self.monitor._refresh_dependency_rules()
        list_call.assert_not_called()
        self.engine.set_dependency_rules.assert_not_called()

    def test_refresh_reads_cm_and_sets_rules_on_engine(self):
        self._infer_with(
            controllers=[_controller_state("forward_position_controller",
                                           required_command=["joint1/position"])],
            hardware=[_hardware_component("RRBot", command=["joint1/position"])])

        self.engine.set_dependency_rules.assert_called_once()
        rule = self.engine.set_dependency_rules.call_args[0][0][0]
        self.assertEqual(rule.controller_name, "forward_position_controller")
        self.assertEqual(rule.required_hardware[0].name, "RRBot")
        self.assertEqual(rule.required_hardware[0].state, LifecycleState.ACTIVE)

    def test_refresh_runs_again_on_next_activity(self):
        # There is no run-once guard, each activity update re-infers and sets the dependency rules again.
        controllers = [_controller_state("forward_position_controller",
                                         required_command=["joint1/position"])]
        hardware = [_hardware_component("RRBot", command=["joint1/position"])]
        self._infer_with(controllers=controllers, hardware=hardware)
        self._infer_with(controllers=controllers, hardware=hardware)
        self.assertEqual(self.engine.set_dependency_rules.call_count, 2)

    def test_no_controllers_yields_empty_rules(self):
        # No controllers yet, empty rule set.
        self._infer_with(controllers=[], hardware=[])
        self.engine.set_dependency_rules.assert_called_once_with([])


if __name__ == '__main__':
    unittest.main()
