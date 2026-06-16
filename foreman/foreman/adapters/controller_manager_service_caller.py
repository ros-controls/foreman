import time
from typing import List

from controller_manager_msgs.srv import CleanupController
from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import SetHardwareComponentState
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.task import Future

from foreman.types import ComponentType
from foreman.types import ForemanError
from foreman.types import ForemanErrorCategory
from foreman.types import LifecycleState
from foreman.types import SystemTransitionCommand


class ControllerManagerServiceCaller:
    """
    Executes a list of SystemTransitionCommands using controller_manager ROS2 services.
    """

    def __init__(self, node: Node, controller_manager_name: str):
        self._node = node
        self._controller_manager_name = controller_manager_name
        self.logger_prefix = "Adapters.ControllerManagerServiceCaller:"

        group = self._node.callback_group_services

        self._client_set_hardware_component_state = self._node.create_client(
            SetHardwareComponentState, f'/{controller_manager_name}/set_hardware_component_state', callback_group=group)
        self._client_configure_controller = self._node.create_client(
            ConfigureController, f'/{controller_manager_name}/configure_controller', callback_group=group)
        self._client_cleanup_controller = self._node.create_client(
            CleanupController, f'/{controller_manager_name}/cleanup_controller', callback_group=group)
        self._client_switch_controller = self._node.create_client(
            SwitchController, f'/{controller_manager_name}/switch_controller', callback_group=group)

        self._node.get_logger().info(
            f"{self.logger_prefix} {self._controller_manager_name} service clients created.")

    def _service_call(self, client, request) -> Future:
        if not client.service_is_ready():
            raise RuntimeError(
                f"Service {client.srv_name} not ready. Is {self._controller_manager_name} running?")
        return client.call_async(request)

    def execute_transition(self, cmd: SystemTransitionCommand) -> Future:
        """Executes a single command and returns the Future."""
        name = cmd.component.name
        goal = cmd.goal_state

        if cmd.component.component_type == ComponentType.HARDWARE:
            self._node.get_logger().info(f"{self.logger_prefix} HW State -> {name} to {goal.name}")
            req = SetHardwareComponentState.Request()
            req.name = name
            req.target_state.id = goal.value
            return self._service_call(self._client_set_hardware_component_state, req)

        elif cmd.component.component_type == ComponentType.CONTROLLER:
            current = cmd.component.lifecycle_state

            if goal == LifecycleState.ACTIVE:
                self._node.get_logger().info(f"{self.logger_prefix} Switch -> Activate {name}")
                req = SwitchController.Request(
                    activate_controllers=[name], strictness=SwitchController.Request.STRICT)
                return self._service_call(self._client_switch_controller, req)

            elif goal == LifecycleState.INACTIVE and current == LifecycleState.ACTIVE:
                self._node.get_logger().info(f"{self.logger_prefix} Switch -> Deactivate {name}")
                req = SwitchController.Request(deactivate_controllers=[
                                               name], strictness=SwitchController.Request.STRICT)
                return self._service_call(self._client_switch_controller, req)

            elif goal == LifecycleState.INACTIVE and current == LifecycleState.UNCONFIGURED:
                self._node.get_logger().info(f"{self.logger_prefix} Configure -> {name}")
                return self._service_call(self._client_configure_controller, ConfigureController.Request(name=name))

            elif goal == LifecycleState.UNCONFIGURED:
                self._node.get_logger().info(f"{self.logger_prefix} Cleanup -> {name}")
                return self._service_call(self._client_cleanup_controller, CleanupController.Request(name=name))

        raise ValueError(f"Unable to process transition command: {cmd}")
