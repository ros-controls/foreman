import time
from typing import List
from rclpy.node import Node
from rclpy.task import Future

from foreman.types import ComponentType, LifecycleState, SystemTransitionCommand, ForemanError, ForemanErrorCategory
from controller_manager_msgs.srv import (
    CleanupController,
    ConfigureController,
    SetHardwareComponentState,
    SwitchController,
)


class ServiceCaller:
    """
    Executes a list of SystemTransitionCommands using controller_manager ROS2 services.
    """

    def __init__(self, node: Node, controller_manager_name: str):
        self._node = node
        # TODO: parametrize this?
        self._timeout = 10.0
        self._controller_manager_name = controller_manager_name

        group = self._node.callback_group_services

        self._client_set_hardware_component_state = self._node.create_client(
            SetHardwareComponentState, f'/{controller_manager_name}/set_hardware_component_state', callback_group=group)
        self._client_configure_controller = self._node.create_client(
            ConfigureController, f'/{controller_manager_name}/configure_controller', callback_group=group)
        self._client_cleanup_controller = self._node.create_client(
            CleanupController, f'/{controller_manager_name}/cleanup_controller', callback_group=group)
        self._client_switch_controller = self._node.create_client(
            SwitchController, f'/{controller_manager_name}/switch_controller', callback_group=group)

        self._node.get_logger().info(f"Adapters.ControllerManager.ServiceCaller: {self._controller_manager_name} service clients created.")

    def _service_call(self, client, request) -> Future:
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise RuntimeError(f"Service {client.srv_name} timed out. Is {self._controller_manager_name} running?")
        return client.call_async(request)

    def execute_transition(self, cmd: SystemTransitionCommand) -> Future:
        """Executes a single command and returns the Future."""
        name = cmd.component.name
        goal = cmd.goal_state

        if cmd.component.component_type == ComponentType.HARDWARE:
            self._node.get_logger().info(f"ServiceCaller:  HW State -> {name} to {goal.name}")
            req = SetHardwareComponentState.Request()
            req.name = name
            req.target_state.id = goal.value
            return self._service_call(self._client_set_hardware_component_state, req)

        elif cmd.component.component_type == ComponentType.CONTROLLER:
            current = cmd.component.lifecycle_state
            
            if goal == LifecycleState.ACTIVE:
                self._node.get_logger().info(f"ServiceCaller:  Switch -> Activate {name}")
                req = SwitchController.Request(activate_controllers=[name], strictness=SwitchController.Request.STRICT)
                return self._service_call(self._client_switch_controller, req)
                
            elif goal == LifecycleState.INACTIVE and current == LifecycleState.ACTIVE:
                self._node.get_logger().info(f"ServiceCaller:  Switch -> Deactivate {name}")
                req = SwitchController.Request(deactivate_controllers=[name], strictness=SwitchController.Request.STRICT)
                return self._service_call(self._client_switch_controller, req)
                
            elif goal == LifecycleState.INACTIVE and current == LifecycleState.UNCONFIGURED:
                self._node.get_logger().info(f"ServiceCaller:  Configure -> {name}")
                return self._service_call(self._client_configure_controller, ConfigureController.Request(name=name))
                
            elif goal == LifecycleState.UNCONFIGURED:
                self._node.get_logger().info(f"ServiceCaller:  Cleanup -> {name}")
                return self._service_call(self._client_cleanup_controller, CleanupController.Request(name=name))
                
        raise ValueError(f"Unable to process transition command: {cmd}")