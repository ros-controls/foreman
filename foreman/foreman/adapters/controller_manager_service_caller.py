from controller_manager_msgs.srv import (
    CleanupController,
    ConfigureController,
    SetHardwareComponentState,
    SwitchController,
)
from rclpy.node import Node
from rclpy.task import Future

from foreman.types import ComponentType, LifecycleState, SystemTransitionCommand


class ControllerManagerServiceCaller:
    """Executes a list of SystemTransitionCommands using controller_manager ROS2 services."""

    def __init__(self, node: Node, controller_manager_name: str):
        self._node = node
        self._controller_manager_name = controller_manager_name
        self.logger_prefix = "Adapters.ControllerManagerServiceCaller:"

        group = self._node.callback_group_services

        self._client_set_hardware_component_state = self._node.create_client(
            SetHardwareComponentState,
            f"/{controller_manager_name}/set_hardware_component_state",
            callback_group=group,
        )
        self._client_configure_controller = self._node.create_client(
            ConfigureController,
            f"/{controller_manager_name}/configure_controller",
            callback_group=group,
        )
        self._client_cleanup_controller = self._node.create_client(
            CleanupController,
            f"/{controller_manager_name}/cleanup_controller",
            callback_group=group,
        )
        self._client_switch_controller = self._node.create_client(
            SwitchController, f"/{controller_manager_name}/switch_controller", callback_group=group
        )

        self._node.get_logger().info(
            f"{self.logger_prefix} {self._controller_manager_name} service clients created."
        )

    def _service_call(self, client, request) -> Future:
        if not client.service_is_ready():
            raise RuntimeError(
                f"Service {client.srv_name} not ready. Is {self._controller_manager_name} running?"
            )
        return client.call_async(request)

    def execute_transition(self, cmd: SystemTransitionCommand) -> Future:
        """Execute a command and return the Future.

        Controller activations and deactivations batched together by the planner
        go out as ONE switch_controller request, so the controller manager hands
        the hardware a single stop/start interface set. Sent as two requests, the
        deactivate half leaves an active component with no command mode claimed,
        which hardware that validates modes in prepare_command_mode_switch()
        rejects. Configure, cleanup and hardware commands are never batched and
        keep their own service call.
        """
        name = cmd.component.name
        goal = cmd.goal_state

        if cmd.component.component_type == ComponentType.HARDWARE:
            self._node.get_logger().info(f"{self.logger_prefix} HW State -> {name} to {goal.name}")
            req = SetHardwareComponentState.Request()
            req.name = name
            req.target_state.id = goal.value
            return self._service_call(self._client_set_hardware_component_state, req)

        elif cmd.component.component_type == ComponentType.CONTROLLER:
            activate = []
            deactivate = []

            for step in cmd.steps:
                step_name = step.component.name
                step_goal = step.goal_state
                step_current = step.component.lifecycle_state

                if step_goal == LifecycleState.ACTIVE:
                    activate.append(step_name)

                elif step_goal == LifecycleState.INACTIVE and step_current == LifecycleState.ACTIVE:
                    deactivate.append(step_name)

                elif (
                    step_goal == LifecycleState.INACTIVE
                    and step_current == LifecycleState.UNCONFIGURED
                ):
                    # Never batched, so this is the only step.
                    self._node.get_logger().info(
                        f"{self.logger_prefix} Configure -> {step_name}"
                    )
                    return self._service_call(
                        self._client_configure_controller,
                        ConfigureController.Request(name=step_name),
                    )

                elif step_goal == LifecycleState.UNCONFIGURED:
                    # Never batched, so this is the only step.
                    self._node.get_logger().info(f"{self.logger_prefix} Cleanup -> {step_name}")
                    return self._service_call(
                        self._client_cleanup_controller,
                        CleanupController.Request(name=step_name),
                    )

            if activate or deactivate:
                self._node.get_logger().info(
                    f"{self.logger_prefix} Switch -> activate {activate}, deactivate {deactivate}"
                )
                req = SwitchController.Request(
                    activate_controllers=activate,
                    deactivate_controllers=deactivate,
                    strictness=SwitchController.Request.STRICT,
                )
                return self._service_call(self._client_switch_controller, req)

        raise ValueError(f"Unable to process transition command: {cmd}")
