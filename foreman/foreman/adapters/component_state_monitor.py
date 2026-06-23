from typing import Dict, List

from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from rclpy.event_handler import QoSSubscriptionMatchedInfo
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from foreman.engine import ForemanEngine
from foreman.types import Component
from foreman.types import ComponentType
from foreman.types import ControllerDependencyRule
from foreman.types import HardwareRequirement
from foreman.types import LifecycleState


class ComponentStateMonitor:
    """
    Observes full system state from all sources, merges the component state and pushes it to the Engine.

    Sources:
      - /<controller_manager>/activity (TRANSIENT_LOCAL): HW + Controllers
      - /<node>/transition_event (VOLATILE, with matched event): Lifecycle Nodes

    Lifecycle node discovery and death detection uses the QpS "matched event" on the
    transition_event topic. When the publisher appears (node started), we call
    get_state/ service once for initial state. When it disappears (node died), we mark FINALIZED.
    https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#matched-events
    """

    def __init__(
        self,
        node: Node,
        engine: ForemanEngine,
        controller_manager_name: str,
        lifecycle_nodes: List[str],
    ):
        self._node = node
        self._engine = engine
        self._logger_prefix = "Adapters.ComponentStateMonitor:"

        self._cm_components: Dict[str, Component] = {}
        self._lc_components: Dict[str, Component] = {}
        self._lc_nodes_alive: Dict[str, bool] = {n: False for n in lifecycle_nodes}

        # --- Controller Manager /activity topic ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._subscription = self._node.create_subscription(
            ControllerManagerActivity,
            f'/{controller_manager_name}/activity',
            self._activity_callback,
            qos_profile,
            callback_group=self._node.callback_group_subscriber
        )
        self._node.get_logger().info(
            f"{self._logger_prefix} Subscribed to /{controller_manager_name}/activity"
        )

        # Service clients used to read the CM's controllers and hardware once, to infer dependencies.
        self._client_list_controllers = self._node.create_client(
            ListControllers,
            f'/{controller_manager_name}/list_controllers',
            callback_group=self._node.callback_group_services
        )
        self._client_list_hardware_components = self._node.create_client(
            ListHardwareComponents,
            f'/{controller_manager_name}/list_hardware_components',
            callback_group=self._node.callback_group_services
        )
        # The two responses arrive separately, here None means "not received yet".
        self._controller_states = None
        self._hardware_components = None
        # Inference runs only once, the first time the CM becomes visible.
        self._dependencies_inferred = False

        # --- Lifecycle node monitoring ---
        self._lc_node_get_state_clients: Dict[str, object] = {}

        for lc_node_name in lifecycle_nodes:
            self._lc_node_get_state_clients[lc_node_name] = self._node.create_client(
                GetState,
                f'/{lc_node_name}/get_state',
                callback_group=self._node.callback_group_services
            )

            # this matched event on transition_event/ topic handles both
            # discovery (publisher appeared) and death detection (publisher disappeared).
            # https://docs.ros.org/en/jazzy/p/rclpy/rclpy.event_handler.html#rclpy.event_handler.SubscriptionEventCallbacks
            # also, we're in a loop, so we need to pass lc_node_name explicitly to the lambda as well.
            event_callbacks = SubscriptionEventCallbacks(
                matched=lambda info, n=lc_node_name: self._on_lifecycle_publisher_matched(n, info)
            )
            self._node.create_subscription(
                TransitionEvent,
                f'/{lc_node_name}/transition_event',
                callback=lambda msg, n=lc_node_name: self._lifecycle_transition_event_callback(
                    n, msg),
                qos_profile=10,
                event_callbacks=event_callbacks,
                callback_group=self._node.callback_group_subscriber
            )

        if lifecycle_nodes:
            self._node.get_logger().info(
                f"{self._logger_prefix} Monitoring lifecycle nodes: {lifecycle_nodes}"
            )

    @staticmethod
    def infer_dependency_rules(controller_states, hardware_components):
        """Map each controller to the hardware that owns the interfaces it requires."""
        interface_owner = {}
        for hardware in hardware_components:
            for interface in list(hardware.command_interfaces) + list(hardware.state_interfaces):
                interface_owner[interface.name] = hardware.name

        dependency_rules = []
        for controller in controller_states:
            required_interfaces = (
                set(controller.required_command_interfaces)
                | set(controller.required_state_interfaces)
            )
            # Resolve each interface to the hardware that owns it.
            required_hardware_names = {
                interface_owner[name] for name in required_interfaces if name in interface_owner
            }
            dependency_rules.append(ControllerDependencyRule(
                controller_name=controller.name,
                required_hardware=[
                    HardwareRequirement(name=hardware_name, state=LifecycleState.ACTIVE)
                    for hardware_name in required_hardware_names
                ],
            ))
        return dependency_rules

    def infer_dependencies(self):
        """Ask the controller_manager (once) for its controllers and hardware to infer dependencies."""
        if self._dependencies_inferred:
            return
        if not (self._client_list_controllers.service_is_ready()
                and self._client_list_hardware_components.service_is_ready()):
            return
        self._dependencies_inferred = True

        controllers_future = self._client_list_controllers.call_async(ListControllers.Request())
        controllers_future.add_done_callback(self._on_list_controllers_response)

        hardware_future = self._client_list_hardware_components.call_async(
            ListHardwareComponents.Request())
        hardware_future.add_done_callback(self._on_list_hardware_components_response)

    def _on_list_controllers_response(self, future):
        # Cache the controllers; rules are built once the hardware response also arrives.
        try:
            self._controller_states = future.result().controller
            self._build_and_push_dependency_rules()
        except Exception as e:
            self._node.get_logger().warning(
                f"{self._logger_prefix} list_controllers failed: {e}")

    def _on_list_hardware_components_response(self, future):
        # Cache the hardware; rules are built once the controllers response also arrives.
        try:
            self._hardware_components = future.result().component
            self._build_and_push_dependency_rules()
        except Exception as e:
            self._node.get_logger().warning(
                f"{self._logger_prefix} list_hardware_components failed: {e}")

    def _build_and_push_dependency_rules(self):
        """Once both responses are in, infer the rules and hand them to the engine."""
        if self._controller_states is None or self._hardware_components is None:
            return
        if not self._controller_states:
            self._node.get_logger().warning(
                f"{self._logger_prefix} No controllers reported by the controller_manager.")
        rules = self.infer_dependency_rules(self._controller_states, self._hardware_components)
        self._engine.update_dependency_rules(rules)

    # TODO: use matched event for this topic as well? That way we know if controller manager dies.
    # Minor. Currently we catch unexpected transitions in the engine (all components go to finalized)
    def _activity_callback(self, msg: ControllerManagerActivity):
        """Parse /activity message into components and push merged state."""
        components = {}

        for hw_msg in msg.hardware_components:
            try:
                components[hw_msg.name] = Component(
                    name=hw_msg.name,
                    component_type=ComponentType.HARDWARE,
                    lifecycle_state=LifecycleState(hw_msg.state.id)
                )
            except ValueError:
                continue

        for ctrl_msg in msg.controllers:
            try:
                components[ctrl_msg.name] = Component(
                    name=ctrl_msg.name,
                    component_type=ComponentType.CONTROLLER,
                    lifecycle_state=LifecycleState(ctrl_msg.state.id)
                )
            except ValueError:
                continue

        self._cm_components = components
        # The CM is visible now, so infer dependencies (here it is guarded to run once).
        self.infer_dependencies()
        self._push_merged_state()

    def _on_lifecycle_publisher_matched(self, name: str, info: QoSSubscriptionMatchedInfo):
        """DDS matched event: lifecycle node's transition_event publisher appeared or disappeared."""
        if info.current_count > 0 and not self._lc_nodes_alive[name]:
            # node appeared, get initial state
            self._lc_nodes_alive[name] = True
            self._lifecycle_call_get_state(name)

        elif info.current_count == 0 and self._lc_nodes_alive[name]:
            # node disappeared, crashed
            self._lc_nodes_alive[name] = False
            self._lc_components[name] = Component(
                name=name,
                component_type=ComponentType.LIFECYCLE_NODE,
                lifecycle_state=LifecycleState.FINALIZED
            )
            self._node.get_logger().warning(
                f"{self._logger_prefix} Lifecycle node '{name}' disconnected."
            )
            self._push_merged_state()

    def _lifecycle_call_get_state(self, name: str):
        """One-shot async call to /<node>/get_state for initial state on discovery."""
        client = self._lc_node_get_state_clients[name]
        future = client.call_async(GetState.Request())
        # TODO: we can maybe use add_done_callback in node.py as well, for ease of use, now that we learned about it?
        future.add_done_callback(lambda f: self._on_lifecycle_get_state_response(name, f))

    def _on_lifecycle_get_state_response(self, name: str, future):
        try:
            response = future.result()
            state = LifecycleState(response.current_state.id)
            self._lc_components[name] = Component(
                name=name,
                component_type=ComponentType.LIFECYCLE_NODE,
                lifecycle_state=state
            )
            self._node.get_logger().info(
                f"{self._logger_prefix} Lifecycle node '{name}' discovered. State: {state.name}"
            )
            self._push_merged_state()

        except Exception as e:
            self._node.get_logger().warning(
                f"{self._logger_prefix} Failed to get state for '{name}': {e}"
            )

    def _lifecycle_transition_event_callback(self, name: str, msg: TransitionEvent):
        """Reactive state update from /<node>/transition_event."""
        try:
            new_state = LifecycleState(msg.goal_state.id)
            self._lc_components[name] = Component(
                name=name,
                component_type=ComponentType.LIFECYCLE_NODE,
                lifecycle_state=new_state
            )
            self._push_merged_state()
        except ValueError:
            pass

    def _push_merged_state(self):
        """Combine all sources and push complete state to the engine."""
        all_components = list(self._cm_components.values()) + list(self._lc_components.values())

        was_ready = self._engine.is_ready
        response = self._engine.set_system_state(all_components)

        if not was_ready and self._engine.is_ready:
            self._node.get_logger().info(
                f"{self._logger_prefix} Foreman is READY. Fresh state received.")

        if not response.success and response.error:
            self._node.get_logger().error(
                f"{self._logger_prefix} [{response.error.category.value}] \n{response.error.message}"
            )
