import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from controller_manager_msgs.msg import ControllerManagerActivity
from foreman.engine import ForemanEngine
from foreman.types import Component, ComponentType, LifecycleState, SystemState


class StateMonitor:
    """
    Southbound Inbound Adapter.
    
    Monitors the Controller Manager's activity topic and maintains the 
    internal SystemState.
    """

    def __init__(self, node: Node, engine: ForemanEngine, controller_manager_name: str):
        self._node = node
        self._engine = engine
        
        # Need TransientLocal here
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._subscription = self._node.create_subscription(
            ControllerManagerActivity,
            f'/{controller_manager_name}/activity',
            self._callback,
            qos_profile,
            callback_group=self._node.callback_group_subscriber
        )
        self._node.get_logger().info(f"Adapters.ControllerManager.StateMonitor: Subscribed to /{controller_manager_name}/activity")

    def _callback(self, msg: ControllerManagerActivity):
        """Parses ROS message to Foreman Components and sets them in the Engine."""
        components = []
        
        for hw_msg in msg.hardware_components:
            try:
                components.append(Component(
                    name=hw_msg.name,
                    component_type=ComponentType.HARDWARE,
                    lifecycle_state=LifecycleState(hw_msg.state.id)
                ))
            except ValueError:
                continue

        for ctrl_msg in msg.controllers:
            try:
                components.append(Component(
                    name=ctrl_msg.name,
                    component_type=ComponentType.CONTROLLER,
                    lifecycle_state=LifecycleState(ctrl_msg.state.id)
                ))
            except ValueError:
                continue

        self._engine.set_system_state(components)