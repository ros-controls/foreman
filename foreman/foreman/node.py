import threading
import sys

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from foreman import adapters
from foreman.parser import parse_yaml_file
from foreman.engine import ForemanEngine
from foreman.types import ComponentType, ForemanError, ForemanErrorCategory


class ForemanNode(Node):
    """
    Glues the Foreman Engine and its adapters.
    """

    def __init__(self):
        super().__init__('foreman_node')

        self.foreman_state_lock = threading.Lock()
        # for error handling ,so we know what and when failed and who to blame
        self._service_call_active_future = False
        self._active_transition = None 
        self.last_transition_time = self.get_clock().now()

        self.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.callback_group_subscriber = ReentrantCallbackGroup()
        self.callback_group_timer = MutuallyExclusiveCallbackGroup()
        
        # CONFIG  =============================================
        self.ros_node_parameters = adapters.RosNodeParameters(node=self)
        self.foreman_parameters = self.ros_node_parameters.load_parameters()
        self.foreman_config = parse_yaml_file(self.foreman_parameters.config_path)

        # CORE ENGINE  =============================================
        self.foreman_engine = ForemanEngine(self.foreman_config, self.foreman_state_lock)

        # ADAPTERS ================================================
        controller_manager_name = self.foreman_config.controller_manager

        self.component_state_monitor = adapters.ComponentStateMonitor(
            node=self,
            engine=self.foreman_engine,
            controller_manager_name=controller_manager_name,
            lifecycle_nodes=self.foreman_config.lifecycle_nodes
        )
        self.controller_manager_service_caller = adapters.ControllerManagerServiceCaller(
            node=self,
            controller_manager_name=controller_manager_name
        )
        self.lifecycle_node_service_caller = adapters.LifecycleNodeServiceCaller(
            node=self,
            lifecycle_nodes=self.foreman_config.lifecycle_nodes
        )

        self.ros_set_goal_server = adapters.RosSetGoalServer(
            node=self, 
            engine=self.foreman_engine
        )

        self.autostart_adapter = adapters.AutostartAdapter(
            node=self,
            engine=self.foreman_engine,
            goal_name="active"
        ) 
        self.autostart = True

        # MAIN LOOP ================================================

        # RUN everything at 10HZ
        # TODO: Configure this?
        self.timer = self.create_timer(
            0.1, 
            self.callback_main_loop, 
            callback_group=self.callback_group_timer
        )
        # TODO: Add pretty print of current state and read config?
        self.get_logger().info("Foreman Node initialized.")

        # If available, start the datalayer adapter
        datalayer_available = True if adapters.DatalayerAdapter else None
        if datalayer_available:
            self.datalayer_adapter = adapters.DatalayerAdapter(ros_logger=self.get_logger(), engine=self.foreman_engine)
            self.get_logger().info("Datalayer adapter initialized.")
            datalayer_available = self.datalayer_adapter.start()

        if not datalayer_available:
            self.get_logger().info("Datalayer adapter not available.")
            self.datalayer_adapter = None
        self.counter = 0

    def callback_main_loop(self):
        """Main loop."""

        if not self.autostart_adapter.is_done:
            self.autostart_adapter.autostart()

        # do we have an active transition running?
        if self._service_call_active_future and self._service_call_active_future.done():
            try:
                result = self._service_call_active_future.result()

                # ControllerManager services return a field "ok" while LifecycleNode services return "success". :/
                if self._active_transition.component.component_type == ComponentType.LIFECYCLE_NODE:
                    ok = result.success
                else:
                    ok = result.ok

                if not ok:
                    comp_name = self._active_transition.component.name if self._active_transition else "unknown"
                    fault = ForemanError(
                        category=ForemanErrorCategory.EXECUTION,
                        message="Service rejected the transition.",
                        component_names=[comp_name]
                    )
                    self._log_and_abort_goal(fault)
            except Exception as e:
                comp_name = self._active_transition.component.name if self._active_transition else "unknown"
                fault = ForemanError(
                    category=ForemanErrorCategory.DELIVERY,
                    message=f"Service call exception: {str(e)}",
                    component_names=[comp_name]
                )
                self._log_and_abort_goal(fault)
            finally:
                self._service_call_active_future = None
                self._active_transition = None
                self.last_transition_time = self.get_clock().now()

        # prevent concurrent service calls to components
        if self._service_call_active_future:
            return
        
        # throttle transitions by transition_pause
        time_since_last = (self.get_clock().now() - self.last_transition_time).nanoseconds / 1e9
        if time_since_last < self.foreman_config.transition_pause:
            return
        
        # Ok, now we get next command
        command = self.foreman_engine.get_next_transition()
        if not command:
            return

        try:
            self._active_transition = command
            if command.component.component_type == ComponentType.LIFECYCLE_NODE:
                self._service_call_active_future = self.lifecycle_node_service_caller.execute_transition(command)
            else:
                self._service_call_active_future = self.controller_manager_service_caller.execute_transition(command)
        except Exception as e:
            fault = ForemanError(
                category=ForemanErrorCategory.EXECUTION,
                message=f"Failed to execute transition: {str(e)}",
                component_names=[command.component.name]
            )
            self._log_and_abort_goal(fault)
            self._active_transition = None

    def _log_and_abort_goal(self, fault: ForemanError):
        self.foreman_engine.abort_goal(fault)
        self.get_logger().error(f"[{fault.category.value}] {fault.message}. Failed components: {fault.component_names}")

    def destroy_node(self):
        """Safely stop adapters when shutting down node"""
        self.get_logger().info("Shutting down adapters...")

        if self.datalayer_adapter:
            try:
                self.datalayer_adapter.stop()
            except Exception as e:
                self.get_logger().error(f"Failed to stop datalayer adapter: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ForemanNode() 
    except Exception as e:
        print(f"[FATAL] [foreman_node]: Failed to initialize: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()