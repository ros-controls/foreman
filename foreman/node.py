import threading
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import foreman.adapters as Adapters
from foreman.parser import parse_yaml_file
from foreman.engine import ForemanEngine
from foreman.types import ForemanError, ForemanErrorCategory


class ForemanNode(Node):
    """
    Glues the Foreman Engine and its adapters.
    """

    def __init__(self):
        super().__init__('foreman_node')

        self.state_lock = threading.Lock()
        # for error handling ,so we know what and when failed and who to blame
        self._service_call_active_future = False
        self._active_transition = None 
        self.last_transition_time = self.get_clock().now()

        
        # CONFIG  =============================================
        self.ros_parameter_adapter = Adapters.ROS.Parameters(node=self)
        self.parameters = self.ros_parameter_adapter.load_parameters()
        self.config = parse_yaml_file(self.parameters.config_path)

        # CORE ENGINE  =============================================
        self.engine = ForemanEngine(self.config, self.state_lock)

        # CONTROLLER MANAGER ADAPTERS ==============================
        self.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.callback_group_subscriber = ReentrantCallbackGroup()
        self.callback_group_timer = MutuallyExclusiveCallbackGroup()

        controller_manager_name = self.config.controller_manager

        self.state_monitor = Adapters.ControllerManager.StateMonitor(
            node=self, 
            engine=self.engine,
            controller_manager_name=controller_manager_name
        )
        self.service_caller = Adapters.ControllerManager.ServiceCaller(
            node=self,
            controller_manager_name=controller_manager_name
        )

        # ADAPTERS TO THE REST OF ROS ==============================

        self.set_goal_server = Adapters.ROS.SetGoalServer(
            node=self, 
            engine=self.engine
        )

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

        # TEST DATALAYER
        self.dl_adapter = Adapters.Datalayer.DatalayerAdapter
        self.timer = self.create_timer(2.0, self.test_datalayer_callback, callback_group=self.callback_group_subscriber)
        self.counter = 0

    def test_datalayer_callback(self):
        msg = f"Foreman Heartbeat: {self.counter}"
        self.dl_adapter.update_test_string(msg)
        self.counter += 1

    def callback_main_loop(self):
        """Main loop."""

        # do we have an active transition running?
        if self._service_call_active_future and self._service_call_active_future.done():
            try:
                result = self._service_call_active_future.result()
                if not result.ok:
                    comp_name = self._active_transition.component.name if self._active_transition else "unknown"
                    fault = ForemanError(
                        category=ForemanErrorCategory.EXECUTION,
                        message="Controller manager rejected the transition.",
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

        # prevent concurrent service calls
        if self._service_call_active_future:
            return
        
        # throttle transitions by transition_pause
        time_since_last = (self.get_clock().now() - self.last_transition_time).nanoseconds / 1e9
        if time_since_last < self.config.transition_pause:
            return
        
        # Ok, now we get next command
        command = self.engine.get_next_transition()
        if not command:
            return

        try:
            self._active_transition = command
            self._service_call_active_future = self.service_caller.execute_transition(command)
        except Exception as e:
            fault = ForemanError(
                category=ForemanErrorCategory.EXECUTION,
                message=f"Failed to execute transition: {str(e)}",
                component_names=[command.component.name]
            )
            self._log_and_abort_goal(fault)
            self._active_transition = None

    def _log_and_abort_goal(self, fault: ForemanError):
        self.engine.abort_goal(fault)
        self.get_logger().error(f"[{fault.category.value}] {fault.message}. Failed components: {fault.component_names}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ForemanNode()

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()