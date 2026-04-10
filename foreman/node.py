import threading
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import foreman.adapters as Adapters
from foreman.parser import parse_yaml_file
from foreman.engine import ForemanEngine


class ForemanNode(Node):
    """
    Glues the Foreman Engine and its adapters.
    """

    def __init__(self):
        super().__init__('foreman_node')
        
        # CONFIG  =============================================
        # TODO: parametrize this!
        config_path = "/home/nikolab/bRobotized/workspaces/grc26/src/foreman/foreman/config/scenario.yaml"
        self.config = parse_yaml_file(config_path)
        # self.declare_parameter('config_path', '')
        # config_path = self.get_parameter('config_path').value
        
        # if not config_path:
        #     raise ValueError("Parameter 'config_path' must be provided")


        self.state_lock = threading.Lock()
        self.is_service_call_in_progress = False

        # CORE ENGINE  =============================================
        self.engine = ForemanEngine(self.config, self.state_lock)

        # CONTROLLER MANAGER ADAPTERS ==============================
        self.callback_group_services = MutuallyExclusiveCallbackGroup()
        self.callback_group_subscriber = ReentrantCallbackGroup()
        self.callback_group_timer = ReentrantCallbackGroup()

        controller_manager_name = self.config.controller_manager

        self.state_monitor = Adapters.ControllerManager.StateMonitor(
            node=self, 
            engine=self.engine,
            controller_manager_name=controller_manager_name
        )
        self.service_caller = Adapters.ControllerManager.ServiceCaller(
            node=self, 
            transition_pause=self.config.transition_pause, 
            controller_manager_name=controller_manager_name
        )

        # ADAPTERS TO THE REST OF ROS ==============================

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

    def callback_main_loop(self):
        """Main loop."""
        if self.is_service_call_in_progress:
            return
        
        commands = self.engine.get_next_transition()

        self.is_service_call_in_progress = True
        
        try:
            # TODO: Can we namespace these to something like "ToControllerManager"
            self.service_caller.execute_transitions(commands)
        except Exception as e:
            self.get_logger().error(f"Execution sequence failed: {e}")
            # TODO: Handle error cases here - where do we transition?
            # TODO: how do we pass mess
        finally:
            self.is_service_call_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    
    node = ForemanNode()

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