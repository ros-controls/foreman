"""
Dummy lifecycle node for testing foreman's lifecycle node adapter.

Run with: ros2 run foreman dummy_lifecycle_node
Transition with: ros2 lifecycle set /dummy_lifecycle_node configure
"""

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn


class DummyLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('dummy_lifecycle_node')
        self.get_logger().info('Dummy lifecycle node started. State: UNCONFIGURED')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() called. Transitioning to INACTIVE.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate() called. Transitioning to ACTIVE.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate() called. Transitioning to INACTIVE.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() called. Transitioning to UNCONFIGURED.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() called. Transitioning to FINALIZED.')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = DummyLifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
