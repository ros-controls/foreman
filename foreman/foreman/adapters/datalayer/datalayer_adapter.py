import os

from ament_index_python.packages import get_package_share_directory
import ctrlxdatalayer
from ctrlxdatalayer.variant import Result

from foreman.adapters.datalayer.ctrlx_connection import get_provider
from foreman.adapters.datalayer.ctrlx_node_set_goal import CtrlXNodeSetGoal
from foreman.adapters.datalayer.ctrlx_node_snapshot import CtrlXNodeSnapshot
from foreman.engine import ForemanEngine


class DatalayerAdapter:
    """
    Main facade for Foreman <-> Data Layer integration.

    This is a single adapter as it maintains a single DatalayerProvider with
    multiple nodes attached to it.
    """

    def __init__(self, ros_logger, engine: ForemanEngine):
        self.ros_logger = ros_logger
        self.engine = engine

        self.system = None
        self.provider = None

        self.snapshot_node = None
        self.set_goal_node = None

        self.snapshot_type_path = "types/foreman/snapshot"

    def start(self) -> bool:
        """Start the system, provider, and registers all nodes."""
        self.system = ctrlxdatalayer.system.System("")
        self.system.start(False)

        self.provider, conn_string = get_provider(self.system)
        if not self.provider:
            self.ros_logger.error(f"Failed to start ctrlX Provider on {conn_string}!")
            return False

        # register our flatbuffer type for snapshot node to use.
        # as per setup.py, the bfbs is in foreman/flatbuffers/...
        bfbs_path = os.path.join(
            get_package_share_directory('foreman'), "flatbuffers", "flatbuffer_foreman.bfbs"
        )
        if self.provider.register_type(self.snapshot_type_path, bfbs_path) != Result.OK:
            self.ros_logger.error("Failed to register ForemanSnapshot type.")
            return False

        # register nodes
        self.snapshot_node = CtrlXNodeSnapshot(
            provider=self.provider,
            address="foreman/snapshot",
            type_path=self.snapshot_type_path,
            engine=self.engine
        )
        self.snapshot_node.register()

        self.set_goal_node = CtrlXNodeSetGoal(
            provider=self.provider,
            address="foreman/set_goal",
            engine=self.engine,
            ros_logger=self.ros_logger
        )
        self.set_goal_node.register()

        self.ros_logger.info("ctrlX Datalayer nodes registered successfully.")
        return True

    def stop(self):
        """Tear down datalayer provider."""
        self.ros_logger.info("Stopping Datalayer adapter...")
        if self.snapshot_node:
            self.snapshot_node.unregister()
        if self.set_goal_node:
            self.set_goal_node.unregister()
        if self.provider:
            self.provider.unregister_type(self.snapshot_type_path)
            self.provider.stop()
            self.provider.close()
        if self.system:
            self.system.stop(False)
        self.ros_logger.info("Datalayer adapter stopped.")
