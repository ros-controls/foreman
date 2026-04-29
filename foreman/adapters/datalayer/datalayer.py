import os
import ctrlxdatalayer
from ctrlxdatalayer.variant import Variant, Result
from ctrlxdatalayer.provider_node import ProviderNodeCallbacks

class ExampleStringNode(ProviderNodeCallbacks):
    """A simple Data Layer node that holds a string."""
    def __init__(self, initial_value: str):
        super().__init__()
        self.data = initial_value

    def on_read(self, userdata: ctrlxdatalayer.clib.userData_c_void_p, address: str, data: Variant) -> Result:
        # This is called when an external client reads this node
        data.set_string(self.data)
        return Result.OK

    def on_write(self, userdata: ctrlxdatalayer.clib.userData_c_void_p, address: str, data: Variant) -> Result:
        # This is called when an external client tries to write to this node
        self.data = data.get_string()
        return Result.OK

class DatalayerAdapter:
    def __init__(self):
        self.system = None
        self.provider = None
        self.my_node = None
        self.node_path = "foreman/test_string"

    def start(self):
        # 1. Initialize the Data Layer System
        self.system = ctrlxdatalayer.system.System("")
        self.system.start(False)

        # 2. Create Provider (Connecting to IPC via the plug)
        # Using IPC connecting to the host ctrlX OS datalayer
        ipc_path = os.environ.get('SNAP_DATA', '/tmp') + '/datalayer/run/snap/ipc/app.sock'
        self.provider = self.system.factory().create_provider(f"ipc://{ipc_path}")
        self.provider.start()

        if not self.provider.is_connected():
            print("Failed to connect to Data Layer!")
            return False

        # 3. Create our custom node and register it
        self.my_node = MySimpleStringNode("Initial Foreman Value")
        result = self.provider.register_node(self.node_path, self.my_node)
        print(f"Registered node '{self.node_path}' with result: {result}")
        
        return True

    def update_test_string(self, new_value: str):
        """Called by your ROS 2 node to update the string internally."""
        if self.my_node:
            self.my_node.data = new_value

    def stop(self):
        if self.provider:
            self.provider.unregister_node(self.node_path)
            self.provider.stop()
        if self.system:
            self.system.stop(False)