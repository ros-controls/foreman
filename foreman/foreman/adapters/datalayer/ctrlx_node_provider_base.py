import ctrlxdatalayer
from ctrlxdatalayer.provider import Provider
from ctrlxdatalayer.provider_node import NodeCallback
from ctrlxdatalayer.provider_node import ProviderNode
from ctrlxdatalayer.provider_node import ProviderNodeCallbacks
from ctrlxdatalayer.variant import Result
from ctrlxdatalayer.variant import Variant


class CtrlXNodeProviderBase:
    """
    Base class for ctrlX Data Layer provider nodes.
    """

    def __init__(self, provider: Provider, address: str, metadata: Variant):
        self.provider = provider
        self.address = address
        self.metadata = metadata

        self.cbs = ProviderNodeCallbacks(
            self._on_create,
            self._on_remove,
            self._on_browse,
            self._on_read,
            self._on_write,
            self._on_metadata,
        )
        self.provider_node = ProviderNode(self.cbs)

    def register(self) -> Result:
        return self.provider.register_node(self.address, self.provider_node)

    def unregister(self):
        self.provider.unregister_node(self.address)

    # --- Default Callbacks ---

    def _on_create(self, userdata, address: str, data: Variant, cb: NodeCallback):
        cb(Result.OK, data)

    def _on_remove(self, userdata, address: str, cb: NodeCallback):
        cb(Result.UNSUPPORTED, None)

    def _on_browse(self, userdata, address: str, cb: NodeCallback):
        with Variant() as new_data:
            new_data.set_array_string([])
            cb(Result.OK, new_data)

    def _on_read(self, userdata, address: str, data: Variant, cb: NodeCallback):
        cb(Result.UNSUPPORTED, None)

    def _on_write(self, userdata, address: str, data: Variant, cb: NodeCallback):
        cb(Result.UNSUPPORTED, None)

    def _on_metadata(self, userdata, address: str, cb: NodeCallback):
        cb(Result.OK, self.metadata)
