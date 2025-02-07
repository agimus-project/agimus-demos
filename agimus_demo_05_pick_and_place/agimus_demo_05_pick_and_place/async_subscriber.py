from copy import deepcopy

from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_system_default, QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions


class AsyncSubscriber:
    def __init__(
        self,
        node: Node,
        message_type: type,
        topic_name: str,
        qos_profile:QoSProfile=qos_profile_system_default,
    ) -> None:
        self._node = node

        # Joint states subscriber
        self._subscription = self._node.create_subscription(
            message_type,
            topic_name,
            self._message_cb,
            qos_profile=qos_profile,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self._future = Future()
        # Initialize the future to be already in state `Done`
        self._future.set_result(None)

    def _message_cb(self, msg):
        """Callback function to latest value of the message."""
        # Complete the future if it was waiting
        if not self._future.done():
            self._future.set_result(msg)

    def wait_for_future(self):
        """Blocks until a new joint state is received."""
        self._future = Future()  # Reset future for next call
        self._node.executor.spin_until_future_complete(self._future)
        return deepcopy(self._future.result())