from collections import deque

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int64

from agimus_msgs.msg import MpcInputArray

from agimus_controller_ros.ros_utils import weighted_traj_point_to_mpc_msg


class AsyncTrajectoryPublisher:
    def __init__(
        self,
        node: Node,
        mpc_input_topic_name: str,
        buffer_size_topic_name: str,
        update_frequency: float,
        buffer_size: int,
    ):
        self._node = node

        self._buffer_size = buffer_size
        self._trajectory_buffer: deque | None = None
        self._msg = None

        self._buffer_size_subscriber = self._node.create_subscription(
            Int64,
            buffer_size_topic_name,
            self._buffer_size_cb,
            10,
        )

        self._mpc_input_publisher = self._node.create_publisher(
            MpcInputArray,
            mpc_input_topic_name,
            qos_profile=QoSProfile(
                depth=1000,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        # self._timer = self._node.create_timer(1.0 / update_frequency, self._publish_cb)

    def _buffer_size_cb(self, msg: Int64):
        """Callback function to latest value of the buffer size of the controller."""
        self._msg = msg

    def _publish_cb(self):
        num_to_publish = self._buffer_size - self._msg.data
        if num_to_publish <= 0:
            return

        def _get_traj_point():
            if len(self._trajectory_buffer) == 1:
                weighted_traj_point = self._trajectory_buffer[0]
            else:
                weighted_traj_point = self._trajectory_buffer.popleft()
            return weighted_traj_point_to_mpc_msg(weighted_traj_point)

        mpc_input_array = MpcInputArray(
            inputs=[_get_traj_point() for _ in range(num_to_publish)]
        )
        self._mpc_input_publisher.publish(mpc_input_array)

    def set_new_trajectory(self, trajectory):
        self._trajectory_buffer = deque(trajectory)
