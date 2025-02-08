import sys

import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos_overriding_options import QoSOverridingOptions

from sensor_msgs.msg import JointState  # noqa: I001


class WaitForNonZeroJoints(Node):
    """Main class implementing ROS node redirecting /robot_description topic
    to parameter of a node."""

    def __init__(self):
        super().__init__("wait_for_non_zero_joints_node")

        self.declare_parameter("timeout", 10.0)
        timeout_dbl = self.get_parameter("timeout").get_parameter_value().double_value
        if timeout_dbl < 0.0:
            e = ParameterException(
                "Parameter 'timeout' can not be negative!",
                ("timeout"),
            )
            self.get_logger().error(str(e))
            raise e
        self._timeout = Duration(seconds=timeout_dbl)

        self.declare_parameter("joints_sum_threshold", 1e-3)
        self._joints_sum_threshold = (
            self.get_parameter("joints_sum_threshold")
            .get_parameter_value()
            .double_value
        )
        if self._joints_sum_threshold < 0.0:
            e = ParameterException(
                "Parameter 'joints_sum_threshold' can not be negative!",
                ("joints_sum_threshold"),
            )
            self.get_logger().error(str(e))
            raise e

        self._joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_cb,
            qos_profile=qos_profile_system_default,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self._exit_code = 1
        self._init_time = self.get_clock().now()

        self.get_logger().info(
            "Node initialized, waiting for '/joint_states' to be published..."
        )

    @property
    def exit_code(self) -> int:
        """Return value of exit code set by the node.

        Returns:
            int: Last value of exit code.
        """
        return self._exit_code

    def _joint_states_cb(self, msg: JointState) -> None:
        """Callback called at every time `/joint_states` message is received.
        If sum of absolute values of joint positions is greater than a set threshold
        node is halted and exit code is set to `0`. If a timeout is reached, the node is
        terminated with exit code `1`.

        Args:
            msg (JointState): Message containing joint states of the robot.

        Raises:
            SystemExit: Sum of absolute positions of joints was greater than threshold.
            SystemExit: Timeout was reached.
        """
        self.get_logger().info("Joint state message received.", once=True)

        if sum([abs(p) for p in msg.position]) > self._joints_sum_threshold:
            self._exit_code = 0
            raise SystemExit()

        if (self.get_clock().now() - self._init_time) > self._timeout:
            self.get_logger().info(
                "Failed to receive non-zero joint "
                + "position before timeout was reached."
            )
            self._exit_code = 1
            raise SystemExit()


def main(args=None):
    rclpy.init(args=args)
    ret = 1
    wait_for_non_zero_joints_node = WaitForNonZeroJoints()
    try:
        rclpy.spin(wait_for_non_zero_joints_node)
    except SystemExit:
        wait_for_non_zero_joints_node.destroy_node()
        ret = wait_for_non_zero_joints_node.exit_code
        rclpy.try_shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
