"""Bridge node: forwards base velocity commands from the MPC to the mobile base controller."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class BaseCmdBridge(Node):
    def __init__(self):
        super().__init__("base_cmd_bridge")
        qos_in  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_out = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Twist, "/cmd_vel", qos_out)
        self._sub = self.create_subscription(Twist, "/base_cmd_vel", self._callback, qos_in)

    def _callback(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
