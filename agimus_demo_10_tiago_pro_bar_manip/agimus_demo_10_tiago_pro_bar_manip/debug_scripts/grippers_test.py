#!/usr/bin/env python3
"""
Debug script: open close grippers
"""

import time
import rclpy
from rclpy.node import Node

# from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Empty

LEFT_GRIPPER_RELEASE_SERVICE = "/gripper_left_grasper_srv/release"
LEFT_GRIPPER_GRASP_SERVICE = "/gripper_left_grasper_srv/grasp"
RIGHT_GRIPPER_RELEASE_SERVICE = "/gripper_right_grasper_srv/release"
RIGHT_GRIPPER_GRASP_SERVICE = "/gripper_right_grasper_srv/grasp"


class GripperTestNode(Node):
    def __init__(self):
        super().__init__("gripper_test_node")

        # client_left = self.create_client(Empty,LEFT_GRIPPER_RELEASE_SERVICE)
        # while not client_left.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # client_left.call_async(Empty.Request())
        self._ocp_dt = 0.1

        self.get_logger().info("Closing grippers")
        self.close_gripper()
        time.sleep(5)

        self.get_logger().info("Opening grippers")
        self.open_gripper()

        self.get_logger().info("Motion finished")

    def _call_grasper_service(
        self,
        service_name: list[str],
        action_label: list[str],
        timeout: list[float] = {5.0, 5.0},
    ) -> tuple:
        self.get_logger().info(f"Creating clients for {service_name}")

        client_left = self.create_client(Empty, service_name[0])
        client_right = self.create_client(Empty, service_name[1])

        deadline = time.time() + timeout
        while time.time() < deadline:
            if client_left.wait_for_service(
                timeout_sec=min(0.2, self._ocp_dt)
            ) and client_right.wait_for_service(timeout_sec=min(0.2, self._ocp_dt)):
                break
        else:
            self.get_logger().info(
                f"One of the Gripper service '{service_name}' is not available."
            )
            return False, False

        future_left = client_left.call_async(Empty.Request())
        future_right = client_right.call_async(Empty.Request())

        while not future_left.done() and future_right.done():
            time.sleep(5)

        if future_left.cancelled():
            self.get_logger().info(
                f"Gripper {action_label} request to '{service_name}' was cancelled."
            )
            return False

        exc = future_left.exception()
        if exc is not None:
            self.get_logger().info(
                f"Gripper {action_label} failed via '{service_name}': {exc}"
            )
            return False

        return True, True

    def open_gripper(
        self,
        timeout: float = 5.0,
    ) -> bool:
        """Open the left gripper in Gazebo."""
        return self._call_grasper_service(
            [LEFT_GRIPPER_RELEASE_SERVICE, RIGHT_GRIPPER_RELEASE_SERVICE],
            ["open", "open"],
            timeout=timeout,
        )

    def close_gripper(
        self,
        timeout: float = 5.0,
    ) -> bool:
        """Close the left gripper in Gazebo."""
        return self._call_grasper_service(
            [LEFT_GRIPPER_GRASP_SERVICE, RIGHT_GRIPPER_GRASP_SERVICE],
            ["close", "close"],
            timeout=timeout,
        )


def main():
    rclpy.init()
    node = GripperTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
