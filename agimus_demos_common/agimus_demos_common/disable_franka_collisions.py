import sys

import rclpy

from agimus_franka_msgs.srv import SetFullCollisionBehavior  # noqa: I001

# Automatically generated file
from agimus_demos_common.disable_franka_collisions_parameters import (
    disable_franka_collisions,
)  # noqa: E402


def main(args=None) -> int:
    rclpy.init(args=args)
    node = rclpy.create_node("disable_franka_collisions")

    try:
        param_listener = disable_franka_collisions.ParamListener(node)
        params = param_listener.get_params()
    except Exception as e:
        node.get_logger().error(str(e))
        return 1

    srv_client = node.create_client(
        SetFullCollisionBehavior, "/service_server/set_full_collision_behavior"
    )

    # Extract all parameters with "thresholds" in name
    params_attributes = [attr for attr in dir(params) if "thresholds" in attr]
    alignment = max(len(attr) for attr in params_attributes) + 1
    # Populate fields expected by the request with values from parameters
    params_dict = {attr: getattr(params, attr) for attr in params_attributes}
    req = SetFullCollisionBehavior.Request(**params_dict)
    node.get_logger().info("Node initialized.")

    # Wait 10 second before timing out
    max_trials = 10
    while not srv_client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info(
            f"Waiting for service '{srv_client.srv_name}'...",
        )
        rclpy.spin_once(node)
        max_trials -= 1
        if max_trials <= 0:
            node.get_logger().error(
                "Failed to set collision limits for Franka robot! "
                + f"Timeout reached when waiting for the service {srv_client.srv_name}!"
            )
            node.destroy_node()
            rclpy.shutdown()
            return 1

    future = srv_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done:
        node.get_logger().error(
            "Failed to set collision limits for Franka robot! "
            + f"Timeout reached when spinning the service {srv_client.srv_name}!"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result = future.result()
    if result is None:
        node.get_logger().error(
            "Failed to set collision limits for Franka robot! "
            + f"Failed to call the service the service {srv_client.srv_name}!"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not result.success:
        node.get_logger().error(
            "Failed to set collision limits for Franka robot! "
            + f"Reason: '{result.error}'"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().warn(
        "Collision limits for Franka robot were changed to values:\n"
        + "".join(
            [
                f"\t{(key + ':').ljust(alignment, ' ')} {val}\n"
                for key, val in params_dict.items()
            ]
        )
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
