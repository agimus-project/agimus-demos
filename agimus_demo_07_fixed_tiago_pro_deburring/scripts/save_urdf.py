#!/usr/bin/env python3
"""
Save the current robot URDF from /robot_description to a file.

Run this from the container once the robot is launched. The saved URDF
can then be used by the calibration scripts on the host (outside the container).

Usage:
    python3 scripts/save_urdf.py
    python3 scripts/save_urdf.py --output /path/to/tiago_pro.urdf
"""

import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_DEFAULT_OUTPUT = Path(__file__).parent.parent / "config" / "tiago_pro.urdf"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", "-o", default=str(_DEFAULT_OUTPUT))
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("urdf_saver")

    received = []
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    node.create_subscription(String, "/robot_description", received.append, qos)

    node.get_logger().info("Waiting for /robot_description ...")
    while not received:
        rclpy.spin_once(node, timeout_sec=0.1)

    urdf = received[0].data
    Path(parsed.output).parent.mkdir(parents=True, exist_ok=True)
    Path(parsed.output).write_text(urdf)
    node.get_logger().info(f"URDF saved to {parsed.output}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
