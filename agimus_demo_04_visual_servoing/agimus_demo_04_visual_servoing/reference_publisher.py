import math
import numpy as np
import pinocchio

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from turtlesim.srv import Spawn


from agimus_controller_ros.ros_utils import (
    get_params_from_node,
    transform_msg_to_se3,
    weighted_traj_point_to_mpc_msg,
    se3_to_transform_msg,
)
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase


# Split SimpleTrajectoryPublisher so that the initialization is in base class while the handling of trajectory is in child class.
# Implement a child class of the above base class that merely publishes a constant point.


class ReferencePublisher(TrajectoryPublisherBase):

    def __init__(self):
        super().__init__("reference_publisher")

        params = get_params_from_node(
            self,
            "agimus_controller_node",
            [
                "ocp.dt",
            ],
        )
        self._dt = params[0].double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # This frame should be the same as the one used in agimus_controller
        # visual servoing residual
        self.camera_frame = "camera_color_optical_frame"
        self.object_frame = "current_object"

    def ready_callback(self):
        # Call on_timer function every second
        self.timer = self.create_timer(self._dt, self.initialize_transform)

    def initialize_transform(self):
        # Timer callback for initialization
        # It waits until a transform in TF is found.
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.object_frame, rclpy.time.Time()
            )
            self._cMo = transform_msg_to_se3(t.transform)
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.camera_frame} to {self.object_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return
        
        # Send the reference used by vision to TF so that it can be visualized in RViz.
        # This transform isn't used by agimus_controller_node.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.object_frame
        t.child_frame_id = self.camera_frame + "_reference"
        t.transform = se3_to_transform_msg(self._cMo.inverse())
        self.tf_static_broadcaster.sendTransform(t)

        model = self.robot_models.robot_model
        data = model.createData()
        q = self.q0
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        tau = pinocchio.rnea(model, data, q, v, a)

        w_q = 0.01 * np.ones(model.nv)
        w_v = 0.01 * np.ones(model.nv)
        w_a = 0.01 * np.ones(model.nv)
        w_tau = 0.000001 * np.ones(model.nv)
        w_pose = 1.0 * np.ones(6)

        self._point = WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                id=0,
                robot_configuration=q,
                robot_velocity=v,
                robot_acceleration=a,
                robot_effort=tau,
                end_effector_poses={self.camera_frame + "_vs": self._cMo.inverse()},
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=w_q,
                w_robot_velocity=w_v,
                w_robot_acceleration=w_a,
                w_robot_effort=w_tau,
                w_end_effector_poses={self.camera_frame + "_vs": w_pose},
            ),
        )

        self.get_logger().info(
            "Starting to publish references",
            throttle_duration_sec=2.0,
        )
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(self._dt, self.publish_reference)

    def publish_reference(self):
        msg = weighted_traj_point_to_mpc_msg(self._point)
        if self._point.point.id < 50:
            msg.w_pose = [0.0] * 6
        self.publisher_.publish(msg)
        self._point.point.id += 1


def main():
    rclpy.init()
    node = ReferencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
