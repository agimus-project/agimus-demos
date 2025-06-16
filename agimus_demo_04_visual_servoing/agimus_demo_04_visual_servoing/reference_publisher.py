import numpy as np
import pinocchio

from geometry_msgs.msg import TransformStamped, PoseStamped
import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from agimus_controller_ros.ros_utils import (
    get_params_from_node,
    transform_msg_to_se3,
    weighted_traj_point_to_mpc_msg,
    se3_to_transform_msg,
    pose_msg_to_se3,
)
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase
from agimus_controller_ros.trajectory_weights_parameters import (
    trajectory_weights_params,
)


def _as_list_of_size(v: list[float], size: int) -> list[float]:
    """
    Convert list of a single element into a list of `size` element.
    If the input list has more than one element, it is returned as is.
    """
    assert isinstance(v, list)
    if len(v) == 1:
        return v * size
    else:
        return v


class ReferencePublisher(TrajectoryPublisherBase):
    """Trajectory publisher that publishes a constant relative pose wrt to the object.
    The relative pose corresponds to the first result from the vision pipeline.
    """

    def __init__(self):
        self._dt = None
        super().__init__("reference_publisher")

        self._traj_weight_param_listener = trajectory_weights_params.ParamListener(self)
        self._traj_weight_params = self._traj_weight_param_listener.get_params()

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
        self.vision_sub = self.create_subscription(
            PoseStamped, "/object/detections", self.apriltag_callback, 5
        )

        self.wMo: pinocchio.SE3 = None
        self.object_frame = "current_object"

    @property
    def end_effector_frame(self) -> str:
        # This frame should be the same as the one used in agimus_controller
        # visual servoing residual
        return self._traj_weight_params.ee_frame_name

    def apriltag_callback(self, pose_msg: PoseStamped):
        if len(pose_msg.header.frame_id) == 0:
            self.world_frame = "fer_link0"
        else:
            self.world_frame = pose_msg.header.frame_id
        self.get_logger().info("World frame set to " + self.world_frame)
        self.wMo_msg = pose_msg
        self.wMo = pose_msg_to_se3(pose_msg.pose)

        self.destroy_subscription(self.vision_sub)

    def ready_callback(self):
        # Although unlikely, it is possible that `ready_callback` is called
        # before self._dt has been set. This happened to me (Joseph) once.
        while self._dt is None:
            self.get_logger().info("Waiting for agimus controller node params.")
            rclpy.spin_once(timeout_sec=1.0)
        self.timer = self.create_timer(self._dt, self.initialize_transform)

    def get_transform(self, parent_frame, child_frame, stamp):
        try:
            transform_msg = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, stamp
            )
            return transform_msg
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {parent_frame} to {child_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return None

    def initialize_transform(self):
        if self.wMo is None:
            self.get_logger().info(
                "Waiting for apriltag detection",
                throttle_duration_sec=1.0,
            )
            return
        wMee_msg = self.get_transform(
            self.world_frame, self.end_effector_frame, self.wMo_msg.header.stamp
        )
        # if no transform available at desired time, take the latest one
        if wMee_msg is None:
            wMee_msg = self.get_transform(
                self.world_frame, self.end_effector_frame, rclpy.time.Time()
            )
            self.get_logger().warn(
                f"Could not find transform from {self.world_frame} to {self.end_effector_frame} at time {self.wMo_msg.header.stamp}. Using latest available value."
            )
        self._wMee = transform_msg_to_se3(wMee_msg.transform)
        self._oMee = self.wMo.inverse() * self._wMee

        # Send the reference used by vision to TF so that it can be visualized in RViz.
        # This transform isn't used by agimus_controller_node.
        oMee_msg = TransformStamped()
        oMee_msg.header.stamp = self.get_clock().now().to_msg()
        oMee_msg.header.frame_id = self.object_frame
        oMee_msg.child_frame_id = self.end_effector_frame + "_reference"
        oMee_msg.transform = se3_to_transform_msg(self._oMee)
        self.tf_static_broadcaster.sendTransform(oMee_msg)

        model = self.robot_models.robot_model
        data = model.createData()
        q = self.current_q
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        tau = pinocchio.rnea(model, data, q, v, a)

        w_q = _as_list_of_size(self._traj_weight_params.w_q, model.nv)
        w_v = _as_list_of_size(self._traj_weight_params.w_qdot, model.nv)
        w_a = _as_list_of_size(self._traj_weight_params.w_qddot, model.nv)
        w_tau = _as_list_of_size(self._traj_weight_params.w_robot_effort, model.nv)
        w_pose = _as_list_of_size(self._traj_weight_params.w_pose, 6)

        self._point = WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                id=0,
                robot_configuration=q,
                robot_velocity=v,
                robot_acceleration=a,
                robot_effort=tau,
                end_effector_poses={self.end_effector_frame + "_vs": self._oMee},
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=w_q,
                w_robot_velocity=w_v,
                w_robot_acceleration=w_a,
                w_robot_effort=w_tau,
                w_end_effector_poses={self.end_effector_frame + "_vs": w_pose},
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
