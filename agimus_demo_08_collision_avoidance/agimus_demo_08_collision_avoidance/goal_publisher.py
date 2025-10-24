import copy
import itertools
from collections import deque

import numpy as np
import pinocchio as pin
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
    interpolate_weights,
)
from agimus_controller_ros.ros_utils import weighted_traj_point_to_mpc_msg
from agimus_msgs.msg import MpcInput
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, Header, Int32
from visualization_msgs.msg import Marker

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from agimus_demo_08_collision_avoidance.goal_publisher_parameters import (
    goal_publisher,
)


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")

        try:
            self._param_listener = goal_publisher.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        self._marker_base: Marker | None = None
        self._buffer_size: int | None = None
        self._nv = len(self._params.joint_reg_pose)
        self._nq = len(self._params.joint_reg_pose)
        self._first_run: bool = True

        self._trajectory_point_base = TrajectoryPoint(
            id=0,
            time_ns=0,
            robot_configuration=np.array(self._params.joint_reg_pose),
            robot_velocity=np.zeros(7),
            robot_acceleration=np.zeros(self._nv),
            robot_effort=np.zeros(self._nv),
            forces={},
            end_effector_poses={self._params.tool_frame_id: None},
        )

        self._goal_poses = {
            name: None for name in ("motion_start_weights", "final_pose_weights")
        }

        self._target_poses = {
            name: pin.SE3(
                pin.Quaternion(np.array(getattr(self._params, name).rotation)),
                np.array(getattr(self._params, name).translation),
            )
            for name in self._params.target_poses_names
        }
        self._goal_cycle = itertools.cycle(self._params.target_poses_names)

        self._trajectory_buffer = deque()

        self._update_params(first_call=True)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._buffer_size_sub = self.create_subscription(
            Int32,
            "mpc_buffer_size",
            self._buffer_size_cb,
            qos_profile=QoSProfile(
                depth=1000,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        # Publishers
        self._mpc_input_pub = self.create_publisher(
            MpcInput,
            "mpc_input",
            qos_profile=QoSProfile(
                depth=1000,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        if self._params.visualization.publish_target_marker:
            self._target_pose_marker_pub = self.create_publisher(
                Marker, "target_pose_marker", 10
            )
        else:
            self._path_publisher = None
            self._mpc_target_pose = None

        # Create a timer that calls the _maybe_run_prediction method every 2 seconds
        self._planner_timer = self.create_timer(
            self._params.ocp_dt * self._params.ocp_buffer_size, self._pose_generator_cb
        )

        self._mpc_input_publisher_timer = self.create_timer(
            self._params.ocp_dt / 2.0, self._publish_mpc_input_cb
        )

        self.get_logger().info("Node initialized!")

    def _update_params(self, first_call: bool = False) -> None:
        if first_call or self._param_listener.is_old(self._params):
            if not first_call:
                self._param_listener.refresh_dynamic_parameters()
                self._params = self._param_listener.get_params()

            for weights_group in self._goal_poses.keys():
                stage_weights = getattr(self._params.weights, weights_group)

                for param_name, expected_len in (
                    ("w_robot_configuration", self._nq),
                    ("w_robot_velocity", self._nq),
                    ("w_robot_effort", self._nq),
                ):
                    param_len = len(getattr(stage_weights, param_name))
                    if param_len != expected_len:
                        e = ParameterException(
                            f"Param '{param_name}' has {param_len} elements while {expected_len} expected!",
                            (f"trajectory.{param_name}", "trajectory"),
                        )
                        self.get_logger().error(str(e))
                        raise e

                self._goal_poses[weights_group] = TrajectoryPointWeights(
                    w_robot_configuration=np.array(stage_weights.w_robot_configuration),
                    w_robot_velocity=np.array(stage_weights.w_robot_velocity),
                    w_robot_acceleration=np.zeros_like(
                        stage_weights.w_robot_configuration
                    ),
                    w_robot_effort=np.array(stage_weights.w_robot_effort),
                    w_end_effector_poses={
                        self._params.tool_frame_id: np.concatenate(
                            (
                                np.asarray(stage_weights.w_frame_translation),
                                np.asarray(stage_weights.w_frame_rotation),
                            )
                        )
                    },
                    w_end_effector_velocities={},
                    w_forces={},
                    w_collision_avoidance=0.0,
                )

            scale = self._params.visualization.marker_size
            self._marker_base = Marker(
                header=Header(frame_id=self._params.world_frame_id),
                ns="goal_publisher",
                type=Marker.SPHERE,
                action=Marker.ADD,
                scale=Vector3(x=scale, y=scale, z=scale),
                color=ColorRGBA(
                    **dict(zip("rgba", self._params.visualization.marker_color))
                ),
                lifetime=Duration(seconds=self._params.ocp_dt * 10.0).to_msg(),
            )

    def _buffer_size_cb(self, msg: Int32) -> None:
        self._buffer_size = msg.data

    def _insert_interpolated_point_to_buffer(
        self,
        start_weights: TrajectoryPointWeights,
        target_weights: TrajectoryPointWeights,
        target_point: TrajectoryPoint,
        time: float,
    ) -> None:
        # Perform gain scheduling
        dt = self._params.ocp_dt
        gain_schedule_points = int(np.ceil(time / dt))
        weighted_trajectory = [
            WeightedTrajectoryPoint(
                point=copy.copy(target_point),
                weights=interpolate_weights(
                    start_weights, target_weights, i / gain_schedule_points * dt
                ),
            )
            for i in range(gain_schedule_points)
        ]
        self._trajectory_buffer.extend(weighted_trajectory)

    def _pose_generator_cb(self) -> None:
        self._update_params()

        if self._buffer_size is None:
            self.get_logger().info(
                f"Waiting for topic '{self._buffer_size_sub.topic}'...",
                throttle_duration_sec=5.0,
            )
            return

        if len(self._trajectory_buffer) > 2 * self._params.ocp_buffer_size:
            return

        target_pose_name = next(self._goal_cycle)
        self.get_logger().info(f"Moving to '{target_pose_name}' target pose.")
        target_point = copy.copy(self._trajectory_point_base)
        target_point.end_effector_poses[self._params.tool_frame_id] = (
            self._target_poses[target_pose_name]
        )

        # Choose different first motion time for safety
        start_motion_time = (
            self._params.schedule.first_motion_time
            if self._first_run
            else self._params.schedule.start_to_final_time
        )
        self._first_run = False

        # Move to the goal
        self._insert_interpolated_point_to_buffer(
            self._goal_poses["motion_start_weights"],
            self._goal_poses["final_pose_weights"],
            target_point,
            start_motion_time,
        )

        # Stay at the goal
        self._insert_interpolated_point_to_buffer(
            self._goal_poses["final_pose_weights"],
            self._goal_poses["final_pose_weights"],
            target_point,
            self._params.schedule.wait_time,
        )

        # Ramp down gains
        self._insert_interpolated_point_to_buffer(
            self._goal_poses["final_pose_weights"],
            self._goal_poses["motion_start_weights"],
            target_point,
            self._params.schedule.final_to_start_time,
        )

    def _publish_mpc_input_cb(self) -> None:
        # Skip if buffer is empty
        if not len(self._trajectory_buffer):
            return

        if self._buffer_size >= self._params.ocp_buffer_size:
            return

        if len(self._trajectory_buffer) == 1:
            weighted_traj_point = self._trajectory_buffer[0]
        else:
            weighted_traj_point = self._trajectory_buffer.popleft()

        mpc_input_msg = weighted_traj_point_to_mpc_msg(weighted_traj_point)
        self._mpc_input_pub.publish(mpc_input_msg)

        if self._params.visualization.publish_target_marker:
            self._marker_base.pose = mpc_input_msg.ee_inputs[0].pose
            self._target_pose_marker_pub.publish(self._marker_base)


def main(args=None):
    rclpy.init()
    try:
        goal_publisher_node = GoalPublisher()
        rclpy.spin(goal_publisher_node)
        goal_publisher_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
