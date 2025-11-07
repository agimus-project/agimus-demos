import copy
import xml.etree.ElementTree as ET
from collections import deque

from pathlib import Path
import pickle

import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
    interpolate_weights,
)
from agimus_controller_ros.ros_utils import weighted_traj_point_to_mpc_msg
from agimus_msgs.msg import MpcInputArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header, Int64, String
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from agimus_demo_07_deburring.deburring_path_planner_parameters import (
    deburring_path_planner,
)
from agimus_demo_07_deburring.planner.trajectory_generators.deburring_trajectory import (
    DeburringPathGenerator,
)
from agimus_demo_07_deburring.planner.trajectory_generators.grasp_trajectory import (
    GraspPathGenerator,
)


class DeburringPathPlanner(Node):
    def __init__(self):
        super().__init__("deburring_path_planner")

        try:
            self._param_listener = deburring_path_planner.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        self._robot_model: pin.Model | None = None
        self._robot_data: pin.Data | None = None
        self._tool_frame_id_name: str = self._params.tool_frame_id
        self._tool_frame_id_pin_frame: int | None = None
        self._joint_states: JointState | None = None
        self._marker_base: Marker | None = None
        self._buffer_size: int | None = None
        self._in_contact: bool | None = None
        self._contact_counter: int = 0
        self._path_generators_initialized = False
        self._nv = len(self._params.moving_joints)
        self._nq = len(self._params.moving_joints)
        self._robot_description: str | None = None
        self._environment_description: str | None = None
        # Rotation used as a conversion between TCP coordinates and HPP handles
        self._R_insert = pin.SE3(
            pin.rpy.rpyToMatrix(np.array(self._params.hpp_handle_to_ee_rot)),
            np.array([0.0, 0.0, 0.0]),
        )

        self._path_generators = {
            name: {"generator": None, "weights": None}
            for name in (
                "follow_joint_trajectory",
                "insert_retract_tool",
                "deburring_motion",
            )
        }

        self._use_precomputed_trajectories = (
            self._params.generators_params.use_precomputed_trajectories
        )
        if self._use_precomputed_trajectories:
            self._trajectories_folder = Path(
                self._params.generators_params.precomputed_trajectories_path
            )
            if not self._trajectories_folder.is_dir():
                e = ParameterException(
                    f"Folder '{self._trajectories_folder.as_posix()}' "
                    "containing precomputed trajectories does not exist. ",
                    ("generators_params.precomputed_trajectories_path"),
                )
                self.get_logger().error(str(e))
                raise e
        else:
            self._trajectories_folder = None

        tree = ET.parse(self._params.handles_srdf_path)
        root = tree.getroot()
        self._handles = {}
        for handle in root.findall("handle"):
            pos = handle.find("position")
            self._handles[handle.get("name")] = pin.XYZQUATToSE3(
                np.concatenate(
                    (
                        np.fromstring(pos.get("xyz"), count=3, sep=" "),
                        np.fromstring(pos.get("xyzw"), count=4, sep=" "),
                    ),
                )
            )

        self._target_handle_name = None
        self._trajectory_buffer = deque()

        self._update_params(first_call=True)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscribers
        self._robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self._robot_description_cb,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self._environment_description_sub = self.create_subscription(
            String,
            "/environment_description_without_collision",
            self._environment_description_cb,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self._joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        self._buffer_size_sub = self.create_subscription(
            Int64, "/agimus_pytroller/buffer_size", self._buffer_size_cb, 10
        )

        self._target_handle_sub = self.create_subscription(
            String, "/target_handle", self._target_handle_cb, 10
        )

        # Publishers
        self._mpc_input_pub = self.create_publisher(
            MpcInputArray,
            "/agimus_pytroller/mpc_input",
            qos_profile=QoSProfile(
                depth=1000,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        if self._params.visualization.publish_path:
            self._path_publisher = self.create_publisher(
                MarkerArray, "diffusion_path", 10
            )
            self._mpc_target_pose = self.create_publisher(
                PoseStamped,
                "target_pose",
                10,
            )
        else:
            self._path_publisher = None
            self._mpc_target_pose = None

        # Create a timer that calls the _maybe_run_prediction method every 2 seconds
        self._planner_timer = self.create_timer(
            1.0 / self._params.rate, self._planner_cb
        )

        self._mpc_input_publisher_timer = self.create_timer(
            self._params.ocp_dt / 10.0, self._publish_mpc_input_cb
        )

        self._last_handle = "none"

        self.get_logger().info("Node initialized!")

    def _update_params(self, first_call: bool = False) -> None:
        if first_call or self._param_listener.is_old(self._params):
            if not first_call:
                self._param_listener.refresh_dynamic_parameters()
                self._params = self._param_listener.get_params()

            for weights_group in self._path_generators.keys():
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
                self._path_generators[weights_group]["weights"] = (
                    TrajectoryPointWeights(
                        w_robot_configuration=np.array(
                            stage_weights.w_robot_configuration
                        ),
                        w_robot_velocity=np.array(stage_weights.w_robot_velocity),
                        w_robot_acceleration=np.zeros_like(
                            stage_weights.w_robot_configuration
                        ),
                        w_robot_effort=np.array(stage_weights.w_robot_effort),
                        w_end_effector_poses={
                            self._tool_frame_id_name: np.concatenate(
                                (
                                    np.asarray(stage_weights.w_frame_translation),
                                    np.asarray(stage_weights.w_frame_rotation),
                                )
                            )
                        },
                        w_end_effector_velocities={},
                        w_forces={
                            self._params.measurement_frame_id: np.concatenate(
                                (
                                    np.array(stage_weights.w_desired_force),
                                    np.zeros(3),
                                )
                            )
                        },
                        w_collision_avoidance=stage_weights.w_collision_avoidance,
                    )
                )

            scale = self._params.visualization.marker_size
            self._marker_base = Marker(
                header=Header(frame_id=self._params.world_frame_id),
                ns="warmstart",
                type=Marker.SPHERE,
                action=Marker.ADD,
                scale=Vector3(x=scale, y=scale, z=scale),
                color=ColorRGBA(
                    **dict(zip("rgba", self._params.visualization.marker_color))
                ),
                lifetime=Duration(seconds=self._params.rate * 1.25).to_msg(),
            )

    def _robot_description_cb(self, msg: String) -> None:
        self._robot_description = msg.data
        self._robot_model = pin.buildModelFromXML(self._robot_description)
        self._robot_data = self._robot_model.createData()
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

    def _environment_description_cb(self, msg: String) -> None:
        self._environment_description = msg.data

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_states = msg

    def _buffer_size_cb(self, msg: Int64) -> None:
        self._buffer_size = msg.data

    def _target_handle_cb(self, msg: String) -> None:
        if msg.data not in self._handles.keys():
            self.get_logger().error(f"Invalid handle name '{msg.data}'")
            return

        self._target_handle_name = msg.data

        self.get_logger().info(f"Received new goal handle point: {msg.data}")

    def _get_remapped_joints(self) -> npt.ArrayLike:
        joint_map = [
            self._joint_states.name.index(joint_name)
            for joint_name in self._params.moving_joints
        ]
        return np.array(self._joint_states.position)[joint_map]

    def _create_trajectory_point(self, i: int, q: npt.ArrayLike) -> TrajectoryPoint:
        pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

        return TrajectoryPoint(
            id=i,
            time_ns=0,
            robot_configuration=q,
            robot_velocity=np.zeros_like(q),
            robot_acceleration=np.zeros(self._nv),
            robot_effort=np.zeros(self._nv),
            forces={},
            end_effector_poses={
                self._tool_frame_id_name: copy.copy(
                    self._robot_data.oMf[self._tool_frame_id_pin_frame]
                )
            },
        )

    def _insert_sequence_to_buffer(
        self, generator_name: str, q: npt.ArrayLike, T_taget: pin.SE3, T_init: pin.SE3
    ) -> None:
        generator = self._path_generators[generator_name]
        wp = self._trajectory_buffer[-1]
        point = wp.point
        w1 = wp.weights
        w2 = generator["weights"]

        # Perform gain scheduling
        dt = self._params.ocp_dt
        gain_schedule_points = int(
            np.ceil(self._params.weights.schedule.interpolation_time / dt)
        )
        weighted_trajectory = [
            WeightedTrajectoryPoint(
                point=copy.copy(point),
                weights=interpolate_weights(w1, w2, i / gain_schedule_points),
            )
            for i in range(gain_schedule_points)
        ]
        self._trajectory_buffer.extend(weighted_trajectory)

        # Compute and insert actual trajectory
        trajectory = generator["generator"].get_path(q, T_taget, T_init=T_init)
        weighted_trajectory = [
            WeightedTrajectoryPoint(point=p, weights=copy.copy(w2)) for p in trajectory
        ]
        self._trajectory_buffer.extend(weighted_trajectory)

    def _planner_cb(self) -> None:
        self._update_params()

        for topic_name, object in (
            (self._robot_description_sub.topic, self._robot_model),
            (self._environment_description_sub.topic, self._environment_description),
            (self._joint_states_sub.topic, self._joint_states),
            (self._buffer_size_sub.topic, self._buffer_size),
        ):
            if object is None:
                self.get_logger().info(
                    f"Waiting for topic '{topic_name}'...",
                    throttle_duration_sec=5.0,
                )
                return

        if not self._path_generators_initialized:
            try:
                if self._params.generators_params.main_generator == "diffusion":
                    from agimus_demo_07_deburring.planner.trajectory_generators.diffusion_trajectory import (
                        DiffusionPathGenerator,
                    )

                    self._path_generators["follow_joint_trajectory"]["generator"] = (
                        DiffusionPathGenerator(
                            wights_path=self._params.generators_params.diffusion_generator.weights_path,
                            sequence_length=self._params.generators_params.diffusion_generator.sequence_length,
                            robot_model=self._robot_model,
                            ocp_dt=self._params.ocp_dt,
                            max_joint_velocity=np.array(
                                self._params.max_joint_velocity
                            ),
                            tool_frame_id=self._tool_frame_id_name,
                        )
                    )
                else:
                    from agimus_demo_07_deburring.planner.trajectory_generators.hpp_trajectory import (
                        HPPPathGenerator,
                    )

                    gen_params = self._params.generators_params.hpp_generator
                    self._path_generators["follow_joint_trajectory"]["generator"] = (
                        HPPPathGenerator(
                            handles_configurations_path=Path(
                                gen_params.handle_configs_path
                            ),
                            robot_description=self._robot_description,
                            environment_description=self._environment_description,
                            ocp_dt=self._params.ocp_dt,
                            tool_frame_id=self._tool_frame_id_name,
                            robot_model=self._robot_model,
                            robot_self_collision_config=Path(
                                gen_params.robot_self_collision_config
                            ),
                            handles_srdf_path=Path(self._params.handles_srdf_path),
                            demo_config=Path(gen_params.demo_config),
                            robot_name=gen_params.robot_name,
                            gripper_name=gen_params.gripper_name,
                            deburred_object_name=gen_params.deburred_object_name,
                            joints_to_shrink=gen_params.joints_to_shrink,
                            joint_shrink_range=gen_params.joint_shrink_range,
                        )
                    )
                self._path_generators["insert_retract_tool"]["generator"] = (
                    GraspPathGenerator(
                        robot_model=self._robot_model,
                        ocp_dt=self._params.ocp_dt,
                        tool_frame_id=self._tool_frame_id_name,
                        max_linear_vel=self._params.generators_params.grasp_generator.max_linear_vel,
                    )
                )
                self._path_generators["deburring_motion"]["generator"] = (
                    DeburringPathGenerator(
                        robot_model=self._robot_model,
                        ocp_dt=self._params.ocp_dt,
                        tool_frame_id=self._tool_frame_id_name,
                        measurement_frame_id=self._params.measurement_frame_id,
                        desired_force=self._params.desired_force,
                        angle=self._params.generators_params.deburring_generator.angle,
                        frequency=self._params.generators_params.deburring_generator.frequency,
                        slope_circles=self._params.generators_params.deburring_generator.slope_circles,
                        n_circles=self._params.generators_params.deburring_generator.n_circles,
                        force_ramp=self._params.force_ramp,
                    )
                )
                self.get_logger().info(
                    "Trajecotry generators initialized!",
                    throttle_duration_sec=5.0,
                )
                self._path_generators_initialized = True
            except Exception as e:
                # Log the exception as an error and forward it
                self.get_logger().error(
                    f"Failed to initialize generators! Reason: {str(e)}",
                )
                raise e

        try:
            T_msg = self._tf_buffer.lookup_transform(
                self._params.world_frame_id,
                self._params.deburred_object_frame_id,
                rclpy.time.Time(),
            ).transform
            T_pylone = pin.SE3(
                pin.Quaternion(np.array([getattr(T_msg.rotation, v) for v in "xyzw"])),
                np.array([getattr(T_msg.translation, v) for v in "xyz"]),
            )
            self._path_generators["follow_joint_trajectory"][
                "generator"
            ].update_deburred_object_pose(T_pylone)
        except TransformException:
            self.get_logger().info(
                "Waiting for transformation between "
                f"'{self._params.world_frame_id}' and '{self._params.pylone_frame_id}'...",
                throttle_duration_sec=5.0,
            )
            return

        if self._target_handle_name is None:
            self.get_logger().info(
                f"Waiting for goal message on topic '{self._target_handle_sub.topic}'...",
                throttle_duration_sec=5.0,
            )
            return

        try:
            target_handle = T_pylone * self._handles[self._target_handle_name]

            robot_configuration = self._get_remapped_joints()

            trajectory_filename = None
            compute_new_trajectory = True
            if self._use_precomputed_trajectories:
                trajectory_filename = (
                    self._trajectories_folder
                    / f"{self._last_handle}_to_{self._target_handle_name}.pickle"
                )
                compute_new_trajectory = (
                    self._last_handle == "none" or not trajectory_filename.is_file()
                )

            if compute_new_trajectory:
                trajectory = self._path_generators["follow_joint_trajectory"][
                    "generator"
                ].get_path(robot_configuration, target_handle, self._target_handle_name)
                if self._use_precomputed_trajectories and self._last_handle != "none":
                    self.get_logger().info(
                        f"Saving new trajectory: {trajectory_filename}"
                    )
                    with open(trajectory_filename, "wb") as handle:
                        pickle.dump(
                            trajectory, handle, protocol=pickle.HIGHEST_PROTOCOL
                        )
            else:
                self.get_logger().info(
                    f"Loading trajectory from a file: {trajectory_filename}"
                )
                with open(trajectory_filename, "rb") as handle:
                    trajectory = pickle.load(handle)

            # Update joint configuration after planning
            robot_configuration = self._get_remapped_joints()
            # Interpolate joint configuration between where robot is and
            # where the trajectory starts to avoid abrupt motion
            q0 = trajectory[0].robot_configuration
            dt = self._params.ocp_dt
            # Compute scaling for each joint
            trajectory_vel = np.abs(q0 - robot_configuration)
            joint_lim = np.asarray(self._params.max_joint_velocity)
            velocity_scale = trajectory_vel / joint_lim
            # Compute minimum number of interpolation steps
            n_interp = np.ceil(velocity_scale / dt)
            # Multiply by two just to make sure it is slow enough
            n_interp = int(np.max(np.append(n_interp, 2))) * 2

            interpolated = np.zeros((n_interp, self._nq))
            # Perform linear interpolation between configurations
            for i in range(self._nq):
                interpolated[:, i] = np.linspace(
                    robot_configuration[i], q0[i], n_interp
                )

            interpolated = [
                self._create_trajectory_point(i, interpolated[i, :])
                for i in range(n_interp)
            ]

            trajectory = interpolated + trajectory

            weighted_trajectory = [
                WeightedTrajectoryPoint(
                    point=p,
                    weights=copy.copy(
                        self._path_generators["follow_joint_trajectory"]["weights"]
                    ),
                )
                for p in trajectory
            ]

            self._trajectory_buffer.clear()
            self._trajectory_buffer.extend(weighted_trajectory)

            q = self._trajectory_buffer[-1].point.robot_configuration
            T_pregrasp = self._trajectory_buffer[-1].point.end_effector_poses[
                self._tool_frame_id_name
            ]

            # Insert into the hole
            self._insert_sequence_to_buffer(
                "insert_retract_tool", q, target_handle * self._R_insert, T_pregrasp
            )
            # Perform deburring
            q = self._trajectory_buffer[-1].point.robot_configuration
            self._insert_sequence_to_buffer(
                "deburring_motion", q, target_handle * self._R_insert, T_pregrasp
            )
            # Retract from the hole
            q = self._trajectory_buffer[-1].point.robot_configuration
            self._insert_sequence_to_buffer(
                "insert_retract_tool", q, T_pregrasp, target_handle * self._R_insert
            )

            # Publish visualization for the path
            if self._params.visualization.publish_path:
                now = self.get_clock().now().to_msg()

                # Keep the markers avlive for the time of validity of the path
                lifetime = len(self._trajectory_buffer) * self._params.ocp_dt * 1.25

                def _crate_marker(id: int, point: TrajectoryPoint) -> Marker:
                    pose = pin.SE3ToXYZQUAT(
                        point.end_effector_poses[self._tool_frame_id_name]
                    )
                    marker = copy.deepcopy(self._marker_base)
                    marker.id = id
                    marker.header.stamp = now
                    marker.pose = Pose(
                        position=Point(**dict(zip("xyz", pose[:3]))),
                        orientation=Quaternion(**dict(zip("xyzw", pose[3:]))),
                    )
                    marker.lifetime = Duration(seconds=lifetime).to_msg()
                    return marker

                self._path_publisher.publish(
                    MarkerArray(
                        markers=[
                            _crate_marker(i, point.point)
                            for i, point in enumerate(self._trajectory_buffer)
                        ],
                    )
                )
        except Exception as e:
            self.get_logger().error(
                f"Failed to plan a trajectory to a handle '{self._target_handle_name}'. "
                f"Reason: {str(e)}"
            )
        self._last_handle = copy.copy(self._target_handle_name)
        self._target_handle_name = None

    def _publish_mpc_input_cb(self) -> None:
        # Skip if buffer is empty
        if not len(self._trajectory_buffer):
            return

        if self._buffer_size >= self._params.ocp_buffer_size:
            return

        def _get_traj_point() -> WeightedTrajectoryPoint:
            if len(self._trajectory_buffer) == 1:
                return self._trajectory_buffer[0]
            else:
                return self._trajectory_buffer.popleft()

        n_points = self._params.ocp_buffer_size - self._buffer_size
        mpc_input_array = MpcInputArray(
            inputs=[
                weighted_traj_point_to_mpc_msg(_get_traj_point())
                for _ in range(n_points)
            ]
        )
        if self._params.visualization.publish_path:
            self._mpc_target_pose.publish(
                PoseStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="fer_link0",
                    ),
                    pose=mpc_input_array.inputs[-1].ee_inputs[0].pose,
                )
            )

        self._mpc_input_pub.publish(mpc_input_array)


def main(args=None):
    rclpy.init()
    try:
        diffusion_warmstart_node = DeburringPathPlanner()
        rclpy.spin(diffusion_warmstart_node)
        diffusion_warmstart_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
