import copy
import xml.etree.ElementTree as ET
from collections import deque
import threading
import time

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header, Int64, String
from visualization_msgs.msg import Marker, MarkerArray

from agimus_msgs.action import DeburringPlanner

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from agimus_demo_07_deburring.deburring_path_planner_parameters import (
    deburring_path_planner,
)
from agimus_demo_07_deburring.planner.trajectory_generators.plastic_deburring_trajectory import (
    PlasticDeburringPathGenerator,
)
from agimus_demo_07_deburring.planner.trajectory_generators.metal_deburring_trajectory import (
    MetalDeburringPathGenerator,
)
from agimus_demo_07_deburring.planner.trajectory_generators.grasp_trajectory import (
    GraspPathGenerator,
)

from agimus_demo_07_deburring.planner.trajecory_smoothers.basic_interpolation import (
    BasicInterpolationSmoother,
)
from agimus_demo_07_deburring.planner.trajecory_smoothers.ocp_smoother import (
    OCPSmoother,
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
        self._robot_description: str | None = None
        self._robot_model_sc: pin.Model | None = None
        self._robot_data_sc: pin.Data | None = None
        self._robot_description_sc: str | None = None
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
        self._environment_description: str | None = None
        self._environment_description_sc: str | None = None
        self._T_pylone: pin.SE3 | None = None
        self._buffer_len: int | None = None
        self._last_handle = "none"
        # Rotation used as a conversion between TCP coordinates and HPP handles
        self._R_insert = pin.SE3(
            pin.rpy.rpyToMatrix(np.array(self._params.hpp_handle_to_ee_rot)),
            np.array(self._params.tool_offset_correction),
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
            clearance = float(handle.get("clearance"))
            T_pregrasp = pin.SE3(np.eye(3), np.array([-clearance, 0.0, 0.0]))

            pose = pin.XYZQUATToSE3(
                np.concatenate(
                    (
                        np.fromstring(pos.get("xyz"), count=3, sep=" "),
                        np.fromstring(pos.get("xyzw"), count=4, sep=" "),
                    ),
                )
            )
            self._handles[handle.get("name")] = {}
            self._handles[handle.get("name")]["pose"] = pose
            self._handles[handle.get("name")]["pregrasp"] = pose * T_pregrasp

        self._node_initialized = False
        self._waiting_for_planner = False
        self._trajectory_buffer_lock = threading.Lock()
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

        self._robot_description_sc_sub = self.create_subscription(
            String,
            "/robot_description_with_collision",
            self._robot_description_sc_cb,
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
        self._environment_description_sc_sub = self.create_subscription(
            String,
            "/environment_description_with_collision",
            self._environment_description_sc_cb,
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

        # Actions
        self._action_server = ActionServer(
            self,
            DeburringPlanner,
            "~/plan_deburring",
            execute_callback=self._planner_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create a timer that calls the _maybe_run_prediction method every 2 seconds
        self._initialization_watchdog = self.create_timer(
            1.0 / self._params.rate, self._initialization_watchdog_cb
        )

        self._mpc_input_publisher_timer = self.create_timer(
            self._params.ocp_dt * 10.0, self._publish_mpc_input_cb
        )

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

    def _robot_description_sc_cb(self, msg: String) -> None:
        self._robot_sc_description = msg.data
        self._robot_sc_model = pin.buildModelFromXML(self._robot_description)
        self._robot_sc_data = self._robot_model.createData()

    def _environment_description_cb(self, msg: String) -> None:
        self._environment_description = msg.data

    def _environment_description_sc_cb(self, msg: String) -> None:
        self._environment_description_sc = msg.data

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_states = msg

    def _buffer_size_cb(self, msg: Int64) -> None:
        self._buffer_size = msg.data

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
    ) -> int:
        generator = self._path_generators[generator_name]
        with self._trajectory_buffer_lock:
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
        trajectory_len = len(weighted_trajectory)
        with self._trajectory_buffer_lock:
            self._trajectory_buffer.extend(weighted_trajectory)

        # Compute and insert actual trajectory
        trajectory = generator["generator"].get_path(q, T_taget, T_init=T_init)
        weighted_trajectory = [
            WeightedTrajectoryPoint(point=p, weights=copy.copy(w2)) for p in trajectory
        ]
        with self._trajectory_buffer_lock:
            self._trajectory_buffer.extend(weighted_trajectory)
        trajectory_len += len(weighted_trajectory)
        return trajectory_len

    def _initialization_watchdog_cb(self) -> None:
        self._update_params()

        for topic_name, object in (
            (self._robot_description_sub.topic, self._robot_model),
            (self._environment_description_sub.topic, self._environment_description),
            (self._robot_description_sc_sub.topic, self._robot_sc_model),
            (
                self._environment_description_sc_sub.topic,
                self._environment_description_sc,
            ),
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
                trajecory_smoothers = None
                interpolation_smoother = BasicInterpolationSmoother(
                    robot_model=self._robot_model,
                    ocp_dt=self._params.ocp_dt,
                    max_joint_velocity=np.array(self._params.max_joint_velocity),
                )
                if self._params.generators_params.trajectory_smoother == "interpolate":
                    trajecory_smoothers = interpolation_smoother
                elif self._params.generators_params.trajectory_smoother == "ocp":
                    smth_params = self._params.generators_params.ocp_smoother
                    trajecory_smoothers = OCPSmoother(
                        self._robot_model,
                        self._robot_description_sc,
                        self._environment_description_sc,
                        interpolation_smoother,
                        smth_params.optimizer_ocp_dt,
                        smth_params.optimizer_ocp_horizon,
                        smth_params.running_costs,
                        smth_params.terminal_costs,
                        smth_params.robot_collision_links,
                        smth_params.environment_links,
                        smth_params.collision_distance,
                        smth_params.joint_shrink_range,
                        smth_params.n_threads,
                        smth_params.solver_iters,
                        smth_params.qp_iters,
                        smth_params.use_line_search,
                        smth_params.callbacks,
                        self._tool_frame_id_name,
                        smth_params.w_robot_velocity,
                        smth_params.w_robot_effort,
                        smth_params.running_w_robot_configuration,
                        smth_params.running_w_frame_rotation,
                        smth_params.running_w_frame_translation,
                        smth_params.terminal_w_robot_configuration,
                        smth_params.terminal_w_frame_rotation,
                        smth_params.terminal_w_frame_translation,
                        self._params.moving_joints,
                    )

                if self._params.generators_params.main_generator_type == "diffusion":
                    from agimus_demo_07_deburring.planner.trajectory_generators.diffusion_trajectory import (
                        DiffusionPathGenerator,
                    )

                    gen_params = self._params.generators_params.diffusion_generator
                    self._path_generators["follow_joint_trajectory"]["generator"] = (
                        DiffusionPathGenerator(
                            wights_path=gen_params.weights_path,
                            sequence_length=gen_params.sequence_length,
                            n_samples=gen_params.n_samples,
                            robot_model=self._robot_model,
                            ocp_dt=self._params.ocp_dt,
                            tool_frame_id=self._tool_frame_id_name,
                            hpp_handle_correction=self._R_insert,
                            trajecory_smoothers=trajecory_smoothers,
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
                            trajecory_smoothers=trajecory_smoothers,
                        )
                    )
                gen_params = self._params.generators_params.grasp_generator
                self._path_generators["insert_retract_tool"]["generator"] = (
                    GraspPathGenerator(
                        robot_model=self._robot_model,
                        ocp_dt=self._params.ocp_dt,
                        tool_frame_id=self._tool_frame_id_name,
                        linear_vel=gen_params.linear_vel,
                        linear_acc=gen_params.linear_acc,
                        linear_jerk=gen_params.linear_jerk,
                        tool_angular_vel=gen_params.tool_angular_vel,
                        tool_angular_acc=gen_params.tool_angular_acc,
                        tool_angular_jerk=gen_params.tool_angular_jerk,
                        insert_joint_angle=gen_params.insert_joint_angle,
                        retract_joint_angle=gen_params.retract_joint_angle,
                    )
                )
                if self._params.generators_params.deburring_generator_type == "plastic":
                    gen_params = (
                        self._params.generators_params.plastic_deburring_generator
                    )
                    self._path_generators["deburring_motion"]["generator"] = (
                        PlasticDeburringPathGenerator(
                            robot_model=self._robot_model,
                            ocp_dt=self._params.ocp_dt,
                            tool_frame_id=self._tool_frame_id_name,
                            measurement_frame_id=self._params.measurement_frame_id,
                            desired_force=gen_params.desired_force,
                            angle=gen_params.angle,
                            frequency=gen_params.frequency,
                            slope_circles=gen_params.slope_circles,
                            n_circles=gen_params.n_circles,
                            force_ramp=gen_params.force_ramp,
                        )
                    )
                else:
                    gen_params = (
                        self._params.generators_params.metal_deburring_generator
                    )
                    self._path_generators["deburring_motion"]["generator"] = (
                        MetalDeburringPathGenerator(
                            robot_model=self._robot_model,
                            ocp_dt=self._params.ocp_dt,
                            tool_frame_id=self._tool_frame_id_name,
                            measurement_frame_id=self._params.measurement_frame_id,
                            tool_angular_vel=gen_params.tool_angular_vel,
                            tool_angular_acc=gen_params.tool_angular_acc,
                            tool_angular_jerk=gen_params.tool_angular_jerk,
                            positioning_time=gen_params.positioning_time,
                            positioning_force=gen_params.positioning_force,
                            deburring_force=gen_params.deburring_force,
                            force_rate_up=gen_params.force_rate_up,
                            force_rate_down=gen_params.force_rate_down,
                            tool_joint_start_angle=gen_params.tool_joint_start_angle,
                            tool_joint_end_angle=gen_params.tool_joint_end_angle,
                            n_repeat=gen_params.n_repeat,
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
            self._T_pylone = pin.SE3(
                pin.Quaternion(np.array([getattr(T_msg.rotation, v) for v in "xyzw"])),
                np.array([getattr(T_msg.translation, v) for v in "xyz"]),
            )
            self._path_generators["follow_joint_trajectory"][
                "generator"
            ].update_deburred_object_pose(self._T_pylone)
        except TransformException:
            self.get_logger().info(
                "Waiting for transformation between "
                f"'{self._params.world_frame_id}' and '{self._params.deburred_object_frame_id}'...",
                throttle_duration_sec=5.0,
            )
            return

        self._node_initialized = True
        self.get_logger().info("Node initialized.", once=True)

        with self._trajectory_buffer_lock:
            if len(self._trajectory_buffer) <= 1 and not self._waiting_for_planner:
                self.get_logger().info(
                    "Waiting for goal to be set via action...",
                    throttle_duration_sec=5.0,
                )
                return

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not self._node_initialized:
            self.get_logger().error(
                "Unable to start planning. The node is not initialized yet!"
            )
            return GoalResponse.REJECT
        if self._waiting_for_planner:
            self.get_logger().error(
                "Unable to start planning. Waiting for the planner!"
            )
            return GoalResponse.REJECT
        with self._trajectory_buffer_lock:
            if len(self._trajectory_buffer) > 1:
                self.get_logger().error(
                    "Unable to accept new goals. Still execution previous one!"
                )
                return GoalResponse.REJECT
        if goal_request.handle_name not in self._handles.keys():
            self.get_logger().error(f"Invalid handle name '{goal_request.handle_name}'")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _) -> CancelResponse:
        self.get_logger().info("Trajectory execution was canceled.")
        with self._trajectory_buffer_lock:
            self._trajectory_buffer.clear()
        return CancelResponse.ACCEPT

    def _planner_cb(self, goal_handle) -> DeburringPlanner.Result:
        self._update_params()

        handle_name = goal_handle.request.handle_name

        self.get_logger().info(f"Received new goal handle point: {handle_name}")

        feedback_msg = DeburringPlanner.Feedback()
        feedback_msg.progress = 0.0
        feedback_msg.planing = True
        goal_handle.publish_feedback(feedback_msg)

        initial_trajectory_len = None
        self._waiting_for_planner = True
        try:
            with self._trajectory_buffer_lock:
                has_elements = len(self._trajectory_buffer) >= 1
                if has_elements:
                    wp = copy.deepcopy(self._trajectory_buffer[0])
                    self._trajectory_buffer.clear()
                    self._trajectory_buffer.append(wp)
                else:
                    self._trajectory_buffer.clear()

            T_handle = self._T_pylone * self._handles[handle_name]["pose"]
            T_pregrasp = self._T_pylone * self._handles[handle_name]["pregrasp"]
            initial_trajectory_len = 0

            if goal_handle.request.do_plan:
                robot_configuration = self._get_remapped_joints()

                trajectory_filename = None
                compute_new_trajectory = True
                if self._use_precomputed_trajectories:
                    trajectory_filename = (
                        self._trajectories_folder
                        / f"{self._last_handle}_to_{handle_name}.pickle"
                    )
                    compute_new_trajectory = (
                        self._last_handle == "none" or not trajectory_filename.is_file()
                    )

                if compute_new_trajectory:
                    trajectory = self._path_generators["follow_joint_trajectory"][
                        "generator"
                    ].get_path(robot_configuration, T_pregrasp, handle_name)
                    if (
                        self._use_precomputed_trajectories
                        and self._last_handle != "none"
                    ):
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
                n_interp = int(np.max(np.append(n_interp, 2)) * 2.5)

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

                with self._trajectory_buffer_lock:
                    self._trajectory_buffer.extend(weighted_trajectory)
                    initial_trajectory_len += len(weighted_trajectory)

            if goal_handle.request.do_insertion:
                if goal_handle.request.do_plan:
                    with self._trajectory_buffer_lock:
                        q = self._trajectory_buffer[-1].point.robot_configuration
                else:
                    q = self._get_remapped_joints()
                T_pregrasp_rot = T_pregrasp * self._R_insert
                T_handle_rot = T_handle * self._R_insert

                # Insert into the hole
                self._path_generators["insert_retract_tool"][
                    "generator"
                ].set_insert_mode()
                segment_len = self._insert_sequence_to_buffer(
                    "insert_retract_tool", q, T_handle_rot, T_pregrasp_rot
                )
                initial_trajectory_len += segment_len
                # Perform deburring
                with self._trajectory_buffer_lock:
                    q = self._trajectory_buffer[-1].point.robot_configuration
                segment_len = self._insert_sequence_to_buffer(
                    "deburring_motion", q, T_handle_rot, T_pregrasp_rot
                )
                initial_trajectory_len += segment_len
                # Retract from the hole
                self._path_generators["insert_retract_tool"][
                    "generator"
                ].set_retract_mode()
                with self._trajectory_buffer_lock:
                    q = self._trajectory_buffer[-1].point.robot_configuration
                segment_len = self._insert_sequence_to_buffer(
                    "insert_retract_tool", q, T_pregrasp_rot, T_handle_rot
                )
                initial_trajectory_len += segment_len

            feedback_msg.planing = False
            goal_handle.publish_feedback(feedback_msg)

            # Publish visualization for the path
            if self._params.visualization.publish_path:
                now = self.get_clock().now().to_msg()

                # Keep the markers avlive for the time of validity of the path
                with self._trajectory_buffer_lock:
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

                with self._trajectory_buffer_lock:
                    self._path_publisher.publish(
                        MarkerArray(
                            markers=[
                                _crate_marker(i, point.point)
                                for i, point in enumerate(self._trajectory_buffer)
                            ],
                        )
                    )
        except Exception as e:
            self._waiting_for_planner = False
            with self._trajectory_buffer_lock:
                self._trajectory_buffer.clear()
            error_msg = (
                "Failed to plan a trajectory to a handle "
                + f"'{handle_name}'. Reason: {str(e)}"
            )
            self.get_logger().error(error_msg)
            result = DeburringPlanner.Result()
            result.error_msg = error_msg
            return result

        self._waiting_for_planner = False
        buffer_len = len(self._trajectory_buffer)
        while rclpy.ok() and buffer_len > 1:
            feedback_msg.progress = 1.0 - (
                len(self._trajectory_buffer) / initial_trajectory_len
            )
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
            with self._trajectory_buffer_lock:
                buffer_len = len(self._trajectory_buffer)

        self._last_handle = handle_name

        goal_handle.succeed()
        return DeburringPlanner.Result()

    def _publish_mpc_input_cb(self) -> None:
        # Skip if buffer is empty
        with self._trajectory_buffer_lock:
            self._buffer_len = len(self._trajectory_buffer)
        if not self._buffer_len:
            return

        if self._buffer_size >= self._params.ocp_buffer_size:
            return

        def _get_traj_point() -> WeightedTrajectoryPoint:
            if len(self._trajectory_buffer) == 1:
                return self._trajectory_buffer[0]
            else:
                return self._trajectory_buffer.popleft()

        n_points = self._params.ocp_buffer_size - self._buffer_size
        with self._trajectory_buffer_lock:
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
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(diffusion_warmstart_node)
        executor.spin()
        diffusion_warmstart_node.destroy_node()
    except (KeyboardInterrupt, ParameterException, ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
