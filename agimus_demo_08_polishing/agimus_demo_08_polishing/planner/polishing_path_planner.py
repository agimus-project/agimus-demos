#!/usr/bin/env python

import copy
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

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

from agimus_demo_08_polishing.polishing_path_planner_parameters import (
    polishing_path_planner,
)
from agimus_demo_08_polishing.planner.trajectory_generators.polishing_trajectory import (
    PolishingPathGenerator,
)

# from agimus_demo_08_polishing.planner.trajectory_generators.diffusion_trajectory import (
#     DiffusionPathGenerator,
# )
from agimus_demo_08_polishing.planner.trajectory_generators.grasp_trajectory import (
    GraspPathGenerator,
)
from agimus_demo_08_polishing.planner.trajectory_generators.hpp_trajectory import (
    HPPPathGenerator,
)
from agimus_demo_08_polishing.planner.trajectory_generators.mesh_polishing_trajectory import (
    MeshPolishingPathGenerator,
)


def generate_sphere_circle_frames(
    *,
    sphere_center_xyz: tuple[float, float, float],
    sphere_radius: float,
    circle_center_xy_in_world: tuple[float, float],
    circle_radius: float,
    samples: int,
    upper_hemisphere: bool,
    outward_normal: bool = True,
) -> list[pin.SE3]:
    """
    Build EE frames along the intersection of:
      - a vertical cylinder whose base is a circle in world XY, and
      - a sphere (center, radius).

    Constraints:
      - EE z-axis is aligned with sphere normal at each point
      - EE x,y span the tangent plane; x = projection of world X onto the tangent

    All frames are expressed in the world frame.
    """
    x_s, y_s, z_s = sphere_center_xyz
    x_c, y_c = circle_center_xy_in_world
    R = float(sphere_radius)
    r = float(circle_radius)

    max_xy_dist = np.hypot(x_c - x_s, y_c - y_s) + r
    if max_xy_dist > R + 1e-9:
        raise ValueError(
            f"XY circle exceeds sphere radius: max_xy_dist={max_xy_dist:.6f} > R={R:.6f}"
        )

    if samples <= 0:
        raise ValueError("Expected strictly positive samples for spherical path.")

    frames: list[pin.SE3] = []
    world_x = np.array([1.0, 0.0, 0.0])

    for k in range(samples):
        theta = 2.0 * np.pi * k / samples

        # XY circle
        x = x_c + r * np.cos(theta)
        y = y_c + r * np.sin(theta)
        dx = x - x_s
        dy = y - y_s

        h2 = R * R - dx * dx - dy * dy
        if h2 < 0.0:
            if h2 > -1e-9:
                h2 = 0.0
            else:
                raise RuntimeError("Computed point lies outside the sphere")
        h = np.sqrt(h2)
        z = z_s + (h if upper_hemisphere else -h)

        p = np.array([x, y, z])

        # Surface normal at p (center -> point)
        z_axis = p - np.array([x_s, y_s, z_s])
        nz = np.linalg.norm(z_axis)
        if nz < 1e-12:
            raise RuntimeError("Degenerate normal on sphere")
        z_axis = z_axis / nz
        if not outward_normal:
            z_axis = -z_axis

        # Tangent x-axis = projection of world X onto tangent plane
        t = world_x - (world_x @ z_axis) * z_axis
        nt = np.linalg.norm(t)
        if nt < 1e-12:
            world_y = np.array([0.0, 1.0, 0.0])
            t = world_y - (world_y @ z_axis) * z_axis
            nt = np.linalg.norm(t)
            if nt < 1e-12:
                raise RuntimeError("Failed to construct tangent frame")
        x_axis = t / nt
        y_axis = np.cross(z_axis, x_axis)
        ny = np.linalg.norm(y_axis)
        if ny < 1e-12:
            raise RuntimeError("Degenerate tangent frame")
        y_axis = y_axis / ny

        Rmat = np.column_stack([x_axis, y_axis, z_axis])
        frames.append(pin.SE3(Rmat, p))

    return frames


class PolishingPathPlanner(Node):
    def __init__(self):
        super().__init__("polishing_path_planner")

        try:
            self._param_listener = polishing_path_planner.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        self._robot_model: pin.Model | None = None
        self._robot_data: pin.Data | None = None
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

        self._fixed_insert_distance = 0.025
        self._path_generators = {
            name: {"generator": None, "weights": None}
            for name in (
                "follow_joint_trajectory",
                "insert_retract_tool",
                "polishing_motion",
            )
        }

        # Mesh generator specific
        self._mesh_generator: MeshPolishingPathGenerator | None = None
        self._last_mesh_params = None

        tree = ET.parse(self._params.handles_srdf_path)
        root = tree.getroot()
        self._handles = {}
        for handle in root.findall("handle"):
            pos = handle.find("position")
            xyz_str = pos.get("xyz").replace(",", " ")
            xyzw_str = pos.get("xyzw").replace(",", " ")
            self._handles[handle.get("name")] = pin.XYZQUATToSE3(
                np.concatenate(
                    (
                        np.fromstring(xyz_str, count=3, sep=" "),
                        np.fromstring(xyzw_str, count=4, sep=" "),
                    ),
                )
            )

        self._static_handles = set(self._handles.keys())
        self._handle_aliases: dict[str, str] = {"camera_target": "polish1"}
        self._dynamic_contacts: dict[str, pin.SE3] = {}
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
        self._mesh_publisher = self.create_publisher(
            Marker,
            "/polishing_mesh",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
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
                            self._params.tool_frame_id: np.concatenate(
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

    def _environment_description_cb(self, msg: String) -> None:
        self._environment_description = msg.data

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_states = msg

    def _buffer_size_cb(self, msg: Int64) -> None:
        self._buffer_size = msg.data

    def _target_handle_cb(self, msg: String) -> None:
        handle_name = msg.data.strip().strip("\"'")
        if not handle_name:
            self.get_logger().error("Received empty handle name")
            return
        if (
            handle_name not in self._handles
            and handle_name not in self._handle_aliases.keys()
        ):
            self.get_logger().error(f"Invalid handle name '{handle_name}'")
            return

        self._target_handle_name = handle_name
        self.get_logger().info(f"Received new goal handle point: {handle_name}")

    def _insert_sequence_to_buffer(
        self, generator_name: str, q: npt.ArrayLike, T_target: pin.SE3, T_init: pin.SE3
    ) -> None:
        generator = self._path_generators[generator_name]
        wp = self._trajectory_buffer[-1]
        point = wp.point
        w1 = wp.weights
        w2 = generator["weights"]

        dt = self._params.ocp_dt
        interpolation_time = max(self._params.weights.schedule.interpolation_time, 0.0)
        gain_schedule_points = max(int(np.round(interpolation_time / dt)), 0)
        for step in range(1, gain_schedule_points + 1):
            t = min(step * dt, interpolation_time)
            scheduled_point = WeightedTrajectoryPoint(
                point=copy.copy(point), weights=interpolate_weights(w1, w2, t)
            )
            self._trajectory_buffer.append(scheduled_point)

        trajectory = generator["generator"].get_path(q, T_target, T_init=T_init)
        weighted_trajectory = [
            WeightedTrajectoryPoint(point=p, weights=copy.copy(w2)) for p in trajectory
        ]
        self._trajectory_buffer.extend(weighted_trajectory)

    def _compute_insert_goal(self, T_pregrasp: pin.SE3, T_contact: pin.SE3) -> pin.SE3:
        """Move a fixed distance along the contact +z axis starting from the pregrasp pose."""
        contact_rot = np.array(T_contact.rotation)
        contact_z = contact_rot[:, 2]
        goal_translation = (
            T_pregrasp.translation + contact_z * self._fixed_insert_distance
        )
        return pin.SE3(contact_rot, np.array(goal_translation).flatten())

    def _compute_pregrasp_from_contact(self, T_contact: pin.SE3) -> pin.SE3:
        eef_rot = np.array(T_contact.rotation)
        eef_z = eef_rot[:, 2]
        pre_translation = T_contact.translation - eef_z * self._fixed_insert_distance
        return pin.SE3(eef_rot, np.array(pre_translation).flatten())

    def _append_spherical_polishing_sequence(self, T_pregrasp: pin.SE3) -> None:
        """
        Replace planar polishing with a spherical path.

        Uses:
          - same insert/retract generators
          - polishing_motion weights only (no trajectory generator)
        """
        sp = self._params.sphere_path
        samples = sp.samples

        target_velocity = getattr(self._params, "sphere_path_velocity", 0.0)
        if target_velocity and target_velocity > 0.0:
            circumference = 2.0 * np.pi * float(sp.circle_radius)
            dt = max(float(self._params.ocp_dt), 1e-9)
            required_samples = max(
                1, int(np.ceil((circumference / target_velocity) / dt))
            )
            samples = max(samples, required_samples)

        base_frames = generate_sphere_circle_frames(
            sphere_center_xyz=tuple(sp.center_xyz),
            sphere_radius=sp.radius,
            circle_center_xy_in_world=tuple(sp.circle_center_xy),
            circle_radius=sp.circle_radius,
            samples=samples,
            upper_hemisphere=sp.upper_hemisphere,
            outward_normal=sp.outward_normal,
        )

        loops_param = getattr(self._params, "sphere_path_loops", 1)
        try:
            loops = int(loops_param)
        except (TypeError, ValueError):
            loops = 1
        loops = max(1, loops)

        if loops > 1:
            frames: list[pin.SE3] = []
            for _ in range(loops):
                frames.extend(pin.SE3(f) for f in base_frames)
        else:
            frames = base_frames

        if not frames:
            raise RuntimeError("Spherical path generation returned no frames.")

        first_contact = frames[0]

        # Insert directly onto the first spherical target pose.
        q = self._trajectory_buffer[-1].point.robot_configuration
        insert_generator = self._path_generators["insert_retract_tool"]["generator"]
        entry_velocity = getattr(self._params, "sphere_entry_velocity", 0.0)
        original_velocity: float | None = None
        if isinstance(entry_velocity, (float, int)) and entry_velocity > 0.0:
            try:
                original_velocity = insert_generator.max_linear_vel
                insert_generator.max_linear_vel = float(entry_velocity)
            except AttributeError:
                original_velocity = None

        try:
            self._insert_sequence_to_buffer(
                "insert_retract_tool", q, first_contact, T_pregrasp
            )
        finally:
            if original_velocity is not None:
                insert_generator.max_linear_vel = original_velocity

        last_wp = self._trajectory_buffer[-1]
        base_point = last_wp.point
        base_weights = self._path_generators["polishing_motion"]["weights"]
        next_id = (base_point.id or 0) + 1
        base_configuration = np.array(base_point.robot_configuration)
        base_velocity = np.array(base_point.robot_velocity)
        base_acceleration = np.array(base_point.robot_acceleration)
        base_effort = np.array(base_point.robot_effort)
        desired_force = float(self._params.desired_force)
        force_local = None
        if desired_force > 0.0:
            force_vec = np.zeros(6)
            force_vec[2] = -desired_force
            force_local = pin.Force(force_vec)

        for idx, T in enumerate(frames):
            forces = dict(base_point.forces)
            if force_local is not None:
                forces[self._params.measurement_frame_id] = T * force_local
            poses = dict(base_point.end_effector_poses)
            poses[self._params.tool_frame_id] = T
            point = TrajectoryPoint(
                id=next_id + idx,
                time_ns=base_point.time_ns,
                robot_configuration=np.copy(base_configuration),
                robot_velocity=np.copy(base_velocity),
                robot_acceleration=np.copy(base_acceleration),
                robot_effort=np.copy(base_effort),
                forces=forces,
                end_effector_poses=poses,
            )
            self._trajectory_buffer.append(
                WeightedTrajectoryPoint(
                    point=point,
                    weights=copy.copy(base_weights),
                )
            )

        # Retract back to pregrasp
        T_inserted = self._trajectory_buffer[-1].point.end_effector_poses[
            self._params.tool_frame_id
        ]
        q = self._trajectory_buffer[-1].point.robot_configuration
        self._insert_sequence_to_buffer(
            "insert_retract_tool", q, T_pregrasp, T_inserted
        )

    def _get_or_update_mesh_generator(self) -> MeshPolishingPathGenerator:
        pg = self._params.generators_params.polishing_generator
        mp = self._params.mesh_path
        mesh_translation_xyz = np.array(
            getattr(mp, "mesh_translation_xyz", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        mesh_position_shift_xyz = np.array(
            getattr(mp, "mesh_position_shift_xyz", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        effective_mesh_translation_xyz = list(
            mesh_translation_xyz + mesh_position_shift_xyz
        )
        mesh_shift_xyz = list(getattr(mp, "mesh_shift_xyz", [0.0, 0.0, 0.0]))
        mesh_refine_enabled = bool(getattr(mp, "mesh_refine_enabled", True))
        mesh_upsample_enabled = bool(getattr(mp, "mesh_upsample_enabled", True))
        mesh_refine_iterations = int(getattr(mp, "mesh_refine_iterations", 5))
        mesh_refine_lambda = float(getattr(mp, "mesh_refine_lambda", 0.5))
        mesh_trajectory_mode = str(getattr(mp, "trajectory_mode", "circle")).lower()
        mesh_line_axis = str(getattr(mp, "line_axis", "x")).lower()
        mesh_line_length = float(getattr(mp, "line_length", 0.1))
        mesh_line_speed = float(getattr(mp, "line_speed", 0.01))
        mesh_line_speed_mode = str(getattr(mp, "line_speed_mode", "triangular")).lower()
        mesh_sequence_axes = [str(axis) for axis in getattr(mp, "sequence_axes", [])]
        mesh_sequence_lengths = [
            float(length) for length in getattr(mp, "sequence_lengths", [])
        ]
        mesh_circle_point_downsample_factor = int(
            getattr(mp, "circle_point_downsample_factor", 1)
        )
        mesh_rotation_z = float(getattr(mp, "mesh_rotation_z", 0.0))
        mesh_outward_normal = bool(getattr(mp, "outward_normal", False))

        current_params = {
            "ocp_dt": self._params.ocp_dt,
            "desired_force": self._params.desired_force,
            "force_ramp": self._params.force_ramp,
            "angle": mp.circle_radius,  # Using radius from mesh_path
            "frequency": pg.frequency,
            "slope_circles": pg.slope_circles,
            "n_circles": pg.n_circles,
            "ply_file_path": mp.ply_file,
            "ply_center_point": tuple(mp.ply_center_xy),
            "robot_center_point": tuple(mp.robot_center_xyz),
            "mesh_translation_xyz": tuple(mesh_translation_xyz),
            "mesh_position_shift_xyz": tuple(mesh_position_shift_xyz),
            "effective_mesh_translation_xyz": tuple(effective_mesh_translation_xyz),
            "mesh_shift_xyz": tuple(mesh_shift_xyz),
            "mesh_refine_enabled": mesh_refine_enabled,
            "mesh_upsample_enabled": mesh_upsample_enabled,
            "mesh_refine_iterations": mesh_refine_iterations,
            "mesh_refine_lambda": mesh_refine_lambda,
            "mesh_trajectory_mode": mesh_trajectory_mode,
            "mesh_line_axis": mesh_line_axis,
            "mesh_line_length": mesh_line_length,
            "mesh_line_speed": mesh_line_speed,
            "mesh_line_speed_mode": mesh_line_speed_mode,
            "mesh_sequence_axes": tuple(mesh_sequence_axes),
            "mesh_sequence_lengths": tuple(mesh_sequence_lengths),
            "mesh_circle_point_downsample_factor": mesh_circle_point_downsample_factor,
            "mesh_rotation_z": mesh_rotation_z,
            "mesh_outward_normal": mesh_outward_normal,
        }

        if self._mesh_generator is None or self._last_mesh_params != current_params:
            self.get_logger().info(
                f"Initializing MeshPolishingPathGenerator for: {mp.ply_file}"
            )
            self._mesh_generator = MeshPolishingPathGenerator(
                robot_model=self._robot_model,
                ocp_dt=self._params.ocp_dt,
                tool_frame_id=self._params.tool_frame_id,
                measurement_frame_id=self._params.measurement_frame_id,
                desired_force=self._params.desired_force,
                force_ramp=self._params.force_ramp,
                angle=mp.circle_radius,
                frequency=pg.frequency,
                slope_circles=pg.slope_circles,
                n_circles=pg.n_circles,
                circle_point_downsample_factor=mesh_circle_point_downsample_factor,
                ply_file_path=mp.ply_file,
                ply_center_point=list(mp.ply_center_xy),
                robot_center_point=list(mp.robot_center_xyz),
                mesh_translation_xyz=effective_mesh_translation_xyz,
                mesh_shift_xyz=mesh_shift_xyz,
                mesh_refine_enabled=mesh_refine_enabled,
                mesh_upsample_enabled=mesh_upsample_enabled,
                mesh_refine_iterations=mesh_refine_iterations,
                mesh_refine_lambda=mesh_refine_lambda,
                trajectory_mode=mesh_trajectory_mode,
                line_axis=mesh_line_axis,
                line_length=mesh_line_length,
                line_speed=mesh_line_speed,
                line_speed_mode=mesh_line_speed_mode,
                sequence_axes=mesh_sequence_axes,
                sequence_lengths=mesh_sequence_lengths,
                mesh_rotation_z=mesh_rotation_z,
                outward_normal=mesh_outward_normal,
            )
            self._last_mesh_params = current_params

        return self._mesh_generator

    def _append_mesh_polishing_sequence(self, T_pregrasp: pin.SE3) -> None:
        if not self._params.mesh_path.ply_file:
            self.get_logger().error("Mesh mode selected but no PLY file provided!")
            return

        generator = self._get_or_update_mesh_generator()

        q = self._trajectory_buffer[-1].point.robot_configuration

        # Generate mesh path starting from entry ramp
        # T_pregrasp is passed but ignored for path shape generation
        mesh_traj = generator.get_path(q, T_pregrasp)

        if not mesh_traj:
            self.get_logger().error("Mesh generator returned empty path")
            return

        # 1. Approach the start of the mesh trajectory
        T_start_polishing = mesh_traj[0].end_effector_poses[self._params.tool_frame_id]

        self._insert_sequence_to_buffer(
            "insert_retract_tool", q, T_start_polishing, T_pregrasp
        )

        # 2. Append the mesh polishing trajectory
        w2 = self._path_generators["polishing_motion"]["weights"]
        wp = self._trajectory_buffer[-1]
        point = wp.point
        w1 = wp.weights

        dt = self._params.ocp_dt
        interpolation_time = max(self._params.weights.schedule.interpolation_time, 0.0)
        gain_schedule_points = max(int(np.round(interpolation_time / dt)), 0)

        for step in range(1, gain_schedule_points + 1):
            t = min(step * dt, interpolation_time)
            scheduled_point = WeightedTrajectoryPoint(
                point=copy.copy(point), weights=interpolate_weights(w1, w2, t)
            )
            self._trajectory_buffer.append(scheduled_point)

        weighted_trajectory = [
            WeightedTrajectoryPoint(point=p, weights=copy.copy(w2)) for p in mesh_traj
        ]
        self._trajectory_buffer.extend(weighted_trajectory)

        # 3. Retract
        q = self._trajectory_buffer[-1].point.robot_configuration
        T_end_polishing = self._trajectory_buffer[-1].point.end_effector_poses[
            self._params.tool_frame_id
        ]

        self._insert_sequence_to_buffer(
            "insert_retract_tool", q, T_pregrasp, T_end_polishing
        )

    def _build_mesh_marker(self, *, stamp, marker_id: int) -> Marker | None:
        if self._params.path_mode != "mesh":
            return None

        if self._mesh_generator is None:
            return None

        ply_file = self._params.mesh_path.ply_file
        if not ply_file:
            return None

        mesh_pose = self._mesh_generator.get_mesh_pose_world()
        pose = pin.SE3ToXYZQUAT(mesh_pose)
        mesh_resource = Path(ply_file).expanduser().resolve(strict=False).as_uri()

        return Marker(
            header=Header(frame_id=self._params.world_frame_id, stamp=stamp),
            ns="mesh",
            id=marker_id,
            type=Marker.MESH_RESOURCE,
            action=Marker.ADD,
            pose=Pose(
                position=Point(**dict(zip("xyz", pose[:3]))),
                orientation=Quaternion(**dict(zip("xyzw", pose[3:]))),
            ),
            scale=Vector3(x=1.0, y=1.0, z=1.0),
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8),
            lifetime=Duration(seconds=0.0).to_msg(),
            mesh_resource=mesh_resource,
            mesh_use_embedded_materials=False,
        )

    def _publish_mesh_marker(self) -> None:
        marker = self._build_mesh_marker(
            stamp=self.get_clock().now().to_msg(),
            marker_id=0,
        )
        if marker is not None:
            self._mesh_publisher.publish(marker)

    def _append_polishing_sequence(
        self, T_pregrasp: pin.SE3, T_contact: pin.SE3
    ) -> None:
        if self._params.path_mode == "sphere":
            self._append_spherical_polishing_sequence(T_pregrasp)
            return
        elif self._params.path_mode == "mesh":
            self._append_mesh_polishing_sequence(T_pregrasp)
            return

        # Default: original planar polishing_motion
        q = self._trajectory_buffer[-1].point.robot_configuration
        T_insert_goal = self._compute_insert_goal(T_pregrasp, T_contact)
        self._insert_sequence_to_buffer(
            "insert_retract_tool", q, T_insert_goal, T_pregrasp
        )

        q = self._trajectory_buffer[-1].point.robot_configuration
        T_inserted = self._trajectory_buffer[-1].point.end_effector_poses[
            self._params.tool_frame_id
        ]
        self._insert_sequence_to_buffer("polishing_motion", q, T_inserted, T_inserted)

        q = self._trajectory_buffer[-1].point.robot_configuration
        self._insert_sequence_to_buffer(
            "insert_retract_tool", q, T_pregrasp, T_inserted
        )

    def _ensure_dynamic_handle(self, handle_name: str) -> bool:
        if handle_name not in self._handle_aliases:
            return handle_name in self._handles

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._params.polished_object_frame_id,
                handle_name,
                rclpy.time.Time(),
            ).transform
        except TransformException:
            self.get_logger().info(
                f"Waiting for transform '{self._params.polished_object_frame_id}' -> '{handle_name}'...",
                throttle_duration_sec=2.0,
            )
            return False

        contact_pose = pin.SE3(
            pin.Quaternion(
                np.array(
                    [
                        tf_msg.rotation.x,
                        tf_msg.rotation.y,
                        tf_msg.rotation.z,
                        tf_msg.rotation.w,
                    ]
                )
            ),
            np.array(
                [
                    tf_msg.translation.x,
                    tf_msg.translation.y,
                    tf_msg.translation.z,
                ]
            ),
        )
        self._dynamic_contacts[handle_name] = contact_pose
        self.get_logger().info(f"Updated dynamic contact '{handle_name}' from TF")
        return True

    def _plan_with_handle(self, handle_name: str, T_pylone: pin.SE3) -> bool:
        planning_handle = self._handle_aliases.get(handle_name, handle_name)

        if planning_handle not in self._handles:
            self.get_logger().error(
                f"Missing static definition for handle '{planning_handle}'"
            )
            return False

        if handle_name != planning_handle:
            if not self._ensure_dynamic_handle(handle_name):
                return False

        try:
            joint_map = [
                self._joint_states.name.index(joint_name)
                for joint_name in self._params.moving_joints
            ]
            robot_configuration = np.array(self._joint_states.position)[joint_map]

            target_handle = T_pylone * self._handles[planning_handle]
            trajectory = self._path_generators["follow_joint_trajectory"][
                "generator"
            ].get_path(robot_configuration, target_handle, planning_handle)

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

            if not len(self._trajectory_buffer):
                raise RuntimeError("Empty trajectory returned by main generator")

            T_pregrasp = self._trajectory_buffer[-1].point.end_effector_poses[
                self._params.tool_frame_id
            ]

            if handle_name in self._dynamic_contacts:
                T_contact = T_pylone * self._dynamic_contacts[handle_name]
            else:
                T_contact = target_handle * self._R_insert

            self._append_polishing_sequence(T_pregrasp, T_contact)
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to plan a trajectory to handle '{handle_name}'. Reason: {str(e)}"
            )
            return False

    def _planner_cb(self) -> None:
        self._update_params()

        if self._params.path_mode == "mesh" and self._robot_model is not None:
            try:
                self._get_or_update_mesh_generator()
                self._publish_mesh_marker()
            except Exception as e:
                self.get_logger().error(f"Failed to publish mesh marker: {str(e)}")

        for topic_name, obj in (
            (self._robot_description_sub.topic, self._robot_model),
            (self._environment_description_sub.topic, self._environment_description),
            (self._joint_states_sub.topic, self._joint_states),
            (self._buffer_size_sub.topic, self._buffer_size),
        ):
            if obj is None:
                self.get_logger().info(
                    f"Waiting for topic '{topic_name}'...",
                    throttle_duration_sec=5.0,
                )
                return

        if not self._path_generators_initialized:
            if self._params.generators_params.main_generator == "diffusion":
                # self._path_generators["follow_joint_trajectory"]["generator"] = DiffusionPathGenerator(...)
                raise NotImplementedError("Diffusion generator not wired here.")
            else:
                gen_params = self._params.generators_params.hpp_generator
                self._path_generators["follow_joint_trajectory"]["generator"] = (
                    HPPPathGenerator(
                        handles_configurations_path=Path(
                            gen_params.handle_configs_path
                        ),
                        robot_description=self._robot_description,
                        environment_description=self._environment_description,
                        ocp_dt=self._params.ocp_dt,
                        tool_frame_id=self._params.tool_frame_id,
                        robot_model=self._robot_model,
                        robot_self_collision_config=Path(
                            gen_params.robot_self_collision_config
                        ),
                        handles_srdf_path=Path(self._params.handles_srdf_path),
                        demo_config=Path(gen_params.demo_config),
                        robot_name=gen_params.robot_name,
                        gripper_name=gen_params.gripper_name,
                        polished_object_name=gen_params.polished_object_name,
                        joints_to_shrink=gen_params.joints_to_shrink,
                        joint_shrink_range=gen_params.joint_shrink_range,
                    )
                )
            self._path_generators["insert_retract_tool"]["generator"] = (
                GraspPathGenerator(
                    robot_model=self._robot_model,
                    ocp_dt=self._params.ocp_dt,
                    tool_frame_id=self._params.tool_frame_id,
                    max_linear_vel=self._params.generators_params.grasp_generator.max_linear_vel,
                )
            )
            self._path_generators["polishing_motion"]["generator"] = (
                PolishingPathGenerator(
                    robot_model=self._robot_model,
                    ocp_dt=self._params.ocp_dt,
                    tool_frame_id=self._params.tool_frame_id,
                    measurement_frame_id=self._params.measurement_frame_id,
                    desired_force=self._params.desired_force,
                    angle=self._params.generators_params.polishing_generator.angle,
                    frequency=self._params.generators_params.polishing_generator.frequency,
                    slope_circles=self._params.generators_params.polishing_generator.slope_circles,
                    n_circles=self._params.generators_params.polishing_generator.n_circles,
                    force_ramp=self._params.force_ramp,
                )
            )
            self.get_logger().info(
                "Trajecotry generators initialized!",
                throttle_duration_sec=5.0,
            )
            self._path_generators_initialized = True

        try:
            T_msg = self._tf_buffer.lookup_transform(
                self._params.world_frame_id,
                self._params.polished_object_frame_id,
                rclpy.time.Time(),
            ).transform
            T_pylone = pin.SE3(
                pin.Quaternion(np.array([getattr(T_msg.rotation, v) for v in "xyzw"])),
                np.array([getattr(T_msg.translation, v) for v in "xyz"]),
            )
            self._path_generators["follow_joint_trajectory"][
                "generator"
            ].update_polished_object_pose(T_pylone)
        except TransformException:
            self.get_logger().info(
                "Waiting for transformation between "
                f"'{self._params.world_frame_id}' and '{self._params.polished_object_frame_id}'...",
                throttle_duration_sec=5.0,
            )
            return

        if self._target_handle_name is None:
            self.get_logger().info(
                f"Waiting for goal message on topic '{self._target_handle_sub.topic}'...",
                throttle_duration_sec=5.0,
            )
            return

        if not self._plan_with_handle(self._target_handle_name, T_pylone):
            return

        self._last_handle = copy.copy(self._target_handle_name)
        self._target_handle_name = None

        if self._params.visualization.publish_path:
            now = self.get_clock().now().to_msg()
            lifetime = len(self._trajectory_buffer) * self._params.ocp_dt * 1.25

            def _crate_marker(id: int, point: TrajectoryPoint) -> Marker:
                pose = pin.SE3ToXYZQUAT(
                    point.end_effector_poses[self._params.tool_frame_id]
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

            def _create_normal_marker(id: int, point: TrajectoryPoint) -> Marker:
                ee_pose = point.end_effector_poses[self._params.tool_frame_id]
                start = np.array(ee_pose.translation, dtype=np.float64)
                tracked_normal = np.array(ee_pose.rotation)[:, 2]
                tracked_normal /= np.linalg.norm(tracked_normal)

                marker_scale = self._params.visualization.marker_size
                normal_length = marker_scale * 6.0
                end = start + normal_length * tracked_normal

                return Marker(
                    header=Header(
                        frame_id=self._params.world_frame_id,
                        stamp=now,
                    ),
                    ns="warmstart_normals",
                    id=id,
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    points=[
                        Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                        Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
                    ],
                    # For ARROW markers with points:
                    # x: shaft diameter, y: head diameter, z: head length
                    scale=Vector3(
                        x=marker_scale * 0.35,
                        y=marker_scale * 0.75,
                        z=marker_scale * 1.4,
                    ),
                    color=ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.95),
                    lifetime=Duration(seconds=lifetime).to_msg(),
                )

            markers = []
            for i, point in enumerate(self._trajectory_buffer):
                markers.append(_crate_marker(i, point.point))
                markers.append(_create_normal_marker(i, point.point))

            self._path_publisher.publish(MarkerArray(markers=markers))

    def _publish_mpc_input_cb(self) -> None:
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
        diffusion_warmstart_node = PolishingPathPlanner()
        rclpy.spin(diffusion_warmstart_node)
        diffusion_warmstart_node.destroy_node()
    except (KeyboardInterrupt, ParameterException):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
