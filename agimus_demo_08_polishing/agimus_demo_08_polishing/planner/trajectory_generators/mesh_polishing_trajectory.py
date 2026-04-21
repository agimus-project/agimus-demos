import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_08_polishing.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)
from agimus_demo_08_polishing.planner.ply_processor import PlyProcessor


class MeshPolishingPathGenerator(GenericTrajectoryGenerator):
    """
    Polishing path generator for arbitrary mesh surfaces.
    """

    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        angle: float,
        frequency: float,
        slope_circles: int,
        n_circles: int,
        circle_point_downsample_factor: int,
        tool_frame_id: str,
        measurement_frame_id: str,
        desired_force: float,
        force_ramp: float,
        ply_file_path: str,
        ply_center_point: list[float],
        robot_center_point: list[float],
        mesh_translation_xyz: list[float],
        mesh_shift_xyz: list[float],
        mesh_refine_enabled: bool,
        mesh_upsample_enabled: bool,
        mesh_refine_iterations: int,
        mesh_refine_lambda: float,
        trajectory_mode: str,
        line_axis: str,
        line_length: float,
        line_speed: float,
        line_speed_mode: str,
        sequence_axes: list[str],
        sequence_lengths: list[float],
        mesh_rotation_z: float,
        outward_normal: bool,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._circle_radius = angle
        self._trajectory_mode = str(trajectory_mode).lower()
        self._line_axis = self._normalize_axis_token(line_axis)
        self._line_length = float(line_length)
        self._line_speed = float(line_speed)
        self._line_speed_mode = str(line_speed_mode).lower()
        self._sequence_axes = [
            self._normalize_axis_token(axis) for axis in sequence_axes
        ]
        self._sequence_lengths = [float(length) for length in sequence_lengths]
        self._mesh_refine_enabled = bool(mesh_refine_enabled)
        self._mesh_upsample_enabled = bool(mesh_upsample_enabled)
        self._mesh_refine_iterations = int(mesh_refine_iterations)
        self._mesh_refine_lambda = float(mesh_refine_lambda)
        self._frequency = frequency
        self._slope_circles = slope_circles
        self._n_circles = n_circles
        self._circle_point_downsample_factor = int(circle_point_downsample_factor)
        self._force_ramp = force_ramp

        if self._trajectory_mode not in ("circle", "line", "sequence"):
            raise ValueError(
                "Mesh trajectory_mode has to be either 'circle', 'line' or 'sequence'."
            )
        if self._line_axis not in ("x", "-x", "y", "-y"):
            raise ValueError("Mesh line_axis has to be one of: 'x', '-x', 'y', '-y'.")
        if self._line_length <= 0.0:
            raise ValueError("Mesh line_length has to be strictly positive.")
        if self._line_speed <= 0.0:
            raise ValueError("Mesh line_speed has to be strictly positive.")
        if self._line_speed_mode not in ("triangular", "constant"):
            raise ValueError(
                "Mesh line_speed_mode has to be either 'triangular' or 'constant'."
            )
        if self._circle_point_downsample_factor < 1:
            raise ValueError(
                "Mesh circle_point_downsample_factor has to be an integer >= 1."
            )
        if self._mesh_refine_iterations < 0:
            raise ValueError("Mesh mesh_refine_iterations has to be >= 0.")
        if not (0.0 <= self._mesh_refine_lambda <= 1.0):
            raise ValueError("Mesh mesh_refine_lambda has to be in [0, 1].")
        if self._trajectory_mode == "sequence":
            if len(self._sequence_axes) == 0:
                raise ValueError(
                    "Mesh sequence mode requires at least one axis in sequence_axes."
                )
            if len(self._sequence_axes) != len(self._sequence_lengths):
                raise ValueError(
                    "Mesh sequence_axes and sequence_lengths must have the same size."
                )
            if any(length <= 0.0 for length in self._sequence_lengths):
                raise ValueError(
                    "Mesh sequence_lengths values have to be strictly positive."
                )

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._ocp_dt = ocp_dt

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

        # Force setup similar to PolishingPathGenerator
        f = np.zeros(6)
        f[2] = -desired_force
        self._desired_force = pin.Force(f)
        self._desired_force_magnitude = desired_force
        self._measurement_frame_id = measurement_frame_id

        if self._trajectory_mode == "circle":
            if self._frequency <= 0.0:
                raise ValueError("Polishing frequency must be strictly positive.")

            self._omega = 2.0 * np.pi * self._frequency
            base_samples_per_circle = max(
                1, int(np.round((1.0 / self._frequency) / self._ocp_dt))
            )
            # Keep timing/speed unchanged: do not alter the number of control samples.
            # Downsampling only reduces the geometric circle discretization on the mesh.
            geometry_samples_per_circle = max(
                4,
                int(
                    np.ceil(
                        base_samples_per_circle
                        / float(self._circle_point_downsample_factor)
                    )
                ),
            )
            self._samples_per_circle = base_samples_per_circle
            self._geometry_samples_per_circle = geometry_samples_per_circle
            self._n_samples = self._samples_per_circle * max(self._n_circles, 0)

            slope_samples = int(self._samples_per_circle * max(self._slope_circles, 0))
            self._slope_samples = min(slope_samples, self._n_samples // 2)
        else:
            # Single pass line/sequence trajectory:
            # triangular profile => average speed is line_speed / 2.
            # constant profile => average speed is line_speed.
            self._omega = 0.0
            self._samples_per_circle = 0
            if self._trajectory_mode == "line":
                path_length = self._line_length
            else:
                path_length = float(np.sum(self._sequence_lengths))
            if self._line_speed_mode == "triangular":
                line_duration = 2.0 * path_length / self._line_speed
            else:
                line_duration = path_length / self._line_speed
            n_line_samples = int(np.round(line_duration / self._ocp_dt))
            self._n_samples = max(2, n_line_samples + 1)
            self._slope_samples = 0
            self._geometry_samples_per_circle = 0

        if self._force_ramp > 0.0:
            self._ramp_samples = max(2, int(np.round(self._force_ramp / self._ocp_dt)))
        else:
            self._ramp_samples = 0

        # Initialize and process mesh
        self._ply_processor = PlyProcessor()
        self._ply_processor.load_ply(ply_file_path)
        if self._mesh_refine_enabled:
            if self._mesh_upsample_enabled:
                self._ply_processor.upsample_mesh()
            if self._mesh_refine_iterations > 0:
                self._ply_processor.refine_mesh(
                    iterations=self._mesh_refine_iterations,
                    lambda_param=self._mesh_refine_lambda,
                )

        # Calculate Z for ply_center_point automatically
        xy_target = np.array(ply_center_point[:2])
        xy_dist = np.linalg.norm(
            self._ply_processor.vertices[:, :2] - xy_target, axis=1
        )

        candidate_indices = np.where(xy_dist < 0.005)[0]

        if len(candidate_indices) > 0:
            candidate_z = self._ply_processor.vertices[candidate_indices, 2]
            # We scan from top (assuming Z is up)
            best_idx = candidate_indices[np.argmax(candidate_z)]
            self._mesh_anchor_point = self._ply_processor.vertices[best_idx]
        else:
            print(
                f"Warning: No vertices found near XY={xy_target}. Using closest 3D point to (x,y,0)."
            )
            fallback_pt = np.array([xy_target[0], xy_target[1], 0.0])
            _, self._mesh_anchor_point, _ = self._ply_processor.get_closest_vertex(
                fallback_pt
            )

        print(f"Calculated mesh anchor point: {self._mesh_anchor_point}")

        self._robot_center_point = np.array(robot_center_point, dtype=np.float64)
        self._mesh_translation = np.array(mesh_translation_xyz, dtype=np.float64)
        self._mesh_shift_world = np.array(mesh_shift_xyz, dtype=np.float64)
        self._mesh_rotation_z = float(mesh_rotation_z)
        self._mesh_rotation = pin.rpy.rpyToMatrix(
            np.array([0.0, 0.0, self._mesh_rotation_z], dtype=np.float64)
        )
        self._mesh_center_world = self._robot_center_point + self._mesh_translation
        self._outward_normal = bool(outward_normal)
        # Path shift is specified in world frame; convert to mesh frame.
        self._path_center_point = (
            self._mesh_anchor_point + self._mesh_rotation.T @ self._mesh_shift_world
        )

        # Pre-compute trajectory poses on the mesh (in Mesh Frame).
        if self._trajectory_mode == "circle":
            self._base_trajectory_poses_mesh = self._ply_processor.compute_trajectory(
                self._path_center_point,
                self._circle_radius,
                num_points=max(100, self._geometry_samples_per_circle),
            )
        elif self._trajectory_mode == "line":
            self._base_trajectory_poses_mesh = self._compute_line_trajectory(
                num_points=max(2, self._n_samples)
            )
        else:
            self._base_trajectory_poses_mesh = self._compute_sequence_trajectory(
                num_points=max(2, self._n_samples)
            )
        if self._outward_normal:
            flip_rotation = np.diag([-1.0, 1.0, -1.0])
            for pose in self._base_trajectory_poses_mesh:
                pose[:3, :3] = pose[:3, :3] @ flip_rotation

    @staticmethod
    def _normalize_axis_token(axis: str) -> str:
        token = str(axis).strip().lower()
        if token in ("x", "+x"):
            return "x"
        if token == "-x":
            return "-x"
        if token in ("y", "+y"):
            return "y"
        if token == "-y":
            return "-y"
        raise ValueError(
            f"Invalid axis token '{axis}'. Supported values are x, +x, -x, y, +y, -y."
        )

    @staticmethod
    def _axis_to_world_vector(axis: str) -> np.ndarray:
        if axis == "x":
            return np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if axis == "-x":
            return np.array([-1.0, 0.0, 0.0], dtype=np.float64)
        if axis == "y":
            return np.array([0.0, 1.0, 0.0], dtype=np.float64)
        if axis == "-y":
            return np.array([0.0, -1.0, 0.0], dtype=np.float64)
        raise ValueError(f"Unsupported normalized axis '{axis}'.")

    def _compose_surface_pose(
        self, point_guess_mesh: np.ndarray, tangent_hint_mesh: np.ndarray
    ) -> np.ndarray:
        _, p_surface, n_surface = self._ply_processor.get_closest_vertex(
            point_guess_mesh
        )

        z_tool = -n_surface
        tangent = tangent_hint_mesh - np.dot(tangent_hint_mesh, n_surface) * n_surface
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm < 1e-6:
            tangent = np.cross(z_tool, np.array([1.0, 0.0, 0.0], dtype=np.float64))
            if np.linalg.norm(tangent) < 1e-6:
                tangent = np.cross(z_tool, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        y_tool = tangent / np.linalg.norm(tangent)

        x_tool = np.cross(y_tool, z_tool)
        x_tool /= np.linalg.norm(x_tool)
        y_tool = np.cross(z_tool, x_tool)
        y_tool /= np.linalg.norm(y_tool)

        T = np.eye(4)
        T[:3, :3] = np.column_stack((x_tool, y_tool, z_tool))
        T[:3, 3] = p_surface
        return T

    def _compute_line_trajectory(self, num_points: int) -> list[np.ndarray]:
        axis_world = self._axis_to_world_vector(self._line_axis)

        axis_mesh = self._mesh_rotation.T @ axis_world
        axis_mesh_norm = np.linalg.norm(axis_mesh)
        if axis_mesh_norm < 1e-9:
            raise RuntimeError("Degenerate line axis for mesh line trajectory.")
        axis_mesh /= axis_mesh_norm

        # Start from the initial center and move only in the selected direction.
        offsets = np.linspace(0.0, self._line_length, num_points, dtype=np.float64)

        trajectory_poses = []
        for s in offsets:
            p_guess = self._path_center_point + s * axis_mesh
            trajectory_poses.append(
                self._compose_surface_pose(
                    point_guess_mesh=p_guess, tangent_hint_mesh=axis_mesh
                )
            )

        return trajectory_poses

    def _compute_sequence_trajectory(self, num_points: int) -> list[np.ndarray]:
        n_segments = len(self._sequence_axes)
        if n_segments == 0:
            raise RuntimeError(
                "Mesh sequence trajectory requires at least one segment."
            )

        segment_lengths = np.asarray(self._sequence_lengths, dtype=np.float64)
        total_length = float(np.sum(segment_lengths))
        if total_length <= 0.0:
            raise RuntimeError(
                "Mesh sequence trajectory has non-positive total length."
            )

        segment_axes_mesh: list[np.ndarray] = []
        segment_starts_mesh: list[np.ndarray] = []
        current_point = np.array(self._path_center_point, dtype=np.float64)
        for axis in self._sequence_axes:
            axis_world = self._axis_to_world_vector(axis)
            axis_mesh = self._mesh_rotation.T @ axis_world
            axis_norm = np.linalg.norm(axis_mesh)
            if axis_norm < 1e-9:
                raise RuntimeError("Degenerate axis in mesh sequence trajectory.")
            axis_mesh /= axis_norm
            segment_axes_mesh.append(axis_mesh)
            segment_starts_mesh.append(current_point.copy())
            current_point = (
                current_point + axis_mesh * segment_lengths[len(segment_axes_mesh) - 1]
            )

        cumulative = np.zeros(n_segments + 1, dtype=np.float64)
        cumulative[1:] = np.cumsum(segment_lengths)
        distances = np.linspace(0.0, total_length, num_points, dtype=np.float64)

        trajectory_poses = []
        for distance in distances:
            segment_index = int(np.searchsorted(cumulative, distance, side="right") - 1)
            segment_index = max(0, min(segment_index, n_segments - 1))
            local_distance = distance - cumulative[segment_index]
            local_distance = min(local_distance, segment_lengths[segment_index])

            p_guess = (
                segment_starts_mesh[segment_index]
                + local_distance * segment_axes_mesh[segment_index]
            )
            trajectory_poses.append(
                self._compose_surface_pose(
                    point_guess_mesh=p_guess,
                    tangent_hint_mesh=segment_axes_mesh[segment_index],
                )
            )

        return trajectory_poses

    def _transform_point_to_world(self, point_mesh: np.ndarray) -> np.ndarray:
        return self._mesh_center_world + self._mesh_rotation @ (
            point_mesh - self._mesh_anchor_point
        )

    def _transform_pose_to_world(
        self, rotation_mesh: np.ndarray, translation_mesh: np.ndarray
    ) -> pin.SE3:
        return pin.SE3(
            self._mesh_rotation @ rotation_mesh,
            self._transform_point_to_world(translation_mesh),
        )

    def get_mesh_pose_world(self) -> pin.SE3:
        translation = (
            self._mesh_center_world - self._mesh_rotation @ self._mesh_anchor_point
        )
        return pin.SE3(self._mesh_rotation, translation)

    def get_path(
        self,
        q0,
        T_final: pin.SE3,
        handle_name: str | None = None,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        pin.forwardKinematics(self._robot_model, self._robot_data, q0)
        pin.updateFramePlacement(
            self._robot_model, self._robot_data, self._tool_frame_id_pin_frame
        )

        # Determine start orientation/pose
        # We use the orientation from the first point of our computed mesh trajectory
        _, c_pos_mesh, _ = self._ply_processor.get_closest_vertex(
            self._path_center_point
        )
        T_start_rot_mesh = self._base_trajectory_poses_mesh[0][:3, :3]
        T_start = self._transform_pose_to_world(T_start_rot_mesh, c_pos_mesh)

        def _generate_force_ramp(
            i: int, ascend: bool, ramp_pose: pin.SE3
        ) -> TrajectoryPoint:
            if self._ramp_samples <= 1:
                scale = 1.0 if ascend else 0.0
            else:
                denominator = float(self._ramp_samples - 1)
                if ascend:
                    scale = i / denominator
                else:
                    scale = (denominator - i) / denominator

            f = self._desired_force * scale

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: ramp_pose * f},
                end_effector_poses={self._tool_frame_id_name: ramp_pose},
            )

        def _generate_sequence(i: int) -> TrajectoryPoint:
            # Calculate scale for slope in/out (used in circle mode).
            scale = 1.0
            if self._slope_samples > 0:
                ramp_in = min(1.0, i / self._slope_samples)
                ramp_out = min(1.0, (self._n_samples - 1 - i) / self._slope_samples)
                scale = max(0.0, min(ramp_in, ramp_out))

            # Interpolate trajectory from pre-computed mesh points.
            n_points = len(self._base_trajectory_poses_mesh)
            if self._trajectory_mode == "circle":
                if self._samples_per_circle <= 0:
                    phase = 0.0
                else:
                    phase = (
                        2.0
                        * np.pi
                        * (i % self._samples_per_circle)
                        / self._samples_per_circle
                    )
                idx_float = (phase / (2 * np.pi)) * n_points
                idx = int(idx_float) % n_points
                next_idx = (idx + 1) % n_points
            else:
                if self._n_samples <= 1:
                    progress = 0.0
                else:
                    u = i / (self._n_samples - 1)
                    if self._line_speed_mode == "triangular":
                        # u in [0, 1], with piecewise progress that yields
                        # line/sequence speed profile 0 -> line_speed -> 0.
                        if u <= 0.5:
                            progress = 2.0 * u * u
                        else:
                            progress = -2.0 * u * u + 4.0 * u - 1.0
                    else:
                        # Constant speed profile along the line/sequence.
                        progress = u
                idx_float = progress * (n_points - 1)
                idx = min(int(idx_float), n_points - 1)
                next_idx = min(idx + 1, n_points - 1)
            alpha = idx_float - idx

            T1 = self._base_trajectory_poses_mesh[idx]
            T2 = self._base_trajectory_poses_mesh[next_idx]

            # Pose interpolation on Mesh
            pos = (1 - alpha) * T1[:3, 3] + alpha * T2[:3, 3]
            rot1 = pin.Quaternion(T1[:3, :3])
            rot2 = pin.Quaternion(T2[:3, :3])
            rot = rot1.slerp(alpha, rot2)

            # If scale < 1, interpolate position towards center (circle mode only).
            if self._trajectory_mode == "circle" and scale < 0.99:
                pos_on_chord = (1 - scale) * self._path_center_point + scale * pos
                _, final_pos_mesh, _ = self._ply_processor.get_closest_vertex(
                    pos_on_chord
                )
                final_pose = self._transform_pose_to_world(rot.matrix(), final_pos_mesh)
            else:
                final_pose = self._transform_pose_to_world(rot.matrix(), pos)

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: final_pose * self._desired_force},
                end_effector_poses={self._tool_frame_id_name: final_pose},
            )

        t2 = [_generate_sequence(i) for i in range(self._n_samples)]
        if t2:
            ramp_start_pose = t2[0].end_effector_poses[self._tool_frame_id_name]
            ramp_end_pose = t2[-1].end_effector_poses[self._tool_frame_id_name]
        else:
            ramp_start_pose = T_start
            ramp_end_pose = T_start

        t1 = [
            _generate_force_ramp(i, ascend=True, ramp_pose=ramp_start_pose)
            for i in range(self._ramp_samples)
        ]
        t3 = [
            _generate_force_ramp(i, ascend=False, ramp_pose=ramp_end_pose)
            for i in range(self._ramp_samples)
        ]

        return t1 + t2 + t3
