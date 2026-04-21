import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_08_polishing.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)


class PolishingPathGenerator(GenericTrajectoryGenerator):
    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        angle: float,
        frequency: float,
        slope_circles: int,
        n_circles: int,
        tool_frame_id: str,
        measurement_frame_id: str,
        desired_force: float,
        force_ramp: float,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._circle_radius = angle
        self._frequency = frequency
        self._slope_circles = slope_circles
        self._n_circles = n_circles
        self._force_ramp = force_ramp

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._ocp_dt = ocp_dt

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )
        f = np.zeros(6)
        # Positive force is easier to understand in configs, but in fact we want a negative one,
        # as it has to act towards the object and not outwards from it
        f[2] = -desired_force
        self._desired_force = pin.Force(f)
        self._measurement_frame_id = measurement_frame_id

        if self._frequency <= 0.0:
            raise ValueError("Polishing frequency must be strictly positive.")

        self._omega = 2.0 * np.pi * self._frequency
        samples_per_circle = max(
            1, int(np.round((1.0 / self._frequency) / self._ocp_dt))
        )
        self._samples_per_circle = samples_per_circle
        self._n_samples = self._samples_per_circle * max(self._n_circles, 0)

        slope_samples = int(self._samples_per_circle * max(self._slope_circles, 0))
        self._slope_samples = min(slope_samples, self._n_samples // 2)

        if self._force_ramp > 0.0:
            self._ramp_samples = max(2, int(np.round(self._force_ramp / self._ocp_dt)))
        else:
            self._ramp_samples = 0

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

        center_pose = pin.SE3(T_init) if T_init is not None else pin.SE3(T_final)
        R_center = np.array(center_pose.rotation)
        p_center = np.array(center_pose.translation)

        force_local = np.array(self._desired_force.vector)[:3]
        force_norm = np.linalg.norm(force_local)
        if force_norm < 1e-9:
            force_dir_local = np.array([0.0, 0.0, 1.0])
        else:
            force_dir_local = force_local / force_norm

        ref_axis = np.array([1.0, 0.0, 0.0])
        if np.abs(ref_axis @ force_dir_local) > 0.95:
            ref_axis = np.array([0.0, 1.0, 0.0])
        axis_u_local = np.cross(force_dir_local, ref_axis)
        axis_u_norm = np.linalg.norm(axis_u_local)
        if axis_u_norm < 1e-9:
            axis_u_local = np.array([0.0, 1.0, 0.0])
            axis_u_norm = 1.0
        axis_u_local /= axis_u_norm
        axis_v_local = np.cross(force_dir_local, axis_u_local)

        def _generate_force_ramp(i: int, ascend: bool) -> TrajectoryPoint:
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
                forces={self._measurement_frame_id: center_pose * f},
                end_effector_poses={self._tool_frame_id_name: center_pose},
            )

        def _generate_sequence(i: int) -> TrajectoryPoint:
            if self._samples_per_circle <= 0:
                phase = 0.0
            else:
                phase = (
                    2.0
                    * np.pi
                    * (i % self._samples_per_circle)
                    / self._samples_per_circle
                )

            if self._slope_samples > 0:
                ramp_in = min(1.0, i / self._slope_samples)
                ramp_out = min(1.0, (self._n_samples - 1 - i) / self._slope_samples)
                scale = max(0.0, min(ramp_in, ramp_out))
            else:
                scale = 1.0

            offset_local = (
                self._circle_radius
                * scale
                * (np.cos(phase) * axis_u_local + np.sin(phase) * axis_v_local)
            )
            offset_world = R_center @ offset_local
            pose = pin.SE3(R_center, p_center + offset_world)

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: center_pose * self._desired_force},
                end_effector_poses={self._tool_frame_id_name: pose},
            )

        t1 = [_generate_force_ramp(i, ascend=True) for i in range(self._ramp_samples)]
        t2 = [_generate_sequence(i) for i in range(self._n_samples)]
        t3 = [_generate_force_ramp(i, ascend=False) for i in range(self._ramp_samples)]

        return t1 + t2 + t3
