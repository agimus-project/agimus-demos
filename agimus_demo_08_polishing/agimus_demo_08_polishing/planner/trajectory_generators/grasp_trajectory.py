import copy

import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_08_polishing.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)


class GraspPathGenerator(GenericTrajectoryGenerator):
    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        max_linear_vel: float,
        tool_frame_id: str,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._max_linear_vel = max_linear_vel

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._ocp_dt = ocp_dt

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

    @property
    def max_linear_vel(self) -> float:
        return float(self._max_linear_vel)

    @max_linear_vel.setter
    def max_linear_vel(self, value: float) -> None:
        if value <= 0.0:
            raise ValueError("max_linear_vel must be strictly positive.")
        self._max_linear_vel = float(value)

    def _ik(self, q_init: npt.ArrayLike, target: pin.SE3):
        q = np.copy(q_init)
        for _ in range(5):
            pin.forwardKinematics(self._robot_model, self._robot_data, q)
            pin.updateFramePlacement(
                self._robot_model, self._robot_data, self._tool_frame_id_pin_frame
            )

            rMf = self._robot_data.oMf[self._tool_frame_id_pin_frame].actInv(target)

            err = pin.log6(rMf).vector
            fJf = pin.computeFrameJacobian(
                self._robot_model,
                self._robot_data,
                q,
                self._tool_frame_id_pin_frame,
                pin.LOCAL,
            )
            rJf = pin.Jlog6(rMf)
            J = -rJf @ fJf
            v = -J.T @ (np.linalg.solve(J @ J.T + 1e-12 * np.eye(6), err))
            q = pin.integrate(self._robot_model, q, v)

            if np.max(np.abs(rMf.translation)) < 1e-6:
                q[-1] = q_init[-1]
                return q
        raise RuntimeError("Failed to converge with IK")

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

        start_pose = copy.copy(T_init)

        # Separately interpolate in SO3 and R3 separately to obtain linear motion
        diff = start_pose.inverse() * T_final
        rot_vel = pin.log3(diff.rotation)

        dist = np.linalg.norm(diff.translation)

        lin_time = dist / self._max_linear_vel

        # Time for the interpolation is slightly larger than needed
        # to account for accelerations and decelerations, plus
        # to presume safety a margin
        interp_time = lin_time * 1.25
        n_interp = max(int(np.ceil(interp_time / self._ocp_dt)), 1)

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            if n_interp == 1:
                t = 1.0
            else:
                t = i / (n_interp - 1)
            target = pin.SE3(np.eye(4))
            # Interpolate rotation
            target.rotation = start_pose.rotation @ pin.exp3(rot_vel * t)
            # Interpolate translation
            start_t = start_pose.translation
            end_t = T_final.translation
            target.translation = (1.0 - t) * start_t + (t) * end_t

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(7),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={},
                end_effector_poses={self._tool_frame_id_name: target},
            )

        return [_create_trajectory_point(i) for i in range(n_interp)]
