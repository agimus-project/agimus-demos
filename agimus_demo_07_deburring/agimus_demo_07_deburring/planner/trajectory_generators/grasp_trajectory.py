import copy

import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_07_deburring.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)

from agimus_demo_07_deburring.planner.trajectory_generators.utils import SCurveGenerator


class GraspPathGenerator(GenericTrajectoryGenerator):
    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        linear_vel: float,
        linear_acc: float,
        linear_jerk: float,
        tool_frame_id: str,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._linear_vel = linear_vel
        self._linear_acc = linear_acc
        self._linear_jerk = linear_jerk

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._ocp_dt = ocp_dt

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

        self._s_curve_generator = SCurveGenerator(
            linear_vel, linear_acc, linear_jerk, self._ocp_dt
        )

    def get_path(
        self,
        q0,
        T_final: pin.SE3,
        handle_name: str | None = None,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        # Copy the object to avoid accidental overriding
        start_pose = copy.copy(T_init)
        # Compute the distance between starting pose and final one
        diff = start_pose.inverse() * T_final
        rot_vel = pin.log3(diff.rotation)
        dist = np.linalg.norm(diff.translation)

        traj, _ = self._s_curve_generator.compute_position_s_curve(np.abs(dist))
        # Rescale s-curve based trajectory to become a blending proportion between
        # initial and final frames for the interpolation.
        time_spacing = traj / dist

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            t = time_spacing[i]
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

        return [_create_trajectory_point(i) for i in range(len(time_spacing))]
