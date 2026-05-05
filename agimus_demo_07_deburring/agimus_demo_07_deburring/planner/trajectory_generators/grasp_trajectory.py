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
        tool_angular_vel: float,
        tool_angular_acc: float,
        tool_angular_jerk: float,
        insert_joint_angle: float,
        retract_joint_angle: float,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._insert_joint_angle = insert_joint_angle
        self._retract_joint_angle = retract_joint_angle
        self._ocp_dt = ocp_dt

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

        self._linear_s_curve_generator = SCurveGenerator(
            linear_vel, linear_acc, linear_jerk, self._ocp_dt
        )

        self._joint_s_curve_generator = SCurveGenerator(
            tool_angular_vel, tool_angular_acc, tool_angular_jerk, self._ocp_dt
        )

    def set_insert_mode(self):
        self._target_j = self._insert_joint_angle

    def set_retract_mode(self):
        self._target_j = self._retract_joint_angle

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
        traj, _ = self._linear_s_curve_generator.compute_position_s_curve(0.0, dist)
        # Rescale s-curve based trajectory to become a blending proportion between
        # initial and final frames for the interpolation.
        if np.isclose(traj[-1], 0.0):
            time_spacing = np.zeros_like(traj)
        else:
            time_spacing = traj / traj[-1]

        # Move first joint to the limit
        j_traj, j_vel = self._joint_s_curve_generator.compute_position_s_curve(
            q0[-1], self._target_j
        )

        if len(j_traj) > len(time_spacing):
            # Keep TCP at the final target while the last joint still converges.
            extra_steps = np.ones(len(j_traj) - len(time_spacing))
            time_spacing = np.concatenate((time_spacing, extra_steps))
        else:
            extra_steps = np.ones(len(time_spacing) - len(j_traj))
            j_traj = np.concatenate((j_traj, extra_steps * j_traj[-1]))
            j_vel = np.concatenate((j_vel, extra_steps * 0.0))

        # Ensure a rest condition at sequence boundary.
        j_vel[-1] = 0.0

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            t = time_spacing[i]
            target = pin.SE3.Identity()
            # Interpolate rotation
            target.rotation = start_pose.rotation @ pin.exp3(rot_vel * t)
            # Interpolate translation
            start_t = start_pose.translation
            end_t = T_final.translation
            target.translation = (1.0 - t) * start_t + (t) * end_t

            q = q0.copy()
            q[-1] = j_traj[i]
            dq = np.zeros(self._nv)
            dq[-1] = j_vel[i]

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q,
                robot_velocity=dq,
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={},
                end_effector_poses={self._tool_frame_id_name: target},
            )

        return [_create_trajectory_point(i) for i in range(len(time_spacing))]
