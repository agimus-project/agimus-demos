import copy
from math import ceil
import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_07_deburring.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)

from agimus_demo_07_deburring.planner.trajectory_generators.utils import SCurveGenerator


class MetalDeburringPathGenerator(GenericTrajectoryGenerator):
    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        tool_angular_vel: float,
        tool_angular_acc: float,
        tool_angular_jerk: float,
        positioning_time: float,
        positioning_force: float,
        deburring_force: float,
        force_rate_up: float,
        force_rate_down: float,
        tool_joint_start_angle: float,
        tool_joint_end_angle: float,
        n_repeat: int,
        tool_frame_id: str,
        measurement_frame_id: str,
    ) -> None:
        self._robot_model = robot_model

        self._tool_joint_end_angle = tool_joint_end_angle
        self._tool_joint_start_angle = tool_joint_start_angle

        self._n_repeat = n_repeat

        self._positioning_force = positioning_force
        self._deburring_force = deburring_force
        self._force_rate_up = force_rate_up
        self._force_rate_down = force_rate_down
        self._positioning_time = positioning_time
        self._ocp_dt = ocp_dt

        self._idx_cnt = 0

        self._s_curve_generator = SCurveGenerator(
            tool_angular_vel, tool_angular_acc, tool_angular_jerk, self._ocp_dt
        )

        motion_time_total, _, _, _, motion_dist_cruise = (
            self._s_curve_generator.compute_params(
                np.abs(self._tool_joint_end_angle - self._tool_joint_start_angle)
            )
        )
        if motion_dist_cruise < 0:
            raise ValueError(
                "Distance between joint limits of metal deburring generator are too short! "
                "Reduce `metal_deburring_generator.tool_angular_vel` or increase the limits."
            )

        if motion_dist_cruise < 0:
            raise ValueError(
                "Distance between joint limits of metal deburring generator are too short! "
                "Reduce `metal_deburring_generator.tool_angular_vel` or increase the limits."
            )

        force_ramp_up_time = (
            self._deburring_force - self._positioning_force
        ) / self._force_rate_up
        # Assume final force is 0
        force_ramp_down_time = self._deburring_force / self._force_rate_down

        if motion_time_total - force_ramp_up_time - force_ramp_down_time < self._ocp_dt:
            raise ValueError(
                "Force ramping time is too long. Increase the rate of change of force!"
            )

        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._tool_frame_id_name = tool_frame_id
        self._force_base = pin.Force(np.zeros(6))
        self._measurement_frame_id = measurement_frame_id

    def _compute_force_ramp(
        self,
        force_start: float,
        force_target: float,
        force_end: float,
        total_time_steps: int,
    ):
        dt = self._ocp_dt
        force_ramp_up_time_steps = ceil(
            (force_target - force_start) / self._force_rate_up / dt
        )
        force_ramp_down_time_steps = np.abs(
            ceil((force_end - force_target) / self._force_rate_down / dt)
        )

        force_holding_time_steps = total_time_steps - (
            force_ramp_up_time_steps + force_ramp_down_time_steps
        )

        ramp_up_increments = np.arange(0, force_ramp_up_time_steps, 1) * dt
        ramp_down_increments = np.arange(0, force_ramp_down_time_steps, 1) * dt

        force_ramp_up = force_start + self._force_rate_up * ramp_up_increments
        force_hold = np.ones(force_holding_time_steps) * force_target
        force_ramp_down = force_target - self._force_rate_down * ramp_down_increments

        return np.concatenate([force_ramp_up, force_hold, force_ramp_down])

    def get_path(
        self,
        q0,
        T_final: pin.SE3,
        handle_name: str | None = None,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        def _generate_traj_points(
            joint_pose: float, joint_vel: float, force: float
        ) -> TrajectoryPoint:
            self._idx_cnt += 1
            # Change rotation of last joint
            q = q0.copy()
            q[-1] = joint_pose
            dq = np.zeros(self._nv)
            dq[-1] = joint_vel
            # Change target force
            f = copy.copy(self._force_base)
            f.linear[2] = -force
            return TrajectoryPoint(
                id=self._idx_cnt,
                time_ns=0,
                robot_configuration=q,
                robot_velocity=dq,
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: T_final * f},
                end_effector_poses={self._tool_frame_id_name: T_final},
            )

        self._idx_cnt = -1
        j0 = q0[-1]
        f_final = self._positioning_force
        traj = []
        for i in range(self._n_repeat):
            if i != 0:
                # Move first joint to the limit
                j_traj, j_vel = self._s_curve_generator.compute_position_s_curve(
                    j0, self._tool_joint_start_angle
                )
                forces = np.ones_like(j_traj) * self._positioning_force
                traj += [
                    _generate_traj_points(q, dq, f)
                    for q, dq, f in zip(j_traj, j_vel, forces)
                ]

            # Increase the force to alignment one
            if i == 0:
                time_steps = ceil(self._positioning_time / self._ocp_dt)
                j_traj = np.ones(time_steps) * self._tool_joint_start_angle
                j_vel = np.zeros_like(j_traj)
                forces = self._compute_force_ramp(
                    0.0, self._positioning_force, self._positioning_force, time_steps
                )
                traj += [
                    _generate_traj_points(q, dq, f)
                    for q, dq, f in zip(j_traj, j_vel, forces)
                ]

            # Start moving while increasing the force.
            # Handle force profile is slower than motion!
            j_traj, j_vel = self._s_curve_generator.compute_position_s_curve(
                self._tool_joint_start_angle, self._tool_joint_end_angle
            )
            forces = self._compute_force_ramp(
                self._positioning_force, self._deburring_force, f_final, j_traj.shape[0]
            )
            traj += [
                _generate_traj_points(q, dq, f)
                for q, dq, f in zip(j_traj, j_vel, forces)
            ]
            j0 = self._tool_joint_end_angle

        return traj
