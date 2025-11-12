import copy
from math import ceil
import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_07_deburring.planner.trajectory_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)


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
        tool_frame_id: str,
        measurement_frame_id: str,
    ) -> None:
        self._robot_model = robot_model

        self._tool_joint_end_angle = tool_joint_end_angle
        self._tool_joint_start_angle = tool_joint_start_angle
        self._tool_angular_vel = tool_angular_vel
        self._tool_angular_acc = tool_angular_acc
        self._tool_angular_jerk = tool_angular_jerk

        self._positioning_force = positioning_force
        self._deburring_force = deburring_force
        self._force_rate_up = force_rate_up
        self._force_rate_down = force_rate_down
        self._positioning_time = positioning_time
        self._ocp_dt = ocp_dt

        self._idx_cnt = 0

        motion_time_total, _, _, _, motion_dist_cruise = self._compute_s_curve_params(
            np.abs(self._tool_joint_end_angle - self._tool_joint_start_angle)
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

    def _compute_s_curve_params(self, dist: float):
        vel_max = self._tool_angular_vel
        acc_max = self._tool_angular_acc
        jerk_max = self._tool_angular_jerk
        time_jerk = acc_max / jerk_max
        time_acc = vel_max / acc_max + time_jerk

        dist_acc = (acc_max * time_acc**2) / 2 - (acc_max**3) / (6 * jerk_max**2)
        dist_cruise = dist - 2.0 * dist_acc

        time_cruise = dist_cruise / vel_max
        time_total = 2 * time_acc + time_cruise
        return time_total, time_cruise, time_acc, time_jerk, dist_cruise

    def _compute_position_s_curve(self, dist: float):
        acc_max = self._tool_angular_acc
        jerk_max = self._tool_angular_jerk
        time_total, time_cruise, time_acc, time_jerk, _ = self._compute_s_curve_params(
            dist
        )

        t = np.arange(0, time_total + self._ocp_dt, self._ocp_dt)
        pos = np.zeros_like(t)
        vel = np.zeros_like(t)
        acc = np.zeros_like(t)
        for i in range(1, len(t)):
            ti = t[i]

            # Acceleration phase
            if ti < time_jerk:
                acc[i] = jerk_max * ti
            elif ti < (time_acc - time_jerk):
                acc[i] = acc_max
            elif ti < time_acc:
                acc[i] = acc_max - jerk_max * (ti - (time_acc - time_jerk))
            # Cruise phase
            elif ti < (time_acc + time_cruise):
                acc[i] = 0.0
            # Deceleration phase
            elif ti < (time_acc + time_cruise + time_jerk):
                acc[i] = -jerk_max * (ti - (time_acc + time_cruise))
            elif ti < (time_total - time_jerk):
                acc[i] = -acc_max
            elif ti <= time_total:
                acc[i] = -acc_max + jerk_max * (ti - (time_total - time_jerk))
            else:
                acc[i] = 0.0

            # Integrate to get velocity and position
            vel[i] = vel[i - 1] + acc[i] * self._ocp_dt
            pos[i] = pos[i - 1] + vel[i] * self._ocp_dt

        return pos, vel

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
        # Move first joint to the limit
        dist = self._tool_joint_start_angle - q0[-1]
        dir = np.sign(dist)
        j_traj, j_vel = self._compute_position_s_curve(np.abs(dist))
        forces = np.zeros_like(j_traj)
        t1 = [
            _generate_traj_points(q, dq, f)
            for q, dq, f in zip(q0[-1] + j_traj * dir, j_vel * dir, forces)
        ]

        # Increase the force to alignment one
        time_steps = ceil(self._positioning_time / self._ocp_dt)
        j_traj = np.ones(time_steps) * self._tool_joint_start_angle
        j_vel = np.zeros_like(j_vel)
        forces = self._compute_force_ramp(
            0.0, self._positioning_force, self._positioning_force, time_steps
        )
        t2 = [
            _generate_traj_points(q, dq, f) for q, dq, f in zip(j_traj, j_vel, forces)
        ]

        # Start moving while increasing the force. Handle force profile is slower than motion!
        dist = self._tool_joint_end_angle - self._tool_joint_start_angle
        j_traj, j_vel = self._compute_position_s_curve(np.abs(dist))
        forces = self._compute_force_ramp(
            self._positioning_force, self._deburring_force, 0.0, j_traj.shape[0]
        )
        t3 = [
            _generate_traj_points(q, dq, f)
            for q, dq, f in zip(j_traj + self._tool_joint_start_angle, j_vel, forces)
        ]

        return t1 + t2 + t3
