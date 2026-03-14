import numpy as np
import pinocchio as pin
import numpy.typing as npt

from agimus_demo_07_deburring.planner.trajecory_smoothers.trajectory_smoother import (
    GenericTrajectorySmoother,
)


class BasicInterpolationSmoother(GenericTrajectorySmoother):
    def __init__(
        self,
        robot_model: pin.Model,
        ocp_dt: float,
        max_joint_velocity: npt.ArrayLike,
    ) -> None:
        self._robot_model = robot_model
        self._nq = robot_model.nq
        self._ocp_dt = ocp_dt
        self._max_joint_velocity = max_joint_velocity

    def __call__(
        self, trajectory: npt.ArrayLike
    ) -> tuple[npt.ArrayLike, npt.ArrayLike]:
        # Compute highest joint velocities between trajectory points
        trajectory_vel = np.gradient(trajectory, axis=0)
        trajectory_vel_max = np.max(np.abs(trajectory_vel), axis=0)
        # Get rescaling between computed velocities and maximum allowed values
        joint_lim = np.asarray(self._max_joint_velocity)
        velocity_scale = trajectory_vel_max / joint_lim
        # Compute minimum number of interpolation steps to not violate joint
        # velocity limits
        n_interp = np.ceil(velocity_scale / self._ocp_dt * trajectory.shape[0])
        # Choose minimum of already predicted steps for the interpolation
        n_interp = int(np.max(np.append(n_interp, trajectory.shape[0])))

        # Interpolate trajectories
        old_times = np.linspace(0, 1, trajectory.shape[0])
        new_times = np.linspace(0, 1, n_interp)
        interpolated = np.zeros((n_interp, self._nq))

        for i in range(self._nq):
            interpolated[:, i] = np.interp(new_times, old_times, trajectory[:, i])

        # Compute joint velocities from a trajectory using finite differences.
        velocities = np.gradient(interpolated, axis=0) / self._ocp_dt

        return interpolated, velocities

    @property
    def n_samples(self) -> int:
        # This smoother doesn't have fixed number of points so return invalid value
        return -1

    def update_poses(self, _: pin.SE3) -> None:
        # Do nothing, there is no way to anticipate changes in the environment in here
        pass
