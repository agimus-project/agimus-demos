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
        self, trajectory: npt.ArrayLike, T_final: pin.SE3
    ) -> tuple[npt.ArrayLike, npt.ArrayLike, float]:
        trajectory = np.asarray(trajectory, dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != self._nq:
            raise ValueError(
                f"Expected a trajectory of shape (N, {self._nq}), "
                + f"got {trajectory.shape}."
            )

        if trajectory.shape[0] <= 1:
            velocities = np.zeros_like(trajectory)
            return trajectory.copy(), velocities, 1.0

        # Use a time parameterization driven by joint velocity limits so execution
        # speed depends on geometric distance, not on incoming point spacing.
        joint_lim = np.asarray(self._max_joint_velocity, dtype=float)
        if np.any(joint_lim <= 0.0):
            raise ValueError("Maximum joint velocities must be strictly positive.")

        delta_q = np.diff(trajectory, axis=0)
        segment_times = np.max(np.abs(delta_q) / joint_lim[np.newaxis, :], axis=1)
        cumulative_times = np.concatenate(([0.0], np.cumsum(segment_times)))
        total_time = cumulative_times[-1]

        # Keep at least two points to avoid degenerate interpolation.
        n_interp = max(int(np.ceil(total_time / self._ocp_dt)) + 1, 2)

        # Interpolate trajectory with uniform controller-rate timing.
        old_times = cumulative_times
        new_times = np.linspace(0.0, total_time, n_interp)
        interpolated = np.zeros((n_interp, self._nq))
        for i in range(self._nq):
            interpolated[:, i] = np.interp(new_times, old_times, trajectory[:, i])

        # Compute joint velocities from finite differences at controller timestep.
        velocities = np.gradient(interpolated, self._ocp_dt, axis=0)

        # Return dummy cost value
        return interpolated, velocities, 1.0

    @property
    def n_samples(self) -> int:
        # This smoother doesn't have fixed number of points so return invalid value
        return -1

    def update_poses(self, _: pin.SE3) -> None:
        # Do nothing, there is no way to anticipate changes in the environment in here
        pass
