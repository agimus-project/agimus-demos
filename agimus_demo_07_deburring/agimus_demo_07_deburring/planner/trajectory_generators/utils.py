import numpy as np
import numpy.typing as npt


class SCurveGenerator:
    def __init__(
        self,
        max_vel: float,
        max_acc: float,
        max_jerk: float,
        ocp_dt: float,
    ):
        self._max_vel = max_vel
        self._max_acc = max_acc
        self._max_jerk = max_jerk
        self._ocp_dt = ocp_dt

    def compute_params(self, dist: float) -> tuple[float, float, float, float]:
        vel_max = self._max_vel
        acc_max = self._max_acc
        jerk_max = self._max_jerk
        time_jerk = acc_max / jerk_max
        time_acc = vel_max / acc_max + time_jerk

        dist_acc = (acc_max * time_acc**2) / 2 - (acc_max**3) / (6 * jerk_max**2)
        dist_cruise = dist - 2.0 * dist_acc

        time_cruise = dist_cruise / vel_max
        time_total = 2 * time_acc + time_cruise
        return time_total, time_cruise, time_acc, time_jerk, dist_cruise

    def compute_position_s_curve(
        self, start: float, end: float
    ) -> tuple[npt.ArrayLike, npt.ArrayLike]:
        dist = end - start
        dir = np.sign(dist)
        dist = np.abs(dist)
        acc_max = self._max_acc
        jerk_max = self._max_jerk
        time_total, time_cruise, time_acc, time_jerk, _ = self.compute_params(dist)

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

        return start + (pos * dir), vel * dir
