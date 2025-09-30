import copy
import numpy as np
import numpy.typing as npt
import pinocchio as pin
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_07_deburring.planner.trajecotry_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)


class DeburringPathGenerator(GenericTrajectoryGenerator):
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

        self._angle = angle
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

        self._omega = 2.0 * np.pi * self._frequency
        circle_samples = self._omega / self._ocp_dt
        self._slope_phase_samples = circle_samples * self._slope_circles
        self._deburr_phase_samples = circle_samples * (
            self._n_circles - 2 * self._slope_circles
        )
        self._ramp_samples = int(self._force_ramp / self._ocp_dt)
        self._n_samples = int(circle_samples * self._n_circles)

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

        def _generate_force_ramp(i: int, ascend: bool) -> TrajectoryPoint:
            if ascend:
                f = self._desired_force * (i * self._ocp_dt) / self._force_ramp
            else:
                f = (
                    self._desired_force
                    * (self._force_ramp - (i * self._ocp_dt))
                    / self._force_ramp
                )
            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: T_final * f},
                end_effector_poses={self._tool_frame_id_name: T_final},
            )

        def _generate_sequence(i: int) -> list[TrajectoryPoint]:
            t = i * self._ocp_dt
            phase = self._omega * t

            if i < self._slope_phase_samples:
                tilt = (i / self._slope_phase_samples) * self._angle
            elif i > self._slope_phase_samples and i < (
                self._slope_phase_samples + self._deburr_phase_samples
            ):
                tilt = self._angle
            else:
                tilt = (
                    (
                        self._slope_phase_samples
                        - (i - self._slope_phase_samples - self._deburr_phase_samples)
                    )
                    / self._slope_phase_samples
                ) * self._angle
            R_1 = pin.SE3(pin.rpy.rpyToMatrix(np.array([0.0, 0.0, phase])), np.zeros(3))
            R_2 = pin.SE3(pin.rpy.rpyToMatrix(np.array([tilt, 0.0, 0.0])), np.zeros(3))
            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q0,
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={self._measurement_frame_id: T_final * self._desired_force},
                end_effector_poses={self._tool_frame_id_name: T_final * R_1 * R_2},
            )

        t1 = [_generate_force_ramp(i, ascend=True) for i in range(self._ramp_samples)]
        t2 = [copy.copy(t1[-1]) for _ in range(int(1.0 / self._ocp_dt))]
        t3 = [_generate_sequence(i) for i in range(self._n_samples)]
        t4 = [_generate_force_ramp(i, ascend=False) for i in range(self._ramp_samples)]

        return t1 + t2 + t3 + t4
