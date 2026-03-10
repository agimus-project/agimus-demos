import copy
from pathlib import Path

import numpy as np
import numpy.typing as npt
import pinocchio as pin
import torch
from agimus_controller.trajectory import TrajectoryPoint

from agimus_demo_07_deburring.planner.trajectory_generators.trajectory_generator import (
    JointSpaceMotionGenerator,
)
from agimus_demo_07_deburring.planner.trajectory_smoothers.trajectory_smoother import (
    GenericTrajectorySmoother,
)

from agimus_demo_07_deburring.planner.diffusion.model import Model


class DiffusionPathGenerator(JointSpaceMotionGenerator):
    def __init__(
        self,
        wights_path: Path,
        robot_model: pin.Model,
        ocp_dt: float,
        max_joint_velocity: npt.ArrayLike,
        sequence_length: int,
        n_samples: int,
        tool_frame_id: str,
        hpp_handle_correction: pin.SE3,
        trajectory_smoother: GenericTrajectorySmoother | None,
    ) -> None:
        self._robot_model = robot_model
        self._robot_data = robot_model.createData()
        self._hpp_handle_correction = hpp_handle_correction

        if trajectory_smoother is None:
            raise RuntimeError(
                "DiffusionPathGenerator requires setting a trajectory smoother!"
            )

        self._trajectory_smoother = trajectory_smoother

        self._model = Model.load_from_checkpoint(wights_path)
        self._model.eval()
        self._model.cuda()

        self._nq = robot_model.nq
        self._nv = robot_model.nv
        self._ocp_dt = ocp_dt
        self._max_joint_velocity = max_joint_velocity
        self._sequence_length = sequence_length
        self._n_samples = n_samples

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

    def update_deburred_object_pose(self, T_deburred_object: pin.SE3) -> None:
        self._trajectory_smoother.update_poses(T_deburred_object)

    def get_path(
        self,
        q0: npt.ArrayLike,
        T_final: pin.SE3,
        handle_name: str | None = None,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        target_xyzquat = pin.SE3ToXYZQUAT(T_final * self._hpp_handle_correction)
        cond_dict = {
            "q0": torch.from_numpy(q0).float().unsqueeze(0).cuda(),
            "goal": torch.from_numpy(target_xyzquat).float().unsqueeze(0).cuda(),
        }

        with torch.no_grad():
            sampled_trajs = self._model.sample(
                cond=cond_dict,
                bs=self._n_samples,
                seq_length=self._sequence_length,
                configuration_size=self._nq,
            )

        for trajectory in sampled_trajs:
            trajectory, velocities = self._trajectory_smoother(trajectory.cpu().numpy())
            # If trajectory is nicely smoothed, use it
            if trajectory is not None:
                break

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            pin.framesForwardKinematics(
                self._robot_model, self._robot_data, trajectory[i, :]
            )
            pin.updateFramePlacement(
                self._robot_model, self._robot_data, self._tool_frame_id_pin_frame
            )

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=trajectory[i, :],
                robot_velocity=velocities[i, :],
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={},
                end_effector_poses={
                    self._tool_frame_id_name: copy.copy(
                        self._robot_data.oMf[self._tool_frame_id_pin_frame]
                    )
                },
            )

        return [_create_trajectory_point(i) for i in range(trajectory.shape[0])]
