import numpy as np
import pinocchio as pin
from pathlib import Path

import numpy.typing as npt

from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
import copy

from agimus_controller.factory.robot_model import RobotModelParameters, RobotModels
from agimus_controller.ocp.ocp_croco_generic import (
    OCPParamsBaseCroco,
    OCPCrocoGeneric,
)
from agimus_controller.ocp_param_base import DTFactorsNSeq


class TrajecotryOptimizer:
    def __init__(
        self,
        robot_description: str,
        environment_description: str,
        moving_joints: list[str],
        ocp_params_path: Path,
        horizon: int,
        ocp_dt: float,
        n_threads: int,
        solver_iters: int,
        qp_iters: int,
        use_line_search: bool,
        callbacks: bool,
        collision_frames_id: list[str],
        ee_tool_frame: str,
        w_robot_velocity: npt.ArrayLike,
        w_robot_effort: npt.ArrayLike,
        running_w_robot_configuration: npt.ArrayLike,
        running_w_frame_rotation: npt.ArrayLike,
        running_w_frame_translation: npt.ArrayLike,
        terminal_w_robot_configuration: npt.ArrayLike,
        terminal_w_frame_rotation: npt.ArrayLike,
        terminal_w_frame_translation: npt.ArrayLike,
    ):
        self._ocp_dt = ocp_dt
        self._horizon = horizon

        robot_params = RobotModelParameters(
            robot_urdf=robot_description,
            env_urdf=environment_description,
            moving_joint_names=moving_joints,
            free_flyer=False,
            collision_as_capsule=True,
            self_collision=False,
            # Hardcode usual armature value to make the solver robust,
            # no need to expose that
            armature=np.ones(len(moving_joints)) * 0.1,
        )
        self._robot_models = RobotModels(robot_params)
        self._robot_model = self._robot_models.robot_model
        self._robot_data = self._robot_model.createData()
        self._nq = self._robot_model.nq
        self._nv = self._robot_model.nv

        self._ee_tool_frame = ee_tool_frame
        self._ee_tool_frame_pin = self._robot_models.robot_model.getFrameId(
            self._ee_tool_frame
        )

        dt_factor_n_seq = DTFactorsNSeq(
            factors=[1],
            n_steps=[self._horizon],
        )
        ocp_params = OCPParamsBaseCroco(
            dt_factor_n_seq=dt_factor_n_seq,
            dt=self._ocp_dt,
            horizon_size=self._horizon,
            solver_iters=solver_iters,
            callbacks=callbacks,
            qp_iters=qp_iters,
            use_debug_data=False,
            n_threads=n_threads,
        )

        self._ocp = OCPCrocoGeneric(self._robot_models, ocp_params, ocp_params_path)
        self._ocp._solver.use_filter_line_search = use_line_search

        # Update pose of all collisions used by OCP
        self._geometries = [
            (geom.name, geom.placement.copy(), pin.GeometryType.COLLISION)
            for geom in self._robot_models.collision_model.geometryObjects
            if geom.name in collision_frames_id
        ]

        weighted_traj_point_base = WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                time_ns=0,
                robot_configuration=np.zeros(self._nq),
                robot_velocity=np.zeros(self._nv),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={},
                end_effector_poses={self._ee_tool_frame: pin.SE3(np.eye(4))},
                end_effector_velocities={},
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=np.zeros(self._nq),
                w_robot_velocity=np.asarray(w_robot_velocity),
                w_robot_acceleration=np.zeros(7),
                w_robot_effort=np.asarray(w_robot_effort),
                w_collision_avoidance=0.0,
                w_forces={},
                w_end_effector_poses={self._ee_tool_frame: np.zeros(6)},
                w_end_effector_velocities={},
            ),
        )
        self._running_wpt = copy.deepcopy(weighted_traj_point_base)
        self._running_wpt.weights.w_robot_configuration = np.asarray(
            running_w_robot_configuration
        )
        self._running_wpt.weights.w_end_effector_poses[self._ee_tool_frame] = (
            np.concatenate(
                (
                    np.asarray(running_w_frame_translation),
                    np.asarray(running_w_frame_rotation),
                )
            )
        )

        self._terminal_wpt = copy.deepcopy(weighted_traj_point_base)
        self._terminal_wpt.weights.w_robot_configuration = np.asarray(
            terminal_w_robot_configuration
        )
        self._terminal_wpt.weights.w_end_effector_poses[self._ee_tool_frame] = (
            np.concatenate(
                (
                    np.asarray(terminal_w_frame_translation),
                    np.asarray(terminal_w_frame_rotation),
                )
            )
        )

    def __call__(
        self, trajectory: npt.ArrayLike, T_final: pin.SE3
    ) -> npt.ArrayLike | None:
        # Assemble initial state
        dq = np.zeros(7)
        x0 = np.hstack((trajectory[0], dq))

        # Precompute velocity to speedup computation
        velocities = np.gradient(trajectory, axis=0) / self._ocp_dt
        x_init = [np.hstack((q, dq)) for q, dq in zip(trajectory, velocities)]
        # Precompute gravity torque for better warmstart
        u_init = [
            pin.rnea(self._robot_model, self._robot_data, q, dq, dq)
            for q in trajectory[:-1]
        ]

        def _create_trajecotry_point(q, is_terminal):
            p = (
                copy.deepcopy(self._running_wpt)
                if not is_terminal
                else copy.deepcopy(self._terminal_wpt)
            )
            p.point.robot_configuration = q
            pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

            T_ee = (
                self._robot_data.oMf[self._ee_tool_frame_pin]
                if not is_terminal
                else T_final
            )

            p.point.end_effector_poses[self._ee_tool_frame] = T_ee
            return p

        # Set references for the OCP
        self._ocp.set_reference_weighted_trajectory(
            [
                _create_trajecotry_point(q, i == (self._horizon - 1))
                for i, q in enumerate(trajectory)
            ]
        )

        self._ocp.solve(x0, x_init, u_init)

        if self._ocp._debug_data.problem_solved:
            return None

        trajectory = np.array([x[: self._nq] for x in self._ocp.ocp_results.states])

        return trajectory

    def update_poses(self, T_pylone: pin.SE3) -> None:
        for name, placement, type in self._geometries:
            self._ocp.update_geometry_placement(name, T_pylone * placement, type)
