import numpy as np
import yaml
import copy
import pinocchio as pin
import numpy.typing as npt
from pathlib import Path
from tempfile import NamedTemporaryFile

from agimus_demo_07_deburring.planner.trajecory_smoothers.trajectory_smoother import (
    GenericTrajectorySmoother,
)
from agimus_demo_07_deburring.planner.trajecory_smoothers.basic_interpolation import (
    BasicInterpolationSmoother,
)
from agimus_demo_07_deburring.planner.ocp.trajectory_optimizer import (
    TrajecotryOptimizer,
)


class OCPSmoother(GenericTrajectorySmoother):
    def __init__(
        self,
        robot_model: pin.Model,
        robot_description: str,
        environment_description: str,
        ocp_dt: float,
        max_joint_velocity: npt.ArrayLike,
        optimizer_ocp_dt: float,
        optimizer_ocp_horizon: int,
        running_costs: list[str],
        terminal_costs: list[str],
        robot_collision_links: list[str],
        environment_links: list[str],
        collision_distance: float,
        joint_shrink_range: float,
        n_threads: int,
        solver_iters: int,
        qp_iters: int,
        use_line_search: bool,
        callbacks: bool,
        parent_collision_frame_id: str,
        extra_collision_frames_id: list[str],
        ee_tool_frame: str,
        running_w_robot_configuration: npt.ArrayLike,
        terminal_w_robot_configuration: npt.ArrayLike,
        w_robot_velocity: npt.ArrayLike,
        w_robot_effort: npt.ArrayLike,
        running_w_frame_rotation: npt.ArrayLike,
        running_w_frame_translation: npt.ArrayLike,
        terminal_w_frame_rotation: npt.ArrayLike,
        terminal_w_frame_translation: npt.ArrayLike,
        moving_joints: list[str],
    ) -> None:
        self._optimizer_ocp_horizon = optimizer_ocp_horizon

        ocp_params_path = self._generate_ocp_config(
            robot_model,
            running_costs,
            terminal_costs,
            robot_collision_links,
            environment_links,
            collision_distance,
            joint_shrink_range,
        )

        self._optimizer = TrajecotryOptimizer(
            robot_description,
            environment_description,
            moving_joints,
            ocp_params_path,
            optimizer_ocp_horizon,
            optimizer_ocp_dt,
            n_threads,
            solver_iters,
            qp_iters,
            use_line_search,
            callbacks,
            parent_collision_frame_id,
            extra_collision_frames_id,
            ee_tool_frame,
            running_w_robot_configuration,
            terminal_w_robot_configuration,
            w_robot_velocity,
            w_robot_effort,
            running_w_frame_rotation,
            running_w_frame_translation,
            terminal_w_frame_rotation,
            terminal_w_frame_translation,
        )

        self._interpolation_smoother = BasicInterpolationSmoother(
            robot_model,
            ocp_dt,
            max_joint_velocity,
        )

    def __call__(
        self, trajectory: npt.ArrayLike
    ) -> tuple[npt.ArrayLike, npt.ArrayLike] | None:
        optimized_trajectory = self._optimizer(trajectory)
        if optimized_trajectory is None:
            return None, None

        return self._interpolation_smoother(optimized_trajectory)

    @property
    def n_samples(self) -> int:
        return self._optimizer_ocp_horizon

    def _generate_ocp_config(
        self,
        robot_model: pin.Model,
        running_costs: list[str],
        terminal_costs: list[str],
        robot_collision_links: list[str],
        environment_links: list[str],
        collision_distance: float,
        joint_shrink_range: float,
    ) -> Path:
        # Permute all collision pairs
        collision_pairs = []
        for rl in robot_collision_links:
            for env in environment_links:
                collision_pairs.append((rl, env, collision_distance))

        constraints = []
        for i, (a, b, lim) in enumerate(collision_pairs):
            constraints.append(
                self._create_constraint(
                    self._create_constraint(
                        f"collision_{i}",
                        lim,
                        "inf",
                        {
                            "class": "ResidualDistanceCollision",
                            "collision_pair": [a, b],
                        },
                    )
                )
            )

        # Absurdly large number to avoid using "inf"
        vel_lim = -10000000.0
        x_min = (
            np.array(robot_model.lowerPositionLimit) * joint_shrink_range
        ).tolist() + [vel_lim] * robot_model.nv
        x_max = (
            np.array(robot_model.upperPositionLimit) * joint_shrink_range
        ).tolist() + [vel_lim] * robot_model.nv

        running_model = {
            "class": "IntegratedActionModelEuler",
            "differential": {
                self._create_cost(self._camel_to_snake(cost), cost)
                for cost in running_costs
            },
            "constraints": [
                self._create_constraint(
                    "state", x_min, x_max, {"class": "ResidualModelState"}
                )
            ],
        }
        running_model["differential"]["constraints"].extend(copy.deepcopy(constraints))

        terminal_model = {
            "class": "IntegratedActionModelEuler",
            "differential": {
                self._create_cost(self._camel_to_snake(cost), cost)
                for cost in terminal_costs
            },
            "constraints": [
                self._create_constraint(
                    "state", x_min, x_max, {"class": "ResidualModelState"}
                )
            ],
        }
        terminal_model["differential"]["constraints"].extend(copy.deepcopy(constraints))

        data = {"running_model": running_model, "terminal_model": terminal_model}

        param_file_path = Path()
        with NamedTemporaryFile(
            mode="w", prefix="agimus_demo_07_path_optimizer_params_", delete=False
        ) as f:
            param_file_path = Path(f.name)
            yaml.dump(data, param_file_path, sort_keys=False)

        return param_file_path

    def _camel_to_snake(self, text: str) -> str:
        out = "".join(["_" + c.lower() if c.isupper() else c for c in text])

        # Remove leading underscore if present
        if out.startswith("_"):
            out = out[1:]
        return out

    def _create_cost(self, name: str, residual_class: str) -> str:
        return {
            "name": name,
            "update": True,
            "weight": 1.0,
            "cost": {
                "class": "CostModelResidual",
                "activation": {
                    "class": "ActivationModelWeightedQuad",
                    "weights": 1.0,
                },
                "residual": {"class": residual_class},
            },
        }

    def _create_constraint(
        self,
        name: str,
        lower: str | float | list,
        upper: str | float | list,
        residual: dict[str],
    ) -> str:
        return {
            "name": name,
            "active": True,
            "constraint": {
                "class": "ConstraintModelResidual",
                "lower": lower,
                "upper": upper,
                "residual": residual,
            },
        }

    def update_poses(self, T_pylone: pin.SE3) -> None:
        self._optimizer.update_poses(T_pylone)
