import copy
from pathlib import Path
import tempfile
import os

import numpy as np
import numpy.typing as npt
import pinocchio as pin
import yaml
from agimus_controller.trajectory import TrajectoryPoint
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    ProblemSolver,
    Robot,
    Rule,
    SecurityMargins,
    loadServerPlugin,
)  # noqa: F811
from hpp.gepetto.manipulation import ViewerFactory

from agimus_demo_08_polishing.planner.hpp.path_planner import PathPlanner
from agimus_demo_08_polishing.planner.trajectory_generators.trajectory_generator import (
    JointSpaceMotionGenerator,
)

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


class BaseObject(object):
    rootJointType = "freeflyer"

    def __init__(self, urdf_path: str, srdf_path: str, name: str):
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator(JointSpaceMotionGenerator):
    def __init__(
        self,
        handles_configurations_path: Path,
        robot_description: str,
        environment_description: str,
        ocp_dt: float,
        tool_frame_id: str,
        robot_model: pin.Model,
        robot_self_collision_config: Path,
        handles_srdf_path: Path,
        demo_config: Path,
        robot_name: str,
        gripper_name: str,
        polished_object_name: str,
        joints_to_shrink: list[str],
        joint_shrink_range: float,
    ) -> None:
        self._ocp_dt = ocp_dt
        self._nq = robot_model.nq
        self._nv = robot_model.nv

        self._robot_model = robot_model
        self._robot_data = robot_model.createData()

        self._tool_frame_id_name = tool_frame_id
        self._tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._tool_frame_id_name
        )

        self._polished_object_name = polished_object_name

        # Load the robot
        Robot.urdfString = robot_description
        Robot.srdfString = robot_self_collision_config.read_text()

        with open(handles_configurations_path, "r") as stream:
            self._handles_q_guesses = yaml.safe_load(stream)

        # Create temporary file with a content of the environment URDF
        with tempfile.NamedTemporaryFile(
            mode="w",
            prefix=f"{polished_object_name}_urdf_",
            suffix=".urdf",
            delete=False,
        ) as f:
            f.write(environment_description)
            self._tmp_polished_object_urdf_file = Path(f.name)

        # Load the object
        polished_object_object = BaseObject(
            urdf_path=self._tmp_polished_object_urdf_file.as_posix(),
            srdf_path=handles_srdf_path.as_posix(),
            name=polished_object_name,
        )

        robot = Robot("robot", robot_name, rootJointType="anchor")
        robot.client.manipulation.robot.insertRobotSRDFModel(
            robot_name,
            demo_config.as_posix(),
        )

        self._ps = ProblemSolver(robot)
        self._ps.loadPlugin("manipulation-spline-gradient-based.so")

        vf = ViewerFactory(self._ps)
        vf.loadObjectModel(polished_object_object, polished_object_object.name)
        self._v = vf.createViewer()

        robot.setJointBounds(
            f"{polished_object_object.name}/root_joint",
            [-1.0, 1.0, -1.0, 1.0, 0.0, 2.2],
        )

        shrinkJointRange(
            robot,
            [f"{robot_name}/{joint_name}" for joint_name in joints_to_shrink],
            joint_shrink_range,
        )

        # Set problem solver params
        self._ps.setErrorThreshold(1e-2)
        self._ps.setMaxIterProjection(20)

        self._ps.setParameter("StatesPathFinder/maxDepth", 2)

        # Load plugins
        self._ps.addPathOptimizer("RandomShortcut")

        self._ps.addPathOptimizer("SimpleTimeParameterization")
        self._ps.setParameter("SimpleTimeParameterization/order", 2)
        self._ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
        # self._ps.setParameter("SimpleTimeParameterization/safety", 0.005)

        cg = ConstraintGraph(robot, "graph")
        factory = ConstraintGraphFactory(cg)
        factory.setGrippers([f"{robot_name}/{gripper_name}"])
        factory.setObjects(
            [polished_object_name],
            [
                [
                    f"{polished_object_name}/{handle}"
                    for handle in self._handles_q_guesses.keys()
                ]
            ],
            [[]],
        )
        factory.setRules([Rule([".*"], [".*"], True)])
        factory.generate()

        sm = SecurityMargins(self._ps, factory, [robot_name, polished_object_name])
        sm.setSecurityMarginBetween(robot_name, polished_object_name, 0.01)
        sm.setSecurityMarginBetween(robot_name, robot_name, 0)
        sm.defaultMargin = 0.03
        sm.apply()

        cg.initialize()

        self._q_init = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.45,
            -0.116,
            0.739,
            0.0,
            0.0,
            0.0,
            1.0,
        ]

        self._v(self._q_init)

        self._path_planner = PathPlanner(self._ps, cg, f"{robot_name}/{gripper_name}")

    def __del__(self):
        # Remove temp file with the URDF
        try:
            os.unlink(self._tmp_polished_object_urdf_file)
        except FileNotFoundError:
            pass

    def update_polished_object_pose(self, T_polished_object: pin.SE3) -> None:
        xyzquat = pin.SE3ToXYZQUAT(T_polished_object)
        self._q_init[-7:-4] = xyzquat[:3]
        self._q_init[-4:] = xyzquat[3:]

    def get_path(
        self,
        q0,
        T_final: pin.SE3,
        handle_name: str,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        self._q_init[: self._nq] = q0
        self._v(self._q_init)
        if not self._path_planner.checkConfigurationValid(self._q_init):
            raise RuntimeError("Invalid initial configuration!")

        if handle_name not in self._handles_q_guesses.keys():
            raise RuntimeError("Invalid handle name!")

        q_guess = self._q_init.copy()
        q_guess[: self._nq] = self._handles_q_guesses[handle_name]
        p = self._path_planner.planPregrasp(
            f"{self._polished_object_name}/{handle_name}", self._q_init, [q_guess]
        )
        # Solution was not found
        if p is None:
            raise RuntimeError("Failed to find path!")
        self._ps.client.basic.problem.addPath(p)

        length = p.length()
        n_traj_points = int(np.ceil(length / self._ocp_dt)) * 4
        trajectory = np.array(
            [p.call(i * self._ocp_dt / 4)[0][: self._nq] for i in range(n_traj_points)]
        )
        p.deleteThis()

        # velocities = np.gradient(trajectory, axis=0) / self._ocp_dt

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            q = trajectory[i, :]
            pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q,
                robot_velocity=np.zeros_like(q),
                robot_acceleration=np.zeros(self._nv),
                robot_effort=np.zeros(self._nv),
                forces={},
                end_effector_poses={
                    self._tool_frame_id_name: copy.copy(
                        self._robot_data.oMf[self._tool_frame_id_pin_frame]
                    )
                },
            )

        return [_create_trajectory_point(i) for i in range(n_traj_points)]
