import copy
from pathlib import Path

import numpy as np
import numpy.typing as npt
import pinocchio as pin
import yaml
from agimus_controller.trajectory import TrajectoryPoint
from ament_index_python.packages import get_package_share_directory
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    ProblemSolver,
    Robot,
    Rule,
    loadServerPlugin,
)  # noqa: F811
from hpp.gepetto.manipulation import ViewerFactory

from agimus_demo_07_deburring.planner.hpp.path_planner import PathPlanner
from agimus_demo_07_deburring.planner.trajecotry_generators.trajectory_generator import (
    GenericTrajectoryGenerator,
)

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


class BaseObject(object):
    rootJointType = "freeflyer"

    def __init__(self, urdf_path: str, srdf_path: str, name: str):
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator(GenericTrajectoryGenerator):
    def __init__(
        self,
        handles_configurations_path: Path,
        robot_description: str,
        ocp_dt: float,
        tool_frame_id: str,
        robot_model: pin.Model,
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

        # Load the robot
        pkg = Path(get_package_share_directory("agimus_demo_07_deburring"))
        Robot.urdfString = robot_description
        Robot.srdfString = (pkg / "environment" / "panda.srdf").read_text()

        with open(handles_configurations_path, "r") as stream:
            self._handles_q_guesses = yaml.safe_load(stream)

        # Load the object
        pylone_object = BaseObject(
            urdf_path=(pkg / "environment" / "pylone.urdf").as_posix(),
            srdf_path=(pkg / "environment" / "pylone.srdf").as_posix(),
            name="pylone",
        )

        robot = Robot("robot", "panda", rootJointType="anchor")
        robot.client.manipulation.robot.insertRobotSRDFModel(
            "panda",
            (pkg / "environment" / "demo.srdf").as_posix(),
        )

        ps = ProblemSolver(robot)
        ps.loadPlugin("manipulation-spline-gradient-based.so")

        vf = ViewerFactory(ps)
        vf.loadObjectModel(pylone_object, pylone_object.name)

        robot.setJointBounds(
            f"{pylone_object.name}/root_joint", [-1.0, 1.0, -1.0, 1.0, 0.0, 2.2]
        )
        shrinkJointRange(robot, [f"panda/fer_joint{i}" for i in range(1, 8)], 0.95)

        # Set problem solver params
        ps.setErrorThreshold(1e-3)
        ps.setMaxIterProjection(40)

        # Load plugins
        ps.addPathOptimizer("RandomShortcut")

        ps.addPathOptimizer("SimpleTimeParameterization")
        ps.setParameter("SimpleTimeParameterization/order", 2)
        ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
        ps.setParameter("SimpleTimeParameterization/safety", 0.05)

        cg = ConstraintGraph(robot, "graph")
        factory = ConstraintGraphFactory(cg)
        factory.setGrippers(["panda/panda_gripper"])
        factory.setObjects(
            [pylone_object.name],
            [[f"pylone/{handle}" for handle in self._handles_q_guesses.keys()]],
            [[]],
        )
        factory.setRules([Rule([".*"], [".*"], True)])
        factory.generate()
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

        self._path_planner = PathPlanner(ps, cg, "panda/panda_gripper")

    def get_path(
        self,
        q0,
        T_final: pin.SE3,
        handle_name: str,
        T_init: pin.SE3 | None = None,
    ) -> list[npt.ArrayLike]:
        self._q_init[: self._nq] = q0
        if not self._path_planner.checkConfigurationValid(self._q_init):
            raise RuntimeError("Invalid initial configuration!")

        if handle_name not in self._handles_q_guesses.keys():
            raise RuntimeError("Invalid handle name!")

        q_guess = self._q_init.copy()
        q_guess[: self._nq] = self._handles_q_guesses[handle_name]
        p = self._path_planner.planPregrasp(
            f"pylone/{handle_name}", self._q_init, [q_guess]
        )
        # Solution was not found
        if p is None:
            raise RuntimeError("Failed to find path!")

        length = p.length()
        n_traj_points = int(np.ceil(length / self._ocp_dt)) * 4
        trajecotry = np.array(
            [p.call(i * self._ocp_dt / 4)[0][: self._nq] for i in range(n_traj_points)]
        )
        p.deleteThis()

        velocities = np.gradient(trajecotry, axis=0) / self._ocp_dt

        def _create_trajectory_point(i: int) -> TrajectoryPoint:
            q = trajecotry[i, :]
            pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

            return TrajectoryPoint(
                id=i,
                time_ns=0,
                robot_configuration=q,
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

        return [_create_trajectory_point(i) for i in range(n_traj_points)]
