# Copyright 2022 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import typing as T
from math import sqrt

# Used if hppcorbaserver is not running in separate script
from agimus_demo_05_pick_and_place.corba import CorbaServer
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import Robot, newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from agimus_demo_05_pick_and_place.bin_picking import BinPicking
from agimus_demo_05_pick_and_place.utils import concatenatePaths, split_path
import numpy as np
from hpp_idl.hpp.core_idl import Path as HPPPath

import time

from agimus_demo_05_pick_and_place.utils import (
    BaseObject,
    get_obj_goal_handles,
    XYZQuatType,
    hack_for_ros2_support_in_hpp,
)
from hpp.rostools import process_xacro, retrieve_resource
import pinocchio

corba = None


class HPPInterface:
    """This interface assumes that there is one object to manipulate and one moving obstacle.
    Both the object and the obstacle are encoded in the space
    Graph creation is specific to this setup
    """

    def __init__(
        self,
        object_name: str = "obj_01",
        robot_urdf_string: str = "",
        robot_srdf_string: str = "",
        start_obj_pose: XYZQuatType = [0.0, -0.2, 0.85, 0.0, 0.0, 0.0, 1.0],
        use_spline_gradient_based_opt: bool = True,
        gripper_open_value: float = 0.04,
        source_bin_pose: XYZQuatType = [0.06, -0.2, 0.761, 0.0, 0.0, 0.0, 1.0],
        destination_bin_pose: XYZQuatType = [0.5, 0.3, 0.761, 0.0, 0.0, 0.0, 1.0],
    ):
        hack_for_ros2_support_in_hpp()

        self.use_spline_gradient_based_opt = use_spline_gradient_based_opt
        self.start_obj_pose = start_obj_pose
        self._goal_obj_pose = None
        self.gripper_open_value = gripper_open_value

        self._goal_gripper_clearance = 0.4
        self._after_picking_clearance = 0.4
        self._point_cloud_res = 0.001

        self.default_obstacle_pose = source_bin_pose
        self.default_obstacle2_pose = destination_bin_pose
        self.default_object_bounds = [-1.0, 1.5, -1.0, 1.0, 0.0, 2.2]
        package_location = "package://agimus_demo_05_pick_and_place"
        urdf_string = (
            process_xacro(package_location + "/urdf/demo.urdf.xacro")
            if robot_urdf_string == ""
            else robot_urdf_string
        )
        Robot.urdfString = urdf_string
        Robot.srdfString = robot_srdf_string

        self.manip_object = BaseObject(
            urdf_path=retrieve_resource(
                f"{package_location}/urdf/tless/{object_name}.urdf"
            ),
            srdf_path=retrieve_resource(
                f"{package_location}/srdf/tless/{object_name}.srdf"
            ),
            name="part",
        )
        self.obstacle_object = BaseObject(
            urdf_path=retrieve_resource(f"{package_location}/urdf/big_box.urdf"),
            srdf_path=retrieve_resource(f"{package_location}/srdf/big_box.srdf"),
            name="source_box",
        )
        self.obstacle2_object = BaseObject(
            urdf_path=retrieve_resource(f"{package_location}/urdf/big_box.urdf"),
            srdf_path=retrieve_resource(f"{package_location}/srdf/big_box.srdf"),
            name="dest_box",
        )
        # Init corbaserver
        global corba
        if corba is None:
            corba = CorbaServer()
        else:
            corba.restart()
        self.setup_problem()

    def set_relative_start_obj_pose(
        self, obj_pose_in_frame: XYZQuatType, q_robot: T.List[float], frame_name: str
    ):
        frame_pose = pinocchio.XYZQUATToSE3(
            self.get_robot_link_position(q_robot, frame_name)
        )
        pose = pinocchio.SE3ToXYZQUAT(
            frame_pose * pinocchio.XYZQUATToSE3(obj_pose_in_frame)
        )
        pose[3:] = pose[3:] / np.linalg.norm(pose[3:])
        self.start_obj_pose = pose.tolist()

    @property
    def goal_obj_pose(self) -> T.Tuple[float, float, float, float, float, float, float]:
        """Returns a list of size 7 contains the pose of the goal,
        defined in the destination box frame.

        The pose is expressed as [tx, ty, tz, qx, qy, qz, qw]
        """
        return self._goal_obj_pose

    def set_goal_obj_pose(self, frame_name: str, pose: T.Tuple[float, float, float]):
        """Sets the position of the goal wrt `frame_name`.

        Only the translation can be changed so only the 3 first element of the input
        are use.
        """
        if self._goal_obj_pose is not None:
            raise RuntimeError("Cannot reset the goal object pose")
        self._goal_obj_pose = pose[:3] + [0, sqrt(2) / 2, 0, -sqrt(2) / 2]
        self.ps.client.manipulation.robot.addGripper(
            frame_name,
            "goal/gripper",
            self._goal_obj_pose,
            self._goal_gripper_clearance,
        )

    def set_robot(self):
        self.robot = Robot("robot", "panda", rootJointType="anchor")
        # self.robot.opticalFrame = "camera_color_optical_frame"
        # TODO: get joint names automatically
        shrinkJointRange(self.robot, [f"panda/fer_joint{i}" for i in range(1, 8)], 0.95)

    def set_problem(self):
        # Setup problem solver and parameters
        self.ps = ProblemSolver(self.robot)
        if self.use_spline_gradient_based_opt:
            self.ps.loadPlugin("spline-gradient-based.so")
        self.ps.addPathOptimizer("EnforceTransitionSemantic")
        if self.use_spline_gradient_based_opt:
            self.ps.addPathOptimizer("SplineGradientBased_bezier5")
        else:
            # TODO Should we always use SimpleTimeParameterization?
            self.ps.addPathOptimizer("SimpleTimeParameterization")
        self.ps.setParameter("SimpleTimeParameterization/order", 2)
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.7)
        self.ps.setParameter("SimpleTimeParameterization/safety", 0.95)
        self.ps.setParameter("BiRRT*/maxStepLength", 0.5 * float(np.sqrt(7)))

        # Add path projector to avoid discontinuities
        self.ps.selectPathProjector("Progressive", 0.05)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)

        self.vf = ViewerFactory(self.ps)

        # load the object
        self.vf.loadObjectModel(self.manip_object, self.manip_object.name)
        self.robot.setJointBounds(
            f"{self.manip_object.name}/root_joint", self.default_object_bounds
        )

        # load moving obstacle
        self.vf.loadObjectModel(self.obstacle_object, self.obstacle_object.name)
        self.vf.loadObjectModel(self.obstacle2_object, self.obstacle2_object.name)
        self.robot.setJointBounds(
            f"{self.obstacle_object.name}/root_joint", self.default_object_bounds
        )
        self.robot.setJointBounds(
            f"{self.obstacle2_object.name}/root_joint", self.default_object_bounds
        )
        print("Part and box loaded")
        self.robot.client.manipulation.robot.insertRobotSRDFModel(
            "panda",
            retrieve_resource("package://agimus_demo_05_pick_and_place/srdf/demo.srdf"),
        )
        # Remove collisions between object and self collision geometries
        # TODO: get link names automatically
        srdfString = '<robot name="demo">'
        for i in range(1, 8):
            srdfString += f'<disable_collisions link1="panda_link{i}_sc" link2="{self.manip_object.name}/base_link" reason="handled otherwise"/>'
        srdfString += "</robot>"
        self.robot.client.manipulation.robot.insertRobotSRDFModelFromString(
            "panda", srdfString
        )

        # Lock gripper in open position.
        self.ps.createLockedJoint(
            "locked_finger_1", "panda/fer_finger_joint1", [self.gripper_open_value]
        )
        self.ps.createLockedJoint(
            "locked_finger_2", "panda/fer_finger_joint2", [self.gripper_open_value]
        )
        self.ps.setConstantRightHandSide("locked_finger_1", True)
        self.ps.setConstantRightHandSide("locked_finger_2", True)

        for name, position in (
            ("source_box", self.default_obstacle_pose),
            ("dest_box", self.default_obstacle2_pose),
        ):
            lj_name = f"locked_{name}"
            self.ps.createLockedJoint(lj_name, f"{name}/root_joint", position)
            self.ps.setConstantRightHandSide(lj_name, True)
        # Add handle of the objects
        self.handles, self.goal_handles = get_obj_goal_handles(
            prefix=self.manip_object.name + "/",
            srdf_path=self.manip_object.srdfFilename,
        )

    def setup_problem(self):
        newProblem()
        self.set_robot()
        self.set_problem()

    def get_robot_link_position(
        self,
        q_robot: T.List[float],
        frame_name: str,
    ) -> T.List[float]:
        """Get the position of a robot frame"""
        # TODO don't assume q_robot is of right size.
        q = self.robot.getCurrentConfig()
        q[: len(q_robot)] = q_robot
        (frame_position,) = self.robot.client.basic.robot.getLinksPosition(
            q, [frame_name]
        )
        return frame_position

    def set_point_cloud(
        self,
        q_robot: T.List[float],
        camera_frame_name: str,
        points: T.List[T.Tuple[float, float, float]],
        colors: T.Optional[T.List[T.Tuple[float, float, float, float]]] = None,
    ):
        frame_position = self.get_robot_link_position(q_robot, camera_frame_name)

        self.vf.loadPointCloudFromPoints(
            "pointcloud", self._point_cloud_res, points, colors=colors
        )

        self.vf.moveObstacle("pointcloud", frame_position)

    def restart(self):
        """This needs to be improved"""
        corba.restart()
        self.setup_problem()

    def _build_bin_picking(
        self,
        enable_collision_between_box_and_part: bool,
        only_free_node: bool,
        build_effector: bool,
        config_box_poses=None,
    ):
        self.binPicking = BinPicking(self.ps, self.use_spline_gradient_based_opt)
        self.binPicking.objects = [
            self.manip_object.name,
            self.obstacle_object.name,
            self.obstacle2_object.name,
        ]
        self.binPicking.robotGrippers = ["panda/panda_gripper"]
        if only_free_node:
            self.binPicking.goalGrippers = []
            self.binPicking.goalHandles = []
            self.binPicking.handles = []
        else:
            self.binPicking.goalGrippers = ["goal/gripper"]
            self.binPicking.goalHandles = self.goal_handles
            self.binPicking.handles = self.handles
        self.binPicking.graphConstraints = [
            "locked_finger_1",
            "locked_finger_2",
            "locked_source_box",
            "locked_dest_box",
        ]

        # TODO: restructure this
        def disable_collision():
            srdf_disable_collisions = """<robot>\n"""
            srdf_disable_collisions_fmt = (
                """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
            )
            srdf_disable_collisions += srdf_disable_collisions_fmt.format(
                "source_box/base_link", "part/base_link"
            )
            srdf_disable_collisions += "</robot>"
            self.robot.client.manipulation.robot.insertRobotSRDFModelFromString(
                "", srdf_disable_collisions
            )

        if not enable_collision_between_box_and_part:
            disable_collision()

        build_time_start = time.time()
        print("Building constraint graph")
        self.binPicking.buildGraph(placement_clearance=self._after_picking_clearance)
        build_time_stop = time.time()
        building_time = build_time_stop - build_time_start
        print("The graph took ", building_time, "s to build.")

        # Create effector
        if build_effector:
            print("Building effector.")  # this hardcodes sequence of steps
            self.binPicking.buildEffectors(
                [f"source_box/base_link_{i}" for i in range(5)], self.q_init
            )
            self.binPicking.buildEffectors(
                [f"dest_box/base_link_{i}" for i in range(5)], self.q_init
            )

            print("Generating goal configurations.")
            self.binPicking.generateGoalConfigs(config_box_poses)

    def plan_pick_and_place(
        self,
        q_init: list[float],
        enable_collision_between_box_and_part: bool = True,
        q_above_source_bin: T.Optional[list[float]] = None,
    ):
        assert self._goal_obj_pose is not None, (
            "Goal object pose should have been set before."
        )

        self.q_init = (
            q_init
            + self.start_obj_pose
            + self.default_obstacle_pose
            + self.default_obstacle2_pose
        )

        self._build_bin_picking(
            enable_collision_between_box_and_part,
            only_free_node=False,
            build_effector=True,
            config_box_poses=(
                q_init  # Not important
                + self.start_obj_pose  # Not important
                + self.default_obstacle_pose
                + self.default_obstacle2_pose
            ),
        )

        res, q_init, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_init
        )
        assert res, f"Robot q_init isn't a valid configuration {err}"
        r = self.robot.rankInConfiguration["part/root_joint"]
        print(q_init)
        print("\nPose of the object : \n", q_init[r : r + 7], "\n")

        found, msg = self.robot.isConfigValid(q_init)
        assert found, f"Init config not collision free: {msg}"

        if q_above_source_bin is None:
            q_start_solve = q_init
            path_to_start = None
        else:
            self.q_above_source_bin = (
                q_above_source_bin
                + self.start_obj_pose
                + self.default_obstacle_pose
                + self.default_obstacle2_pose
            )

            res, self.q_above_source_bin, err = (
                self.binPicking.graph.applyNodeConstraints(
                    "free", self.q_above_source_bin
                )
            )
            assert res
            found, msg = self.robot.isConfigValid(self.q_above_source_bin)
            assert found, msg

            q_start_solve = self.q_above_source_bin
            path_to_start = self.binPicking.move_in_free(q_init, q_start_solve)

        # Resolving the path to the object
        print("[INFO] Object found with no collision")
        print("Solving ...")
        res, paths = self.binPicking.solve(q_start_solve)
        if res:
            grasp_path, placing_path, freefly_path = split_path(
                paths, self.binPicking.c_robot()
            )
            if path_to_start is not None:
                grasp_path = concatenatePaths(
                    [path_to_start, grasp_path], self.binPicking.c_robot()
                )
            self.ps.client.basic.problem.addPath(grasp_path)
            self.ps.client.basic.problem.addPath(placing_path)
            self.ps.client.basic.problem.addPath(freefly_path)
            print("Path generated.")
            return grasp_path, placing_path, freefly_path
        else:
            # In this case, p is a string containing the error message
            print("No solution", paths)
            return None

    def plan_free_motion(
        self,
        q_init: list[float],
        q_goal: list[float],
    ):
        self.q_init = (
            q_init
            + self.start_obj_pose
            + self.default_obstacle_pose
            + self.default_obstacle2_pose
        )
        self.q_goal = (
            q_goal
            + self.start_obj_pose
            + self.default_obstacle_pose
            + self.default_obstacle2_pose
        )

        self._build_bin_picking(True, only_free_node=True, build_effector=False)

        res, self.q_init, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_init
        )
        assert res, f"Robot q_init isn't a valid configuration {err}"
        q_init_ok, msg_init = self.robot.isConfigValid(self.q_init)

        res, self.q_goal, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_goal
        )
        assert res, f"Robot q_goal isn't a valid configuration {err}"
        q_goal_ok, msg_goal = self.robot.isConfigValid(self.q_goal)

        if not q_init_ok:
            print("q_init is not collision free", msg_init)
        elif not q_goal_ok:
            print("q_goal is not collision free", msg_goal)
        else:
            print("Solving ...")
            p = self.binPicking.move_in_free(self.q_init, self.q_goal)
            print("Free path", p)
            return p
        return

    def plan_calib_motion(
        self, q_init: list[float], box_handles: list[str], n_samples: int = 0
    ) -> list[T.Optional[HPPPath]]:
        """Calculates a set of motion to calibrate the position of the bins.

        It returns a list of either a HPP path or None. None marks the time when the
        gripper should be closed then opened.
        """

        gripper = "panda/panda_gripper"
        self.ps.createLockedJoint(
            "locked_object", f"{self.manip_object.name}/root_joint", self.start_obj_pose
        )
        self.ps.setConstantRightHandSide("locked_object", True)

        bp = self.binPicking = BinPicking(self.ps, self.use_spline_gradient_based_opt)
        bp.objects = [
            self.manip_object.name,
            self.obstacle_object.name,
            self.obstacle2_object.name,
        ]
        bp.robotGrippers = [gripper]
        bp.handles = box_handles
        bp.graphConstraints = [
            "locked_finger_1",
            "locked_finger_2",
            "locked_object",
            "locked_source_box",
            "locked_dest_box",
        ]
        bp.buildGraph(placement_clearance=self._after_picking_clearance)

        self.q_init = (
            q_init
            + self.start_obj_pose
            + self.default_obstacle_pose
            + self.default_obstacle2_pose
        )

        res, self.q_init, err = bp.graph.applyNodeConstraints("free", self.q_init)
        assert res, f"Robot q_init isn't a valid configuration {err}"
        q_init_ok, msg_init = self.robot.isConfigValid(self.q_init)

        assert q_init_ok, f"q_init is not collision free: {msg_init}"

        edge_fmt = "{gripper} > {handle} | f_{num}"
        last_q = self.q_init
        paths = []
        for handle in box_handles:
            edges = [
                edge_fmt.format(gripper=gripper, handle=handle, num="01"),
                edge_fmt.format(gripper=gripper, handle=handle, num="12"),
            ]
            path, report = bp.generateConsecutivePaths(
                edges, last_q, Nsamples=n_samples
            )

            assert path is not None, f"Failed to generate path: {report}"
            bp.transitionPlanner.setEdge(bp.graph.edges[edges[0]])
            p_to_start, res, msg = bp.transitionPlanner.directPath(
                last_q, path.initial(), True
            )
            assert res, f"Failed to generate path: {msg}"
            p_to_start = bp.wd(p_to_start)
            p_to_start = bp.wd(
                bp.transitionPlanner.timeParameterization(p_to_start.asVector())
            )

            last_q = path.initial()
            paths.append(p_to_start)
            paths.append(path)
            paths.append(None)
            paths.append(path.reverse())

        bp.transitionPlanner.setEdge(bp.graph.edges["Loop | f"])
        back_to_init, res, msg = bp.transitionPlanner.directPath(
            last_q, self.q_init, True
        )
        paths.append(back_to_init)

        # For gepetto GUI
        global_path = concatenatePaths(
            [p for p in paths if p is not None], bp.c_robot()
        )
        self.ps.hppcorba.problem.addPath(global_path)
        print(f"Added to path vector at index {self.ps.numberPaths() - 1}")

        return paths


# ____________________________________________________________________________
def get_q_dq_ddq_arrays_from_path(hpp_path, robot_ndof=7, dt=0.01):
    total_time = hpp_path.length()
    print(total_time)
    T = int(total_time / dt)
    q_array = []
    dq_array = []
    ddq_array = []
    for iter in range(T):
        iter_time = total_time * iter / (T - 1)
        q_array.append(np.array(hpp_path.call(iter_time)[0][:robot_ndof]))
        dq_array.append(np.array(hpp_path.derivative(iter_time, 1)[:robot_ndof]))
        ddq_array.append(np.array(hpp_path.derivative(iter_time, 2)[:robot_ndof]))
    return q_array, dq_array, ddq_array
