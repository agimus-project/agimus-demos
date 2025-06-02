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

from math import sqrt

# Used if hppcorbaserver is not running in separate script
# from agimus_demo_06_regrasp.corba import CorbaServer
from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import Robot, newProblem, ProblemSolver, Client
from hpp.gepetto.manipulation import ViewerFactory
from agimus_demo_06_regrasp.bin_picking import BinPicking
import numpy as np

import time

from agimus_demo_06_regrasp.utils import (
    split_path,
    BaseObject,
    get_obj_goal_handles,
    XYZQuatType,
    multiply_poses,
)
from hpp.rostools import process_xacro, retrieve_resource
from agimus_controller.trajectory import TrajectoryPoint


def hack_for_ros2_support_in_hpp():
    import os

    if "ROS_PACKAGE_PATH" not in os.environ and "AMENT_PREFIX_PATH" in os.environ:
        os.environ["ROS_PACKAGE_PATH"] = ":".join(
            v + "/share" for v in os.environ["AMENT_PREFIX_PATH"].split(":")
        )


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
    ):
        hack_for_ros2_support_in_hpp()

        self.use_spline_gradient_based_opt = use_spline_gradient_based_opt
        self.start_obj_pose = start_obj_pose
        self._goal_obj_pose = None
        self.gripper_open_value = gripper_open_value

        self._goal_gripper_clearance = 0.05
        self._after_picking_clearance = 1.05
        self._point_cloud_res = 0.001

        self.default_object_bounds = [-1.0, 1.5, -1.0, 1.0, 0.0, 2.2]
        package_location = "package://agimus_demo_06_regrasp"
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

        # Init corbaserver programmatically
        # self.corba = CorbaServer()
        # if hppcorbaserver runs already
        Client().problem.resetProblem()
        self.setup_problem()

    @property
    def goal_obj_pose(self) -> XYZQuatType:
        """Returns a list of size 7 contains the pose of the goal,
        defined in the panda support link frame.

        The pose is expressed as [tx, ty, tz, qx, qy, qz, qw]
        """
        return self._goal_obj_pose

    @goal_obj_pose.setter
    def goal_obj_pose(self, pose: XYZQuatType):
        """Sets the position of the goal wrt panda support link.

        Only the translation can be changed so only the 3 first element of the input
        are used.
        The x-axis is facing up since it is the direction of pregrasp motion
        """
        if self._goal_obj_pose is not None:
            raise RuntimeError("Cannot reset the goal object pose")
        self._goal_obj_pose = pose[:3] + [0, sqrt(2) / 2, 0, -sqrt(2) / 2]
        self.ps.client.manipulation.robot.addGripper(
            "panda/support_link",
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
        self.ps.setParameter("SimpleTimeParameterization/maxAcceleration", 0.2)
        self.ps.setParameter("SimpleTimeParameterization/safety", 0.95)

        # Add path projector to avoid discontinuities
        self.ps.selectPathProjector("Progressive", 0.05)
        self.ps.selectPathValidation("Graph-Progressive", 0.01)

        self.vf = ViewerFactory(self.ps)

        # load the object
        self.vf.loadObjectModel(self.manip_object, self.manip_object.name)
        self.robot.setJointBounds(
            f"{self.manip_object.name}/root_joint", self.default_object_bounds
        )
        print("Part loaded")
        self.robot.client.manipulation.robot.insertRobotSRDFModel(
            "panda",
            retrieve_resource("package://agimus_demo_06_regrasp/srdf/demo.srdf"),
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

        # Add an empty point cloud
        self.ps.client.basic.obstacle.loadPointCloudFromPoints(
            "pointcloud", self._point_cloud_res, []
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

        # Add handle of the objects
        self.handles, self.goal_handles = get_obj_goal_handles(
            prefix=self.manip_object.name + "/",
            srdf_path=self.manip_object.srdfFilename,
        )

    def setup_problem(self):
        newProblem()
        self.set_robot()
        self.set_problem()

    def add_handle(self, transform: XYZQuatType, handle_id: str):
        # Create handle from Grasp generator
        self.ps.client.manipulation.robot.addHandle(
            f"{self.manip_object.name}/base_link",
            f"{self.manip_object.name}/handle{handle_id}",
            transform,  # xyz_quatxyzw
            0.03,  # default clearance for handle
            [1, 1, 1, 1, 1, 1],
        )
        # rotate around x np.pi to account for both possible orientations of gripper
        self.ps.client.manipulation.robot.addHandle(
            f"{self.manip_object.name}/base_link",
            f"{self.manip_object.name}/handle{handle_id}_rotated",
            multiply_poses(transform, [0, 0, 0, 1, 0, 0, 0]),  # xyz_quatxyzw
            0.03,  # default clearance for handle
            [1, 1, 1, 1, 1, 1],
        )

        # Create a goal handle that is opposite to generated grasp
        # (rotate 180 degrees around y-axis)
        self.ps.client.manipulation.robot.addHandle(
            f"{self.manip_object.name}/base_link",
            f"{self.manip_object.name}/goal_handle{handle_id}",
            multiply_poses(
                transform, [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            ),  # xyz_quatxyzw
            0.01,  # default clearance for goal handle
            # [1, 1, 1, 0, 1, 1]  # goal mask allows rotation around x-axis
            [1, 1, 1, 0, 0, 0],  # goal mask allows any rotation
        )
        # Add handles to respective lists
        self.handles.append(f"{self.manip_object.name}/handle{handle_id}")
        self.handles.append(f"{self.manip_object.name}/handle{handle_id}_rotated")
        self.goal_handles.append(f"{self.manip_object.name}/goal_handle{handle_id}")

    def get_robot_link_position(
        self,
        q_robot: list[float],
        frame_name: str,
    ) -> list[float]:
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
        q_robot: list[float],
        camera_frame_name: str,
        points: list[tuple[float, float, float]],
        colors: list[tuple[float, float, float, float]] | None = None,
    ):
        frame_position = self.get_robot_link_position(q_robot, camera_frame_name)

        self.vf.loadPointCloudFromPoints(
            "pointcloud", self._point_cloud_res, points, colors=colors
        )

        self.vf.moveObstacle("pointcloud", frame_position)

    def restart(self):
        """This needs to be improved"""
        self.corba.restart()
        self.setup_problem()

    def _build_bin_picking(
        self,
        build_effector: bool,
        q_with_init_obj_poses: XYZQuatType,
        goal_handles: list[str],
    ):
        for handles in goal_handles:
            assert handles in self.goal_handles, f"{handles} not in {self.goal_handles}"
        self.binPicking = BinPicking(self.ps, self.use_spline_gradient_based_opt)
        self.binPicking.objects = [self.manip_object.name]
        self.binPicking.robotGrippers = ["panda/panda_gripper"]
        self.binPicking.goalGrippers = ["goal/gripper"]
        self.binPicking.goalHandles = goal_handles
        self.binPicking.handles = self.handles
        self.binPicking.graphConstraints = ["locked_finger_1", "locked_finger_2"]

        build_time_start = time.time()
        print("Building constraint graph")
        self.binPicking.buildGraph(placement_clearance=self._after_picking_clearance)
        build_time_stop = time.time()
        building_time = build_time_stop - build_time_start
        print("The graph took ", building_time, "s to build.")

        # Create effector
        if build_effector:
            print("Building effector.")  # this hardcodes sequence of steps
            self.binPicking.buildEffectors([], self.q_init)

            print("Generating goal configurations.")
            self.binPicking.generateGoalConfigs(q_with_init_obj_poses)
        # print(self.binPicking.placePaths.keys())
        # breakpoint()
        # print(self.binPicking.placePaths)

    def plan_pick_and_place(self, q_init: list[float], goal_handles: list[str]):
        assert self._goal_obj_pose is not None, (
            "Goal object pose should have been set before."
        )

        self.q_init = q_init + self.start_obj_pose

        self._build_bin_picking(
            build_effector=True,
            q_with_init_obj_poses=(self.q_init.copy()),
            goal_handles=goal_handles,
        )

        # look on goal placements found by bin_picking
        for gripper in self.binPicking.placePaths.keys():
            for part_handle in self.binPicking.placePaths[gripper].keys():
                q_start = self.binPicking.placePaths[gripper][part_handle].initial()
                self.robot.setCurrentConfig(q_start)
                input(f"Look on place path q0 for handle {part_handle}")

        # self.robot.setCurrentConfig(q_init)
        # print(self.binPicking.graph.getNode(q_init))
        # input("Look on the robot in gepetto-viewer (after projecting to free)")

        res, q_init, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_init
        )
        assert res, f"Robot q_init isn't a valid configuration {err}"
        r = self.robot.rankInConfiguration["part/root_joint"]
        print(q_init)
        print("\nPose of the object : \n", q_init[r : r + 7], "\n")

        found, msg = self.robot.isConfigValid(q_init)
        self.robot.setCurrentConfig(q_init)
        print(self.binPicking.graph.getNode(q_init))
        input("Look on the robot in gepetto-viewer (after projecting to free)")

        # Resolving the path to the object
        if found:
            print("[INFO] Object found with no collision")
            print("Solving ...")
            res, p, interesting_q = self.binPicking.solve(q_init)
            for q in interesting_q:
                self.robot.setCurrentConfig(q)
                input(f"Look on interesting q {q}")

            if res:
                print("Pick and place path", p)
                print(p.length())
                grasp_path, placing_path, freefly_path = split_path(
                    p, self.binPicking.c_robot()
                )
                self.ps.client.basic.problem.addPath(grasp_path)
                self.ps.client.basic.problem.addPath(placing_path)
                self.ps.client.basic.problem.addPath(freefly_path)
                print("Path generated.")
                return grasp_path, placing_path, freefly_path
            else:
                # In this case, p is a string containing the error message
                print("No solution", p)
                return None

        else:
            print("[INFO] Initial configuration is invalid:")
            print(msg)
            return None

    def plan_free_motion(
        self,
        q_init: list[float],
        q_goal: list[float],
    ):
        assert self._goal_obj_pose is not None, (
            "Goal object pose should have been set before."
        )

        # object_static = np.isclose(self.start_obj_pose, self.goal_obj_pose).all()
        self.q_init = q_init + self.start_obj_pose
        if q_goal is None:
            q_goal = q_init.copy()
        self.q_goal = q_goal + self.goal_obj_pose

        self._build_bin_picking(True, build_effector=False)

        res, q_init, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_init
        )
        assert res, f"Robot q_init isn't a valid configuration {err}"
        q_init_ok, msg_init = self.robot.isConfigValid(q_init)

        res, q_goal, err = self.binPicking.graph.applyNodeConstraints(
            "free", self.q_goal
        )
        assert res, f"Robot q_goal isn't a valid configuration {err}"
        q_goal_ok, msg_goal = self.robot.isConfigValid(q_goal)

        if not q_init_ok:
            print("q_init is not collision free", msg_init)
        elif not q_goal_ok:
            print("q_goal is not collision free", msg_goal)
        else:
            print("Solving ...")
            p = self.binPicking.move_in_free(q_init, self.q_goal)
            print("Free path", p)
            return p
        return


# ____________________________________________________________________________
def get_traj_points_from_path(hpp_path, robot_ndof=7, dt=0.01):
    total_time = hpp_path.length()
    print(total_time)
    T = int(total_time / dt)
    traj_point_list = []
    for iter in range(T):
        iter_time = total_time * iter / (T - 1)  # iter * dt

        traj_point_list.append(
            TrajectoryPoint(
                robot_configuration=np.array(hpp_path.call(iter_time)[0][:robot_ndof]),
                robot_velocity=np.array(hpp_path.derivative(iter_time, 1)[:robot_ndof]),
                robot_acceleration=np.array(
                    hpp_path.derivative(iter_time, 2)[:robot_ndof]
                ),
                end_effector_poses={"link0": np.eye(4)},
            )
        )
    return traj_point_list
