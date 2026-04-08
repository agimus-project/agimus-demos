from math import fabs

import numpy as np
from hpp.corbaserver import wrap_delete as wd


def concatenatePaths(paths):
    if len(paths) == 0:
        return None
    p = paths[0].asVector()
    for q in paths[1:]:
        assert np.linalg.norm(np.array(p.end()) - np.array(q.initial())) < 1e-8
        p.appendPath(q)
    return p


def getMaxVelocity(path):
    res = np.zeros(7)
    t = 0
    while t < path.length():
        v = path.derivative(t, 1)
        for i in range(7):
            res[i] = max(fabs(v[i]), res[i])
        t += 0.01
    return res


class PathPlanner(object):
    def __init__(self, ps, cg):
        self.ps = ps
        self.graph = cg
        self.robot = ps.robot
        self.cproblem = wd(ps.hppcorba.problem.getProblem())
        self.cgraph = wd(self.cproblem.getConstraintGraph())
        self.crobot = wd(self.cproblem.robot())
        self.cdistance = wd(
            ps.hppcorba.problem.createDistance("Weighed", self.cproblem)
        )
        self.croadmap = wd(
            ps.client.manipulation.problem.createRoadmap(self.cdistance, self.crobot)
        )
        self.croadmap.constraintGraph(self.cgraph)
        self.transitionPlanner = ps.client.basic.problem.createPathPlanner(
            "TransitionPlanner", self.cproblem, self.croadmap
        )

        self.transitionPlanner.setPathProjector("Progressive", 0.05)
        self.transitionPlanner.maxIterations(15000)
        self.transitionPlanner.addPathOptimizer("RandomShortcut")
        self.transitionPlanner.addPathOptimizer(
            "SimpleTimeParameterization"
        )  # SplineGradientBased_bezier3
        self.transitionPlanner.setEdge(cg.edges["Loop | f"])
        self.problem = ps.hppcorba.problem

    def checkConfigurationValid(self, q):
        res, msg = self.robot.isConfigValid(q)
        return res

    def compute_base_pose_from_handle(
        self, handle_pose, distance=0.6, angle_offset=0.0
    ):
        """
        Give a pose for so2 object close from the handle pose with the same orientation
        """

        hx, hy, hz = handle_pose[:3]
        qx, qy, qz, qw = handle_pose[3:]

        # PLacing in direction of the object
        theta = np.arctan2(hy, hx) + angle_offset

        # Close to the object
        x = hx + distance
        y = hy

        return x, y, -np.cos(theta), np.sin(theta)

    def generateGraspingConfigurations(
        self,
        gripper,
        handle,
        q_init,
        testCollision=True,
        attempts=10000,
        v=None,
        logger=None,
    ):
        """Generate grasping configurations (pregrasp, grasp, preplace) for a given gripper and handle.

        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            testCollision (bool): Whether to test for collision when generating configurations.
            attempts (int): The number of attempts to generate valid configurations.
        Returns:
            tuple: A tuple containing the pregrasp configuration, grasp configuration, and preplace configuration. If no valid configurations are found, returns (None, None, None).
        """
        self.problem.addNumericalConstraints(
            "cp",
            [
                "tiago_pro/left grasps reinforcement_bar/left",
                "tiago_pro/right grasps reinforcement_bar/right",
                "preplace_reinforcement_bar",
                "locked_plate/root_joint",
                "place_reinforcement_bar/complement",
            ],
            [0, 0, 0, 0, 0],
        )
        prefix = f"{gripper} > {handle} | f"
        self.problem.setRightHandSideFromConfigByName(
            "place_reinforcement_bar/complement", q_init
        )
        self.problem.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
        handle_idx = self.robot.rankInConfiguration["reinforcement_bar/root_joint"]
        handle_pose = q_init[handle_idx : handle_idx + 7].copy()
        r = self.robot.rankInConfiguration["tiago_pro/root_joint"]
        q_base = self.compute_base_pose_from_handle(
            handle_pose, distance=0.8, angle_offset=0.0
        )
        print(f"Handle pose: {handle_pose}")
        print(f"Computed base pose from handle: {q_base}")
        res = False
        for i in range(attempts):
            q = q_init[:]
            q[r : r + 4] = q_base
            _, q, _ = self.graph.applyNodeConstraints("free", q)
            edge = "transit_free"
            v(q)
            res, qap, err = self.graph.generateTargetConfig(edge, q_init, q)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qap, self.graph.edges[edge]
                )
                if not res:
                    continue

            # Project random configuration in pregrasp reachable from q_init
            edge = prefix + "_01"
            res, qpg, err = self.graph.generateTargetConfig(edge, qap, qap)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qpg, self.graph.edges[edge]
                )
                if not res:
                    continue
            # Generate corresponding grasp configuration
            edge = prefix + "_12"
            res, qg, err = self.graph.generateTargetConfig(edge, qpg, qpg)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qg, self.graph.edges[edge]
                )
                if not res:
                    continue
            # Generate corresponding preplace configuration
            edge = prefix + "_23"
            res, qpp, err = self.graph.generateTargetConfig(edge, qg, qg)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qpp, self.graph.edges[edge]
                )
                if not res:
                    continue
            if res:
                break
        self.problem.resetConstraints()
        if res:
            return qap, qpg, qg, qpp
        return None, None, None, None

    def planPathtoBarHandling(self, gripper, handle, q_init, v, logger):
        qap, qpg, qg, qpp = self.generateGraspingConfigurations(
            gripper,
            handle,
            q_init,
            testCollision=True,
            attempts=10000,
            v=v,
            logger=logger,
        )
        print("Generated configurations for grasping")
        if not qpg or not qg or not qpp:
            print("Fail to generate intermediate config")
            return None

        prefix = f"{gripper} > {handle} | f"
        v(qap)
        #
        # Segment 0
        #
        edge = "transit_free"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p0, res, msg = self.transitionPlanner.directPath(q_init, qap, True)

        # ------------------------------------------------------------------
        # Segment 1: free space -> pregrasp  (RRT-like, most under-optimal)
        # ------------------------------------------------------------------
        edge = prefix + "_01"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p1 = self.transitionPlanner.planPath(qap, [qpg], True)
        if not p1:
            return None

        # Optimise p1 specifically: it is the longest segment and benefits
        # most from a shortcut pass before concatenation.
        p1_opt = self.transitionPlanner.optimizePath(p1)
        p1.deleteThis()
        p1 = p1_opt

        # ------------------------------------------------------------------
        # Segment 2: pregrasp -> grasp  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_12"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p2, res, msg = self.transitionPlanner.directPath(qpg, qg, True)
        p21 = p2.asVector()
        p22 = self.transitionPlanner.timeParameterization(p21)
        p2.deleteThis()
        p21.deleteThis()
        p2 = p22
        if not res:
            if p2:
                p2.deleteThis()
            p1.deleteThis()
            return None

        # ------------------------------------------------------------------
        # Segment 3: grasp -> preplace  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_23"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p3, res, msg = self.transitionPlanner.directPath(qg, qpp, True)
        p31 = p3.asVector()
        p32 = self.transitionPlanner.timeParameterization(p31)
        p3.deleteThis()
        p31.deleteThis()
        p3 = p32
        if not res:
            if p3:
                p3.deleteThis()
            if p2:
                p2.deleteThis()
            p1.deleteThis()
            return None

        # ------------------------------------------------------------------
        # Concatenate and run a final global optimisation pass
        # ------------------------------------------------------------------
        result = concatenatePaths([p0, p1, p2, p3])
        self.problem.addPath(result)
        p0.deleteThis()
        p1.deleteThis()
        p2.deleteThis()
        p3.deleteThis()
        return wd(result)

    # def planPathToBarPlacement(self, gripper, handle, q_init, q_goal):
    #     qpg, qp, qpp = self.generatePlacementConfigurations(
    #         gripper, handle, q_init, q_goal, testCollision=True, attempts=10000
    #     )
    #     if not qpg or not qp or not qpp:
    #         return None

    #     prefix = f"{gripper} > {handle} | f"

    #     # Plan between q_init and q_preplace
    #     edge = prefix + "_34"
    #     self.transitionPlanner.setEdge(self.graph.edges[edge])
    #     p1 = self.transitionPlanner.planPath(q_init, [qpp], True)
    #     if not p1:
    #         return None

    #     # Plan between q_preplace and q_place
    #     edge = prefix + "_23"
    #     self.transitionPlanner.setEdge(self.graph.edges[edge])
    #     p2, res, msg = self.transitionPlanner.directPath(qpp, qp, True)
    #     p21 = p2.asVector()
    #     p22 = self.transitionPlanner.timeParameterization(p21)
    #     p2.deleteThis()
    #     p21.deleteThis()
    #     p2 = p22
    #     if not res:
    #         if p2:
    #             p2.deleteThis()
    #         p1.deleteThis()
    #         return None

    #     # Plan between q_place and q_pregrasp for release
    #     edge = prefix + "_12"
    #     self.transitionPlanner.setEdge(self.graph.edges[edge])
    #     p3, res, msg = self.transitionPlanner.directPath(qp, qpg, True)
    #     p31 = p3.asVector()
    #     p32 = self.transitionPlanner.timeParameterization(p31)
    #     p3.deleteThis()
    #     p31.deleteThis()
    #     p3 = p32
    #     if not res:
    #         if p3:
    #             p3.deleteThis()
    #         if p2:
    #             p2.deleteThis()
    #         p1.deleteThis()
    #         return None

    #     # Plan between q_pregrasp and q_goal
    #     # edge = prefix + "_01"
    #     # self.transitionPlanner.setEdge(self.graph.edges[edge])
    #     # p4, res, msg = self.transitionPlanner.directPath(qpg, q_goal, True)
    #     # p41 = p4.asVector()
    #     # p42 = self.transitionPlanner.timeParameterization(p41)
    #     # p4.deleteThis()
    #     # p41.deleteThis()
    #     # p4 = p42
    #     # if not res:
    #     #     if p4:
    #     #         p4.deleteThis()
    #     #     if p3:
    #     #         p3.deleteThis()
    #     #     if p2:
    #     #         p2.deleteThis()
    #     #     p1.deleteThis()
    #     #     return None

    #     result = concatenatePaths([p1, p2, p3])
    #     self.problem.addPath(result)
    #     p1.deleteThis()
    #     p2.deleteThis()
    #     p3.deleteThis()
    #     return wd(result)
