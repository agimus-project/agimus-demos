from math import fabs

import numpy as np
from hpp.corbaserver import wrap_delete as wd


def concatenatePaths(paths, logger=None):
    if len(paths) == 0:
        return None
    p = paths[0].asVector()
    for q in paths[1:]:
        dist = np.linalg.norm(np.array(p.end()) - np.array(q.initial()))
        if dist > 1e-3:
            if logger:
                logger.warn(
                    f"Concatenating two paths with a discontinuity of {dist:.4f} between {p.end()} and {q.initial()}."
                )
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
        self.problem = ps.hppcorba.problem
        self._set_transition_planner()

    def _set_transition_planner(self):
        self.croadmap = self.ps.client.manipulation.problem.createRoadmap(
            self.cdistance, self.crobot
        )
        self.croadmap.constraintGraph(self.cgraph)
        self.transitionPlanner = self.ps.client.basic.problem.createPathPlanner(
            "TransitionPlanner", self.cproblem, self.croadmap
        )
        self.transitionPlanner.setPathProjector("Progressive", 0.05)
        self.transitionPlanner.maxIterations(15000)
        self.transitionPlanner.addPathOptimizer("RandomShortcut")
        self.transitionPlanner.addPathOptimizer(
            "SimpleTimeParameterization"
        )  # SplineGradientBased_bezier3
        self.transitionPlanner.setEdge(self.graph.edges["Loop | f"])

    def checkConfigurationValid(self, q):
        res, msg = self.robot.isConfigValid(q)
        return res

    def compute_base_pose_from_handle(
        self, handle_pose, current_base_pose, distance=0.75
    ):
        """
        Compute a base pose close to the bar (at a certain 'distance'),
        while preserving the current axis between the robot and the bar to avoid
        teleporting it to the other side of the table or inside it.
        """
        hx, hy = handle_pose[0], handle_pose[1]
        rx, ry = current_base_pose[0], current_base_pose[1]

        vx = rx - hx
        vy = ry - hy
        norm = np.hypot(vx, vy)

        if (
            norm < 1e-3
        ):  # If the robot is too close from the bar, we cannot reliably keep the same axis, so we choose a default one (facing the table)
            vx, vy = -1.0, 0.0
            norm = 1.0

        # Normalize vector
        vx /= norm
        vy /= norm

        x = hx + vx * distance
        y = hy + vy * distance

        theta = np.arctan2(-vy, -vx)

        return x, y, np.cos(theta), np.sin(theta)

    def reset_problem_state(self):
        """Nettoie proprement le solveur pour une nouvelle planification."""
        self.ps.resetConstraints()
        self.ps.client.basic.problem.resetRoadmap()  # Efface les noeuds précédents
        # Recréer le planner pour s'assurer qu'il pointe sur une roadmap vierge
        self._set_transition_planner()

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
        self.ps.resetConstraints()
        self.ps.setInitialConfig(q_init)
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
        self.ps.setRightHandSideFromConfigByName(
            "place_reinforcement_bar/complement", q_init
        )
        self.ps.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
        handle_idx = self.robot.rankInConfiguration["reinforcement_bar/root_joint"]
        handle_pose = q_init[handle_idx : handle_idx + 7].copy()
        r = self.robot.rankInConfiguration["tiago_pro/root_joint"]
        current_base_pose = q_init[r : r + 4].copy()
        q_base = self.compute_base_pose_from_handle(
            handle_pose, current_base_pose, distance=0.75
        )
        v(q_init)
        res = False
        qap, qpg, qg, qpp = None, None, None, None
        for i in range(attempts):
            q = q_init[:]
            q[r : r + 4] = q_base
            _, q, _ = self.graph.applyNodeConstraints("free", q)
            edge = "transit_free"
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
        if res:
            return qap, qpg, qg, qpp
        return None, None, None, None

    def generatePlacementConfigurations(
        self,
        gripper,
        handle,
        q_init,
        target_bar_pose,
        testCollision=True,
        attempts=10,
        v=None,
        logger=None,
    ):
        """Generate placing configurations (approach, preplacement, release, post-realese) for a given gripper and handle.

        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            target_bar_pose (list): The desired final configuration of the bar
            testCollision (bool): Whether to test for collision when generating configurations.
            attempts (int): The number of attempts to generate valid configurations.
        Returns:
            tuple: A tuple containing the pregrasp configuration, grasp configuration, and preplace configuration. If no valid configurations are found, returns (None, None, None).
        """
        self.ps.resetConstraints()
        self.ps.setInitialConfig(q_init)
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
        prefix = f"{gripper} < {handle} | 0-0"
        # Set target bar pose in the constraints
        q_goal = q_init[:]
        handle_idx = self.robot.rankInConfiguration["reinforcement_bar/root_joint"]
        q_goal[handle_idx : handle_idx + 7] = target_bar_pose
        self.ps.setRightHandSideFromConfigByName(
            "place_reinforcement_bar/complement", q_goal
        )
        self.ps.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
        base_idx = self.robot.rankInConfiguration["tiago_pro/root_joint"]

        # handle q_target
        current_base_pose = q_init[base_idx : base_idx + 4].copy()

        # Compute a base pose for placement that is close to the bar
        q_base_place = self.compute_base_pose_from_handle(
            target_bar_pose, current_base_pose, distance=0.75
        )
        self._log(logger, "INFO", f"Target bar pose: {target_bar_pose}")
        self._log(logger, "INFO", f"Computed base pose for placement: {q_base_place}")

        qap, qpp, qp, qrel = None, None, None, None

        # Compute poses
        # APPROACH
        q = q_init[:]
        q[base_idx : base_idx + 4] = q_base_place
        node_grasps = f"{gripper} grasps {handle}"
        _, q, _ = self.graph.applyNodeConstraints(node_grasps, q)
        edge = "transit_grasp"
        res, qap, err = self.graph.generateTargetConfig(edge, q_init, q)
        if not res:
            self._log(logger, "WARN", f"qap generation failed — {err}")
            return None, None, None, None
        if testCollision:
            res, msg = self.transitionPlanner.validateConfiguration(
                qap, self.graph.edges[edge]
            )
            if not res:
                self._log(logger, "WARN", f"qap fail — {msg}")
                return None, None, None, None
        # PREPLACE
        edge = prefix + "_32"
        res, qpp, err = self.graph.generateTargetConfig(edge, qap, qap)
        if not res:
            self._log(logger, "WARN", f"qpp (Loop | 0-0) generation failed — {err}")
            return qap, None, None, None
        if testCollision:
            res, msg = self.transitionPlanner.validateConfiguration(
                qpp, self.graph.edges[edge]
            )
            if not res:
                self._log(logger, "WARN", f"qpp (Loop | 0-0) fail — {msg}")
                return qap, None, None, None
        # Placement
        edge = prefix + "_21"
        res, qp, err = self.graph.generateTargetConfig(edge, qpp, qpp)
        if not res:
            self._log(logger, "WARN", f"qp (Loop | 0-0) generation failed — {err}")
            return qap, qpp, None, None
        if testCollision:
            res, msg = self.transitionPlanner.validateConfiguration(
                qp, self.graph.edges[edge]
            )
            if not res:
                self._log(logger, "WARN", f"qp (Loop | 0-0) fail — {msg}")
                return qap, qpp, None, None
        # Release
        edge = prefix + "_10"
        res, qrel, err = self.graph.generateTargetConfig(edge, qp, qp)
        if not res:
            self._log(logger, "WARN", f"qpg generation failed — {err}")
            return qap, qpp, qp, None
        if testCollision:
            res, msg = self.transitionPlanner.validateConfiguration(
                qrel, self.graph.edges[edge]
            )
            if not res:
                self._log(logger, "WARN", f"qpg fail — {msg}")
                return qap, qpp, qp, None
        return qap, qpp, qp, qrel

    def _log(self, logger, level, msg):
        if logger is None:
            print(f"[PathPlanner][{level}] {msg}")
        elif level == "ERROR":
            logger.error(f"[PathPlanner] {msg}")
        elif level == "WARN":
            logger.warn(f"[PathPlanner] {msg}")
        else:
            logger.info(f"[PathPlanner] {msg}")

    def planPathtoBarHandling(self, gripper, handle, q_init, v, logger):
        self.reset_problem_state()
        res, q_init, msg = self.graph.applyNodeConstraints("free", q_init)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"applyNodeConstraints('free', q_init) failed — err={msg}. Using raw q_init.",
            )

        qap, qpg, qg, qpp = self.generateGraspingConfigurations(
            gripper,
            handle,
            q_init,
            testCollision=True,
            attempts=10,
            v=v,
            logger=logger,
        )
        self.ps.resetConstraints()
        if not qpg or not qg or not qpp:
            self._log(logger, "ERROR", "Fail to generate intermediate config")
            return None
        else:
            self._log(logger, "INFO", "Generated configurations for grasping")

        prefix = f"{gripper} > {handle} | f"
        v(q_init)
        #
        # Segment 0
        #
        edge = "transit_free"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p0, res, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not p0 or not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 0 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        # ------------------------------------------------------------------
        # Segment 1: free space -> pregrasp  (RRT-like, most under-optimal)
        # ------------------------------------------------------------------
        edge = prefix + "_01"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p1 = self.transitionPlanner.planPath(qap, [qpg], True)
        if not p1:
            self._log(
                logger, "ERROR", f"Segment 1 FAILED: planPath on '{edge}' returned None"
            )
            return None

        # Optimise p1 specifically: it is the longest segment and benefits
        # most from a shortcut pass before concatenation.
        p1_opt = self.transitionPlanner.optimizePath(p1)
        p1 = p1_opt

        # ------------------------------------------------------------------
        # Segment 2: pregrasp -> grasp  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_12"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p2, res, msg = self.transitionPlanner.directPath(qpg, qg, True)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 2 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        p21 = p2.asVector()
        p22 = self.transitionPlanner.timeParameterization(p21)
        p2 = p22

        # ------------------------------------------------------------------
        # Segment 3: grasp -> preplace  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_23"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p3, res, msg = self.transitionPlanner.directPath(qg, qpp, True)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 3 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        p31 = p3.asVector()
        p32 = self.transitionPlanner.timeParameterization(p31)
        p3 = p32

        # ------------------------------------------------------------------
        # Concatenate and run a final global optimisation pass
        # ------------------------------------------------------------------
        result = concatenatePaths([p0, p1, p2, p3], logger=logger)
        self.problem.addPath(result)
        self.problem.resetConstraints()
        self._log(logger, "INFO", "Grasping path planned successfully.")
        return wd(result)

    def planPathtoBarPlacement(
        self, gripper, handle, q_init, target_bar_pose, v, logger
    ):
        self.reset_problem_state()
        res, q_init, msg = self.graph.applyNodeConstraints(
            f"{gripper} grasps {handle}", q_init
        )

        if not res:
            self._log(
                logger,
                "ERROR",
                f"applyNodeConstraints('free', q_init) failed — err={msg}. Using raw q_init.",
            )

        qap, qpp, qp, qrel = self.generatePlacementConfigurations(
            gripper,
            handle,
            q_init,
            target_bar_pose,
            testCollision=True,
            v=v,
            logger=logger,
        )
        self.ps.resetConstraints()
        for q in [qap, qpp, qp, qrel]:
            if q is not None:
                # v(q)
                pass
            else:
                self._log(
                    logger, "WARN", f"One of the generated configurations is None ({q})"
                )
        if not qpp or not qp or not qrel:
            self._log(logger, "ERROR", "Fail to generate intermediate config")
            return None
        else:
            self._log(logger, "INFO", "Generated configurations for Placement")

        prefix = f"{gripper} < {handle} | 0-0"
        #
        # Segment 0
        #
        v(q_init)
        edge = "transit_grasp"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p0, res, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not p0 or not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 0 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        self._log(logger, "INFO", "TEST FLAG 1")
        # ------------------------------------------------------------------
        # Segment 1: grasp approach -> preplacement  (RRT-like, most under-optimal)
        # ------------------------------------------------------------------
        edge = prefix + "_32"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p1 = self.transitionPlanner.planPath(qap, [qpp], True)
        if not p1:
            self._log(
                logger, "ERROR", f"Segment 1 FAILED: planPath on '{edge}' returned None"
            )
            return None

        # Optimise p1 specifically: it is the longest segment and benefits
        # most from a shortcut pass before concatenation.
        p1_opt = self.transitionPlanner.optimizePath(p1)
        p1 = p1_opt
        self._log(logger, "INFO", "TEST FLAG 2")

        # ------------------------------------------------------------------
        # Segment 2: preplacement -> placement  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_21"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p2, res, msg = self.transitionPlanner.directPath(qpp, qp, True)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 2 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        p21 = p2.asVector()
        p22 = self.transitionPlanner.timeParameterization(p21)
        p2 = p22
        self._log(logger, "INFO", "TEST FLAG 3")
        # ------------------------------------------------------------------
        # Segment 3: placement -> release  (constrained direct path)
        # ------------------------------------------------------------------
        edge = prefix + "_10"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p3, res, msg = self.transitionPlanner.directPath(qp, qrel, True)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 2 FAILED: directPath on '{edge}' failed — {msg}",
            )
            return None
        p31 = p3.asVector()
        p32 = self.transitionPlanner.timeParameterization(p31)
        p3 = p32
        self._log(logger, "INFO", "TEST FLAG 4")

        # ------------------------------------------------------------------
        # Concatenate and run a final global optimisation pass
        # ------------------------------------------------------------------
        result = concatenatePaths([p0, p1, p2, p3])
        self._log(logger, "INFO", "TEST FLAG 5")
        self.problem.addPath(result)
        self._log(logger, "INFO", "TEST FLAG 6")
        self.problem.resetConstraints()
        self._log(logger, "INFO", "TEST FLAG 7")
        return wd(result)
