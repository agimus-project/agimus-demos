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
        res, _ = self.robot.isConfigValid(q)
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

    def generateGraspingConfigurations(
        self,
        gripper,
        handle,
        q_init,
        testCollision=True,
        attempts=10,
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
        self.ps.setInitialConfig(q_init)
        self.problem.addNumericalConstraints(
            "cp",
            [
                "tiago_pro/left grasps reinforcement_bar/left",
                "tiago_pro/right grasps reinforcement_bar/right",
                "locked_plate/root_joint",
                "locked_reinforcement_bar/root_joint",
                "locked_table/root_joint",
            ],
            [0, 0, 0, 0, 0],
        )
        prefix = f"{gripper} > {handle} | f"
        self.ps.setRightHandSideFromConfigByName(
            "locked_reinforcement_bar/root_joint", q_init
        )
        self.ps.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
        self.ps.setRightHandSideFromConfigByName("locked_table/root_joint", q_init)
        handle_idx = self.robot.rankInConfiguration["reinforcement_bar/root_joint"]
        handle_pose = q_init[handle_idx : handle_idx + 7].copy()
        r = self.robot.rankInConfiguration["tiago_pro/root_joint"]
        current_base_pose = q_init[r : r + 4].copy()
        q_base = self.compute_base_pose_from_handle(
            handle_pose, current_base_pose, distance=0.75
        )
        print(f"Handle pose: {handle_pose}")
        print(f"Computed base pose from handle: {q_base}")
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
                self._log(
                    logger,
                    "WARN",
                    f"Attempt {i + 1}/{attempts}: qap generation failed — {err}",
                )
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qap, self.graph.edges[edge]
                )
                if not res:
                    self._log(
                        logger, "WARN", f"Attempt {i + 1}/{attempts}: qap fail — {msg}"
                    )
                    continue

            # Project random configuration in pregrasp reachable from q_init
            edge = prefix + "_01"
            res, qpg, err = self.graph.generateTargetConfig(edge, qap, qap)
            if not res:
                self._log(
                    logger, "WARN", f"Attempt {i + 1}/{attempts}: qpg fail — {msg}"
                )
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
                self._log(
                    logger, "WARN", f"Attempt {i + 1}/{attempts}: qg fail — {msg}"
                )
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qg, self.graph.edges[edge]
                )
                if not res:
                    self._log(
                        logger, "WARN", f"Attempt {i + 1}/{attempts}: qg fail — {msg}"
                    )
                    continue
            # Generate a pose to elevate the bar above its initial position, to avoid collision with the table during manipulation
            edge = "Loop | 0-0"  # the robot cannot move its base in this edge, so the generated configuration will be similar to qg but with the bar lifted
            qpp_guess = qg[:]  # On part de la position saisie
            # On monte la barre de 15 cm sur l'axe Z du monde
            qpp_guess[handle_idx + 2] += 0.15
            torso_idx = self.robot.rankInConfiguration["tiago_pro/torso_lift_joint"]
            qpp_guess[torso_idx] += 0.15
            res, qpp, err = self.graph.generateTargetConfig(edge, qg, qpp_guess)
            if not res:
                self._log(
                    logger,
                    "WARN",
                    f"Attempt {i + 1}/{attempts}: qpp (Loop | 0-0) fail — {err}",
                )
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qpp, self.graph.edges["Loop | 0-0"]
                )
                if not res:
                    continue
            if res:
                break
        self._log(logger, "WARN", f"Attempt q_grasp : {qg}\nq_preplace : {qpp}")
        return qap, qpg, qg, qpp

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
            v=v,
            logger=logger,
        )
        for q in [qap, qpg, qg, qpp]:
            if q is not None:
                # v(q)
                pass
            else:
                self._log(
                    logger, "WARN", f"One of the generated configurations is None ({q})"
                )
        if not qpg or not qg:
            self._log(logger, "ERROR", "Fail to generate intermediate config")
            return None
        else:
            self._log(logger, "INFO", "Generated configurations for grasping")

        prefix = f"{gripper} > {handle} | f"
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
            p0.deleteThis()
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
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 2 FAILED: directPath on '{edge}' failed — {msg}",
            )
            if p2:
                p2.deleteThis()
            p1.deleteThis()
            p0.deleteThis()
            return None
        p21 = p2.asVector()
        p22 = self.transitionPlanner.timeParameterization(p21)
        p2.deleteThis()
        p21.deleteThis()
        p2 = p22

        # ------------------------------------------------------------------
        # Segment 2: pregrasp -> grasp  (constrained direct path)
        # ------------------------------------------------------------------
        edge = "Loop | 0-0"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p3, res, msg = self.transitionPlanner.directPath(qg, qpp, True)
        if not res:
            self._log(
                logger,
                "ERROR",
                f"Segment 2 FAILED: directPath on '{edge}' failed — {msg}",
            )
            if p3:
                p3.deleteThis()
            p2.deleteThis()
            p1.deleteThis()
            p0.deleteThis()
            return None
        p31 = p3.asVector()
        p32 = self.transitionPlanner.timeParameterization(p31)
        p3.deleteThis()
        p31.deleteThis()
        p3 = p32

        # ------------------------------------------------------------------
        # Concatenate and run a final global optimisation pass
        # ------------------------------------------------------------------
        result = concatenatePaths([p0, p1, p2, p3])
        self.problem.addPath(result)
        p0.deleteThis()
        p1.deleteThis()
        p2.deleteThis()
        p3.deleteThis()
        self.problem.resetConstraints()
        return wd(result)
