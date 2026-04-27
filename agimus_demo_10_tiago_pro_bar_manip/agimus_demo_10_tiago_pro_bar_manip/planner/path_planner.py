import numpy as np

from pyhpp.core import (
    RandomShortcut,
    SplineGradientBased_bezier3,
    SimpleTimeParameterization,
    ProgressiveProjector,
)
from pyhpp.manipulation import TransitionPlanner
from pyhpp.core.path import Vector as PathVector


def concatenatePaths(paths, logger=None):
    if not paths:
        return None
    pv = PathVector.create(paths[0].outputSize(), paths[0].outputDerivativeSize())
    for p in paths:
        pv.appendPath(p)
    return pv


class PathPlanner:
    def __init__(self, problem, cg, robot):
        self.problem = problem
        self.graph = cg
        self.robot = robot
        # graph MUST be initialized before this constructor is called
        # (enforced by HPPPathGenerator which calls graph.initialize()
        #  inside _generate_constraint_graph() before creating PathPlanner).

    def _get_idx_q(self, joint_name):
        """Get idx_q for a joint via Pinocchio model."""
        model = self.robot.model()
        if not model.existJointName(joint_name):
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        jid = model.getJointId(joint_name)
        return model.joints[jid].idx_q

    def _set_transition_planner(self):
        """Initialize TransitionPlanner with appropriate settings for our problem."""
        self.transitionPlanner = TransitionPlanner(self.problem)

        inner_problem = self.transitionPlanner.innerProblem()
        projector = ProgressiveProjector(
            inner_problem.distance(),
            inner_problem.steeringMethod(),
            0.05,
        )
        self.transitionPlanner.pathProjector(projector)

        self.transitionPlanner.maxIterations(1000)

        self.transitionPlanner.clearPathOptimizers()
        self.transitionPlanner.addPathOptimizer(RandomShortcut(self.problem))
        stp = SimpleTimeParameterization(self.problem)
        stp.order = 2
        stp.maxAcceleration = 0.2
        self.transitionPlanner.addPathOptimizer(stp)

        self.transitionPlanner.setTransition(self.graph.getTransition("Loop | f"))

    def checkConfigurationValid(self, q):
        """Check if configuration q is valid (collision-free and satisfies constraints)."""
        res, _msg = self.problem.isConfigValid(q)
        return res

    def compute_base_pose_from_handle(
        self, handle_pose, current_base_pose, distance=0.75
    ):
        """Compute a base pose in front of the handle, facing it."""
        hx, hy = handle_pose[0], handle_pose[1]
        rx, ry = current_base_pose[0], current_base_pose[1]
        vx, vy = rx - hx, ry - hy
        norm = np.hypot(vx, vy)
        if norm < 1e-3:
            vx, vy, norm = -1.0, 0.0, 1.0
        vx /= norm
        vy /= norm
        x, y = hx + vx * distance, hy + vy * distance
        theta = np.arctan2(-vy, -vx)
        return x, y, np.cos(theta), np.sin(theta)

    def generateGraspingConfigurations(
        self,
        gripper,
        handle,
        q_init,
        testCollision=True,
        logger=None,
    ):
        """Generate grasping configurations (pregrasp, grasp, preplace) for a given gripper and handle.

        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            testCollision (bool): Whether to test for collision when generating configurations.
            logger: Optional logger for logging messages.
        Returns:
            tuple: A tuple containing the pregrasp configuration, grasp configuration, and preplace configuration. If no valid configurations are found, returns (None, None, None).
        """
        prefix = f"{gripper} > {handle} | f"

        handle_idx = self._get_idx_q("reinforcement_bar/root_joint")
        base_idx = self._get_idx_q("tiago_pro/root_joint")

        # _asq() is mandatory: pyhpp takes Eigen::Ref, not Python lists.
        handle_pose = q_init[handle_idx : handle_idx + 7]
        base_pose = q_init[base_idx : base_idx + 4]
        q_base = self.compute_base_pose_from_handle(handle_pose, base_pose)

        res = False
        qap = qpg = qg = qpp = None
        # == APPROACH CONFIGURATION ==
        # Generate a configuration that satisfies the "transit_free" constraints, with the base positioned in front of the handle.
        q = q_init.copy()
        q[base_idx : base_idx + 4] = q_base
        _, q, _ = self.graph.applyStateConstraints(
            self.graph.getState("free"), self._asq(q)
        )

        transition = self.graph.getTransition("transit_free")
        res, qap, _ = self.graph.generateTargetConfig(transition, q_init, q)
        if not res:
            return None, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, _ = pv.validateConfiguration(qap)
            if not res:
                self._log(
                    logger, "ERROR", "Collision detected for approach configuration"
                )
                return None, None, None, None
        # == PREGRASP CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_01")
        res, qpg, _ = self.graph.generateTargetConfig(transition, qap, qap)
        if not res:
            return qap, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, _ = pv.validateConfiguration(qpg)
            if not res:
                self._log(
                    logger, "ERROR", "Collision detected for pregrasp configuration"
                )
                return qap, None, None, None
        # == PICK CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_12")
        res, qg, _ = self.graph.generateTargetConfig(transition, qpg, qpg)
        if not res:
            return qap, qpg, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, _ = pv.validateConfiguration(qg)
            if not res:
                self._log(logger, "ERROR", "Collision detected for grasp configuration")
                return qap, qpg, None, None
        # == PREPLACE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_23")
        res, qpp, _ = self.graph.generateTargetConfig(transition, qg, qg)
        if not res:
            return qap, qpg, qg, None
        if testCollision:
            pv = transition.pathValidation()
            res, _ = pv.validateConfiguration(qpp)
            if not res:
                self._log(
                    logger, "ERROR", "Collision detected for preplace configuration"
                )
                return qap, qpg, qg, None

        return qap, qpg, qg, qpp

    def generatePlacementConfigurations(
        self,
        gripper,
        handle,
        q_init,
        target_bar_pose,
        testCollision=True,
        logger=None,
    ):
        """Generate placing configurations (approach, preplacement, release, post-realese) for a given gripper and handle.

        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            target_bar_pose (list): The desired final configuration of the bar
            testCollision (bool): Whether to test for collision when generating configurations.
            logger: Optional logger for logging messages.
        Returns:
            tuple: A tuple containing the pregrasp configuration, grasp configuration, and preplace configuration. If no valid configurations are found, returns (None, None, None).
        """
        prefix = f"{gripper} < {handle} | 0-0"

        handle_idx = self._get_idx_q("reinforcement_bar/root_joint")
        base_idx = self._get_idx_q("tiago_pro/root_joint")

        q_init = self._asq(q_init)
        q_goal = q_init.copy()
        q_goal[handle_idx : handle_idx + 7] = target_bar_pose

        base_pose = q_init[base_idx : base_idx + 4]
        q_base_place = self.compute_base_pose_from_handle(target_bar_pose, base_pose)

        qap, qpp, qp, qrel = None, None, None, None

        # == APPROACH CONFIGURATION ==
        # Generate a configuration that satisfies the "transit_grasp" constraints, with the base positioned in front of the target placement pose.
        q = q_init.copy()
        q[base_idx : base_idx + 4] = q_base_place
        _, q, _ = self.graph.applyStateConstraints(
            self.graph.getState(f"{gripper} grasps {handle}"), self._asq(q)
        )
        transition = self.graph.getTransition("transit_grasp")
        res, qap, err = self.graph.generateTargetConfig(transition, q_init, q)
        if not res:
            self._log(logger, "WARN", f"qap generation failed — {err}")
            return None, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qap)
            if not res:
                self._log(logger, "WARN", f"qap fail — {msg}")
                return None, None, None, None
        # == PREPLACE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_32")
        res, qpp, err = self.graph.generateTargetConfig(transition, qap, qap)
        if not res:
            self._log(logger, "WARN", f"qpp generation failed — {err}")
            return qap, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qpp)
            if not res:
                self._log(logger, "WARN", f"qpp fail — {msg}")
                return qap, None, None, None
        # == PLACE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_21")
        res, qp, err = self.graph.generateTargetConfig(transition, qpp, qpp)
        if not res:
            self._log(logger, "WARN", f"qp generation failed — {err}")
            return qap, qpp, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qp)
            if not res:
                self._log(logger, "WARN", f"qp fail — {msg}")
                return qap, qpp, None, None
        # == RELEASE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_10")
        res, qrel, err = self.graph.generateTargetConfig(transition, qp, qp)
        if not res:
            self._log(logger, "WARN", f"qrel generation failed — {err}")
            return qap, qpp, qp, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qrel)
            if not res:
                self._log(logger, "WARN", f"qrel fail — {msg}")
                return qap, qpp, qp, None
        return qap, qpp, qp, qrel

    def _log(self, logger, level, msg):
        """
        Log a message with the given level using the provided logger, or print to console if logger is None.
        """
        if logger is None:
            print(f"[PathPlanner][{level}] {msg}")
        elif level == "ERROR":
            logger.error(f"[PathPlanner] {msg}")
        elif level == "WARN":
            logger.warn(f"[PathPlanner] {msg}")
        else:
            logger.info(f"[PathPlanner] {msg}")

    def _asq(self, q) -> np.ndarray:
        """Convert a configuration to a numpy array of type float64."""
        return np.asarray(q, dtype=np.float64)

    def _goals_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert a single configuration q into a 2D numpy array with shape (1, config_size) in Fortran order."""
        q_goal = np.zeros((1, self.robot.configSize()), order="F")
        q_goal[0, :] = q
        return q_goal

    def optimizePath(self, path, logger=None):
        """Optimize a path using shortcutting and spline optimization, with logging."""
        shortcut = RandomShortcut(self.problem)
        spline_opt = SplineGradientBased_bezier3(self.problem)
        try:
            for i in range(3):
                p_new = shortcut.optimize(path)
                tr_before = path.timeRange()
                tr_after = p_new.timeRange()
                dt = (tr_before.second - tr_before.first) - (
                    tr_after.second - tr_after.first
                )
                path = p_new
                self._log(
                    logger,
                    "INFO",
                    f"  path shortcut pass {i + 1}/3: {tr_after.second - tr_after.first:.2f} s  (−{dt:.2f} s)",
                )
                if dt < 1e-3:
                    break
        except Exception:
            self._log(logger, "ERROR", "  path shortcut failed: {e}")
        try:
            path = spline_opt.optimize(path)
            tr = path.timeRange()
            self._log(logger, "INFO", f"  path spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            self._log(logger, "ERROR", f"  path spline optimisation failed: {e}")
        return path

    def planPathtoBarHandling(self, gripper, handle, q_init, logger):
        """
        Plan a path from the initial configuration to a grasping configuration for the specified gripper and handle.
        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            logger: Optional logger for logging messages.
        Returns:
            PathVector: A path vector representing the planned path, or None if planning failed.
        """
        self._set_transition_planner()

        res, q_init, _ = self.graph.applyStateConstraints(
            self.graph.getState("free"), q_init
        )
        if not res:
            self._log(
                logger,
                "WARN",
                "applyStateConstraints('free') failed — using raw q_init",
            )

        qap, qpg, qg, qpp = self.generateGraspingConfigurations(
            gripper,
            handle,
            self._asq(q_init),
            testCollision=True,
            attempts=100,
            logger=logger,
        )
        if qap is None or qpg is None or qg is None or qpp is None:
            self._log(logger, "ERROR", "Failed to generate grasping configurations")
            return None
        self._log(logger, "INFO", "Grasping configurations generated")

        prefix = f"{gripper} > {handle} | f"

        # ==================================================================
        # Segment 0: free → approach (direct, transit_free)
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition("transit_free"))
        res, p0, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 0 FAILED — {msg}")
            return None

        # ==================================================================
        # Segment 1: grasp approach -> preplacement (RRT, prefix + "_01")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_01"))
        p1 = self.transitionPlanner.planPath(qap, self._goals_matrix(qpg), True)
        if not p1:
            self._log(logger, "ERROR", "Segment 1 FAILED")
            return None
        # p1 = self.transitionPlanner.optimizePath(p1)
        # p1 = self.optimizePath(p1, logger=logger)

        # ==================================================================
        # Segment 2: pregrasp → grasp (constrained direct, prefix + "_12")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_12"))
        res, p2, msg = self.transitionPlanner.directPath(qpg, qg, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 2 FAILED — {msg}")
            return None
        p2_vector = PathVector.create(p2.outputSize(), p2.outputDerivativeSize())
        p2_vector.appendPath(p2)
        # p2 = self.transitionPlanner.timeParameterization(p2_vector)
        # p2 = self.optimizePath(p2_vector, logger=logger)

        # ==================================================================
        # Segment 3: grasp → preplace (constrained direct, prefix + "_23")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_23"))
        res, p3, msg = self.transitionPlanner.directPath(qg, qpp, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 3 FAILED — {msg}")
            return None
        p3_vector = PathVector.create(p3.outputSize(), p3.outputDerivativeSize())
        p3_vector.appendPath(p3)
        # p3 = self.transitionPlanner.timeParameterization(p3_vector)
        # p3 = self.optimizePath(p3_vector, logger=logger)

        self._log(logger, "INFO", "Grasping path planned successfully.")
        return concatenatePaths([p0, p1, p2, p3], logger=logger)

    def planPathtoBarPlacement(self, gripper, handle, q_init, target_bar_pose, logger):
        self._set_transition_planner()
        """
        Plan a path from the grasping configuration to the placement configuration for the specified gripper and handle.
        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot (after grasping).
            target_bar_pose (list): The desired final configuration of the bar.
            logger: Optional logger for logging messages.
        Returns:
            PathVector: A path vector representing the planned path, or None if planning failed.
        """

        res, q_init, _ = self.graph.applyStateConstraints(
            self.graph.getState(f"{gripper} grasps {handle}"), q_init
        )
        if not res:
            self._log(logger, "WARN", "applyStateConstraints for grasp state failed")

        qap, qpp, qp, qrel = self.generatePlacementConfigurations(
            gripper, handle, q_init, target_bar_pose, testCollision=True, logger=logger
        )
        if qap is None or qpp is None or qp is None or qrel is None:
            self._log(logger, "ERROR", "Failed to generate placement configurations")
            return None
        self._log(logger, "INFO", "Placement configurations generated")

        prefix = f"{gripper} < {handle} | 0-0"
        # ==================================================================
        # Segment 0: grasp approach (direct, transit_grasp)
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition("transit_grasp"))
        res, p0, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 0 FAILED — {msg}")
            return None

        # ==================================================================
        # Segment 1: approach → preplacement (RRT, prefix + "_32")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_32"))
        p1 = self.transitionPlanner.planPath(qap, self._goals_matrix(qpp), True)
        if not p1:
            self._log(logger, "ERROR", "Segment 1 FAILED")
            return None

        # p1 = self.transitionPlanner.optimizePath(p1)
        # p1 = self.optimizePath(p1, logger=logger)

        # ==================================================================
        # Segment 2: preplacement → placement (constrained direct, prefix + "_21")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_21"))
        res, p2, msg = self.transitionPlanner.directPath(qpp, qp, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 2 FAILED — {msg}")
            return None
        # p2 = self.transitionPlanner.timeParameterization(p2)
        # p2 = self.optimizePath(p2, logger=logger)

        # ==================================================================
        # Segment 3: placement → release (constrained direct, prefix + "_10")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_10"))
        res, p3, msg = self.transitionPlanner.directPath(qp, qrel, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 3 FAILED — {msg}")
            return None
        # p3 = self.transitionPlanner.timeParameterization(p3)
        # p3 = self.optimizePath(p3, logger=logger)

        self._log(logger, "INFO", "Placement path planned successfully.")
        return concatenatePaths([p0, p1, p2, p3])
